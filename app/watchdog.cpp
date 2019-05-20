#include "watchdog.hpp"

#include "watchdog_service.hpp"

#include <endian.h>

#include <cstdint>
#include <ipmid/api.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/elog.hpp>
#include <phosphor-logging/log.hpp>
#include <string>
#include <xyz/openbmc_project/Common/error.hpp>

using phosphor::logging::commit;
using phosphor::logging::level;
using phosphor::logging::log;
using sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

static bool lastCallSuccessful = false;

void reportError()
{
    // We don't want to fill the SEL with errors if the daemon dies and doesn't
    // come back but the watchdog keeps on ticking. Instead, we only report the
    // error if we haven't reported one since the last successful call
    if (!lastCallSuccessful)
    {
        return;
    }
    lastCallSuccessful = false;

    // TODO: This slow down the end of the IPMI transaction waiting
    // for the commit to finish. commit<>() can take at least 5 seconds
    // to complete. 5s is very slow for an IPMI command and ends up
    // congesting the IPMI channel needlessly, especially if the watchdog
    // is ticking fairly quickly and we have some transient issues.
    commit<InternalFailure>();
}

ipmi::RspType<> ipmiAppResetWatchdogTimer()
{
    try
    {
        WatchdogService wd_service;

        // Notify the caller if we haven't initialized our timer yet
        // so it can configure actions and timeouts
        if (!wd_service.getInitialized())
        {
            lastCallSuccessful = true;

            constexpr uint8_t ccWatchdogNotInit = 0x80;
            return ipmi::response(ccWatchdogNotInit);
        }

        // The ipmi standard dictates we enable the watchdog during reset
        wd_service.resetTimeRemaining(true);
        lastCallSuccessful = true;
        return ipmi::responseSuccess();
    }
    catch (const InternalFailure& e)
    {
        reportError();
        return ipmi::responseUnspecifiedError();
    }
    catch (const std::exception& e)
    {
        const std::string e_str = std::string("wd_reset: ") + e.what();
        log<level::ERR>(e_str.c_str());
        reportError();
        return ipmi::responseUnspecifiedError();
    }
    catch (...)
    {
        log<level::ERR>("wd_reset: Unknown Error");
        reportError();
        return ipmi::responseUnspecifiedError();
    }
}

static constexpr uint8_t wd_dont_stop = 0x1 << 6;
static constexpr uint8_t wd_timeout_action_mask = 0x3;

static constexpr uint8_t wdTimerUseResTimer1 = 0x0;
static constexpr uint8_t wdTimerUseResTimer2 = 0x6;
static constexpr uint8_t wdTimerUseResTimer3 = 0x7;

static constexpr uint8_t wdTimeoutActionTimer = 0x40;
static constexpr uint8_t wdTimeoutInterruptTimer = 0x04;

enum class IpmiAction : uint8_t
{
    None = 0x0,
    HardReset = 0x1,
    PowerOff = 0x2,
    PowerCycle = 0x3,
};

/** @brief Converts an IPMI Watchdog Action to DBUS defined action
 *  @param[in] ipmi_action The IPMI Watchdog Action
 *  @return The Watchdog Action that the ipmi_action maps to
 */
WatchdogService::Action ipmiActionToWdAction(IpmiAction ipmi_action)
{
    switch (ipmi_action)
    {
        case IpmiAction::None:
        {
            return WatchdogService::Action::None;
        }
        case IpmiAction::HardReset:
        {
            return WatchdogService::Action::HardReset;
        }
        case IpmiAction::PowerOff:
        {
            return WatchdogService::Action::PowerOff;
        }
        case IpmiAction::PowerCycle:
        {
            return WatchdogService::Action::PowerCycle;
        }
        default:
        {
            throw std::domain_error("IPMI Action is invalid");
        }
    }
}

enum class IpmiTimerUse : uint8_t
{
    Reserved = 0x0,
    BIOSFRB2 = 0x1,
    BIOSPOST = 0x2,
    OSLoad = 0x3,
    SMSOS = 0x4,
    OEM = 0x5,
};

WatchdogService::TimerUse ipmiTimerUseToWdTimerUse(IpmiTimerUse ipmiTimerUse)
{
    switch (ipmiTimerUse)
    {
        case IpmiTimerUse::Reserved:
        {
            return WatchdogService::TimerUse::Reserved;
        }
        case IpmiTimerUse::BIOSFRB2:
        {
            return WatchdogService::TimerUse::BIOSFRB2;
        }
        case IpmiTimerUse::BIOSPOST:
        {
            return WatchdogService::TimerUse::BIOSPOST;
        }
        case IpmiTimerUse::OSLoad:
        {
            return WatchdogService::TimerUse::OSLoad;
        }
        case IpmiTimerUse::SMSOS:
        {
            return WatchdogService::TimerUse::SMSOS;
        }
        case IpmiTimerUse::OEM:
        {
            return WatchdogService::TimerUse::OEM;
        }
        default:
        {
            return WatchdogService::TimerUse::Reserved;
        }
    }
}

static uint8_t timerLogFlags = 0;
static uint8_t timerActions = 0;
static uint8_t timerUseExpirationFlags = 0;

/**@brief The Set Watchdog Timer ipmi command.
 *
 * @param
 * - timerUse
 * - dontStopTimer
 * - dontLog
 * - timerAction
 * - pretimeout
 * - expireFlags
 * - initialCountdown
 *
 * @return completion code on success.
 **/
ipmi::RspType<> ipmiSetWatchdogTimer(
    uint3_t timerUse, uint3_t reserved, bool dontStopTimer, bool dontLog,
    uint3_t timeoutAction, uint1_t reserved1, uint3_t preTimeoutInterrupt,
    uint1_t reserved2, uint8_t preTimeoutInterval, uint1_t reserved3,
    uint5_t expFlagValue, uint2_t reserved4, uint16_t initialCountdown)
{
    if ((timerUse == wdTimerUseResTimer1) ||
        (timerUse == wdTimerUseResTimer2) ||
        (timerUse == wdTimerUseResTimer3) ||
        (timeoutAction == wdTimeoutActionTimer) ||
        (preTimeoutInterrupt == wdTimeoutInterruptTimer) ||
        (reserved1 | reserved2 | reserved3 | reserved4))
    {
        return ipmi::responseInvalidFieldRequest();
    }

    if (preTimeoutInterval > (initialCountdown / 10))
    {
        return ipmi::responseInvalidFieldRequest();
    }

    timerLogFlags = static_cast<uint8_t>(dontLog);
    timerActions &= static_cast<uint8_t>(timeoutAction) |
                    static_cast<uint8_t>(preTimeoutInterrupt) << 4;

    try
    {
        WatchdogService wd_service;
        // Stop the timer if the don't stop bit is not set
        if (!(dontStopTimer))
        {
            wd_service.setEnabled(false);
        }

        // Set the action based on the request
        const auto ipmi_action = static_cast<IpmiAction>(
            static_cast<uint8_t>(timeoutAction) & wd_timeout_action_mask);
        wd_service.setExpireAction(ipmiActionToWdAction(ipmi_action));

        const auto ipmiTimerUse =
            static_cast<IpmiTimerUse>(static_cast<uint8_t>(timerUse));
        wd_service.setTimerUse(ipmiTimerUseToWdTimerUse(ipmiTimerUse));

        wd_service.setExpiredTimerUse(WatchdogService::TimerUse::Reserved);

        timerUseExpirationFlags &= static_cast<uint8_t>(~expFlagValue) << 2;

        // Set the new interval and the time remaining deci -> mill seconds
        const uint64_t interval = initialCountdown * 100;
        wd_service.setInterval(interval);
        wd_service.setTimeRemaining(interval);

        // Mark as initialized so that future resets behave correctly
        wd_service.setInitialized(true);

        lastCallSuccessful = true;
        return ipmi::responseSuccess();
    }
    catch (const std::domain_error&)
    {
        return ipmi::responseInvalidFieldRequest();
    }
    catch (const InternalFailure& e)
    {
        reportError();
        return ipmi::responseUnspecifiedError();
    }
    catch (const std::exception& e)
    {
        const std::string e_str = std::string("wd_set: ") + e.what();
        log<level::ERR>(e_str.c_str());
        reportError();
        return ipmi::responseUnspecifiedError();
    }
    catch (...)
    {
        log<level::ERR>("wd_set: Unknown Error");
        reportError();
        return ipmi::responseUnspecifiedError();
    }
}

/** @brief Converts a DBUS Watchdog Action to IPMI defined action
 *  @param[in] wd_action The DBUS Watchdog Action
 *  @return The IpmiAction that the wd_action maps to
 */
IpmiAction wdActionToIpmiAction(WatchdogService::Action wd_action)
{
    switch (wd_action)
    {
        case WatchdogService::Action::None:
        {
            return IpmiAction::None;
        }
        case WatchdogService::Action::HardReset:
        {
            return IpmiAction::HardReset;
        }
        case WatchdogService::Action::PowerOff:
        {
            return IpmiAction::PowerOff;
        }
        case WatchdogService::Action::PowerCycle:
        {
            return IpmiAction::PowerCycle;
        }
        default:
        {
            // We have no method via IPMI to signal that the action is unknown
            // or unmappable in some way.
            // Just ignore the error and return NONE so the host can reconcile.
            return IpmiAction::None;
        }
    }
}

IpmiTimerUse wdTimerUseToIpmiTimerUse(WatchdogService::TimerUse wdTimerUse)
{
    switch (wdTimerUse)
    {
        case WatchdogService::TimerUse::Reserved:
        {
            return IpmiTimerUse::Reserved;
        }
        case WatchdogService::TimerUse::BIOSFRB2:
        {
            return IpmiTimerUse::BIOSFRB2;
        }
        case WatchdogService::TimerUse::BIOSPOST:
        {
            return IpmiTimerUse::BIOSPOST;
        }
        case WatchdogService::TimerUse::OSLoad:
        {
            return IpmiTimerUse::OSLoad;
        }

        case WatchdogService::TimerUse::SMSOS:
        {
            return IpmiTimerUse::SMSOS;
        }
        case WatchdogService::TimerUse::OEM:
        {
            return IpmiTimerUse::OEM;
        }
        default:
        {
            return IpmiTimerUse::Reserved;
        }
    }
}

static constexpr uint8_t wd_running = 0x1 << 6;

/**@brief The getWatchdogTimer ipmi command.
 *
 * @return Completion code plus timer details.
 * - timerUse
 * - timerAction
 * - pretimeout
 * - expireFlags
 * - initialCountdown
 * - presentCountdown
 **/
ipmi::RspType<uint8_t,  // timerUse
              uint8_t,  // timerAction
              uint8_t,  // pretimeout
              uint8_t,  // expireFlags
              uint16_t, // initial Countdown - Little Endian (deciseconds)
              uint16_t  // present Countdown - Little Endian (deciseconds)
              >
    ipmiGetWatchdogTimer()
{
    uint8_t expireFlags = 0;
    uint16_t presentCountdown = 0;
    uint8_t pretimeout = 0;

    try
    {
        WatchdogService wd_service;
        WatchdogService::Properties wd_prop = wd_service.getProperties();

        // Build and return the response
        uint8_t timerUse = 0;
        timerUse |= timerLogFlags;

        uint8_t timerAction = timerActions;

        // Interval and timeRemaining need converted from milli -> deci seconds
        uint16_t initialCountdown = htole16(wd_prop.interval / 100);

        if (wd_prop.expiredTimerUse != WatchdogService::TimerUse::Reserved)
        {
            timerUseExpirationFlags |=
                1 << static_cast<uint8_t>(
                    wdTimerUseToIpmiTimerUse(wd_prop.expiredTimerUse));
        }

        if (wd_prop.enabled)
        {
            timerUse |= wd_running;
            presentCountdown = htole16(wd_prop.timeRemaining / 100);
            expireFlags = 0;
        }
        else
        {
            if (wd_prop.expiredTimerUse == WatchdogService::TimerUse::Reserved)
            {
                presentCountdown = initialCountdown;
                expireFlags = 0;
            }
            else
            {
                presentCountdown = 0;
                expireFlags = timerUseExpirationFlags;
            }
        }

        timerUse |=
            static_cast<uint8_t>(wdTimerUseToIpmiTimerUse(wd_prop.timerUse));

        // TODO: Do something about having pretimeout support
        pretimeout = 0;

        lastCallSuccessful = true;
        return ipmi::responseSuccess(timerUse, timerAction, pretimeout,
                                     expireFlags, initialCountdown,
                                     presentCountdown);
    }
    catch (const InternalFailure& e)
    {
        reportError();
        return ipmi::responseUnspecifiedError();
    }
    catch (const std::exception& e)
    {
        const std::string e_str = std::string("wd_get: ") + e.what();
        log<level::ERR>(e_str.c_str());
        reportError();
        return ipmi::responseUnspecifiedError();
    }
    catch (...)
    {
        log<level::ERR>("wd_get: Unknown Error");
        reportError();
        return ipmi::responseUnspecifiedError();
    }
}
