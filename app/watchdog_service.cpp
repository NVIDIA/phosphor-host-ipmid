#include "watchdog_service.hpp"

#include <sdbusplus/bus.hpp>
#include <sdbusplus/message.hpp>
#include <string>
#include <xyz/openbmc_project/State/Watchdog/server.hpp>

#include "host-ipmid/ipmid-api.h"

using sdbusplus::message::variant_ns::get;
using sdbusplus::message::variant_ns::variant;
using sdbusplus::xyz::openbmc_project::State::server::convertForMessage;
using sdbusplus::xyz::openbmc_project::State::server::Watchdog;

static constexpr char wd_path[] = "/xyz/openbmc_project/watchdog/host0";
static constexpr char wd_intf[] = "xyz.openbmc_project.State.Watchdog";
static constexpr char prop_intf[] = "org.freedesktop.DBus.Properties";

ipmi::ServiceCache WatchdogService::wd_service(wd_intf, wd_path);

WatchdogService::WatchdogService()
    : bus(ipmid_get_sd_bus_connection())
{
}

WatchdogService::Properties WatchdogService::getProperties()
{
    auto request = wd_service.newMethodCall(bus, prop_intf, "GetAll");
    request.append(wd_intf);
    auto response = bus.call(request);
    if (response.is_method_error())
    {
        wd_service.invalidate();
        throw std::runtime_error("Failed to get watchdog properties");
    }

    std::map<std::string, variant<bool, uint64_t, std::string>> properties;
    response.read(properties);
    Properties wd_prop;
    wd_prop.initialized = get<bool>(properties.at("Initialized"));
    wd_prop.enabled = get<bool>(properties.at("Enabled"));
    wd_prop.expireAction = Watchdog::convertActionFromString(
            get<std::string>(properties.at("ExpireAction")));
    wd_prop.interval = get<uint64_t>(properties.at("Interval"));
    wd_prop.timeRemaining = get<uint64_t>(properties.at("TimeRemaining"));
    return wd_prop;
}

template <typename T>
void WatchdogService::setProperty(const std::string& key, const T& val)
{
    auto request = wd_service.newMethodCall(bus, prop_intf, "Set");
    request.append(wd_intf, key, variant<T>(val));
    auto response = bus.call(request);
    if (response.is_method_error())
    {
        wd_service.invalidate();
        throw std::runtime_error(std::string("Failed to set property: ") + key);
    }
}

void WatchdogService::setInitialized(bool initialized)
{
    setProperty("Initialized", initialized);
}

void WatchdogService::setEnabled(bool enabled)
{
    setProperty("Enabled", enabled);
}

void WatchdogService::setExpireAction(Action expireAction)
{
    setProperty("ExpireAction", convertForMessage(expireAction));
}

void WatchdogService::setInterval(uint64_t interval)
{
    setProperty("Interval", interval);
}

void WatchdogService::setTimeRemaining(uint64_t timeRemaining)
{
    setProperty("TimeRemaining", timeRemaining);
}