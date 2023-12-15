/*
// Copyright (c) 2017 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/

#pragma once
#include <dbus-sdr/sdrutils.hpp>

#include <cstdint>

#pragma pack(push, 1)

constexpr auto oemType = "";

struct SensorThresholdResp
{
    uint8_t readable;
    uint8_t lowernc;
    uint8_t lowercritical;
    uint8_t lowernonrecoverable;
    uint8_t uppernc;
    uint8_t uppercritical;
    uint8_t uppernonrecoverable;
};

#pragma pack(pop)

enum class IPMIThresholdRespBits
{
    lowerNonCritical,
    lowerCritical,
    lowerNonRecoverable,
    upperNonCritical,
    upperCritical,
    upperNonRecoverable
};

enum class IPMISensorReadingByte2 : uint8_t
{
    eventMessagesEnable = (1 << 7),
    sensorScanningEnable = (1 << 6),
    readingStateUnavailable = (1 << 5),
};

enum class IPMISensorReadingByte3 : uint8_t
{
    upperNonRecoverable = (1 << 5),
    upperCritical = (1 << 4),
    upperNonCritical = (1 << 3),
    lowerNonRecoverable = (1 << 2),
    lowerCritical = (1 << 1),
    lowerNonCritical = (1 << 0),
    presenceDetected = (1 << 0),
    procPresenceDetected = (1 << 7),
    cableStatus = (1 << 0),
    configurationError = (1 << 1),
    drivePresenceDetected = (1 << 0),
    driveFault = (1 << 1),
    drivePredictiveFailure = (1 << 2),
    failureDetected = (1 << 1),
    inputLost = (1 << 3),
    watchdogExpire = (1 << 0),
    watchdogHardReset = (1 << 1),
    watchdogPowerOff = (1 << 2),
    watchdogPowerCycle = (1 << 3),
    fullyRedundant = (1 << 0),
    redundancyLost = (1 << 1),
    redundancyDegraded = (1 << 2),
    sufficientFromRedundant = (1 << 3),
    sufficientFromInsufficient = (1 << 4),
    insufficient = (1 << 5),
    degrardedFromFull = (1 << 6),
    degradedFromNonRedundant = (1 << 7),
    selCleared = (1 << 2),
    selFull = (1 << 4),
    selAlmostFull = (1 << 5),
};

enum class IPMISensorEventEnableByte2 : uint8_t
{
    eventMessagesEnable = (1 << 7),
    sensorScanningEnable = (1 << 6),
};

enum class IPMISensorEventEnableThresholds : uint8_t
{
    nonRecoverableThreshold = (1 << 6),
    criticalThreshold = (1 << 5),
    nonCriticalThreshold = (1 << 4),
    upperNonRecoverableGoingHigh = (1 << 3),
    upperNonRecoverableGoingLow = (1 << 2),
    upperCriticalGoingHigh = (1 << 1),
    upperCriticalGoingLow = (1 << 0),
    upperNonCriticalGoingHigh = (1 << 7),
    upperNonCriticalGoingLow = (1 << 6),
    lowerNonRecoverableGoingHigh = (1 << 5),
    lowerNonRecoverableGoingLow = (1 << 4),
    lowerCriticalGoingHigh = (1 << 3),
    lowerCriticalGoingLow = (1 << 2),
    lowerNonCriticalGoingHigh = (1 << 1),
    lowerNonCriticalGoingLow = (1 << 0),
};

enum class IPMIGetSensorEventEnableThresholds : uint8_t
{
    lowerNonCriticalGoingLow = 0,
    lowerNonCriticalGoingHigh = 1,
    lowerCriticalGoingLow = 2,
    lowerCriticalGoingHigh = 3,
    lowerNonRecoverableGoingLow = 4,
    lowerNonRecoverableGoingHigh = 5,
    upperNonCriticalGoingLow = 6,
    upperNonCriticalGoingHigh = 7,
    upperCriticalGoingLow = 8,
    upperCriticalGoingHigh = 9,
    upperNonRecoverableGoingLow = 10,
    upperNonRecoverableGoingHigh = 11,
};

/**
 * @enum power_supply sensor event enable bit mask
 */
enum class IPMISensorEventEnablePower : uint8_t
{
    presenceDetected = (1 << 0),
    failureDetected = (1 << 1),
    inputLost = (1 << 3),
};

/**
 * @enum processor sensor event enable bit mask
 */
enum class IPMISensorEventEnableProc : uint8_t
{
    procPresenceDetected = (1 << 7),
};

/**
 * @enum ipmb discrete sensor event enable bit mask
 */
enum class IPMISensorEventEnableCable : uint8_t
{
    cableStatus = (1 << 0),
    configurationError = (1 << 1),
};

/**
 * @enum drive slot sensor event enable bit mask
 */
enum class IPMISensorEventEnableDrive : uint8_t
{
    drivePresenceDetected = (1 << 0),
    driveFault = (1 << 1),
    drivePredictiveFailure = (1 << 2),
};

/**
 * @enum watchdog sensor event enable bit mask
 */
enum class IPMISensorEventEnableWatchdog : uint8_t
{
    timerExpired = (1 << 0),
    hardReset = (1 << 1),
    powerDown = (1 << 2),
    powerCycle = (1 << 3),
};

/**
 * @enum event logging sensor event enable bit mask
 * when sel record is cleared 2nd bit will be asseted
 * when sel record is full 4th bit will be asseted
 * when sel record is in 75% capacity 5th byte will be asseted
 */
enum class IPMISensorEventEnableSEL : uint8_t
{
    cleared = (1 << 2),
    full = (1 << 4),
    almostFull = (1 << 5),
};

/**
 * @enum redundancy sensor event enable bit mask
 * when full redundancy is gained 0th bit will be asserted
 * when entered any non-redundant state 1st bit will be asserted
 * when redundancy still exists, but less than full level,2nd bit
 *   will get asserted.
 * when redundancy is lost but unit is functioning with minumum
 *   resource needed for 'normal' operation 3rd bit asserted
 * when unit regained minimum resources needed for 'normal' operation
 *  4th bit asserted
 * when unit is non-redundant and has insufficient resource to maintain
 *  normal operation, 5th bit asserted
 * when Unit has lost some redundant resource but is still in redundant
 *  state, 6th bit asserted
 * when unit regained some resources  and is redundant but not fully
 * redundant , 7th bit aserted
 */
enum class IPMISensorEventEnableRedundancy : uint8_t
{
    fullyRedundant = (1 << 0),
    redundancyLost = (1 << 1),
    redundancyDegraded = (1 << 2),
    sufficientFromRedundant = (1 << 3),
    sufficientFromInsufficient = (1 << 4),
    insufficient = (1 << 5),
    degrardedFromFull = (1 << 6),
    degradedFromNonRedundant = (1 << 7),
};

/**
 * @enum GPU sensor event enable bit mask
 */
enum class IPMISensorEventEnableGPU : uint8_t
{
    resetRequired = 0x0f,
};

enum class IPMINetfnSensorCmds : ipmi_cmd_t
{
    ipmiCmdGetDeviceSDRInfo = 0x20,
    ipmiCmdGetDeviceSDR = 0x21,
    ipmiCmdReserveDeviceSDRRepo = 0x22,
    ipmiCmdSetSensorThreshold = 0x26,
    ipmiCmdGetSensorThreshold = 0x27,
    ipmiCmdGetSensorEventEnable = 0x29,
    ipmiCmdGetSensorEventStatus = 0x2B,
    ipmiCmdGetSensorReading = 0x2D,
    ipmiCmdGetSensorType = 0x2F,
    ipmiCmdSetSensorReadingAndEventStatus = 0x30,
};

namespace ipmi
{

SensorSubTree& getSensorTree()
{
    static SensorSubTree sensorTree;
    return sensorTree;
}

static ipmi_ret_t
    getSensorConnection(ipmi::Context::ptr ctx, uint8_t sensnum,
                        std::string& connection, std::string& path,
                        std::vector<std::string>* interfaces = nullptr)
{
    auto& sensorTree = getSensorTree();
    if (!getSensorSubtree(sensorTree) && sensorTree.empty())
    {
        return IPMI_CC_RESPONSE_ERROR;
    }

    if (ctx == nullptr)
    {
        return IPMI_CC_RESPONSE_ERROR;
    }

    path = getPathFromSensorNumber((ctx->lun << 8) | sensnum);
    if (path.empty())
    {
        return IPMI_CC_INVALID_FIELD_REQUEST;
    }

    for (const auto& sensor : sensorTree)
    {
        if (path == sensor.first)
        {
            connection = sensor.second.begin()->first;
            if (interfaces)
                *interfaces = sensor.second.begin()->second;
            break;
        }
    }

    return 0;
}

struct IPMIThresholds
{
    std::optional<uint8_t> warningLow;
    std::optional<uint8_t> warningHigh;
    std::optional<uint8_t> criticalLow;
    std::optional<uint8_t> criticalHigh;
};
namespace dcmi
{
    constexpr auto groupExtIpmi = 0xdc;

    constexpr auto cmdGetSensorInfo = 0x07;

}

namespace dcmi
{

struct sensorInfo
{
    std::string objectPath;
    uint8_t type;
    uint16_t recordId;
    uint8_t entityId;
    uint8_t entityInstance;
};

} // namespace dcmi

} // namespace ipmi
