/*
// Copyright (c) 2018 Intel Corporation
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

#include "dbus-sdr/sdrutils.hpp"

#include "dbus-sdr/sensorutils.hpp"
#include <optional>
#include <unordered_set>

#ifdef FEATURE_HYBRID_SENSORS

#include <ipmid/utils.hpp>
namespace ipmi
{
namespace sensor
{
extern const IdInfoMap sensors;
} // namespace sensor
} // namespace ipmi

#endif

namespace details
{
uint16_t getSensorSubtree(std::shared_ptr<SensorSubTree>& subtree)
{
    static std::shared_ptr<SensorSubTree> sensorTreePtr;
    static uint16_t sensorUpdatedIndex = 0;
    std::shared_ptr<sdbusplus::asio::connection> dbus = getSdBus();
    static sdbusplus::bus::match_t sensorAdded(
        *dbus,
        "type='signal',member='InterfacesAdded',arg0path='/xyz/openbmc_project/"
        "sensors/'",
        [](sdbusplus::message_t&) { sensorTreePtr.reset(); });

    static sdbusplus::bus::match_t sensorRemoved(
        *dbus,
        "type='signal',member='InterfacesRemoved',arg0path='/xyz/"
        "openbmc_project/sensors/'",
        [](sdbusplus::message_t&) { sensorTreePtr.reset(); });

    if (sensorTreePtr)
    {
        subtree = sensorTreePtr;
        return sensorUpdatedIndex;
    }

    sensorTreePtr = std::make_shared<SensorSubTree>();

    static constexpr const int32_t depth = 2;

    auto lbdUpdateSensorTree = [&dbus](const char* path,
                                       const auto& interfaces) {
        auto mapperCall = dbus->new_method_call(
            "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper", "GetSubTree");
        SensorSubTree sensorTreePartial;

        mapperCall.append(path, depth, interfaces);

        try
        {
            auto mapperReply = dbus->call(mapperCall);
            mapperReply.read(sensorTreePartial);
        }
        catch (const sdbusplus::exception_t& e)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "fail to update subtree",
                phosphor::logging::entry("PATH=%s", path),
                phosphor::logging::entry("WHAT=%s", e.what()));
            return false;
        }
        if constexpr (debug)
        {
            std::fprintf(stderr, "IPMI updated: %zu sensors under %s\n",
                         sensorTreePartial.size(), path);
        }
        sensorTreePtr->merge(std::move(sensorTreePartial));
        return true;
    };

    // Add sensors to SensorTree
    static constexpr const std::array sensorInterfaces = {
        "xyz.openbmc_project.Sensor.Value",
        "xyz.openbmc_project.Sensor.ValueMutability",
        "xyz.openbmc_project.Sensor.Threshold.Warning",
        "xyz.openbmc_project.Sensor.Threshold.Critical"};
    static constexpr const std::array vrInterfaces = {
        "xyz.openbmc_project.Control.VoltageRegulatorMode"};
    static constexpr const std::array discreteInterfaces = {
        "xyz.openbmc_project.Inventory.Item.PowerSupply",
        "xyz.openbmc_project.Inventory.Item.Cpu",
        "xyz.openbmc_project.Inventory.Item.Cable",
        "xyz.openbmc_project.Inventory.Item.Drive",
        "xyz.openbmc_project.Inventory.Item.Watchdog",
        "xyz.openbmc_project.Control.PowerSupplyRedundancy",
        "xyz.openbmc_project.Inventory.Item.SEL",
        "xyz.openbmc_project.Inventory.Item.GPU"};
	static constexpr const std::array bootProgressInterfaces = {
        "xyz.openbmc_project.State.Boot.Progress"};

    bool sensorRez = lbdUpdateSensorTree("/xyz/openbmc_project/sensors",
                                         sensorInterfaces);
#ifdef FEATURE_HYBRID_SENSORS

    if (!ipmi::sensor::sensors.empty())
    {
        for (const auto& sensor : ipmi::sensor::sensors)
        {
            // Threshold sensors should not be emplaced in here.
            if (boost::starts_with(sensor.second.sensorPath,
                                   "/xyz/openbmc_project/sensors/"))
            {
                continue;
            }

            // The bus service name is not listed in ipmi::sensor::Info. Give it
            // an empty string. For those function using non-threshold sensors,
            // the bus service name will be retrieved in an alternative way.
            boost::container::flat_map<std::string, std::vector<std::string>>
                connectionMap{
                    {"", {sensor.second.propertyInterfaces.begin()->first}}};
            sensorTreePtr->emplace(sensor.second.sensorPath, connectionMap);
        }
    }

#endif

    // Error if searching for sensors failed.
    if (!sensorRez)
    {
        return sensorUpdatedIndex;
    }

#ifdef ENABLE_DYNAMIC_SENSORS_REMOVE_EXCEEDED_SCALE
    auto checksdrUpdateSensorTree = [&dbus]() {
        std::vector<std::string> removeobjecpaths;
        constexpr auto valueInterface = "xyz.openbmc_project.Sensor.Value";
        constexpr auto availabilityInterface = "xyz.openbmc_project.State.Decorator.Availability";
        for (const auto& sensor : *sensorTreePtr)
        {
            const std::string& sensorObjPath = sensor.first;
            const std::string& service = sensor.second.begin()->first.c_str();
            double max = 127;
            double min = -128;

            auto method =
            dbus->new_method_call(service.c_str(), sensorObjPath.c_str(),
                                "org.freedesktop.DBus.Properties", "Get");
            method.append(valueInterface, "MaxValue");
            try
            {
                std::variant<double> value;
                auto reply =  dbus->call(method);
                reply.read(value);
                max=std::get<double>(value);
            }
            catch (const sdbusplus::exception_t& ex)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "failed to get maximum value property");
                continue;
            }

            method =
            dbus->new_method_call(service.c_str(), sensorObjPath.c_str(),
                                "org.freedesktop.DBus.Properties", "Get");
            method.append(valueInterface, "MinValue");
            try
            {
                std::variant<double> value;
                auto reply =  dbus->call(method);
                reply.read(value);
                min=std::get<double>(value);
            }
            catch (const sdbusplus::exception_t& ex)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "failed to get minimum value property");
                continue;
            }

            int16_t mValue = 0;
            int16_t bValue = 0;
            int8_t rExp = 0;
            int8_t bExp = 0;
            bool bSigned = false;

            if (!ipmi::getSensorAttributes(max, min, mValue, rExp, bValue, bExp, bSigned))
            {
                 removeobjecpaths.emplace_back(sensorObjPath);
                 continue;
            }

            // Sensor Availability check
            bool isAvailable = true;
            auto method2 =
                dbus->new_method_call(service.c_str(), sensorObjPath.c_str(),
                                      "org.freedesktop.DBus.Properties", "Get");
            method2.append(availabilityInterface, "Available");
            try
            {
                std::variant<bool> available;
                auto reply =  dbus->call(method2);
                reply.read(available);
                isAvailable = std::get<bool>(available);
            }
            catch (const sdbusplus::exception_t& ex)
            {
                // Default set the sensor available as true if the 'available' property is not present
                isAvailable = true;
            }
            if (isAvailable == false)
            {
                removeobjecpaths.emplace_back(sensorObjPath);
            }
        }
        // Remove if cannot calculate sdr
        for (const auto& objpath : removeobjecpaths)
        {
            sensorTreePtr->erase(objpath);
        }
    };

    // Filter out over
    (void)checksdrUpdateSensorTree();
#endif
    // Add VR control as optional search path.
    (void)lbdUpdateSensorTree("/xyz/openbmc_project/vr", vrInterfaces);
    // Add Power Supply sensors
    (void)lbdUpdateSensorTree("/xyz/openbmc_project/sensors/system/chassis",
                              discreteInterfaces);
    // Add discrete sensors
    (void)lbdUpdateSensorTree("/xyz/openbmc_project/sensors/motherboard",
                              discreteInterfaces);
    (void)lbdUpdateSensorTree( "/xyz/openbmc_project/sensors/cable",
                              discreteInterfaces);
    (void)lbdUpdateSensorTree("/xyz/openbmc_project/sensors/power",
                              discreteInterfaces);
    (void)lbdUpdateSensorTree("/xyz/openbmc_project/sensors/gpuboard/",
                              discreteInterfaces);
    (void)lbdUpdateSensorTree("/xyz/openbmc_project/sensors/drive",
                              discreteInterfaces);
    // Add boot progress sensor
    (void)lbdUpdateSensorTree("/xyz/openbmc_project/state",
                              bootProgressInterfaces);

    subtree = sensorTreePtr;
    sensorUpdatedIndex++;
    // The SDR is being regenerated, wipe the old stats
    sdrStatsTable.wipeTable();
    sdrWriteTable.wipeTable();
    return sensorUpdatedIndex;
}

bool getSensorNumMap(std::shared_ptr<SensorNumMap>& sensorNumMap)
{
    static std::shared_ptr<SensorNumMap> sensorNumMapPtr;
    bool sensorNumMapUpated = false;
    static uint16_t prevSensorUpdatedIndex = 0;
    std::shared_ptr<SensorSubTree> sensorTree;
    uint16_t curSensorUpdatedIndex = details::getSensorSubtree(sensorTree);
    if (!sensorTree)
    {
        return sensorNumMapUpated;
    }

    if ((curSensorUpdatedIndex == prevSensorUpdatedIndex) && sensorNumMapPtr)
    {
        sensorNumMap = sensorNumMapPtr;
        return sensorNumMapUpated;
    }
    prevSensorUpdatedIndex = curSensorUpdatedIndex;

    sensorNumMapPtr = std::make_shared<SensorNumMap>();

    uint16_t sensorNum = 0;
    uint16_t sensorIndex = 0;
    for (const auto& sensor : *sensorTree)
    {
        sensorNumMapPtr->insert(
            SensorNumMap::value_type(sensorNum, sensor.first));
        sensorIndex++;
        if (sensorIndex == maxSensorsPerLUN)
        {
            sensorIndex = lun1Sensor0;
        }
        else if (sensorIndex == (lun1Sensor0 | maxSensorsPerLUN))
        {
            // Skip assigning LUN 0x2 any sensors
            sensorIndex = lun3Sensor0;
        }
        else if (sensorIndex == (lun3Sensor0 | maxSensorsPerLUN))
        {
            // this is an error, too many IPMI sensors
            throw std::out_of_range("Maximum number of IPMI sensors exceeded.");
        }
        sensorNum = sensorIndex;
    }
    sensorNumMap = sensorNumMapPtr;
    sensorNumMapUpated = true;
    return sensorNumMapUpated;
}
} // namespace details

bool getSensorSubtree(SensorSubTree& subtree)
{
    std::shared_ptr<SensorSubTree> sensorTree;
    details::getSensorSubtree(sensorTree);
    if (!sensorTree)
    {
        return false;
    }

    subtree = *sensorTree;
    return true;
}

#ifdef FEATURE_HYBRID_SENSORS
// Static sensors are listed in sensor-gen.cpp.
ipmi::sensor::IdInfoMap::const_iterator
    findStaticSensor(const std::string& path)
{
    return std::find_if(
        ipmi::sensor::sensors.begin(), ipmi::sensor::sensors.end(),
        [&path](const ipmi::sensor::IdInfoMap::value_type& findSensor) {
        return findSensor.second.sensorPath == path;
    });
}
#endif

std::string getSensorTypeStringFromPath(const std::string& path)
{
    // get sensor type string from path, path is defined as
    // /xyz/openbmc_project/sensors/<type>/label
    size_t typeEnd = path.rfind("/");
    if (typeEnd == std::string::npos)
    {
        return path;
    }
    size_t typeStart = path.rfind("/", typeEnd - 1);
    if (typeStart == std::string::npos)
    {
        return path;
    }
    // Start at the character after the '/'
    typeStart++;
    return path.substr(typeStart, typeEnd - typeStart);
}

uint8_t getSensorTypeFromPath(const std::string& path)
{
    uint8_t sensorType = 0;
    std::string type = getSensorTypeStringFromPath(path);
    auto findSensor = sensorTypes.find(type.c_str());
    if (findSensor != sensorTypes.end())
    {
        sensorType =
            static_cast<uint8_t>(std::get<sensorTypeCodes>(findSensor->second));
    } // else default 0x0 RESERVED

    return sensorType;
}

uint16_t getSensorNumberFromPath(const std::string& path)
{
    std::shared_ptr<SensorNumMap> sensorNumMapPtr;
    details::getSensorNumMap(sensorNumMapPtr);
    if (!sensorNumMapPtr)
    {
        return invalidSensorNumber;
    }

    try
    {
        return sensorNumMapPtr->right.at(path);
    }
    catch (const std::out_of_range& e)
    {
        return invalidSensorNumber;
    }
}

uint8_t getSensorEventTypeFromPath(const std::string& path)
{
    uint8_t sensorEventType = 0;
    std::string type = getSensorTypeStringFromPath(path);
    auto findSensor = sensorTypes.find(type.c_str());
    if (findSensor != sensorTypes.end())
    {
        sensorEventType = static_cast<uint8_t>(
            std::get<sensorEventTypeCodes>(findSensor->second));
    }

    return sensorEventType;
}

std::string getPathFromSensorNumber(uint16_t sensorNum)
{
    std::shared_ptr<SensorNumMap> sensorNumMapPtr;
    details::getSensorNumMap(sensorNumMapPtr);
    if (!sensorNumMapPtr)
    {
        return std::string();
    }

    try
    {
        return sensorNumMapPtr->left.at(sensorNum);
    }
    catch (const std::out_of_range& e)
    {
        return std::string();
    }
}

/**
 * @brief Gets entityInstance from sensor name
 *
 * @param name - sensor name
 * @return entityInstance - returns entity instance
 */
uint8_t getEntityInstanceFromName(const std::string& name)
{
    uint8_t entityInstance = 0x01;

    std::size_t instanceLast = name.size();

    std::size_t instanceFirst = name.find_last_not_of("0123456789");
    if (instanceFirst == std::string::npos)
    {
        return entityInstance;
    }
    instanceFirst++;
    if (instanceFirst != instanceLast)
    {
        entityInstance = std::stoi(name.substr(instanceFirst, instanceLast));
    }

    return entityInstance;
}

namespace ipmi
{


struct ipmi_event_sensor_types {
	uint8_t	code;
	uint8_t	offset;
	uint8_t   data;
	const char	* desc;
};

const struct ipmi_event_sensor_types generic_event_types[] = {
    /* Threshold Based States */
    { 0x01, 0x00, 0xff, "Lower Non-critical going low " },
    { 0x01, 0x01, 0xff, "Lower Non-critical going high" },
    { 0x01, 0x02, 0xff, "Lower Critical going low " },
    { 0x01, 0x03, 0xff, "Lower Critical going high" },
    { 0x01, 0x04, 0xff, "Lower Non-recoverable going low " },
    { 0x01, 0x05, 0xff, "Lower Non-recoverable going high" },
    { 0x01, 0x06, 0xff, "Upper Non-critical going low " },
    { 0x01, 0x07, 0xff, "Upper Non-critical going high" },
    { 0x01, 0x08, 0xff, "Upper Critical going low " },
    { 0x01, 0x09, 0xff, "Upper Critical going high" },
    { 0x01, 0x0a, 0xff, "Upper Non-recoverable going low " },
    { 0x01, 0x0b, 0xff, "Upper Non-recoverable going high" },
    /* DMI-based "usage state" States */
    { 0x02, 0x00, 0xff, "Transition to Idle" },
    { 0x02, 0x01, 0xff, "Transition to Active" },
    { 0x02, 0x02, 0xff, "Transition to Busy" },
    /* Digital-Discrete Event States */
    { 0x03, 0x00, 0xff, "State Deasserted" },
    { 0x03, 0x01, 0xff, "State Asserted" },
    { 0x04, 0x00, 0xff, "Predictive Failure Deasserted" },
    { 0x04, 0x01, 0xff, "Predictive Failure Asserted" },
    { 0x05, 0x00, 0xff, "Limit Not Exceeded" },
    { 0x05, 0x01, 0xff, "Limit Exceeded" },
    { 0x06, 0x00, 0xff, "Performance Met" },
    { 0x06, 0x01, 0xff, "Performance Lags" },
    /* Severity Event States */
    { 0x07, 0x00, 0xff, "Transition to OK" },
    { 0x07, 0x01, 0xff, "Transition to Non-critical from OK" },
    { 0x07, 0x02, 0xff, "Transition to Critical from less severe" },
    { 0x07, 0x03, 0xff, "Transition to Non-recoverable from less severe" },
    { 0x07, 0x04, 0xff, "Transition to Non-critical from more severe" },
    { 0x07, 0x05, 0xff, "Transition to Critical from Non-recoverable" },
    { 0x07, 0x06, 0xff, "Transition to Non-recoverable" },
    { 0x07, 0x07, 0xff, "Monitor" },
    { 0x07, 0x08, 0xff, "Informational" },
    /* Availability Status States */
    { 0x08, 0x00, 0xff, "Device Absent" },
    { 0x08, 0x01, 0xff, "Device Present" },
    { 0x09, 0x00, 0xff, "Device Disabled" },
    { 0x09, 0x01, 0xff, "Device Enabled" },
    { 0x0a, 0x00, 0xff, "Transition to Running" },
    { 0x0a, 0x01, 0xff, "Transition to In Test" },
    { 0x0a, 0x02, 0xff, "Transition to Power Off" },
    { 0x0a, 0x03, 0xff, "Transition to On Line" },
    { 0x0a, 0x04, 0xff, "Transition to Off Line" },
    { 0x0a, 0x05, 0xff, "Transition to Off Duty" },
    { 0x0a, 0x06, 0xff, "Transition to Degraded" },
    { 0x0a, 0x07, 0xff, "Transition to Power Save" },
    { 0x0a, 0x08, 0xff, "Install Error" },
    /* Redundancy States */
    { 0x0b, 0x00, 0xff, "Fully Redundant" },
    { 0x0b, 0x01, 0xff, "Redundancy Lost" },
    { 0x0b, 0x02, 0xff, "Redundancy Degraded" },
    { 0x0b, 0x03, 0xff, "Non-Redundant: Sufficient from Redundant" },
    { 0x0b, 0x04, 0xff, "Non-Redundant: Sufficient from Insufficient" },
    { 0x0b, 0x05, 0xff, "Non-Redundant: Insufficient Resources" },
    { 0x0b, 0x06, 0xff, "Redundancy Degraded from Fully Redundant" },
    { 0x0b, 0x07, 0xff, "Redundancy Degraded from Non-Redundant" },
    /* ACPI Device Power States */
    { 0x0c, 0x00, 0xff, "D0 Power State" },
    { 0x0c, 0x01, 0xff, "D1 Power State" },
    { 0x0c, 0x02, 0xff, "D2 Power State" },
    { 0x0c, 0x03, 0xff, "D3 Power State" },
    /* END */
    { 0x00, 0x00, 0xff, NULL },
};

const struct ipmi_event_sensor_types sensor_specific_event_types[] = {
    /* Physical Security */
    { 0x05, 0x00, 0xff, "General Chassis intrusion" },
    { 0x05, 0x01, 0xff, "Drive Bay intrusion" },
    { 0x05, 0x02, 0xff, "I/O Card area intrusion" },
    { 0x05, 0x03, 0xff, "Processor area intrusion" },
    { 0x05, 0x04, 0xff, "System unplugged from LAN" },
    { 0x05, 0x05, 0xff, "Unauthorized dock" },
    { 0x05, 0x06, 0xff, "FAN area intrusion" },
    /* Platform Security */
    { 0x06, 0x00, 0xff, "Front Panel Lockout violation attempted" },
    { 0x06, 0x01, 0xff, "Pre-boot password violation - user password" },
    { 0x06, 0x02, 0xff, "Pre-boot password violation - setup password" },
    { 0x06, 0x03, 0xff, "Pre-boot password violation - network boot password" },
    { 0x06, 0x04, 0xff, "Other pre-boot password violation" },
    { 0x06, 0x05, 0xff, "Out-of-band access password violation" },
    /* Processor */
    { 0x07, 0x00, 0xff, "IERR" },
    { 0x07, 0x01, 0xff, "Thermal Trip" },
    { 0x07, 0x02, 0xff, "FRB1/BIST failure" },
    { 0x07, 0x03, 0xff, "FRB2/Hang in POST failure" },
    { 0x07, 0x04, 0xff, "FRB3/Processor startup/init failure" },
    { 0x07, 0x05, 0xff, "Configuration Error" },
    { 0x07, 0x06, 0xff, "SM BIOS Uncorrectable CPU-complex Error" },
    { 0x07, 0x07, 0xff, "Presence detected" },
    { 0x07, 0x08, 0xff, "Disabled" },
    { 0x07, 0x09, 0xff, "Terminator presence detected" },
    { 0x07, 0x0a, 0xff, "Throttled" },
    { 0x07, 0x0b, 0xff, "Uncorrectable machine check exception" },
    { 0x07, 0x0c, 0xff, "Correctable machine check error" },
    /* Power Supply */
    { 0x08, 0x00, 0xff, "Presence detected" },
    { 0x08, 0x01, 0xff, "Failure detected" },
    { 0x08, 0x02, 0xff, "Predictive failure" },
    { 0x08, 0x03, 0xff, "Power Supply AC lost" },
    { 0x08, 0x04, 0xff, "AC lost or out-of-range" },
    { 0x08, 0x05, 0xff, "AC out-of-range, but present" },
    { 0x08, 0x06, 0x00, "Config Error: Vendor Mismatch" },
    { 0x08, 0x06, 0x01, "Config Error: Revision Mismatch" },
    { 0x08, 0x06, 0x02, "Config Error: Processor Missing" },
    { 0x08, 0x06, 0x03, "Config Error: Power Supply Rating Mismatch" },
    { 0x08, 0x06, 0x04, "Config Error: Voltage Rating Mismatch" },
    { 0x08, 0x06, 0xff, "Config Error" },
    { 0x08, 0x06, 0xff, "Power Supply Inactive" },
    /* Power Unit */
    { 0x09, 0x00, 0xff, "Power off/down" },
    { 0x09, 0x01, 0xff, "Power cycle" },
    { 0x09, 0x02, 0xff, "240VA power down" },
    { 0x09, 0x03, 0xff, "Interlock power down" },
    { 0x09, 0x04, 0xff, "AC lost" },
    { 0x09, 0x05, 0xff, "Soft-power control failure" },
    { 0x09, 0x06, 0xff, "Failure detected" },
    { 0x09, 0x07, 0xff, "Predictive failure" },
    /* Memory */
    { 0x0c, 0x00, 0xff, "Correctable ECC" },
    { 0x0c, 0x01, 0xff, "Uncorrectable ECC" },
    { 0x0c, 0x02, 0xff, "Parity" },
    { 0x0c, 0x03, 0xff, "Memory Scrub Failed" },
    { 0x0c, 0x04, 0xff, "Memory Device Disabled" },
    { 0x0c, 0x05, 0xff, "Correctable ECC logging limit reached" },
    { 0x0c, 0x06, 0xff, "Presence Detected" },
    { 0x0c, 0x07, 0xff, "Configuration Error" },
    { 0x0c, 0x08, 0xff, "Spare" },
    { 0x0c, 0x09, 0xff, "Throttled" },
    { 0x0c, 0x0a, 0xff, "Critical Overtemperature" },
    /* Drive Slot */
    { 0x0d, 0x00, 0xff, "Drive Present" },
    { 0x0d, 0x01, 0xff, "Drive Fault" },
    { 0x0d, 0x02, 0xff, "Predictive Failure" },
    { 0x0d, 0x03, 0xff, "Hot Spare" },
    { 0x0d, 0x04, 0xff, "Parity Check In Progress" },
    { 0x0d, 0x05, 0xff, "In Critical Array" },
    { 0x0d, 0x06, 0xff, "In Failed Array" },
    { 0x0d, 0x07, 0xff, "Rebuild In Progress" },
    { 0x0d, 0x08, 0xff, "Rebuild Aborted" },
    /* System Firmware Error */
    { 0x0f, 0x00, 0x00, "Unspecified" },
    { 0x0f, 0x00, 0x01, "No system memory installed" },
    { 0x0f, 0x00, 0x02, "No usable system memory" },
    { 0x0f, 0x00, 0x03, "Unrecoverable IDE device failure" },
    { 0x0f, 0x00, 0x04, "Unrecoverable system-board failure" },
    { 0x0f, 0x00, 0x05, "Unrecoverable diskette failure" },
    { 0x0f, 0x00, 0x06, "Unrecoverable hard-disk controller failure" },
    { 0x0f, 0x00, 0x07, "Unrecoverable PS/2 or USB keyboard failure" },
    { 0x0f, 0x00, 0x08, "Removable boot media not found" },
    { 0x0f, 0x00, 0x09, "Unrecoverable video controller failure" },
    { 0x0f, 0x00, 0x0a, "No video device selected" },
    { 0x0f, 0x00, 0x0b, "BIOS corruption detected" },
    { 0x0f, 0x00, 0x0c, "CPU voltage mismatch" },
    { 0x0f, 0x00, 0x0d, "CPU speed mismatch failure" },
    { 0x0f, 0x00, 0xff, "Unknown Error" },
    /* System Firmware Hang */
    { 0x0f, 0x01, 0x00, "Unspecified" },
    { 0x0f, 0x01, 0x01, "Memory initialization" },
    { 0x0f, 0x01, 0x02, "Hard-disk initialization" },
    { 0x0f, 0x01, 0x03, "Secondary CPU Initialization" },
    { 0x0f, 0x01, 0x04, "User authentication" },
    { 0x0f, 0x01, 0x05, "User-initiated system setup" },
    { 0x0f, 0x01, 0x06, "USB resource configuration" },
    { 0x0f, 0x01, 0x07, "PCI resource configuration" },
    { 0x0f, 0x01, 0x08, "Option ROM initialization" },
    { 0x0f, 0x01, 0x09, "Video initialization" },
    { 0x0f, 0x01, 0x0a, "Cache initialization" },
    { 0x0f, 0x01, 0x0b, "SMBus initialization" },
    { 0x0f, 0x01, 0x0c, "Keyboard controller initialization" },
    { 0x0f, 0x01, 0x0d, "Management controller initialization" },
    { 0x0f, 0x01, 0x0e, "Docking station attachment" },
    { 0x0f, 0x01, 0x0f, "Enabling docking station" },
    { 0x0f, 0x01, 0x10, "Docking station ejection" },
    { 0x0f, 0x01, 0x11, "Disabling docking station" },
    { 0x0f, 0x01, 0x12, "Calling operating system wake-up vector" },
    { 0x0f, 0x01, 0x13, "System boot initiated" },
    { 0x0f, 0x01, 0x14, "Motherboard initialization" },
    { 0x0f, 0x01, 0x15, "reserved" },
    { 0x0f, 0x01, 0x16, "Floppy initialization" },
    { 0x0f, 0x01, 0x17, "Keyboard test" },
    { 0x0f, 0x01, 0x18, "Pointing device test" },
    { 0x0f, 0x01, 0x19, "Primary CPU initialization" },
    { 0x0f, 0x01, 0xff, "Unknown Hang" },
    /* System Firmware Progress */
    { 0x0f, 0x02, 0x00, "Unspecified" },
    { 0x0f, 0x02, 0x01, "Memory initialization" },
    { 0x0f, 0x02, 0x02, "Hard-disk initialization" },
    { 0x0f, 0x02, 0x03, "Secondary CPU Initialization" },
    { 0x0f, 0x02, 0x04, "User authentication" },
    { 0x0f, 0x02, 0x05, "User-initiated system setup" },
    { 0x0f, 0x02, 0x06, "USB resource configuration" },
    { 0x0f, 0x02, 0x07, "PCI resource configuration" },
    { 0x0f, 0x02, 0x08, "Option ROM initialization" },
    { 0x0f, 0x02, 0x09, "Video initialization" },
    { 0x0f, 0x02, 0x0a, "Cache initialization" },
    { 0x0f, 0x02, 0x0b, "SMBus initialization" },
    { 0x0f, 0x02, 0x0c, "Keyboard controller initialization" },
    { 0x0f, 0x02, 0x0d, "Management controller initialization" },
    { 0x0f, 0x02, 0x0e, "Docking station attachment" },
    { 0x0f, 0x02, 0x0f, "Enabling docking station" },
    { 0x0f, 0x02, 0x10, "Docking station ejection" },
    { 0x0f, 0x02, 0x11, "Disabling docking station" },
    { 0x0f, 0x02, 0x12, "Calling operating system wake-up vector" },
    { 0x0f, 0x02, 0x13, "System boot initiated" },
    { 0x0f, 0x02, 0x14, "Motherboard initialization" },
    { 0x0f, 0x02, 0x15, "reserved" },
    { 0x0f, 0x02, 0x16, "Floppy initialization" },
    { 0x0f, 0x02, 0x17, "Keyboard test" },
    { 0x0f, 0x02, 0x18, "Pointing device test" },
    { 0x0f, 0x02, 0x19, "Primary CPU initialization" },
    { 0x0f, 0x02, 0xff, "Unknown Progress" },
    /* Event Logging Disabled */
    { 0x10, 0x00, 0xff, "Correctable memory error logging disabled" },
    { 0x10, 0x01, 0xff, "Event logging disabled" },
    { 0x10, 0x02, 0xff, "Log area reset/cleared" },
    { 0x10, 0x03, 0xff, "All event logging disabled" },
    { 0x10, 0x04, 0xff, "Log full" },
    { 0x10, 0x05, 0xff, "Log almost full" },
    /* Watchdog 1 */
    { 0x11, 0x00, 0xff, "BIOS Reset" },
    { 0x11, 0x01, 0xff, "OS Reset" },
    { 0x11, 0x02, 0xff, "OS Shut Down" },
    { 0x11, 0x03, 0xff, "OS Power Down" },
    { 0x11, 0x04, 0xff, "OS Power Cycle" },
    { 0x11, 0x05, 0xff, "OS NMI/Diag Interrupt" },
    { 0x11, 0x06, 0xff, "OS Expired" },
    { 0x11, 0x07, 0xff, "OS pre-timeout Interrupt" },
    /* System Event */
    { 0x12, 0x00, 0xff, "System Reconfigured" },
    { 0x12, 0x01, 0xff, "OEM System boot event" },
    { 0x12, 0x02, 0xff, "Undetermined system hardware failure" },
    { 0x12, 0x03, 0xff, "Entry added to auxiliary log" },
    { 0x12, 0x04, 0xff, "PEF Action" },
    { 0x12, 0x05, 0xff, "Timestamp Clock Sync" },
    /* Critical Interrupt */
    { 0x13, 0x00, 0xff, "NMI/Diag Interrupt" },
    { 0x13, 0x01, 0xff, "Bus Timeout" },
    { 0x13, 0x02, 0xff, "I/O Channel check NMI" },
    { 0x13, 0x03, 0xff, "Software NMI" },
    { 0x13, 0x04, 0xff, "PCI PERR" },
    { 0x13, 0x05, 0xff, "PCI SERR" },
    { 0x13, 0x06, 0xff, "EISA failsafe timeout" },
    { 0x13, 0x07, 0xff, "Bus Correctable error" },
    { 0x13, 0x08, 0xff, "Bus Uncorrectable error" },
    { 0x13, 0x09, 0xff, "Fatal NMI" },
    { 0x13, 0x0a, 0xff, "Bus Fatal Error" },
    { 0x13, 0x0b, 0xff, "Bus Degraded" },
    /* Button */
    { 0x14, 0x00, 0xff, "Power Button pressed" },
    { 0x14, 0x01, 0xff, "Sleep Button pressed" },
    { 0x14, 0x02, 0xff, "Reset Button pressed" },
    { 0x14, 0x03, 0xff, "FRU Latch" },
    { 0x14, 0x04, 0xff, "FRU Service" },
    /* Chip Set */
    { 0x19, 0x00, 0xff, "Soft Power Control Failure" },
    { 0x19, 0x01, 0xff, "Thermal Trip" },
    /* Cable/Interconnect */
    { 0x1b, 0x00, 0xff, "Connected" },
    { 0x1b, 0x01, 0xff, "Config Error" },
    /* System Boot Initiated */
    { 0x1d, 0x00, 0xff, "Initiated by power up" },
    { 0x1d, 0x01, 0xff, "Initiated by hard reset" },
    { 0x1d, 0x02, 0xff, "Initiated by warm reset" },
    { 0x1d, 0x03, 0xff, "User requested PXE boot" },
    { 0x1d, 0x04, 0xff, "Automatic boot to diagnostic" },
    { 0x1d, 0x05, 0xff, "OS initiated hard reset" },
    { 0x1d, 0x06, 0xff, "OS initiated warm reset" },
    { 0x1d, 0x07, 0xff, "System Restart" },
    /* Boot Error */
    { 0x1e, 0x00, 0xff, "No bootable media" },
    { 0x1e, 0x01, 0xff, "Non-bootable disk in drive" },
    { 0x1e, 0x02, 0xff, "PXE server not found" },
    { 0x1e, 0x03, 0xff, "Invalid boot sector" },
    { 0x1e, 0x04, 0xff, "Timeout waiting for selection" },
    /* OS Boot */
    { 0x1f, 0x00, 0xff, "A: boot completed" },
    { 0x1f, 0x01, 0xff, "C: boot completed" },
    { 0x1f, 0x02, 0xff, "PXE boot completed" },
    { 0x1f, 0x03, 0xff, "Diagnostic boot completed" },
    { 0x1f, 0x04, 0xff, "CD-ROM boot completed" },
    { 0x1f, 0x05, 0xff, "ROM boot completed" },
    { 0x1f, 0x06, 0xff, "boot completed - device not specified" },
    { 0x1f, 0x07, 0xff, "Installation started" },
    { 0x1f, 0x08, 0xff, "Installation completed" },
    { 0x1f, 0x09, 0xff, "Installation aborted" },
    { 0x1f, 0x0a, 0xff, "Installation failed" },
    /* OS Stop/Shutdown */
    { 0x20, 0x00, 0xff, "Error during system startup" },
    { 0x20, 0x01, 0xff, "Run-time critical stop" },
    { 0x20, 0x02, 0xff, "OS graceful stop" },
    { 0x20, 0x03, 0xff, "OS graceful shutdown" },
    { 0x20, 0x04, 0xff, "PEF initiated soft shutdown" },
    { 0x20, 0x05, 0xff, "Agent not responding" },
    /* Slot/Connector */
    { 0x21, 0x00, 0xff, "Fault Status" },
    { 0x21, 0x01, 0xff, "Identify Status" },
    { 0x21, 0x02, 0xff, "Device Installed" },
    { 0x21, 0x03, 0xff, "Ready for Device Installation" },
    { 0x21, 0x04, 0xff, "Ready for Device Removal" },
    { 0x21, 0x05, 0xff, "Slot Power is Off" },
    { 0x21, 0x06, 0xff, "Device Removal Request" },
    { 0x21, 0x07, 0xff, "Interlock" },
    { 0x21, 0x08, 0xff, "Slot is Disabled" },
    { 0x21, 0x09, 0xff, "Spare Device" },
    /* System ACPI Power State */
    { 0x22, 0x00, 0xff, "S0/G0: working" },
    { 0x22, 0x01, 0xff, "S1: sleeping with system hw & processor context maintained" },
    { 0x22, 0x02, 0xff, "S2: sleeping, processor context lost" },
    { 0x22, 0x03, 0xff, "S3: sleeping, processor & hw context lost, memory retained" },
    { 0x22, 0x04, 0xff, "S4: non-volatile sleep/suspend-to-disk" },
    { 0x22, 0x05, 0xff, "S5/G2: soft-off" },
    { 0x22, 0x06, 0xff, "S4/S5: soft-off" },
    { 0x22, 0x07, 0xff, "G3: mechanical off" },
    { 0x22, 0x08, 0xff, "Sleeping in S1/S2/S3 state" },
    { 0x22, 0x09, 0xff, "G1: sleeping" },
    { 0x22, 0x0a, 0xff, "S5: entered by override" },
    { 0x22, 0x0b, 0xff, "Legacy ON state" },
    { 0x22, 0x0c, 0xff, "Legacy OFF state" },
    { 0x22, 0x0e, 0xff, "Unknown" },
    /* Watchdog 2 */
    { 0x23, 0x00, 0xff, "Timer expired" },
    { 0x23, 0x01, 0xff, "Hard reset" },
    { 0x23, 0x02, 0xff, "Power down" },
    { 0x23, 0x03, 0xff, "Power cycle" },
    { 0x23, 0x04, 0xff, "reserved" },
    { 0x23, 0x05, 0xff, "reserved" },
    { 0x23, 0x06, 0xff, "reserved" },
    { 0x23, 0x07, 0xff, "reserved" },
    { 0x23, 0x08, 0xff, "Timer interrupt" },
    /* Platform Alert */
    { 0x24, 0x00, 0xff, "Platform generated page" },
    { 0x24, 0x01, 0xff, "Platform generated LAN alert" },
    { 0x24, 0x02, 0xff, "Platform Event Trap generated" },
    { 0x24, 0x03, 0xff, "Platform generated SNMP trap, OEM format" },
    /* Entity Presence */
    { 0x25, 0x00, 0xff, "Present" },
    { 0x25, 0x01, 0xff, "Absent" },
    { 0x25, 0x02, 0xff, "Disabled" },
    /* LAN */
    { 0x27, 0x00, 0xff, "Heartbeat Lost" },
    { 0x27, 0x01, 0xff, "Heartbeat" },
    /* Management Subsystem Health */
    { 0x28, 0x00, 0xff, "Sensor access degraded or unavailable" },
    { 0x28, 0x01, 0xff, "Controller access degraded or unavailable" },
    { 0x28, 0x02, 0xff, "Management controller off-line" },
    { 0x28, 0x03, 0xff, "Management controller unavailable" },
    { 0x28, 0x04, 0xff, "Sensor failure" },
    { 0x28, 0x05, 0xff, "FRU failure" },
    /* Battery */
    { 0x29, 0x00, 0xff, "Low" },
    { 0x29, 0x01, 0xff, "Failed" },
    { 0x29, 0x02, 0xff, "Presence Detected" },
    /* Version Change */
    { 0x2b, 0x00, 0xff, "Hardware change detected" },
    { 0x2b, 0x01, 0x00, "Firmware or software change detected" },
    { 0x2b, 0x01, 0x01, "Firmware or software change detected, Mngmt Ctrl Dev Id" },
    { 0x2b, 0x01, 0x02, "Firmware or software change detected, Mngmt Ctrl Firm Rev" },
    { 0x2b, 0x01, 0x03, "Firmware or software change detected, Mngmt Ctrl Dev Rev" },
    { 0x2b, 0x01, 0x04, "Firmware or software change detected, Mngmt Ctrl Manuf Id" },
    { 0x2b, 0x01, 0x05, "Firmware or software change detected, Mngmt Ctrl IPMI Vers" },
    { 0x2b, 0x01, 0x06, "Firmware or software change detected, Mngmt Ctrl Aux Firm Id" },
    { 0x2b, 0x01, 0x07, "Firmware or software change detected, Mngmt Ctrl Firm Boot Block" },
    { 0x2b, 0x01, 0x08, "Firmware or software change detected, Mngmt Ctrl Other" },
    { 0x2b, 0x01, 0x09, "Firmware or software change detected, BIOS/EFI change" },
    { 0x2b, 0x01, 0x0A, "Firmware or software change detected, SMBIOS change" },
    { 0x2b, 0x01, 0x0B, "Firmware or software change detected, O/S change" },
    { 0x2b, 0x01, 0x0C, "Firmware or software change detected, O/S loader change" },
    { 0x2b, 0x01, 0x0D, "Firmware or software change detected, Service Diag change" },
    { 0x2b, 0x01, 0x0E, "Firmware or software change detected, Mngmt SW agent change" },
    { 0x2b, 0x01, 0x0F, "Firmware or software change detected, Mngmt SW App change" },
    { 0x2b, 0x01, 0x10, "Firmware or software change detected, Mngmt SW Middle" },
    { 0x2b, 0x01, 0x11, "Firmware or software change detected, Prog HW Change (FPGA)" },
    { 0x2b, 0x01, 0x12, "Firmware or software change detected, board/FRU module change" },
    { 0x2b, 0x01, 0x13, "Firmware or software change detected, board/FRU component change" },
    { 0x2b, 0x01, 0x14, "Firmware or software change detected, board/FRU replace equ ver" },
    { 0x2b, 0x01, 0x15, "Firmware or software change detected, board/FRU replace new ver" },
    { 0x2b, 0x01, 0x16, "Firmware or software change detected, board/FRU replace old ver" },
    { 0x2b, 0x01, 0x17, "Firmware or software change detected, board/FRU HW conf change" },
    { 0x2b, 0x02, 0xff, "Hardware incompatibility detected" },
    { 0x2b, 0x03, 0xff, "Firmware or software incompatibility detected" },
    { 0x2b, 0x04, 0xff, "Invalid or unsupported hardware version" },
    { 0x2b, 0x05, 0xff, "Invalid or unsupported firmware or software version" },
    { 0x2b, 0x06, 0xff, "Hardware change success" },
    { 0x2b, 0x07, 0x00, "Firmware or software change success" },
    { 0x2b, 0x07, 0x01, "Firmware or software change success, Mngmt Ctrl Dev Id" },
    { 0x2b, 0x07, 0x02, "Firmware or software change success, Mngmt Ctrl Firm Rev" },
    { 0x2b, 0x07, 0x03, "Firmware or software change success, Mngmt Ctrl Dev Rev" },
    { 0x2b, 0x07, 0x04, "Firmware or software change success, Mngmt Ctrl Manuf Id" },
    { 0x2b, 0x07, 0x05, "Firmware or software change success, Mngmt Ctrl IPMI Vers" },
    { 0x2b, 0x07, 0x06, "Firmware or software change success, Mngmt Ctrl Aux Firm Id" },
    { 0x2b, 0x07, 0x07, "Firmware or software change success, Mngmt Ctrl Firm Boot Block" },
    { 0x2b, 0x07, 0x08, "Firmware or software change success, Mngmt Ctrl Other" },
    { 0x2b, 0x07, 0x09, "Firmware or software change success, BIOS/EFI change" },
    { 0x2b, 0x07, 0x0A, "Firmware or software change success, SMBIOS change" },
    { 0x2b, 0x07, 0x0B, "Firmware or software change success, O/S change" },
    { 0x2b, 0x07, 0x0C, "Firmware or software change success, O/S loader change" },
    { 0x2b, 0x07, 0x0D, "Firmware or software change success, Service Diag change" },
    { 0x2b, 0x07, 0x0E, "Firmware or software change success, Mngmt SW agent change" },
    { 0x2b, 0x07, 0x0F, "Firmware or software change success, Mngmt SW App change" },
    { 0x2b, 0x07, 0x10, "Firmware or software change success, Mngmt SW Middle" },
    { 0x2b, 0x07, 0x11, "Firmware or software change success, Prog HW Change (FPGA)" },
    { 0x2b, 0x07, 0x12, "Firmware or software change success, board/FRU module change" },
    { 0x2b, 0x07, 0x13, "Firmware or software change success, board/FRU component change" },
    { 0x2b, 0x07, 0x14, "Firmware or software change success, board/FRU replace equ ver" },
    { 0x2b, 0x07, 0x15, "Firmware or software change success, board/FRU replace new ver" },
    { 0x2b, 0x07, 0x16, "Firmware or software change success, board/FRU replace old ver" },
    { 0x2b, 0x07, 0x17, "Firmware or software change success, board/FRU HW conf change" },
    /* FRU State */
    { 0x2c, 0x00, 0xff, "Not Installed" },
    { 0x2c, 0x01, 0xff, "Inactive" },
    { 0x2c, 0x02, 0xff, "Activation Requested" },
    { 0x2c, 0x03, 0xff, "Activation in Progress" },
    { 0x2c, 0x04, 0xff, "Active" },
    { 0x2c, 0x05, 0xff, "Deactivation Requested" },
    { 0x2c, 0x06, 0xff, "Deactivation in Progress" },
    { 0x2c, 0x07, 0xff, "Communication lost" },
    /* PICMG FRU Hot Swap */
    { 0xF0, 0x00, 0xFF, "Transition to M0" },
    { 0xF0, 0x01, 0xFF, "Transition to M1" },
    { 0xF0, 0x02, 0xFF, "Transition to M2" },
    { 0xF0, 0x03, 0xFF, "Transition to M3" },
    { 0xF0, 0x04, 0xFF, "Transition to M4" },
    { 0xF0, 0x05, 0xFF, "Transition to M5" },
    { 0xF0, 0x06, 0xFF, "Transition to M6" },
    { 0xF0, 0x07, 0xFF, "Transition to M7" },
    /* PICMG IPMB Physical Link */
    { 0xF1, 0x00, 0xff, "IPMB-A disabled, IPMB-B disabled" },
    { 0xF1, 0x01, 0xff, "IPMB-A enabled, IPMB-B disabled" },
    { 0xF1, 0x02, 0xff, "IPMB-A disabled, IPMB-B enabled" },
    { 0xF1, 0x03, 0xff, "IPMB-A enabled, IPMB-B enabled" },
    /* PICMG Module Hot Swap */
    { 0xF2, 0x00, 0xff, "Module Handle Closed" },
    { 0xF2, 0x01, 0xff, "Module Handle Opened" },
    { 0xF2, 0x02, 0xff, "Quiesced" },
    { 0x00, 0x00, 0xff, NULL },
};

#define DATA_BYTE2_SPECIFIED_MASK	0xc0    /* event_data[0] bit mask */
#define ALL_OFFSETS_SPECIFIED  0xff
std::string getSelEventMessage(const std::string& sensorPath,  std::array<uint8_t, eventDataSize>& eventData)
{
    const struct ipmi_event_sensor_types *evt;
    int8_t sensorType = getSensorTypeFromPath(sensorPath);
    uint8_t eventType = getSensorEventTypeFromPath(sensorPath);
    size_t sizeOfEvents;
    if (eventType == 0x6f)
    {
        evt = sensor_specific_event_types; 
        sizeOfEvents= sizeof(sensor_specific_event_types) / sizeof(sensor_specific_event_types[0]);
    }
    else
    {
        evt = generic_event_types;
        sizeOfEvents= sizeof(generic_event_types) / sizeof(generic_event_types[0]);
    }
    
    uint8_t offset = eventData[0] & 0xf;
    for (uint32_t i = 0; i <  sizeOfEvents; ++i, ++evt)
    {
        if ((evt->code == sensorType) &&
            (evt->offset == offset && evt->desc) &&
            ((evt->data == ALL_OFFSETS_SPECIFIED) ||
             ((eventData[0] & DATA_BYTE2_SPECIFIED_MASK) &&
              (evt->data == eventData[1]))))
              {
                return std::string(evt->desc);
              }
    }
    return "";
}

std::map<std::string, std::vector<std::string>>
    getObjectInterfaces(const char* path)
{
    std::map<std::string, std::vector<std::string>> interfacesResponse;
    std::vector<std::string> interfaces;
    std::shared_ptr<sdbusplus::asio::connection> dbus = getSdBus();

    sdbusplus::message_t getObjectMessage =
        dbus->new_method_call("xyz.openbmc_project.ObjectMapper",
                              "/xyz/openbmc_project/object_mapper",
                              "xyz.openbmc_project.ObjectMapper", "GetObject");
    getObjectMessage.append(path, interfaces);

    try
    {
        sdbusplus::message_t response = dbus->call(getObjectMessage);
        response.read(interfacesResponse);
    }
    catch (const std::exception& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to GetObject", phosphor::logging::entry("PATH=%s", path),
            phosphor::logging::entry("WHAT=%s", e.what()));
    }

    return interfacesResponse;
}

std::map<std::string, Value> getEntityManagerProperties(const char* path,
                                                        const char* interface)
{
    std::map<std::string, Value> properties;
    std::shared_ptr<sdbusplus::asio::connection> dbus = getSdBus();

    sdbusplus::message_t getProperties =
        dbus->new_method_call("xyz.openbmc_project.EntityManager", path,
                              "org.freedesktop.DBus.Properties", "GetAll");
    getProperties.append(interface);

    try
    {
        sdbusplus::message_t response = dbus->call(getProperties);
        response.read(properties);
    }
    catch (const std::exception& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Failed to GetAll", phosphor::logging::entry("PATH=%s", path),
            phosphor::logging::entry("INTF=%s", interface),
            phosphor::logging::entry("WHAT=%s", e.what()));
    }

    return properties;
}

// Fetch the ipmiDecoratorPaths to get the list of dbus objects that
// have ipmi decorator to prevent unnessary dbus call to fetch the info
std::optional<std::unordered_set<std::string>>&
    getIpmiDecoratorPaths(const std::optional<ipmi::Context::ptr>& ctx)
{
    static std::optional<std::unordered_set<std::string>> ipmiDecoratorPaths;

    if (!ctx.has_value() || ipmiDecoratorPaths != std::nullopt)
    {
        return ipmiDecoratorPaths;
    }

    boost::system::error_code ec;
    std::vector<std::string> paths =
        (*ctx)->bus->yield_method_call<std::vector<std::string>>(
            (*ctx)->yield, ec, "xyz.openbmc_project.ObjectMapper",
            "/xyz/openbmc_project/object_mapper",
            "xyz.openbmc_project.ObjectMapper", "GetSubTreePaths", "/",
            int32_t(0),
            std::array<const char*, 1>{
                "xyz.openbmc_project.Inventory.Decorator.Ipmi"});
    if (ec)
    {
        return ipmiDecoratorPaths;
    }

    ipmiDecoratorPaths = std::unordered_set<std::string>(paths.begin(),
                                                         paths.end());
    return ipmiDecoratorPaths;
}

const std::string* getSensorConfigurationInterface(
    const std::map<std::string, std::vector<std::string>>&
        sensorInterfacesResponse)
{
    auto entityManagerService =
        sensorInterfacesResponse.find("xyz.openbmc_project.EntityManager");
    if (entityManagerService == sensorInterfacesResponse.end())
    {
        return nullptr;
    }

    // Find the fan configuration first (fans can have multiple configuration
    // interfaces).
    for (const auto& entry : entityManagerService->second)
    {
        if (entry == "xyz.openbmc_project.Configuration.AspeedFan" ||
            entry == "xyz.openbmc_project.Configuration.I2CFan" ||
            entry == "xyz.openbmc_project.Configuration.NuvotonFan")
        {
            return &entry;
        }
    }

    for (const auto& entry : entityManagerService->second)
    {
        if (boost::algorithm::starts_with(entry,
                                          "xyz.openbmc_project.Configuration."))
        {
            return &entry;
        }
    }

    return nullptr;
}

// Follow Association properties for Sensor back to the Board dbus object to
// check for an EntityId and EntityInstance property.
void updateIpmiFromAssociation(
    const std::string& path,
    const std::unordered_set<std::string>& ipmiDecoratorPaths,
    const DbusInterfaceMap& sensorMap, uint8_t& entityId,
    uint8_t& entityInstance)
{
    namespace fs = std::filesystem;

    auto sensorAssociationObject =
        sensorMap.find("xyz.openbmc_project.Association.Definitions");
    if (sensorAssociationObject == sensorMap.end())
    {
        if constexpr (debug)
        {
            std::fprintf(stderr, "path=%s, no association interface found\n",
                         path.c_str());
        }

        return;
    }

    auto associationObject =
        sensorAssociationObject->second.find("Associations");
    if (associationObject == sensorAssociationObject->second.end())
    {
        if constexpr (debug)
        {
            std::fprintf(stderr, "path=%s, no association records found\n",
                         path.c_str());
        }

        return;
    }

    std::vector<Association> associationValues =
        std::get<std::vector<Association>>(associationObject->second);

    // loop through the Associations looking for the right one:
    for (const auto& entry : associationValues)
    {
        // forward, reverse, endpoint
        const std::string& forward = std::get<0>(entry);
        const std::string& reverse = std::get<1>(entry);
        const std::string& endpoint = std::get<2>(entry);

        // We only currently concern ourselves with chassis+all_sensors.
        if (!(forward == "chassis" && reverse == "all_sensors"))
        {
            continue;
        }

        // the endpoint is the board entry provided by
        // Entity-Manager. so let's grab its properties if it has
        // the right interface.

        // just try grabbing the properties first.
        ipmi::PropertyMap::iterator entityIdProp;
        ipmi::PropertyMap::iterator entityInstanceProp;
        if (ipmiDecoratorPaths.contains(endpoint))
        {
            std::map<std::string, Value> ipmiProperties =
                getEntityManagerProperties(
                    endpoint.c_str(),
                    "xyz.openbmc_project.Inventory.Decorator.Ipmi");

            entityIdProp = ipmiProperties.find("EntityId");
            entityInstanceProp = ipmiProperties.find("EntityInstance");
            if (entityIdProp != ipmiProperties.end())
            {
                entityId = static_cast<uint8_t>(
                    std::get<uint64_t>(entityIdProp->second));
            }
            if (entityInstanceProp != ipmiProperties.end())
            {
                entityInstance = static_cast<uint8_t>(
                    std::get<uint64_t>(entityInstanceProp->second));
            }
        }

        // Now check the entity-manager entry for this sensor to see
        // if it has its own value and use that instead.
        //
        // In theory, checking this first saves us from checking
        // both, except in most use-cases identified, there won't be
        // a per sensor override, so we need to always check both.
        std::string sensorNameFromPath = fs::path(path).filename();

        std::string sensorConfigPath = endpoint + "/" + sensorNameFromPath;

        // Download the interfaces for the sensor from
        // Entity-Manager to find the name of the configuration
        // interface.
        std::map<std::string, std::vector<std::string>>
            sensorInterfacesResponse =
                getObjectInterfaces(sensorConfigPath.c_str());

        const std::string* configurationInterface =
            getSensorConfigurationInterface(sensorInterfacesResponse);

        // If there are multi association path settings and only one path exist,
        // we need to continue if cannot find configuration interface for this
        // sensor.
        if (!configurationInterface)
        {
            continue;
        }

        // We found a configuration interface.
        std::map<std::string, Value> configurationProperties =
            getEntityManagerProperties(sensorConfigPath.c_str(),
                                       configurationInterface->c_str());

        entityIdProp = configurationProperties.find("EntityId");
        entityInstanceProp = configurationProperties.find("EntityInstance");
        if (entityIdProp != configurationProperties.end())
        {
            entityId =
                static_cast<uint8_t>(std::get<uint64_t>(entityIdProp->second));
        }
        if (entityInstanceProp != configurationProperties.end())
        {
            entityInstance = static_cast<uint8_t>(
                std::get<uint64_t>(entityInstanceProp->second));
        }

        // stop searching Association records.
        break;
    } // end for Association vectors.

    if constexpr (debug)
    {
        std::fprintf(stderr, "path=%s, entityId=%d, entityInstance=%d\n",
                     path.c_str(), entityId, entityInstance);
    }
}

} // namespace ipmi
