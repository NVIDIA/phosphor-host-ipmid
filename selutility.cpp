#include "config.h"

#include "selutility.hpp"

#include <ipmid/api.hpp>
#include <ipmid/types.hpp>
#include <ipmid/utils.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <charconv>
#include <chrono>
#include <filesystem>
#include <vector>

extern const ipmi::sensor::InvObjectIDMap invSensors;
using namespace phosphor::logging;
using InternalFailure =
    sdbusplus::error::xyz::openbmc_project::common::InternalFailure;

namespace ipmi
{

namespace sel
{

namespace internal
{

void constructSEL(uint8_t recordType, std::chrono::milliseconds timestamp,
                  const additionalDataMap& m,
                  [[maybe_unused]] const entryDataMap& entryData,
                  GetSELEntryResponse& record)
{
    if (recordType != systemEventRecord)
    {
        log<level::ERR>("Invalid recordType");
        elog<InternalFailure>();
    }

    // Default values when there is no matched sensor
    record.event.eventRecord.sensorType = 0;
    record.event.eventRecord.sensorNum = 0xFF;
    record.event.eventRecord.eventType = 0;

    auto iter = m.find(strSensorPath);
    assert(iter != m.end());
    const auto& sensorPath = iter->second;
    auto sensorIter = invSensors.find(sensorPath);

    if (sensorIter != invSensors.end())
    {
        // There is a matched sensor
        record.event.eventRecord.sensorType = sensorIter->second.sensorType;
        record.event.eventRecord.sensorNum = sensorIter->second.sensorID;

        iter = m.find(strEventDir);
        assert(iter != m.end());
        auto eventDir = static_cast<uint8_t>(convert(iter->second));
        uint8_t assert = eventDir ? assertEvent : deassertEvent;
        record.event.eventRecord.eventType =
            assert | sensorIter->second.eventReadingType;
    }
    record.event.eventRecord.recordType = recordType;
    record.event.eventRecord.timeStamp = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(timestamp).count());
    iter = m.find(strGenerateId);
    assert(iter != m.end());
    record.event.eventRecord.generatorID =
        static_cast<uint16_t>(convert(iter->second));
    record.event.eventRecord.eventMsgRevision = eventMsgRevision;
    iter = m.find(strSensorData);
    assert(iter != m.end());
    auto sensorData = convertVec(iter->second);
    // The remaining 3 bytes are the sensor data
    memcpy(&record.event.eventRecord.eventData1, sensorData.data(),
           std::min(sensorData.size(), static_cast<size_t>(selDataSize)));
}

GetSELEntryResponse
    prepareSELEntry(const std::string& objPath,
                    ipmi::sensor::InvObjectIDMap::const_iterator iter)
{
    GetSELEntryResponse record{};
    uint16_t recordId;
    entryDataMap entryData;
    std::chrono::milliseconds chronoTimeStamp = getEntryData(objPath, entryData,
                                                             recordId);

    bool isFromSELLogger = false;
    additionalDataMap m;

    // The recordID are with the same offset between different types,
    // so we are safe to set the recordID here
    record.event.eventRecord.recordID = recordId;

    auto iterId = entryData.find(propAdditionalData);
    if (iterId != entryData.end())
    {
        // Check if it's a SEL from phosphor-sel-logger which shall contain
        // the record ID, etc
        const auto& addData = std::get<AdditionalData>(iterId->second);
        m = parseAdditionalData(addData);
        auto recordTypeIter = m.find(strRecordType);
        if (recordTypeIter != m.end())
        {
            // It is a SEL from phosphor-sel-logger
            isFromSELLogger = true;
        }
        else
        {
            // Not a SEL from phosphor-sel-logger, it shall have a valid
            // invSensor
            if (iter == invSensors.end())
            {
                log<level::ERR>("System event sensor not found");
                elog<InternalFailure>();
            }
        }
    }

    if (isFromSELLogger)
    {
        // It is expected to be a custom SEL entry
        auto recordType = static_cast<uint8_t>(convert(m[strRecordType]));
        auto isOEM = isRecordOEM(recordType);
        if (isOEM)
        {
            constructOEMSEL(recordType, chronoTimeStamp, m, record);
        }
        else
        {
            constructSEL(recordType, chronoTimeStamp, m, entryData, record);
        }
    }
    else
    {
        record.event.eventRecord.timeStamp = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::seconds>(chronoTimeStamp)
                .count());

        record.event.eventRecord.recordType = systemEventRecord;
        record.event.eventRecord.generatorID = generatorID;
        record.event.eventRecord.eventMsgRevision = eventMsgRevision;

        record.event.eventRecord.sensorType = iter->second.sensorType;
        record.event.eventRecord.sensorNum = iter->second.sensorID;
        record.event.eventRecord.eventData1 = iter->second.eventOffset;

        // Read Resolved from the log entry.
        auto iterResolved = entryData.find(propResolved);
        if (iterResolved == entryData.end())
        {
            log<level::ERR>("Error in reading Resolved field of logging entry");
            elog<InternalFailure>();
        }

        // Evaluate if the event is assertion or deassertion event
        if (std::get<bool>(iterResolved->second))
        {
            record.event.eventRecord.eventType = deassertEvent |
                                                 iter->second.eventReadingType;
        }
        else
        {
            record.event.eventRecord.eventType = iter->second.eventReadingType;
        }
    }

    return record;
}

} // namespace internal

GetSELEntryResponse convertLogEntrytoSEL(const std::string& objPath)
{
    sdbusplus::bus_t bus{ipmid_get_sd_bus_connection()};

    static constexpr auto assocIntf =
        "xyz.openbmc_project.Association.Definitions";
    static constexpr auto assocProp = "Associations";

    auto service = ipmi::getService(bus, assocIntf, objPath);

    // Read the Associations interface.
    auto methodCall = bus.new_method_call(service.c_str(), objPath.c_str(),
                                          propIntf, "Get");
    methodCall.append(assocIntf);
    methodCall.append(assocProp);

    using AssociationList =
        std::vector<std::tuple<std::string, std::string, std::string>>;

    std::variant<AssociationList> list;
    try
    {
        auto reply = bus.call(methodCall);
        reply.read(list);
    }
    catch (const std::exception& e)
    {
        log<level::ERR>("Error in reading Associations interface",
                        entry("ERROR=%s", e.what()));
        elog<InternalFailure>();
    }

    auto& assocs = std::get<AssociationList>(list);

    /*
     * Check if the log entry has any callout associations, if there is a
     * callout association try to match the inventory path to the corresponding
     * IPMI sensor.
     */
    for (const auto& item : assocs)
    {
        if (std::get<0>(item).compare(CALLOUT_FWD_ASSOCIATION) == 0)
        {
            auto iter = invSensors.find(std::get<2>(item));
            if (iter == invSensors.end())
            {
                iter = invSensors.find(BOARD_SENSOR);
                if (iter == invSensors.end())
                {
                    log<level::ERR>("Motherboard sensor not found");
                    elog<InternalFailure>();
                }
            }

            return internal::prepareSELEntry(objPath, iter);
        }
    }

    // If there are no callout associations link the log entry to system event
    // sensor
    auto iter = invSensors.find(SYSTEM_SENSOR);
    return internal::prepareSELEntry(objPath, iter);
}

} // namespace sel

} // namespace ipmi
