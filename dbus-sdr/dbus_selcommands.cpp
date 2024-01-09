#include "commonselutility.hpp"
#include "dbus-sdr/sdrutils.hpp"
#include "dbus-sdr/storagecommands.hpp"
#include "xyz/openbmc_project/Logging/Entry/server.hpp"

#include <filesystem>
#include <ipmid/api.hpp>
#include <ipmid/utils.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <xyz/openbmc_project/Logging/SEL/error.hpp>
#include "config.h"
#include <fstream>

constexpr auto selEraseTimestamp = "/var/lib/ipmi/sel_erase_time";
static constexpr auto logObjPath = "/xyz/openbmc_project/logging";
static constexpr auto logInterface = "xyz.openbmc_project.Logging.Create";
static constexpr auto capacityInterface = "xyz.openbmc_project.Logging.Capacity";
constexpr auto dbusProperty = "org.freedesktop.DBus.Properties";
using ErrLvl = sdbusplus::xyz::openbmc_project::Logging::server::Entry::Level;
auto sevLvl = ErrLvl::Informational;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

namespace
{
constexpr auto SYSTEMD_TIME_SERVICE = "org.freedesktop.timedate1";
constexpr auto SYSTEMD_TIME_PATH = "/org/freedesktop/timedate1";
constexpr auto SYSTEMD_TIME_INTERFACE = "org.freedesktop.timedate1";

constexpr auto logWatchPath = "/xyz/openbmc_project/logging";
constexpr auto logBasePath = "/xyz/openbmc_project/logging/entry";
constexpr auto logEntryIntf = "xyz.openbmc_project.Logging.Entry";
constexpr auto logDeleteIntf = "xyz.openbmc_project.Object.Delete";
} // namespace

void registerStorageFunctions() __attribute__((constructor));

constexpr uint8_t firstEntryId = 1;

using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;
using namespace phosphor::logging;

using SELRecordID = uint16_t;
using SELEntry = ipmi::sel::SELEventRecordFormat;
using SELCacheMap = std::map<SELRecordID, SELEntry>;

SELCacheMap selCacheMap __attribute__((init_priority(101)));
bool selCacheMapInitialized;
std::unique_ptr<sdbusplus::bus::match::match> selAddedMatch
    __attribute__((init_priority(101)));
std::unique_ptr<sdbusplus::bus::match::match> selRemovedMatch
    __attribute__((init_priority(101)));
std::unique_ptr<sdbusplus::bus::match::match> selUpdatedMatch
    __attribute__((init_priority(101)));

inline uint16_t getLoggingId(const std::string& p)
{
    namespace fs = std::filesystem;
    fs::path entryPath(p);
    return std::stoul(entryPath.filename().string());
}

namespace ipmi
{

namespace sel
{

namespace internal
{

GetSELEntryResponse createSELEntry(const std::string& objPath)
{
    ipmi::sel::GetSELEntryResponse record{};

    uint16_t recordId;
    entryDataMap entryData;
    std::chrono::milliseconds chronoTimeStamp =
        getEntryData(objPath, entryData, recordId);

    record.event.eventRecord.recordID = recordId;
    additionalDataMap m;
    auto iterData = entryData.find(propAdditionalData);
    if (iterData == entryData.end())
    {
        log<level::ERR>("SEL AdditionalData  Not available");
        elog<InternalFailure>();
    }

    const auto& addData = std::get<AdditionalData>(iterData->second);
    m = parseAdditionalData(addData);
    auto recordType = static_cast<uint8_t>(convert(m[strRecordType]));
    if (recordType != systemEventRecord)
    {
        log<level::ERR>("Invalid recordType");
        elog<InternalFailure>();
    }
    // Default values when there is no matched sensor
    record.event.eventRecord.sensorType = 0;
    record.event.eventRecord.sensorNum = 0xFF;
    record.event.eventRecord.eventType = 0;
    std::string sensorPath("");
    auto iter = m.find(strSensorPath);
    if (iter != m.end())
    {
        sensorPath = iter->second;
    }
    else
    {
        log<level::ERR>("Event not from matched sensor, Hence logging it with "
                        "default values");
    }

    if (!sensorPath.empty())
    {
        try
        {
            record.event.eventRecord.sensorNum =
                getSensorNumberFromPath(sensorPath);
            record.event.eventRecord.eventType =
                getSensorEventTypeFromPath(sensorPath);
            record.event.eventRecord.sensorType =
                getSensorTypeFromPath(sensorPath);
        }
        catch (...)
        {
            log<level::ERR>("Failed to get dynamic sensor properties");
            elog<InternalFailure>();
        }
    }

    record.event.eventRecord.eventMsgRevision = eventMsgRevision;
    record.event.eventRecord.generatorID = 0;

    iter = m.find(strGenerateId);
    if (iter != m.end())
    {
        record.event.eventRecord.generatorID =
            static_cast<uint16_t>(convert(iter->second));
    }

    iter = m.find(strEventDir);
    if (iter != m.end())
    {
        auto eventDir = static_cast<uint8_t>(convert(iter->second));
        uint8_t assert = eventDir ? assertEvent : deassertEvent;
        record.event.eventRecord.eventType |= assert;
    }

    record.event.eventRecord.recordType = recordType;
    record.event.eventRecord.timeStamp = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::seconds>(chronoTimeStamp)
            .count());

    auto sensorData = std::vector<unsigned char>(0);
    iter = m.find(strSensorData);
    if (iter != m.end())
        sensorData = convertVec(iter->second);

    // The remaining 3 bytes are the sensor data
    memcpy(&record.event.eventRecord.eventData1, sensorData.data(),
           std::min(sensorData.size(), static_cast<size_t>(selDataSize)));

    return record;
}

} // namespace internal
} // namespace sel
} // namespace ipmi

std::optional<std::pair<uint16_t, SELEntry>>
    parseLoggingEntry(const std::string& p)
{
    try
    {
        auto id = getLoggingId(p);
        ipmi::sel::GetSELEntryResponse record{};
        record = ipmi::sel::internal::createSELEntry(p);
        return std::pair<uint16_t, SELEntry>({id, std::move(record.event)});
    }
    catch (const std::exception& e)
    {
        fprintf(stderr, "Failed to convert %s to SEL: %s\n", p.c_str(),
                e.what());
    }
    return std::nullopt;
}

std::string getLoggingObjPath(uint16_t id)
{
    return std::string(ipmi::sel::logBasePath) + "/" + std::to_string(id);
}

void selAddedCallback(sdbusplus::message::message& m)
{
    sdbusplus::message::object_path objPath;
    try
    {
        m.read(objPath);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("Failed to read object path");
        return;
    }
    std::string p = objPath;
    auto entry = parseLoggingEntry(p);
    if (entry)
    {
        selCacheMap.insert(std::move(*entry));
    }
}

void saveEraseTimeStamp()
{
   std::filesystem::path path(selEraseTimestamp);
   std::ofstream eraseTimeFile(path);
   if (!eraseTimeFile.good())
   {
       std::cerr << "Failed to open sel_erase_time file";
   }

   eraseTimeFile.close();
}

void selRemovedCallback(sdbusplus::message::message& m)
{
    sdbusplus::message::object_path objPath;
    try
    {
        m.read(objPath);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("Failed to read object path");
    }
    try
    {
        std::string p = objPath;
        selCacheMap.erase(getLoggingId(p));
	saveEraseTimeStamp();
    }
    catch (const std::invalid_argument& e)
    {
        log<level::ERR>("Invalid logging entry ID");
    }
}

void selUpdatedCallback(sdbusplus::message::message& m)
{
    std::string p = m.get_path();
    auto entry = parseLoggingEntry(p);
    if (entry)
    {
        selCacheMap.insert_or_assign(entry->first, std::move(entry->second));
    }
}

void registerSelCallbackHandler()
{
    using namespace sdbusplus::bus::match::rules;
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
    if (!selAddedMatch)
    {
        selAddedMatch = std::make_unique<sdbusplus::bus::match::match>(
            bus, interfacesAdded(logWatchPath),
            std::bind(selAddedCallback, std::placeholders::_1));
    }
    if (!selRemovedMatch)
    {
        selRemovedMatch = std::make_unique<sdbusplus::bus::match::match>(
            bus, interfacesRemoved(logWatchPath),
            std::bind(selRemovedCallback, std::placeholders::_1));
    }
    if (!selUpdatedMatch)
    {
        selUpdatedMatch = std::make_unique<sdbusplus::bus::match::match>(
            bus,
            type::signal() + member("PropertiesChanged"s) +
                interface("org.freedesktop.DBus.Properties"s) +
                argN(0, logEntryIntf),
            std::bind(selUpdatedCallback, std::placeholders::_1));
    }
}

void initSELCache()
{
    registerSelCallbackHandler();
    ipmi::sel::ObjectPaths paths;
    try
    {
        ipmi::sel::readLoggingObjectPaths(paths);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("Failed to get logging object paths");
        return;
    }
    for (const auto& p : paths)
    {
        auto entry = parseLoggingEntry(p);
        if (entry)
        {
            selCacheMap.insert(std::move(*entry));
        }
    }
    selCacheMapInitialized = true;
}


/** @brief implements the delete SEL entry command
 * @request
 *   - reservationID; // reservation ID.
 *   - selRecordID;   // SEL record ID.
 *
 *  @returns ipmi completion code plus response data
 *   - Record ID of the deleted record
 */
ipmi::RspType<uint16_t // deleted record ID
              >
    deleteSELEntry(uint16_t reservationID, uint16_t selRecordID)
{

    namespace fs = std::filesystem;

    if (!checkSELReservation(reservationID))
    {
        return ipmi::responseInvalidReservationId();
    }

    // Per the IPMI spec, need to cancel the reservation when a SEL entry is
    // deleted
    cancelSELReservation();

    if (!selCacheMapInitialized)
    {
        // In case the initSELCache() fails, try it again
        initSELCache();
    }
   if (selCacheMap.empty())
    {
        return ipmi::responseSensorInvalid();
    }

    SELCacheMap::const_iterator iter;
    uint16_t delRecordID = 0;

    if (selRecordID == ipmi::sel::firstEntry)
    {
        delRecordID = selCacheMap.begin()->first;
    }
    else if (selRecordID == ipmi::sel::lastEntry)
    {
        delRecordID = selCacheMap.rbegin()->first;
    }
    else
    {
        iter = selCacheMap.find(selRecordID);
        if (iter == selCacheMap.end())
        {
            return ipmi::responseSensorInvalid();
        }
        delRecordID = selRecordID;
    }

    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
    std::string service;

    auto objPath = getLoggingObjPath(iter->first);
    try
    {
        service = ipmi::getService(bus, ipmi::sel::logDeleteIntf, objPath);
    }
    catch (const std::runtime_error& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }

    auto methodCall = bus.new_method_call(service.c_str(), objPath.c_str(),
                                          ipmi::sel::logDeleteIntf, "Delete");
    auto reply = bus.call(methodCall);
    if (reply.is_method_error())
    {
        return ipmi::responseUnspecifiedError();
    }

    return ipmi::responseSuccess(delRecordID);
}

/** @brief implements the Clear SEL command
 * @request
 *   - reservationID   // Reservation ID.
 *   - clr             // char array { 'C'(0x43h), 'L'(0x4Ch), 'R'(0x52h) }
 *   - eraseOperation; // requested operation.
 *
 *  @returns ipmi completion code plus response data
 *   - erase status
 */

ipmi::RspType<uint8_t // erase status
              >
    clearSEL(uint16_t reservationID, const std::array<char, 3>& clr,
             uint8_t eraseOperation)
{
    static constexpr std::array<char, 3> clrOk = {'C', 'L', 'R'};
    if (clr != clrOk)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    if (!checkSELReservation(reservationID))
    {
        return ipmi::responseInvalidReservationId();
    }

    /*
     * Erasure status cannot be fetched from DBUS, so always return erasure
     * status as `erase completed`.
     */
    if (eraseOperation == ipmi::sel::getEraseStatus)
    {
        return ipmi::responseSuccess(
            static_cast<uint8_t>(ipmi::sel::eraseComplete));
    }
 // Per the IPMI spec, need to cancel any reservation when the SEL is cleared
    cancelSELReservation();

    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
    auto service = ipmi::getService(bus, ipmi::sel::logIntf, ipmi::sel::logObj);
    auto method =
        bus.new_method_call(service.c_str(), ipmi::sel::logObj,
                            ipmi::sel::logIntf, ipmi::sel::logDeleteAllMethod);
    try
    {
        bus.call_noreply(method);
    }
    catch (const sdbusplus::exception::exception& e)
    {
        log<level::ERR>("Error eraseAll ", entry("ERROR=%s", e.what()));
        return ipmi::responseUnspecifiedError();
    }

    return ipmi::responseSuccess(
        static_cast<uint8_t>(ipmi::sel::eraseComplete));
}

template <typename TP>
std::time_t to_time_t(TP tp)
{
    using namespace std::chrono;
    auto sctp = time_point_cast<system_clock::duration>(tp - TP::clock::now()
              + system_clock::now());
    return system_clock::to_time_t(sctp);
}

static int getFileTimestamp(const std::filesystem::path& file)
{
      std::error_code ec;
      std::filesystem::file_time_type ftime = std::filesystem::last_write_time(file,ec);
      if(ec)
      {
         return ::ipmi::sel::invalidTimeStamp;
      }

      return to_time_t(ftime);
}

/** @brief implements the get SEL Info command
 *  @returns IPMI completion code plus response data
 *   - selVersion - SEL revision
 *   - entries    - Number of log entries in SEL.
 *   - freeSpace  - Free Space in bytes.
 *   - addTimeStamp - Most recent addition timestamp
 *   - eraseTimeStamp - Most recent erase timestamp
 *   - operationSupport - Reserve & Delete SEL operations supported
 */
ipmi::RspType<uint8_t,  // SEL revision.
              uint16_t, // number of log entries in SEL.
              uint16_t, // free Space in bytes.
              uint32_t, // most recent addition timestamp
              uint32_t, // most recent erase timestamp.
              bool,     // SEL allocation info supported
              bool,     // reserve SEL supported
              bool,     // partial Add SEL Entry supported
              bool,     // delete SEL supported
              uint3_t,  // reserved
              bool      // overflow flag
              >
    ipmiStorageGetSelInfo()
{
    uint16_t entries = 0;
    // Most recent addition timestamp.
    uint32_t addTimeStamp = ipmi::sel::invalidTimeStamp;

    // Most recent delete timestamp
    uint32_t eraseTimeStamp = getFileTimestamp(selEraseTimestamp);

    if (!selCacheMapInitialized)
    {
        // In case the initSELCache() fails, try it again
        initSELCache();
    }
    if (!selCacheMap.empty())
    {
        entries = static_cast<uint16_t>(selCacheMap.size());

        try
        {
            auto objPath = getLoggingObjPath(selCacheMap.rbegin()->first);
            addTimeStamp = static_cast<uint32_t>(
                (ipmi::sel::getEntryTimeStamp(objPath).count()));
        }
        catch (const InternalFailure& e)
        {
        }
        catch (const std::runtime_error& e)
        {
            log<level::ERR>(e.what());
        }
    }

    constexpr uint8_t selVersion = ipmi::sel::selVersion;
    constexpr uint16_t maxDefineEntries = MAX_SEL_ENTRIES;
    uint16_t freeSpace = (maxDefineEntries - entries) * ipmi::sel::selRecordSize;
    constexpr uint3_t reserved{0};

    return ipmi::responseSuccess(
        selVersion, entries, freeSpace, addTimeStamp, eraseTimeStamp,
        ipmi::sel::operationSupport::getSelAllocationInfo,
        ipmi::sel::operationSupport::reserveSel,
        ipmi::sel::operationSupport::partialAddSelEntry,
        ipmi::sel::operationSupport::deleteSel, reserved,
         ipmi::sel::operationSupport::overflow);
}

using systemEventType = std::tuple<
    uint32_t, // Timestamp
    uint16_t, // Generator ID
    uint8_t,  // EvM Rev
    uint8_t,  // Sensor Type
    uint8_t,  // Sensor Number
    uint7_t,  // Event Type
    bool,     // Event Direction
    std::array<uint8_t, dynamic_sensors::ipmi::sel::systemEventSize>>; // Event
                                                                       // Data

ipmi::RspType<uint16_t, // Next Record ID
              uint16_t, // Record ID
              uint8_t,  // Record Type
              std::variant<systemEventType>>
ipmiGetSELEntry(uint16_t reservationID, uint16_t selRecordID,
                           uint8_t offset, uint8_t readLength)
{
    if (reservationID != 0 )
    {
        if (!checkSELReservation(reservationID))
        {
            return ipmi::responseInvalidReservationId();
        }
    }

    if (!selCacheMapInitialized)
    {
        initSELCache();
        selCacheMapInitialized = true;
    }

    if (selCacheMap.empty())
    {
        return ipmi::responseSensorInvalid();;
    }

    SELCacheMap::const_iterator iter;

   if ( selRecordID == ipmi::sel::firstEntry)
   {
           iter = selCacheMap.begin();
   }
   else if (selRecordID == ipmi::sel::lastEntry)
    {
        if (selCacheMap.size() > 1)
        {
            iter = selCacheMap.end();
            --iter;
        }
        else
        {
            // Only one entry exists, return the first
            iter = selCacheMap.begin();
        }
    }
    else
    {
        iter = selCacheMap.find(selRecordID);
        if (iter == selCacheMap.end())
        {
            return ipmi::responseSensorInvalid();
        }
    }

    ipmi::sel::GetSELEntryResponse record{0, iter->second};

    // Identify the next SEL record ID
    ++iter;

    if (iter == selCacheMap.end())
    {
        record.nextRecordID = ipmi::sel::lastEntry;
    }
    else
    {
        record.nextRecordID = iter->first;
    }

    bool eventDir = record.event.eventRecord.eventType >> 7;
    uint7_t eventType = record.event.eventRecord.eventType;
    std::array<uint8_t, 3> eventData{record.event.eventRecord.eventData1,
                                     record.event.eventRecord.eventData2,
                                     record.event.eventRecord.eventData3};

    return ipmi::responseSuccess(
        static_cast<uint16_t>(record.nextRecordID),
        static_cast<uint16_t>(record.event.eventRecord.recordID),
        static_cast<uint8_t>(record.event.eventRecord.recordType),
        systemEventType{
            static_cast<uint32_t>(record.event.eventRecord.timeStamp),
            static_cast<uint8_t>(record.event.eventRecord.generatorID),
            static_cast<uint8_t>(record.event.eventRecord.eventMsgRevision),
            static_cast<uint8_t>(record.event.eventRecord.sensorType),
            static_cast<uint8_t>(record.event.eventRecord.sensorNum), eventType,
            eventDir, eventData});
}

ipmi::RspType<uint16_t // recordID of the Added SEL entry
              >
    ipmiStorageAddSEL(uint16_t recordID, uint8_t recordType, uint32_t timeStamp,
                      uint16_t generatorID, uint8_t evmRev, uint8_t sensorType,
                      uint8_t sensorNumber, uint8_t eventDir,
                      std::array<uint8_t, eventDataSize> eventData)
{
    static constexpr auto systemRecordType = 0x02;
    cancelSELReservation();
    auto selDataStr = ipmi::sel::toHexStr(eventData);
    if (recordType == systemRecordType)
    {
        std::string objpath("");
        try
        {
            objpath = getPathFromSensorNumber(sensorNumber);
        }
        catch (...)
        {
            log<level::ERR>("Failed to get sensor object path");
        }

        bool assert = (eventDir & 0x80) ? false : true;
        std::string messageID = ipmi::getSelEventMessage(objpath, eventData);

        sdbusplus::bus::bus bus(ipmid_get_sd_bus_connection());
        std::map<std::string, std::string> addData;
        addData["namespace"] = "SEL";
        addData["SENSOR_DATA"] = selDataStr.c_str();
        addData["SEL_MESSAGE_ID"] = messageID.c_str();
        addData["SENSOR_PATH"] = objpath.c_str();
        addData["EVENT_DIR"] = std::to_string(assert);
        addData["GENERATOR_ID"] = std::to_string(generatorID);
        addData["RECORD_TYPE"] = std::to_string(recordType);

        try
        {
            auto service = ipmi::getService(bus, logInterface, logObjPath);
            auto method = bus.new_method_call(service.c_str(), logObjPath,
                                              logInterface, "Create");
            method.append(messageID, sevLvl, addData);
            bus.call_noreply(method);
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to create D-Bus log entry for SEL, ERROR="
                      << e.what() << "\n";
        }
    }
    else
        return ipmi::responseUnspecifiedError();

    if (selCacheMap.empty())
    {
        recordID = firstEntryId;
        return ipmi::responseSuccess(recordID);
    }

    auto beginIter = selCacheMap.rbegin();
    recordID = beginIter->first;

    return ipmi::responseSuccess(++recordID);
}

ipmi::RspType<uint32_t> ipmiStorageGetSELTime()
{
    struct timespec selTime = {};

    if (clock_gettime(CLOCK_REALTIME, &selTime) < 0)
    {
        return ipmi::responseUnspecifiedError();
    }

    return ipmi::responseSuccess(selTime.tv_sec);
}

ipmi::RspType<uint8_t> ipmiStorageSetErrorInfoCap(size_t capacity)
{
    cancelSELReservation();
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
    try
    {
        auto service = ipmi::getService(bus, capacityInterface, logObjPath);
        auto method = bus.new_method_call(service.c_str(), logObjPath,
                                          capacityInterface, "SetInfoLogCapacity");
        method.append(capacity);

        bus.call_noreply(method);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to set capacity for SEL error entry, ERROR="
                  << e.what() << "\n";
        return ipmi::responseUnspecifiedError();
    }

    return ipmi::responseSuccess(
        static_cast<uint8_t>(ipmi::sel::eraseComplete));
}

ipmi::RspType<size_t> ipmiStorageGetErrorInfoCap()
{
    std::variant<size_t> capacity;
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
    sdbusplus::message::message response;
    try
    {
        auto service = ipmi::getService(bus, capacityInterface, logObjPath);
        auto method = bus.new_method_call(service.c_str(), logObjPath,
                                          dbusProperty, "Get");
        method.append(capacityInterface, "InfoLogCapacity");
        response = bus.call(method);
        response.read(capacity);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to get capacity for SEL error entry, ERROR="
                  << e.what() << "\n";
        return ipmi::responseUnspecifiedError();
    }

    return ipmi::responseSuccess(
        static_cast<size_t>(std::get<size_t>(capacity)));
}

void registerStorageFunctions()
{

    selCacheMapInitialized = false;

    // <Add SEL Entry>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdAddSelEntry,
                          ipmi::Privilege::Operator, ipmiStorageAddSEL);

    // <Get SEL Entry>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetSelEntry, ipmi::Privilege::User,
                          ipmiGetSELEntry);

    // <Get SEL Info>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetSelInfo, ipmi::Privilege::User,
                          ipmiStorageGetSelInfo);

    // <Delete SEL Entry>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdDeleteSelEntry,
                          ipmi::Privilege::Operator, deleteSELEntry);

    // <Clear SEL>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdClearSel, ipmi::Privilege::Operator,
                          clearSEL);

   // <Get SEL Time>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetSelTime, ipmi::Privilege::User,
                          ipmiStorageGetSELTime);

    // <Set SEL Error Info Entry Capacity>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdSetErrorInfoCap,
                          ipmi::Privilege::Operator, ipmiStorageSetErrorInfoCap);

    // <Get SEL Error Info Entry Capacity>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetErrorInfoCap,
                          ipmi::Privilege::Operator, ipmiStorageGetErrorInfoCap);

    /*Note:
     * <Set SEL Time> and  <Reserve SEl>
     * These are the common handlers and the implementation of these are
     * available in storagehandler.cpp file
     */
}
