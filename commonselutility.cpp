#include "commonselutility.hpp"

#include <ipmid/api.hpp>
#include <ipmid/types.hpp>
#include <ipmid/utils.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <charconv>
#include <chrono>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using namespace phosphor::logging;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

namespace ipmi
{

namespace sel
{

namespace internal
{

static void mergeLoggingEntryProperty(const std::string& name,
                                      const ipmi::Value& v, entryDataMap& out)
{
    if (name == "Id")
    {
        if (const uint32_t* u = std::get_if<uint32_t>(&v))
        {
            out[name] = *u;
        }
    }
    else if (name == "Timestamp")
    {
        if (const uint64_t* u = std::get_if<uint64_t>(&v))
        {
            out[name] = *u;
        }
    }
    else if (name == "AdditionalData")
    {
        if (const auto* m = std::get_if<std::map<std::string, std::string>>(&v))
        {
            out[name] = *m;
        }
    }
}

/** Parse the entry with format like key=val */
std::pair<std::string, std::string> parseEntry(const std::string& entry)
{
    constexpr auto equalSign = "=";
    auto pos = entry.find(equalSign);
    assert(pos != std::string::npos);
    auto key = entry.substr(0, pos);
    auto val = entry.substr(pos + 1);
    return {key, val};
}

// Parse SEL data and stored in additionalDataMap
additionalDataMap parseAdditionalData(const AdditionalData& data)
{
    return data;
}

// convert required SEL data in to integer
int convert(const std::string_view& str, int base)
{
    int ret;
    std::from_chars(str.data(), str.data() + str.size(), ret, base);
    return ret;
}

// Convert the string to a vector of uint8_t, where the str is formatted as hex
std::vector<uint8_t> convertVec(const std::string_view& str)
{
    std::vector<uint8_t> ret;
    auto len = str.size() / 2;
    ret.reserve(len);
    for (size_t i = 0; i < len; ++i)
    {
        ret.emplace_back(
            static_cast<uint8_t>(convert(str.substr(i * 2, 2), 16)));
    }
    return ret;
}

/** Construct OEM SEL record according to IPMI spec 32.2, 32.3. */
void constructOEMSEL(uint8_t recordType, std::chrono::milliseconds timestamp,
                     const additionalDataMap& m, GetSELEntryResponse& record)
{
    auto dataIter = m.find(strSensorData);
    assert(dataIter != m.end());
    auto sensorData = convertVec(dataIter->second);
    if (recordType >= 0xC0 && recordType < 0xE0)
    {
        record.event.oemCD.timeStamp = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::seconds>(timestamp)
                .count());
        record.event.oemCD.recordType = recordType;
        // The ManufactureID and OEM Defined are packed in the sensor data
        // Fill the 9 bytes of Manufacture ID and oemDefined
        memcpy(&record.event.oemCD.manufacturerID, sensorData.data(),
               std::min(sensorData.size(), static_cast<size_t>(oemCDDataSize)));
    }
    else if (recordType >= 0xE0)
    {
        record.event.oemEF.recordType = recordType;
        // The remaining 13 bytes are the OEM Defined data
        memcpy(&record.event.oemEF.oemDefined, sensorData.data(),
               std::min(sensorData.size(), static_cast<size_t>(oemEFDataSize)));
    }
}

// The SEL record ID is based on the phosphor-logging object path name,
// The path name is a number of uint32, need to convert it to uint16
// according to the IPMI spec
uint16_t convertSelIdToU16(uint32_t id)
{
    if (id >= std::numeric_limits<uint16_t>::max())
    {
        return static_cast<uint16_t>(
            id % std::numeric_limits<uint16_t>::max() + 1);
    }

    return static_cast<uint16_t>(id);
}

std::chrono::milliseconds getEntryData(const entryDataMap& entryData,
                                       uint16_t& recordId)
{
    static constexpr auto propId = "Id";
    auto iterId = entryData.find(propId);
    if (iterId == entryData.end())
    {
        log<level::ERR>("Error in reading Id of logging entry");
        elog<InternalFailure>();
    }
    recordId = static_cast<uint16_t>(std::get<uint32_t>(iterId->second));

    static constexpr auto propTimeStamp = "Timestamp";
    auto iterTimeStamp = entryData.find(propTimeStamp);
    if (iterTimeStamp == entryData.end())
    {
        log<level::ERR>("Error in reading Timestamp of logging entry");
        elog<InternalFailure>();
    }
    return std::chrono::milliseconds(std::get<uint64_t>(iterTimeStamp->second));
}

/* Retrive entry data from dbus object such as entry ID,
 * Timestamp and recordID.
 */
std::chrono::milliseconds getEntryData(
    const std::string& objPath, entryDataMap& entryData, uint16_t& recordId)
{
    sdbusplus::bus_t bus{ipmid_get_sd_bus_connection()};
    auto service = ipmi::getService(bus, logEntryIntf, objPath);

    // Read all the log entry properties.
    auto methodCall = bus.new_method_call(
        service.c_str(), objPath.c_str(), propIntf, "GetAll");
    methodCall.append(logEntryIntf);

    auto reply = bus.call(methodCall);
    if (reply.is_method_error())
    {
        log<level::ERR>("Error in reading logging property entries");
        elog<InternalFailure>();
    }

    reply.read(entryData);
    return getEntryData(entryData, recordId);
}

bool readLoggingEntryDataBulk(std::map<std::string, entryDataMap>& outByPath)
{
    outByPath.clear();

    try
    {
        sdbusplus::bus_t bus{ipmid_get_sd_bus_connection()};
        auto service = ipmi::getService(bus, objMgrIntf, std::string(logObj));
        auto tree = ipmi::getManagedObjects(bus, service, std::string(logObj));

        const std::string entryPrefix = std::string(logBasePath) + "/";
        for (const auto& [objPath, ifaces] : tree)
        {
            const std::string p = static_cast<std::string>(objPath);
            // Filter out the non-Logging.Entry objects.
            if (p.rfind(entryPrefix, 0) != 0)
            {
                continue;
            }
            auto it = ifaces.find(logEntryIntf);
            // Skip objects that don't implement the Logging.Entry interface.
            if (it == ifaces.end())
            {
                continue;
            }
            entryDataMap tmpMap;
            // Merge the properties of the Logging.Entry interface into
            // entryDataMap.
            for (const auto& [propName, val] : it->second)
            {
                mergeLoggingEntryProperty(propName, val, tmpMap);
            }
            // Only include entries that have Id, Timestamp, and AdditionalData.
            if (tmpMap.size() >= 3)
            {
                outByPath.emplace(p, std::move(tmpMap));
            }
        }
        return true;
    }
    catch (const std::exception& e)
    {
        log<level::INFO>("readLoggingEntryDataBulk failed",
                         entry("ERROR=%s", e.what()));
        return false;
    }
}

} // namespace internal

std::chrono::seconds getEntryTimeStamp(const std::string& objPath)
{
    sdbusplus::bus_t bus{ipmid_get_sd_bus_connection()};

    auto service = ipmi::getService(bus, logEntryIntf, objPath);

    using namespace std::string_literals;
    static const auto propTimeStamp = "Timestamp"s;

    auto methodCall =
        bus.new_method_call(service.c_str(), objPath.c_str(), propIntf, "Get");
    methodCall.append(logEntryIntf);
    methodCall.append(propTimeStamp);

    auto reply = bus.call(methodCall);
    if (reply.is_method_error())
    {
        log<level::ERR>("Error in reading Timestamp from Entry interface");
        elog<InternalFailure>();
    }

    std::variant<uint64_t> timeStamp;
    reply.read(timeStamp);

    std::chrono::milliseconds chronoTimeStamp(std::get<uint64_t>(timeStamp));

    return std::chrono::duration_cast<std::chrono::seconds>(chronoTimeStamp);
}

void readLoggingObjectPaths(ObjectPaths& paths)
{
    sdbusplus::bus_t bus{ipmid_get_sd_bus_connection()};
    auto depth = 0;
    paths.clear();

    auto mapperCall = bus.new_method_call(
        mapperBusName, mapperObjPath, mapperIntf, "GetSubTreePaths");
    mapperCall.append(logBasePath);
    mapperCall.append(depth);
    mapperCall.append(ObjectPaths({logEntryIntf}));

    try
    {
        auto reply = bus.call(mapperCall);
        reply.read(paths);
    }
    catch (const sdbusplus::exception_t& e)
    {
        if (strcmp(e.name(),
                   "xyz.openbmc_project.Common.Error.ResourceNotFound"))
        {
            throw;
        }
    }

    std::sort(paths.begin(), paths.end(),
              [](const std::string& a, const std::string& b) {
        namespace fs = std::filesystem;
        fs::path pathA(a);
        fs::path pathB(b);
        auto idA = std::stoul(pathA.filename().string());
        auto idB = std::stoul(pathB.filename().string());

        return idA < idB;
    });
}
} // namespace sel
} // namespace ipmi
