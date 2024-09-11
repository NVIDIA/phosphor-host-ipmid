#include "commonselutility.hpp"

#include <ipmid/api.hpp>
#include <ipmid/types.hpp>
#include <ipmid/utils.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Common/error.hpp>

#include <charconv>
#include <chrono>
#include <filesystem>
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
    std::map<std::string, std::string> ret;

    for (const auto& d : data)
    {
        ret.insert(parseEntry(d));
    }
    return ret;
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

/* Retrive entry data from dbus object such as entry ID,
 * Timestamp and recordID.
 */
std::chrono::milliseconds getEntryData(const std::string& objPath,
                                       entryDataMap& entryData,
                                       uint16_t& recordId)
{
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
    auto service = ipmi::getService(bus, logEntryIntf, objPath);

    // Read all the log entry properties.
    auto methodCall = bus.new_method_call(service.c_str(), objPath.c_str(),
                                          propIntf, "GetAll");
    methodCall.append(logEntryIntf);

    auto reply = bus.call(methodCall);
    if (reply.is_method_error())
    {
        log<level::ERR>("Error in reading logging property entries");
        elog<InternalFailure>();
    }

    //    entryDataMap entryData;
    reply.read(entryData);
    // Read Id from the log entry.
    static constexpr auto propId = "Id";
    auto iterId = entryData.find(propId);
    if (iterId == entryData.end())
    {
        log<level::ERR>("Error in reading Id of logging entry");
        elog<InternalFailure>();
    }

    recordId = static_cast<uint16_t>(std::get<uint32_t>(iterId->second));

    // Read Timestamp from the log entry.
    static constexpr auto propTimeStamp = "Timestamp";
    auto iterTimeStamp = entryData.find(propTimeStamp);
    if (iterTimeStamp == entryData.end())
    {
        log<level::ERR>("Error in reading Timestamp of logging entry");
        elog<InternalFailure>();
    }
    std::chrono::milliseconds chronoTimeStamp(
        std::get<uint64_t>(iterTimeStamp->second));
    return chronoTimeStamp;
}

} // namespace internal

std::chrono::seconds getEntryTimeStamp(const std::string& objPath)
{
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};

    auto service = ipmi::getService(bus, logEntryIntf, objPath);

    using namespace std::string_literals;
    static const auto propTimeStamp = "Timestamp"s;

    auto methodCall = bus.new_method_call(service.c_str(), objPath.c_str(),
                                          propIntf, "Get");
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

    auto mapperCall = bus.new_method_call(mapperBusName, mapperObjPath,
                                          mapperIntf, "GetSubTreePaths");
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
