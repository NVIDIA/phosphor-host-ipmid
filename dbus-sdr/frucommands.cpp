#include "dbus-sdr/sdrutils.hpp"
#include "dbus-sdr/storagecommands.hpp"

#include <boost/algorithm/string.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/process.hpp>
#include <ipmid/api.hpp>
#include <ipmid/message.hpp>
#include <ipmid/types.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/message/types.hpp>
#include <sdbusplus/timer.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string_view>

static constexpr bool DEBUG = false;

namespace ipmi
{

namespace storage
{

constexpr static const size_t maxMessageSize = 64;
constexpr static const size_t maxFruSdrNameSize = 16;
using ObjectType =
    boost::container::flat_map<std::string,
                               boost::container::flat_map<std::string, Value>>;
using ManagedObjectType =
    boost::container::flat_map<sdbusplus::message::object_path, ObjectType>;
using ManagedEntry = std::pair<sdbusplus::message::object_path, ObjectType>;

constexpr static const char* fruDeviceServiceName =
    "xyz.openbmc_project.FruDevice";
constexpr static const char* entityManagerServiceName =
    "xyz.openbmc_project.EntityManager";
constexpr static const size_t writeTimeoutSeconds = 10;
constexpr static const char* chassisTypeRackMount = "23";
constexpr static const char* chassisTypeMainServer = "17";

// event direction is bit[7] of eventType where 1b = Deassertion event
constexpr static const uint8_t deassertionEvent = 0x80;

static std::vector<uint8_t> fruCache;
static constexpr uint16_t invalidBus = 0xFFFF;
static constexpr uint8_t invalidAddr = 0xFF;
static constexpr uint8_t typeASCIILatin8 = 0xC0;
static uint16_t cacheBus = invalidBus;
static uint8_t cacheAddr = invalidAddr;
static uint8_t lastDevId = 0xFF;

static uint16_t writeBus = invalidBus;
static uint8_t writeAddr = invalidAddr;

std::unique_ptr<sdbusplus::Timer> writeTimer = nullptr;
static std::vector<sdbusplus::bus::match_t> fruMatches;

ManagedObjectType frus;

// we unfortunately have to build a map of hashes in case there is a
// collision to verify our dev-id
boost::container::flat_map<uint8_t, std::pair<uint16_t, uint8_t>> deviceHashes;
void registerStorageFRUFunctions() __attribute__((constructor));

bool writeFru(const std::vector<uint8_t>& fru)
{
    if (writeBus == invalidBus && writeAddr == invalidAddr)
    {
        return true;
    }
    lastDevId = 0xFF;
    std::shared_ptr<sdbusplus::asio::connection> dbus = getSdBus();
    sdbusplus::message_t writeFru = dbus->new_method_call(
        fruDeviceServiceName, "/xyz/openbmc_project/FruDevice",
        "xyz.openbmc_project.FruDeviceManager", "WriteFru");
    writeFru.append(writeBus, writeAddr, fru);
    try
    {
        sdbusplus::message_t writeFruResp = dbus->call(writeFru);
    }
    catch (const sdbusplus::exception_t&)
    {
        // todo: log sel?
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "error writing fru");
        return false;
    }
    writeBus = invalidBus;
    writeAddr = invalidAddr;
    return true;
}

void writeFruCache()
{
    writeFru(fruCache);
}

void createTimers()
{
    writeTimer = std::make_unique<sdbusplus::Timer>(writeFruCache);
}

void recalculateHashes()
{
    deviceHashes.clear();
    // hash the object paths to create unique device id's. increment on
    // collision
    std::hash<std::string> hasher;
    for (const auto& fru : frus)
    {
        auto fruIface = fru.second.find("xyz.openbmc_project.FruDevice");
        if (fruIface == fru.second.end())
        {
            continue;
        }

        auto busFind = fruIface->second.find("BUS");
        auto addrFind = fruIface->second.find("ADDRESS");
        if (busFind == fruIface->second.end() ||
            addrFind == fruIface->second.end())
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "fru device missing Bus or Address",
                phosphor::logging::entry("FRU=%s", fru.first.str.c_str()));
            continue;
        }

        uint16_t fruBus = std::get<uint32_t>(busFind->second);
        uint8_t fruAddr = std::get<uint32_t>(addrFind->second);
        auto chassisFind = fruIface->second.find("CHASSIS_TYPE");
        std::string chassisType;
        if (chassisFind != fruIface->second.end())
        {
            chassisType = std::get<std::string>(chassisFind->second);
        }

        uint8_t fruHash = 0;
        if (chassisType.compare(chassisTypeRackMount) != 0 &&
            chassisType.compare(chassisTypeMainServer) != 0)
        {
            fruHash = hasher(fru.first.str);
            // can't be 0xFF based on spec, and 0 is reserved for baseboard
            if (fruHash == 0 || fruHash == 0xFF)
            {
                fruHash = 1;
            }
        }
        std::pair<uint16_t, uint8_t> newDev(fruBus, fruAddr);

        bool emplacePassed = false;
        while (!emplacePassed)
        {
            auto resp = deviceHashes.emplace(fruHash, newDev);
            emplacePassed = resp.second;
            if (!emplacePassed)
            {
                fruHash++;
                // can't be 0xFF based on spec, and 0 is reserved for
                // baseboard
                if (fruHash == 0XFF)
                {
                    fruHash = 0x1;
                }
            }
        }
    }
}

void replaceCacheFru(
    const std::shared_ptr<sdbusplus::asio::connection>& bus,
    boost::asio::yield_context& yield,
    [[maybe_unused]] const std::optional<std::string>& path = std::nullopt)
{
    boost::system::error_code ec;

    frus = bus->yield_method_call<ManagedObjectType>(
        yield, ec, fruDeviceServiceName, "/",
        "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
    if (ec)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "GetMangagedObjects for replaceCacheFru failed",
            phosphor::logging::entry("ERROR=%s", ec.message().c_str()));

        return;
    }
    recalculateHashes();
}

std::pair<ipmi::Cc, std::vector<uint8_t>> getFru(ipmi::Context::ptr ctx,
                                                 uint8_t devId)
{
    if (lastDevId == devId && devId != 0xFF)
    {
        return {ipmi::ccSuccess, fruCache};
    }

    auto deviceFind = deviceHashes.find(devId);
    if (deviceFind == deviceHashes.end())
    {
        return {IPMI_CC_SENSOR_INVALID, {}};
    }

    cacheBus = deviceFind->second.first;
    cacheAddr = deviceFind->second.second;

    boost::system::error_code ec;

    std::vector<uint8_t> fru =
        ctx->bus->yield_method_call<std::vector<uint8_t>>(
            ctx->yield, ec, fruDeviceServiceName,
            "/xyz/openbmc_project/FruDevice",
            "xyz.openbmc_project.FruDeviceManager", "GetRawFru", cacheBus,
            cacheAddr);
    if (ec)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Couldn't get raw fru",
            phosphor::logging::entry("ERROR=%s", ec.message().c_str()));

        cacheBus = invalidBus;
        cacheAddr = invalidAddr;
        return {ipmi::ccResponseError, {}};
    }

    fruCache.clear();
    lastDevId = devId;
    fruCache = fru;

    return {ipmi::ccSuccess, fru};
}

void writeFruIfRunning()
{
    if (!writeTimer->isRunning())
    {
        return;
    }
    writeTimer->stop();
    writeFruCache();
}

void startMatch(void)
{
    if (fruMatches.size())
    {
        return;
    }

    fruMatches.reserve(2);

    auto bus = getSdBus();
    fruMatches.emplace_back(*bus,
                            "type='signal',arg0path='/xyz/openbmc_project/"
                            "FruDevice/',member='InterfacesAdded'",
                            [](sdbusplus::message_t& message) {
        sdbusplus::message::object_path path;
        ObjectType object;
        try
        {
            message.read(path, object);
        }
        catch (const sdbusplus::exception_t&)
        {
            return;
        }
        auto findType = object.find("xyz.openbmc_project.FruDevice");
        if (findType == object.end())
        {
            return;
        }
        writeFruIfRunning();
        frus[path] = object;
        recalculateHashes();
        lastDevId = 0xFF;
    });

    fruMatches.emplace_back(*bus,
                            "type='signal',arg0path='/xyz/openbmc_project/"
                            "FruDevice/',member='InterfacesRemoved'",
                            [](sdbusplus::message_t& message) {
        sdbusplus::message::object_path path;
        std::set<std::string> interfaces;
        try
        {
            message.read(path, interfaces);
        }
        catch (const sdbusplus::exception_t&)
        {
            return;
        }
        auto findType = interfaces.find("xyz.openbmc_project.FruDevice");
        if (findType == interfaces.end())
        {
            return;
        }
        writeFruIfRunning();
        frus.erase(path);
        recalculateHashes();
        lastDevId = 0xFF;
    });

    // call once to populate
    boost::asio::spawn(*getIoContext(), [](boost::asio::yield_context yield) {
        replaceCacheFru(getSdBus(), yield);
    });
}

/** @brief implements the read FRU data command
 *  @param fruDeviceId        - FRU Device ID
 *  @param fruInventoryOffset - FRU Inventory Offset to write
 *  @param countToRead        - Count to read
 *
 *  @returns ipmi completion code plus response data
 *   - countWritten  - Count written
 */
ipmi::RspType<uint8_t,             // Count
              std::vector<uint8_t> // Requested data
              >
    ipmiStorageReadFruData(ipmi::Context::ptr ctx, uint8_t fruDeviceId,
                           uint16_t fruInventoryOffset, uint8_t countToRead)
{
    if (fruDeviceId == 0xFF)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    auto [status, fru] = getFru(ctx, fruDeviceId);
    if (status != ipmi::ccSuccess)
    {
        return ipmi::response(status);
    }

    size_t fromFruByteLen = 0;
    if (countToRead + fruInventoryOffset < fru.size())
    {
        fromFruByteLen = countToRead;
    }
    else if (fru.size() > fruInventoryOffset)
    {
        fromFruByteLen = fru.size() - fruInventoryOffset;
    }
    else
    {
        return ipmi::responseReqDataLenExceeded();
    }

    std::vector<uint8_t> requestedData;

    requestedData.insert(requestedData.begin(),
                         fru.begin() + fruInventoryOffset,
                         fru.begin() + fruInventoryOffset + fromFruByteLen);

    return ipmi::responseSuccess(static_cast<uint8_t>(requestedData.size()),
                                 requestedData);
}

/** @brief implements the write FRU data command
 *  @param fruDeviceId        - FRU Device ID
 *  @param fruInventoryOffset - FRU Inventory Offset to write
 *  @param dataToWrite        - Data to write
 *
 *  @returns ipmi completion code plus response data
 *   - countWritten  - Count written
 */
ipmi::RspType<uint8_t>
    ipmiStorageWriteFruData(ipmi::Context::ptr ctx, uint8_t fruDeviceId,
                            uint16_t fruInventoryOffset,
                            std::vector<uint8_t>& dataToWrite)
{
    if (fruDeviceId == 0xFF)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    size_t writeLen = dataToWrite.size();

    auto [status, fru] = getFru(ctx, fruDeviceId);
    if (status != ipmi::ccSuccess)
    {
        return ipmi::response(status);
    }
    size_t lastWriteAddr = fruInventoryOffset + writeLen;
    if (fru.size() < lastWriteAddr)
    {
        fru.resize(fruInventoryOffset + writeLen);
    }

    std::copy(dataToWrite.begin(), dataToWrite.begin() + writeLen,
              fru.begin() + fruInventoryOffset);

    bool atEnd = false;

    if (fru.size() >= sizeof(FRUHeader))
    {
        FRUHeader* header = reinterpret_cast<FRUHeader*>(fru.data());

        size_t areaLength = 0;
        size_t lastRecordStart = std::max(
            {header->internalOffset, header->chassisOffset, header->boardOffset,
             header->productOffset, header->multiRecordOffset});
        lastRecordStart *= 8; // header starts in are multiples of 8 bytes

        if (header->multiRecordOffset)
        {
            // This FRU has a MultiRecord Area
            uint8_t endOfList = 0;
            // Walk the MultiRecord headers until the last record
            while (!endOfList)
            {
                // The MSB in the second byte of the MultiRecord header signals
                // "End of list"
                endOfList = fru[lastRecordStart + 1] & 0x80;
                // Third byte in the MultiRecord header is the length
                areaLength = fru[lastRecordStart + 2];
                // This length is in bytes (not 8 bytes like other headers)
                areaLength += 5; // The length omits the 5 byte header
                if (!endOfList)
                {
                    // Next MultiRecord header
                    lastRecordStart += areaLength;
                }
            }
        }
        else
        {
            // This FRU does not have a MultiRecord Area
            // Get the length of the area in multiples of 8 bytes
            if (lastWriteAddr > (lastRecordStart + 1))
            {
                // second byte in record area is the length
                areaLength = fru[lastRecordStart + 1];
                areaLength *= 8; // it is in multiples of 8 bytes
            }
        }
        if (lastWriteAddr >= (areaLength + lastRecordStart))
        {
            atEnd = true;
        }
    }
    uint8_t countWritten = 0;

    writeBus = cacheBus;
    writeAddr = cacheAddr;
    if (atEnd)
    {
        // cancel timer, we're at the end so might as well send it
        writeTimer->stop();
        if (!writeFru(fru))
        {
            return ipmi::responseInvalidFieldRequest();
        }
        countWritten = std::min(fru.size(), static_cast<size_t>(0xFF));
    }
    else
    {
        fruCache = fru; // Write-back
        // start a timer, if no further data is sent  to check to see if it is
        // valid
        writeTimer->start(std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::seconds(writeTimeoutSeconds)));
        countWritten = 0;
    }

    return ipmi::responseSuccess(countWritten);
}

/** @brief implements the get FRU inventory area info command
 *  @param fruDeviceId  - FRU Device ID
 *
 *  @returns IPMI completion code plus response data
 *   - inventorySize - Number of possible allocation units
 *   - accessType    - Allocation unit size in bytes.
 */
ipmi::RspType<uint16_t, // inventorySize
              uint8_t>  // accessType
    ipmiStorageGetFruInvAreaInfo(ipmi::Context::ptr ctx, uint8_t fruDeviceId)
{
    if (fruDeviceId == 0xFF)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    auto [ret, fru] = getFru(ctx, fruDeviceId);
    if (ret != ipmi::ccSuccess)
    {
        return ipmi::response(ret);
    }

    constexpr uint8_t accessType =
        static_cast<uint8_t>(GetFRUAreaAccessType::byte);

    return ipmi::responseSuccess(fru.size(), accessType);
}

ipmi_ret_t getFruSdrCount(ipmi::Context::ptr, size_t& count)
{
    count = deviceHashes.size();
    return IPMI_CC_OK;
}

ipmi_ret_t getFruSdrs([[maybe_unused]] ipmi::Context::ptr ctx, size_t index,
                      get_sdr::SensorDataFruRecord& resp)
{
    if (deviceHashes.size() < index)
    {
        return IPMI_CC_INVALID_FIELD_REQUEST;
    }
    auto device = deviceHashes.begin() + index;
    uint16_t& bus = device->second.first;
    uint8_t& address = device->second.second;
    std::string propertyName;

    boost::container::flat_map<std::string, Value>* fruData = nullptr;
    auto fru = std::find_if(frus.begin(), frus.end(),
                            [bus, address, &fruData](ManagedEntry& entry) {
        auto findFruDevice = entry.second.find("xyz.openbmc_project.FruDevice");
        if (findFruDevice == entry.second.end())
        {
            return false;
        }
        fruData = &(findFruDevice->second);
        auto findBus = findFruDevice->second.find("BUS");
        auto findAddress = findFruDevice->second.find("ADDRESS");
        if (findBus == findFruDevice->second.end() ||
            findAddress == findFruDevice->second.end())
        {
            return false;
        }
        if (std::get<uint32_t>(findBus->second) != bus)
        {
            return false;
        }
        if (std::get<uint32_t>(findAddress->second) != address)
        {
            return false;
        }
        return true;
    });
    if (fru == frus.end())
    {
        return IPMI_CC_RESPONSE_ERROR;
    }
    auto findProductName = fruData->find("BOARD_PRODUCT_NAME");
    auto findBoardName = fruData->find("PRODUCT_PRODUCT_NAME");
    if (findProductName != fruData->end())
    {
        propertyName = std::get<std::string>(findProductName->second);
    }
    else if (findBoardName != fruData->end())
    {
        propertyName = std::get<std::string>(findBoardName->second);
    }
    std::string name;

#ifdef USING_ENTITY_MANAGER_DECORATORS

    boost::container::flat_map<std::string, Value>* entityData = nullptr;

    // todo: this should really use caching, this is a very inefficient lookup
    boost::system::error_code ec;

    ManagedObjectType entities = ctx->bus->yield_method_call<ManagedObjectType>(
        ctx->yield, ec, entityManagerServiceName,
        "/xyz/openbmc_project/inventory", "org.freedesktop.DBus.ObjectManager",
        "GetManagedObjects");

    if (ec)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "GetMangagedObjects for ipmiStorageGetFruInvAreaInfo failed",
            phosphor::logging::entry("ERROR=%s", ec.message().c_str()));

        return ipmi::ccResponseError;
    }

    auto entity =
        std::find_if(entities.begin(), entities.end(),
                     [bus, address, &entityData, &name](ManagedEntry& entry) {
        auto findFruDevice = entry.second.find(
            "xyz.openbmc_project.Inventory.Decorator.I2CDevice");
        if (findFruDevice == entry.second.end())
        {
            return false;
        }

        // Integer fields added via Entity-Manager json are uint64_ts by
        // default.
        auto findBus = findFruDevice->second.find("Bus");
        auto findAddress = findFruDevice->second.find("Address");

        if (findBus == findFruDevice->second.end() ||
            findAddress == findFruDevice->second.end())
        {
            return false;
        }
        if ((std::get<uint64_t>(findBus->second) != bus) ||
            (std::get<uint64_t>(findAddress->second) != address))
        {
            return false;
        }

        auto fruName = findFruDevice->second.find("Name");
        if (fruName != findFruDevice->second.end())
        {
            name = std::get<std::string>(fruName->second);
        }

        // At this point we found the device entry and should return
        // true.
        auto findIpmiDevice =
            entry.second.find("xyz.openbmc_project.Inventory.Decorator.Ipmi");
        if (findIpmiDevice != entry.second.end())
        {
            entityData = &(findIpmiDevice->second);
        }

        return true;
    });

    if (entity == entities.end())
    {
        if constexpr (DEBUG)
        {
            std::fprintf(stderr, "Ipmi or FruDevice Decorator interface "
                                 "not found for Fru\n");
        }
    }

#endif

    if (name.empty())
    {
        std::vector<std::string> nameProperties = {
            "PRODUCT_PRODUCT_NAME",  "BOARD_PRODUCT_NAME",
            "PRODUCT_PART_NUMBER",   "BOARD_PART_NUMBER",
            "PRODUCT_MANUFACTURER",  "BOARD_MANUFACTURER",
            "PRODUCT_SERIAL_NUMBER", "BOARD_SERIAL_NUMBER"};

        for (const std::string& prop : nameProperties)
        {
            auto findProp = fruData->find(prop);
            if (findProp != fruData->end())
            {
                name = std::get<std::string>(findProp->second);
                break;
            }
        }
    }

    if (name.empty())
    {
        if (propertyName.empty())
        {
            name = "UNKNOWN";
        }
        else
        {
            name = propertyName;
        }
    }
    if (name.size() > maxFruSdrNameSize)
    {
        name = name.substr(0, maxFruSdrNameSize);
    }
    size_t sizeDiff = maxFruSdrNameSize - name.size();

    resp.header.record_id_lsb = 0x0; // calling code is to implement these
    resp.header.record_id_msb = 0x0;
    resp.header.sdr_version = ipmiSdrVersion;
    resp.header.record_type = get_sdr::SENSOR_DATA_FRU_RECORD;
    resp.header.record_length = sizeof(resp.body) + sizeof(resp.key) - sizeDiff;
    resp.key.deviceAddress = 0x20;
    resp.key.fruID = device->first;
    resp.key.accessLun = 0x80; // logical / physical fru device
    resp.key.channelNumber = 0x0;
    resp.body.reserved = 0x0;
    resp.body.deviceType = 0x10;
    resp.body.deviceTypeModifier = 0x0;

    uint8_t entityID = 0;
    uint8_t entityInstance = 0x1;

#ifdef USING_ENTITY_MANAGER_DECORATORS
    if (entityData)
    {
        auto entityIdProperty = entityData->find("EntityId");
        auto entityInstanceProperty = entityData->find("EntityInstance");

        if (entityIdProperty != entityData->end())
        {
            entityID = static_cast<uint8_t>(
                std::get<uint64_t>(entityIdProperty->second));
        }
        if (entityInstanceProperty != entityData->end())
        {
            entityInstance = static_cast<uint8_t>(
                std::get<uint64_t>(entityInstanceProperty->second));
        }
    }
#endif

    resp.body.entityID = entityID;
    resp.body.entityInstance = entityInstance;

    resp.body.oem = 0x0;
    resp.body.deviceIDLen = ipmi::storage::typeASCIILatin8 | name.size();
    name.copy(resp.body.deviceID, name.size());

    return IPMI_CC_OK;
}

std::vector<uint8_t>
    getType8SDRs(ipmi::sensor::EntityInfoMap::const_iterator& entity,
                 uint16_t recordId)
{
    std::vector<uint8_t> resp;
    get_sdr::SensorDataEntityRecord data{};

    /* Header */
    get_sdr::header::set_record_id(recordId, &(data.header));
    // Based on IPMI Spec v2.0 rev 1.1
    data.header.sdr_version = SDR_VERSION;
    data.header.record_type = 0x08;
    data.header.record_length = sizeof(data.key) + sizeof(data.body);

    /* Key */
    data.key.containerEntityId = entity->second.containerEntityId;
    data.key.containerEntityInstance = entity->second.containerEntityInstance;
    get_sdr::key::set_flags(entity->second.isList, entity->second.isLinked,
                            &(data.key));
    data.key.entityId1 = entity->second.containedEntities[0].first;
    data.key.entityInstance1 = entity->second.containedEntities[0].second;

    /* Body */
    data.body.entityId2 = entity->second.containedEntities[1].first;
    data.body.entityInstance2 = entity->second.containedEntities[1].second;
    data.body.entityId3 = entity->second.containedEntities[2].first;
    data.body.entityInstance3 = entity->second.containedEntities[2].second;
    data.body.entityId4 = entity->second.containedEntities[3].first;
    data.body.entityInstance4 = entity->second.containedEntities[3].second;

    resp.insert(resp.end(), (uint8_t*)&data, ((uint8_t*)&data) + sizeof(data));

    return resp;
}

std::vector<uint8_t> getType12SDRs(uint16_t index, uint16_t recordId)
{
    std::vector<uint8_t> resp;
    if (index == 0)
    {
        std::string bmcName = "Basbrd Mgmt Ctlr";
        Type12Record bmc(recordId, 0x20, 0, 0, 0xbf, 0x2e, 1, 0, bmcName);
        uint8_t* bmcPtr = reinterpret_cast<uint8_t*>(&bmc);
        resp.insert(resp.end(), bmcPtr, bmcPtr + sizeof(Type12Record));
    }
    else if (index == 1)
    {
        std::string meName = "Mgmt Engine";
        Type12Record me(recordId, 0x2c, 6, 0x24, 0x21, 0x2e, 2, 0, meName);
        uint8_t* mePtr = reinterpret_cast<uint8_t*>(&me);
        resp.insert(resp.end(), mePtr, mePtr + sizeof(Type12Record));
    }
    else
    {
        throw std::runtime_error("getType12SDRs:: Illegal index " +
                                 std::to_string(index));
    }

    return resp;
}

void registerStorageFRUFunctions()
{
    createTimers();
    startMatch();

    // <Get FRU Inventory Area Info>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdGetFruInventoryAreaInfo,
                          ipmi::Privilege::User, ipmiStorageGetFruInvAreaInfo);
    // <READ FRU Data>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdReadFruData, ipmi::Privilege::User,
                          ipmiStorageReadFruData);

    // <WRITE FRU Data>
    ipmi::registerHandler(ipmi::prioOpenBmcBase, ipmi::netFnStorage,
                          ipmi::storage::cmdWriteFruData,
                          ipmi::Privilege::Operator, ipmiStorageWriteFruData);
}
} // namespace storage
} // namespace ipmi
