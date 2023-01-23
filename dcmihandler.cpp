#include "config.h"

#include "dcmihandler.hpp"

#include "user_channel/channel_layer.hpp"

#include <bitset>
#include <cmath>
#include <fstream>
#include <ipmid/api.hpp>
#include <ipmid/utils.hpp>
#include <nlohmann/json.hpp>
#include <phosphor-logging/elog-errors.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/bus.hpp>
#include <variant>
#include <xyz/openbmc_project/Common/error.hpp>

using namespace phosphor::logging;
using InternalFailure =
    sdbusplus::xyz::openbmc_project::Common::Error::InternalFailure;

void register_netfn_dcmi_functions() __attribute__((constructor));

constexpr auto PCAP_PATH = "/xyz/openbmc_project/control/host0/power_cap";
constexpr auto PCAP_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";

constexpr auto POWER_CAP_PROP = "PowerCap";
constexpr auto POWER_CAP_ENABLE_PROP = "PowerCapEnable";

constexpr auto DCMI_PARAMETER_REVISION = 2;
constexpr auto DCMI_SPEC_MAJOR_VERSION = 1;
constexpr auto DCMI_SPEC_MINOR_VERSION = 5;
constexpr auto DCMI_CONFIG_PARAMETER_REVISION = 1;
constexpr auto DCMI_RAND_BACK_OFF_MASK = 0x80;
constexpr auto DCMI_OPTION_60_43_MASK = 0x02;
constexpr auto DCMI_OPTION_12_MASK = 0x01;
constexpr auto DCMI_ACTIVATE_DHCP_MASK = 0x01;
constexpr auto DCMI_ACTIVATE_DHCP_REPLY = 0x00;
constexpr auto DCMI_SET_CONF_PARAM_REQ_PACKET_MAX_SIZE = 0x04;
constexpr auto DCMI_SET_CONF_PARAM_REQ_PACKET_MIN_SIZE = 0x03;
constexpr auto DCMI_SET_PWR_LIMIT_EXCEPTION_ACTION_RESERVED = 0x12;
constexpr auto DCMI_SET_PWR_LIMIT_EXCEPTION_ACTION_RESERVED1 = 0xFF;
constexpr auto DHCP_TIMING1 = 0x04;       // 4 sec
constexpr auto DHCP_TIMING2_UPPER = 0x00; // 2 min
constexpr auto DHCP_TIMING2_LOWER = 0x78;
constexpr auto DHCP_TIMING3_UPPER = 0x00; // 64 sec
constexpr auto DHCP_TIMING3_LOWER = 0x40;
// When DHCP Option 12 is enabled the string "SendHostName=true" will be
// added into n/w configuration file and the parameter
// SendHostNameEnabled will set to true.
constexpr auto DHCP_OPT12_ENABLED = "SendHostNameEnabled";

constexpr auto SENSOR_VALUE_INTF = "xyz.openbmc_project.Sensor.Value";
constexpr auto SENSOR_VALUE_PROP = "Value";

constexpr auto controlPowerModeIntf = "xyz.openbmc_project.Control.Power.Mode";
constexpr auto controlPowerCapIntf =  "xyz.openbmc_project.Control.Power.Cap";
constexpr auto control = "/xyz/openbmc_project/control/";

using namespace phosphor::logging;

namespace dcmi
{

// Refer Table 6-14, DCMI Entity ID Extension, DCMI v1.5 spec
static const std::map<uint8_t, std::string> entityIdToName{
    {0x40, "inlet"}, {0x37, "inlet"},     {0x41, "cpu"},
    {0x03, "cpu"},   {0x42, "baseboard"}, {0x07, "baseboard"}};

bool isDCMIPowerMgmtSupported()
{
    auto data = parseJSONConfig(gDCMICapabilitiesConfig);

    return (gDCMIPowerMgmtSupported == data.value(gDCMIPowerMgmtCapability, 0));
}

uint32_t getPcap(ipmi::Context::ptr& ctx)
{
    std::string service;
    uint32_t pcap = 0;
    auto ec = ipmi::getService(ctx, PCAP_INTERFACE, PCAP_PATH, service);
    if (!ec)
    {
        ec = ipmi::getDbusProperty(ctx, service, PCAP_PATH, PCAP_INTERFACE,
                                   POWER_CAP_PROP, pcap);
    }
    if (ec)
    {
        log<level::ERR>("Error in getting PowerCap value");
        elog<InternalFailure>();
    }
    return pcap;
}

bool getPcapEnabled(ipmi::Context::ptr& ctx)
{
    std::string service;
    bool pcapEnabled = false;
    auto ec = ipmi::getService(ctx, PCAP_INTERFACE, PCAP_PATH, service);
    if (!ec)
    {
        ec = ipmi::getDbusProperty(ctx, service, PCAP_PATH, PCAP_INTERFACE,
                                   POWER_CAP_ENABLE_PROP, pcapEnabled);
    }
    if (ec)
    {
        log<level::ERR>("Error in getting PowerCapEnable status");
        elog<InternalFailure>();
    }
    return pcapEnabled;
}

void setPcap(ipmi::Context::ptr& ctx, uint32_t powerCap)
{
    std::string service;

    auto ec = ipmi::getService(ctx, PCAP_INTERFACE, PCAP_PATH, service);
    if (!ec)
    {
        ec = ipmi::setDbusProperty(ctx, service, PCAP_PATH, PCAP_INTERFACE,
                                   POWER_CAP_PROP, powerCap);
    }
    if (ec)
    {
        log<level::ERR>("Error in setPcap property");
        elog<InternalFailure>();
    }
}

void setPcapEnable(ipmi::Context::ptr& ctx, bool enabled)
{
    std::string service;

    auto ec = ipmi::getService(ctx, PCAP_INTERFACE, PCAP_PATH, service);
    if (!ec)
    {
        ec = ipmi::setDbusProperty(ctx, service, PCAP_PATH, PCAP_INTERFACE,
                                   POWER_CAP_ENABLE_PROP, enabled);
    }
    if (ec)
    {
        log<level::ERR>("Error in setPcapEnabled property");
        elog<InternalFailure>();
    }
}

void readAssetTagObjectTree(ipmi::Context::ptr& ctx,
                            dcmi::assettag::ObjectTree& objectTree)
{
    static constexpr auto inventoryRoot = "/xyz/openbmc_project/inventory/";

    auto ec = ipmi::getAllDbusObjects(ctx, inventoryRoot, dcmi::assetTagIntf,
                                      objectTree);
    if (ec)
    {
        log<level::ERR>(
            "Error in readAssetTagObjectTree getAllDbusObjects call");
        elog<InternalFailure>();
    }

    if (objectTree.empty())
    {
        log<level::ERR>("AssetTag property is not populated");
        elog<InternalFailure>();
    }
}

std::string readAssetTag(ipmi::Context::ptr& ctx)
{
    dcmi::assettag::ObjectTree objectTree;

    // Read the object tree with the inventory root to figure out the object
    // that has implemented the Asset tag interface.
    readAssetTagObjectTree(ctx, objectTree);
    std::string assetTag;
    auto ec = ipmi::getDbusProperty(
        ctx, (objectTree.begin()->second.begin()->first).c_str(),
        (objectTree.begin()->first).c_str(), dcmi::assetTagIntf,
        dcmi::assetTagProp, assetTag);
    if (ec)
    {
        log<level::ERR>("Error in reading asset tag");
        elog<InternalFailure>();
    }

    return assetTag;
}

void writeAssetTag(ipmi::Context::ptr& ctx, const std::string& assetTag)
{
    dcmi::assettag::ObjectTree objectTree;
    readAssetTagObjectTree(ctx, objectTree);

    auto ec = ipmi::setDbusProperty(
        ctx, (objectTree.begin()->second.begin()->first).c_str(),
        (objectTree.begin()->first).c_str(), dcmi::assetTagIntf,
        dcmi::assetTagProp, assetTag);
    if (ec)
    {
        log<level::ERR>("Error in writing asset tag");
        elog<InternalFailure>();
    }
}

std::string getHostName(ipmi::Context::ptr& ctx)
{
    std::string service;
    std::string hostName;
    auto ec =
        ipmi::getService(ctx, networkConfigIntf, networkConfigObj, service);
    if (!ec)
    {
        ec = ipmi::getDbusProperty(ctx, service, networkConfigObj,
                                   networkConfigIntf, hostNameProp, hostName);
    }
    if (ec)
    {
        log<level::ERR>("Error in get HostName");
        elog<InternalFailure>();
    }
    return hostName;
}

bool getDHCPEnabled()
{
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};

    auto ethdevice = ipmi::getChannelName(ethernetDefaultChannelNum);
    auto ethernetObj =
        ipmi::getDbusObject(bus, ethernetIntf, networkRoot, ethdevice);
    auto service = ipmi::getService(bus, ethernetIntf, ethernetObj.first);
    auto value = ipmi::getDbusProperty(bus, service, ethernetObj.first,
   		    ethernetIntf, "DHCPEnabled");
    auto enumValue = std::get<std::string>(value);
    if (enumValue == "xyz.openbmc_project.Network.EthernetInterface.DHCPConf.none")
    {
	    return false;
    }
    else
    {
	    return true;
    }	
}

bool getDHCPOption(std::string prop)
{
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};

    auto service = ipmi::getService(bus, dhcpIntf, dhcpObj);
    auto value = ipmi::getDbusProperty(bus, service, dhcpObj, dhcpIntf, prop);

    return std::get<bool>(value);
}

void setDHCPOption(std::string prop, bool value)
{
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};

    auto service = ipmi::getService(bus, dhcpIntf, dhcpObj);
    ipmi::setDbusProperty(bus, service, dhcpObj, dhcpIntf, prop, value);
}

Json parseJSONConfig(const std::string& configFile)
{
    std::ifstream jsonFile(configFile);
    if (!jsonFile.is_open())
    {
        log<level::ERR>("Temperature readings JSON file not found");
        elog<InternalFailure>();
    }

    auto data = Json::parse(jsonFile, nullptr, false);
    if (data.is_discarded())
    {
        log<level::ERR>("Temperature readings JSON parser failure");
        elog<InternalFailure>();
    }

    return data;
}
void restartSystemdUnit(const std::string& unit)
{
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};

    try
    {
        auto method = bus.new_method_call(systemBusName, systemPath,
                                          systemIntf, "RestartUnit");
        method.append(unit.c_str(), "replace");
        bus.call_noreply(method);
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        log<level::ERR>("Failed to restart nslcd service",
                        entry("ERR=%s", ex.what()));
        elog<InternalFailure>();
    }
}
} // namespace dcmi

ipmi::RspType<uint16_t, // Reserved.
              uint8_t,  // Exception action.
              uint16_t, // Power limit requested in watts.
              uint32_t, // Correction time limit in milliseconds.
              uint16_t, // Reserved.
              uint16_t  // Statistics sampling period in seconds.
              >
    getPowerLimit(ipmi::Context::ptr ctx, uint16_t reqByte)
{
    if (!dcmi::isDCMIPowerMgmtSupported())
    {
        log<level::ERR>("DCMI Power management is unsupported!");
        return ipmi::responseInvalidCommand();
    }

    if (reqByte != 0)
    {
        return ipmi::responseInvalidFieldRequest();
    }

    uint32_t pcapValue = 0;
    bool pcapEnable = false;

    try
    {
        pcapValue = dcmi::getPcap(ctx);
        pcapEnable = dcmi::getPcapEnabled(ctx);
    }
    catch (const InternalFailure& e)
    {
        return ipmi::responseUnspecifiedError();
    }

    /*
     * Exception action if power limit is exceeded and cannot be controlled
     * with the correction time limit is hardcoded to Hard Power Off system
     * and log event to SEL.
     */
    uint8_t exceptionAction = 0x01;
    uint16_t powerLimit = static_cast<uint16_t>(pcapValue);
    uint16_t reserved = 0;
    uint32_t correctionTime = 0;
    uint16_t reserved1 = 0;
    uint16_t samplingPeriod = 0;

    /*
     * Correction time limit and Statistics sampling period is currently not
     * populated.
     */
    if (pcapEnable)
    {
        return ipmi::responseSuccess(reserved, exceptionAction, powerLimit,
                                     correctionTime, reserved1, samplingPeriod);
    }
    else
    {
        return ipmi::responseNoActivePowerLimit(reserved, exceptionAction,
                                                powerLimit, correctionTime,
                                                reserved1, samplingPeriod);
    }
}

ipmi::RspType<> setPowerLimit(ipmi::Context::ptr ctx, uint16_t reserved,
                              uint8_t reserved1, uint8_t exceptionAction,
                              uint16_t powerLimit, uint32_t correctionTime,
                              uint16_t reserved2, uint16_t samplingPeriod)
{
    if (!dcmi::isDCMIPowerMgmtSupported())
    {
        log<level::ERR>("DCMI Power management is unsupported!");
        return ipmi::responseInvalidCommand();
    }

    if ((reserved != 0) || (reserved1 != 0) || (reserved2 != 0))
    {
        return ipmi::responseInvalidFieldRequest();
    }

    sdbusplus::bus::bus sdbus{ipmid_get_sd_bus_connection()};
    if ((exceptionAction >= DCMI_SET_PWR_LIMIT_EXCEPTION_ACTION_RESERVED) &&
        (exceptionAction <= DCMI_SET_PWR_LIMIT_EXCEPTION_ACTION_RESERVED1))
    {
        return ipmi::responseInvalidFieldRequest();
    }
    
     // Only process the power limit requested in watts.
    try
    {
        ipmi::DbusObjectInfo objectTree;
        auto ec = ipmi::getDbusObject(ctx, controlPowerModeIntf, control, "",
                                      objectTree);
        if (!ec)
        {
            ec = ipmi::setDbusProperty(ctx, objectTree.second, objectTree.first,
                                       controlPowerCapIntf, POWER_CAP_PROP,
                                       static_cast<uint32_t>(powerLimit));
        }
        if (ec)
        {
            return ipmi::responseUnspecifiedError();
        }
    }
    catch (sdbusplus::exception_t& e)
    {
        return ipmi::responseUnspecifiedError();
    }
    // Only process the power limit requested in watts.
    try
    {
        dcmi::setPcap(ctx, powerLimit);
    }
    catch (sdbusplus::exception_t& e)
    {
        return ipmi::responseUnspecifiedError();
    }

    log<level::INFO>("Set Power Cap", entry("POWERCAP=%u", powerLimit));

    return ipmi::responseSuccess();
}

ipmi::RspType<> applyPowerLimit(ipmi::Context::ptr ctx,
                                uint8_t powerlimitaction, uint16_t reserved)
{
    if (!dcmi::isDCMIPowerMgmtSupported())
    {
        log<level::ERR>("DCMI Power management is unsupported!");
        return ipmi::responseInvalidCommand();
    }

    // Reserved bytes must be zero
    if (reserved != 0)
    {
        log<level::ERR>(
            "DCMI Activate/Deactivate reserved field contents modified");
        return ipmi::responseInvalidFieldRequest();
    }

    // To Activate/Deactivate power limit action can be 0 or 1
    if (powerlimitaction < 0 || powerlimitaction > 1)
    {
        log<level::ERR>("DCMI Activate/Deactivate parameter out of range");
        return ipmi::responseParmOutOfRange();
    }
    try
    {
        ipmi::DbusObjectInfo objectTree;
        auto ec = ipmi::getDbusObject(ctx, controlPowerModeIntf, control, "",
                                      objectTree);
        if (ec)
        {
            return ipmi::responseUnspecifiedError();
        }
        if (powerlimitaction)
        {
            ipmi::setDbusProperty(
                ctx, objectTree.second, objectTree.first, controlPowerModeIntf,
                "PowerMode",
                std::string("xyz.openbmc_project.Control.Power."
                            "Mode.PowerMode.PowerSaving"));
        }
        else
        {
            ipmi::setDbusProperty(
                ctx, objectTree.second, objectTree.first, controlPowerModeIntf,
                "PowerMode",
                std::string("xyz.openbmc_project.Control.Power."
                            "Mode.PowerMode.MaximumPerformance"));
        }
    }
    catch (const sdbusplus::exception_t& e)
    {
        return ipmi::responseUnspecifiedError();
    }
    try
    {
        dcmi::setPcapEnable(ctx, static_cast<bool>(powerlimitaction));
    }
    catch (const InternalFailure& e)
    {
        return ipmi::responseUnspecifiedError();
    }

    log<level::INFO>("Set Power Cap Enable",
                     entry("POWERCAPENABLE=%u", powerlimitaction));

    return ipmi::responseSuccess();
}

ipmi::RspType<uint8_t,             // Total Asset Tag Length
              std::vector<uint8_t> // Asset Tag Data
              >
    getAssetTag(ipmi::Context::ptr ctx, uint8_t offset, uint8_t bytes)
{
    // Verify offset to read and number of bytes to read are not exceeding the
    // range.
    if ((offset > dcmi::assetTagMaxOffset) || (bytes > dcmi::maxBytes) ||
        ((offset + bytes) > dcmi::assetTagMaxSize))
    {
        return ipmi::responseParmOutOfRange();
    }

    std::string assetTag;

    try
    {
        assetTag = dcmi::readAssetTag(ctx);
    }
    catch (const InternalFailure& e)
    {
        return ipmi::responseUnspecifiedError();
    }

    std::vector<uint8_t> assetTagData{};
    uint8_t tagLength = 0;
    // Return if the asset tag is not populated.
    if (!assetTag.size())
    {
        tagLength = 0;
        assetTagData.clear();
        return ipmi::responseSuccess(tagLength, assetTagData);
    }

    // If the asset tag is longer than 63 bytes, restrict it to 63 bytes to suit
    // Get Asset Tag command.
    if (assetTag.size() > dcmi::assetTagMaxSize)
    {
        assetTag.resize(dcmi::assetTagMaxSize);
    }

    // If the requested offset is beyond the asset tag size.
    if (offset >= assetTag.size())
    {
        return ipmi::responseParmOutOfRange();
    }

    auto returnData = assetTag.substr(offset, bytes);

    tagLength = assetTag.size();
    assetTagData.assign(returnData.begin(), returnData.end());

    return ipmi::responseSuccess(tagLength, assetTagData);
}

ipmi::RspType<uint8_t // Total Asset Tag Length
              >
    setAssetTag(ipmi::Context::ptr ctx, uint8_t offset, uint8_t bytes,
                std::vector<uint8_t> assetTagData)
{
    // Verify offset to read and number of bytes to read are not exceeding the
    // range.
    if ((offset > dcmi::assetTagMaxOffset) || (bytes > dcmi::maxBytes) ||
        ((offset + bytes) > dcmi::assetTagMaxSize))
    {
        return ipmi::responseParmOutOfRange();
    }

    try
    {
        auto assetTag = dcmi::readAssetTag(ctx);
        if (offset > assetTag.size())
        {
            return ipmi::responseParmOutOfRange();
        }

        std::string assetTagDataStr(assetTagData.begin(), assetTagData.end());
        assetTag.replace(offset, (assetTag.size() - offset),
                         assetTagDataStr.c_str(), bytes);
        dcmi::writeAssetTag(ctx, assetTag);

        uint8_t tagLength = assetTag.size();
        return ipmi::responseSuccess(tagLength);
    }
    catch (const std::exception& e)
    {
        return ipmi::responseUnspecifiedError();
    }
}

ipmi::RspType<uint8_t,             // ID String Length
              std::vector<uint8_t> // Data
              >
    getMgmntCtrlIdStr(ipmi::Context::ptr ctx, uint8_t offset, uint8_t bytes)
{
    if (bytes > dcmi::maxBytes || (offset + bytes) > dcmi::maxCtrlIdStrLen)
    {
        return ipmi::responseParmOutOfRange();
    }

    std::string hostName;
    try
    {
        hostName = dcmi::getHostName(ctx);
    }
    catch (const InternalFailure& e)
    {
        return ipmi::responseUnspecifiedError();
    }

    if (offset >= hostName.length())
    {
        return ipmi::responseParmOutOfRange();
    }
    auto responseStr = hostName.substr(offset, bytes);
    uint8_t strLen = hostName.length();
    std::vector<uint8_t> data(responseStr.begin(), responseStr.end());

    return ipmi::responseSuccess(strLen, data);
}

ipmi::RspType<uint8_t // Last Offset Written
              >
    setMgmntCtrlIdStr(ipmi::Context::ptr ctx, uint8_t offset, uint8_t bytes,
                      std::vector<uint8_t> data)
{
    static std::array<char, dcmi::maxCtrlIdStrLen + 1> newCtrlIdStr;

    if (bytes > dcmi::maxBytes ||
        ((offset + bytes) > dcmi::maxCtrlIdStrLen + 1) ||
        ((offset + bytes) == dcmi::maxCtrlIdStrLen + 1 &&
         data[bytes - 1] != '\0'))
    {
        return ipmi::responseParmOutOfRange();
    }

    try
    {
        /* if there is no old value and offset is not 0 */
        if (newCtrlIdStr[0] == '\0' && offset != 0)
        {
            /* read old ctrlIdStr */
            auto hostName = dcmi::getHostName(ctx);
            hostName.resize(dcmi::maxCtrlIdStrLen);
            std::copy(begin(hostName), end(hostName), begin(newCtrlIdStr));
            newCtrlIdStr[hostName.length()] = '\0';
        }

        /* replace part of string and mark byte after the last as \0 */
        auto restStrIter =
            std::copy_n(data.begin(), bytes, begin(newCtrlIdStr) + offset);
        /* if the last written byte is not 64th - add '\0' */
        if (offset + bytes <= dcmi::maxCtrlIdStrLen)
        {
            *restStrIter = '\0';
        }

        /* if input data contains '\0' whole string is sent - update hostname */
        auto it = std::find(data.begin(), data.begin() + bytes, '\0');
        if (it != data.begin() + bytes)
        {
            sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};
            ipmi::setDbusProperty(bus, dcmi::networkServiceName,
                                  dcmi::networkConfigObj,
                                  dcmi::networkConfigIntf, dcmi::hostNameProp,
                                  std::string(newCtrlIdStr.data()));
        }
    }
    catch (const InternalFailure& e)
    {
        return ipmi::responseUnspecifiedError();
    }

    return ipmi::responseSuccess(offset + bytes);
}

// List of the capabilities under each parameter
dcmi::DCMICaps dcmiCaps = {
    // Supported DCMI Capabilities
    {dcmi::DCMICapParameters::SUPPORTED_DCMI_CAPS,
     {3,
      {{"PowerManagement", 2, 0, 1},
       {"OOBSecondaryLan", 3, 2, 1},
       {"SerialTMODE", 3, 1, 1},
       {"InBandSystemInterfaceChannel", 3, 0, 1}}}},
    // Mandatory Platform Attributes
    {dcmi::DCMICapParameters::MANDATORY_PLAT_ATTRIBUTES,
     {5,
      {{"SELAutoRollOver", 1, 15, 1},
       {"FlushEntireSELUponRollOver", 1, 14, 1},
       {"RecordLevelSELFlushUponRollOver", 1, 13, 1},
       {"NumberOfSELEntries", 1, 0, 12},
       {"TempMonitoringSamplingFreq", 5, 0, 8}}}},
    // Optional Platform Attributes
    {dcmi::DCMICapParameters::OPTIONAL_PLAT_ATTRIBUTES,
     {2,
      {{"PowerMgmtDeviceSlaveAddress", 1, 1, 7},
       {"BMCChannelNumber", 2, 4, 4},
       {"DeviceRivision", 2, 0, 4}}}},
    // Manageability Access Attributes
    {dcmi::DCMICapParameters::MANAGEABILITY_ACCESS_ATTRIBUTES,
     {3,
      {{"MandatoryPrimaryLanOOBSupport", 1, 0, 8},
       {"OptionalSecondaryLanOOBSupport", 2, 0, 8},
       {"OptionalSerialOOBMTMODECapability", 3, 0, 8}}}}};
ipmi::RspType<uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>
    getDCMICapabilities(uint8_t parameterselector)
{

    std::ifstream dcmiCapFile(dcmi::gDCMICapabilitiesConfig);
    if (!dcmiCapFile.is_open())
    {
        log<level::ERR>("DCMI Capabilities file not found");
        return ipmi::responseUnspecifiedError();
    }

    auto data = nlohmann::json::parse(dcmiCapFile, nullptr, false);
    if (data.is_discarded())
    {
        log<level::ERR>("DCMI Capabilities JSON parser failure");
        return ipmi::responseUnspecifiedError();
    }

    // get list of capabilities in a parameter
    auto caps =
        dcmiCaps.find(static_cast<dcmi::DCMICapParameters>(parameterselector));
    if (caps == dcmiCaps.end())
    {
        log<level::ERR>("Invalid input parameter");
        return ipmi::responseInvalidFieldRequest();
    }

    std::vector<uint8_t> capData(caps->second.size, 0);
    // For each capabilities in a parameter fill the data from
    // the json file based on the capability name.
    for (auto cap : caps->second.capList)
    {
        // If the data is beyond first byte boundary, insert in a
        // 16bit pattern for example number of SEL entries are represented
        // in 12bits.
        if ((cap.length + cap.position) > dcmi::gByteBitSize)
        {
            uint16_t val = data.value(cap.name.c_str(), 0);
            // According to DCMI spec v1.5, max number of SEL entries is
            // 4096, but bit 12b of DCMI capabilities Mandatory Platform
            // Attributes field is reserved and therefore we can use only
            // the provided 12 bits with maximum value of 4095.
            // We're playing safe here by applying the mask
            // to ensure that provided value will fit into 12 bits.
            if (cap.length > dcmi::gByteBitSize)
            {
                val &= dcmi::gMaxSELEntriesMask;
            }
            val <<= cap.position;
            capData[cap.bytePosition - 1] |= static_cast<uint8_t>(val);
            capData[cap.bytePosition] |= val >> dcmi::gByteBitSize;
        }
        else
        {
            capData[cap.bytePosition - 1] |= data.value(cap.name.c_str(), 0)
                                             << cap.position;
        }
    }

    uint8_t major = DCMI_SPEC_MAJOR_VERSION;
    uint8_t minor = DCMI_SPEC_MINOR_VERSION;
    uint8_t paramRevision = DCMI_PARAMETER_REVISION;

    return ipmi::responseSuccess(major, minor, paramRevision, capData);
}

namespace dcmi
{
namespace temp_readings
{

Temperature readTemp(ipmi::Context::ptr& ctx, const std::string& dbusService,
                     const std::string& dbusPath)
{
    // Read the temperature value from d-bus object. Need some conversion.
    // As per the interface xyz.openbmc_project.Sensor.Value, the temperature
    // is an double and in degrees C. It needs to be scaled by using the
    // formula Value * 10^Scale. The ipmi spec has the temperature as a uint8_t,
    // with a separate single bit for the sign.

    ipmi::PropertyMap result;
    ipmi::getAllDbusProperties(ctx, dbusService, dbusPath, SENSOR_VALUE_INTF,
                               result);
    auto temperature =
        std::visit(ipmi::VariantToDoubleVisitor(), result.at("Value"));
    double absTemp = std::abs(temperature);

    auto findFactor = result.find("Scale");
    double factor = 0.0;
    if (findFactor != result.end())
    {
        factor = std::visit(ipmi::VariantToDoubleVisitor(), findFactor->second);
    }
    double scale = std::pow(10, factor);

    auto tempDegrees = absTemp * scale;
    // Max absolute temp as per ipmi spec is 128.
    if (tempDegrees > maxTemp)
    {
        tempDegrees = maxTemp;
    }

    return std::make_tuple(static_cast<uint8_t>(tempDegrees),
                           (temperature < 0));
}

std::tuple<ResponseList, NumInstances>
    read(ipmi::Context::ptr& ctx, const std::string& type, uint8_t instance)
{
    ResponseList response{};
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};

    if (!instance)
    {
        log<level::ERR>("Expected non-zero instance");
        elog<InternalFailure>();
    }

    auto data = parseJSONConfig(gDCMISensorsConfig);
    static const std::vector<Json> empty{};
    std::vector<Json> readings = data.value(type, empty);
    size_t numInstances = readings.size();
    for (const auto& j : readings)
    {
        uint8_t instanceNum = j.value("instance", 0);
        // Not the instance we're interested in
        if (instanceNum != instance)
        {
            continue;
        }

        std::string path = j.value("dbus", "");
        std::string service;
        auto ec = ipmi::getService(ctx, "xyz.openbmc_project.Sensor.Value",
                                   path, service);
        if (ec)
        {
            return std::make_tuple(response, 0);
        }

        Response r{};
        r.instance = instance;
        uint8_t temp{};
        bool sign{};
        std::tie(temp, sign) = readTemp(ctx, service, path);
        r.temperature = temp;
        r.sign = sign;
        response.push_back(r);

        // Found the instance we're interested in
        break;
    }
    if (response.empty())
    {
        return std::make_tuple(response, 0);
    }
    if (numInstances > maxInstances)
    {
        numInstances = maxInstances;
    }
    return std::make_tuple(response, numInstances);
}

std::tuple<ResponseList, NumInstances> readAll(ipmi::Context::ptr& ctx,
                                               const std::string& type,
                                               uint8_t instanceStart)
{
    ResponseList response{};
    sdbusplus::bus::bus bus{ipmid_get_sd_bus_connection()};

    size_t numInstances = 0;
    auto data = parseJSONConfig(gDCMISensorsConfig);
    static const std::vector<Json> empty{};
    std::vector<Json> readings = data.value(type, empty);
    numInstances = readings.size();
    for (const auto& j : readings)
    {
        try
        {
            // Max of 8 response data sets
            if (response.size() == maxDataSets)
            {
                break;
            }

            uint8_t instanceNum = j.value("instance", 0);
            // Not in the instance range we're interested in
            if (instanceNum < instanceStart)
            {
                continue;
            }

            std::string path = j.value("dbus", "");
            std::string service;
            auto ec = ipmi::getService(ctx, "xyz.openbmc_project.Sensor.Value",
                                       path, service);
            if (ec)
            {
                continue;
            }
            Response r{};
            r.instance = instanceNum;
            uint8_t temp{};
            bool sign{};
            std::tie(temp, sign) = readTemp(ctx, service, path);
            r.temperature = temp;
            r.sign = sign;
            response.push_back(r);
        }
        catch (const std::exception& e)
        {
            log<level::DEBUG>(e.what());
            continue;
        }
    }

    if (numInstances > maxInstances)
    {
        numInstances = maxInstances;
    }
    return std::make_tuple(response, numInstances);
}

} // namespace temp_readings
} // namespace dcmi

ipmi::RspType<uint8_t,              // No of instances for requested id
              uint8_t,              // No of sets of temperature data
              std::vector<uint16_t> // Temperature Data
              >
    getTempReadings(ipmi::Context::ptr ctx, int8_t sensorType, uint8_t entityId,
                    uint8_t entityInstance, uint8_t instanceStart)
{
    uint8_t numInstances = 0, numDataSets = 0;
    auto it = dcmi::entityIdToName.find(entityId);

    if (it == dcmi::entityIdToName.end())
    {
        log<level::ERR>("Unknown Entity ID", entry("ENTITY_ID=%d", entityId));
        return ipmi::responseInvalidFieldRequest();
    }

    if (sensorType != dcmi::temperatureSensorType)
    {
        log<level::ERR>("Invalid sensor type",
                        entry("SENSOR_TYPE=%d", sensorType));
        return ipmi::responseInvalidFieldRequest();
    }

    dcmi::temp_readings::ResponseList temps{};
    try
    {
        if (!entityInstance)
        {
            // Read all instances
            std::tie(temps, numInstances) =
                dcmi::temp_readings::readAll(ctx, it->second, instanceStart);
        }
        else
        {
            // Read one instance
            std::tie(temps, numInstances) =
                dcmi::temp_readings::read(ctx, it->second, entityInstance);
        }
        numDataSets = temps.size();
    }
    catch (const InternalFailure& e)
    {
        return ipmi::responseUnspecifiedError();
    }
    if (!numInstances)
    {
        return ipmi::responseUnspecifiedError();
    }

    std::vector<uint16_t> tempVals{};
    for (size_t i = 0; i < numDataSets; i++)
    {
        uint16_t tempVal = temps[i].temperature | temps[i].sign;
        tempVal = (temps[i].instance << 8) | tempVal;
        tempVals.push_back(tempVal);
    }
    return ipmi::responseSuccess(numInstances, numDataSets, tempVals);
}

int64_t getPwrReading(ipmi::Context::ptr& ctx)
{
    std::ifstream sensorFile(POWER_READING_SENSOR);
    std::string objectPath;
    if (!sensorFile.is_open())
    {
        log<level::ERR>("Power reading configuration file not found",
                        entry("POWER_SENSOR_FILE=%s", POWER_READING_SENSOR));
        elog<InternalFailure>();
    }

    auto data = nlohmann::json::parse(sensorFile, nullptr, false);
    if (data.is_discarded())
    {
        log<level::ERR>("Error in parsing configuration file",
                        entry("POWER_SENSOR_FILE=%s", POWER_READING_SENSOR));
        elog<InternalFailure>();
    }

    objectPath = data.value("path", "");
    if (objectPath.empty())
    {
        log<level::ERR>("Power sensor D-Bus object path is empty",
                        entry("POWER_SENSOR_FILE=%s", POWER_READING_SENSOR));
        elog<InternalFailure>();
    }

    // Return default value if failed to read from D-Bus object
    double power = 0;
    try
    {
        std::string service;
        auto ec = ipmi::getService(ctx, SENSOR_VALUE_INTF, objectPath, service);
        if (!ec)
        {
            ec = ipmi::getDbusProperty(ctx, service, objectPath,
                                       SENSOR_VALUE_INTF, SENSOR_VALUE_PROP,
                                       power);
        }
        if (ec)
        {
            log<level::ERR>("Error in getPcapEnabled property");
            elog<InternalFailure>();
        }
    }
    catch (const std::exception& e)
    {
        log<level::ERR>("Failure to read power value from D-Bus object",
                        entry("OBJECT_PATH=%s", objectPath.c_str()),
                        entry("INTERFACE=%s", SENSOR_VALUE_INTF));
    }
    return static_cast<int64_t>(power);
}

ipmi_ret_t setDCMIConfParams(ipmi_netfn_t netfn, ipmi_cmd_t cmd,
                             ipmi_request_t request, ipmi_response_t response,
                             ipmi_data_len_t data_len, ipmi_context_t context)
{
    auto requestData =
        reinterpret_cast<const dcmi::SetConfParamsRequest*>(request);

    if (*data_len < DCMI_SET_CONF_PARAM_REQ_PACKET_MIN_SIZE ||
        *data_len > DCMI_SET_CONF_PARAM_REQ_PACKET_MAX_SIZE)
    {
        log<level::ERR>("Invalid Requested Packet size",
                        entry("PACKET SIZE=%d", *data_len));
        *data_len = 0;
        return IPMI_CC_INVALID_FIELD_REQUEST;
    }
    *data_len = 0;

    try
    {
        // Take action based on the Parameter Selector
        switch (
            static_cast<dcmi::DCMIConfigParameters>(requestData->paramSelect))
        {
            case dcmi::DCMIConfigParameters::ActivateDHCP:

                if ((requestData->data[0] & DCMI_ACTIVATE_DHCP_MASK) &&
                    dcmi::getDHCPEnabled())
                {
                     dcmi::restartSystemdUnit(dcmi::networkdService);
                }
                break;

            case dcmi::DCMIConfigParameters::DiscoveryConfig:

                if (requestData->data[0] & DCMI_OPTION_12_MASK)
                {
                    dcmi::setDHCPOption(DHCP_OPT12_ENABLED, true);
                }
                else
                {
                    dcmi::setDHCPOption(DHCP_OPT12_ENABLED, false);
                }

                // Systemd-networkd doesn't support Random Back off
                if (requestData->data[0] & DCMI_RAND_BACK_OFF_MASK)
                {
                    return IPMI_CC_INVALID;
                }
                break;
            // Systemd-networkd doesn't allow to configure DHCP timigs
            case dcmi::DCMIConfigParameters::DHCPTiming1:
            case dcmi::DCMIConfigParameters::DHCPTiming2:
            case dcmi::DCMIConfigParameters::DHCPTiming3:
            default:
                return IPMI_CC_INVALID;
        }
    }
    catch (const std::exception& e)
    {
        log<level::ERR>(e.what());
        return IPMI_CC_UNSPECIFIED_ERROR;
    }
    return IPMI_CC_OK;
}

ipmi::RspType<uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>
    getDCMIConfParams(ipmi::Context::ptr ctx, uint8_t paramselect, uint8_t)
{
    std::vector<uint8_t> data{0};
    uint8_t major, minor, paramRevision;

    try
    {
        // Take action based on the Parameter Selector
        switch (static_cast<dcmi::DCMIConfigParameters>(paramselect))
        {
            case dcmi::DCMIConfigParameters::ActivateDHCP:
                data[0] = DCMI_ACTIVATE_DHCP_REPLY;
                break;
            case dcmi::DCMIConfigParameters::DiscoveryConfig:
                if (dcmi::getDHCPOption(DHCP_OPT12_ENABLED))
                {
                    data[0] |= DCMI_OPTION_12_MASK;
                }
                break;
            // Get below values from Systemd-networkd source code
            case dcmi::DCMIConfigParameters::DHCPTiming1:
                data[0] = DHCP_TIMING1;
                break;
            case dcmi::DCMIConfigParameters::DHCPTiming2:
                data[0] = DHCP_TIMING2_LOWER;
                data.push_back(DHCP_TIMING2_UPPER);
                break;
            case dcmi::DCMIConfigParameters::DHCPTiming3:
                data[0] = DHCP_TIMING3_LOWER;
                data.push_back(DHCP_TIMING3_UPPER);
                break;
            default:
                return ipmi::responseInvalidCommand();
        }
    }
    catch (const std::exception& e)
    {
        log<level::ERR>(e.what());
        return ipmi::responseUnspecifiedError();
    }
    major = DCMI_SPEC_MAJOR_VERSION;
    minor = DCMI_SPEC_MINOR_VERSION;
    paramRevision = DCMI_CONFIG_PARAMETER_REVISION;

    return ipmi::responseSuccess(major, minor, paramRevision, data);
}

ipmi::RspType<uint16_t, //!< Current power in watts
              uint16_t, //!< Minimum power over sampling duration
              uint16_t, //!< Maximum power over sampling duration
              uint16_t, //!< Average power over sampling duration
              uint32_t, //!< IPMI specification based time stamp
              uint32_t, //!< Statistics reporting time
              uint8_t   //!< Power Reading State
              >
    getPowerReading(ipmi::Context::ptr ctx, uint8_t mode, uint8_t modeAttribute,
                    uint8_t reserved)
{
    uint32_t timeStamp = 0;
    uint32_t timeFrame = 0;
    uint8_t powerReadingState = 0;

    if (!dcmi::isDCMIPowerMgmtSupported())
    {
        log<level::ERR>("DCMI Power management is unsupported!");
        return ipmi::responseInvalidCommand();
    }

    int64_t power = 0;
    try
    {
        power = getPwrReading(ctx);
    }
    catch (const InternalFailure& e)
    {
        log<level::ERR>("Error in reading power sensor value",
                        entry("INTERFACE=%s", SENSOR_VALUE_INTF),
                        entry("PROPERTY=%s", SENSOR_VALUE_PROP));
        return ipmi::responseUnspecifiedError();
    }

    // TODO: openbmc/openbmc#2819
    // Minimum, Maximum, Average power, TimeFrame, TimeStamp,
    // PowerReadingState readings need to be populated
    // after Telemetry changes.
    uint16_t totalPower = static_cast<uint16_t>(power);

    uint16_t currentPower = totalPower;
    uint16_t minimumPower = totalPower;
    uint16_t maximumPower = totalPower;
    uint16_t averagePower = totalPower;

    return ipmi::responseSuccess(currentPower, minimumPower, maximumPower,
                                 averagePower, timeStamp, timeFrame,
                                 powerReadingState);
}

namespace dcmi
{
namespace sensor_info
{

Response createFromJson(const Json& config)
{
    Response response{};
    uint16_t recordId = config.value("record_id", 0);
    response.recordIdLsb = recordId & 0xFF;
    response.recordIdMsb = (recordId >> 8) & 0xFF;
    return response;
}

std::tuple<ResponseList, NumInstances>
    read(const std::string& type, uint8_t instance, const Json& config)
{
    ResponseList responses{};

    if (!instance)
    {
        log<level::ERR>("Expected non-zero instance");
        elog<InternalFailure>();
    }

    static const std::vector<Json> empty{};
    std::vector<Json> readings = config.value(type, empty);
    size_t numInstances = readings.size();
    for (const auto& reading : readings)
    {
        uint8_t instanceNum = reading.value("instance", 0);
        // Not the instance we're interested in
        if (instanceNum != instance)
        {
            continue;
        }

        Response response = createFromJson(reading);
        responses.push_back(response);

        // Found the instance we're interested in
        break;
    }

    if (responses.empty())
    {
        return std::make_tuple(responses, 0);
    }

    if (numInstances > maxInstances)
    {
        log<level::DEBUG>("Trimming IPMI num instances",
                          entry("NUM_INSTANCES=%d", numInstances));
        numInstances = maxInstances;
    }

    return std::make_tuple(responses, numInstances);
}

std::tuple<ResponseList, NumInstances>
    readAll(const std::string& type, uint8_t instanceStart, const Json& config)
{
    ResponseList responses{};

    size_t numInstances = 0;
    static const std::vector<Json> empty{};
    std::vector<Json> readings = config.value(type, empty);
    numInstances = readings.size();
    for (const auto& reading : readings)
    {
        try
        {
            // Max of 8 records
            if (responses.size() == maxRecords)
            {
                break;
            }

            uint8_t instanceNum = reading.value("instance", 0);
            // Not in the instance range we're interested in
            if (instanceNum < instanceStart)
            {
                continue;
            }

            Response response = createFromJson(reading);
            responses.push_back(response);
        }
        catch (const std::exception& e)
        {
            log<level::DEBUG>(e.what());
            continue;
        }
    }

    if (numInstances > maxInstances)
    {
        log<level::DEBUG>("Trimming IPMI num instances",
                          entry("NUM_INSTANCES=%d", numInstances));
        numInstances = maxInstances;
    }
    return std::make_tuple(responses, numInstances);
}

} // namespace sensor_info
} // namespace dcmi

ipmi::RspType<uint8_t,              // No of instances for requested id
              uint8_t,              // No of record ids in the response
              std::vector<uint16_t> // SDR Record ID corresponding to the Entity
                                    // IDs
              >
    getSensorInfo(ipmi::Context::ptr ctx, int8_t sensorType, uint8_t entityId,
                  uint8_t entityInstance, uint8_t instanceStart)
{
    auto it = dcmi::entityIdToName.find(entityId);
    if (it == dcmi::entityIdToName.end())
    {
        log<level::ERR>("Unknown Entity ID", entry("ENTITY_ID=%d", entityId));
        return ipmi::responseInvalidFieldRequest();
    }

    if (sensorType != dcmi::temperatureSensorType)
    {
        log<level::ERR>("Invalid sensor type",
                        entry("SENSOR_TYPE=%d", sensorType));
        return ipmi::responseInvalidFieldRequest();
    }

    dcmi::sensor_info::ResponseList sensors{};
    static dcmi::Json config{};
    static bool parsed = false;
    uint8_t numRecords = 0, numInstances = 0;

    try
    {
        if (!parsed)
        {
            config = dcmi::parseJSONConfig(dcmi::gDCMISensorsConfig);
            parsed = true;
        }

        if (!entityInstance)
        {
            // Read all instances
            std::tie(sensors, numInstances) =
                dcmi::sensor_info::readAll(it->second, instanceStart, config);
        }
        else
        {
            // Read one instance
            std::tie(sensors, numInstances) =
                dcmi::sensor_info::read(it->second, entityInstance, config);
        }
        numRecords = sensors.size();
    }
    catch (const InternalFailure& e)
    {
        return ipmi::responseUnspecifiedError();
    }

    if (!numInstances)
    {
        return ipmi::responseUnspecifiedError();
    }
    std::vector<uint16_t> sensorRec{};
    for (size_t i = 0; i < numRecords; i++)
    {
        uint16_t senRec = sensors[i].recordIdMsb & 0xFF;
        senRec = (senRec << 8) | sensors[i].recordIdLsb;
        sensorRec.push_back(senRec);
    }

    return ipmi::responseSuccess(numInstances, numRecords, sensorRec);
}

void register_netfn_dcmi_functions()
{
    // <Get Power Limit>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdGetPowerLimit,
                               ipmi::Privilege::User, getPowerLimit);

    // <Set Power Limit>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdSetPowerLimit,
                               ipmi::Privilege::Operator, setPowerLimit);

    // <Activate/Deactivate Power Limit>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdActDeactivatePwrLimit,
                               ipmi::Privilege::Operator, applyPowerLimit);

    // <Get Asset Tag>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdGetAssetTag,
                               ipmi::Privilege::User, getAssetTag);

    // <Set Asset Tag>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdSetAssetTag,
                               ipmi::Privilege::Operator, setAssetTag);

    // <Get Management Controller Identifier String>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdGetMgmtCntlrIdString,
                               ipmi::Privilege::User, getMgmntCtrlIdStr);

    // <Set Management Controller Identifier String>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdSetMgmtCntlrIdString,
                               ipmi::Privilege::Admin, setMgmntCtrlIdStr);

    // <Get DCMI capabilities>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdGetDcmiCapabilitiesInfo,
                               ipmi::Privilege::User, getDCMICapabilities);

    // <Get Temperature Readings>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdGetTemperatureReadings,
                               ipmi::Privilege::User, getTempReadings);

    // <Get Power Reading>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdGetPowerReading,
                               ipmi::Privilege::User, getPowerReading);

#ifndef FEATURE_DYNAMIC_SENSORS
    // <Get Sensor Info>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdGetDcmiSensorInfo,
                               ipmi::Privilege::User, getSensorInfo);
#endif

    // <Get DCMI Configuration Parameters>
    ipmi::registerGroupHandler(ipmi::prioOpenBmcBase, ipmi::groupDCMI,
                               ipmi::dcmi::cmdGetDcmiConfigParameters,
                               ipmi::Privilege::User, getDCMIConfParams);

    // <Set DCMI Configuration Parameters>
    ipmi_register_callback(NETFUN_GRPEXT, dcmi::Commands::SET_CONF_PARAMS, NULL,
                           setDCMIConfParams, PRIVILEGE_ADMIN);

    return;
}
// 956379
