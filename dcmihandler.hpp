#pragma once

#include "nlohmann/json.hpp"

#include <map>
#include <ipmid/utils.hpp>
#include <sdbusplus/bus.hpp>

#include <map>
#include <string>
#include <vector>

namespace dcmi
{

static constexpr auto propIntf = "org.freedesktop.DBus.Properties";
static constexpr auto assetTagIntf =
    "xyz.openbmc_project.Inventory.Decorator.AssetTag";
static constexpr auto assetTagProp = "AssetTag";
static constexpr auto networkServiceName = "xyz.openbmc_project.Network";
static constexpr auto networkConfigObj = "/xyz/openbmc_project/network/config";
static constexpr auto networkConfigIntf =
    "xyz.openbmc_project.Network.SystemConfiguration";
static constexpr auto hostNameProp = "HostName";
static constexpr auto temperatureSensorType = 0x01;
static constexpr size_t maxInstances = 255;
static constexpr uint8_t maxRecords = 8;
static constexpr auto gDCMISensorsConfig =
    "/usr/share/ipmi-providers/dcmi_sensors.json";
static constexpr auto ethernetIntf =
    "xyz.openbmc_project.Network.EthernetInterface";
static constexpr auto ethernetDefaultChannelNum = 0x1;
static constexpr auto networkRoot = "/xyz/openbmc_project/network";
static constexpr auto dhcpObj = "/xyz/openbmc_project/network/dhcp";
static constexpr auto dhcpIntf =
    "xyz.openbmc_project.Network.DHCPConfiguration";
static constexpr auto systemBusName = "org.freedesktop.systemd1";
static constexpr auto systemPath = "/org/freedesktop/systemd1";
static constexpr auto systemIntf = "org.freedesktop.systemd1.Manager";
static constexpr auto gDCMICapabilitiesConfig =
    "/usr/share/ipmi-providers/dcmi_cap.json";
static constexpr auto gDCMIPowerMgmtCapability = "PowerManagement";
static constexpr auto gDCMIPowerMgmtSupported = 0x1;
static constexpr auto gMaxSELEntriesMask = 0xFFF;
static constexpr auto gByteBitSize = 8;
constexpr auto networkdService = "systemd-networkd.service";

namespace assettag
{

using ObjectPath = std::string;
using Service = std::string;
using Interfaces = std::vector<std::string>;
using ObjectTree = std::map<ObjectPath, std::map<Service, Interfaces>>;

} // namespace assettag

namespace temp_readings
{
static constexpr auto maxDataSets = 8;
static constexpr auto maxTemp = 127; // degrees C

/** @struct Response
 *
 *  DCMI payload for Get Temperature Readings response
 */
struct Response
{
#if BYTE_ORDER == LITTLE_ENDIAN
    uint8_t temperature : 7; //!< Temperature reading in Celsius
    uint8_t sign : 1;        //!< Sign bit
#endif
#if BYTE_ORDER == BIG_ENDIAN
    uint8_t sign : 1;        //!< Sign bit
    uint8_t temperature : 7; //!< Temperature reading in Celsius
#endif
    uint8_t instance; //!< Entity instance number
} __attribute__((packed));

/** @brief Restart the systemd unit
*  @param[in] unit - systemd unit name which needs to be
*                    restarted.
*/
void restartSystemdUnit(const std::string& unit);


using ResponseList = std::vector<Response>;
using Value = uint8_t;
using Sign = bool;
using Temperature = std::tuple<Value, Sign>;
} // namespace temp_readings

namespace sensor_info
{
static constexpr auto maxRecords = 8;

/** @struct Response
 *
 *  DCMI payload for Get Sensor Info response
 */
struct Response
{
    uint8_t recordIdLsb; //!< SDR record id LS byte
    uint8_t recordIdMsb; //!< SDR record id MS byte
} __attribute__((packed));

using ResponseList = std::vector<Response>;
} // namespace sensor_info

static constexpr auto groupExtId = 0xDC;

static constexpr auto assetTagMaxOffset = 62;
static constexpr auto assetTagMaxSize = 63;
static constexpr auto maxBytes = 16;
static constexpr size_t maxCtrlIdStrLen = 63;

/** @brief Check whether DCMI power management is supported
 *         in the DCMI Capabilities config file.
 *
 *  @return True if DCMI power management is supported
 */
bool isDCMIPowerMgmtSupported();

/** @brief Read the object tree to fetch the object path that implemented the
 *         Asset tag interface.
 *
 *  @param[in] Context - ctx
 *  @param[in,out] objectTree - object tree
 *
 *  @return On success return the object tree with the object path that
 *          implemented the AssetTag interface.
 */
void readAssetTagObjectTree(ipmi::Context::ptr& ctx,
                            dcmi::assettag::ObjectTree& objectTree);

/** @brief Read the asset tag of the server
 *
 *  @param[in] Context - ctx
 *  @return On success return the asset tag.
 */
std::string readAssetTag(ipmi::Context::ptr& ctx);

/** @brief Write the asset tag to the asset tag DBUS property
 *
 *  @param[in] Context - ctx
 *  @param[in] assetTag - Asset Tag to be written to the property.
 */
void writeAssetTag(ipmi::Context::ptr& ctx, const std::string& assetTag);

/** @brief Read the current power cap value
 *
 *  @param[in] Context - ctx
 *
 *  @return On success return the power cap value.
 */
uint32_t getPcap(ipmi::Context::ptr& ctx);

/** @brief Check if the power capping is enabled
 *
 *  @param[in] Context - ctx
 *
 *  @return true if the powerCap is enabled and false if the powercap
 *          is disabled.
 */
bool getPcapEnabled(ipmi::Context::ptr& ctx);

/** @brief Set the power cap value
 *
 *  @param[in] Context - ctx
 *  @param[in] powerCap - power cap value
 */
void setPcap(ipmi::Context::ptr& ctx, uint32_t powerCap);

/** @brief Enable or disable the power capping
 *
 *  @param[in] Context - ctx
 *  @param[in] enabled - enable/disable
 */
void setPcapEnable(ipmi::Context::ptr& ctx, bool enabled);

/** @enum DCMICapParameters
 *
 * DCMI Capability parameters
 */
enum class DCMICapParameters
{
    SUPPORTED_DCMI_CAPS = 0x01,             //!< Supported DCMI Capabilities
    MANDATORY_PLAT_ATTRIBUTES = 0x02,       //!< Mandatory Platform Attributes
    OPTIONAL_PLAT_ATTRIBUTES = 0x03,        //!< Optional Platform Attributes
    MANAGEABILITY_ACCESS_ATTRIBUTES = 0x04, //!< Manageability Access Attributes
};

/** @struct DCMICap
 *
 *  DCMI capabilities protocol info.
 */
struct DCMICap
{
    std::string name;     //!< Name of DCMI capability.
    uint8_t bytePosition; //!< Starting byte number from DCMI spec.
    uint8_t position;     //!< bit position from the DCMI spec.
    uint8_t length;       //!< Length of the value from DCMI spec.
};

using DCMICapList = std::vector<DCMICap>;

/** @struct DCMICapEntry
 *
 *  DCMI capabilities list and size for each parameter.
 */
struct DCMICapEntry
{
    uint8_t size;        //!< Size of capability array in bytes.
    DCMICapList capList; //!< List of capabilities for a parameter.
};

using DCMICaps = std::map<DCMICapParameters, DCMICapEntry>;

/** @brief Parse out JSON config file.
 *
 *  @param[in] configFile - JSON config file name
 *
 *  @return A json object
 */
Json parseJSONConfig(const std::string& configFile);

namespace temp_readings
{
/** @brief Read temperature from a d-bus object, scale it as per dcmi
 *         get temperature reading requirements.
 *
 *  @param[in] Context - ctx
 *  @param[in] dbusService - the D-Bus service
 *  @param[in] dbusPath - the D-Bus path
 *
 *  @return A temperature reading
 */
Temperature readTemp(ipmi::Context::ptr& ctx, const std::string& dbusService,
                     const std::string& dbusPath);

/** @brief Read temperatures and fill up DCMI response for the Get
 *         Temperature Readings command. This looks at a specific
 *         instance.
 *
 *  @param[in] Context - ctx
 *  @param[in] type - one of "inlet", "cpu", "baseboard"
 *  @param[in] instance - A non-zero Entity instance number
 *
 *  @return A tuple, containing a temperature reading and the
 *          number of instances.
 */
std::tuple<ResponseList, NumInstances>
    read(ipmi::Context::ptr& ctx, const std::string& type, uint8_t instance);

/** @brief Read temperatures and fill up DCMI response for the Get
 *         Temperature Readings command. This looks at a range of
 *         instances.
 *
 *  @param[in] Context - ctx
 *  @param[in] type - one of "inlet", "cpu", "baseboard"
 *  @param[in] instanceStart - Entity instance start index
 *
 *  @return A tuple, containing a list of temperature readings and the
 *          number of instances.
 */
std::tuple<ResponseList, NumInstances> readAll(ipmi::Context::ptr& ctx,
                                               const std::string& type,
                                               uint8_t instanceStart);
} // namespace temp_readings

namespace sensor_info
{
/** @brief Create response from JSON config.
 *
 *  @param[in] config - JSON config info about DCMI sensors
 *
 *  @return Sensor info response
 */
Response createFromJson(const Json& config);

/** @brief Read sensor info and fill up DCMI response for the Get
 *         Sensor Info command. This looks at a specific
 *         instance.
 *
 *  @param[in] type - one of "inlet", "cpu", "baseboard"
 *  @param[in] instance - A non-zero Entity instance number
 *  @param[in] config - JSON config info about DCMI sensors
 *
 *  @return A tuple, containing a sensor info response and
 *          number of instances.
 */
std::tuple<ResponseList, NumInstances>
    read(const std::string& type, uint8_t instance, const Json& config);

/** @brief Read sensor info and fill up DCMI response for the Get
 *         Sensor Info command. This looks at a range of
 *         instances.
 *
 *  @param[in] type - one of "inlet", "cpu", "baseboard"
 *  @param[in] instanceStart - Entity instance start index
 *  @param[in] config - JSON config info about DCMI sensors
 *
 *  @return A tuple, containing a list of sensor info responses and the
 *          number of instances.
 */
std::tuple<ResponseList, NumInstances>
    readAll(const std::string& type, uint8_t instanceStart, const Json& config);
} // namespace sensor_info

/** @brief Read power reading from power reading sensor object
 *
 *  @param[in] bus - dbus connection
 *
 *  @return total power reading
 */
int64_t getPowerReading(sdbusplus::bus::bus& bus);

/**
 *  @brief Parameters for DCMI Configuration Parameters
 */
enum class DCMIConfigParameters : uint8_t
{
    ActivateDHCP = 1,
    DiscoveryConfig,
    DHCPTiming1,
    DHCPTiming2,
    DHCPTiming3,
};
} // namespace dcmi
