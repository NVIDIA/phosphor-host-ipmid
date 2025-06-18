/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES.
 * All rights reserved. SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ipmiallowlist.hpp>
#include <ipmid/api.hpp>
#include <ipmid/utils.hpp>
#include <phosphor-logging/lg2.hpp>
#include <settings.hpp>
#include <xyz/openbmc_project/Control/Security/RestrictionMode/server.hpp>

#include <algorithm>
#include <array>

using namespace sdbusplus::xyz::openbmc_project::Control::Security::server;

namespace ipmi
{

// put the filter provider in an unnamed namespace
namespace
{

/** @class AllowlistFilter
 *
 * Class that implements an IPMI message filter based
 * on incoming interface and a restriction mode setting
 */
class AllowlistFilter
{
  public:
    AllowlistFilter();
    ~AllowlistFilter() = default;
    AllowlistFilter(const AllowlistFilter&) = delete;
    AllowlistFilter(AllowlistFilter&&) = delete;
    AllowlistFilter& operator=(const AllowlistFilter&) = delete;
    AllowlistFilter& operator=(AllowlistFilter&&) = delete;

  private:
    void postInit();
    void cacheRestrictedAndPostCompleteMode();
    void handleRestrictedModeChange(sdbusplus::message_t& m);
    void handlePostCompleteChange(sdbusplus::message_t& m);
    void updatePostComplete(const std::string& value);
    void updateRestrictionMode(const std::string& value);
    ipmi::Cc filterMessage(ipmi::message::Request::ptr request);

    static constexpr RestrictionMode::Modes restrictionModeNone =
        RestrictionMode::Modes::None;
    static constexpr RestrictionMode::Modes restrictionModeAllowlist =
        RestrictionMode::Modes::Allowlist;
    static constexpr RestrictionMode::Modes restrictionModeRestricted =
        RestrictionMode::Modes::ProvisionedHostAllowlist;
    static constexpr RestrictionMode::Modes restrictionModeDenyAll =
        RestrictionMode::Modes::ProvisionedHostDisabled;

    RestrictionMode::Modes restrictionMode = restrictionModeNone;
    bool postCompleted = true;
    int channelSMM = -1;
    std::shared_ptr<sdbusplus::asio::connection> bus;
    std::unique_ptr<sdbusplus::bus::match_t> modeChangeMatch;
    std::unique_ptr<sdbusplus::bus::match_t> modeIntfAddedMatch;
    std::unique_ptr<sdbusplus::bus::match_t> postCompleteMatch;
    std::unique_ptr<sdbusplus::bus::match_t> postCompleteIntfAddedMatch;

    static constexpr const char restrictionModeIntf[] =
        "xyz.openbmc_project.Control.Security.RestrictionMode";
    static constexpr const char* systemOsStatusIntf =
        "xyz.openbmc_project.State.Boot.Progress";
    static constexpr const char* restrictionModePath =
        "/xyz/openbmc_project/control/host0/restriction_mode";
    static constexpr const char* systemOsStatusPath =
        "/xyz/openbmc_project/state/host0";
};

/**
 * @brief Find and return the channel number for the system interface.
 *
 * @return The channel number if found, otherwise -1.
 */
static inline int getSystemInterfaceChannel()
{
    ipmi::ChannelInfo chInfo;

    for (int channel = 0; channel < ipmi::maxIpmiChannels; channel++)
    {
        if (ipmi::getChannelInfo(channel, chInfo) != ipmi::ccSuccess)
        {
            continue;
        }

        if (static_cast<ipmi::EChannelMediumType>(chInfo.mediumType) ==
                ipmi::EChannelMediumType::systemInterface &&
            channel != ipmi::channelSystemIface)
        {
            lg2::info("System Interface channel number: {CHANNEL}", "CHANNEL",
                      channel);
            return channel;
        }
    }

    lg2::error("Unable to find System Interface Channel Info");
    return -1;
}

/**
 * @brief Constructor for the AllowlistFilter class.
 *
 * Initializes the D-Bus connection, registers the IPMI message filter,
 * and caches the restricted mode and POST complete status.
 */
AllowlistFilter::AllowlistFilter()
{
    bus = getSdBus();

    lg2::info("Loading Allowlist filter");

    ipmi::registerFilter(ipmi::prioOpenBmcBase,
                         [this](ipmi::message::Request::ptr request) {
        return filterMessage(request);
    });

    channelSMM = getSystemInterfaceChannel();
    // wait until io->run is going to fetch RestrictionMode
    post_work([this]() { postInit(); });
}

/**
 * @brief Cache the restricted mode and POST complete status.
 *
 * Retrieves the restricted mode and POST complete status from D-Bus
 * and updates the internal variables.
 */
void AllowlistFilter::cacheRestrictedAndPostCompleteMode()
{
    try
    {
        auto service = ipmi::getService(*bus, restrictionModeIntf,
                                        restrictionModePath);
        ipmi::Value v =
            ipmi::getDbusProperty(*bus, service, restrictionModePath,
                                  restrictionModeIntf, "RestrictionMode");
        auto& mode = std::get<std::string>(v);
        restrictionMode = RestrictionMode::convertModesFromString(mode);
        lg2::info("Read restriction mode: {MODE}", "MODE",
                  static_cast<int>(restrictionMode));
    }
    catch (const std::exception&)
    {
        lg2::error("Could not initialize RestrictionMode, "
                   "defaulting to RestrictionMode::None");
    }

    try
    {
        auto service = ipmi::getService(*bus, systemOsStatusIntf,
                                        systemOsStatusPath);
        ipmi::Value v = ipmi::getDbusProperty(*bus, service, systemOsStatusPath,
                                              systemOsStatusIntf,
                                              "BootProgress");
        auto& value = std::get<std::string>(v);
        updatePostComplete(value);
        lg2::info("Read POST complete value: {VALUE}", "VALUE", postCompleted);
    }
    catch (const std::exception&)
    {
        lg2::error("Error in OperatingSystemState Get");
        postCompleted = true;
    }
}

/**
 * @brief Update the restriction mode.
 *
 * Converts the string value to a RestrictionMode enum and updates the internal
 * variable.
 */
void AllowlistFilter::updateRestrictionMode(const std::string& value)
{
    restrictionMode = RestrictionMode::convertModesFromString(value);
    lg2::info("Updated restriction mode: {MODE}", "MODE",
              static_cast<int>(restrictionMode));
}

/**
 * @brief Handle changes in the restriction mode.
 *
 * Reads the signal member from the message and updates the restriction mode
 * based on the property changes.
 */
void AllowlistFilter::handleRestrictedModeChange(sdbusplus::message_t& m)
{
    std::string signal = m.get_member();
    if (signal == "PropertiesChanged")
    {
        std::string intf;
        std::vector<std::pair<std::string, ipmi::Value>> propertyList;
        m.read(intf, propertyList);
        for (const auto& property : propertyList)
        {
            if (property.first == "RestrictionMode")
            {
                updateRestrictionMode(std::get<std::string>(property.second));
            }
        }
    }
    else if (signal == "InterfacesAdded")
    {
        sdbusplus::message::object_path path;
        DbusInterfaceMap restModeObj;
        m.read(path, restModeObj);
        auto intfItr = restModeObj.find(restrictionModeIntf);
        if (intfItr == restModeObj.end())
        {
            return;
        }
        PropertyMap& propertyList = intfItr->second;
        auto itr = propertyList.find("RestrictionMode");
        if (itr == propertyList.end())
        {
            return;
        }
        updateRestrictionMode(std::get<std::string>(itr->second));
    }
}

/**
 * @brief Update the POST complete status.
 *
 * Converts the string value to a boolean and updates the internal
 * variable.
 */
void AllowlistFilter::updatePostComplete(const std::string& value)
{
    postCompleted =
        (value ==
         "xyz.openbmc_project.State.Boot.Progress.ProgressStages.OSRunning") ||
        (value ==
         "xyz.openbmc_project.State.Boot.Progress.ProgressStages.OSStart");
    lg2::info(postCompleted ? "Updated to POST Complete"
                            : "Updated to !POST Complete");
}

/**
 * @brief Handle changes in the POST complete status.
 *
 * Reads the signal member from the message and updates the POST complete status
 * based on the property changes.
 */
void AllowlistFilter::handlePostCompleteChange(sdbusplus::message_t& m)
{
    std::string signal = m.get_member();
    if (signal == "PropertiesChanged")
    {
        std::string intf;
        std::vector<std::pair<std::string, ipmi::Value>> propertyList;
        m.read(intf, propertyList);
        for (const auto& property : propertyList)
        {
            if (property.first == "OperatingSystemState")
            {
                updatePostComplete(std::get<std::string>(property.second));
            }
        }
    }
    else if (signal == "InterfacesAdded")
    {
        sdbusplus::message::object_path path;
        DbusInterfaceMap postCompleteObj;
        m.read(path, postCompleteObj);
        auto intfItr = postCompleteObj.find(systemOsStatusIntf);
        if (intfItr == postCompleteObj.end())
        {
            return;
        }
        PropertyMap& propertyList = intfItr->second;
        auto itr = propertyList.find("OperatingSystemState");
        if (itr == propertyList.end())
        {
            return;
        }
        updatePostComplete(std::get<std::string>(itr->second));
    }
}

/**
 * @brief Initialize the matches for the restriction mode and POST complete
 * status.
 *
 * Sets up the D-Bus matches for the restriction mode and POST complete status
 * signals and connects them to the respective handler functions.
 */
void AllowlistFilter::postInit()
{
    namespace rules = sdbusplus::bus::match::rules;

    const std::string filterStrModeChange =
        rules::type::signal() + rules::member("PropertiesChanged") +
        rules::interface("org.freedesktop.DBus.Properties") +
        rules::argN(0, restrictionModeIntf);

    const std::string filterStrModeIntfAdd =
        rules::interfacesAdded() + rules::argNpath(0, restrictionModePath);

    const std::string filterStrPostComplete =
        rules::type::signal() + rules::member("PropertiesChanged") +
        rules::interface("org.freedesktop.DBus.Properties") +
        rules::argN(0, systemOsStatusIntf);

    const std::string filterStrPostIntfAdd =
        rules::interfacesAdded() + rules::argNpath(0, systemOsStatusPath);

    modeChangeMatch = std::make_unique<sdbusplus::bus::match_t>(
        *bus, filterStrModeChange,
        [this](sdbusplus::message_t& m) { handleRestrictedModeChange(m); });

    modeIntfAddedMatch = std::make_unique<sdbusplus::bus::match_t>(
        *bus, filterStrModeIntfAdd,
        [this](sdbusplus::message_t& m) { handleRestrictedModeChange(m); });

    postCompleteMatch = std::make_unique<sdbusplus::bus::match_t>(
        *bus, filterStrPostComplete,
        [this](sdbusplus::message_t& m) { handlePostCompleteChange(m); });

    postCompleteIntfAddedMatch = std::make_unique<sdbusplus::bus::match_t>(
        *bus, filterStrPostIntfAdd,
        [this](sdbusplus::message_t& m) { handlePostCompleteChange(m); });

    // Initialize restricted mode
    cacheRestrictedAndPostCompleteMode();
}

/**
 * @brief Check if the request is in the allowlist.
 *
 * @param[in] request - IPMI messahe request
 * @returns true if the request is in the allowlist, false otherwise.
 */
bool isInAllowlist(const ipmi::message::Request::ptr& request)
{
    // not check channel mask
    return std::any_of(allowlist.cbegin(), allowlist.cend(),
                       [&](const netfncmd_tuple& entry) {
        return std::get<0>(entry) == request->ctx->netFn &&
               std::get<1>(entry) == request->ctx->cmd;
    });
}

/**
 * @brief Check if the request is in the allowlist with channel mask.
 *
 * @param[in] request - IPMI messahe request
 * @returns true if the request is in the allowlist, false otherwise.
 */
bool isInAllowlistWithChannel(const ipmi::message::Request::ptr& request)
{
    // check channel mask
    auto channelMask = static_cast<unsigned short>(1 << request->ctx->channel);
    return std::binary_search(
        allowlist.cbegin(), allowlist.cend(),
        std::make_tuple(request->ctx->netFn, request->ctx->cmd, channelMask),
        [](const netfncmd_tuple& first, const netfncmd_tuple& value) {
        return (std::get<2>(first) & std::get<2>(value))
                   ? first < std::make_tuple(std::get<0>(value),
                                             std::get<1>(value),
                                             std::get<2>(first))
                   : first < value;
    });
}

/**
 * @brief Filter IPMI messages with RestrictedMode
 *
 * @param[in] request - IPMI messahe request
 * @returns IPMI completion code success or error.
 */
ipmi::Cc AllowlistFilter::filterMessage(ipmi::message::Request::ptr request)
{
    // no special handling for non-system-interface channels
    if (!(request->ctx->channel == ipmi::channelSystemIface ||
          request->ctx->channel == channelSMM))
    {
        return ipmi::ccSuccess;
    }

    switch (restrictionMode)
    {
        case restrictionModeNone: // None mode
        {
            return ipmi::ccSuccess;
        }
        case restrictionModeAllowlist: // Allowlist mode
        {
            return isInAllowlist(request) ? ipmi::ccSuccess
                                          : ipmi::ccInsufficientPrivilege;
        }
        case restrictionModeRestricted: // ProvisionedHostAllowlist mode
        {
            if (!postCompleted)
            {
                // POST not completed, allow all
                return ipmi::ccSuccess;
            }
            return isInAllowlistWithChannel(request)
                       ? ipmi::ccSuccess
                       : ipmi::ccInsufficientPrivilege;
        }
        case restrictionModeDenyAll: // ProvisionedHostDisabled mode
        {
            if (postCompleted)
            {
                // POST completed, deny all
                return ipmi::ccInsufficientPrivilege;
            }
            return isInAllowlistWithChannel(request)
                       ? ipmi::ccSuccess
                       : ipmi::ccInsufficientPrivilege;
        }
        default: // mode not supported
        {
            lg2::error("RestrictionMode:{MODE}, not supported", "MODE",
                       restrictionMode);
            return ipmi::ccInsufficientPrivilege;
        }
    }

    return ipmi::ccSuccess;
}

// instantiate the AllowlistFilter when this shared object is loaded
AllowlistFilter allowlistFilter;

} // namespace

} // namespace ipmi
