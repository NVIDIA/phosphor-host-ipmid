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

#include <sys/inotify.h>
#include <sys/stat.h>
#include <unistd.h>

#include <boost/asio/posix/stream_descriptor.hpp>
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
    ~AllowlistFilter();
    AllowlistFilter(const AllowlistFilter&) = delete;
    AllowlistFilter(AllowlistFilter&&) = delete;
    AllowlistFilter& operator=(const AllowlistFilter&) = delete;
    AllowlistFilter& operator=(AllowlistFilter&&) = delete;

  private:
    void postInit();
    void startPostCompleteFileWatch();
    void armInotifyRead();
    void cacheRestrictedMode();
    void handleRestrictedModeChange(sdbusplus::message_t& m);
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
    // inotify-based file watch (persistent)
    int inotifyFd{-1};
    int inotifyWd{-1};
    std::unique_ptr<boost::asio::posix::stream_descriptor> inotifyStream;
    std::array<char, 4096> inotifyBuf{};

    static constexpr const char restrictionModeIntf[] =
        "xyz.openbmc_project.Control.Security.RestrictionMode";
    static constexpr const char* restrictionModePath =
        "/xyz/openbmc_project/control/host0/restriction_mode";
    static constexpr const char* postCompleteDir = "/run/bmc-state";
    static constexpr const char* postCompleteFile =
        "/run/bmc-state/CPU_BOOT_DONE-I";
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

AllowlistFilter::~AllowlistFilter()
{
    // Cancel asio descriptor first to stop any pending reads
    if (inotifyStream)
    {
        boost::system::error_code ec;
        inotifyStream->cancel(ec);
        inotifyStream->release();
        inotifyStream.reset();
    }

    // Remove watch if installed
    if (inotifyWd >= 0 && inotifyFd >= 0)
    {
        inotify_rm_watch(inotifyFd, inotifyWd);
        inotifyWd = -1;
    }

    // Close fd if open
    if (inotifyFd >= 0)
    {
        close(inotifyFd);
        inotifyFd = -1;
    }
}

/**
 * @brief Cache the restricted mode.
 *
 * Reads RestrictionMode from D-Bus
 * (xyz.openbmc_project.Control.Security.RestrictionMode) and updates internal
 * state.
 */
void AllowlistFilter::cacheRestrictedMode()
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
 * @brief Start watching the POST complete file using inotify
 *
 * Creates an inotify instance, adds a watch for the POST complete file, and
 * starts the asynchronous read operation.
 */
void AllowlistFilter::startPostCompleteFileWatch()
{
    // Create inotify (non-blocking)
    inotifyFd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (inotifyFd < 0)
    {
        lg2::error("inotify_init1 failed");
        return;
    }

    // Watch directory for create/move-in/delete/move-out
    inotifyWd =
        inotify_add_watch(inotifyFd, postCompleteDir,
                          IN_CREATE | IN_MOVED_TO | IN_DELETE | IN_MOVED_FROM);
    if (inotifyWd < 0)
    {
        lg2::error("inotify_add_watch failed for {DIR}", "DIR",
                   postCompleteDir);
        close(inotifyFd);
        inotifyFd = -1;
        return;
    }

    // Race-window close: check once after installing watch
    struct stat st{};
    if (stat(postCompleteFile, &st) == 0)
    {
        if (!postCompleted)
        {
            postCompleted = true;
            lg2::info("Updated to POST Complete (created during watch setup)");
        }
    }
    else
    {
        if (postCompleted)
        {
            postCompleted = false;
            lg2::info("Updated to !POST Complete (file absent at start)");
        }
    }

    inotifyStream = std::make_unique<boost::asio::posix::stream_descriptor>(
        bus->get_io_context(), inotifyFd);
    armInotifyRead();
}

/**
 * @brief Arm the inotify read operation.
 *
 * Asynchronously reads any inotify events and updates postCompleted
 * by checking the file existence with stat().
 */
void AllowlistFilter::armInotifyRead()
{
    if (!inotifyStream)
    {
        return;
    }

    inotifyStream->async_read_some(
        boost::asio::buffer(inotifyBuf),
        [this](const boost::system::error_code& ec, std::size_t /*bytes*/) {
        if (ec)
        {
            if (ec != boost::asio::error::operation_aborted)
            {
                lg2::error("inotify async_read error");
            }
            return;
        }

        // check if the POST complete file exists
        struct stat st{};
        bool exists = (stat(postCompleteFile, &st) == 0);
        if (exists != postCompleted)
        {
            postCompleted = exists;
            lg2::info(postCompleted ? "Updated to POST Complete"
                                    : "Updated to !POST Complete");
        }

        // Re-arm for the next event
        armInotifyRead();
    });
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

    modeChangeMatch = std::make_unique<sdbusplus::bus::match_t>(
        *bus, filterStrModeChange,
        [this](sdbusplus::message_t& m) { handleRestrictedModeChange(m); });

    modeIntfAddedMatch = std::make_unique<sdbusplus::bus::match_t>(
        *bus, filterStrModeIntfAdd,
        [this](sdbusplus::message_t& m) { handleRestrictedModeChange(m); });

    // Initialize restricted mode
    cacheRestrictedMode();

    // Start watching the POST complete file using inotify
    startPostCompleteFileWatch();
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

    // Allow all commands before POST completes
    if (!postCompleted)
    {
        return ipmi::ccSuccess;
    }

    bool allowed = true;
    switch (restrictionMode)
    {
        case restrictionModeNone: // None mode
            allowed = true;
            break;
        case restrictionModeAllowlist: // Allowlist mode
            allowed = isInAllowlist(request);
            break;
        case restrictionModeRestricted: // ProvisionedHostAllowlist mode
            allowed = isInAllowlistWithChannel(request);
            break;
        case restrictionModeDenyAll: // ProvisionedHostDisabled mode
        default:                     // mode not supported
            allowed = false;
            break;
    }

    if (!allowed)
    {
        lg2::error(
            "Blocked IPMI cmd netfn={NETFN} cmd={CMD} ch={CH} reason=not-in-allowlist",
            "NETFN", request->ctx->netFn, "CMD", request->ctx->cmd, "CH",
            request->ctx->channel);
        return ipmi::ccInsufficientPrivilege;
    }

    return ipmi::ccSuccess;
}

// instantiate the AllowlistFilter when this shared object is loaded
AllowlistFilter allowlistFilter;

} // namespace

} // namespace ipmi
