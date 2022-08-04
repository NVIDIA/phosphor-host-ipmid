#pragma once
#include "commonselutility.hpp"

#include <ipmid/types.hpp>
#include <sdbusplus/server.hpp>

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace ipmi
{

namespace sel
{

/** @brief Convert logging entry to SEL
 *
 *  @param[in] objPath - DBUS object path of the logging entry.
 *
 *  @return On success return the response of Get SEL entry command.
 */
GetSELEntryResponse convertLogEntrytoSEL(const std::string& objPath);

namespace internal
{

/** @brief Convert logging entry to SEL event record
 *
 *  @param[in] objPath - DBUS object path of the logging entry.
 *  @param[in] iter - Iterator to the sensor data corresponding to the logging
 *                    entry
 *
 *  @return On success return the SEL event record, throw an exception in case
 *          of failure.
 */
GetSELEntryResponse
    prepareSELEntry(const std::string& objPath,
                    ipmi::sensor::InvObjectIDMap::const_iterator iter);

} // namespace internal
} // namespace sel

} // namespace ipmi
