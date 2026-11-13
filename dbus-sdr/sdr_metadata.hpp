// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#pragma once

#include <stdexcept>
#include <string_view>

namespace ipmi::sensor
{
// Only acquisition failures are retryable. Malformed serialized records and
// unexpected exceptions must not be hidden in an endless background retry.
class SdrBuildError : public std::runtime_error
{
  public:
    using std::runtime_error::runtime_error;
};

inline bool isSdrDescriptorProperty(std::string_view interface,
                                    std::string_view property)
{
    if (interface == "xyz.openbmc_project.Sensor.Value")
    {
        return property == "MaxValue" || property == "MinValue" ||
               property == "Unit" || property == "Scale";
    }
    if (interface == "xyz.openbmc_project.Sensor.Threshold.Warning")
    {
        return property == "WarningHigh" || property == "WarningLow";
    }
    if (interface == "xyz.openbmc_project.Sensor.Threshold.Critical")
    {
        return property == "CriticalHigh" || property == "CriticalLow";
    }
    return (interface == "xyz.openbmc_project.Sensor.ValueMutability" &&
            property == "Mutable") ||
           (interface == "xyz.openbmc_project.Association.Definitions" &&
            property == "Associations") ||
           (interface == "xyz.openbmc_project.State.Decorator.Availability" &&
            property == "Available");
}
} // namespace ipmi::sensor

namespace ipmi
{
// Clear descriptor acquisitions after a relevant repository input change.
void invalidateSdrMetadata();
} // namespace ipmi
