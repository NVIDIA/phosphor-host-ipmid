// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <optional>

namespace ipmi::sensor
{
// Bound acquisition attempts for one input revision. A relevant invalidation
// starts a new budget; malformed input does not justify a periodic retry loop.
class SdrBuildRecovery
{
  public:
    explicit SdrBuildRecovery(size_t limit = 5) :
        limit(std::max(size_t{1}, limit))
    {}

    std::optional<std::chrono::seconds> failed(bool retryable)
    {
        if (stopped)
        {
            return std::nullopt;
        }
        ++failures;
        stopped = !retryable || failures >= limit;
        if (stopped)
        {
            return std::nullopt;
        }
        constexpr auto maximumDelay = std::chrono::seconds(30);
        // 2^5 seconds reaches the cap without an unbounded shift.
        constexpr size_t maximumBackoffExponent = 5;
        const auto delay = std::chrono::seconds(
            size_t{1} << std::min(failures - 1, maximumBackoffExponent));
        return std::min(delay, maximumDelay);
    }

    void reset()
    {
        failures = 0;
        stopped = false;
    }

    bool degraded() const
    {
        return stopped;
    }

  private:
    size_t limit;
    size_t failures = 0;
    bool stopped = false;
};
} // namespace ipmi::sensor
