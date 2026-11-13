// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#pragma once

#include <cstddef>
#include <optional>

namespace ipmi::storage
{
// Event-loop confined. A refresh reply may commit only if neither the provider
// nor its object directory changed while the D-Bus request yielded.
class FruDiscoveryState
{
  public:
    size_t revision() const
    {
        return version;
    }

    bool ready() const
    {
        return initialized;
    }

    void changed()
    {
        ++version;
    }

    void invalidate()
    {
        changed();
        initialized = false;
    }

    std::optional<size_t> beginRefresh()
    {
        if (running)
        {
            return std::nullopt;
        }
        running = true;
        return version;
    }

    // The background SDR builder owns bounded retry/backoff and re-arms after
    // a rejected refresh. This helper never schedules work or waits for a host.
    bool finishRefresh(size_t revision, bool success)
    {
        running = false;
        if (!success || revision != version)
        {
            return false;
        }
        initialized = true;
        return true;
    }

  private:
    size_t version = 0;
    bool initialized = false;
    bool running = false;
};
} // namespace ipmi::storage
