// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

namespace ipmi::sensor
{

// Retain allocations until the IPMI provider daemon restarts. Removed objects
// keep their identity: an ID cached by a client must never select a different
// object. The finite pool bounds memory and makes exhaustion an explicit
// failure.
template <typename Key, typename Id>
class StableIds
{
  public:
    explicit StableIds(std::vector<Id> pool) : pool(std::move(pool)) {}

    std::optional<Id> allocate(const Key& key,
                               std::optional<Id> preferred = std::nullopt)
    {
        if (auto existing = ids.find(key); existing != ids.end())
        {
            return existing->second;
        }
        auto start = preferred ? std::find(pool.begin(), pool.end(), *preferred)
                               : pool.begin();
        if (start == pool.end())
        {
            start = pool.begin();
        }
        const auto offset = static_cast<size_t>(start - pool.begin());
        for (size_t index = 0; index < pool.size(); ++index)
        {
            const auto id = pool[(offset + index) % pool.size()];
            if (keys.contains(id))
            {
                continue;
            }
            keys.emplace(id, key);
            ids.emplace(key, id);
            return id;
        }
        return std::nullopt;
    }

  private:
    std::vector<Id> pool;
    std::map<Key, Id> ids;
    std::map<Id, Key> keys;
};

} // namespace ipmi::sensor
