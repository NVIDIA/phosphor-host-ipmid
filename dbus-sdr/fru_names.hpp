// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>

namespace ipmi::sensor
{
using FruLocation = std::pair<uint16_t, uint8_t>;
struct FruDirectory
{
    uint64_t epoch = 0;
    std::map<uint8_t, FruLocation> bindings;
};

// Acquisition versions belong to the provider and object being read. Sensor
// discovery elsewhere must not reject a successful FRU-name response.
class FruNames
{
  public:
    using Source = std::pair<std::string, std::string>;
    struct Token
    {
        Source source;
        uint64_t provider;
        uint64_t object;
    };

    Token begin(const Source& source)
    {
        return {source, providers[source.first], objects[source.second]};
    }

    void changed(const std::string& path)
    {
        ++objects[path];
    }

    bool providerChanged(const std::string& provider)
    {
        auto found = providers.find(provider);
        if (found == providers.end())
        {
            return false;
        }
        ++found->second;
        std::erase_if(entries, [&provider](const auto& item) {
            return item.first.first == provider;
        });
        return true;
    }

    void removed(const std::string& path)
    {
        changed(path);
        std::erase_if(entries, [this, &path](const auto& item) {
            if (item.first.second != path)
            {
                return false;
            }
            retiredLocations.insert(item.second.location);
            return true;
        });
    }

    std::optional<bool> commit(const Token& token, FruLocation location,
                               std::string name)
    {
        if (providers[token.source.first] != token.provider ||
            objects[token.source.second] != token.object)
        {
            return std::nullopt;
        }
        ++objects[token.source.second];
        name.resize(std::min(name.size(), size_t(16)));
        auto found = entries.find(token.source);
        if (found != entries.end() && found->second.location == location &&
            found->second.name == name)
        {
            return false;
        }
        if (found != entries.end() && found->second.location != location)
        {
            retiredLocations.insert(found->second.location);
        }
        entries.insert_or_assign(
            token.source, Entry{location, std::move(name)});
        return true;
    }

    std::map<FruLocation, std::string> resolved() const
    {
        std::map<FruLocation, std::string> result;
        std::map<FruLocation, bool> conflicts;
        for (const auto& [source, entry] : entries)
        {
            auto [it, inserted] = result.emplace(entry.location, entry.name);
            if (!inserted && it->second != entry.name)
            {
                conflicts[entry.location] = true;
            }
        }
        for (const auto& [location, conflict] : conflicts)
        {
            result.erase(location);
        }
        for (const auto& location : retiredLocations)
        {
            if (!conflicts.contains(location))
            {
                result.try_emplace(location, "");
            }
        }
        return result;
    }

  private:
    struct Entry
    {
        FruLocation location;
        std::string name;
    };
    std::map<Source, Entry> entries;
    std::map<std::string, uint64_t> providers;
    std::map<std::string, uint64_t> objects;
    std::set<FruLocation> retiredLocations;
};
} // namespace ipmi::sensor
