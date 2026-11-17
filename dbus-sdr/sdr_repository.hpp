// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#pragma once

#include "fru_names.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <compare>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ipmi::sensor
{

struct SdrRequester
{
    int channel;
    uint32_t session;
    int address;
    int host;

    auto operator<=>(const SdrRequester&) const = default;
};

struct SdrSnapshot
{
    size_t revision = 0;
    uint32_t lastAddition = 0xffffffff;
    uint32_t lastRemoval = 0xffffffff;
    std::vector<std::vector<uint8_t>> records;
    std::map<uint16_t, std::string> sensorPaths;
    FruDirectory fruDirectory;
    std::map<uint8_t, std::string> resolvedFruNames;

    // Work only on owned bytes. Empty metadata is an explicit fallback result;
    // missing metadata is unknown and may retain a completed, bound alias.
    std::optional<size_t> refreshFruNames(
        const FruDirectory& current,
        const std::map<FruLocation, std::string>& names,
        const SdrSnapshot* previous = nullptr)
    {
        if (fruDirectory.epoch != current.epoch)
        {
            return std::nullopt;
        }
        for (const auto& [id, location] : fruDirectory.bindings)
        {
            auto found = current.bindings.find(id);
            if (found == current.bindings.end() || found->second != location)
            {
                return std::nullopt;
            }
        }
        size_t updated = 0;
        for (auto& record : records)
        {
            if (record.size() < 16 || record[3] != 0x11)
            {
                continue;
            }
            const uint8_t id = record[6];
            auto binding = fruDirectory.bindings.find(id);
            if (binding == fruDirectory.bindings.end())
            {
                return std::nullopt;
            }
            auto name = names.find(binding->second);
            std::string selected;
            if (name != names.end())
            {
                selected = name->second;
                // Clearing or moving an alias needs a fresh fallback record,
                // rather than a name-only patch over completed canonical bytes.
                if (selected.empty() && resolvedFruNames.contains(id))
                {
                    return std::nullopt;
                }
            }
            else if (previous &&
                     previous->fruDirectory.epoch == fruDirectory.epoch)
            {
                auto oldBinding = previous->fruDirectory.bindings.find(id);
                auto oldName = previous->resolvedFruNames.find(id);
                if (oldBinding != previous->fruDirectory.bindings.end() &&
                    oldBinding->second == binding->second &&
                    oldName != previous->resolvedFruNames.end())
                {
                    selected = oldName->second;
                }
            }
            if (selected.empty())
            {
                continue;
            }
            selected.resize(std::min(selected.size(), size_t(16)));
            resolvedFruNames.insert_or_assign(id, selected);
            const auto length = size_t(record[15] & 0x1f);
            if (record.size() == 16 + length && length == selected.size() &&
                std::equal(selected.begin(), selected.end(),
                           record.begin() + 16, [](char left, uint8_t right) {
                return static_cast<uint8_t>(left) == right;
            }))
            {
                continue;
            }
            record.resize(16 + selected.size());
            record[4] = record.size() - 5;
            record[15] = (record[15] & 0xe0) | selected.size();
            std::copy(selected.begin(), selected.end(), record.begin() + 16);
            ++updated;
        }
        return updated;
    }

    struct TemperatureSensor
    {
        std::string path;
        uint16_t recordId;
        uint8_t entityId;
        uint8_t entityInstance;
    };

    // Accept consecutive wire records; reject a truncated final record without
    // modifying the directory. Structure padding is removed by its producer.
    bool appendRecords(const std::vector<uint8_t>& bytes)
    {
        constexpr size_t headerSize = 5;
        for (size_t offset = 0; offset < bytes.size();)
        {
            if (bytes.size() - offset < headerSize)
            {
                return false;
            }
            const size_t size = headerSize + bytes[offset + 4];
            if (size > bytes.size() - offset)
            {
                return false;
            }
            offset += size;
        }
        for (size_t offset = 0; offset < bytes.size();)
        {
            const size_t size = headerSize + bytes[offset + 4];
            records.emplace_back(bytes.begin() + offset,
                                 bytes.begin() + offset + size);
            offset += size;
        }
        return true;
    }

    std::array<size_t, 4> sensorCountsByLun() const
    {
        std::array<size_t, 4> counts{};
        for (const auto& record : records)
        {
            if (record.size() >= 8 && record[3] >= 1 && record[3] <= 3)
            {
                ++counts[record[6] & 3];
            }
        }
        return counts;
    }

    // DCMI must describe this published directory, not rebuild live metadata
    // or mistake a stable sensor number for a positional SDR record ID.
    std::vector<TemperatureSensor> temperatureSensors() const
    {
        std::vector<TemperatureSensor> result;
        for (size_t index = 0; index < records.size(); ++index)
        {
            const auto& record = records[index];
            constexpr size_t recordTypeOffset = 3;
            constexpr size_t ownerLunOffset = 6;
            constexpr size_t sensorNumberOffset = 7;
            constexpr size_t entityIdOffset = 8;
            constexpr size_t entityInstanceOffset = 9;
            constexpr size_t sensorTypeOffset = 12;
            constexpr size_t minimumSize = sensorTypeOffset + 1;
            constexpr uint8_t fullSensorRecord = 0x01;
            constexpr uint8_t temperatureSensor = 0x01;
            if (record.size() < minimumSize ||
                record[recordTypeOffset] != fullSensorRecord ||
                record[sensorTypeOffset] != temperatureSensor)
            {
                continue;
            }
            const uint16_t number = ((record[ownerLunOffset] & 3) << 8) |
                                    record[sensorNumberOffset];
            auto path = sensorPaths.find(number);
            if (path != sensorPaths.end())
            {
                result.push_back(
                    {path->second, static_cast<uint16_t>(index),
                     record[entityIdOffset], record[entityInstanceOffset]});
            }
        }
        return result;
    }
};

enum class SdrReadStatus
{
    success,
    notReady,
    invalidReservation,
    invalidRecord,
    invalidOffset,
};

struct SdrReadResult
{
    SdrReadStatus status;
    uint16_t next = 0xffff;
    std::vector<uint8_t> data;
};

// Event-loop confined. No lock may be held across a D-Bus yield. A completed
// builder stages owned bytes; publication is one shared-pointer assignment.
// Only one repository generation is externally visible at a time.
class SdrRepository
{
  public:
    using Clock = std::chrono::steady_clock;
    using Time = Clock::time_point;

    explicit SdrRepository(
        Clock::duration idleLimit = std::chrono::seconds(5),
        Clock::duration lifetimeLimit = std::chrono::seconds(60)) :
        idleLimit(idleLimit), lifetimeLimit(lifetimeLimit)
    {}

    bool stage(SdrSnapshot snapshot, Time now)
    {
        // 0xffff is the last-record selector, never an ordinary record ID.
        if (snapshot.records.size() > 0xffff ||
            (published && snapshot.revision < published->revision) ||
            (pending && snapshot.revision < pending->revision))
        {
            return false;
        }
        for (size_t index = 0; index < snapshot.records.size(); ++index)
        {
            auto& record = snapshot.records[index];
            if (record.size() < 5 || record.size() != size_t(record[4]) + 5)
            {
                return false;
            }
            record[0] = index & 0xff;
            record[1] = index >> 8;
        }
        pending = std::make_shared<const SdrSnapshot>(std::move(snapshot));
        tick(now);
        return true;
    }

    std::shared_ptr<const SdrSnapshot> current(Time now)
    {
        tick(now);
        return published;
    }

    std::shared_ptr<const SdrSnapshot> latestCompleted() const
    {
        return pending ? pending : published;
    }

    std::optional<uint16_t> reserve(const SdrRequester& owner, Time now)
    {
        // IPMI 33.11.2: a new Reserve cancels the previous reservation.
        validReservation = false;
        lease.reset();
        tick(now);
        if (!published)
        {
            return std::nullopt;
        }
        if (++sequence == 0)
        {
            ++sequence;
        }
        validReservation = true;
        reservationOwner = owner;
        lease = Lease{owner, now, now};
        return sequence;
    }

    SdrReadResult read(const SdrRequester& owner, uint16_t reservation,
                       uint16_t recordId, uint8_t offset, uint8_t count,
                       Time now)
    {
        tick(now);
        // Only nonzero-offset partial reads require a reservation. Offset-zero
        // readers may retain an old token; their whole record is still atomic.
        if (offset != 0 &&
            (!validReservation || reservation == 0 || reservation != sequence ||
             reservationOwner != owner))
        {
            return {SdrReadStatus::invalidReservation};
        }
        auto snapshot = published;
        if (!snapshot)
        {
            return {SdrReadStatus::notReady};
        }
        if (snapshot->records.empty())
        {
            return {SdrReadStatus::invalidRecord};
        }
        const size_t index =
            recordId == 0xffff ? snapshot->records.size() - 1 : recordId;
        if (index >= snapshot->records.size())
        {
            return {SdrReadStatus::invalidRecord};
        }
        const auto& record = snapshot->records[index];
        if (offset >= record.size())
        {
            return {SdrReadStatus::invalidOffset};
        }
        if (!lease)
        {
            lease = Lease{owner, now, now};
        }
        else if (lease->owner == owner)
        {
            lease->lastUse = now;
        }
        if (lease->owner == owner && lease->lastRecord != index)
        {
            const auto expected =
                lease->lastRecord ? size_t(*lease->lastRecord) + 1 : 0;
            lease->sequential = lease->sequential && index == expected &&
                                (!lease->lastRecord || lease->recordComplete);
            lease->lastRecord = static_cast<uint16_t>(index);
            lease->contiguousEnd = 0;
            lease->recordComplete = false;
            ++lease->visited;
        }
        const size_t end = std::min(record.size(), size_t(offset) + count);
        if (lease->owner == owner && offset <= lease->contiguousEnd)
        {
            lease->contiguousEnd = std::max(lease->contiguousEnd, end);
            lease->recordComplete = lease->contiguousEnd == record.size();
        }
        const uint16_t next = index + 1 < snapshot->records.size()
                                  ? static_cast<uint16_t>(index + 1)
                                  : 0xffff;
        SdrReadResult result{SdrReadStatus::success,
                             next,
                             {record.begin() + offset, record.begin() + end}};
        // All earlier records and this record must have been read in full.
        // Header-only or out-of-order readers release through bounded expiry.
        if (lease->owner == owner && lease->sequential &&
            lease->visited == snapshot->records.size() && next == 0xffff &&
            count != 0 && lease->recordComplete)
        {
            lease.reset();
            tick(now);
        }
        return result;
    }

    std::optional<Time> nextDeadline() const
    {
        if (!lease || !pending)
        {
            return std::nullopt;
        }
        return std::min(lease->lastUse + idleLimit,
                        lease->started + lifetimeLimit);
    }

    void tick(Time now)
    {
        if (lease && (now - lease->lastUse >= idleLimit ||
                      now - lease->started >= lifetimeLimit))
        {
            lease.reset();
        }
        if (!lease && pending)
        {
            // Publishing a replacement can change record IDs. Cancel the old
            // reservation before exposing it (IPMI 33.11.2).
            validReservation = false;
            published = std::exchange(pending, {});
        }
    }

  private:
    struct Lease
    {
        SdrRequester owner;
        Time started;
        Time lastUse;
        std::optional<uint16_t> lastRecord;
        size_t visited = 0;
        size_t contiguousEnd = 0;
        bool recordComplete = false;
        bool sequential = true;
    };

    Clock::duration idleLimit;
    Clock::duration lifetimeLimit;
    std::shared_ptr<const SdrSnapshot> published;
    std::shared_ptr<const SdrSnapshot> pending;
    std::optional<Lease> lease;
    uint16_t sequence = 0;
    bool validReservation = false;
    SdrRequester reservationOwner{};
};

} // namespace ipmi::sensor

namespace ipmi
{
// Called by provider change handlers on the IPMI io_context.
void invalidateSdrRepository(bool addition = true, bool removal = true);
void notifyFruNamesChanged();
} // namespace ipmi
