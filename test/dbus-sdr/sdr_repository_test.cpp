// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#include "dbus-sdr/sdr_metadata.hpp"
#include "dbus-sdr/sdr_repository.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{
using namespace std::chrono_literals;
using namespace ipmi::sensor;
constexpr SdrRequester host{15, 0, 0, 0};
constexpr SdrRequester lan{1, 17, 0, 0};
constexpr SdrRepository::Time start{};

static SdrSnapshot generation(size_t revision, size_t sensors)
{
    SdrSnapshot result;
    result.revision = revision;
    result.lastAddition = revision;
    for (size_t index = 0; index < sensors; ++index)
    {
        result.records.push_back(
            {0, 0, 0x51, 0x01, 2, static_cast<uint8_t>(index), 0xaa});
    }
    result.records.push_back({0, 0, 0x51, 0x11, 2, 7, 0xbb}); // PDB locator
    result.records.push_back({0, 0, 0x51, 0x12, 2, 0, 0xcc});
    return result;
}

TEST(SdrRepository, ReservedHeaderBodyAndNextIdUseOneGeneration)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 100), start));
    auto token = repo.reserve(host, start);
    ASSERT_TRUE(token);
    auto header = repo.read(host, 0, 100, 0, 5, start);
    ASSERT_EQ(header.status, SdrReadStatus::success);
    ASSERT_GE(header.data.size(), 4u);
    EXPECT_EQ(header.data[3], 0x11);
    EXPECT_EQ(header.next, 101);
    ASSERT_TRUE(repo.stage(generation(2, 101), start + 1s));
    auto body = repo.read(host, token.value_or(0), 100, 5, 2, start + 2s);
    ASSERT_EQ(body.status, SdrReadStatus::success);
    EXPECT_EQ(body.next, header.next);
    EXPECT_EQ(body.data, (std::vector<uint8_t>{7, 0xbb}));
    EXPECT_EQ(repo.current(start + 2s)->revision, 1u);
}

TEST(SdrRepository, UnreservedRepeatedOffsetZeroReadsAlsoStayConsistent)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 100), start));
    auto header = repo.read(host, 0, 100, 0, 5, start);
    ASSERT_EQ(header.status, SdrReadStatus::success);
    ASSERT_TRUE(repo.stage(generation(2, 101), start + 1s));
    auto body = repo.read(host, 0, 100, 0, 7, start + 2s);
    ASSERT_EQ(body.status, SdrReadStatus::success);
    ASSERT_GE(body.data.size(), 4u);
    EXPECT_EQ(body.data[3], 0x11);
    EXPECT_EQ(body.next, header.next);
}

TEST(SdrRepository, SensorRemovalCannotRejectAnAdvertisedNextRecord)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 100), start));
    auto token = repo.reserve(host, start);
    auto before = repo.read(host, 0, 100, 0, 5, start);
    ASSERT_EQ(before.status, SdrReadStatus::success);
    ASSERT_TRUE(repo.stage(generation(2, 0), start + 1s));
    auto next = repo.read(host, 0, before.next, 0, 5, start + 2s);
    ASSERT_EQ(next.status, SdrReadStatus::success);
    ASSERT_GE(next.data.size(), 4u);
    EXPECT_EQ(next.data[3], 0x12);
    EXPECT_EQ(repo.current(start + 2s)->revision, 1u);
    ASSERT_TRUE(token);
}

TEST(SdrRepository, FinalHeaderDoesNotPublishBeforeItsBody)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    auto token = repo.reserve(host, start);
    ASSERT_TRUE(token);
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    ASSERT_EQ(repo.read(host, 0, 0, 0, 7, start + 1s).status,
              SdrReadStatus::success);
    auto header = repo.read(host, 0, 1, 0, 5, start + 2s);
    ASSERT_EQ(header.status, SdrReadStatus::success);
    EXPECT_EQ(header.next, 0xffff);
    EXPECT_EQ(repo.current(start + 2s)->revision, 1u);
    auto body = repo.read(host, token.value_or(0), 1, 5, 2, start + 3s);
    ASSERT_EQ(body.status, SdrReadStatus::success);
    EXPECT_EQ(body.data, (std::vector<uint8_t>{0, 0xcc}));
    EXPECT_EQ(repo.current(start + 3s)->revision, 2u);
    EXPECT_EQ(repo.read(host, token.value_or(0), 1, 5, 2, start + 3s).status,
              SdrReadStatus::invalidReservation);
}

TEST(SdrRepository, EarlierHeadersDoNotCompleteTheWalk)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    const auto reservation = repo.reserve(host, start);
    ASSERT_TRUE(reservation);
    const auto token = reservation.value_or(0);
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    ASSERT_EQ(repo.read(host, token, 0, 0, 5, start + 1s).status,
              SdrReadStatus::success);
    ASSERT_EQ(repo.read(host, token, 1, 0, 7, start + 2s).status,
              SdrReadStatus::success);
    EXPECT_EQ(repo.current(start + 2s)->revision, 1u);
    auto body = repo.read(host, token, 0, 5, 2, start + 3s);
    EXPECT_EQ(body.status, SdrReadStatus::success);
    EXPECT_EQ(body.data, (std::vector<uint8_t>{7, 0xbb}));
    repo.tick(start + 8s);
    EXPECT_EQ(repo.current(start + 8s)->revision, 2u);
}

TEST(SdrRepository, AGapInTheFinalRecordDoesNotCompleteTheWalk)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    const auto reservation = repo.reserve(host, start);
    ASSERT_TRUE(reservation);
    const auto token = reservation.value_or(0);
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    ASSERT_EQ(repo.read(host, token, 0, 0, 7, start + 1s).status,
              SdrReadStatus::success);
    ASSERT_EQ(repo.read(host, token, 1, 0, 5, start + 2s).status,
              SdrReadStatus::success);
    ASSERT_EQ(repo.read(host, token, 1, 6, 1, start + 3s).status,
              SdrReadStatus::success);
    EXPECT_EQ(repo.current(start + 3s)->revision, 1u);
    ASSERT_EQ(repo.read(host, token, 1, 5, 2, start + 4s).status,
              SdrReadStatus::success);
    EXPECT_EQ(repo.current(start + 4s)->revision, 2u);
}

TEST(SdrRepository, StaleReservationOnlyProtectsNonzeroOffsetReads)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    const auto reservation = repo.reserve(host, start);
    ASSERT_TRUE(reservation);
    const auto token = reservation.value_or(0);
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    repo.tick(start + 5s);
    EXPECT_EQ(repo.read(host, token, 0, 5, 2, start + 6s).status,
              SdrReadStatus::invalidReservation);
    EXPECT_EQ(repo.read(host, 0, 0, 0, 5, start + 6s).status,
              SdrReadStatus::success);
}

TEST(SdrRepository, AnotherRequesterCannotUseAReservedPartialRead)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    const auto reservation = repo.reserve(host, start);
    ASSERT_TRUE(reservation);
    const auto token = reservation.value_or(0);
    EXPECT_EQ(repo.read(lan, token, 0, 5, 2, start).status,
              SdrReadStatus::invalidReservation);
    EXPECT_EQ(repo.read(host, token, 0, 5, 2, start).status,
              SdrReadStatus::success);
}

TEST(SdrRepository, AbandonedAndHeaderOnlyWalksPublishAfterInactivity)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    ASSERT_EQ(repo.read(host, 0, 1, 0, 5, start).status,
              SdrReadStatus::success);
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    repo.tick(start + 5s);
    EXPECT_EQ(repo.current(start + 5s)->revision, 2u);
}

TEST(SdrRepository, ContinuousReadsCannotPinTheRepositoryForever)
{
    SdrRepository repo(5s, 10s);
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    auto token = repo.reserve(host, start);
    ASSERT_TRUE(token);
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    for (int second = 1; second < 10; ++second)
    {
        EXPECT_EQ(repo.read(host, token.value_or(0), 0, 5, 1,
                            start + std::chrono::seconds(second))
                      .status,
                  SdrReadStatus::success);
    }
    EXPECT_EQ(repo.read(host, token.value_or(0), 0, 5, 1, start + 10s).status,
              SdrReadStatus::invalidReservation);
    EXPECT_EQ(repo.current(start + 10s)->revision, 2u);
}

TEST(SdrRepository, OtherClientsCannotExtendTheOwnersLease)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    ASSERT_TRUE(repo.reserve(host, start));
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    ASSERT_EQ(repo.read(lan, 0, 0, 0, 5, start + 4s).status,
              SdrReadStatus::success);
    EXPECT_EQ(repo.current(start + 5s)->revision, 2u);
}

TEST(SdrRepository, NewReservationCancelsThePreviousOwnerAndPublishesPending)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    auto first = repo.reserve(host, start);
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    auto second = repo.reserve(lan, start + 2s);
    ASSERT_TRUE(first);
    ASSERT_TRUE(second);
    EXPECT_NE(first.value_or(0), second.value_or(0));
    EXPECT_EQ(repo.current(start + 2s)->revision, 2u);
    EXPECT_EQ(repo.read(host, first.value_or(0), 0, 5, 1, start + 2s).status,
              SdrReadStatus::invalidReservation);
    EXPECT_EQ(repo.read(host, second.value_or(0), 0, 5, 1, start + 2s).status,
              SdrReadStatus::invalidReservation);
    EXPECT_EQ(repo.read(lan, second.value_or(0), 0, 5, 1, start + 2s).status,
              SdrReadStatus::success);
}

TEST(SdrRepository, FailedBuildLeavesThePublishedGenerationAvailable)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    auto invalid = generation(2, 1);
    invalid.records.back().resize(5);
    EXPECT_FALSE(repo.stage(std::move(invalid), start));
    EXPECT_EQ(repo.current(start)->revision, 1u);
    EXPECT_EQ(repo.read(host, 0, 0, 0, 7, start).status,
              SdrReadStatus::success);
}

TEST(SdrRepository, LatestPendingBuildWinsAndOlderCompletionCannotRollBack)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    ASSERT_TRUE(repo.reserve(host, start));
    ASSERT_TRUE(repo.stage(generation(3, 2), start + 1s));
    EXPECT_FALSE(repo.stage(generation(2, 1), start + 2s));
    repo.tick(start + 5s);
    EXPECT_EQ(repo.current(start + 5s)->revision, 3u);
    EXPECT_FALSE(repo.stage(generation(1, 0), start + 6s));
}

TEST(SdrRepository, EmptyColdAndOutOfRangeReadsReturnExplicitErrors)
{
    SdrRepository repo;
    EXPECT_FALSE(repo.reserve(host, start));
    EXPECT_EQ(repo.read(host, 0, 0, 0, 5, start).status,
              SdrReadStatus::notReady);
    ASSERT_TRUE(repo.stage(SdrSnapshot{}, start));
    EXPECT_EQ(repo.read(host, 0, 0xffff, 0, 5, start).status,
              SdrReadStatus::invalidRecord);
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    EXPECT_EQ(repo.read(host, 0, 2, 0, 5, start).status,
              SdrReadStatus::invalidRecord);
    EXPECT_EQ(repo.read(host, 0, 0, 7, 1, start).status,
              SdrReadStatus::invalidReservation);
    auto token = repo.reserve(host, start);
    ASSERT_TRUE(token);
    EXPECT_EQ(repo.read(host, token.value_or(0), 0, 7, 1, start).status,
              SdrReadStatus::invalidOffset);
    const auto last = repo.read(host, 0, 0xffff, 0, 5, start);
    ASSERT_EQ(last.status, SdrReadStatus::success);
    ASSERT_GE(last.data.size(), 4u);
    EXPECT_EQ(last.data[3], 0x12);
}

TEST(SdrRepository, ReservationWrapNeverIssuesZero)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    for (size_t index = 0; index < 0x10001; ++index)
    {
        auto token = repo.reserve(host, start);
        ASSERT_TRUE(token);
        ASSERT_NE(token.value_or(0), 0);
    }
}

TEST(SdrRepository, SnapshotStorageIsReleasedAfterPublication)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 0), start));
    std::weak_ptr<const SdrSnapshot> old = repo.current(start);
    ASSERT_TRUE(repo.reserve(host, start));
    ASSERT_TRUE(repo.stage(generation(2, 1), start + 1s));
    EXPECT_FALSE(old.expired());
    repo.tick(start + 5s);
    EXPECT_TRUE(old.expired());
}

TEST(SdrRepository, RandomLastRecordReadDoesNotFinishAnExistingWalk)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(generation(1, 100), start));
    auto token = repo.reserve(host, start);
    ASSERT_EQ(repo.read(host, 0, 0, 0, 5, start).status,
              SdrReadStatus::success);
    ASSERT_TRUE(repo.stage(generation(2, 101), start + 1s));
    ASSERT_EQ(repo.read(host, 0, 0xffff, 0, 7, start + 2s).status,
              SdrReadStatus::success);
    EXPECT_EQ(repo.current(start + 2s)->revision, 1u);
    ASSERT_TRUE(token);
    EXPECT_EQ(repo.read(host, token.value_or(0), 0, 5, 2, start + 3s).status,
              SdrReadStatus::success);
}

TEST(SdrMetadata, DescriptorChangesExcludeTelemetryAndAlarms)
{
    EXPECT_TRUE(isSdrDescriptorProperty(
        "xyz.openbmc_project.Sensor.Value", "MaxValue"));
    EXPECT_TRUE(isSdrDescriptorProperty(
        "xyz.openbmc_project.Sensor.Value", "MinValue"));
    EXPECT_TRUE(isSdrDescriptorProperty(
        "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningHigh"));
    EXPECT_TRUE(isSdrDescriptorProperty(
        "xyz.openbmc_project.Sensor.ValueMutability", "Mutable"));
    EXPECT_TRUE(isSdrDescriptorProperty(
        "xyz.openbmc_project.Association.Definitions", "Associations"));
    EXPECT_FALSE(
        isSdrDescriptorProperty("xyz.openbmc_project.Sensor.Value", "Value"));
    EXPECT_FALSE(isSdrDescriptorProperty(
        "xyz.openbmc_project.Sensor.Threshold.Warning", "WarningAlarmHigh"));
    EXPECT_FALSE(isSdrDescriptorProperty(
        "xyz.openbmc_project.State.Decorator.OperationalStatus", "Functional"));
}

TEST(SdrRepository, DcmiRecordIdsAndEntitiesFollowThePublishedGeneration)
{
    auto first = generation(1, 0);
    first.records.insert(first.records.begin(),
                         {0, 0, 0x51, 1, 9, 0x20, 1, 0x34, 3, 7, 0, 0, 1, 1});
    first.sensorPaths.emplace(0x134, "/sensors/temperature/cpu");
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(first, start));
    ASSERT_TRUE(repo.reserve(host, start));
    auto next = first;
    next.revision = 2;
    next.records.insert(next.records.begin(), {0, 0, 0x51, 3, 2, 0, 0});
    next.records[1][9] = 9;
    ASSERT_TRUE(repo.stage(next, start + 1s));
    const auto pinned = repo.current(start + 1s)->temperatureSensors();
    ASSERT_EQ(pinned.size(), 1u);
    EXPECT_EQ(pinned[0].recordId, 0);
    EXPECT_EQ(pinned[0].entityInstance, 7);
    EXPECT_EQ(pinned[0].path, "/sensors/temperature/cpu");
    repo.tick(start + 5s);
    const auto published = repo.current(start + 5s)->temperatureSensors();
    ASSERT_EQ(published.size(), 1u);
    EXPECT_EQ(published[0].recordId, 1);
    EXPECT_EQ(published[0].entityInstance, 9);
}

} // namespace
