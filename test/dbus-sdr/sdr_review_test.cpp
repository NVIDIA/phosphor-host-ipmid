// SPDX-License-Identifier: Apache-2.0
#include "dbus-sdr/sdr_repository.hpp"
#include "dbus-sdr/stable_ids.hpp"

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
constexpr SdrRequester owner{15, 0, 0, 0};
constexpr SdrRepository::Time zero{};

SdrSnapshot makeSnapshot(size_t revision, size_t count)
{
    SdrSnapshot snapshot;
    snapshot.revision = revision;
    for (size_t index = 0; index < count; ++index)
    {
        snapshot.records.push_back(
            {0xa5, 0xa5, 0x51, 0x11, 2, static_cast<uint8_t>(index), 0x5a});
    }
    return snapshot;
}

TEST(SdrReview, OutOfOrderCompleteRecordsDoNotFinishTheWalk)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(makeSnapshot(1, 3), zero));
    ASSERT_TRUE(repo.reserve(owner, zero));
    ASSERT_TRUE(repo.stage(makeSnapshot(2, 4), zero + 1s));
    for (const uint16_t record : {1, 0, 2})
    {
        ASSERT_EQ(repo.read(owner, 0, record, 0, 7, zero + 2s).status,
                  SdrReadStatus::success);
    }
    EXPECT_EQ(repo.current(zero + 2s)->revision, 1u);
    repo.tick(zero + 7s);
    EXPECT_EQ(repo.current(zero + 7s)->revision, 2u);
}

TEST(SdrReview, WrongTokenProtectsPartialReadWhileOffsetZeroIgnoresIt)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(makeSnapshot(1, 1), zero));
    const auto reservation = repo.reserve(owner, zero);
    ASSERT_TRUE(reservation);
    const auto token = reservation.value_or(0);
    ASSERT_TRUE(repo.stage(makeSnapshot(2, 2), zero + 1s));
    const auto wrong = static_cast<uint16_t>(token + 1);
    EXPECT_EQ(repo.read(owner, wrong, 0, 0, 5, zero + 2s).status,
              SdrReadStatus::success);
    EXPECT_EQ(repo.read(owner, wrong, 0, 5, 2, zero + 2s).status,
              SdrReadStatus::invalidReservation);
    EXPECT_EQ(repo.current(zero + 2s)->revision, 1u);
    EXPECT_EQ(repo.read(owner, token, 0, 5, 2, zero + 3s).status,
              SdrReadStatus::success);
    EXPECT_EQ(repo.current(zero + 3s)->revision, 2u);
}

TEST(SdrReview, RecordIdsAreStampedInBothBytes)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(makeSnapshot(1, 258), zero));
    const auto snapshot = repo.current(zero);
    for (const uint16_t id : {0, 1, 255, 256, 257})
    {
        ASSERT_EQ(snapshot->records[id][0], id & 0xff);
        ASSERT_EQ(snapshot->records[id][1], id >> 8);
    }
}

TEST(SdrReview, RepositoryLimitReservesFFFFForTheLastSelector)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(makeSnapshot(1, 0xffff), zero));
    EXPECT_EQ(repo.current(zero)->records.size(), 0xffffu);
    const auto last = repo.read(owner, 0, 0xffff, 0, 7, zero);
    ASSERT_EQ(last.status, SdrReadStatus::success);
    EXPECT_EQ(last.data[0], 0xfe);
    EXPECT_EQ(last.data[1], 0xff);
    EXPECT_EQ(last.next, 0xffff);
    EXPECT_FALSE(repo.stage(makeSnapshot(2, 0x10000), zero));
    EXPECT_EQ(repo.current(zero)->revision, 1u);
}

TEST(SdrReview, DcmiFiltersNonTemperatureAndNonFullRecords)
{
    SdrSnapshot snapshot;
    snapshot.records = {
        {0, 0, 0x51, 1, 8, 0x20, 1, 0x31, 3, 1, 0, 0, 1},
        {0, 0, 0x51, 1, 8, 0x20, 1, 0x32, 3, 2, 0, 0, 2},
        {0, 0, 0x51, 2, 8, 0x20, 1, 0x33, 3, 3, 0, 0, 1},
        {0, 0, 0x51, 3, 8, 0x20, 1, 0x34, 3, 4, 0, 0, 1},
        {0, 0, 0x51, 1, 8, 0x20, 1, 0x35, 3, 5, 0, 0, 1},
        {0, 0, 0x51, 1, 2, 0x20, 1},
    };
    snapshot.sensorPaths = {{0x131, "/temperature/cpu"},
                            {0x132, "/voltage/cpu"},
                            {0x133, "/compact"},
                            {0x134, "/event"}};
    const auto temperatures = snapshot.temperatureSensors();
    ASSERT_EQ(temperatures.size(), 1u);
    EXPECT_EQ(temperatures[0].path, "/temperature/cpu");
    EXPECT_EQ(temperatures[0].recordId, 0);
    EXPECT_EQ(temperatures[0].entityInstance, 1);
}

TEST(SdrReview, PreferredAllocationUsesAnExplicitPoolIdentity)
{
    const std::vector<uint16_t> pool{0x02, 0x103, 0x304, 0x05};
    for (size_t index = 0; index < pool.size(); ++index)
    {
        StableIds<std::string, uint16_t> ids(pool);
        EXPECT_EQ(ids.allocate("sensor", pool[index]), pool[index]);
    }
}

TEST(SdrReview, ZeroByteLastReadDoesNotFinishTheWalk)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(makeSnapshot(1, 1), zero));
    ASSERT_TRUE(repo.reserve(owner, zero));
    ASSERT_TRUE(repo.stage(makeSnapshot(2, 2), zero + 1s));
    const auto read = repo.read(owner, 0, 0, 0, 0, zero + 2s);
    EXPECT_EQ(read.status, SdrReadStatus::success);
    EXPECT_TRUE(read.data.empty());
    EXPECT_EQ(repo.current(zero + 2s)->revision, 1u);
}

TEST(SdrReview, UnreservedWalkKeepsEverySuccessorAcrossRepeatedAdditions)
{
    SdrRepository repo;
    ASSERT_TRUE(repo.stage(makeSnapshot(1, 4), zero));
    for (uint16_t id = 0; id < 4; ++id)
    {
        const auto header = repo.read(owner, 0, id, 0, 5, zero + 1s);
        ASSERT_EQ(header.status, SdrReadStatus::success);
        ASSERT_TRUE(repo.stage(makeSnapshot(2 + id, 5 + id), zero + 1s));
        const auto body = repo.read(owner, 0, id, 0, 7, zero + 1s);
        ASSERT_EQ(body.status, SdrReadStatus::success);
        EXPECT_EQ(body.next, header.next);
        EXPECT_EQ(body.data[5], id);
        EXPECT_EQ(body.next, id == 3 ? 0xffff : id + 1);
    }
    EXPECT_EQ(repo.current(zero + 1s)->revision, 5u);
}
TEST(SdrReview, ConcatenatedRecordsRemainSeparateAndTruncationIsAtomic)
{
    SdrSnapshot snapshot;
    const std::vector<uint8_t> bytes{
        0, 0, 0x51, 1, 1, 0xaa, 0, 0, 0x51, 3, 1, 0xbb};
    ASSERT_TRUE(snapshot.appendRecords(bytes));
    ASSERT_EQ(snapshot.records.size(), 2u);
    EXPECT_EQ(snapshot.records[0].back(), 0xaa);
    EXPECT_EQ(snapshot.records[1].back(), 0xbb);
    auto truncated = bytes;
    truncated.pop_back();
    EXPECT_FALSE(snapshot.appendRecords(truncated));
    EXPECT_EQ(snapshot.records.size(), 2u);
}

TEST(SdrReview, LeaseTimerOnlyExistsForPendingPublication)
{
    SdrRepository repo;
    EXPECT_FALSE(repo.nextDeadline());
    ASSERT_TRUE(repo.stage(makeSnapshot(1, 2), zero));
    ASSERT_TRUE(repo.reserve(owner, zero));
    EXPECT_FALSE(repo.nextDeadline());
    ASSERT_TRUE(repo.stage(makeSnapshot(2, 3), zero + 1s));
    EXPECT_EQ(repo.nextDeadline(), zero + 5s);
    ASSERT_EQ(repo.read(owner, 0, 0, 0, 5, zero + 2s).status,
              SdrReadStatus::success);
    EXPECT_EQ(repo.nextDeadline(), zero + 7s);
    repo.tick(zero + 7s);
    EXPECT_FALSE(repo.nextDeadline());
    EXPECT_EQ(repo.current(zero + 7s)->revision, 2u);
}

TEST(SdrReview, PerLunCountsExcludeNonSensorRecords)
{
    SdrSnapshot snapshot;
    snapshot.records = {
        {0, 0, 0x51, 1, 3, 0x20, 0, 0x31},
        {0, 0, 0x51, 2, 3, 0x20, 1, 0x32},
        {0, 0, 0x51, 3, 3, 0x20, 3, 0x33},
        {0, 0, 0x51, 0x11, 3, 0x20, 0, 7},
    };
    EXPECT_EQ(snapshot.sensorCountsByLun(),
              (std::array<size_t, 4>{1, 1, 0, 1}));
}
} // namespace
