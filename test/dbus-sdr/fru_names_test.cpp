// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#include "dbus-sdr/fru_names.hpp"
#include "dbus-sdr/sdr_repository.hpp"

#include <gtest/gtest.h>

namespace
{
using namespace ipmi::sensor;
using namespace std::chrono_literals;
constexpr FruLocation location{12, 0x54};
const FruNames::Source source{"example.Inventory", "/example/board"};
constexpr SdrRequester host{15, 0, 0, 0};
constexpr SdrRepository::Time now{};

SdrSnapshot view(size_t revision = 1, std::string name = "EEPROM fallback",
                 size_t sensors = 1)
{
    SdrSnapshot result;
    result.revision = revision;
    result.fruDirectory = {7, {{27, location}}};
    for (size_t i = 0; i < sensors; ++i)
    {
        result.records.push_back({0, 0, 0x51, 1, 2, uint8_t(i), 0});
    }
    std::vector<uint8_t> fru(16);
    fru[2] = 0x51;
    fru[3] = 0x11;
    fru[4] = 11 + name.size();
    fru[5] = 0x20;
    fru[6] = 27;
    fru[15] = 0xc0 | name.size();
    fru.insert(fru.end(), name.begin(), name.end());
    result.records.push_back(fru);
    return result;
}

std::string label(const SdrSnapshot& snapshot)
{
    const auto& record = snapshot.records.back();
    return {record.begin() + 16, record.end()};
}

TEST(FruNames, UnrelatedDiscoveryDoesNotRejectACompletedName)
{
    FruNames names;
    auto token = names.begin(source);
    for (size_t i = 0; i < 2000; ++i)
    {
        names.changed("/example/unrelated");
    }
    ASSERT_EQ(names.commit(token, location, "Board_A"), true);
    auto snapshot = view();
    EXPECT_EQ(snapshot.refreshFruNames(snapshot.fruDirectory, names.resolved()),
              1u);
    EXPECT_EQ(label(snapshot), "Board_A");
    EXPECT_EQ(snapshot.records.back()[4] + 5u, snapshot.records.back().size());
}

TEST(FruNames, ChangedObjectRejectsLateReply)
{
    FruNames names;
    auto token = names.begin(source);
    names.changed(source.second);
    EXPECT_FALSE(names.commit(token, location, "Old_Name"));
    EXPECT_TRUE(names.resolved().empty());
}

TEST(FruNames, ProviderRestartRejectsOldOwnerReply)
{
    FruNames names;
    auto token = names.begin(source);
    ASSERT_TRUE(names.providerChanged(source.first));
    EXPECT_FALSE(names.commit(token, location, "Old_Name"));
    ASSERT_EQ(names.commit(names.begin(source), location, "New_Name"), true);
    EXPECT_EQ(names.resolved().at(location), "New_Name");
}

TEST(FruNames, CompletedNewerAcquisitionRejectsAnOlderReply)
{
    FruNames names;
    auto old = names.begin(source);
    auto newer = names.begin(source);
    ASSERT_EQ(names.commit(newer, location, "New_Name"), true);
    EXPECT_FALSE(names.commit(old, location, "Old_Name"));
    EXPECT_EQ(names.resolved().at(location), "New_Name");
}

TEST(FruNames, ProviderLossClearsAcquiredNames)
{
    FruNames names;
    ASSERT_EQ(names.commit(names.begin(source), location, "Board_A"), true);
    ASSERT_TRUE(names.providerChanged(source.first));
    EXPECT_TRUE(names.resolved().empty());
}

TEST(FruNames, MetadataMovingToAnotherLocationRetiresItsOldBinding)
{
    FruNames names;
    ASSERT_EQ(names.commit(names.begin(source), location, "Board_A"), true);
    FruLocation moved{13, 0x55};
    ASSERT_EQ(names.commit(names.begin(source), moved, "Board_A"), true);
    EXPECT_EQ(names.resolved().at(location), "");
    EXPECT_EQ(names.resolved().at(moved), "Board_A");
}

TEST(FruNames, ConflictingProvidersDoNotSelectAnArbitraryName)
{
    FruNames names;
    ASSERT_EQ(names.commit(names.begin(source), location, "Board_A"), true);
    ASSERT_EQ(names.commit(names.begin({"other.Inventory", "/other"}), location,
                           "Board_B"),
              true);
    EXPECT_TRUE(names.resolved().empty());
}

TEST(FruNames, RemovingAnObjectRejectsItsLateReplyAndRetiresItsAlias)
{
    FruNames names;
    ASSERT_EQ(names.commit(names.begin(source), location, "Board_A"), true);
    auto stale = names.begin(source);
    names.removed(source.second);
    EXPECT_FALSE(names.commit(stale, location, "Stale_Name"));
    EXPECT_EQ(names.resolved().at(location), "");
}

TEST(FruNames, ProviderLossResolvesAConflictWithoutDroppingOtherProvider)
{
    FruNames names;
    ASSERT_EQ(names.commit(names.begin(source), location, "Board_A"), true);
    FruNames::Source other{"other.Inventory", "/other"};
    ASSERT_EQ(names.commit(names.begin(other), location, "Board_B"), true);
    ASSERT_TRUE(names.resolved().empty());
    ASSERT_TRUE(names.providerChanged(other.first));
    EXPECT_EQ(names.resolved().at(location), "Board_A");
}

TEST(FruNames, NameChangePreservesHeaderBodyViewUntilReaderExpires)
{
    SdrRepository repository;
    ASSERT_TRUE(repository.stage(view(), now));
    auto header = repository.read(host, 0, 1, 0, 5, now);
    auto replacement = *repository.latestCompleted();
    ASSERT_EQ(replacement.refreshFruNames(
                  replacement.fruDirectory, {{location, "Board_A"}}),
              1u);
    replacement.revision = 2;
    ASSERT_TRUE(repository.stage(replacement, now + 1s));
    auto body = repository.read(host, 0, 1, 0, 255, now + 2s);
    EXPECT_EQ(body.data[4], header.data[4]);
    EXPECT_EQ(std::string(body.data.begin() + 16, body.data.end()),
              "EEPROM fallback");
    EXPECT_EQ(repository.current(now + 8s)->revision, 2u);
    EXPECT_EQ(label(*repository.current(now + 8s)), "Board_A");
}

TEST(FruNames, NameUpdateUsesPendingCompletedSensorSet)
{
    SdrRepository repository;
    ASSERT_TRUE(repository.stage(view(), now));
    ASSERT_TRUE(repository.reserve(host, now));
    ASSERT_TRUE(repository.stage(view(2, "EEPROM fallback", 3), now + 1s));
    auto corrected = *repository.latestCompleted();
    ASSERT_EQ(corrected.records.size(), 4u);
    ASSERT_EQ(corrected.refreshFruNames(
                  corrected.fruDirectory, {{location, "Board_A"}}),
              1u);
    corrected.revision = 3;
    ASSERT_TRUE(repository.stage(corrected, now + 2s));
    EXPECT_EQ(repository.current(now + 6s)->records.size(), 4u);
    EXPECT_EQ(label(*repository.current(now + 6s)), "Board_A");
}

TEST(FruNames, RemovedFruCannotReceiveANamePatch)
{
    auto snapshot = view();
    auto directory = snapshot.fruDirectory;
    directory.bindings.clear();
    EXPECT_FALSE(snapshot.refreshFruNames(directory, {{location, "Board_A"}}));
}

TEST(FruNames, ReboundIdentityCannotReceiveANamePatch)
{
    auto snapshot = view();
    auto directory = snapshot.fruDirectory;
    directory.bindings[27] = {14, 0x55};
    EXPECT_FALSE(snapshot.refreshFruNames(directory, {{location, "Board_A"}}));
}

TEST(FruNames, FruProviderEpochChangeRejectsCompletedOldDirectory)
{
    auto snapshot = view();
    auto directory = snapshot.fruDirectory;
    ++directory.epoch;
    EXPECT_FALSE(snapshot.refreshFruNames(directory, {{location, "Board_A"}}));
}

TEST(FruNames, LaterFullBuildRetainsResolvedAliasDuringMetadataOutage)
{
    auto previous = view();
    ASSERT_EQ(previous.refreshFruNames(
                  previous.fruDirectory, {{location, "Board_A"}}),
              1u);
    auto candidate = view(100, "EEPROM fallback", 3);
    ASSERT_EQ(candidate.refreshFruNames(candidate.fruDirectory, {}, &previous),
              1u);
    EXPECT_EQ(label(candidate), "Board_A");
}

TEST(FruNames, ExplicitlyUnnamedMetadataAllowsFullBuildFallback)
{
    auto previous = view();
    ASSERT_EQ(previous.refreshFruNames(
                  previous.fruDirectory, {{location, "Board_A"}}),
              1u);
    auto candidate = view(2);
    EXPECT_EQ(candidate.refreshFruNames(
                  candidate.fruDirectory, {{location, ""}}, &previous),
              0u);
    EXPECT_EQ(label(candidate), "EEPROM fallback");
}

TEST(FruNames, OrdinaryFallbackNeedsNoMetadataProvider)
{
    auto candidate = view();
    EXPECT_EQ(candidate.refreshFruNames(candidate.fruDirectory, {}), 0u);
    EXPECT_EQ(label(candidate), "EEPROM fallback");
}

TEST(FruNames, ClearingCanonicalAliasRequiresFreshFallbackRecords)
{
    auto previous = view();
    ASSERT_EQ(previous.refreshFruNames(
                  previous.fruDirectory, {{location, "Board_A"}}),
              1u);
    auto patch = previous;
    EXPECT_FALSE(patch.refreshFruNames(patch.fruDirectory, {{location, ""}}));
    auto full = view(2);
    EXPECT_EQ(
        full.refreshFruNames(full.fruDirectory, {{location, ""}}, &previous),
        0u);
    EXPECT_EQ(label(full), "EEPROM fallback");
}

TEST(FruNames, LongResolvedNameHasConsistentRecordLengths)
{
    auto candidate = view();
    ASSERT_EQ(candidate.refreshFruNames(
                  candidate.fruDirectory, {{location, std::string(40, 'A')}}),
              1u);
    EXPECT_EQ(label(candidate).size(), 16u);
    EXPECT_EQ(candidate.records.back().size(), 32u);
    EXPECT_EQ(candidate.records.back()[4], 27u);
    EXPECT_EQ(candidate.records.back()[15], 0xd0u);
}

TEST(FruNames, IdenticalLatin1NameDoesNotGenerateRepeatedReplacements)
{
    auto candidate = view();
    const std::string name{"Board_\xe9"};
    ASSERT_EQ(
        candidate.refreshFruNames(candidate.fruDirectory, {{location, name}}),
        1u);
    EXPECT_EQ(
        candidate.refreshFruNames(candidate.fruDirectory, {{location, name}}),
        0u);
}
} // namespace
