// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#include "dbus-sdr/fru_discovery.hpp"

#include <gtest/gtest.h>

namespace
{
using ipmi::storage::FruDiscoveryState;

TEST(FruDiscovery, AProviderRestartRejectsTheInflightReply)
{
    FruDiscoveryState state;
    const auto original = state.beginRefresh();
    ASSERT_TRUE(original);
    state.invalidate();
    EXPECT_FALSE(state.beginRefresh());
    EXPECT_FALSE(state.finishRefresh(original.value_or(0), true));
    EXPECT_FALSE(state.ready());
    const auto replacement = state.beginRefresh();
    ASSERT_TRUE(replacement);
    EXPECT_NE(original, replacement);
    EXPECT_TRUE(state.finishRefresh(replacement.value_or(0), true));
    EXPECT_TRUE(state.ready());
}

TEST(FruDiscovery, AnObjectSignalRejectsAnOlderDirectoryReply)
{
    FruDiscoveryState state;
    const auto original = state.beginRefresh();
    ASSERT_TRUE(original);
    state.changed();
    EXPECT_FALSE(state.finishRefresh(original.value_or(0), true));
    EXPECT_FALSE(state.ready());
    const auto replacement = state.beginRefresh();
    ASSERT_TRUE(replacement);
    EXPECT_TRUE(state.finishRefresh(replacement.value_or(0), true));
}

TEST(FruDiscovery, FailedAcquisitionCanBeRetried)
{
    FruDiscoveryState state;
    const auto original = state.beginRefresh();
    ASSERT_TRUE(original);
    EXPECT_FALSE(state.finishRefresh(original.value_or(0), false));
    EXPECT_FALSE(state.ready());
    const auto replacement = state.beginRefresh();
    ASSERT_TRUE(replacement);
    EXPECT_TRUE(state.finishRefresh(replacement.value_or(0), true));
}

TEST(FruDiscovery, IncrementalUpdatesKeepAnInitializedDirectoryReady)
{
    FruDiscoveryState state;
    const auto initial = state.beginRefresh();
    ASSERT_TRUE(initial);
    EXPECT_TRUE(state.finishRefresh(initial.value_or(0), true));
    const auto before = state.revision();
    state.changed();
    EXPECT_NE(before, state.revision());
    EXPECT_TRUE(state.ready());
    state.invalidate();
    EXPECT_FALSE(state.ready());
}
} // namespace
