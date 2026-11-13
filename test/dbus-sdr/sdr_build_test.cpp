// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#include "dbus-sdr/sdr_build.hpp"

#include <chrono>

#include <gtest/gtest.h>

namespace
{
using namespace std::chrono_literals;
using ipmi::sensor::SdrBuildRecovery;

TEST(SdrBuildRecovery, PersistentAcquisitionFailureStopsAfterFiveAttempts)
{
    SdrBuildRecovery recovery;
    EXPECT_EQ(recovery.failed(true), 1s);
    EXPECT_EQ(recovery.failed(true), 2s);
    EXPECT_EQ(recovery.failed(true), 4s);
    EXPECT_EQ(recovery.failed(true), 8s);
    EXPECT_FALSE(recovery.failed(true));
    EXPECT_TRUE(recovery.degraded());
    EXPECT_FALSE(recovery.failed(true));
}

TEST(SdrBuildRecovery, NewInputsOrSuccessRestoreTheRetryBudget)
{
    SdrBuildRecovery recovery(2);
    ASSERT_TRUE(recovery.failed(true));
    EXPECT_FALSE(recovery.failed(true));
    recovery.reset();
    EXPECT_FALSE(recovery.degraded());
    EXPECT_EQ(recovery.failed(true), 1s);
}

TEST(SdrBuildRecovery, UnexpectedFailureWaitsForNewInputs)
{
    SdrBuildRecovery recovery;
    EXPECT_FALSE(recovery.failed(false));
    EXPECT_TRUE(recovery.degraded());
    EXPECT_FALSE(recovery.failed(true));
    recovery.reset();
    EXPECT_EQ(recovery.failed(true), 1s);
}

TEST(SdrBuildRecovery, DelayIsCappedForAnExplicitlyLargerBudget)
{
    SdrBuildRecovery recovery(40);
    for (size_t attempt = 0; attempt < 39; ++attempt)
    {
        auto delay = recovery.failed(true);
        ASSERT_TRUE(delay);
        EXPECT_LE(delay.value_or(0s), 30s);
    }
    EXPECT_FALSE(recovery.failed(true));
}

TEST(SdrBuildRecovery, ZeroLimitStillAllowsOneAttempt)
{
    SdrBuildRecovery recovery(0);
    EXPECT_FALSE(recovery.failed(true));
    EXPECT_TRUE(recovery.degraded());
}
} // namespace
