// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES.
#include "dbus-sdr/stable_ids.hpp"

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

using ipmi::sensor::StableIds;

TEST(StableIds, DenseAllocationUsesTheDeclaredPoolOrder)
{
    StableIds<std::string, uint16_t> ids({0, 1, 2, 3});
    EXPECT_EQ(ids.allocate("A"), 0);
    EXPECT_EQ(ids.allocate("B"), 1);
    EXPECT_EQ(ids.allocate("C"), 2);
}

TEST(StableIds, NoncollidingIdentitiesDoNotDependOnDiscoveryOrder)
{
    StableIds<std::string, uint16_t> ab({0, 1, 2, 3});
    StableIds<std::string, uint16_t> ba({0, 1, 2, 3});
    const auto a = ab.allocate("A", 0);
    const auto b = ab.allocate("B", 1);
    EXPECT_EQ(ba.allocate("B", 1), b);
    EXPECT_EQ(ba.allocate("A", 0), a);
}

TEST(StableIds, CollisionDoesNotRemapAnAdvertisedIdentity)
{
    StableIds<std::string, uint16_t> ids({0, 1});
    EXPECT_EQ(ids.allocate("A", 0), 0);
    EXPECT_EQ(ids.allocate("B", 0), 1);
    EXPECT_EQ(ids.allocate("A", 1), 0);
    EXPECT_FALSE(ids.allocate("C", 0));
    EXPECT_EQ(ids.allocate("B", 0), 1);
}

TEST(StableIds, DiscoveryBeforeExistingObjectsDoesNotChangeTheirIds)
{
    StableIds<std::string, uint16_t> ids({0, 1, 2, 3});
    EXPECT_EQ(ids.allocate("sensor_B"), 0);
    EXPECT_EQ(ids.allocate("sensor_C"), 1);
    EXPECT_EQ(ids.allocate("sensor_A"), 2);
    EXPECT_EQ(ids.allocate("sensor_B"), 0);
    EXPECT_EQ(ids.allocate("sensor_C"), 1);
}

TEST(StableIds, RemovalAndRediscoveryNeverReassignAnAdvertisedId)
{
    StableIds<std::string, uint8_t> ids({1, 2, 3});
    EXPECT_EQ(ids.allocate("FRU_A", 2), 2);
    // A disappears from the active directory. Its allocation remains reserved.
    EXPECT_EQ(ids.allocate("FRU_B", 2), 3);
    EXPECT_EQ(ids.allocate("FRU_A", 1), 2);
    EXPECT_EQ(ids.allocate("FRU_C", 3), 1);
    EXPECT_FALSE(ids.allocate("FRU_D"));
    EXPECT_EQ(ids.allocate("FRU_B"), 3);
}

TEST(StableIds, SensorPoolSkipsReservedNumbersAndLunTwo)
{
    std::vector<uint16_t> pool;
    for (const uint16_t lun : {0, 1, 3})
    {
        for (uint16_t number = 0; number < 0xff; ++number)
        {
            pool.push_back((lun << 8) | number);
        }
    }
    StableIds<std::string, uint16_t> ids(pool);
    for (size_t index = 0; index < pool.size(); ++index)
    {
        EXPECT_EQ(ids.allocate(std::to_string(index)), pool[index]);
    }
    EXPECT_FALSE(ids.allocate("overflow"));
    EXPECT_EQ(ids.allocate("255"), 0x100);
    EXPECT_EQ(ids.allocate("510"), 0x300);
}

TEST(StableIds, EmptyPoolDoesNotLoopOrDivideByZero)
{
    StableIds<std::string, uint8_t> ids({});
    EXPECT_FALSE(ids.allocate("FRU", 7));
    EXPECT_FALSE(ids.allocate("FRU", 7));
}
