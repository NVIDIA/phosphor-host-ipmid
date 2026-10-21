#include "dbus-sdr/sdrutils.hpp"

#include "gtest/gtest.h"

/**
 * @brief Tests sdr dbus to IPMI
 */
TEST(sdrUtils, TranslateToIPMI)
{
    std::string name1 = "psu1";
    std::string name2 = "TEMP_M.2_2";
    std::string name3 = "TEMPP3V3_4";
    std::string name4 = "sel_event";
    std::string name5 = "psu12";

    // Expected Entity Instance from sensor name
    EXPECT_EQ(getEntityInstanceFromName(name1), 1);
    EXPECT_EQ(getEntityInstanceFromName(name2), 2);
    EXPECT_EQ(getEntityInstanceFromName(name3), 4);
    EXPECT_EQ(getEntityInstanceFromName(name4), 1);
    EXPECT_EQ(getEntityInstanceFromName(name5), 12);
}

TEST(sdrUtils, SensorNumberIncludesGeneratorLun)
{
    // IPMB generator: the second generator-ID byte carries the LUN.
    EXPECT_EQ(getSensorNumberWithLun(0x0020, 0x01), 0x0001);
    EXPECT_EQ(getSensorNumberWithLun(0x0120, 0x01), 0x0101);
    EXPECT_EQ(getSensorNumberWithLun(0x0320, 0x13), 0x0313);

    // System-software generators do not carry a LUN.
    EXPECT_EQ(getSensorNumberWithLun(0x0041, 0x01), 0x0001);
    EXPECT_EQ(getSensorNumberWithLun(0x0043, 0x13), 0x0013);
}
