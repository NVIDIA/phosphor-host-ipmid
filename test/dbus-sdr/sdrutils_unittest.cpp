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
