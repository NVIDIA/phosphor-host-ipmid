#include "dbus-sdr/sdrutils.hpp"

#include "gtest/gtest.h"

using Level = sdbusplus::xyz::openbmc_project::Logging::server::Entry::Level;

TEST(selSeverity, AssertedThresholdExcursionsCarrySeverity)
{
    EXPECT_EQ(ipmi::getSelEventSeverity(0x01, 0x00, true), Level::Warning);
    EXPECT_EQ(ipmi::getSelEventSeverity(0x01, 0x07, true), Level::Warning);
    EXPECT_EQ(ipmi::getSelEventSeverity(0x01, 0x02, true), Level::Critical);
    EXPECT_EQ(ipmi::getSelEventSeverity(0x01, 0x09, true), Level::Critical);
    EXPECT_EQ(ipmi::getSelEventSeverity(0x01, 0x04, true), Level::Alert);
    EXPECT_EQ(ipmi::getSelEventSeverity(0x01, 0x0b, true), Level::Alert);
}

TEST(selSeverity, ReturnsTowardNormalStayInformational)
{
    for (uint8_t offset : {0x01, 0x03, 0x05, 0x06, 0x08, 0x0a})
    {
        EXPECT_EQ(ipmi::getSelEventSeverity(0x01, offset, true),
                  Level::Informational);
    }
}

TEST(selSeverity, DeassertionsStayInformational)
{
    for (uint8_t offset : {0x00, 0x02, 0x04, 0x07, 0x09, 0x0b})
    {
        EXPECT_EQ(ipmi::getSelEventSeverity(0x01, offset, false),
                  Level::Informational);
    }
}

TEST(selSeverity, NonThresholdEventTypesStayInformational)
{
    EXPECT_EQ(ipmi::getSelEventSeverity(0x6f, 0x09, true),
              Level::Informational);
    EXPECT_EQ(ipmi::getSelEventSeverity(0x03, 0x02, true),
              Level::Informational);
}

TEST(selSeverity, OffsetsOutsideTheThresholdTableStayInformational)
{
    EXPECT_EQ(ipmi::getSelEventSeverity(0x01, 0x0c, true),
              Level::Informational);
    EXPECT_EQ(ipmi::getSelEventSeverity(0x01, 0x0f, true),
              Level::Informational);
}
