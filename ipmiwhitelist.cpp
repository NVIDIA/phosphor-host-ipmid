#include <ipmiallowlist.hpp>

const std::vector<netfncmd_pair> allowlist = {

    {0x00, 0x00}, //<Chassis>:<Chassis Capabiliti>
    {0x00, 0x01}, //<Chassis>:<Get Chassis Status>
    {0x00, 0x02}, //<Chassis>:<Chassis Control>
    {0x00, 0x05}, //<Chassis>:<Set Chassis Capabilities>
    {0x00, 0x06}, //<Chassis>:<Set Power Restore Policy>
    {0x00, 0x08}, //<Chassis>:<Set System Boot Options>
    {0x00, 0x09}, //<Chassis>:<Get System Boot Options>
    {0x00, 0x0F}, //<Chassis>:<Get POH Counter Command>
    {0x04, 0x02}, //<Sensor/Event>:<Platform event>
    {0x04, 0x2D}, //<Sensor/Event>:<Get Sensor Reading>
    {0x04, 0x2F}, //<Sensor/Event>:<Get Sensor Type>
    {0x04, 0x30}, //<Sensor/Event>:<Set Sensor Reading and Event Status>
    {0x06, 0x01}, //<App>:<Get Device ID>
    {0x06, 0x04}, //<App>:<Get Self Test Results>
    {0x06, 0x06}, //<App>:<Set ACPI Power State>
    {0x06, 0x07}, //<App>:<Get ACPI Power State>
    {0x06, 0x08}, //<App>:<Get Device GUID>
    {0x06, 0x22}, //<App>:<Reset Watchdog Timer>
    {0x06, 0x24}, //<App>:<Set Watchdog Timer>
    {0x06, 0x25}, //<App>:<Get Watchdog Timer>
    {0x06, 0x2E}, //<App>:<Set BMC Global Enables>
    {0x06, 0x2F}, //<App>:<Get BMC Global Enables>
    {0x06, 0x31}, //<App>:<Get Message Flags>
    {0x06, 0x35}, //<App>:<Read Event Message Buffer>
    {0x06, 0x36}, //<App>:<Get BT Interface Capabilities>
    {0x06, 0x37}, //<App>:<Get System GUID>
    {0x06, 0x42}, //<App>:<Get Channel Info Command>
    {0x06, 0x4D}, //<App>:<Get User Payload Access>
    {0x06, 0x4E}, //<App>:<Get Channel Payload Support>
    {0x06, 0x4F}, //<App>:<Get Channel Payload Version>
    {0x06, 0x54}, //<App>:<Get Channel Cipher Suites>
    {0x0A, 0x10}, //<Storage>:<Get FRU Inventory Area Info>
    {0x0A, 0x11}, //<Storage>:<Read FRU Data>
    {0x0A, 0x20}, //<Storage>:<Get SDR Repository Info>
    {0x0A, 0x22}, //<Storage>:<Reserve SDR Repository>
    {0x0A, 0x23}, //<Storage>:<Get SDR>
    {0x0A, 0x40}, //<Storage>:<Get SEL Info>
    {0x0A, 0x42}, //<Storage>:<Reserve SEL>
    {0x0A, 0x44}, //<Storage>:<Add SEL Entry>
    {0x0A, 0x48}, //<Storage>:<Get SEL Time>
    {0x0A, 0x49}, //<Storage>:<Set SEL Time>
    {0x0A, 0x5C}, //<Storage>:<Get SEL Time UTC Offset>
    {0x0C, 0x02}, //<Transport>:<Get LAN Configuration Parameters>
    {0x0C, 0x21}, //<Transport>:<Set SOL Configuration Parameters>
    {0x0C, 0x22}, //<Transport>:<Get SOL Configuration Parameters>
    {0x2C, 0x00}, //<Group Extension>:<Group Extension Command>
    {0x2C, 0x01}, //<Group Extension>:<Get DCMI Capabilities>
    {0x2C, 0x02}, //<Group Extension>:<Get Power Reading>
    {0x2C, 0x03}, //<Group Extension>:<Get Power Limit>
    {0x2C, 0x06}, //<Group Extension>:<Get Asset Tag>
    {0x2C, 0x07}, //<Group Extension>:<Get Sensor Info>
    {0x2C, 0x10}, //<Group Extension>:<Get Temperature Readings>
};
