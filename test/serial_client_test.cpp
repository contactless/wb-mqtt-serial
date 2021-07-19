#include "serial_driver.h"
#include "fake_serial_port.h"
#include "fake_serial_device.h"
#include "log.h"

#include <wblib/testing/fake_driver.h>
#include <wblib/testing/fake_mqtt.h>
#include <wblib/driver_args.h>

#include <string>
#include <map>
#include <memory>
#include <algorithm>
#include <cassert>
#include <gtest/gtest.h>

#include "tcp_port_settings.h"

using namespace std;
using namespace WBMQTT;
using namespace WBMQTT::Testing;

#define LOG(logger) ::logger.Log() << "[serial client test] "

class TSerialClientTest: public TLoggedFixture
{
protected:
    TSerialClientTest()
    {
        PortOpenCloseSettings.ReopenTimeout = std::chrono::milliseconds(0);
    }

    void SetUp();
    void TearDown();
    PRegister Reg(int addr, RegisterFormat fmt = U16, double scale = 1,
        double offset = 0, double round_to = 0, EWordOrder word_order = EWordOrder::BigEndian) {
        return TRegister::Intern(
            Device, TRegisterConfig::Create(
            TFakeSerialDevice::REG_FAKE, addr, fmt, scale, offset, round_to, true, false,
            "fake", std::unique_ptr<uint64_t>(), word_order));
    }
    PFakeSerialPort Port;
    PSerialClient SerialClient;
    PFakeSerialDevice Device;
    TSerialDeviceFactory DeviceFactory;

    bool HasSetupRegisters = false;
    TPortOpenCloseLogic::TSettings PortOpenCloseSettings;
};

class TSerialClientReopenTest: public TSerialClientTest
{
public:
    TSerialClientReopenTest()
    {
        PortOpenCloseSettings.ReopenTimeout = std::chrono::milliseconds(700);
    }
};

class TSerialClientTestWithSetupRegisters: public TSerialClientTest
{
public:
    TSerialClientTestWithSetupRegisters()
    {
        HasSetupRegisters = true;
    }
};

void TSerialClientTest::SetUp()
{
    RegisterProtocols(DeviceFactory);
    TFakeSerialDevice::Register(DeviceFactory);

    TLoggedFixture::SetUp();
    Port = std::make_shared<TFakeSerialPort>(*this);
    auto config = std::make_shared<TDeviceConfig>("fake_sample", "1", "fake");
    config->MaxReadRegisters = 0;

    if (HasSetupRegisters) {
        PRegisterConfig reg1 = TRegisterConfig::Create(TFakeSerialDevice::REG_FAKE, 100);
        config->AddSetupItem(PDeviceSetupItemConfig(new TDeviceSetupItemConfig("setup1", reg1, "10")));

        PRegisterConfig reg2 = TRegisterConfig::Create(TFakeSerialDevice::REG_FAKE, 101);
        config->AddSetupItem(PDeviceSetupItemConfig(new TDeviceSetupItemConfig("setup2", reg2, "11")));

        PRegisterConfig reg3 = TRegisterConfig::Create(TFakeSerialDevice::REG_FAKE, 102);
        config->AddSetupItem(PDeviceSetupItemConfig(new TDeviceSetupItemConfig("setup3", reg3, "12")));
    }

    config->FrameTimeout = std::chrono::milliseconds(100);
    Device = std::make_shared<TFakeSerialDevice>(config, Port, DeviceFactory.GetProtocol("fake"));
    Device->InitSetupItems();
    std::vector<PSerialDevice> devices;
    devices.push_back(Device);
    SerialClient = std::make_shared<TSerialClient>(devices, Port, PortOpenCloseSettings);
#if 0
    SerialClient->SetModbusDebug(true);
#endif
    SerialClient->SetReadCallback([this](PRegister reg, bool changed) {
            Emit() << "Read Callback: <"
                   << reg->Device()->ToString() << ":" << reg->TypeName << ": " << reg->GetAddress() << "> becomes "
                   << SerialClient->GetTextValue(reg) << (changed ? "" : " [unchanged]");
        });
    SerialClient->SetErrorCallback(
        [this](PRegister reg, TRegisterHandler::TErrorState errorState) {
            const char* what;
            switch (errorState) {
            case TRegisterHandler::WriteError:
                what = "write error";
                break;
            case TRegisterHandler::ReadError:
                what = "read error";
                break;
            case TRegisterHandler::ReadWriteError:
                what = "read+write error";
                break;
            default:
                what = "no error";
            }
            Emit() << "Error Callback: <" << reg->Device()->ToString() << ":" << reg->TypeName << ": " << reg->GetAddress() << ">: " << what;
        });
}

void TSerialClientTest::TearDown()
{
    TLoggedFixture::TearDown();
    TRegister::DeleteIntern();
    TFakeSerialDevice::ClearDevices();
}

TEST_F(TSerialClientTest, PortOpenError)
{
    // The test checks recovery logic after port opening failure
    // TSerialClient must try to open port during every cycle and set /meta/error for controls

    PRegister reg0 = Reg(0, U8);
    PRegister reg1 = Reg(1, U8);

    SerialClient->AddRegister(reg0);
    SerialClient->AddRegister(reg1);

    Port->SetAllowOpen(false);

    Note() << "Cycle() [port open error]";
    SerialClient->Cycle();

    Note() << "Cycle() [port open error2]";
    SerialClient->Cycle();

    Port->SetAllowOpen(true);

    Note() << "Cycle() [successful port open]";
    SerialClient->Cycle();
}

TEST_F(TSerialClientReopenTest, ReopenTimeout)
{
    // The test checks recovery logic after port opening failure
    // TSerialClient must try to open port during every cycle and set /meta/error for controls

    PRegister reg0 = Reg(0, U8);
    PRegister reg1 = Reg(1, U8);

    SerialClient->AddRegister(reg0);
    SerialClient->AddRegister(reg1);

    Port->SetAllowOpen(false);

    Note() << "Cycle() [port open error]";
    SerialClient->Cycle();

    Port->SetAllowOpen(true);

    // The open will be unsuccessful because of 700ms timeout since last failed open
    Note() << "Cycle() [port open error2]";
    SerialClient->Cycle();

    Note() << "Cycle() [successful port open]";
    SerialClient->Cycle();
}

TEST_F(TSerialClientTest, Poll)
{

    PRegister reg0 = Reg(0, U8);
    PRegister reg1 = Reg(1, U8);
    PRegister discrete10 = Reg(10);
    PRegister reg22 = Reg(22);
    PRegister reg33 = Reg(33);

    SerialClient->AddRegister(reg0);
    SerialClient->AddRegister(reg1);
    SerialClient->AddRegister(discrete10);
    SerialClient->AddRegister(reg22);
    SerialClient->AddRegister(reg33);

    Note() << "Cycle()";
    SerialClient->Cycle();

    Device->Registers[1] = 1;
    Device->Registers[10] = 1;
    Device->Registers[22] = 4242;
    Device->Registers[33] = 42000;

    Note() << "Cycle()";
    SerialClient->Cycle();

    EXPECT_EQ(to_string(0), SerialClient->GetTextValue(reg0));
    EXPECT_EQ(to_string(1), SerialClient->GetTextValue(reg1));
    EXPECT_EQ(to_string(1), SerialClient->GetTextValue(discrete10));
    EXPECT_EQ(to_string(4242), SerialClient->GetTextValue(reg22));
    EXPECT_EQ(to_string(42000), SerialClient->GetTextValue(reg33));
}

TEST_F(TSerialClientTest, Write)
{
    PRegister reg1 = Reg(1);
    PRegister reg20 = Reg(20);
    SerialClient->AddRegister(reg1);
    SerialClient->AddRegister(reg20);

    Note() << "Cycle()";
    SerialClient->Cycle();

    SerialClient->SetTextValue(reg1, "1");
    SerialClient->SetTextValue(reg20, "4242");

    for (int i = 0; i < 3; ++i) {
        Note() << "Cycle()";
        SerialClient->Cycle();

        EXPECT_EQ(to_string(1), SerialClient->GetTextValue(reg1));
        EXPECT_EQ(1, Device->Registers[1]);
        EXPECT_EQ(to_string(4242), SerialClient->GetTextValue(reg20));
        EXPECT_EQ(4242, Device->Registers[20]);
    }
}

TEST_F(TSerialClientTest, S8)
{
    PRegister reg20 = Reg(20, S8);
    PRegister reg30 = Reg(30, S8);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    Note() << "server -> client: 10, 20";
    Device->Registers[20] = 10;
    Device->Registers[30] = 20;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(10), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(to_string(20), SerialClient->GetTextValue(reg30));

    Note() << "server -> client: -2, -3";
    Device->Registers[20] = 254;
    Device->Registers[30] = 253;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(-2), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(to_string(-3), SerialClient->GetTextValue(reg30));

    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg20, "10");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(10), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(10, Device->Registers[20]);

    Note() << "client -> server: -2";
    SerialClient->SetTextValue(reg20, "-2");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(-2), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(254, Device->Registers[20]);
}

TEST_F(TSerialClientTest, Char8)
{
    PRegister reg20 = Reg(20, Char8);
    PRegister reg30 = Reg(30, Char8);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    Note() << "server -> client: 65, 66";
    Device->Registers[20] = 65;
    Device->Registers[30] = 66;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("A", SerialClient->GetTextValue(reg20));
    EXPECT_EQ("B", SerialClient->GetTextValue(reg30));

    Note() << "client -> server: '!'";
    SerialClient->SetTextValue(reg20, "!");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("!", SerialClient->GetTextValue(reg20));
    EXPECT_EQ(33, Device->Registers[20]);
}

TEST_F(TSerialClientTest, S64)
{
    PRegister reg20 = Reg(20, S64);
    PRegister reg30 = Reg(30, S64);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    Note() << "server -> client: 10, 20";
    Device->Registers[20] = 0x00AA;
    Device->Registers[21] = 0x00BB;
    Device->Registers[22] = 0x00CC;
    Device->Registers[23] = 0x00DD;
    Device->Registers[30] = 0xFFFF;
    Device->Registers[31] = 0xFFFF;
    Device->Registers[32] = 0xFFFF;
    Device->Registers[33] = 0xFFFF;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0x00AA00BB00CC00DD), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(to_string(-1), SerialClient->GetTextValue(reg30));

    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg20, "10");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(10), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0, Device->Registers[20]);
    EXPECT_EQ(0, Device->Registers[21]);
    EXPECT_EQ(0, Device->Registers[22]);
    EXPECT_EQ(10, Device->Registers[23]);

    Note() << "client -> server: -2";
    SerialClient->SetTextValue(reg20, "-2");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(-2), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0xFFFF, Device->Registers[20]);
    EXPECT_EQ(0xFFFF, Device->Registers[21]);
    EXPECT_EQ(0xFFFF, Device->Registers[22]);
    EXPECT_EQ(0xFFFE, Device->Registers[23]);
}

TEST_F(TSerialClientTest, U64)
{
    PRegister reg20 = Reg(20, U64);
    PRegister reg30 = Reg(30, U64);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    Note() << "server -> client: 10, 20";
    Device->Registers[20] = 0x00AA;
    Device->Registers[21] = 0x00BB;
    Device->Registers[22] = 0x00CC;
    Device->Registers[23] = 0x00DD;
    Device->Registers[30] = 0xFFFF;
    Device->Registers[31] = 0xFFFF;
    Device->Registers[32] = 0xFFFF;
    Device->Registers[33] = 0xFFFF;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0x00AA00BB00CC00DD), SerialClient->GetTextValue(reg20));
    EXPECT_EQ("18446744073709551615", SerialClient->GetTextValue(reg30));

    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg20, "10");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(10), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0, Device->Registers[20]);
    EXPECT_EQ(0, Device->Registers[21]);
    EXPECT_EQ(0, Device->Registers[22]);
    EXPECT_EQ(10, Device->Registers[23]);

    Note() << "client -> server: -2";
    SerialClient->SetTextValue(reg20, "-2");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("18446744073709551614", SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0xFFFF, Device->Registers[20]);
    EXPECT_EQ(0xFFFF, Device->Registers[21]);
    EXPECT_EQ(0xFFFF, Device->Registers[22]);
    EXPECT_EQ(0xFFFE, Device->Registers[23]);
}

TEST_F(TSerialClientTest, S32)
{
    PRegister reg20 = Reg(20, S32);
    PRegister reg30 = Reg(30, S32);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    // create scaled register
    PRegister reg24 = Reg(24, S32, 0.001);
    SerialClient->AddRegister(reg24);

    Note() << "server -> client: 10, 20";
    Device->Registers[20] = 0x00AA;
    Device->Registers[21] = 0x00BB;
    Device->Registers[30] = 0xFFFF;
    Device->Registers[31] = 0xFFFF;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0x00AA00BB), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(to_string(-1), SerialClient->GetTextValue(reg30));

    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg20, "10");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(10), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0, Device->Registers[20]);
    EXPECT_EQ(10, Device->Registers[21]);

    Note() << "client -> server: -2";
    SerialClient->SetTextValue(reg20, "-2");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(-2), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0xFFFF, Device->Registers[20]);
    EXPECT_EQ(0xFFFE, Device->Registers[21]);

    Note() << "client -> server: -0.123 (scaled)";
    SerialClient->SetTextValue(reg24, "-0.123");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-0.123", SerialClient->GetTextValue(reg24));
    EXPECT_EQ(0xffff, Device->Registers[24]);
    EXPECT_EQ(0xff85, Device->Registers[25]);

    Note() << "server -> client: 0xffff 0xff85 (scaled)";
    Device->Registers[24] = 0xffff;
    Device->Registers[25] = 0xff85;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-0.123", SerialClient->GetTextValue(reg24));
}

TEST_F(TSerialClientTest, S24)
{
    PRegister reg20 = Reg(20, S24);
    PRegister reg30 = Reg(30, S24);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    // create scaled register
    PRegister reg24 = Reg(24, S24, 0.001);
    SerialClient->AddRegister(reg24);

    Note() << "server -> client: 10, 20";
    Device->Registers[20] = 0x002A;
    Device->Registers[21] = 0x00BB;
    Device->Registers[30] = 0x00FF;
    Device->Registers[31] = 0xFFFF;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0x002A00BB), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(to_string(-1), SerialClient->GetTextValue(reg30));

    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg20, "10");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(10), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0, Device->Registers[20]);
    EXPECT_EQ(10, Device->Registers[21]);

    Note() << "client -> server: -2";
    SerialClient->SetTextValue(reg20, "-2");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(-2), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0x00FF, Device->Registers[20]);
    EXPECT_EQ(0xFFFE, Device->Registers[21]);

    Note() << "client -> server: -0.123 (scaled)";
    SerialClient->SetTextValue(reg24, "-0.123");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-0.123", SerialClient->GetTextValue(reg24));
    EXPECT_EQ(0x00FF, Device->Registers[24]);
    EXPECT_EQ(0xFF85, Device->Registers[25]);

    Note() << "server -> client: 0x00ff 0xff85 (scaled)";
    Device->Registers[24] = 0x00FF;
    Device->Registers[25] = 0xFF85;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-0.123", SerialClient->GetTextValue(reg24));
}

TEST_F(TSerialClientTest, WordSwap)
{
    PRegister reg20 = Reg(20, S32, 1, 0, 0, EWordOrder::LittleEndian);
    PRegister reg24 = Reg(24, U64, 1, 0, 0, EWordOrder::LittleEndian);
    SerialClient->AddRegister(reg24);
    SerialClient->AddRegister(reg20);

    Note() << "server -> client: 0x00BB, 0x00AA";
    Device->Registers[20] = 0x00BB;
    Device->Registers[21] = 0x00AA;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0x00AA00BB), SerialClient->GetTextValue(reg20));

    Note() << "client -> server: -2";
    SerialClient->SetTextValue(reg20, "-2");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(-2), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0xFFFE, Device->Registers[20]);
    EXPECT_EQ(0xFFFF, Device->Registers[21]);

    // U64
    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg24, "47851549213065437"); // 0x00AA00BB00CC00DD
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0x00AA00BB00CC00DD), SerialClient->GetTextValue(reg24));
    EXPECT_EQ(0x00DD, Device->Registers[24]);
    EXPECT_EQ(0x00CC, Device->Registers[25]);
    EXPECT_EQ(0x00BB, Device->Registers[26]);
    EXPECT_EQ(0x00AA, Device->Registers[27]);

}

TEST_F(TSerialClientTest, U32)
{
    PRegister reg20 = Reg(20, U32);
    PRegister reg30 = Reg(30, U32);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    Note() << "server -> client: 10, 20";
    Device->Registers[20] = 0x00AA;
    Device->Registers[21] = 0x00BB;
    Device->Registers[30] = 0xFFFF;
    Device->Registers[31] = 0xFFFF;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0x00AA00BB), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(to_string(0xFFFFFFFF), SerialClient->GetTextValue(reg30));

    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg20, "10");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(10), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0, Device->Registers[20]);
    EXPECT_EQ(10, Device->Registers[21]);

    Note() << "client -> server: -1 (overflow)";
    SerialClient->SetTextValue(reg20, "-1");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0xFFFFFFFF), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0xFFFF, Device->Registers[20]);
    EXPECT_EQ(0xFFFF, Device->Registers[21]);

    Device->Registers[22] = 123;
    Device->Registers[23] = 123;
    Note() << "client -> server: 4294967296 (overflow)";
    SerialClient->SetTextValue(reg20, "4294967296");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(0), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0, Device->Registers[20]);
    EXPECT_EQ(0, Device->Registers[21]);

    //boundaries check
    EXPECT_EQ(123, Device->Registers[22]);
    EXPECT_EQ(123, Device->Registers[23]);
}

TEST_F(TSerialClientTest, BCD32)
{
    PRegister reg20 = Reg(20, BCD32);
    SerialClient->AddRegister(reg20);
    Device->Registers[22] = 123;
    Device->Registers[23] = 123;

    Note() << "server -> client: 0x1234 0x5678";
    Device->Registers[20] = 0x1234;
    Device->Registers[21] = 0x5678;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(12345678), SerialClient->GetTextValue(reg20));

    Note() << "client -> server: 12345678";
    SerialClient->SetTextValue(reg20, "12345678");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(12345678), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0x1234, Device->Registers[20]);
    EXPECT_EQ(0x5678, Device->Registers[21]);

    Note() << "client -> server: 567890";
    SerialClient->SetTextValue(reg20, "567890");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(567890), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0x0056, Device->Registers[20]);
    EXPECT_EQ(0x7890, Device->Registers[21]);

    //boundaries check
    EXPECT_EQ(123, Device->Registers[22]);
    EXPECT_EQ(123, Device->Registers[23]);
}

TEST_F(TSerialClientTest, BCD24)
{
    PRegister reg20 = Reg(20, BCD24);
    SerialClient->AddRegister(reg20);
    Device->Registers[22] = 123;
    Device->Registers[23] = 123;

    Note() << "server -> client: 0x0034 0x5678";
    Device->Registers[20] = 0x0034;
    Device->Registers[21] = 0x5678;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(345678), SerialClient->GetTextValue(reg20));

    Note() << "client -> server: 567890";
    SerialClient->SetTextValue(reg20, "567890");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(567890), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0x0056, Device->Registers[20]);
    EXPECT_EQ(0x7890, Device->Registers[21]);

    //boundaries check
    EXPECT_EQ(123, Device->Registers[22]);
    EXPECT_EQ(123, Device->Registers[23]);
}

TEST_F(TSerialClientTest, BCD16)
{
    PRegister reg20 = Reg(20, BCD16);
    SerialClient->AddRegister(reg20);
    Device->Registers[21] = 123;

    Note() << "server -> client: 0x1234";
    Device->Registers[20] = 0x1234;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(1234), SerialClient->GetTextValue(reg20));

    Note() << "client -> server: 1234";
    SerialClient->SetTextValue(reg20, "1234");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(1234), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0x1234, Device->Registers[20]);

    //boundaries check
    EXPECT_EQ(123, Device->Registers[21]);
}

TEST_F(TSerialClientTest, BCD8)
{
    PRegister reg20 = Reg(20, BCD8);
    SerialClient->AddRegister(reg20);
    Device->Registers[21] = 123;

    Note() << "server -> client: 0x12";
    Device->Registers[20] = 0x12;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(12), SerialClient->GetTextValue(reg20));

    Note() << "client -> server: 12";
    SerialClient->SetTextValue(reg20, "12");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ(to_string(12), SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0x12, Device->Registers[20]);

    //boundaries check
    EXPECT_EQ(123, Device->Registers[21]);
}

TEST_F(TSerialClientTest, Float32)
{
	// create scaled register
    PRegister reg24 = Reg(24, Float, 100);
    SerialClient->AddRegister(reg24);

    PRegister reg20 = Reg(20, Float);
    PRegister reg30 = Reg(30, Float);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    Note() << "server -> client: 0x45d2 0x0000, 0x449d 0x8000";
    Device->Registers[20] = 0x45d2;
    Device->Registers[21] = 0x0000;
    Device->Registers[30] = 0x449d;
    Device->Registers[31] = 0x8000;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("6720", SerialClient->GetTextValue(reg20));
    EXPECT_EQ("1260", SerialClient->GetTextValue(reg30));

    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg20, "10");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("10", SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0x4120, Device->Registers[20]);
    EXPECT_EQ(0x0000, Device->Registers[21]);

    Note() << "client -> server: -0.00123";
    SerialClient->SetTextValue(reg20, "-0.00123");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-0.00123", SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0xbaa1, Device->Registers[20]);
    EXPECT_EQ(0x37f4, Device->Registers[21]);

    Note() << "client -> server: -0.123 (scaled)";
    SerialClient->SetTextValue(reg24, "-0.123");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-0.123", SerialClient->GetTextValue(reg24));
    EXPECT_EQ(0xbaa1, Device->Registers[24]);
    EXPECT_EQ(0x37f4, Device->Registers[25]);

    Note() << "server -> client: 0x449d 0x8000 (scaled)";
    Device->Registers[24] = 0x449d;
    Device->Registers[25] = 0x8000;
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("126000", SerialClient->GetTextValue(reg24));
}

TEST_F(TSerialClientTest, Double64)
{
	// create scaled register
    PRegister reg24 = Reg(24, Double, 100);
    SerialClient->AddRegister(reg24);

    PRegister reg20 = Reg(20, Double);
    PRegister reg30 = Reg(30, Double);
    SerialClient->AddRegister(reg20);
    SerialClient->AddRegister(reg30);

    Note() << "server -> client: 40ba401f7ced9168 , 4093b148b4395810";
    Device->Registers[20] = 0x40ba;
    Device->Registers[21] = 0x401f;
    Device->Registers[22] = 0x7ced;
    Device->Registers[23] = 0x9168;

    Device->Registers[30] = 0x4093;
    Device->Registers[31] = 0xb148;
    Device->Registers[32] = 0xb439;
    Device->Registers[33] = 0x5810;

    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("6720.123", SerialClient->GetTextValue(reg20));
    EXPECT_EQ("1260.321", SerialClient->GetTextValue(reg30));

    Note() << "client -> server: 10";
    SerialClient->SetTextValue(reg20, "10.9999");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("10.9999", SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0x4025, Device->Registers[20]);
    EXPECT_EQ(0xfff2, Device->Registers[21]);
    EXPECT_EQ(0xe48e, Device->Registers[22]);
    EXPECT_EQ(0x8a72, Device->Registers[23]);

    Note() << "client -> server: -0.00123";
    SerialClient->SetTextValue(reg20, "-0.00123");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-0.00123", SerialClient->GetTextValue(reg20));
    EXPECT_EQ(0xbf54, Device->Registers[20]);
    EXPECT_EQ(0x26fe, Device->Registers[21]);
    EXPECT_EQ(0x718a, Device->Registers[22]);
    EXPECT_EQ(0x86d7, Device->Registers[23]);

    Note() << "client -> server: -0.123 (scaled)";
    SerialClient->SetTextValue(reg24, "-0.123");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-0.123", SerialClient->GetTextValue(reg24));
    EXPECT_EQ(0xbf54, Device->Registers[24]);
    EXPECT_EQ(0x26fe, Device->Registers[25]);
    EXPECT_EQ(0x718a, Device->Registers[26]);
    EXPECT_EQ(0x86d7, Device->Registers[27]);

    Note() << "server -> client: 4093b00000000000 (scaled)";
    Device->Registers[24] = 0x4093;
    Device->Registers[25] = 0xb000;
    Device->Registers[26] = 0x0000;
    Device->Registers[27] = 0x0000;

    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("126000", SerialClient->GetTextValue(reg24));
}

TEST_F(TSerialClientTest, offset)
{
    // create scaled register with offset
    PRegister reg24 = Reg(24, S16, 3, -15);
    SerialClient->AddRegister(reg24);

    Note() << "client -> server: -87 (scaled)";
    SerialClient->SetTextValue(reg24, "-87");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("-87", SerialClient->GetTextValue(reg24));
    EXPECT_EQ(0xffe8, Device->Registers[24]);
}

TEST_F(TSerialClientTest, Round)
{
    PRegister reg24_0_01 = Reg(24, Float, 1, 0, 0.01);
    PRegister reg26_1 = Reg(26, Float, 1, 0, 1);
    PRegister reg28_10 = Reg(28, Float, 1, 0, 10);
    PRegister reg30_0_2 = Reg(30, Float, 1, 0, 0.2);
    PRegister reg32_0_01 = Reg(32, Float, 1, 0, 0.01);

    SerialClient->AddRegister(reg24_0_01);
    SerialClient->AddRegister(reg26_1);
    SerialClient->AddRegister(reg28_10);
    SerialClient->AddRegister(reg30_0_2);
    SerialClient->AddRegister(reg32_0_01);

    Note() << "client -> server: 12.345 (not rounded)";
    SerialClient->SetTextValue(reg24_0_01, "12.345");
    SerialClient->SetTextValue(reg26_1, "12.345");
    SerialClient->SetTextValue(reg28_10, "12.345");
    SerialClient->SetTextValue(reg30_0_2, "12.345");
    SerialClient->SetTextValue(reg32_0_01, "12.344");
    Note() << "Cycle()";
    SerialClient->Cycle();
    EXPECT_EQ("12.35", SerialClient->GetTextValue(reg24_0_01));
    EXPECT_EQ("12", SerialClient->GetTextValue(reg26_1));
    EXPECT_EQ("10", SerialClient->GetTextValue(reg28_10));
    EXPECT_EQ("12.4", SerialClient->GetTextValue(reg30_0_2));
    EXPECT_EQ("12.34", SerialClient->GetTextValue(reg32_0_01));

    union {
        uint32_t words;
        float value;
    } data;

    data.words = Device->Read2Registers(24);
    ASSERT_EQ(12.35f, data.value);

    data.words = Device->Read2Registers(26);
    ASSERT_EQ(12.f, data.value);

    data.words = Device->Read2Registers(28);
    ASSERT_EQ(10.f, data.value);

    data.words = Device->Read2Registers(30);
    ASSERT_EQ(12.4f, data.value);

    data.words = Device->Read2Registers(32);
    ASSERT_EQ(12.34f, data.value);
}

TEST_F(TSerialClientTest, Errors)
{
    PRegister reg20 = Reg(20);
    SerialClient->AddRegister(reg20);

    Note() << "Cycle() [first start]";
    SerialClient->Cycle();

    Device->BlockReadFor(20, true);
    Device->BlockWriteFor(20, true);

    for (int i = 0; i < 3; i++) {
        Note() << "Cycle() [read, rw blacklisted]";
        SerialClient->Cycle();
    }

    SerialClient->SetTextValue(reg20, "42");
    Note() << "Cycle() [write, rw blacklisted]";
    SerialClient->Cycle();

    Device->BlockWriteFor(20, false);
    SerialClient->SetTextValue(reg20, "42");
    Note() << "Cycle() [write, r blacklisted]";
    SerialClient->Cycle();

    Device->BlockWriteFor(20, true);
    SerialClient->SetTextValue(reg20, "43");
    Note() << "Cycle() [write, rw blacklisted]";
    SerialClient->Cycle();

    Device->BlockReadFor(20, false);
    Note() << "Cycle() [write, w blacklisted]";
    SerialClient->Cycle();

    Device->BlockWriteFor(20, false);
    SerialClient->SetTextValue(reg20, "42");
    Note() << "Cycle() [write, nothing blacklisted]";
    SerialClient->Cycle();


    reg20->ErrorValue = std::make_unique<uint64_t>(42);
    Note() << "Cycle() [read, set error value for register]";
    SerialClient->Cycle();
    SerialClient->GetTextValue(reg20);

    SerialClient->Cycle();
}

TEST_F(TSerialClientTestWithSetupRegisters, SetupOk)
{
    PRegister reg20 = Reg(20);
    SerialClient->AddRegister(reg20);
    SerialClient->Cycle();
    EXPECT_FALSE(Device->GetIsDisconnected());
}

TEST_F(TSerialClientTestWithSetupRegisters, SetupFail)
{
    PRegister reg20 = Reg(20);
    SerialClient->AddRegister(reg20);
    Device->BlockWriteFor(101, true);
    SerialClient->Cycle();
    EXPECT_TRUE(Device->GetIsDisconnected());
    Device->BlockWriteFor(101, false);
    SerialClient->Cycle();
    EXPECT_FALSE(Device->GetIsDisconnected());
}

class TSerialClientIntegrationTest: public TSerialClientTest
{
protected:
    void SetUp();
    void TearDown();
    void FilterConfig(const std::string& device_name);
    void Publish(const std::string & topic, const std::string & payload, uint8_t qos = 0, bool retain = true);
    void PublishWaitOnValue(const std::string & topic, const std::string & payload, uint8_t qos = 0, bool retain = true);

    /** reconnect test functions **/
    static void DeviceTimeoutOnly(const PSerialDevice & device, chrono::milliseconds timeout);
    static void DeviceMaxFailCyclesOnly(const PSerialDevice & device, int cycleCount);
    static void DeviceTimeoutAndMaxFailCycles(const PSerialDevice & device, chrono::milliseconds timeout, int cycleCount);

    PMQTTSerialDriver StartReconnectTest1Device(bool miss = false, bool pollIntervalTest = false);
    PMQTTSerialDriver StartReconnectTest2Devices();

    void ReconnectTest1Device(function<void()> && thunk, bool pollIntervalTest = false);
    void ReconnectTest2Devices(function<void()> && thunk);

    PFakeMqttBroker MqttBroker;
    PFakeMqttClient MqttClient;
    PDeviceDriver   Driver;
    PMQTTSerialDriver SerialDriver;
    PHandlerConfig  Config;

    static const char * const Name;
    static const char * const OtherName;
};

const char * const TSerialClientIntegrationTest::Name = "serial-client-integration-test";
const char * const TSerialClientIntegrationTest::OtherName = "serial-client-integration-test-other";

void TSerialClientIntegrationTest::SetUp()
{
    SetMode(E_Unordered);
    TSerialClientTest::SetUp();

    MqttBroker = NewFakeMqttBroker(*this);
    MqttClient = MqttBroker->MakeClient(Name);
    auto backend = NewDriverBackend(MqttClient);
    Driver = NewDriver(TDriverArgs{}
        .SetId(Name)
        .SetBackend(backend)
        .SetIsTesting(true)
        .SetReownUnknownDevices(true)
        .SetUseStorage(true)
        .SetStoragePath("/tmp/wb-mqtt-serial-test.db")
    );

    Driver->StartLoop();

    Json::Value configSchema = LoadConfigSchema(GetDataFilePath("../wb-mqtt-serial.schema.json"));
    AddFakeDeviceType(configSchema);
    AddRegisterType(configSchema, "fake");
    TTemplateMap t;
    Config = LoadConfig(GetDataFilePath("configs/config-test.json"), 
                        DeviceFactory,
                        configSchema,
                        t,
                        [=](const Json::Value&) {return std::make_pair(Port, false);});
}

void TSerialClientIntegrationTest::TearDown()
{
    if (SerialDriver) {
        SerialDriver->ClearDevices();
    }
    Driver->StopLoop();
    TSerialClientTest::TearDown();
}

void TSerialClientIntegrationTest::FilterConfig(const std::string& device_name)
{
    for (auto port_config: Config->PortConfigs) {
        LOG(Info) << "port devices: " << port_config->Devices.size();
        port_config->Devices.erase(
            remove_if(port_config->Devices.begin(),
                      port_config->Devices.end(),
                      [device_name](auto device) {
                          return device->DeviceConfig()->Name != device_name;
                      }),
            port_config->Devices.end());
    }
    Config->PortConfigs.erase(
        remove_if(Config->PortConfigs.begin(),
                  Config->PortConfigs.end(),
                  [](PPortConfig port_config) {
                      return port_config->Devices.empty();
                  }),
        Config->PortConfigs.end());
    ASSERT_FALSE(Config->PortConfigs.empty()) << "device not found: " << device_name;
}

void TSerialClientIntegrationTest::DeviceTimeoutOnly(const PSerialDevice & device, chrono::milliseconds timeout)
{
    device->DeviceConfig()->DeviceTimeout = timeout;
    device->DeviceConfig()->DeviceMaxFailCycles = 0;
}

void TSerialClientIntegrationTest::DeviceMaxFailCyclesOnly(const PSerialDevice & device, int cycleCount)
{
    device->DeviceConfig()->DeviceTimeout = chrono::milliseconds(0);
    device->DeviceConfig()->DeviceMaxFailCycles = cycleCount;
}

void TSerialClientIntegrationTest::DeviceTimeoutAndMaxFailCycles(const PSerialDevice & device, chrono::milliseconds timeout, int cycleCount)
{
    device->DeviceConfig()->DeviceTimeout = timeout;
    device->DeviceConfig()->DeviceMaxFailCycles = cycleCount;
}

void TSerialClientIntegrationTest::Publish(const std::string & topic, const std::string & payload, uint8_t qos, bool retain)
{
    MqttBroker->Publish("em-test-other", {TMqttMessage{topic, payload, qos, retain}});
}

void TSerialClientIntegrationTest::PublishWaitOnValue(const std::string & topic, const std::string & payload, uint8_t qos, bool retain)
{
    auto done = std::make_shared<WBMQTT::TPromise<void>>();
    Driver->On<WBMQTT::TControlOnValueEvent>([=](const WBMQTT::TControlOnValueEvent & event) {
        if (!done->IsFulfilled()) {
            done->Complete();
        }
    });
    Publish(topic, payload, qos, retain);
    done->GetFuture().Sync();
}

TEST_F(TSerialClientIntegrationTest, ControlTitle)
{
    FilterConfig("ControlTitleTest");
    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);
}

TEST_F(TSerialClientIntegrationTest, OnValue)
{
    FilterConfig("OnValueTest");

    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("0x90");

    if (!device) {
        throw std::runtime_error("device not found or wrong type");
    }

    device->Registers[0] = 0;
    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    PublishWaitOnValue("/devices/OnValueTest/controls/Relay 1/on", "1", 0, true);

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();
    ASSERT_EQ(500, device->Registers[0]);
}

TEST_F(TSerialClientIntegrationTest, OffValue)
{
    FilterConfig("OnValueTest");

    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("0x90");

    if (!device) {
        throw std::runtime_error("device not found or wrong type");
    }

    device->Registers[0] = 500;
    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    PublishWaitOnValue("/devices/OnValueTest/controls/Relay 1/on", "0", 0, true);

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();
    ASSERT_EQ(200, device->Registers[0]);
}


TEST_F(TSerialClientIntegrationTest, OnValueError)
{
    FilterConfig("OnValueTest");

    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("0x90");

    if (!device) {
        throw std::runtime_error("device not found or wrong type");
    }

    device->Registers[0] = 0;
    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    device->BlockWriteFor(0, true);

    PublishWaitOnValue("/devices/OnValueTest/controls/Relay 1/on", "1", 0, true);

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    device->BlockWriteFor(0, false);

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    ASSERT_EQ(500, device->Registers[0]);
}

TEST_F(TSerialClientIntegrationTest, WordSwap)
{
    FilterConfig("WordsLETest");

    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("0x91");

    if (!device) {
        throw std::runtime_error("device not found or wrong type");
    }

    device->Registers[0] = 0;
    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    PublishWaitOnValue("/devices/WordsLETest/controls/Voltage/on", "123", 0, true);

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();
    ASSERT_EQ(123, device->Registers[0]);
    ASSERT_EQ(0, device->Registers[1]);
    ASSERT_EQ(0, device->Registers[2]);
    ASSERT_EQ(0, device->Registers[3]);
}

TEST_F(TSerialClientIntegrationTest, Round)
{
    FilterConfig("RoundTest");

    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("0x92");

    if (!device) {
        throw std::runtime_error("device not found or wrong type");
    }

    device->Registers[0] = 0;
    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    PublishWaitOnValue("/devices/RoundTest/controls/Float_0_01/on", "12.345", 0, true);
    PublishWaitOnValue("/devices/RoundTest/controls/Float_1/on", "12.345", 0, true);
    PublishWaitOnValue("/devices/RoundTest/controls/Float_10/on", "12.345", 0, true);
    PublishWaitOnValue("/devices/RoundTest/controls/Float_0_2/on", "12.345", 0, true);

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    union {
        uint32_t words;
        float value;
    } data;

    data.words = device->Read2Registers(0);
    ASSERT_EQ(12.35f, data.value);

    data.words = device->Read2Registers(2);
    ASSERT_EQ(12.f, data.value);

    data.words = device->Read2Registers(4);
    ASSERT_EQ(10.f, data.value);

    data.words = device->Read2Registers(6);
    ASSERT_EQ(12.4f, data.value);
}

TEST_F(TSerialClientIntegrationTest, Errors)
{
    FilterConfig("DDL24");

    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("23");

    if (!device) {
        throw std::runtime_error("device not found or wrong type");
    }

    Note() << "LoopOnce() [first start]";
    SerialDriver->LoopOnce();

    device->BlockReadFor(4, true);
    device->BlockWriteFor(4, true);
    device->BlockReadFor(7, true);
    device->BlockWriteFor(7, true);

    Note() << "LoopOnce() [read, rw blacklisted]";
    SerialDriver->LoopOnce();

    PublishWaitOnValue("/devices/ddl24/controls/RGB/on", "10;20;30", 0, true);
    PublishWaitOnValue("/devices/ddl24/controls/White/on", "42", 0, true);

    Note() << "LoopOnce() [write, rw blacklisted]";
    SerialDriver->LoopOnce();

    device->BlockReadFor(4, false);
    device->BlockWriteFor(4, false);
    device->BlockReadFor(7, false);
    device->BlockWriteFor(7, false);

    Note() << "LoopOnce() [read, nothing blacklisted]";
    SerialDriver->LoopOnce();

    PublishWaitOnValue("/devices/ddl24/controls/RGB/on", "10;20;30", 0, true);
    PublishWaitOnValue("/devices/ddl24/controls/White/on", "42", 0, true);

    Note() << "LoopOnce() [write, nothing blacklisted]";
    SerialDriver->LoopOnce();

    Note() << "LoopOnce() [read, nothing blacklisted] (2)";
    SerialDriver->LoopOnce();
}

TEST_F(TSerialClientIntegrationTest, SetupErrors)
{
    FilterConfig("DDL24");

    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("23");

    if (!device) {
        throw std::runtime_error("device not found or wrong type");
    }

    device->BlockWriteFor(1, true);

    Note() << "LoopOnce() [first start, write blacklisted]";
    SerialDriver->LoopOnce();

    device->BlockWriteFor(1, false);
    device->BlockWriteFor(2, true);

    Note() << "LoopOnce() [write blacklisted]";
    SerialDriver->LoopOnce();

    device->BlockWriteFor(2, false);

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();
}

TEST_F(TSerialClientIntegrationTest, ErrorValue)
{
    FilterConfig("ErrorValueTest");

    SerialDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("0x96");

    if (!device) {
        throw std::runtime_error("device not found or wrong type");
    }

    Note() << "LoopOnce()";
    SerialDriver->LoopOnce();

    device->Registers[0]  = 0x7FFF;
    device->Registers[1]  = 0x7FFF;
    device->Registers[2]  = 0x7F;
    device->Registers[3]  = 0x7F;
    device->Registers[4]  = 0x7F;
    device->Registers[5]  = 0xFFFF;
    device->Registers[6]  = 0x7F;
    device->Registers[7]  = 0xFFFF;
    device->Registers[8]  = 0x7FFF;
    device->Registers[9]  = 0xFFFF;
    device->Registers[10] = 0x7FFF;
    device->Registers[11] = 0xFFFF;
    device->Registers[12] = 0x7FFF;
    device->Registers[13] = 0xFFFF;
    device->Registers[14] = 0xFFFF;
    device->Registers[15] = 0xFFFF;
    device->Registers[16] = 0x7FFF;
    device->Registers[17] = 0xFFFF;
    device->Registers[18] = 0xFFFF;
    device->Registers[19] = 0xFFFF;
    device->Registers[20] = 0x7FFF;
    device->Registers[21] = 0xFFFF;
    device->Registers[22] = 0xFFFF;
    device->Registers[23] = 0xFFFF;
    device->Registers[24] = 0x7FFF;
    device->Registers[25] = 0xFFFF;

    // fake little-endian
    device->Registers[30] = 0x7FFF;
    device->Registers[31] = 0x7FFF;
    device->Registers[32] = 0x7F;
    device->Registers[33] = 0x7F;
    device->Registers[34] = 0xFFFF;
    device->Registers[35] = 0x7F;
    device->Registers[36] = 0xFFFF;
    device->Registers[37] = 0x7F;
    device->Registers[38] = 0xFFFF;
    device->Registers[39] = 0x7FFF;
    device->Registers[40] = 0xFFFF;
    device->Registers[41] = 0x7FFF;
    device->Registers[42] = 0xFFFF;
    device->Registers[43] = 0xFFFF;
    device->Registers[44] = 0xFFFF;
    device->Registers[45] = 0x7FFF;
    device->Registers[46] = 0xFFFF;
    device->Registers[47] = 0xFFFF;
    device->Registers[48] = 0xFFFF;
    device->Registers[49] = 0x7FFF;
    device->Registers[50] = 0xFFFF;
    device->Registers[51] = 0xFFFF;
    device->Registers[52] = 0xFFFF;
    device->Registers[53] = 0x7FFF;
    device->Registers[54] = 0xFFFF;
    device->Registers[55] = 0x7FFF;

    Note() << "LoopOnce() [all errors]";
    SerialDriver->LoopOnce();
}


TEST_F(TSerialClientIntegrationTest, SlaveIdCollision)
{
    Json::Value configSchema = LoadConfigSchema(GetDataFilePath("../wb-mqtt-serial.schema.json"));
    TTemplateMap t;

    EXPECT_THROW(LoadConfig(GetDataFilePath("configs/config-collision-test.json"), 
                                        DeviceFactory,
                                        configSchema,
                                        t,
                                        [=](const Json::Value&) {return std::make_pair(Port, false);}),
                    TConfigParserException);

    EXPECT_THROW(LoadConfig(GetDataFilePath("configs/config-collision-test2.json"), 
                                        DeviceFactory,
                                        configSchema,
                                        t,
                                        [=](const Json::Value&) {return std::make_pair(Port, false);}),
                    TConfigParserException);

    EXPECT_NO_THROW(LoadConfig(GetDataFilePath("configs/config-no-collision-test.json"), 
                                        DeviceFactory,
                                        configSchema,
                                        t,
                                        [=](const Json::Value&) {return std::make_pair(Port, false);}));

    EXPECT_NO_THROW(LoadConfig(GetDataFilePath("configs/config-no-collision-test2.json"), 
                                        DeviceFactory,
                                        configSchema,
                                        t,
                                        [=](const Json::Value&) {return std::make_pair(Port, false);}));
}

/** Reconnect test cases **/
/*
    Test configurations:
    1) 1 device; 1 poll interval for all registers;
    2) 1 device; one long poll interval for one registers, one short poll interval for the rest of registers;
    3) 2 devices; 1 poll interval for all registers; only 1 device disconnect - connect

    Test conditions:
    1) device_timeout_ms only
    2) device_fail_cycles only
    3) device_timeout_ms and device_fail_cycles
*/

PMQTTSerialDriver TSerialClientIntegrationTest::StartReconnectTest1Device(bool miss, bool pollIntervalTest)
{
    Json::Value configSchema = LoadConfigSchema(GetDataFilePath("../wb-mqtt-serial.schema.json"));
    AddFakeDeviceType(configSchema);
    AddRegisterType(configSchema, "fake");
    TTemplateMap t;
    Config = LoadConfig(GetDataFilePath("configs/reconnect_test_1_device.json"), 
                                            DeviceFactory,
                                            configSchema,
                                            t,
                                            [=](const Json::Value&) {return std::make_pair(Port, false);});

    if (pollIntervalTest) {
        Config->PortConfigs[0]->Devices[0]->DeviceConfig()->DeviceChannelConfigs[0]->RegisterConfigs[0]->PollInterval = chrono::seconds(100);
    }

    PMQTTSerialDriver mqttDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto device = TFakeSerialDevice::GetDevice("12");

    {   // Test initial WriteInitValues
        Note() << "LoopOnce() [first start]";
        mqttDriver->LoopOnce();

        EXPECT_EQ(42, device->Registers[1]);
        EXPECT_EQ(24, device->Registers[2]);
    }

    {   // Test read
        Note() << "LoopOnce()";
        mqttDriver->LoopOnce();
    }

    {   // Device disconnected
        Note() << "SimulateDisconnect(true)";
        device->SetIsConnected(false);

        {
            bool by_timeout = Device->DeviceConfig()->DeviceTimeout.count() > 0;
            bool by_cycles = Device->DeviceConfig()->DeviceMaxFailCycles > 0;

            if (by_timeout) {
                auto delay = Device->DeviceConfig()->DeviceTimeout;

                if (miss) {
                    delay -= chrono::milliseconds(100);
                }

                if (by_cycles) {
                    auto disconnectTimepoint = std::chrono::steady_clock::now();

                    // Couple of unsuccessful reads
                    while (std::chrono::steady_clock::now() - disconnectTimepoint < delay) {
                        Note() << "LoopOnce()";
                        mqttDriver->LoopOnce();
                        usleep(std::chrono::duration_cast<std::chrono::microseconds>(delay).count() / device->DeviceConfig()->DeviceMaxFailCycles);
                    }
                } else {
                    auto disconnectTimepoint = std::chrono::steady_clock::now();

                    // Couple of unsuccessful reads
                    while (std::chrono::steady_clock::now() - disconnectTimepoint < delay) {
                        Note() << "LoopOnce()";
                        mqttDriver->LoopOnce();
                        usleep(std::chrono::duration_cast<std::chrono::microseconds>(delay).count() / 10);
                    }
                }
            } else if (by_cycles) {
                // Couple of unsuccessful reads
                auto remainingCycles = Device->DeviceConfig()->DeviceMaxFailCycles;

                if (miss) {
                    --remainingCycles;
                }

                while (remainingCycles--) {
                    Note() << "LoopOnce()";
                    mqttDriver->LoopOnce();
                }
            } else {
                auto disconnectTimepoint = std::chrono::steady_clock::now();
                auto delay = chrono::milliseconds(10000);

                // Couple of unsuccessful reads
                while (std::chrono::steady_clock::now() - disconnectTimepoint < delay) {
                    Note() << "LoopOnce()";
                    mqttDriver->LoopOnce();
                    usleep(std::chrono::duration_cast<std::chrono::microseconds>(delay).count() / 10);
                }
            }
        }



        // Final unsuccessful read after timeout, after this loop we expect device to be counted as disconnected
        Note() << "LoopOnce()";
        mqttDriver->LoopOnce();
    }

    return mqttDriver;
}

PMQTTSerialDriver TSerialClientIntegrationTest::StartReconnectTest2Devices()
{
    Json::Value configSchema = LoadConfigSchema(GetDataFilePath("../wb-mqtt-serial.schema.json"));
    AddFakeDeviceType(configSchema);
    AddRegisterType(configSchema, "fake");
    TTemplateMap t;
    Config = LoadConfig(GetDataFilePath("configs/reconnect_test_2_devices.json"), 
                                            DeviceFactory,
                                            configSchema,
                                            t,
                                            [=](const Json::Value&) {return std::make_pair(Port, false);});

    PMQTTSerialDriver mqttDriver = make_shared<TMQTTSerialDriver>(Driver, Config);

    auto dev1 = TFakeSerialDevice::GetDevice("12");
    auto dev2 = TFakeSerialDevice::GetDevice("13");

    {   // Test initial setup
        Note() << "LoopOnce() [first start]";
        mqttDriver->LoopOnce();

        EXPECT_EQ(42, dev1->Registers[1]);
        EXPECT_EQ(24, dev1->Registers[2]);

        EXPECT_EQ(32, dev2->Registers[1]);
        EXPECT_EQ(64, dev2->Registers[2]);
    }

    {   // Test read
        Note() << "LoopOnce()";
        mqttDriver->LoopOnce();
    }

    {   // Device disconnected
        Note() << "SimulateDisconnect(true)";
        dev1->SetIsConnected(false);

        bool by_timeout = Device->DeviceConfig()->DeviceTimeout.count() > 0;
        bool by_cycles = Device->DeviceConfig()->DeviceMaxFailCycles > 0;

        if (by_timeout) {
            if (by_cycles) {
                auto disconnectTimepoint = std::chrono::steady_clock::now();

                // Couple of unsuccessful reads
                while (std::chrono::steady_clock::now() - disconnectTimepoint < Device->DeviceConfig()->DeviceTimeout) {
                    Note() << "LoopOnce()";
                    mqttDriver->LoopOnce();
                    usleep(std::chrono::duration_cast<std::chrono::microseconds>(Device->DeviceConfig()->DeviceTimeout).count() / Device->DeviceConfig()->DeviceMaxFailCycles);
                }
            } else {
                auto disconnectTimepoint = std::chrono::steady_clock::now();

                // Couple of unsuccessful reads
                while (std::chrono::steady_clock::now() - disconnectTimepoint < Device->DeviceConfig()->DeviceTimeout) {
                    Note() << "LoopOnce()";
                    mqttDriver->LoopOnce();
                    usleep(std::chrono::duration_cast<std::chrono::microseconds>(Device->DeviceConfig()->DeviceTimeout).count() / 10);
                }
            }
        } else if (by_cycles) {
            // Couple of unsuccessful reads
            auto remainingCycles = Device->DeviceConfig()->DeviceMaxFailCycles;

            while (remainingCycles--) {
                Note() << "LoopOnce()";
                mqttDriver->LoopOnce();
            }
        } else {
            auto disconnectTimepoint = std::chrono::steady_clock::now();
            auto delay = chrono::milliseconds(10000);

            // Couple of unsuccessful reads
            while (std::chrono::steady_clock::now() - disconnectTimepoint < delay) {
                Note() << "LoopOnce()";
                mqttDriver->LoopOnce();
                usleep(std::chrono::duration_cast<std::chrono::microseconds>(delay).count() / 10);
            }
        }

        // Final unsuccessful read after timeout, after this loop we expect device to be counted as disconnected
        Note() << "LoopOnce()";
        mqttDriver->LoopOnce();
    }

    return mqttDriver;
}

void TSerialClientIntegrationTest::ReconnectTest1Device(function<void()> && thunk, bool pollIntervalTest)
{
    auto observer = StartReconnectTest1Device(false, pollIntervalTest);
    auto device = TFakeSerialDevice::GetDevice("12");

    device->Registers[1] = 0;   // reset those to make sure that setup section is wrote again
    device->Registers[2] = 0;

    thunk();

    {   // Loop to check limited polling
        Note() << "LoopOnce() (limited polling expected)";
        observer->LoopOnce();
    }

    {   // Device is connected back
        Note() << "SimulateDisconnect(false)";
        device->SetIsConnected(true);

        Note() << "LoopOnce()";
        observer->LoopOnce();

        EXPECT_EQ(42, device->Registers[1]);
        EXPECT_EQ(24, device->Registers[2]);
    }
}

void TSerialClientIntegrationTest::ReconnectTest2Devices(function<void()> && thunk)
{
    auto observer = StartReconnectTest2Devices();

    auto dev1 = TFakeSerialDevice::GetDevice("12");
    auto dev2 = TFakeSerialDevice::GetDevice("13");

    dev1->Registers[1] = 0;   // reset those to make sure that setup section is wrote again
    dev1->Registers[2] = 0;

    dev2->Registers[1] = 1;   // set control values to make sure that no setup section is written
    dev2->Registers[2] = 2;

    thunk();

    {   // Loop to check limited polling
        Note() << "LoopOnce() (limited polling expected)";
        observer->LoopOnce();
    }

    {   // Device is connected back
        Note() << "SimulateDisconnect(false)";
        dev1->SetIsConnected(true);

        Note() << "LoopOnce()";
        auto future = MqttBroker->WaitForPublish("/devices/reconnect-test-1/controls/I2/meta/error");
        observer->LoopOnce();

        EXPECT_EQ(42, dev1->Registers[1]);
        EXPECT_EQ(24, dev1->Registers[2]);

        EXPECT_EQ(1, dev2->Registers[1]);
        EXPECT_EQ(2, dev2->Registers[2]);

        future.Wait();
    }
}

TEST_F(TSerialClientIntegrationTest, ReconnectTimeout)
{
    ReconnectTest1Device([&]{
        DeviceTimeoutOnly(Device, DefaultDeviceTimeout);
    });
}

TEST_F(TSerialClientIntegrationTest, ReconnectCycles)
{
    ReconnectTest1Device([&]{
        DeviceMaxFailCyclesOnly(Device, 10);
    });
}

TEST_F(TSerialClientIntegrationTest, ReconnectTimeoutAndCycles)
{
    ReconnectTest1Device([&]{
        DeviceTimeoutAndMaxFailCycles(Device, DefaultDeviceTimeout, 10);
    });
}

TEST_F(TSerialClientIntegrationTest, ReconnectRegisterWithBigPollInterval)
{
    auto t1 = chrono::steady_clock::now();

    ReconnectTest1Device([&]{
        DeviceTimeoutOnly(Device, DefaultDeviceTimeout);
    }, true);

    auto time = chrono::steady_clock::now() - t1;

    EXPECT_LT(time, chrono::seconds(100));
}

TEST_F(TSerialClientIntegrationTest, Reconnect2)
{
    ReconnectTest2Devices([&]{
        DeviceTimeoutOnly(Device, DefaultDeviceTimeout);
    });
}

TEST_F(TSerialClientIntegrationTest, ReconnectMiss)
{
    auto observer = StartReconnectTest1Device(true);

    auto device = TFakeSerialDevice::GetDevice("12");
    if (!device) {
        throw std::runtime_error("device not found");
    }

    device->Registers[1] = 1;   // set control values to make sure that no setup section is written
    device->Registers[2] = 2;

    {   // Device is connected back before timeout
        Note() << "SimulateDisconnect(false)";
        device->SetIsConnected(true);

        Note() << "LoopOnce()";
        auto future = MqttBroker->WaitForPublish("/devices/reconnect-test/controls/I2");
        observer->LoopOnce();
        future.Wait();

        EXPECT_EQ(1, device->Registers[1]);
        EXPECT_EQ(2, device->Registers[2]);
    }
}
