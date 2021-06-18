#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

#include "uniel_device.h"
#include "log.h"

#define LOG(logger) ::logger.Log() << "[uniel device] "

namespace
{
    enum {
        READ_CMD           = 0x05,
        WRITE_CMD          = 0x06,
        SET_BRIGHTNESS_CMD = 0x0a
    };

    const TRegisterTypes RegisterTypes{
        { TUnielDevice::REG_RELAY,      "relay",      "switch", U8 },
        { TUnielDevice::REG_INPUT,      "input",      "value",  U8, true },
        { TUnielDevice::REG_PARAM,      "param",      "value",  U8 },
        // "value", not "range" because 'max' cannot be specified here.
        { TUnielDevice::REG_BRIGHTNESS, "brightness", "value",  U8 }
    };
}

void TUnielDevice::Register(TSerialDeviceFactory& factory)
{
    factory.RegisterProtocol(new TUint32SlaveIdProtocol("uniel", RegisterTypes), 
                             new TBasicDeviceFactory<TUnielDevice>("#/definitions/simple_device_with_setup",
                                                                   "#/definitions/common_channel"));
}

TUnielDevice::TUnielDevice(PDeviceConfig config, PPort port, PProtocol protocol)
    : TSerialDevice(config, port, protocol), TUInt32SlaveId(config->SlaveId)
{}

void TUnielDevice::WriteCommand(uint8_t cmd, uint8_t mod, uint8_t b1, uint8_t b2, uint8_t b3)
{
    Port()->CheckPortOpen();
    uint8_t buf[8];
    buf[0] = buf[1] = 0xff;
    buf[2] = cmd;
    buf[3] = mod;
    buf[4] = b1;
    buf[5] = b2;
    buf[6] = b3;
    buf[7] = (cmd + mod + b1 + b2 + b3) & 0xff;
    Port()->WriteBytes(buf, 8);
}

void TUnielDevice::ReadResponse(uint8_t cmd, uint8_t* response)
{
    uint8_t buf[5];
    for (;;) {
        uint8_t first = Port()->ReadByte(DeviceConfig()->ResponseTimeout);
        if (first != 0xff) {
            LOG(Warn) << "resync";
            continue;
        }
        uint8_t second = Port()->ReadByte(DeviceConfig()->FrameTimeout);
        if (second == 0xff) {
            second = Port()->ReadByte(DeviceConfig()->FrameTimeout);
        }
        buf[0] = second;
        uint8_t s = second;
        for (int i = 1; i < 5; ++i) {
            buf[i] = Port()->ReadByte(DeviceConfig()->FrameTimeout);
            s += buf[i];
        }

        if (Port()->ReadByte(DeviceConfig()->FrameTimeout) != s)
            throw TSerialDeviceTransientErrorException("uniel: warning: checksum failure");

        break;
    }

    if (buf[0] != cmd)
        throw TSerialDeviceTransientErrorException("bad command code in response");

    if (buf[1] != 0)
        throw TSerialDeviceTransientErrorException("bad module address in response");

    for (int i = 2; i < 5; ++i)
        *response++ = buf[i];
}

uint64_t TUnielDevice::ReadRegister(PRegister reg)
{
    auto addr = GetUint32RegisterAddress(reg->GetAddress());
    WriteCommand(READ_CMD, SlaveId, 0, uint8_t(addr), 0);
    uint8_t response[3] = {0};
    ReadResponse(READ_CMD, response);
    if (response[1] != uint8_t(addr))
        throw TSerialDeviceTransientErrorException("register index mismatch");

    if (reg->Type == REG_RELAY)
        return response[0] ? 1 : 0;
    return response[0];
}

void TUnielDevice::WriteRegister(PRegister reg, uint64_t value)
{
    auto addr = GetUint32RegisterAddress(reg->GetAddress());
    uint8_t cmd;
    if (reg->Type == REG_BRIGHTNESS) {
        cmd = SET_BRIGHTNESS_CMD;
        addr >>= 8;
    } else {
        cmd = WRITE_CMD;
    }
    if (reg->Type == REG_RELAY && value != 0)
        value = 255;
    WriteCommand(cmd, SlaveId, value, addr, 0);
    uint8_t response[3];
    ReadResponse(cmd, response);
    if (response[1] != addr)
        throw TSerialDeviceTransientErrorException("register index mismatch");
    if (response[0] != value)
        throw TSerialDeviceTransientErrorException("written register value mismatch");
}
