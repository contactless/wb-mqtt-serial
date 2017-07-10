#pragma once

#include <string>
#include <memory>
#include <exception>
#include <unordered_set>
#include <functional>
#include <cstdint>
#include <cstring>

#include "serial_device.h"
#include "crc16.h"

// Common device base for electricity meters
template<class Proto>
class TEMDevice: public TBasicProtocolSerialDevice<Proto> {
public:
    static const int DefaultTimeoutMs = 1000;

    TEMDevice(PDeviceConfig device_config, PAbstractSerialPort port, PProtocol protocol)
        : TBasicProtocolSerialDevice<Proto>(device_config, port, protocol)
    {}

    void WriteRegister(PRegister reg, uint64_t value)
    {
        throw TSerialDeviceException("EM protocol: writing to registers not supported");
    }

protected:
    enum ErrorType {
        NO_ERROR,
        NO_OPEN_SESSION,
		UNSUPPORTED_ERROR,
        OTHER_ERROR
    };
    const int   MAX_LEN = 64;

    virtual bool ConnectionSetup() = 0;
    virtual ErrorType CheckForException(uint8_t* frame, int len, const char** message) = 0;
    void WriteCommand( uint8_t cmd, uint8_t* payload, int len)
    {
        uint8_t buf[MAX_LEN], *p = buf;
        if (len + 3 + SlaveIdWidth > MAX_LEN)
            throw TSerialDeviceException("outgoing command too long");

        // SlaveId is sent in reverse (little-endian) order
        for (int i = 0; i < SlaveIdWidth; ++i) {
            *p++ = (this->SlaveId & (0xFF << (8*i))) >> (8*i);
        }

        *p++ = cmd;
        while (len--)
            *p++ = *payload++;
        uint16_t crc = CRC16::CalculateCRC16(buf, p - buf);
        *p++ = crc >> 8;
        *p++ = crc & 0xff;
        this->Port()->WriteBytes(buf, p - buf);
    }

    bool ReadResponse( int expectedByte1, uint8_t* payload, int len,
                               TAbstractSerialPort::TFrameCompletePred frame_complete = 0)
    {
        uint8_t buf[MAX_LEN], *p = buf;
        int nread = this->Port()->ReadFrame(buf, MAX_LEN, this->DeviceConfig()->FrameTimeout, frame_complete);
        if (nread < 3 + SlaveIdWidth)
            throw TSerialDeviceTransientErrorException("frame too short");

        uint16_t crc = CRC16::CalculateCRC16(buf, nread - 2),
            crc1 = buf[nread - 2],
            crc2 = buf[nread - 1],
            actualCrc = (crc1 << 8) + crc2;
        if (crc != actualCrc)
            throw TSerialDeviceTransientErrorException("invalid crc");

        for (int i = 0; i < SlaveIdWidth; ++i) {
            if (*p++ != (this->SlaveId & (0xFF << (8*i))) >> (8*i)) {
                throw TSerialDeviceTransientErrorException("invalid slave id");
            }
        }

        const char* msg;
        ErrorType err = CheckForException(buf, nread, &msg);
        if (err == NO_OPEN_SESSION)
            return false;
        if (err == UNSUPPORTED_ERROR)
        	throw TSerialDeviceUnsupportedRegisterException(msg);
        if (err != NO_ERROR)
            throw TSerialDeviceTransientErrorException(msg);

        if (expectedByte1 >= 0 && *p++ != expectedByte1)
            throw TSerialDeviceTransientErrorException("invalid command code in the response");

        int actualPayloadSize = nread - (p - buf) - 2;
        if (len >= 0 && len != actualPayloadSize)
            throw TSerialDeviceTransientErrorException("unexpected frame size");
        else
            len = actualPayloadSize;

        std::memcpy(payload, p, len);
        return true;
    }
    void Talk( uint8_t cmd, uint8_t* payload, int payload_len,
              int expected_byte1, uint8_t* resp_payload, int resp_payload_len,
              TAbstractSerialPort::TFrameCompletePred frame_complete = 0)
    {
        EnsureSlaveConnected();
        WriteCommand(cmd, payload, payload_len);
        try {
            while (!ReadResponse(expected_byte1, resp_payload, resp_payload_len, frame_complete)) {
                EnsureSlaveConnected( true);
                WriteCommand(cmd, payload, payload_len);
            }
        } catch ( const TSerialDeviceTransientErrorException& e) {
            this->Port()->SkipNoise();
            throw;
        }
    }

    uint8_t SlaveIdWidth = 1;

private:
    void EnsureSlaveConnected(bool force = false)
    {
        if (!force && ConnectedSlaves.find(this->SlaveId) != ConnectedSlaves.end())
            return;

        ConnectedSlaves.erase(this->SlaveId);
        this->Port()->SkipNoise();
        if (!ConnectionSetup())
            throw TSerialDeviceTransientErrorException("failed to establish meter connection");

        ConnectedSlaves.insert(this->SlaveId);
    }

    std::unordered_set<uint8_t> ConnectedSlaves;
};
