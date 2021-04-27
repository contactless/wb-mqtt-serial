#pragma once
#include <string>
#include <stdexcept>
#include <string.h>

class TSerialDeviceException: public std::runtime_error {
public:
    TSerialDeviceException(const std::string& message): std::runtime_error("Serial protocol error: " + message) {}
};

class TSerialDeviceErrnoException: public TSerialDeviceException {
    int ErrnoValue;
public:
    TSerialDeviceErrnoException(const std::string& message, int errnoValue): 
        TSerialDeviceException(message + strerror(errnoValue) + " (" + std::to_string(errnoValue) + ")"),
        ErrnoValue(errnoValue)
    {}

    int GetErrnoValue() const
    {
        return ErrnoValue;
    }
};

class TSerialDeviceTransientErrorException: public TSerialDeviceException {
public:
    TSerialDeviceTransientErrorException(const std::string& message): TSerialDeviceException(message) {}
};

/** The exceptiona class should be used for indicating reception of a valid answer from device,
 *  but with information about internal device's error.
 *  Polling of other registers of the device during current cycle is allowed, but current register read should be marked as read error.
 *  Example: reading curtain motor position returns invalid position due to motor misconfiguration.
 */
class TSerialDeviceInternalErrorException: public TSerialDeviceTransientErrorException {
public:
    TSerialDeviceInternalErrorException(const std::string& message): TSerialDeviceTransientErrorException(message) {}
};

class TSerialDevicePermanentRegisterException: public TSerialDeviceException {
public:
	TSerialDevicePermanentRegisterException(const std::string& message): TSerialDeviceException(message) {}
};
