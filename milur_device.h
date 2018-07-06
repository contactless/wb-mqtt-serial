#pragma once

#include <string>
#include <memory>
#include <exception>
#include <cstdint>

#include "em_device.h"

class TMilurDevice;
typedef TBasicProtocol<TMilurDevice> TMilurProtocol;

class TMilurDevice: public TEMDevice<TMilurProtocol> {
public:
    static const int DefaultTimeoutMs = 1000;
    static const int FrameTimeoutMs = 50;
    enum RegisterType {
        REG_PARAM = 0,
        REG_POWER,
        REG_ENERGY,
        REG_FREQ,
        REG_POWERFACTOR
    };

    TMilurDevice(PDeviceConfig device_config, PPort port, PProtocol protocol);
    void Read(const TIRDeviceQuery &) override;
    void Prepare();


protected:
    bool ConnectionSetup();
    ErrorType CheckForException(uint8_t* frame, int len, const char** message);
    int GetExpectedSize(int type) const;
};

typedef std::shared_ptr<TMilurDevice> PMilurDevice;
