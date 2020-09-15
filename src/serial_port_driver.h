#pragma once
#include "serial_config.h"
#include "serial_client.h"
#include "register_handler.h"

#include <wblib/declarations.h>

#include <chrono>
#include <memory>
#include <unordered_map>


struct TDeviceChannel : public TDeviceChannelConfig
{
    TDeviceChannel(PSerialDevice device, PDeviceChannelConfig config)
        : TDeviceChannelConfig(*config)
        , Device(device)
    {
        for (const auto &reg_config: config->RegisterConfigs) {
            Registers.push_back(TRegister::Intern(device, reg_config));
        }
    }

    std::string Describe() const
    {
        return "channel '" + Name + "' of device '" + DeviceId + "'";
    }

    PSerialDevice Device;
    std::vector<PRegister> Registers;
};

typedef std::shared_ptr<TDeviceChannel> PDeviceChannel;

using TPublishTime = std::chrono::time_point<std::chrono::steady_clock>;
using PPublishTime = std::unique_ptr<TPublishTime>;

struct TDeviceChannelState
{
    TDeviceChannelState(const PDeviceChannel & channel, TRegisterHandler::TErrorState error, PPublishTime && time)
        : Channel(channel)
        , ErrorState(error)
        , LastPublishTime(std::move(time))
    {}

    PDeviceChannel                  Channel;
    TRegisterHandler::TErrorState   ErrorState;
    PPublishTime                    LastPublishTime;
};


class TSerialPortDriver: public std::enable_shared_from_this<TSerialPortDriver>
{
public:
    TSerialPortDriver(WBMQTT::PDeviceDriver mqttDriver, PPortConfig port_config, PPort port_override = 0);
    ~TSerialPortDriver();
    void SetUpDevices();
    void Cycle();
    bool WriteInitValues();
    void ClearDevices() noexcept;

    static void HandleControlOnValueEvent(const WBMQTT::TControlOnValueEvent & event);

private:
    WBMQTT::TLocalDeviceArgs From(const PSerialDevice & device);
    WBMQTT::TControlArgs From(const PDeviceChannel & channel);

    void SetValueToChannel(const PDeviceChannel & channel, const std::string & value);
    bool NeedToPublish(PRegister reg, bool changed);
    void OnValueRead(PRegister reg, bool changed);
    TRegisterHandler::TErrorState RegErrorState(PRegister reg);
    void UpdateError(PRegister reg, TRegisterHandler::TErrorState errorState);

    WBMQTT::PDeviceDriver MqttDriver;
    PPortConfig Config;
    PPort Port;
    PSerialClient SerialClient;
    std::vector<PSerialDevice> Devices;

    std::unordered_map<PRegister, TDeviceChannelState> RegisterToChannelStateMap;
};

typedef std::shared_ptr<TSerialPortDriver> PSerialPortDriver;

struct TControlLinkData
{
    std::weak_ptr<TSerialPortDriver>  PortDriver;
    std::weak_ptr<TDeviceChannel>     DeviceChannel;
};
