#include "serial_port_driver.h"
#include "log.h"

#include <wblib/wbmqtt.h>

#include <algorithm>
#include <sstream>
#include <iostream>
#include <cassert>

#include "serial_port.h"
#include "tcp_port.h"

using namespace std;
using namespace WBMQTT;

#define LOG(logger) ::logger.Log() << "[serial port driver] "

TSerialPortDriver::TSerialPortDriver(WBMQTT::PDeviceDriver             mqttDriver,
                                     PPortConfig                       portConfig,
                                     const WBMQTT::TPublishParameters& publishPolicy)
    : MqttDriver(mqttDriver),
      Config(portConfig),
      PublishPolicy(publishPolicy)
{
    Description = Config->Port->GetDescription(false);
    SerialClient = PSerialClient(new TSerialClient(Config->Devices, Config->Port, Config->OpenCloseSettings));
}

const std::string& TSerialPortDriver::GetShortDescription() const
{
    return Description;
}

void TSerialPortDriver::SetUpDevices()
{
    SerialClient->SetReadCallback([this](PRegister reg, bool changed) {
        OnValueRead(reg, changed);
    });
    SerialClient->SetErrorCallback([this](PRegister reg, TRegisterHandler::TErrorState state) {
        UpdateError(reg, state);
    });

    LOG(Debug) << "setting up devices at " << Config->Port->GetDescription();

    try {
        auto tx = MqttDriver->BeginTx();

        for (auto device: Config->Devices) {
            auto mqttDevice = tx->CreateDevice(From(device)).GetValue();
            assert(mqttDevice);
            Devices.push_back(device);
            // init channels' registers
            for (auto & channelConfig: device->DeviceConfig()->DeviceChannelConfigs) {
                try {
                    auto channel = std::make_shared<TDeviceChannel>(device, channelConfig);
                    channel->Control = mqttDevice->CreateControl(tx, From(channel)).GetValue();
                    for (auto & reg: channel->Registers) {
                        RegisterToChannelStateMap.emplace(reg, TDeviceChannelState{channel, TRegisterHandler::UnknownErrorState});
                        SerialClient->AddRegister(reg);
                    }
                } catch (const exception & e) {
                    LOG(Error) << "unable to create control: '" << e.what() << "'";
                }
            }
            mqttDevice->RemoveUnusedControls(tx).Sync();
        }
    } catch (const exception & e) {
        LOG(Error) << "unable to create device: '" << e.what() << "' Cleaning.";
        ClearDevices();
        throw;
    } catch (...) {
        LOG(Error) << "unable to create device or control. Cleaning.";
        ClearDevices();
        throw;
    }
}

void TSerialPortDriver::HandleControlOnValueEvent(const WBMQTT::TControlOnValueEvent & event)
{
    const auto & value = event.RawValue;
    const auto & linkData = event.Control->GetUserData().As<TControlLinkData>();

    const auto & portDriver = linkData.PortDriver.lock();
    const auto & channel = linkData.DeviceChannel.lock();

    if (!portDriver || !channel) {
        if (!portDriver) {
            LOG(Error) << "event for non existent port driver from control '" << event.Control->GetDevice()->GetId() << "/" << event.Control->GetId() << "' value: '" << value << "'";
        }

        if (!channel) {
            LOG(Error) << "event for non existent device channel from control '" << event.Control->GetDevice()->GetId() << "/" << event.Control->GetId() << "' value: '" << value << "'";
        }

        return;
    }

    portDriver->SetValueToChannel(channel, value);
}

void TSerialPortDriver::SetValueToChannel(const PDeviceChannel & channel, const string & value)
{
    const auto & registers = channel->Registers;

    std::vector<std::string> valueItems = StringSplit(value, ';');

    if (valueItems.size() != registers.size()) {
        LOG(Warn) << "invalid value for " << channel->Describe() << ": '" << value << "'";
        return;
    }

    for (size_t i = 0; i < registers.size(); ++i) {
        PRegister reg = registers[i];
        LOG(Debug) << "setting device register: " << reg->ToString() << " <- " << valueItems[i];

        try {
            auto valueToSet = valueItems[i];
            if (!channel->OnValue.empty() && valueItems[i] == "1") {
                valueToSet = channel->OnValue;
            } else if (!channel->OffValue.empty() && valueItems[i] == "0") {
                valueToSet = channel->OffValue;
            }
            SerialClient->SetTextValue(reg, valueToSet);

        } catch (std::exception& err) {
            LOG(Warn) << "invalid value for " << channel->Describe() << ": '" << value << "' : " << err.what();
            return;
        }
    }
}

void TSerialPortDriver::OnValueRead(PRegister reg, bool changed)
{
    auto it = RegisterToChannelStateMap.find(reg);
    if (it == RegisterToChannelStateMap.end()) {
        LOG(Warn) << "got unexpected register from serial client";
        return;
    }
    const auto & channel = it->second.Channel;
    const auto & registers = channel->Registers;

    if (changed && ::Debug.IsEnabled()) {
        LOG(Debug) << "register value change: " << reg->ToString() << " <- " << SerialClient->GetTextValue(reg);
    }

    std::string value;
    if (!channel->OnValue.empty() && SerialClient->GetTextValue(reg) == channel->OnValue) {
        value = "1";
        LOG(Debug) << "OnValue: " << channel->OnValue << "; value: " << value;
    } else if (!channel->OffValue.empty() && SerialClient->GetTextValue(reg) == channel->OffValue) {
        value = "0";
        LOG(Debug) << "OffValue: " << channel->OffValue << "; value: " << value;
    } else {
        for (size_t i = 0; i < registers.size(); ++i) {
            PRegister reg = registers[i];
            // avoid publishing incomplete value
            if (!SerialClient->DidRead(reg))
                return;
            if (i)
                value += ";";
            value += SerialClient->GetTextValue(reg);
        }
    }

    channel->UpdateValue(*MqttDriver, PublishPolicy, value);
}

TRegisterHandler::TErrorState TSerialPortDriver::RegErrorState(PRegister reg)
{
    auto it = RegisterToChannelStateMap.find(reg);
    if (it == RegisterToChannelStateMap.end())
        return TRegisterHandler::UnknownErrorState;
    return it->second.ErrorState;
}

void TSerialPortDriver::UpdateError(PRegister reg, TRegisterHandler::TErrorState errorState)
{
    auto it = RegisterToChannelStateMap.find(reg);
    if (it == RegisterToChannelStateMap.end()) {
        LOG(Warn) << "got unexpected register from serial client";
        return;
    }
    const auto & channel = it->second.Channel;
    const auto & registers = channel->Registers;

    it->second.ErrorState = errorState;
    size_t errorMask = 0;
    for (auto r: registers) {
        auto error = RegErrorState(r);
        if (error <= TRegisterHandler::ReadWriteError) {
            errorMask |= error;
        }
    }

    const std::array<const char*, 4> errorFlags = {"", "w", "r", "rw"};
    channel->UpdateError(*MqttDriver, errorFlags[errorMask]);
}

void TSerialPortDriver::Cycle()
{
    try {
        SerialClient->Cycle();
    } catch (TSerialDeviceException& e) {
        LOG(Error) << "FATAL: " << e.what() << ". Stopping event loops.";
        exit(1);
    }
}

void TSerialPortDriver::ClearDevices() noexcept
{
    try {
        {
            auto tx = MqttDriver->BeginTx();

            for (const auto & device: Devices) {
                try {
                    tx->RemoveDeviceById(device->DeviceConfig()->Id).Sync();
                    LOG(Debug) << "device " << device->DeviceConfig()->Id << " removed successfully";
                } catch (const exception & e) {
                    LOG(Warn) << "exception during device removal: " << e.what();
                } catch (...) {
                    LOG(Warn) << "unknown exception during device removal";
                }
            }
        }
        Devices.clear();
        SerialClient->ClearDevices();
    } catch (const exception & e) {
        LOG(Warn) << "TSerialPortDriver::ClearDevices(): " << e.what();
    } catch (...) {
        LOG(Warn) << "TSerialPortDriver::ClearDevices(): unknown exception";
    }
}

TLocalDeviceArgs TSerialPortDriver::From(const PSerialDevice & device)
{
    return TLocalDeviceArgs{}.SetId(device->DeviceConfig()->Id)
                             .SetTitle(device->DeviceConfig()->Name)
                             .SetIsVirtual(true)
                             .SetDoLoadPrevious(true);
}

TControlArgs TSerialPortDriver::From(const PDeviceChannel & channel)
{
    auto args = TControlArgs{}.SetId(channel->MqttId)
                              .SetOrder(channel->Order)
                              .SetType(channel->Type)
                              .SetReadonly(channel->ReadOnly)
                              .SetUserData(TControlLinkData{ shared_from_this(), channel });

    if (isnan(channel->Max)) {
        if (channel->Type == "range" || channel->Type == "dimmer") {
            args.SetMax(65535);
        }
    } else {
        args.SetMax(channel->Max);
    }

    if (!isnan(channel->Min)) {
        args.SetMin(channel->Min);
    }

    if (channel->Precision != 0.0) {
        args.SetPrecision(channel->Precision);
    }

    if (channel->Name != channel->MqttId) {
        args.SetTitle(channel->Name);
    }

    return args;
}

void TDeviceChannel::UpdateError(WBMQTT::TDeviceDriver& deviceDriver, const std::string& error)
{
    if (CachedErrorFlg.empty() || (CachedErrorFlg != error)) {
        CachedErrorFlg = error;
        {
            auto tx = deviceDriver.BeginTx();
            Control->SetError(tx, error).Sync();
        }
    }
}

void TDeviceChannel::UpdateValue(WBMQTT::TDeviceDriver& deviceDriver, const WBMQTT::TPublishParameters& publishPolicy, const std::string& value)
{
    if (!CachedErrorFlg.empty()) {
        PublishValue(deviceDriver, value);
        return;
    }
    switch (publishPolicy.Policy) {
        case TPublishParameters::PublishOnlyOnChange: {
            if (CachedCurrentValue != value) {
                PublishValue(deviceDriver, value);
            }
            break;
        }
        case TPublishParameters::PublishAll: {
            PublishValue(deviceDriver, value);
            break;
        }
        case TPublishParameters::PublishSomeUnchanged: {
            auto now = std::chrono::steady_clock::now();
            if ((CachedCurrentValue != value) || 
                (now - LastControlUpdate >= publishPolicy.PublishUnchangedInterval))
            {
                PublishValue(deviceDriver, value);
            }
            break;
        }
    }
}

void TDeviceChannel::PublishValue(WBMQTT::TDeviceDriver& deviceDriver, const std::string& value)
{
    if (::Debug.IsEnabled()) {
        LOG(Debug) << Describe() << " <-- " << value;
    }
    CachedCurrentValue = value;
    CachedErrorFlg.clear();
    LastControlUpdate = std::chrono::steady_clock::now();
    {
        auto tx = deviceDriver.BeginTx();
        Control->SetRawValue(tx, value).Sync();
    }
}
