#include <iostream>
#include "serial_device.h"
#include <unistd.h>

TSerialDevice::TSerialDevice(PDeviceConfig config, PAbstractSerialPort port, PProtocol protocol)
    : Delay(config->Delay)
    , SerialPort(port)
    , _DeviceConfig(config)
    , _Protocol(protocol)
	, LastSuccessfulRead()
	, IsDisconnected(false)
{}

TSerialDevice::~TSerialDevice() 
{
    /* TSerialDeviceFactory::RemoveDevice(shared_from_this()); */    
}

std::string TSerialDevice::ToString() const
{
    return DeviceConfig()->Name + "(" + DeviceConfig()->SlaveId + ")";
}

std::list<PRegisterRange> TSerialDevice::SplitRegisterList(const std::list<PRegister> reg_list) const
{
    std::list<PRegisterRange> r;
    for (auto reg: reg_list)
        r.push_back(std::make_shared<TSimpleRegisterRange>(reg));
    return r;
}

void TSerialDevice::Prepare()
{
    SerialPort->Sleep(Delay);
}

void TSerialDevice::EndPollCycle() {}

void TSerialDevice::ReadRegisterRange(PRegisterRange range)
{
    PSimpleRegisterRange simple_range = std::dynamic_pointer_cast<TSimpleRegisterRange>(range);
    if (!simple_range)
        throw std::runtime_error("simple range expected");
    simple_range->Reset();
    for (auto reg: simple_range->RegisterList()) {
        try {
	    if (DeviceConfig()->GuardInterval.count()){
	        usleep(DeviceConfig()->GuardInterval.count());
	    }
            simple_range->SetValue(reg, ReadRegister(reg));
        } catch (const TSerialDeviceTransientErrorException& e) {
            simple_range->SetError(reg);
            std::ios::fmtflags f(std::cerr.flags());
            std::cerr << "TSerialDevice::ReadRegisterRange(): warning: " << e.what() << " [slave_id is "
                      << reg->Device()->ToString() + "]" << std::endl;
            std::cerr.flags(f);
        }
    }
}

void TSerialDevice::OnSuccessfulRead()
{
	LastSuccessfulRead = std::chrono::steady_clock::now();
	IsDisconnected = false;
}

void TSerialDevice::OnFailedRead()
{
	if (LastSuccessfulRead == std::chrono::steady_clock::time_point()) {
		LastSuccessfulRead = std::chrono::steady_clock::now();
	}

	if (std::chrono::steady_clock::now() - LastSuccessfulRead > _DeviceConfig->DeviceTimeout) {
		IsDisconnected = true;
	}
}

bool TSerialDevice::GetIsDisconnected() const
{
	return IsDisconnected;
}

std::unordered_map<std::string, PProtocol>
    *TSerialDeviceFactory::Protocols = 0;

void TSerialDeviceFactory::RegisterProtocol(PProtocol protocol)
{
    if (!Protocols)
        Protocols = new std::unordered_map<std::string, PProtocol>();

    Protocols->insert(std::make_pair(protocol->GetName(), protocol));
}

const PProtocol TSerialDeviceFactory::GetProtocolEntry(PDeviceConfig device_config)
{
    return TSerialDeviceFactory::GetProtocol(device_config->Protocol);
}

PProtocol TSerialDeviceFactory::GetProtocol(const std::string &name)
{
    auto it = Protocols->find(name);
    if (it == Protocols->end())
        throw TSerialDeviceException("unknown serial protocol: " + name);
    return it->second;
}

PSerialDevice TSerialDeviceFactory::CreateDevice(PDeviceConfig device_config, PAbstractSerialPort port)
{
    return GetProtocolEntry(device_config)->CreateDevice(device_config, port);
}

void TSerialDeviceFactory::RemoveDevice(PSerialDevice device)
{
    if (device) {
        device->Protocol()->RemoveDevice(device);
    } else {
        throw TSerialDeviceException("can't remove empty device");
    }
}

PRegisterTypeMap TSerialDeviceFactory::GetRegisterTypes(PDeviceConfig device_config)
{
    return GetProtocolEntry(device_config)->GetRegTypes();
}

template<>
int TBasicProtocolConverter<int>::ConvertSlaveId(const std::string &s) const
{
    try {
        return std::stoi(s, /* pos = */ 0, /* base = */ 0);
    } catch (const std::logic_error &e) {
        throw TSerialDeviceException("slave ID \"" + s + "\" is not convertible to string");
    }
}
