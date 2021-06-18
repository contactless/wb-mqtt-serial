#include "modbus_io_device.h"
#include "modbus_common.h"
#include "modbus_device.h"

namespace 
{
    const TRegisterTypes ModbusIORegisterTypes({
            { Modbus::REG_HOLDING, "holding", "value", U16 },
            { Modbus::REG_COIL, "coil", "switch", U8 },
            { Modbus::REG_DISCRETE, "discrete", "switch", U8, true },
            { Modbus::REG_INPUT, "input", "value", U16, true }
        });

    int GetSecondaryId(const std::string& fullId)
    {
        try {
            auto delimiter_it = fullId.find(':');
            return std::stoi(fullId.substr(delimiter_it + 1), 0, 0);
        } catch (const std::logic_error &e) {
            throw TSerialDeviceException("slave ID \"" + fullId + "\" is not convertible to modbus_io id");
        }
    }

    class TModbusIOProtocol: public IProtocol
    {
        std::unique_ptr<Modbus::IModbusTraitsFactory> ModbusTraitsFactory;
    public:
        TModbusIOProtocol(const char* name) : IProtocol(name, ModbusIORegisterTypes)
        {}

        bool IsSameSlaveId(const std::string& id1, const std::string& id2) const override
        {
            if (TUInt32SlaveId(id1) == TUInt32SlaveId(id2)) {
                return GetSecondaryId(id1) == GetSecondaryId(id2);
            }
            return false;
        }

        bool IsModbus() const override
        {
            return true;
        }
    };
}

void TModbusIODevice::Register(TSerialDeviceFactory& factory)
{
    factory.RegisterProtocol(new TModbusIOProtocol("modbus_io"), 
                             new TModbusDeviceFactory<TModbusIODevice>(std::make_unique<Modbus::TModbusRTUTraitsFactory>()));
    factory.RegisterProtocol(new TModbusIOProtocol("modbus_io-tcp"), 
                             new TModbusDeviceFactory<TModbusIODevice>(std::make_unique<Modbus::TModbusTCPTraitsFactory>()));
}

TModbusIODevice::TModbusIODevice(std::unique_ptr<Modbus::IModbusTraits> modbusTraits, PDeviceConfig config, PPort port, PProtocol protocol)
    : TSerialDevice(config, port, protocol), TUInt32SlaveId(config->SlaveId), ModbusTraits(std::move(modbusTraits))
{
    auto SecondaryId = GetSecondaryId(config->SlaveId);
    Shift = (((SecondaryId - 1) % 4) + 1) * DeviceConfig()->Stride + DeviceConfig()->Shift;
    config->FrameTimeout = std::max(config->FrameTimeout, port->GetSendTime(3.5));
}

std::list<PRegisterRange> TModbusIODevice::SplitRegisterList(const std::list<PRegister> & reg_list, bool enableHoles) const
{
    return Modbus::SplitRegisterList(reg_list, *DeviceConfig(), enableHoles);
}

void TModbusIODevice::WriteRegister(PRegister reg, uint64_t value)
{
    Modbus::WriteRegister(*ModbusTraits, *Port(), SlaveId, *reg, value, Shift);
}

std::list<PRegisterRange> TModbusIODevice::ReadRegisterRange(PRegisterRange range)
{
    return Modbus::ReadRegisterRange(*ModbusTraits, *Port(), SlaveId, range, Shift);
}

bool TModbusIODevice::WriteSetupRegisters()
{
    return Modbus::WriteSetupRegisters(*ModbusTraits, *Port(), SlaveId, SetupItems, Shift);
}
