#include <iomanip>
#include <iostream>

#include "register_handler.h"

TRegisterHandler::TRegisterHandler(PSerialDevice dev, PRegister reg, PBinarySemaphore flush_needed, bool debug)
    : Dev(dev), Reg(reg), FlushNeeded(flush_needed), Debug(debug) {}

TRegisterHandler::TErrorState TRegisterHandler::UpdateReadError(bool error) {
    TErrorState newState;
    if (error) {
        newState = ErrorState == WriteError ||
            ErrorState == ReadWriteError ?
            ReadWriteError : ReadError;
    } else {
        newState = ErrorState == ReadWriteError ||
            ErrorState == WriteError ?
            WriteError : NoError;
    }

    if (ErrorState == newState)
        return ErrorStateUnchanged;
    ErrorState = newState;
    return ErrorState;
}

TRegisterHandler::TErrorState TRegisterHandler::UpdateWriteError(bool error) {
    TErrorState newState;
    if (error) {
        newState = ErrorState == ReadError ||
            ErrorState == ReadWriteError ?
            ReadWriteError : WriteError;
    } else {
        newState = ErrorState == ReadWriteError ||
            ErrorState == ReadError ?
            ReadError : NoError;
    }

    if (ErrorState == newState)
        return ErrorStateUnchanged;
    ErrorState = newState;
    return ErrorState;
}

bool TRegisterHandler::NeedToPoll()
{
    std::lock_guard<std::mutex> lock(SetValueMutex);
    return Reg->Poll && !Dirty;
}

TRegisterHandler::TErrorState TRegisterHandler::AcceptDeviceValue(uint64_t new_value, bool ok, bool *changed)
{
    *changed = false;

    // don't poll write-only and dirty registers
    if (!NeedToPoll())
        return ErrorStateUnchanged;

    if (!ok)
        return UpdateReadError(true);


    bool first_poll = !DidReadReg;
    DidReadReg = true;

    if (Reg->HasErrorValue && Reg->ErrorValue == new_value) {
        if (Debug) {
            std::cerr << "register " << Reg->ToString() << " contains error value" << std::endl;
        }
        return UpdateReadError(true);
    }

    SetValueMutex.lock();
    if (Value != new_value) {
        if (Dirty) {
            SetValueMutex.unlock();
            return UpdateReadError(false);
        }

        Value = new_value;
        SetValueMutex.unlock();

        if (Debug) {
            std::ios::fmtflags f(std::cerr.flags());
            std::cerr << "new val for " << Reg->ToString() << ": " << std::hex << new_value << std::endl;
            std::cerr.flags(f);
        }
        *changed = true;
        return UpdateReadError(false);
    } else
        SetValueMutex.unlock();

    *changed = first_poll;
    return UpdateReadError(false);
}

bool TRegisterHandler::NeedToFlush()
{
    std::lock_guard<std::mutex> lock(SetValueMutex);
    return Dirty;
}

TRegisterHandler::TErrorState TRegisterHandler::Flush()
{
    if (!NeedToFlush())
        return ErrorStateUnchanged;

    {
        std::lock_guard<std::mutex> lock(SetValueMutex);
        Dirty = false;
    }

    try {
        Device()->WriteRegister(Reg, Value);
    } catch (const TSerialDeviceTransientErrorException& e) {
        std::ios::fmtflags f(std::cerr.flags());
        std::cerr << "TRegisterHandler::Flush(): warning: " << e.what() << " for device " <<
            Reg->Device()->ToString() <<  std::endl;
        std::cerr.flags(f);
        return UpdateWriteError(true);
    }
    return UpdateWriteError(false);
}

uint64_t TRegisterHandler::InvertWordOrderIfNeeded(const uint64_t value) const
{
    if (Reg->WordOrder == EWordOrder::BigEndian) {
        return value;
    }

    uint64_t result = 0;
    uint64_t cur_value = value;

    for (int i = 0; i < Reg->Width(); ++i) {
        uint16_t last_word = (((uint64_t) cur_value) & 0xFFFF);
        result <<= 16;
        result |= last_word;
        cur_value >>= 16;
    }
    return result;
}

std::string TRegisterHandler::TextValue() const
{
    return ConvertSlaveValue(InvertWordOrderIfNeeded(Value));
}

std::string TRegisterHandler::ConvertSlaveValue(uint64_t value) const
{
    switch (Reg->Format) {
    case S8:
        return ToScaledTextValue(int8_t(value & 0xff));
    case S16:
        return ToScaledTextValue(int16_t(value & 0xffff));
    case S24:
        {
            uint32_t v = value & 0xffffff;
            if (v & 0x800000) // fix sign (TBD: test)
                v |= 0xff000000;
            return ToScaledTextValue(int32_t(v));
        }
    case S32:
        return ToScaledTextValue(int32_t(value & 0xffffffff));
    case S64:
        return ToScaledTextValue(int64_t(value));
    case BCD8:
        return ToScaledTextValue(PackedBCD2Int(value, WordSizes::W8_SZ));
    case BCD16:
        return ToScaledTextValue(PackedBCD2Int(value, WordSizes::W16_SZ));
    case BCD24:
        return ToScaledTextValue(PackedBCD2Int(value, WordSizes::W24_SZ));
    case BCD32:
        return ToScaledTextValue(PackedBCD2Int(value, WordSizes::W32_SZ));
	case Float:
        {
            union {
                uint32_t raw;
                float v;
            } tmp;
            tmp.raw = value;
            return ToScaledTextValue(tmp.v);
        }
    case Double:
		{
            union {
                uint64_t raw;
                double v;
            } tmp;
            tmp.raw = value;
            return ToScaledTextValue(tmp.v);
		}
    case Char8:
        return std::string(1, value & 0xff);
    default:
        return ToScaledTextValue(value);
    }
}

void TRegisterHandler::SetTextValue(const std::string& v)
{
    {
        // don't hold the lock while notifying the client below
        std::lock_guard<std::mutex> lock(SetValueMutex);
        Dirty = true;
        Value = InvertWordOrderIfNeeded(ConvertMasterValue(v));
    }
    FlushNeeded->Signal();
}

uint64_t TRegisterHandler::ConvertMasterValue(const std::string& str) const
{
    switch (Reg->Format) {
    case S8:
        return FromScaledTextValue<int64_t>(str) & 0xff;
    case S16:
        return FromScaledTextValue<int64_t>(str) & 0xffff;
    case S24:
        return FromScaledTextValue<int64_t>(str) & 0xffffff;
    case S32:
        return FromScaledTextValue<int64_t>(str) & 0xffffffff;
    case S64:
        return FromScaledTextValue<int64_t>(str);
    case U8:
        return FromScaledTextValue<uint64_t>(str) & 0xff;
    case U16:
        return FromScaledTextValue<uint64_t>(str) & 0xffff;
    case U24:
        return FromScaledTextValue<uint64_t>(str) & 0xffffff;
    case U32:
        return FromScaledTextValue<uint64_t>(str) & 0xffffffff;
    case U64:
        return FromScaledTextValue<uint64_t>(str);
	case Float:
		{
            union {
                uint32_t raw;
                float v;
            } tmp;

			tmp.v = FromScaledTextValue<double>(str);
            return tmp.raw;
		}
	case Double:
		{
            union {
                uint64_t raw;
                double v;
            } tmp;
			tmp.v = FromScaledTextValue<double>(str);
            return tmp.raw;
		}
    case Char8:
        return str.empty() ? 0 : uint8_t(str[0]);
    case BCD8:
        return IntToPackedBCD(FromScaledTextValue<uint64_t>(str) & 0xFF, WordSizes::W8_SZ);
    case BCD16:
        return IntToPackedBCD(FromScaledTextValue<uint64_t>(str) & 0xFFFF, WordSizes::W16_SZ);
    case BCD24:
        return IntToPackedBCD(FromScaledTextValue<uint64_t>(str) & 0xFFFFFF, WordSizes::W24_SZ);
    case BCD32:
        return IntToPackedBCD(FromScaledTextValue<uint64_t>(str) & 0xFFFFFFFF, WordSizes::W32_SZ);
    default:
        return FromScaledTextValue<uint64_t>(str);
    }
}
