#pragma once

#include "port_settings.h"

#include <string>
#include <sstream>

struct TTcpPortSettings;

::std::ostream& operator<<(::std::ostream& os, const TTcpPortSettings & settings);

const int DEFAULT_PORT_FAIL_CYCLES = 2;

struct TTcpPortSettings final: TPortSettings
{
    TTcpPortSettings(const std::string & address = "localhost",
                     uint16_t port = 0,
                     const std::chrono::milliseconds & responseTimeout = std::chrono::milliseconds(500),
                     const std::chrono::milliseconds & connectionTimeout = std::chrono::milliseconds(5000),
                     int failCycles = DEFAULT_PORT_FAIL_CYCLES)
        : TPortSettings(responseTimeout)
        , Address(address)
        , Port(port)
        , ConnectionTimeout(connectionTimeout)
        , ConnectionMaxFailCycles(failCycles)
    {}

    std::string ToString() const override
    {
        std::ostringstream ss;
        ss << *this;
        return ss.str();
    }

    std::string GetNamePostfix() const override
    {
        return "(at " + Address + ":" + std::to_string(Port) + ")";
    }

    std::string                 Address;
    uint16_t                    Port;
    std::chrono::milliseconds   ConnectionTimeout;
    int                         ConnectionMaxFailCycles;
};

using PTcpPortSettings = std::shared_ptr<TTcpPortSettings>;

inline ::std::ostream& operator<<(::std::ostream& os, const TTcpPortSettings & settings) {
    return os << "<" << settings.Address << ":" << settings.Port <<
        " timeout " << settings.ResponseTimeout.count() << ">";
}
