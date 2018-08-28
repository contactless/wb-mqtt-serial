#pragma once

#include <chrono>
#include <memory>
#include <string>

struct TPortSettings
{
    TPortSettings(const std::chrono::milliseconds & responseTimeout)
        : ResponseTimeout(responseTimeout)
    {}
    virtual ~TPortSettings() = default;

    virtual std::string ToString() const = 0;
    virtual std::string GetNamePostfix() const = 0;

    std::chrono::milliseconds ResponseTimeout;
};

using PPortSettings = std::shared_ptr<TPortSettings>;
