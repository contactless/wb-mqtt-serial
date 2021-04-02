#pragma once

#include <string>
#include <sstream>
#include <memory>
#include <vector>
#include <exception>

#include "register.h"

#if defined(__APPLE__) || defined(__APPLE_CC__)
#   include <json/json.h>
#else
#   include <jsoncpp/json/json.h>
#endif

#include <wblib/json_utils.h>
#include <wblib/driver_args.h>

#include "port.h"
#include "serial_device.h"

struct TDeviceTemplate
{
    std::string Type;
    std::string Title;
    Json::Value Schema;

    TDeviceTemplate(const std::string& type, const std::string title, const Json::Value& schema);
};

class ITemplateMap
{
    public:
        virtual ~ITemplateMap() = default;

        virtual const TDeviceTemplate& GetTemplate(const std::string& deviceType) = 0;
        virtual std::vector<std::string> GetDeviceTypes() const = 0;
};

class TTemplateMap: public ITemplateMap
{
        /**
         *  @brief Device type to template file path mapping.
         *         Files in map are jsons with device_type parameter, but aren't yet validated against schema.
         */
        std::unordered_map<std::string, std::string>                       TemplateFiles;

        /**
         * @brief Device type to TDeviceTemplate mapping.
         *        Valid and parsed templates.
         */
        std::unordered_map<std::string, std::shared_ptr<TDeviceTemplate> > ValidTemplates;

        std::unique_ptr<WBMQTT::JSON::TValidator>                          Validator;

        Json::Value Validate(const std::string& deviceType, const std::string& filePath);
        std::shared_ptr<TDeviceTemplate> GetTemplatePtr(const std::string& deviceType);
    public:
        TTemplateMap() = default;

        /**
         * @brief Construct a new TTemplateMap object.
         *        Throws TConfigParserException if can't open templatesDir.
         * 
         * @param templatesDirs directory with templates
         * @param templateSchema JSON Schema for template file validation
         */
        TTemplateMap(const std::string& templatesDir, const Json::Value& templateSchema);

        /**
         * @brief Add templates from templatesDir to map.
         *        Throws TConfigParserException if can't open templatesDir.
         * 
         * @param templatesDir directory with templates
         */
        void AddTemplatesDir(const std::string& templatesDir);

        const TDeviceTemplate& GetTemplate(const std::string& deviceType) override;

        std::vector<std::string> GetDeviceTypes() const override;

        std::vector<std::shared_ptr<TDeviceTemplate>> GetTemplatesOrderedByName();
};

class TSubDevicesTemplateMap: public ITemplateMap
{
        std::unordered_map<std::string, TDeviceTemplate> Templates;
        std::string                                      DeviceType;
    public:
        TSubDevicesTemplateMap(const std::string& deviceType, const Json::Value& device);

        const TDeviceTemplate& GetTemplate(const std::string& deviceType) override;

        std::vector<std::string> GetDeviceTypes() const override;
};

struct TPortConfig 
{
    PPort                      Port;
    std::vector<PDeviceConfig> DeviceConfigs;
    std::chrono::milliseconds  PollInterval          = DefaultPollInterval;
    std::chrono::microseconds  RequestDelay          = std::chrono::microseconds::zero();

    /**
     * @brief Maximum allowed time from request to response for any device connected to the port.
     * -1 if not set, DefaultResponseTimeout will be used.
     * The timeout is used if device's ResponseTimeout is not set or if device's ResponseTimeout is smaller.
     */
    std::chrono::milliseconds  ResponseTimeout = std::chrono::milliseconds(-1);

    bool IsModbusTcp = false;

    void AddDeviceConfig(PDeviceConfig device_config, TSerialDeviceFactory& deviceFactory);
};

typedef std::shared_ptr<TPortConfig> PPortConfig;

struct THandlerConfig 
{
    bool                       Debug = false;
    WBMQTT::TPublishParameters PublishParameters;
    std::vector<PPortConfig>   PortConfigs;

    void AddPortConfig(PPortConfig portConfig);
};

typedef std::shared_ptr<THandlerConfig> PHandlerConfig;

class TConfigParserException: public std::runtime_error {
public:
    TConfigParserException(const std::string& message);
};

Json::Value LoadConfigTemplatesSchema(const std::string& templateSchemaFileName, const Json::Value& configSchema);
void AddProtocolType(Json::Value& configSchema, const std::string& protocolType);
void AddRegisterType(Json::Value& configSchema, const std::string& registerType);

typedef std::function<std::pair<PPort, bool>(const Json::Value& config)> TPortFactoryFn;
std::pair<PPort, bool> DefaultPortFactory(const Json::Value& port_data);

Json::Value LoadConfigSchema(const std::string& schemaFileName);

PHandlerConfig LoadConfig(const std::string& configFileName,
                          TSerialDeviceFactory& deviceFactory,
                          const Json::Value& configSchema,
                          TTemplateMap& templates,
                          TPortFactoryFn portFactory = DefaultPortFactory);

bool IsSubdeviceChannel(const Json::Value& channelSchema);

std::string GetDeviceKey(const std::string& deviceType);
std::string GetSubdeviceSchemaKey(const std::string& deviceType, const std::string& subDeviceType);
std::string GetSubdeviceKey(const std::string& subDeviceType);

void AppendParams(Json::Value& dst, const Json::Value& src);
void SetIfExists(Json::Value& dst, const std::string& dstKey, const Json::Value& src, const std::string& srcKey);
std::string DecorateIfNotEmpty(const std::string& prefix, const std::string& str, const std::string& postfix = std::string());
std::string GetProtocolName(const Json::Value& deviceDescription);