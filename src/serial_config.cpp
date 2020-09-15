#include "serial_config.h"
#include "log.h"

#include <set>
#include <fstream>
#include <stdexcept>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <cstdlib>
#include <memory>

#include "tcp_port_settings.h"
#include "serial_port_settings.h"

#define LOG(logger) ::logger.Log() << "[serial config] "

using namespace std;
using namespace WBMQTT::JSON;

namespace WBMQTT
{
    namespace JSON
    {
        template <> inline bool Is<std::chrono::milliseconds>(const Json::Value& value)
        {
            return value.isInt();
        }

        template <> inline std::chrono::milliseconds As<std::chrono::milliseconds>(const Json::Value& value)
        {
            return std::chrono::milliseconds(value.asInt());
        }

        template <> inline bool Is<std::chrono::microseconds>(const Json::Value& value)
        {
            return value.isInt();
        }

        template <> inline std::chrono::microseconds As<std::chrono::microseconds>(const Json::Value& value)
        {
            return std::chrono::microseconds(value.asInt());
        }
    }
}

namespace {
    const char* DefaultProtocol = "modbus";

    bool EndsWith(const string& str, const string& suffix)
    {
        return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    template <class T> T Read(const Json::Value& root, const std::string& key, const T& defaultValue) {
        T value;
        if (Get(root, key, value)) {
            return value;
        }
        return defaultValue;
    }

    int ToInt(const Json::Value& v, const std::string& title)
    {
        if (v.isInt())
            return v.asInt();

        if (v.isString()) {
            try {
                return std::stoi(v.asString(), /*pos= */ 0, /*base= */ 0);
            } catch (const std::logic_error& e) {}
        }

        throw TConfigParserException(
            title + ": plain integer or '0x..' hex string expected instead of '" + v.asString() +
            "'");
        // v.asString() should give a bit more information what config this exception came from
    }

    uint64_t ToUint64(const Json::Value& v, const string& title)
    {
        if (v.isUInt())
            return v.asUInt();
        if (v.isInt()) {
            int val = v.asInt();
            if (val >= 0) {
                return val;
            }
        }

        if (v.isString()) {
            auto val = v.asString();
            if (val.find("-") == std::string::npos) {
                // don't try to parse strings containing munus sign
                try {
                    return stoull(val, /*pos= */ 0, /*base= */ 0);
                } catch (const logic_error& e) {}
            }
        }

        throw TConfigParserException(
            title + ": 32 bit plain unsigned integer (64 bit when quoted) or '0x..' hex string expected instead of '" + v.asString() +
            "'");
    }

    int GetInt(const Json::Value& obj, const std::string& key)
    {
        return ToInt(obj[key], key);
    }

    PRegisterConfig LoadRegisterConfig(PDeviceConfig device_config,
                                       const Json::Value& register_data,
                                       std::string& default_type_str,
                                       const std::string& channel_name)
    {
        int address, bit_offset = 0, bit_width = 0;
        {
            const auto & addressValue = register_data["address"];
            if (addressValue.isString()) {
                const auto & addressStr = addressValue.asString();
                auto pos1 = addressStr.find(':');
                if (pos1 == string::npos) {
                    address = GetInt(register_data, "address");
                } else {
                    auto pos2 = addressStr.find(':', pos1 + 1);

                    address = stoi(addressStr.substr(0, pos1));
                    bit_offset = stoi(addressStr.substr(pos1 + 1, pos2));

                    if (bit_offset < 0 || bit_offset > 255) {
                        throw TConfigParserException("channel \"" + channel_name 
                                                      + "\" error during address parsing: bit shift must be in range [0, 255] (address string: '" 
                                                      + addressStr + "')");
                    }

                    if (pos2 != string::npos) {
                        bit_width = stoi(addressStr.substr(pos2 + 1));
                        if (bit_width < 0 || bit_width > 64) {
                            throw TConfigParserException("channel \"" + channel_name 
                                                      + "\" error during address parsing: bit count must be in range [0, 64] (address string: '"
                                                      + addressStr + "')");
                        }
                    }
                }
            } else {
                address = GetInt(register_data, "address");
            }
        }

        string reg_type_str = register_data["reg_type"].asString();
        default_type_str = "text";
        auto it = device_config->TypeMap->find(reg_type_str);
        if (it == device_config->TypeMap->end())
            throw TConfigParserException("channel \"" + channel_name + "\" invalid register type: " + reg_type_str + " -- " + device_config->DeviceType);
        if (!it->second.DefaultControlType.empty())
            default_type_str = it->second.DefaultControlType;

        RegisterFormat format = register_data.isMember("format") ?
            RegisterFormatFromName(register_data["format"].asString()) :
            it->second.DefaultFormat;

        EWordOrder word_order = register_data.isMember("word_order") ?
            WordOrderFromName(register_data["word_order"].asString()) :
            it->second.DefaultWordOrder;

        double scale        = Read(register_data, "scale",    1.0); // TBD: check for zero, too
        double offset       = Read(register_data, "offset",   0);
        double round_to     = Read(register_data, "round_to", 0.0);
        bool   readonly     = it->second.ReadOnly;
        if (Get(register_data, "readonly", readonly)) {
            if (it->second.ReadOnly && !readonly) {
                LOG(Warn) << "Channel \"" << channel_name << "\" unable to make register of type \"" << it->second.Name << "\" writable";
                readonly = true;
            }
        }

        bool has_error_value = false;
        uint64_t error_value = 0;
        if (register_data.isMember("error_value")) {
            has_error_value = true;
            error_value = ToUint64(register_data["error_value"], "error_value");
        }

        PRegisterConfig reg = TRegisterConfig::Create(
            it->second.Index,
            address, format, scale, offset, round_to, true, readonly,
            it->second.Name, has_error_value, error_value, word_order, bit_offset, bit_width);
        
        Get(register_data, "poll_interval", reg->PollInterval);
        return reg;
    }

    void LoadChannel(PDeviceConfig device_config, const Json::Value& channel_data)
    {
        std::string name = channel_data["name"].asString();
        std::string default_type_str;
        std::vector<PRegisterConfig> registers;
        if (channel_data.isMember("consists_of")) {

            std::chrono::milliseconds poll_interval(Read(channel_data, "poll_interval", std::chrono::milliseconds(-1)));

            const Json::Value& reg_data = channel_data["consists_of"];
            for(Json::ArrayIndex i = 0; i < reg_data.size(); ++i) {
                std::string def_type;
                auto reg = LoadRegisterConfig(device_config, reg_data[i], def_type, name);
                /* the poll_interval specified for the specific register has a precedence over the one specified for the compound channel */
                if ((reg->PollInterval.count() < 0) && (poll_interval.count() >= 0))
                    reg->PollInterval = poll_interval;
                registers.push_back(reg);
                if (!i)
                    default_type_str = def_type;
                else if (registers[i]->ReadOnly != registers[0]->ReadOnly)
                    throw TConfigParserException(("can't mix read-only and writable registers "
                                                "in one channel -- ") + device_config->DeviceType);
            }
        } else {
            registers.push_back(LoadRegisterConfig(device_config, channel_data, default_type_str, name));
        }

        std::string type_str(Read(channel_data, "type", default_type_str));
        if (type_str == "wo-switch") {
            type_str = "switch";
            for (auto& reg: registers)
                reg->Poll = false;
        }

        std::string on_value;
        if (channel_data.isMember("on_value")) {
            if (registers.size() != 1)
                throw TConfigParserException("can only use on_value for single-valued controls -- " +
                                            device_config->DeviceType);
            on_value = std::to_string(GetInt(channel_data, "on_value"));
        }

        int max = -1;
        if (channel_data.isMember("max"))
            max = GetInt(channel_data, "max");

        int order        = device_config->NextOrderValue();
        PDeviceChannelConfig channel(new TDeviceChannelConfig(name, type_str, device_config->Id, order,
                                                on_value, max, registers[0]->ReadOnly,
                                                registers));
        device_config->AddChannel(channel);
    }

    void MergeChannelProperties(Json::Value& dst, const Json::Value& src, const std::string& logPrefix)
    {
        for (auto itProp = src.begin(); itProp != src.end(); ++itProp) {
            if (itProp.name() == "poll_interval") {
                dst[itProp.name()] = *itProp;
                LOG(Info) << logPrefix << " override property \"" << itProp.name() << "\"";
                continue;
            }
            if (itProp.name() == "readonly") {
                if ((itProp ->asString() != "true") && (dst.isMember(itProp.name())) && (dst[itProp.name()].asString() == "true")) {
                    LOG(Warn) << logPrefix << " can't override property \"" << itProp.name() << "\"";
                    continue;
                }
                if (dst[itProp.name()] != *itProp) {
                    dst[itProp.name()] = *itProp;
                    LOG(Info) << logPrefix << " override property \"" << itProp.name() << "\"";
                }
                continue;
            }
            if (itProp.name() == "name") {
                continue;
            }
            LOG(Warn) << logPrefix << " can't override property \"" << itProp.name() << "\"";
        }
    }

    void UpdateChannels(Json::Value& dst, const Json::Value& src, const std::string& logPrefix)
    {
        std::unordered_map<std::string, Json::ArrayIndex> channelNames;

        for (Json::ArrayIndex i = 0; i < dst.size(); ++i) {
            channelNames.emplace(dst[i]["name"].asString(), i);
        }

        for (const auto& elem: src) {
            string channelName(elem["name"].asString());
            if (channelNames.count(channelName)) {
                MergeChannelProperties(dst[channelNames[channelName]], elem, logPrefix + " channel \"" + channelName + "\"");
            } else {
                dst.append(elem);
            }
        }
    }

    void AppendUserData(Json::Value& dst, const Json::Value& src, const std::string& deviceName)
    {
        for (auto itProp = src.begin(); itProp != src.end(); ++itProp) {
            if (itProp.name() != "channels") {
                LOG(Info) << deviceName << " override property " << itProp.name();
                dst[itProp.name()] = *itProp;
            }
        }
        UpdateChannels(dst["channels"], src["channels"], "\"" + deviceName + "\"");
    }

    void LoadSetupItem(PDeviceConfig device_config, const Json::Value& item_data)
    {
        int address = GetInt(item_data, "address");
        std::string reg_type_str = item_data["reg_type"].asString();
        int type = 0;
        if (!reg_type_str.empty()) {
            auto it = device_config->TypeMap->find(reg_type_str);
            if (it == device_config->TypeMap->end())
                throw TConfigParserException("invalid setup register type: " +
                                            reg_type_str + " -- " + device_config->DeviceType);
            type = it->second.Index;
        }
        RegisterFormat format = U16;
        if (item_data.isMember("format"))
            format = RegisterFormatFromName(item_data["format"].asString());
        PRegisterConfig reg = TRegisterConfig::Create(
            type, address, format, 1, 0, 0, true, true, "<unspec>");

        int value = GetInt(item_data, "value");
        std::string name(Read(item_data, "title", std::string("<unnamed>")));
        device_config->AddSetupItem(PDeviceSetupItemConfig(new TDeviceSetupItemConfig(name, reg, value)));
    }

    void LoadDeviceTemplatableConfigPart(PDeviceConfig device_config, const Json::Value& device_data, TGetRegisterTypeMapFn getRegisterTypeMapFn)
    {
        Get(device_data, "protocol", device_config->Protocol);
        if (device_config->Protocol.empty()) {
            device_config->Protocol = DefaultProtocol;
        }

        device_config->TypeMap = getRegisterTypeMapFn(device_config);

        if (device_data.isMember("setup")) {
            for(const auto& setupItem: device_data["setup"])
                LoadSetupItem(device_config, setupItem);
        }

        if (device_data.isMember("password")) {
            device_config->Password.clear();
            for(const auto& passwordItem: device_data["password"])
                device_config->Password.push_back(ToInt(passwordItem, "password item"));
        }

        Get(device_data, "delay_ms", device_config->Delay);
        if (Get(device_data, "delay_usec", device_config->Delay)) // compat
            device_config->Delay = device_config->Delay / 1000;

        Get(device_data, "frame_timeout_ms",       device_config->FrameTimeout);
        Get(device_data, "device_timeout_ms",      device_config->DeviceTimeout);
        Get(device_data, "device_max_fail_cycles", device_config->DeviceMaxFailCycles);
        Get(device_data, "max_reg_hole",           device_config->MaxRegHole);
        Get(device_data, "max_bit_hole",           device_config->MaxBitHole);
        Get(device_data, "max_read_registers",     device_config->MaxReadRegisters);
        Get(device_data, "guard_interval_us",      device_config->GuardInterval);
        Get(device_data, "stride",                 device_config->Stride);
        Get(device_data, "shift",                  device_config->Shift);
        Get(device_data, "access_level",           device_config->AccessLevel);

        if (device_data.isMember("channels")) {
            for (const auto& channel_data: device_data["channels"]) {
                LoadChannel(device_config, channel_data);
            }
        }
    }

    void LoadDevice(PPortConfig port_config,
                    const Json::Value& device_data,
                    const std::string& default_id,
                    TTemplateMap& templates,
                    TGetRegisterTypeMapFn getRegisterTypeMapFn)
    {
        if (device_data.isMember("enabled") && !device_data["enabled"].asBool())
            return;

        PDeviceConfig device_config = std::make_shared<TDeviceConfig>();
        device_config->Id = Read(device_data, "id",  default_id);
        Get(device_data, "name", device_config->Name);

        if (device_data["slave_id"].isString())
            device_config->SlaveId = device_data["slave_id"].asString();
        else // legacy
            device_config->SlaveId = std::to_string(device_data["slave_id"].asInt());

        auto device_poll_interval = std::chrono::milliseconds(-1);
        Get(device_data, "poll_interval", device_poll_interval);

        if (device_data.isMember("device_type")) {
            device_config->DeviceType = device_data["device_type"].asString();
            auto tmpl = templates.GetTemplate(device_config->DeviceType);
            if (tmpl.isMember("name")) {
                if (device_config->Name == "") {
                    device_config->Name = tmpl["name"].asString() + " " + device_config->SlaveId;
                }
            } else {
                if (device_config->Name == "") {
                    throw TConfigParserException(
                        "Property name is missing in " + device_config->DeviceType + " template");
                }
            }

            if (tmpl.isMember("id")) {
                if (device_config->Id == default_id)
                    device_config->Id = tmpl["id"].asString() + "_" + device_config->SlaveId;
            }
            AppendUserData(tmpl, device_data, device_config->Name);
            LoadDeviceTemplatableConfigPart(device_config, tmpl, getRegisterTypeMapFn);
        } else {
            LoadDeviceTemplatableConfigPart(device_config, device_data, getRegisterTypeMapFn);
        }
        if (device_config->DeviceChannelConfigs.empty())
            throw TConfigParserException("the device has no channels: " + device_config->Name);

        if (device_config->GuardInterval.count() == 0) {
            device_config->GuardInterval = port_config->GuardInterval;
        }

        port_config->AddDeviceConfig(device_config);
        for (auto channel: device_config->DeviceChannelConfigs) {
            for (auto reg: channel->RegisterConfigs) {
                if (reg->PollInterval.count() < 0) {
                    reg->PollInterval = ((device_poll_interval.count() >= 0) ? device_poll_interval : port_config->PollInterval);
                }
            }
        }
    }

    void LoadPort(PHandlerConfig handlerConfig, const Json::Value& port_data, const std::string& id_prefix, TTemplateMap& templates, TGetRegisterTypeMapFn getRegisterTypeMapFn)
    {
        if (port_data.isMember("enabled") && !port_data["enabled"].asBool())
            return;

        auto port_config = make_shared<TPortConfig>();

        auto port_type = port_data.get("port_type", "serial").asString();

        if (port_type == "serial") {
            auto serial_port_settings = make_shared<TSerialPortSettings>(port_data["path"].asString());

            Get(port_data, "baud_rate", serial_port_settings->BaudRate);

            if (port_data.isMember("parity"))
                serial_port_settings->Parity = port_data["parity"].asCString()[0];

            Get(port_data, "data_bits", serial_port_settings->DataBits);
            Get(port_data, "stop_bits", serial_port_settings->StopBits);

            port_config->ConnSettings = serial_port_settings;
        } else {
            if (port_type != "tcp") {
                throw TConfigParserException("invalid port_type: '" + port_type + "'");
            }

            auto tcp_port_settings = make_shared<TTcpPortSettings>(port_data["address"].asString(), GetInt(port_data, "port"));

            Get(port_data, "connection_timeout_ms",      tcp_port_settings->ConnectionTimeout);
            Get(port_data, "connection_max_fail_cycles", tcp_port_settings->ConnectionMaxFailCycles);

            port_config->ConnSettings = tcp_port_settings;
        }

        Get(port_data, "response_timeout_ms", port_config->ConnSettings->ResponseTimeout);
        Get(port_data, "poll_interval",       port_config->PollInterval);
        Get(port_data, "guard_interval_us",   port_config->GuardInterval);

        const Json::Value& array = port_data["devices"];
        for(Json::Value::ArrayIndex index = 0; index < array.size(); ++index)
            LoadDevice(port_config, array[index], id_prefix + std::to_string(index), templates, getRegisterTypeMapFn);

        handlerConfig->AddPortConfig(port_config);
    }
}

TTemplateMap::TTemplateMap(const std::string& templatesDir, const Json::Value& templateSchema): 
    Validator(new  WBMQTT::JSON::TValidator(templateSchema)) 
{
    DIR *dir;
    struct dirent *dirp;
    struct stat filestat;

    if ((dir = opendir(templatesDir.c_str())) == NULL)
        throw TConfigParserException("Cannot open " + templatesDir + " directory");

    while ((dirp = readdir(dir))) {
        std::string dname = dirp->d_name;
        if(!EndsWith(dname, ".json"))
            continue;

        std::string filepath = templatesDir + "/" + dname;
        if (stat(filepath.c_str(), &filestat)) continue;
        if (S_ISDIR(filestat.st_mode)) continue;

        try {
            Json::Value root = WBMQTT::JSON::Parse(filepath);
            TemplateFiles[root["device_type"].asString()] = filepath;
        } catch (const std::exception& e) {
            LOG(Error) << "Failed to parse " << filepath << "\n" << e.what();
            continue;
        }
    }
    closedir(dir);
}

const Json::Value& TTemplateMap::GetTemplate(const std::string& deviceType) 
{
    if (!Validator) {
        throw std::runtime_error("Can't find template for " + deviceType);
    }
    try {
        return ValidTemplates.at(deviceType);
    } catch ( const std::out_of_range& ) {
        std::string filePath;
        try {
            filePath = TemplateFiles.at(deviceType);
        } catch ( const std::out_of_range& ) {
            throw std::runtime_error("Can't find template for " + deviceType);
        }
        Json::Value root(WBMQTT::JSON::Parse(filePath));
        Validator->Validate(root);
        TemplateFiles.erase(filePath);
        ValidTemplates.emplace(filePath, root["device"]);
        return ValidTemplates[filePath];
    }
}

Json::Value LoadConfigTemplatesSchema(const std::string& templateSchemaFileName, const Json::Value& configSchema)
{
    Json::Value schema = WBMQTT::JSON::Parse(templateSchemaFileName);
    schema["definitions"] = configSchema["definitions"];
    schema["definitions"]["device"]["properties"].removeMember("slave_id");
    schema["definitions"]["device"].removeMember("required");
    return schema;
}

void AddProtocolType(Json::Value& configSchema, const std::string& protocolType)
{
    configSchema["definitions"]["deviceProtocol"]["enum"].append(protocolType);
}

void AddRegisterType(Json::Value& configSchema, const std::string& registerType)
{
    configSchema["definitions"]["reg_type"]["enum"].append(registerType);
}

Json::Value LoadConfigSchema(const std::string& schemaFileName)
{
    Json::Value configSchema(Parse(schemaFileName));
    // We use nonstandard syntax for #/definitions/device/properties/device_type in enum field
    // "enum": {
    //     "directories": ["/usr/share/wb-mqtt-serial/templates"],
    //     "pointer": "/device_type",
    //     "pattern": "^.*\\.json$" },
    // Validator will complain about it. So let's remove it.
    configSchema["definitions"]["device"]["properties"]["device_type"].removeMember("enum");
    return configSchema;
}

PHandlerConfig LoadConfig(const std::string& configFileName, 
                          TGetRegisterTypeMapFn getRegisterTypeMapFn,
                          const Json::Value& configSchema,
                          TTemplateMap& templates)
{
    PHandlerConfig handlerConfig(new THandlerConfig);
    Json::Value Root(Parse(configFileName));

    Validate(Root, configSchema);

    Get(Root, "debug", handlerConfig->Debug);
    Get(Root, "max_unchanged_interval", handlerConfig->MaxUnchangedInterval);

    const Json::Value& array = Root["ports"];
    for(Json::Value::ArrayIndex index = 0; index < array.size(); ++index)
        LoadPort(handlerConfig, array[index], "wb-modbus-" + std::to_string(index) + "-", templates, getRegisterTypeMapFn); // XXX old default prefix for compat

    // check are there any devices defined
    for (const auto& port_config : handlerConfig->PortConfigs) {
        if (!port_config->DeviceConfigs.empty()) { // found one
            return handlerConfig;
        }
    }

    throw TConfigParserException("no devices defined in config. Nothing to do");
}

PHandlerConfig LoadConfig(const std::string& configFileName,
                          TGetRegisterTypeMapFn getRegisterTypeMapFn,
                          const Json::Value& configSchema)
{
    TTemplateMap t;
    return LoadConfig(configFileName, getRegisterTypeMapFn, configSchema, t);
}

void TPortConfig::AddDeviceConfig(PDeviceConfig device_config)
{
    // try to found duplicate of this device
    for (PDeviceConfig dev : DeviceConfigs) {
        if (dev->SlaveId == device_config->SlaveId &&
            dev->Protocol == device_config->Protocol)
            throw TConfigParserException("device redefinition: " + dev->Protocol + ":" + dev->SlaveId);
    }

    DeviceConfigs.push_back(device_config);
}
