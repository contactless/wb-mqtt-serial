#include <set>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <cstdlib>
#include <memory>

#include "serial_config.h"
#include "serial_port_settings.h"
#include "tcp_port_settings.h"

using namespace std;

namespace {
    const char* DefaultProtocol = "modbus";

    bool EndsWith(const string& str, const string& suffix)
    {
        return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
    }

    using TIndexedJsonArray = map<string, Json::ArrayIndex>;

    class TPropertyIsEmptyError {};
    class TIsNotArrayError {};
    class TIsNotObjectError {};

    TIndexedJsonArray IndexArrayOfObjectsBy(const string & property_name, const Json::Value & array)
    {
        if (!array.isArray()) {
            throw TIsNotArrayError();
        }

        TIndexedJsonArray result;

        for(Json::ArrayIndex i = 0; i < array.size(); ++i) {
            const auto & object = array[i];
            if (!object.isObject()) {
                throw TIsNotObjectError();
            }

            const auto & property = object[property_name].asString();
            if (property.empty()) {
                throw TPropertyIsEmptyError();
            }

            result[property] = i;
        }

        return result;
    }
}

TTemplate::TTemplate(const Json::Value& device_data):
    DeviceData(device_data)
{
    const Json::Value channels = DeviceData["channels"];
    if (!channels.isArray())
        throw TConfigParserException("template channels member must be an object");
    for(Json::ArrayIndex i = 0; i < channels.size(); ++i) {
        const auto& channel_data = channels[i];
        if (!channel_data.isObject())
            throw TConfigParserException("template channel definition is not an object");
        if (!channel_data.isMember("name"))
            throw TConfigParserException("template channel without name");
        string name = channel_data["name"].asString();
        if (name.empty())
            throw TConfigParserException("template channel with empty name");
    }
}

TConfigTemplateParser::TConfigTemplateParser(const string& template_config_dir, bool debug)
    : DirectoryName(template_config_dir),
      Debug(debug),
      Templates(new TTemplateMap) {}

PTemplateMap TConfigTemplateParser::Parse()
{
    DIR *dir;
    struct dirent *dirp;
    struct stat filestat;

    if ((dir = opendir(DirectoryName.c_str())) == NULL)
        throw TConfigParserException("Cannot open templates directory");

    while ((dirp = readdir(dir))) {
        string dname = dirp->d_name;
        if(!EndsWith(dname, ".json"))
            continue;

        string filepath = DirectoryName + "/" + dname;
        if (stat(filepath.c_str(), &filestat)) continue;
        if (S_ISDIR(filestat.st_mode)) continue;

        ifstream input_stream;
        input_stream.open(filepath);
        if (!input_stream.is_open()) {
            cerr << "Error while trying to open template config file " << filepath << endl;
            continue;
        }

        Json::Reader reader;
        Json::Value root;
        if (!reader.parse(input_stream, root, false)) {
            cerr << "Failed to parse JSON: " << filepath << " " << reader.getFormattedErrorMessages()
                      << endl;
            continue;
        }

        LoadDeviceTemplate(root, filepath);
    }

    closedir(dir);
    return Templates;
}

void TConfigTemplateParser::LoadDeviceTemplate(const Json::Value& root, const string& filepath)
{
    if (!root.isObject())
        throw TConfigParserException("malformed config in file " + filepath);

    if (root.isMember("device_type")) {
        try {
            shared_ptr<TTemplate> t = make_shared<TTemplate>(root["device"]);
            (*Templates)[root["device_type"].asString()] = t;
        } catch (TConfigParserException &e){
            if (Debug) cerr << "malformed template " << filepath << ": " << e.what() << endl;
        }
    } else if (Debug) {
        cerr << "no device_type in json template in file " << filepath << endl;
    }
}

TConfigParser::TConfigParser(const string& config_fname, bool force_debug,
                             TGetRegisterTypeMap get_register_type_map,
                             PTemplateMap templates)
    : ConfigFileName(config_fname),
      HandlerConfig(new THandlerConfig),
      GetRegisterTypeMap(get_register_type_map),
      Templates(templates)
{
    HandlerConfig->Debug = force_debug;
}

PRegisterConfig TConfigParser::LoadRegisterConfig(PDeviceConfig device_config,
                                      const Json::Value& register_data,
                                      string& default_type_str)
{
    int address = GetInt(register_data, "address");
    cout << "address: " << address << endl;
    string reg_type_str = register_data["reg_type"].asString();
    default_type_str = "text";
    auto it = device_config->TypeMap->find(reg_type_str);
    if (it == device_config->TypeMap->end())
        throw TConfigParserException("invalid register type: " + reg_type_str + " -- " + device_config->DeviceType);
    if (!it->second.DefaultControlType.empty())
        default_type_str = it->second.DefaultControlType;

    RegisterFormat format = register_data.isMember("format") ?
        RegisterFormatFromName(register_data["format"].asString()) :
        it->second.DefaultFormat;

    EWordOrder word_order = register_data.isMember("word_order") ?
        WordOrderFromName(register_data["word_order"].asString()) :
        it->second.DefaultWordOrder;

    double scale = 1;
    if (register_data.isMember("scale"))
        scale = register_data["scale"].asDouble(); // TBD: check for zero, too

    double offset = 0;
    if (register_data.isMember("offset"))
        offset = register_data["offset"].asDouble();

    double round_to = 0.0;
    if (register_data.isMember("round_to")) {
        round_to = register_data["round_to"].asDouble();
        if (round_to < 0) {
            throw TConfigParserException("round_to must be greater than or equal to 0 -- " +
                                         device_config->DeviceType);
        }
    }

    bool force_readonly = false;
    if (register_data.isMember("readonly"))
        force_readonly = register_data["readonly"].asBool();

    bool has_error_value = false;
    uint64_t error_value = 0;
    if (register_data.isMember("error_value")) {
        has_error_value = true;
        error_value = ToUint64(register_data["error_value"], "error_value");
    }

    PRegisterConfig reg = TRegisterConfig::Create(
        it->second.Index,
        address, format, scale, offset, round_to, true, force_readonly || it->second.ReadOnly,
        it->second.Name, has_error_value, error_value, word_order);
    if (register_data.isMember("poll_interval"))
        reg->PollInterval = chrono::milliseconds(GetInt(register_data, "poll_interval"));
    return reg;
}

void TConfigParser::MergeAndLoadChannels(PDeviceConfig device_config, const Json::Value& device_data, PTemplate tmpl)
{
    TIndexedJsonArray device_channels_index_by_name;
    set<Json::ArrayIndex> loaded;

    if (device_data.isMember("channels")) {
        const auto & device_channels = device_data["channels"];
        try {
            device_channels_index_by_name = IndexArrayOfObjectsBy("name", device_channels);
        } catch (const TIsNotArrayError &) {
            throw TConfigParserException("device channels member must be an array");
        } catch (const TIsNotObjectError &) {
            throw TConfigParserException("channel definition is not an object");
        } catch (const TPropertyIsEmptyError &) {
            throw TConfigParserException("channel name is empty");
        }
    }

    if (tmpl) {
        // Load template channels first
        auto template_channels = tmpl->DeviceData["channels"];
        const auto & device_channels = device_data["channels"];

        for (Json::ArrayIndex i = 0; i < template_channels.size(); ++i) {
            auto & channel_data = template_channels[i];
            const auto & name = channel_data["name"].asString();

            auto it = device_channels_index_by_name.find(name);
            if (it != device_channels_index_by_name.end()) {
                const Json::Value & override_channel_data = device_channels[it->second];

                for (auto it = override_channel_data.begin(); it != override_channel_data.end(); ++it) {
                    cerr << "override property " << it.memberName() << endl;
                    // Channel fields from current device config
                    // take precedence over template field values
                    channel_data[it.memberName()] = *it;
                }
                loaded.insert(it->second);
            }

            LoadChannel(device_config, channel_data);
        }
    }

    if (device_data.isMember("channels")) {
        // Load remaining channels that were declared in main config
        const auto & device_channels = device_data["channels"];
        for (Json::ArrayIndex i = 0; i < device_channels.size(); ++i) {
            if (loaded.count(i)) {
                continue;
            }
            LoadChannel(device_config, device_channels[i]);
        }
    }
}

void TConfigParser::LoadChannel(PDeviceConfig device_config, const Json::Value& channel_data)
{
    if (!channel_data.isObject())
        throw TConfigParserException("malformed config -- " + device_config->DeviceType);

    string name = channel_data["name"].asString();
    if (name.empty())
        throw TConfigParserException("channel name is empty");

    string default_type_str;
    vector<PRegisterConfig> registers;
    if (channel_data.isMember("consists_of")) {
        const Json::Value reg_data = channel_data["consists_of"];
        if (!reg_data.isArray())
            throw TConfigParserException("consists_of must be an array");
        chrono::milliseconds poll_interval(-1);
        if (channel_data.isMember("poll_interval"))
            poll_interval = chrono::milliseconds(GetInt(channel_data, "poll_interval"));
        for(Json::ArrayIndex i = 0; i < reg_data.size(); ++i) {
            string def_type;
            auto reg = LoadRegisterConfig(device_config, reg_data[i], def_type);
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
        if (!registers.size())
            throw TConfigParserException("empty \"consists_of\" section -- " + device_config->DeviceType);
    } else
        registers.push_back(LoadRegisterConfig(device_config, channel_data, default_type_str));

    string type_str = channel_data["type"].asString();
    if (type_str.empty())
        type_str = default_type_str;
    if (type_str == "wo-switch") {
        type_str = "switch";
        for (auto& reg: registers)
            reg->Poll = false;
    }

    string on_value = "";
    if (channel_data.isMember("on_value")) {
        if (registers.size() != 1)
            throw TConfigParserException("can only use on_value for single-valued controls -- " +
                                         device_config->DeviceType);
        on_value = to_string(GetInt(channel_data, "on_value"));
    }

    int max = -1;
    if (channel_data.isMember("max"))
        max = GetInt(channel_data, "max");

    int order = device_config->NextOrderValue();
    PDeviceChannelConfig channel(new TDeviceChannelConfig(name, type_str, device_config->Id, order,
                                              on_value, max, registers[0]->ReadOnly,
                                              registers));
    device_config->AddChannel(channel);
}

void TConfigParser::LoadSetupItem(PDeviceConfig device_config, const Json::Value& item_data)
{
    if (!item_data.isObject())
        throw TConfigParserException("malformed config -- " + device_config->DeviceType);

    string name = item_data.isMember("title") ?
        item_data["title"].asString() : "<unnamed>";
    if (!item_data.isMember("address"))
        throw TConfigParserException("no address specified for init item");

    int address = GetInt(item_data, "address");
    string reg_type_str = item_data["reg_type"].asString();
    int type = 0;
    string type_name = "<unspec>";
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
        type, address, format, 1, 0, 0, true, true, type_name);

    if (!item_data.isMember("value"))
        throw TConfigParserException("no reg specified for init item");
    int value = GetInt(item_data, "value");
    device_config->AddSetupItem(PDeviceSetupItemConfig(new TDeviceSetupItemConfig(name, reg, value)));
}

void TConfigParser::LoadDeviceTemplatableConfigPart(PDeviceConfig device_config, const Json::Value& device_data)
{
    if (device_data.isMember("protocol"))
        device_config->Protocol = device_data["protocol"].asString();
    else if (device_config->Protocol.empty())
        device_config->Protocol = DefaultProtocol;
    device_config->TypeMap = GetRegisterTypeMap(device_config);

    if (device_data.isMember("setup")) {
        const Json::Value array = device_data["setup"];
        for(unsigned int index = 0; index < array.size(); ++index)
            LoadSetupItem(device_config, array[index]);
    }

    if (device_data.isMember("password")) {
        const Json::Value array = device_data["password"];
        device_config->Password.clear();
        for(unsigned int index = 0; index < array.size(); ++index)
            device_config->Password.push_back(ToInt(array[index], "password item"));
    }

    if (device_data.isMember("delay_usec")) // compat
        device_config->Delay = chrono::milliseconds(GetInt(device_data, "delay_usec") / 1000);
    else if (device_data.isMember("delay_ms"))
        device_config->Delay = chrono::milliseconds(GetInt(device_data, "delay_ms"));

    if (device_data.isMember("frame_timeout_ms"))
        device_config->FrameTimeout = chrono::milliseconds(GetInt(device_data, "frame_timeout_ms"));
    if (device_data.isMember("device_timeout_ms"))
        device_config->DeviceTimeout = chrono::milliseconds(GetInt(device_data, "device_timeout_ms"));
    if (device_data.isMember("device_max_fail_cycles"))
        device_config->DeviceMaxFailCycles = GetInt(device_data, "device_max_fail_cycles");
    if (device_data.isMember("max_reg_hole"))
        device_config->MaxRegHole = GetInt(device_data, "max_reg_hole");
    if (device_data.isMember("max_bit_hole"))
        device_config->MaxBitHole = GetInt(device_data, "max_bit_hole");
    if (device_data.isMember("max_read_registers"))
        device_config->MaxReadRegisters = GetInt(device_data, "max_read_registers");
    if (device_data.isMember("guard_interval_us"))
        device_config->GuardInterval = chrono::microseconds(GetInt(device_data, "guard_interval_us"));
    if (device_data.isMember("stride"))
        device_config->Stride = GetInt(device_data, "stride");
    if (device_data.isMember("shift"))
        device_config->Shift = GetInt(device_data, "shift");
}

int TConfigParser::ToInt(const Json::Value& v, const string& title)
{
    if (v.isInt())
        return v.asInt();

    if (v.isString()) {
        try {
            return stoi(v.asString(), /*pos= */ 0, /*base= */ 0);
        } catch (const logic_error& e) {}
    }

    throw TConfigParserException(
        title + ": plain integer or '0x..' hex string expected instead of '" + v.asString() +
        "'");
    // v.asString() should give a bit more information what config this exception came from
}

uint64_t TConfigParser::ToUint64(const Json::Value& v, const string& title)
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

int TConfigParser::GetInt(const Json::Value& obj, const string& key)
{
    return ToInt(obj[key], key);
}

PHandlerConfig TConfigParser::Parse()
{
    Json::Reader reader;
    if (ConfigFileName.empty())
        throw TConfigParserException("Please specify config file with -c option");

    ifstream myfile (ConfigFileName);
    if (myfile.fail())
        throw TConfigParserException("Serial driver configuration file not found: " + ConfigFileName);

    if(!reader.parse(myfile, Root, false))
        throw TConfigParserException("Failed to parse JSON: " + reader.getFormatedErrorMessages());

    LoadConfig();

    // check duplicate devices
    ResolveDeviceCollisions();

    return HandlerConfig;
}

void TConfigParser::ResolveDeviceCollisions() const
{
    unordered_map<string, pair<int, PDeviceConfig>> seen_ids;
    unordered_map<string, map<int, set<PDeviceConfig>>> collisions_by_id;

    unordered_map<string, pair<PPortConfig, PDeviceConfig>> seen_names;
    unordered_map<string, map<PPortConfig, set<PDeviceConfig>>> collisions_by_name;

    int port_index = 0;
    for (auto port_config: HandlerConfig->PortConfigs) {
        for (auto device_config: port_config->DeviceConfigs) {
            {   // ids
                auto ins_res = seen_ids.insert({device_config->Id, {port_index, device_config}});

                if (!ins_res.second) {   // collision
                    auto & collisions = collisions_by_id[device_config->Name];
                    collisions[ins_res.first->second.first].insert(ins_res.first->second.second);
                    collisions[port_index].insert(device_config);
                }
            }

            {   // names
                auto ins_res = seen_names.insert({device_config->Name, {port_config, device_config}});

                if (!ins_res.second) {   // collision
                    auto & collisions = collisions_by_name[device_config->Name];
                    collisions[ins_res.first->second.first].insert(ins_res.first->second.second);
                    collisions[port_config].insert(device_config);
                }
            }
        }
        ++port_index;
    }

    for (const auto & id_collisions: collisions_by_id) {
        auto port_count = id_collisions.second.size();

        if (port_count == 1) {   // no point in differentiating by port if there is only one port
            continue;
        }

        for (const auto & port_index_device: id_collisions.second) {
            const auto port_index = port_index_device.first;

            for (const auto & device_config: port_index_device.second) {
                device_config->Id += "_" + to_string(port_index);

                for (const auto & device_channel_config: device_config->DeviceChannelConfigs) {
                    device_channel_config->DeviceId = device_config->Id;
                }
            }
        }
    }

    for (const auto & name_collisions: collisions_by_name) {
        auto port_count = name_collisions.second.size();

        for (const auto & port_device: name_collisions.second) {
            const auto & port_config = port_device.first;
            auto device_count = port_device.second.size();
            int n = 1;
            for (const auto & device_config: port_device.second) {
                if (port_count > 1)
                    device_config->Name += " " + port_config->ConnSettings->GetNamePostfix();

                if (device_count > 1)
                    device_config->Name += " " + to_string(n++);
            }
        }
    }
}

void TConfigParser::LoadDevice(PPortConfig port_config,
                               const Json::Value& device_data,
                               const string& default_id)
{
    if (!device_data.isObject())
        throw TConfigParserException("malformed config");
    if (device_data.isMember("enabled") && !device_data["enabled"].asBool())
        return;

    PTemplate tmpl;
    PDeviceConfig device_config = make_shared<TDeviceConfig>();
    device_config->Id = device_data.isMember("id") ? device_data["id"].asString() : default_id;
    device_config->Name = device_data.isMember("name") ? device_data["name"].asString() : "";

    if (!device_data.isMember("slave_id"))
        throw TConfigParserException("Propery slave_id is missing");
    if (device_data["slave_id"].isString())
        device_config->SlaveId = device_data["slave_id"].asString();
    else if (device_data["slave_id"].isInt()) // legacy
        device_config->SlaveId = to_string(device_data["slave_id"].asInt());
    else
        throw TConfigParserException("Wrong type for property slave_id: should be string or integer, but "
                                        + to_string(device_data["slave_id"].type()) + " given");


    auto device_poll_interval = chrono::milliseconds(-1);
    if (device_data.isMember("poll_interval"))
        device_poll_interval = chrono::milliseconds(
            GetInt(device_data, "poll_interval"));

    if (device_data.isMember("device_type")){
        device_config->DeviceType = device_data["device_type"].asString();
        auto it = Templates->find(device_config->DeviceType);
        if (it != Templates->end()) {
            tmpl = it->second;
            if (tmpl->DeviceData.isMember("name")) {
                if (device_config->Name == "")
                    device_config->Name = tmpl->DeviceData["name"].asString() + " " +
                        device_config->SlaveId;
            } else if (device_config->Name == "")
                    throw TConfigParserException(
                        "Property device_name is missing in " + device_config->DeviceType + " template");

            if (tmpl->DeviceData.isMember("id")) {
                if (device_config->Id == default_id)
                    device_config->Id = tmpl->DeviceData["id"].asString() + "_" +
                        device_config->SlaveId;
            }

            LoadDeviceTemplatableConfigPart(device_config, tmpl->DeviceData);
        } else
            throw TConfigParserException(
                "Can't find the template for '" + device_config->DeviceType + "' device type.");
    }

    LoadDeviceTemplatableConfigPart(device_config, device_data);
    MergeAndLoadChannels(device_config, device_data, tmpl);

    if (device_config->DeviceChannelConfigs.empty())
        throw TConfigParserException("the device has no channels: " + device_config->Name);

    if (device_config->GuardInterval.count() == 0) {
        device_config->GuardInterval = port_config->GuardInterval;
    }

    port_config->AddDeviceConfig(device_config);
    for (auto channel: device_config->DeviceChannelConfigs) {
        for (auto reg: channel->RegisterConfigs) {
            if (reg->PollInterval.count() < 0) {
                if (device_poll_interval.count() >= 0) {
                    reg->PollInterval = device_poll_interval;
                } else {
                    reg->PollInterval = port_config->PollInterval;
                }
            }
        }
    }
}

void TConfigParser::LoadPort(const Json::Value& port_data,
                             const string& id_prefix)
{
    if (!port_data.isObject())
        throw TConfigParserException("malformed config");

    if (port_data.isMember("enabled") && !port_data["enabled"].asBool())
        return;

    auto port_config = make_shared<TPortConfig>();

    auto port_type = port_data.get("port_type", "serial").asString();

    if (port_type == "serial") {
        if (!port_data.isMember("path")) {
            throw TConfigParserException("path is not specified");
        }

        auto serial_port_settings = make_shared<TSerialPortSettings>(port_data["path"].asString());

        if (port_data.isMember("baud_rate"))
            serial_port_settings->BaudRate = GetInt(port_data, "baud_rate");

        if (port_data.isMember("parity"))
            serial_port_settings->Parity = port_data["parity"].asCString()[0]; // FIXME (can be '\0')

        if (port_data.isMember("data_bits"))
            serial_port_settings->DataBits = GetInt(port_data, "data_bits");

        if (port_data.isMember("stop_bits"))
            serial_port_settings->StopBits = GetInt(port_data, "stop_bits");

        port_config->ConnSettings = serial_port_settings;

    } else if (port_type == "tcp") {
        if (!port_data.isMember("address")) {
            throw TConfigParserException("address is not specified");
        }

        if (!port_data.isMember("port")) {
            throw TConfigParserException("port number is not specified");
        }

        auto tcp_port_settings = make_shared<TTcpPortSettings>(port_data["address"].asString(), GetInt(port_data, "port"));

        if (port_data.isMember("connection_timeout_ms")) {
            tcp_port_settings->ConnectionTimeout = chrono::milliseconds(GetInt(port_data, "connection_timeout_ms"));
        }

        if (port_data.isMember("connection_max_fail_cycles")) {
            tcp_port_settings->ConnectionMaxFailCycles = GetInt(port_data, "connection_max_fail_cycles");
        }

        port_config->ConnSettings = tcp_port_settings;

    } else {
        throw TConfigParserException("invalid port_type: '" + port_type + "'");
    }

    if (port_data.isMember("response_timeout_ms"))
        port_config->ConnSettings->ResponseTimeout = chrono::milliseconds(GetInt(port_data, "response_timeout_ms"));

    if (port_data.isMember("poll_interval"))
        port_config->PollInterval = chrono::milliseconds(GetInt(port_data, "poll_interval"));

    if (port_data.isMember("guard_interval_us"))
        port_config->GuardInterval = chrono::microseconds(GetInt(port_data, "guard_interval_us"));

    const Json::Value array = port_data["devices"];
    for(unsigned int index = 0; index < array.size(); ++index)
        LoadDevice(port_config, array[index], id_prefix + to_string(index));

    HandlerConfig->AddPortConfig(port_config);
}

void TConfigParser::LoadConfig()
{
    if (!Root.isObject())
        throw TConfigParserException("malformed config");

    if (!Root.isMember("ports"))
        throw TConfigParserException("no ports specified");

    if (Root.isMember("debug"))
        HandlerConfig->Debug = HandlerConfig->Debug || Root["debug"].asBool();

    if (Root.isMember("max_unchanged_interval"))
        HandlerConfig->MaxUnchangedInterval = Root["max_unchanged_interval"].asInt();

    const Json::Value array = Root["ports"];
    for(unsigned int index = 0; index < array.size(); ++index)
        LoadPort(array[index], "wb-modbus-" + to_string(index) + "-"); // XXX old default prefix for compat

    // check are there any devices defined
    for (const auto& port_config : HandlerConfig->PortConfigs) {
        if (!port_config->DeviceConfigs.empty()) { // found one
            return;
        }
    }

    throw TConfigParserException("no devices defined in config. Nothing to do");
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
