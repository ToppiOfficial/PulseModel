#pragma once

std::string MorphKey(const std::string& name)
{
    constexpr char digits[] = "0123456789abcdef";
    std::string key;
    for (unsigned char ch : name) { key += digits[ch >> 4]; key += digits[ch & 15]; }
    return key;
}

std::string MorphName(const std::string& name)
{
    char utf8[96]{};
#ifdef _WIN32
    wchar_t wide[32]{};
    const int count = MultiByteToWideChar(932, MB_ERR_INVALID_CHARS, name.data(), int(name.size()), wide, 32);
    const int size = count ? WideCharToMultiByte(CP_UTF8, 0, wide, count, utf8, sizeof(utf8), nullptr, nullptr) : 0;
    if (size) return std::string(utf8, size);
#else
    const auto decoder = iconv_open("UTF-8", "CP932");
    if (decoder != (iconv_t)-1)
    {
        char* input = const_cast<char*>(name.data());
        char* output = utf8;
        size_t remaining = name.size(), capacity = sizeof(utf8);
        const bool valid = iconv(decoder, &input, &remaining, &output, &capacity) != size_t(-1);
        iconv_close(decoder);
        if (valid && remaining == 0) return std::string(utf8, size_t(output - utf8));
    }
#endif
    return "0x" + MorphKey(name);
}

// Hexadecimal keys keep custom morph names independent of display-name decoding.
FaceMap ReadApiFaceMap(const nlohmann::json& entries)
{
    if (!entries.is_array() || entries.size() > 8192)
        throw std::runtime_error("API member \"faceMap\" must be an array of at most 8192 mappings");

    FaceMap result;
    for (const auto& entry : entries)
    {
        if (!entry.is_object()) throw std::runtime_error("Each face mapping must be an object");
        for (const auto& member : entry.items())
            if (member.key() != "morph" && member.key() != "controller" && member.key() != "strength")
                throw std::runtime_error("Unknown face mapping member: " + member.key());

        const auto key = entry.at("morph").get<std::string>();
        const auto controller = entry.at("controller").get<std::string>();
        const double strength = entry.value("strength", 1.0);
        if (key.empty() || key.size() > 30 || key.size() % 2)
            throw std::runtime_error("A morph key must be 1 to 15 Shift-JIS bytes encoded as hexadecimal");
        if (controller.empty() || controller.size() > 4096 || controller.find('\0') != std::string::npos)
            throw std::runtime_error("Invalid face mapping controller");
        if (!std::isfinite(strength) || std::abs(strength) > 100)
            throw std::runtime_error("Face mapping strength must be between -100 and 100");

        std::string morph;
        for (size_t i = 0; i < key.size(); i += 2)
        {
            const auto digit = [](char ch) {
                if (ch >= '0' && ch <= '9') return ch - '0';
                if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
                if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
                return -1;
            };
            const int hi = digit(key[i]), lo = digit(key[i + 1]);
            if (hi < 0 || lo < 0 || (hi == 0 && lo == 0)) throw std::runtime_error("Invalid hexadecimal morph key");
            morph += char(hi * 16 + lo);
        }

        auto& targets = result[morph];
        if (std::any_of(targets.begin(), targets.end(), [&](const auto& target) {
            return !_stricmp(target.first.c_str(), controller.c_str());
        })) throw std::runtime_error("Duplicate morph and controller mapping");
        targets.emplace_back(controller, float(strength));
    }
    return result;
}

std::vector<mdldecompiler::FlexTrack> ModelControllers(const Mdl& model)
{
    std::vector<mdldecompiler::FlexTrack> controllers;
    const auto& header = *model.hdr;
    const auto* data = model.At<fm::mstudioflexcontroller_t>(model.buf.data(), header.flexcontrollerindex,
                                                           header.numflexcontrollers);
    for (int i = 0; data && i < header.numflexcontrollers; ++i)
        controllers.push_back({model.Str(&data[i], data[i].sznameindex), data[i].min, data[i].max, {}});
    return controllers;
}

void EmitFaceMapping(const nlohmann::json& request)
{
    pulse::api::CheckMembers(request, {"motions", "model"});
    nlohmann::json morphs = nlohmann::json::array(), presets = nlohmann::json::object();
    std::set<std::string> known;
    static const std::map<std::string, std::string> labels{
        {"a", "A"}, {"e", "E"}, {"i", "I"}, {"o", "O"}, {"u", "U"},
        {"blink", "Blink"}, {"wink", "Wink (left)"}, {"winkright", "Wink (right)"},
        {"wink2", "Wink 2 (left)"}, {"wink2right", "Wink 2 (right)"}, {"laugh", "Laugh"},
        {"surprised", "Surprised"}, {"smile", "Smile"}, {"frown", "Frown"},
        {"troubled", "Troubled"}, {"angry", "Angry"}, {"serious", "Serious"},
        {"browup", "Brow up"}, {"browdown", "Brow down"}};
    for (const auto& face : kFace)
    {
        const auto label = labels.find(face.id);
        morphs.push_back({{"morph", MorphKey(face.sj)}, {"name", face.u8}, {"label", label == labels.end() ? face.u8 : label->second}});
        known.insert(face.sj);
    }

    const auto paths = pulse::api::GetStrings(request, "motions");
    if (paths.size() > 32) throw std::runtime_error("At most 32 motion paths may be inspected");
    for (const auto& path : paths)
    {
        std::map<std::string, std::vector<Key>> bones;
        Morphs tracks;
        std::vector<CamKey> cameras;
        uint32_t last = 0;
        std::string error;
        if (!LoadVmd(path, bones, tracks, cameras, last, error)) throw std::runtime_error(error);
        for (const auto& [name, keys] : tracks)
        {
            if (name.empty() || !known.insert(name).second) continue;
            if (known.size() > 4096) throw std::runtime_error("Motion exceeds the 4096 facial morph budget");
            const auto text = MorphName(name);
            morphs.push_back({{"morph", MorphKey(name)}, {"name", text}, {"label", text}});
        }
    }

    for (const auto* name : {"facs", "visemes", "direct", "direct-scaled"})
    {
        auto entries = nlohmann::json::array();
        for (const auto& [morph, targets] : BuiltinFaceMap(name))
            for (const auto& [controller, strength] : targets)
                entries.push_back({{"morph", MorphKey(morph)}, {"controller", controller}, {"strength", strength}});
        presets[name] = std::move(entries);
    }

    auto controllers = nlohmann::json::array();
    const auto path = pulse::api::GetString(request, "model");
    if (!path.empty())
    {
        Mdl model;
        std::string error;
        if (!LoadMdl(path, model, error)) throw std::runtime_error(error);
        for (const auto& controller : ModelControllers(model)) controllers.push_back(controller.name);
    }
    pulse::api::Record("faceMapping", nlohmann::json{{"morphs", morphs}, {"presets", presets},
                                                   {"controllers", controllers}}.dump());
}
