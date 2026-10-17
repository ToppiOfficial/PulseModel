std::string StructureNumber(float value) {
    if (!std::isfinite(value)) return JsonString(std::isnan(value) ? "nan" : value < 0 ? "-inf" : "inf");
    std::ostringstream stream;
    stream.precision(std::numeric_limits<float>::max_digits10);
    stream << value;
    return stream.str();
}

std::string StructureState(const Ctx& c) {
    auto paths = [](const std::vector<fs::path>& list) {
        std::string out = "[";
        for (const fs::path& p : list) {
            if (out.size() > 1) out += ',';
            out += JsonString(p.u8string());
        }
        return out + "]";
    };
    auto strings = [](const std::vector<std::string>& list) {
        std::string out = "[";
        for (const std::string& s : list) {
            if (out.size() > 1) out += ',';
            out += JsonString(s);
        }
        return out + "]";
    };
    auto vector = [](const pm::Vector3& v) {
        return "[" + StructureNumber(v.x) + "," + StructureNumber(v.y) + "," + StructureNumber(v.z) + "]";
    };
    std::string weights = "[";
    for (const auto& entry : c.in.defaultWeights) {
        if (weights.size() > 1) weights += ',';
        weights += JsonObject({{"bone", JsonString(entry.bone)}, {"weight", StructureNumber(entry.weight)},
            {"positionWeight", StructureNumber(entry.posweight)}});
    }
    weights += "]";
    std::string variables = "{";
    for (const auto& v : c.variables) {
        if (variables.size() > 1) variables += ',';
        variables += JsonString(v.first) + ":" + JsonString(v.second);
    }
    variables += "}";
    return JsonObject({{"scale", StructureNumber(c.in.scale)}, {"origin", vector(c.in.adjust)},
        {"rotation", vector(c.in.rotation)}, {"rotationSet", c.in.rotationSet ? "true" : "false"},
        {"sourceUpAxisY", source::DmxUpAxisY() ? "true" : "false"}, {"defaultWeights", weights},
        {"cd", JsonString(c.curDir.u8string())}, {"sourceDirs", paths(c.sourceDirStack)},
        {"searchDirs", paths(c.searchDirs)}, {"includeDirs", paths(c.includeDirs)},
        {"launchSearchDirs", paths(c.launchSearchDirs)}, {"launchIncludeDirs", paths(c.launchIncludeDirs)},
        {"cdmaterials", strings(c.in.cdmaterials)}, {"fps", StructureNumber(c.defaultFps)},
        {"motionRollback", StructureNumber(c.defaultMotionRollback)}, {"fadeIn", StructureNumber(c.defaultFadeIn)},
        {"fadeOut", StructureNumber(c.defaultFadeOut)}, {"variables", variables},
        {"lowercaseSequences", c.lcaseSequences ? "true" : "false"}, {"activities", strings(c.allowedActivities)},
        {"unlockBones", c.unlockDefineBones ? "true" : "false"}, {"hitboxSet", std::to_string(c.activeHitboxSet)}});
}

void StructureTouch(Ctx& c, const Token& token) {
    if (!g_structure || token.node < 0 || c.rawCollect) return;
    StructureNode& node = g_structure->nodes[token.node];
    g_structure->current = token.node;
    if (!node.active) {
        node.active = node.evaluated = true;
        if (node.complete) node.raw = false;
        node.state = StructureState(c);
        node.boundary = g_structure->boundary;
    }
}

void StructureRead(Ctx& c) {
    if (!c.rawCollect) {
        for (size_t i = c.observedPos; i < c.pos && i < c.toks.size(); ++i)
            if (StructureCommandToken(c.toks[i])) StructureTouch(c, c.toks[i]);
    }
    c.observedPos = c.pos;
}

struct StructureCommandScope {
    Ctx& c;
    int node;
    std::string before;
    size_t anims, sequences, bodyparts, poses, lods, sources, bones, skins;
    float scale;
    StructureCommandScope(Ctx& context, const Token& token) : c(context), node(token.node),
        anims(c.in.anims.size()), sequences(c.in.sequences.size()), bodyparts(c.in.bodyparts.size()),
        poses(c.in.poseparams.size()), lods(c.in.scriptLods.size()), sources(c.in.sources.size()),
        bones(c.in.importbones.size()), skins(c.in.skinFamilies.size()), scale(c.in.scale) {
        if (g_structure) {
            StructureTouch(c, token);
            if (node >= 0) {
                auto& n = g_structure->nodes[node];
                n.movable = true;
                if (c.pos + 1 < c.toks.size() && !StructureCommandToken(c.toks[c.pos + 1]) &&
                    c.toks[c.pos + 1].text != "{" && c.toks[c.pos + 1].text != "}" &&
                    n.command != "$definemacro")
                    n.label = c.toks[c.pos + 1].text;
            }
            before = StructureState(c);
        }
    }
    ~StructureCommandScope() {
        if (!g_structure || node < 0) return;
        StructureRead(c);
        StructureNode& n = g_structure->nodes[node];
        if (before != StructureState(c))
            g_structure->boundary = n.command + (n.command == "$transformmodel" && scale != c.in.scale ? " scale" : "") +
                " at line " + std::to_string(n.line) +
                " (" + g_structure->files[n.file].path + ")";
        auto index = [&](const char* kind, size_t first, size_t count, auto& values) {
            for (size_t i = first; i < count; ++i) {
                g_structure->Define(kind, values[i].name, node);
                g_structure->events.push_back("@index " + JsonObject({{"node", std::to_string(node)},
                    {"kind", JsonString(kind)}, {"name", JsonString(values[i].name)}, {"index", std::to_string(i)}}));
            }
        };
        index("animation", anims, c.in.anims.size(), c.in.anims);
        index("sequence", sequences, c.in.sequences.size(), c.in.sequences);
        index("modelgroup", bodyparts, c.in.bodyparts.size(), c.in.bodyparts);
        index("poseparameter", poses, c.in.poseparams.size(), c.in.poseparams);
        for (size_t i = sources; i < c.in.sources.size(); ++i)
            for (const auto& bone : c.in.sources[i]->localBone)
                g_structure->definitions.emplace(std::make_pair("bone", bone.name), node);
        for (size_t i = bones; i < c.in.importbones.size(); ++i)
            g_structure->Define("bone", c.in.importbones[i].name, node);
        for (size_t i = lods; i < c.in.scriptLods.size(); ++i)
            g_structure->events.push_back("@index " + JsonObject({{"node", std::to_string(node)},
                {"kind", JsonString("lod")}, {"name", JsonString(n.label)}, {"index", std::to_string(i)}}));
        for (size_t i = skins; i < c.in.skinFamilies.size(); ++i)
            g_structure->events.push_back("@index " + JsonObject({{"node", std::to_string(node)},
                {"kind", JsonString("skin")}, {"name", JsonString(n.label + " family " + std::to_string(i + 1))},
                {"index", std::to_string(i + 1)}}));
        if (n.command == "$definemacro") g_structure->Define("macro", n.label, node);
        if (n.command == "$definevariable" || n.command == "$redefinevariable")
            g_structure->variableNodes[n.label] = node;
        g_structure->current = node;
    }
};
