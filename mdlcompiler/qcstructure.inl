// Included beside the loader's tokens so structure uses the same lexer.

std::string JsonString(const std::string& value) { return pulse::api::String(value); }

std::string JsonObject(std::initializer_list<std::pair<std::string, std::string>> fields) {
    std::string out = "{";
    for (const auto& field : fields) {
        if (out.size() > 1) out += ',';
        out += JsonString(field.first) + ":" + field.second;
    }
    return out + "}";
}

void StructureRecord(const char* tag, const std::string& json) {
    pulse::api::Record(tag, json);
}

std::string StructurePath(const std::string& value) {
    std::error_code ec;
    fs::path path = fs::weakly_canonical(pulse::FilePath(value), ec);
    if (ec) path = fs::absolute(pulse::FilePath(value)).lexically_normal();
    return path.u8string();
}

std::string StructureKey(const std::string& value) {
    std::string result = StructurePath(value);
#ifdef _WIN32
    for (char& ch : result) if (ch >= 'A' && ch <= 'Z') ch += 'a' - 'A';
#endif
    return result;
}

bool StructureCommandToken(const Token& t) {
    return !t.quoted && t.text.size() > 1 && t.text[0] == '$' &&
           t.text.find('$', 1) == std::string::npos;
}

bool IsCommandName(const std::string& name);
bool IsCondOp(const std::string& text);
bool CondNumeric(const std::string& text);

struct StructureNode {
    int id = -1, file = -1, parent = -1, call = -1, line = 0;
    size_t start = 0, end = 0, close = 0;
    std::string command, label, state, boundary, preview;
    bool active = false, evaluated = false, generated = false, raw = false, complete = true, movable = false;
};

struct StructureCapture {
    struct File { std::string path, text; };
    struct Ref { int node; std::string kind, name; int definition; bool resolved, evaluated; };
    std::vector<File> files;
    std::vector<StructureNode> nodes;
    std::vector<Ref> refs;
    std::vector<std::string> errors, events;
    std::map<std::string, std::string> overlays;
    std::map<std::pair<std::string, std::string>, int> definitions;
    std::map<std::string, int> variableNodes;
    int current = -1;
    std::string boundary;

    int FileId(const std::string& path, const std::string& text) {
        const std::string key = StructurePath(path);
        for (size_t i = 0; i < files.size(); ++i)
            if (StructureKey(files[i].path) == StructureKey(key)) return static_cast<int>(i);
        files.push_back({key, text});
        return static_cast<int>(files.size() - 1);
    }

    const std::string* Overlay(const std::string& path) const {
        auto it = overlays.find(StructureKey(path));
        return it == overlays.end() ? nullptr : &it->second;
    }

    void Index(const std::string& text, const std::string& path,
               std::vector<Token>& tokens, int parent = -1) {
        const int file = FileId(path, text);
        const size_t first = nodes.size();
        for (size_t i = 0; i < tokens.size(); ++i) {
            const Token& t = tokens[i];
            if (!StructureCommandToken(t) || _stricmp(t.text.c_str(), "$endmacro") == 0)
                continue;
            StructureNode n;
            n.id = static_cast<int>(nodes.size());
            n.file = file;
            n.parent = parent;
            n.start = t.start;
            n.end = t.end;
            n.line = t.line;
            n.command = t.text;
            for (char& ch : n.command) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            n.raw = !IsCommandName(t.text) && _stricmp(t.text.c_str(), "$include") != 0;
            if (i + 1 < tokens.size() && !StructureCommandToken(tokens[i + 1]) &&
                tokens[i + 1].text != "{" && tokens[i + 1].text != "}")
                n.label = tokens[i + 1].text;
            const bool macro = _stricmp(t.text.c_str(), "$definemacro") == 0;
            int depth = macro ? 1 : 0;
            bool braced = false;
            for (size_t j = i + 1; j < tokens.size(); ++j) {
                const Token& u = tokens[j];
                if (macro) {
                    if (!u.quoted && _stricmp(u.text.c_str(), "$definemacro") == 0) ++depth;
                    if (!u.quoted && _stricmp(u.text.c_str(), "$endmacro") == 0 && --depth == 0) {
                        n.end = u.end;
                        break;
                    }
                } else if (!u.quoted) {
                    if (u.text == "{") { ++depth; braced = true; }
                    else if (u.text == "}") {
                        if (depth == 0) break;
                        if (--depth == 0 && braced) {
                            n.close = u.start;
                            n.end = u.end;
                            break;
                        }
                    } else if (depth == 0 && StructureCommandToken(u)) break;
                }
                n.end = u.end;
            }
            if (depth > 0) { n.end = text.size(); n.complete = false; n.raw = true; }
            if (n.command == "$if" || n.command == "$elif" || n.command == "$ifdef" ||
                n.command == "$ifndef" || n.command == "$switch") {
                n.label.clear();
                for (size_t j = i + 1; j < tokens.size(); ++j) {
                    const auto& operand = tokens[j];
                    if (!operand.quoted && operand.text == "{") break;
                    if (!n.label.empty()) n.label += ' ';
                    n.label += operand.quoted ? "\"" + operand.text + "\"" : operand.text;
                }
            }
            nodes.push_back(std::move(n));
        }
        std::vector<int> stack;
        for (size_t i = first; i < nodes.size(); ++i) {
            while (!stack.empty() && nodes[stack.back()].end < nodes[i].end) stack.pop_back();
            if (!stack.empty()) nodes[i].parent = stack.back();
            stack.push_back(static_cast<int>(i));
        }
        stack.clear();
        size_t next = first;
        for (Token& t : tokens) {
            while (!stack.empty() && nodes[stack.back()].end <= t.start) stack.pop_back();
            while (next < nodes.size() && nodes[next].start <= t.start) stack.push_back(static_cast<int>(next++));
            if (!stack.empty() && nodes[stack.back()].end >= t.end) t.node = stack.back();
        }
        for (size_t i = 0; i < tokens.size(); ++i) {
            const Token& t = tokens[i];
            if (t.node < 0 || t.quoted || t.start != nodes[t.node].start) continue;
            const std::string& command = nodes[t.node].command;
            bool inMacro = false;
            for (int p = nodes[t.node].parent; p >= 0; p = nodes[p].parent)
                if (nodes[p].command == "$definemacro") { inMacro = true; break; }
            if (inMacro) continue;
            if (command == "$if" || command == "$elif" || command == "$ifdef" || command == "$ifndef") {
                for (size_t j = i + 1; j < tokens.size() && tokens[j].text != "{"; ++j) {
                    const Token& operand = tokens[j];
                    if (operand.quoted || CondNumeric(operand.text) || IsCondOp(operand.text) ||
                        operand.text == "&&" || operand.text == "||") continue;
                    std::string variable = operand.text;
                    if (variable.size() > 2 && variable.front() == '$' && variable.back() == '$')
                        variable = variable.substr(1, variable.size() - 2);
                    std::string literal;
                    if (j + 2 < tokens.size() && IsCondOp(tokens[j + 1].text) &&
                        (tokens[j + 2].quoted || CondNumeric(tokens[j + 2].text))) literal = tokens[j + 2].text;
                    else if (j >= 2 && IsCondOp(tokens[j - 1].text) &&
                        (tokens[j - 2].quoted || CondNumeric(tokens[j - 2].text))) literal = tokens[j - 2].text;
                    Condition(t.node, variable, literal);
                }
            }
            if (command == "$case" && i + 1 < tokens.size()) {
                int p = nodes[t.node].parent;
                if (p >= 0 && nodes[p].command == "$switch")
                    Condition(p, nodes[p].label, tokens[i + 1].text);
            }
        }
        if (!tokens.empty() && !StructureCommandToken(tokens.front())) {
            StructureNode raw;
            raw.id = static_cast<int>(nodes.size());
            raw.file = file;
            raw.parent = parent;
            raw.command = "raw";
            raw.line = tokens.front().line;
            raw.start = tokens.front().start;
            raw.end = first < nodes.size() ? nodes[first].start : text.size();
            raw.raw = true;
            nodes.push_back(std::move(raw));
        }
    }

    void Error(const std::string& file, int line, const std::string& message) {
        int id = -1, fileId = -1;
        std::string path = StructurePath(file);
        for (size_t i = 0; i < files.size(); ++i)
            if (StructureKey(files[i].path) == StructureKey(path)) {
                fileId = static_cast<int>(i); path = files[i].path; break;
            }
        if (current >= 0 && nodes[current].generated && nodes[current].line <= line) {
            id = current;
            fileId = nodes[current].file;
            path = files[fileId].path;
        } else if (fileId >= 0 && line > 0) {
            size_t offset = 0;
            for (int l = 1; l < line && offset < files[fileId].text.size(); ++l) {
                const auto end = files[fileId].text.find('\n', offset);
                offset = end == std::string::npos ? files[fileId].text.size() : end + 1;
            }
            for (const auto& n : nodes)
                if (n.file == fileId && !n.generated && n.line <= line && n.end >= offset &&
                    (id < 0 || n.end - n.start < nodes[id].end - nodes[id].start)) id = n.id;
        }
        errors.push_back(JsonObject({{"file", JsonString(path)}, {"line", std::to_string(line)},
            {"node", std::to_string(id)}, {"text", JsonString(message)}}));
        if (id >= 0) nodes[id].raw = true;
    }

    void Inactive(const std::vector<Token>& tokens) {
        for (const Token& t : tokens) if (t.node >= 0 && !nodes[t.node].active)
            nodes[t.node].evaluated = true;
    }

    void Expand(std::vector<Token>& body, int call) {
        std::map<int, int> remap;
        for (Token& t : body) {
            if (t.node < 0) continue;
            const int source = t.node;
            auto it = remap.find(source);
            if (it == remap.end()) {
                StructureNode n = nodes[source];
                n.id = static_cast<int>(nodes.size());
                n.call = call;
                n.parent = call;
                n.generated = true;
                n.active = false;
                n.evaluated = false;
                n.state.clear();
                n.preview.clear();
                nodes.push_back(std::move(n));
                it = remap.emplace(source, static_cast<int>(nodes.size() - 1)).first;
            }
            t.node = it->second;
            if (!nodes[t.node].preview.empty()) nodes[t.node].preview += ' ';
            nodes[t.node].preview += t.quoted ? "\"" + t.text + "\"" : t.text;
        }
        for (const auto& mapping : remap) {
            const int p = nodes[mapping.first].parent;
            const auto it = remap.find(p);
            if (it != remap.end()) nodes[mapping.second].parent = it->second;
        }
    }

    void Condition(int node, const std::string& variable, const std::string& value) {
        events.push_back("@cond " + JsonObject({{"node", std::to_string(node)},
            {"variable", JsonString(variable)}, {"value", JsonString(value)}}));
    }

    void Define(const std::string& kind, const std::string& name, int node) {
        definitions[{kind, name}] = node;
    }

    void Reference(int node, const std::string& kind, const std::string& name, bool resolved, bool evaluated = false) {
        auto it = definitions.find({kind, name});
        refs.push_back({node, kind, name, it == definitions.end() ? -1 : it->second, resolved, evaluated});
    }

    void Print(bool success) const {
        StructureRecord("structure", "{\"version\":1}");
        for (size_t i = 0; i < files.size(); ++i) {
            std::string hex;
            hex.reserve(files[i].text.size() * 2);
            const char* digits = "0123456789abcdef";
            for (unsigned char ch : files[i].text) { hex += digits[ch >> 4]; hex += digits[ch & 15]; }
            StructureRecord("file", JsonObject({{"id", std::to_string(i)},
                {"path", JsonString(files[i].path)}, {"bytes", JsonString(hex)}}));
        }
        for (const StructureNode& n : nodes) {
            StructureRecord("node", JsonObject({{"id", std::to_string(n.id)}, {"file", std::to_string(n.file)},
                {"parent", std::to_string(n.parent)}, {"call", std::to_string(n.call)},
                {"start", std::to_string(n.start)}, {"end", std::to_string(n.end)}, {"close", std::to_string(n.close)},
                {"line", std::to_string(n.line)}, {"command", JsonString(n.command)}, {"label", JsonString(n.label)},
                {"active", n.active ? "true" : "false"}, {"evaluated", n.evaluated || success ? "true" : "false"},
                {"generated", n.generated ? "true" : "false"}, {"raw", n.raw ? "true" : "false"},
                {"complete", n.complete ? "true" : "false"}, {"movable", n.movable ? "true" : "false"},
                {"preview", JsonString(n.preview)}}));
            if (!n.state.empty()) StructureRecord("state", JsonObject({{"node", std::to_string(n.id)},
                {"value", n.state}, {"boundary", JsonString(n.boundary)}}));
        }
        for (const auto& definition : definitions)
            StructureRecord("def", JsonObject({{"kind", JsonString(definition.first.first)},
                {"name", JsonString(definition.first.second)}, {"node", std::to_string(definition.second)}}));
        for (const Ref& ref : refs) {
            int definition = ref.definition;
            if (definition < 0)
                for (const auto& entry : definitions)
                    if ((entry.first.first == ref.kind || (ref.kind == "animation-or-sequence" &&
                        (entry.first.first == "animation" || entry.first.first == "sequence"))) &&
                        _stricmp(entry.first.second.c_str(), ref.name.c_str()) == 0) {
                        definition = entry.second;
                        break;
                    }
            StructureRecord("ref", JsonObject({{"node", std::to_string(ref.node)}, {"kind", JsonString(ref.kind)},
                {"name", JsonString(ref.name)}, {"definition", std::to_string(definition)},
                {"evaluated", ref.evaluated || ref.resolved || definition >= 0 ? "true" : "false"},
                {"resolved", ref.resolved || definition >= 0 ? "true" : "false"}}));
        }
        for (const std::string& event : events) pulse::api::Line(event);
        for (const std::string& error : errors) StructureRecord("error", error);
        StructureRecord("end", success ? "{\"status\":\"ok\"}" : "{\"status\":\"error\"}");
    }
};
