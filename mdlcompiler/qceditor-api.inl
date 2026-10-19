struct EditorNestedCommand {
    const char* name;
    const char* syntax;
    const char* starter;
};

const EditorNestedCommand editorMeshCommands[] = {
    {"$toonoutline", "$toonoutline <stream> <thickness> [<min> <max>] [noweld] [permaterial] [forceuseoutline] [thicknessclamp <factor>] [materialname <name>] [cdmaterial <dir>]", "$toonoutline \"\" 0.5"},
    {"$cullvertex", "$cullvertex <stream> <threshold>", "$cullvertex \"\" 0.5"},
    {"$exceptionlist", "$exceptionlist [exclusive|inclusive] { <names> }", "$exceptionlist {\n    \"\"\n}"},
    {"$skinnedbonecull", "$skinnedbonecull aggressive|tree", "$skinnedbonecull tree"},
    {"$removemesh", "$removemesh <material>", "$removemesh \"\""},
    {"$removemeshword", "$removemeshword <keyword>", "$removemeshword \"\""},
    {"$inflate", "$inflate <amount> [material <names> | mesh <names>]", "$inflate 0"},
    {"$use_skeletalaware_decimation", "$use_skeletalaware_decimation", "$use_skeletalaware_decimation"},
    {"$flipnormals", "$flipnormals", "$flipnormals"},
    {"$weld", "$weld [seams] [sharp <degrees>]", "$weld"},
    {"$decimate", "$decimate <factor>", "$decimate 1"},
    {"$decimatemesh", "$decimatemesh <factor> <meshes>", "$decimatemesh 1 \"\""},
    {"$scalebone", "$scalebone <bone> <factor> [x] [y] [z]", "$scalebone \"\" 1"},
    {"$nomorph", "$nomorph", "$nomorph"},
    {"$nofacial", "$nofacial", "$nofacial"},
    {"$vta", "$vta <file> { flex <name> frame <index> [position <value>] [decay <value>] }", "$vta \"\" {\n    flex \"\" frame 0\n}"},
    {"$vca", "$vca <file> [<controller>]", "$vca \"\""}
};

const EditorNestedCommand* EditorNested(const std::string& command) {
    for (const auto& entry : editorMeshCommands) if (command == entry.name) return &entry;
    return nullptr;
}

bool EditorAcceptsNested(const std::string& parent, const std::string& command) {
    return parent == "$rendermesh" ||
        (parent == "$rendermeshcmdlist" && command != "$exceptionlist");
}

// Authored parameters use the loader's tokens, before variable expansion.
std::string EditorLabels(const std::string& command) {
    static const std::map<std::string, std::string> labels = {
        {"$modelname", "Path"}, {"$scale", "Scale"}, {"$origin", "X|Y|Z|Rotation"},
        {"$upaxis", "Axis"}, {"$animation", "Name|Source"}, {"$sequence", "Name|Animation or source"},
        {"$rendermesh", "Name|Source"}, {"$body", "Name|Source"}, {"$model", "Name|Source"},
        {"$modelgroup", "Name"}, {"$bodygroup", "Name"}, {"$poseparameter", "Name|Minimum|Maximum"},
        {"$attachment", "Name"}, {"$definevariable", "Name|Value"},
        {"$redefinevariable", "Name|Value"}, {"$definemacro", "Name"},
        {"$include", "Path"}, {"$includemodel", "Path"}, {"$cdmaterials", "Path"},
        {"$pushd", "Directory"}, {"$addsearchdir", "Directory"},
        {"$addincludesearchdir", "Directory"}, {"$surfaceprop", "Surface"},
        {"$renamebone", "From|To"}, {"$renamematerial", "From|To"},
        {"$jigglebone", "Bone"}, {"$if", "Condition"}, {"$elif", "Condition"},
        {"$toonoutline", "Stream|Thickness|Minimum weight|Maximum weight"},
        {"$cullvertex", "Stream|Threshold"}, {"$skinnedbonecull", "Mode"},
        {"$removemesh", "Material"}, {"$removemeshword", "Keyword"},
        {"$inflate", "Amount"}, {"$decimate", "Factor"}, {"$decimatemesh", "Factor|Mesh"},
        {"$scalebone", "Bone|Factor"}, {"$vta", "Path"}, {"$vca", "Path|Controller"},
        {"$ifdef", "Variable"}, {"$ifndef", "Variable"}, {"$switch", "Variable"}, {"$case", "Value"}
    };
    const auto found = labels.find(command);
    return found == labels.end() ? std::string() : found->second;
}

std::string EditorParameterSchema(const std::string& command) {
    std::string result = "[";
    std::istringstream names(EditorLabels(command));
    std::string name;
    size_t index = 0;
    while (std::getline(names, name, '|')) {
        if (result.size() > 1) result += ',';
        result += JsonObject({{"index", std::to_string(index++)}, {"label", JsonString(name)},
            {"type", JsonString("authored-value")}});
    }
    return result + ']';
}

std::string EditorParameters(const std::string& command, const std::vector<Token>& tokens,
                             size_t start, size_t end, const std::map<size_t, EditorField>& fields,
                             const std::map<size_t, std::pair<size_t, size_t>>& commands) {
    std::vector<std::string> labels;
    std::istringstream names(EditorLabels(command));
    std::string name;
    while (std::getline(names, name, '|')) labels.push_back(name);
    std::string result = "[";
    auto first = std::lower_bound(tokens.begin(), tokens.end(), start,
        [](const Token& token, size_t position) { return token.start < position; });
    if (first == tokens.end() || first->start != start) return "[]";
    size_t index = 0;
    int depth = 0;
    std::vector<size_t> commandEnds;
    const auto endCommand = [&](size_t position) {
        if (result.size() > 1) result += ',';
        result += JsonObject({{"kind", JsonString("end-command")}, {"start", std::to_string(position)},
            {"end", std::to_string(position)}, {"value", JsonString("")}, {"quoted", "false"}});
    };
    for (auto it = first + 1; it != tokens.end() && it->end <= end; ++it) {
        const Token& token = *it;
        while (!commandEnds.empty() && token.start >= commandEnds.back()) {
            endCommand(commandEnds.back()); commandEnds.pop_back();
        }
        if (!token.quoted && _stricmp(token.text.c_str(), "$endmacro") == 0) continue;
        if (result.size() > 1) result += ',';
        const bool open = !token.quoted && token.text == "{";
        const bool close = !token.quoted && token.text == "}";
        if (close) --depth;
        auto label = depth == 0 && index < labels.size() && !open && !close ? labels[index] :
            open ? "Options" : close ? "End options" : "Argument " + std::to_string(index + 1);
        const auto kind = open ? "group" : close ? "end-group" : StructureCommandToken(token) ? "command" : token.quoted ? "string" :
            token.text.find('$') != std::string::npos ? "expression" : "word";
        const auto field = fields.find(token.start);
        if (field != fields.end()) label = field->second.label;
        auto type = field != fields.end() ? field->second.type : std::string("value");
        if (label == "Path" || label == "Source" || label.find("filename") != std::string::npos) type = "path";
        std::string options = "[]";
        if (command == "$upaxis" && index == 0) options = "[\"x\",\"-x\",\"y\",\"-y\",\"z\",\"-z\"]";
        size_t fieldEnd = token.end, contentEnd = token.end;
        if (kind == std::string("command")) {
            if (const auto nested = commands.find(token.start); nested != commands.end() && nested->second.first <= end) {
                fieldEnd = nested->second.first;
                contentEnd = nested->second.second ? nested->second.second : fieldEnd;
                commandEnds.push_back(fieldEnd);
                label = token.text;
            }
        }
        if (open) {
            int nested = 1;
            for (auto next = it + 1; next != tokens.end() && next->end <= end; ++next) {
                if (!next->quoted && next->text == "{") ++nested;
                if (!next->quoted && next->text == "}" && --nested == 0) {
                    fieldEnd = next->end; contentEnd = next->start; break;
                }
            }
        }
        result += JsonObject({{"start", std::to_string(token.start)}, {"end", std::to_string(fieldEnd)},
            {"contentEnd", std::to_string(contentEnd)}, {"type", JsonString(type)}, {"choices", options},
            {"evaluated", field != fields.end() ? JsonString(field->second.evaluated) : "null"},
            {"referenceKind", field != fields.end() ? JsonString(field->second.reference) : JsonString("")},
            {"line", std::to_string(token.line)}, {"depth", std::to_string(std::max(0, depth))},
            {"label", JsonString(label)}, {"kind", JsonString(kind)},
            {"value", JsonString(token.text)}, {"quoted", token.quoted ? "true" : "false"}});
        if (open) ++depth;
        if (!open && !close) ++index;
    }
    while (!commandEnds.empty()) { endCommand(commandEnds.back()); commandEnds.pop_back(); }
    return result + ']';
}

void EditorRead(const Token& token, const char* label, const char* type,
                const char* reference, const std::string& evaluated) {
    if (!g_structure || token.node < 0 || token.node >= static_cast<int>(g_structure->nodes.size())) return;
    const auto& node = g_structure->nodes[token.node];
    if (node.generated) return;
    g_structure->parameterFields[node.file][token.start] = {label, type, reference, evaluated};
}

void EditorMissing(const Token& token, const char* label, const char* type, const char* reference) {
    if (!g_structure) return;
    const auto id = token.node >= 0 ? token.node : g_structure->current;
    if (id < 0 || id >= static_cast<int>(g_structure->nodes.size()) || g_structure->nodes[id].generated) return;
    g_structure->missingParameters[id] = {label, type, reference, {}};
}

std::string EditorHex(const std::string& bytes) {
    static constexpr char digits[] = "0123456789abcdef";
    std::string result;
    result.reserve(bytes.size() * 2);
    for (unsigned char ch : bytes) { result += digits[ch >> 4]; result += digits[ch & 15]; }
    return result;
}

struct EditorRequest {
    std::string revision;
    std::string path, operation, value, kind, before;
    size_t start = 0, end = 0;
    size_t selected = std::string::npos, branch = std::string::npos;
};

struct EditorPatch {
    size_t start;
    std::string before, after;
    struct Origin { size_t start, end, offset; };
    std::vector<Origin> origins;
};

std::string EditorSlotKind(const StructureCapture& capture, const StructureNode& node) {
    for (auto it = capture.definitions.lower_bound({"modelgroup", ""});
         it != capture.definitions.end() && it->first.first == "modelgroup"; ++it)
        if (it->second == node.id) return "modelgroup";
    return node.command;
}

bool EditorCanMove(const StructureCapture& capture, const StructureNode& from, const StructureNode& to) {
    if (from.generated || to.generated || !from.complete || !to.complete || !from.movable || !to.movable ||
        !from.active || !to.active || from.file != to.file || from.parent != to.parent ||
        from.state.empty() || from.state != to.state || from.boundary != to.boundary ||
        EditorSlotKind(capture, from) != EditorSlotKind(capture, to)) return false;
    const auto low = std::min(from.start, to.start), high = std::max(from.start, to.start);
    for (const auto& node : capture.nodes) {
        if (node.file != from.file || node.generated) continue;
        if (node.start > low && node.start < high && (node.command == "$include" || node.command == "$if" ||
            node.command == "$ifdef" || node.command == "$ifndef" || node.command == "$elif" ||
            node.command == "$else" || node.command == "$switch" || node.command == "$case" || node.command == "$default")) return false;
        if (node.id != from.id && ((node.start == from.start && node.end == from.end) ||
            (node.start == to.start && node.end == to.end)) && (node.state != from.state || node.parent != from.parent)) return false;
    }
    return true;
}

std::vector<std::string> EditorStateSignature(const StructureCapture& capture) {
    std::set<int> containers;
    for (const auto& node : capture.nodes) if (node.parent >= 0) containers.insert(node.parent);
    std::vector<std::string> result;
    for (const auto& node : capture.nodes) {
        if (containers.count(node.id) || !node.active || node.state.empty()) continue;
        const auto& file = capture.files.at(node.file);
        result.push_back(JsonObject({{"file", JsonString(file.path)}, {"command", JsonString(node.command)},
            {"label", JsonString(node.label)}, {"source", JsonString(node.generated ? node.preview : file.text.substr(node.start, node.end - node.start))},
            {"state", node.state}}));
    }
    std::sort(result.begin(), result.end());
    return result;
}

std::vector<EditorPatch> EditorReorder(const EditorRequest& request, const StructureCapture& capture,
                                      std::string& source) {
    int fileId = -1;
    const auto pathKey = StructureKey(request.path);
    for (size_t i = 0; i < capture.files.size(); ++i)
        if (StructureKey(capture.files[i].path) == pathKey) { fileId = static_cast<int>(i); break; }
    const auto found = std::find_if(capture.nodes.begin(), capture.nodes.end(), [&](const StructureNode& node) {
        return !node.generated && node.file == fileId &&
            node.start == request.start && node.end == request.end;
    });
    if (found == capture.nodes.end()) throw std::runtime_error("The command to move is no longer present");
    const auto& from = *found;
    std::vector<const StructureNode*> slots;
    std::set<size_t> seen;
    for (const auto& node : capture.nodes)
        if (!node.generated && node.file == from.file && node.parent == from.parent &&
            EditorSlotKind(capture, node) == EditorSlotKind(capture, from) && seen.insert(node.start).second)
            slots.push_back(&node);
    std::sort(slots.begin(), slots.end(), [](const auto* a, const auto* b) { return a->start < b->start; });
    std::vector<EditorPatch> patches;
    const auto text = [&](const StructureNode& node) { return source.substr(node.start, node.end - node.start); };
    if (request.operation == "gather") {
        const StructureNode* first = nullptr;
        for (const auto* slot : slots) if (EditorCanMove(capture, from, *slot)) { first = slot; break; }
        if (!first) throw std::runtime_error("The compiler cannot verify this command's movement context");
        const auto newline = source.find("\r\n") != std::string::npos ? std::string("\r\n") : std::string("\n");
        const auto line = first->start ? source.rfind('\n', first->start - 1) : std::string::npos;
        std::string indent = source.substr(line == std::string::npos ? 0 : line + 1,
            first->start - (line == std::string::npos ? 0 : line + 1));
        if (indent.find_first_not_of(" \t") != std::string::npos) indent.clear();
        auto grouped = text(*first);
        std::vector<EditorPatch::Origin> origins{{first->start, first->end, 0}};
        for (const auto* slot : slots) {
            if (slot == first || !EditorCanMove(capture, *first, *slot)) continue;
            grouped += newline + indent;
            origins.push_back({slot->start, slot->end, grouped.size()});
            grouped += text(*slot);
            patches.push_back({slot->start, text(*slot), {}});
        }
        if (!patches.empty()) patches.push_back({first->start, text(*first), std::move(grouped), std::move(origins)});
    } else {
        if (request.value.empty() || request.value.find_first_not_of("0123456789") != std::string::npos)
            throw std::runtime_error("Invalid command reorder target");
        const auto target = std::stoull(request.value);
        const auto to = std::find_if(slots.begin(), slots.end(), [&](const auto* node) { return node->start == target; });
        const auto sourceSlot = std::find_if(slots.begin(), slots.end(), [&](const auto* node) { return node->start == from.start; });
        if (to == slots.end() || sourceSlot == slots.end()) throw std::runtime_error("The command reorder target is no longer present");
        auto destination = to - slots.begin();
        const auto origin = sourceSlot - slots.begin();
        if (request.operation == "reorder-after") ++destination;
        if (origin < destination) --destination;
        const auto low = std::min(origin, destination), high = std::max(origin, destination);
        std::vector<const StructureNode*> values;
        for (auto i = low; i <= high; ++i) {
            if (!EditorCanMove(capture, from, *slots[i]))
                throw std::runtime_error("Move blocked by a file, command, branch, include, or compiler-state boundary");
            values.push_back(slots[i]);
        }
        if (origin < destination) std::rotate(values.begin(), values.begin() + 1, values.end());
        else if (origin > destination) std::rotate(values.rbegin(), values.rbegin() + 1, values.rend());
        for (auto i = low; i <= high; ++i) {
            const auto* origin = values[i - low];
            patches.push_back({slots[i]->start, text(*slots[i]), text(*origin), {{origin->start, origin->end, 0}}});
        }
    }
    if (patches.empty()) patches.push_back({request.start, request.before, request.before});
    std::sort(patches.begin(), patches.end(), [](const auto& a, const auto& b) { return a.start > b.start; });
    for (const auto& patch : patches) source.replace(patch.start, patch.before.size(), patch.after);
    return patches;
}

EditorRequest ReadEditorRequest(std::string& buffers) {
    size_t cursor = 12;
    const auto line = [&]() {
        const auto end = buffers.find('\n', cursor);
        if (end == std::string::npos) throw std::runtime_error("Truncated editor request");
        const auto result = buffers.substr(cursor, end - cursor);
        cursor = end + 1;
        return result;
    };
    const auto number = [&](const std::string& value) {
        if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
            throw std::runtime_error("Invalid editor offset");
        const auto result = std::stoull(value);
        if (result > 64 * 1024 * 1024) throw std::runtime_error("Editor offset exceeds limit");
        return static_cast<size_t>(result);
    };
    const auto field = [&]() {
        const auto length = number(line());
        if (length > buffers.size() - cursor) throw std::runtime_error("Truncated editor field");
        const auto result = buffers.substr(cursor, length);
        cursor += length;
        return result;
    };
    EditorRequest request;
    request.revision = field(); request.path = field(); request.operation = field();
    request.start = number(field()); request.end = number(field());
    request.before = field(); request.value = field(); request.kind = field();
    const auto selected = field(), branch = field();
    if (!selected.empty()) request.selected = number(selected);
    if (!branch.empty()) request.branch = number(branch);
    if (request.revision.empty() || request.revision.find_first_not_of("0123456789") != std::string::npos)
        throw std::runtime_error("Invalid editor revision");
    buffers.erase(0, cursor);
    return request;
}

std::string EditorValue(const EditorRequest& request) {
    if (request.value.find('\0') != std::string::npos || request.value.find('"') != std::string::npos)
        throw std::runtime_error("A QC parameter cannot contain NUL or a quotation mark");
    const bool quoted = request.kind == "string";
    if (quoted) return '"' + request.value + '"';
    if (request.kind != "word" && request.kind != "expression")
        throw std::runtime_error("Unsupported parameter kind");
    std::vector<Token> tokens;
    std::string error;
    if (!Tokenize(request.value, request.path, tokens, &error) || tokens.size() != 1 ||
        tokens[0].start != 0 || tokens[0].end != request.value.size() || tokens[0].quoted ||
        tokens[0].text == "{" || tokens[0].text == "}")
        throw std::runtime_error("Use one unquoted value, or enable Quoted for a string");
    return request.value;
}

bool EditorBranch(const std::string& command) {
    return command == "$if" || command == "$ifdef" || command == "$ifndef" || command == "$elif" ||
        command == "$else" || command == "$switch" || command == "$case" || command == "$default";
}

const StructureNode* EditorNestedParent(const EditorRequest& request, const StructureCapture& syntax) {
    if (!EditorNested(request.value)) return nullptr;
    const StructureNode* target = nullptr;
    const bool gap = request.kind == "gap-before" || request.kind == "gap-after";
    for (const auto& node : syntax.nodes) {
        if ((gap && node.start == request.start) || (!gap && node.start == request.selected)) {
            target = &node;
            break;
        }
    }
    if (!gap && request.kind != "automatic")
        for (const auto& node : syntax.nodes)
            if (node.start < request.start && request.start <= node.end &&
                (!target || node.start > target->start)) target = &node;
    for (; target; target = target->parent >= 0 ? &syntax.nodes.at(target->parent) : nullptr)
        if (EditorAcceptsNested(target->command, request.value) && target->complete) return target;
    throw std::runtime_error("Select a render mesh or a compatible render mesh command list to add " + request.value);
}

size_t EditorInsertion(const EditorRequest& request, const StructureCapture& syntax, const std::string& source) {
    const auto* nestedParent = EditorNestedParent(request, syntax);
    const auto nodeAt = [&](size_t start) -> const StructureNode* {
        for (const auto& node : syntax.nodes) if (node.start == start) return &node;
        return nullptr;
    };
    const auto* selected = nodeAt(request.selected);
    const auto* filtered = nodeAt(request.branch);
    const auto lineStart = [&](size_t offset) {
        const auto newline = offset ? source.rfind('\n', offset - 1) : std::string::npos;
        const auto start = newline == std::string::npos ? 0 : newline + 1;
        return source.substr(start, offset - start).find_first_not_of(" \t") == std::string::npos ? start : offset;
    };
    if (request.kind == "gap-before" || request.kind == "gap-after") {
        const auto* target = nodeAt(request.start);
        if (!target || !target->complete) throw std::runtime_error("Select a complete command insertion target");
        if (nestedParent && target->id == nestedParent->id)
            throw std::runtime_error("Drop this option inside a render mesh");
        return request.kind == "gap-before" ? target->start : target->end;
    }
    if (nestedParent) return nestedParent->close ? nestedParent->close : nestedParent->end;
    if (request.kind != "automatic") return request.start;
    const auto& command = request.value;
    const bool arm = command == "$elif" || command == "$else" || command == "$case" || command == "$default";
    if (EditorBranch(command)) {
        if (!arm) {
            if (selected && selected->complete && selected->close) return lineStart(selected->close);
            if (filtered && filtered->complete && filtered->close) return lineStart(filtered->close);
            return source.size();
        }
        if (!filtered || !filtered->complete) throw std::runtime_error("Select the branch to extend");
        if (command == "$case" || command == "$default") {
            const auto* sw = filtered->command == "$switch" ? filtered :
                filtered->parent >= 0 ? &syntax.nodes.at(filtered->parent) : nullptr;
            if (!sw || sw->command != "$switch" || !sw->close)
                throw std::runtime_error("Select a switch branch to extend");
            for (const auto& child : syntax.nodes) if (child.parent == sw->id && child.command == "$default") {
                if (command == "$default") throw std::runtime_error("This switch already has a default arm");
                return lineStart(child.start);
            }
            return lineStart(sw->close);
        }
        if (filtered->command != "$if" && filtered->command != "$ifdef" && filtered->command != "$ifndef" &&
            filtered->command != "$elif" && filtered->command != "$else")
            throw std::runtime_error("Select an if branch to extend");
        std::vector<const StructureNode*> siblings;
        for (const auto& node : syntax.nodes) if (node.parent == filtered->parent) siblings.push_back(&node);
        std::sort(siblings.begin(), siblings.end(), [](const auto* a, const auto* b) { return a->start < b->start; });
        auto at = std::find(siblings.begin(), siblings.end(), filtered);
        if (at == siblings.end()) throw std::runtime_error("The selected branch is no longer present");
        while (at != siblings.begin() && ((*at)->command == "$elif" || (*at)->command == "$else")) --at;
        const auto* last = *at;
        for (++at; at != siblings.end() && ((*at)->command == "$elif" || (*at)->command == "$else"); ++at) {
            if ((*at)->command == "$else") {
                if (command == "$else") throw std::runtime_error("This branch already has an else arm");
                return lineStart((*at)->start);
            }
            last = *at;
        }
        return last->end;
    }
    const auto branchOf = [&](const StructureNode& node) {
        for (const auto* parent = &node; parent; parent = parent->parent >= 0 ? &syntax.nodes.at(parent->parent) : nullptr)
            if (EditorBranch(parent->command)) return parent->id;
        return -1;
    };
    const int branch = filtered ? filtered->id : selected ? branchOf(*selected) : -1;
    const StructureNode* last = nullptr;
    for (const auto& node : syntax.nodes)
        if (node.command == command && branchOf(node) == branch && (!last || node.start > last->start)) last = &node;
    if (last) return last->end;
    if (branch >= 0) {
        const auto& node = syntax.nodes.at(branch);
        if (!node.complete || !node.close) throw std::runtime_error("The branch has no complete insertion point");
        return node.close;
    }
    if (selected && selected->parent < 0 && selected->complete) return selected->end;
    static const std::vector<std::string> sections = {"output", "materials", "rendering", "modelgroups", "skins",
        "skeleton", "attachments", "hitboxes", "animation", "poseparams", "ik", "physics", "flex", "includes", "macros", "branches", "misc"};
    auto previous = std::find(sections.begin(), sections.end(), EditorCommandSection(command));
    while (previous != sections.begin() && previous != sections.end()) {
        --previous;
        for (const auto& node : syntax.nodes)
            if (node.parent < 0 && EditorCommandSection(node.command) == *previous && (!last || node.start > last->start)) last = &node;
        if (last) return last->end;
    }
    return source.size();
}

std::vector<EditorPatch> ApplyEditorRequest(const EditorRequest& request, std::string& source,
                                           const StructureCapture* baseline) {
    if (request.start > request.end || request.end > source.size() ||
        source.substr(request.start, request.end - request.start) != request.before)
        throw std::runtime_error("The QC document changed; refresh before editing this parameter");
    if (request.operation == "reorder-before" || request.operation == "reorder-after" || request.operation == "gather") {
        if (!baseline) throw std::runtime_error("Command movement requires compiler analysis");
        return EditorReorder(request, *baseline, source);
    }
    std::vector<Token> tokens;
    std::string error;
    if (!Tokenize(source, request.path, tokens, &error)) throw std::runtime_error(error);
    const auto token = std::lower_bound(tokens.begin(), tokens.end(), request.start,
        [](const Token& value, size_t position) { return value.start < position; });
    std::string after;
    auto patchStart = request.start;
    const auto newline = source.find("\r\n") != std::string::npos ? std::string("\r\n") : std::string("\n");
    if (request.operation == "erase-command" || request.operation == "duplicate-command") {
        StructureCapture syntax;
        syntax.Index(source, request.path, tokens);
        const auto node = std::find_if(syntax.nodes.begin(), syntax.nodes.end(), [&](const StructureNode& node) {
            return node.start == request.start && node.end == request.end && node.complete;
        });
        if (node == syntax.nodes.end()) throw std::runtime_error("Select a complete command to edit");
        if (request.operation == "duplicate-command") after = request.before + newline + request.before;
    } else if (request.operation == "insert-command") {
        if (request.start != request.end) throw std::runtime_error("Invalid command insertion range");
        StructureCapture syntax;
        syntax.Index(source, request.path, tokens);
        patchStart = EditorInsertion(request, syntax, source);
        const auto* nestedParent = EditorNestedParent(request, syntax);
        after = EditorCommandStarter(request.value);
        if (after.empty()) throw std::runtime_error("Unsupported command template");
        std::string formatted;
        for (const char ch : after) formatted += ch == '\n' ? newline : std::string(1, ch);
        after = newline + formatted + newline;
        if (nestedParent && !nestedParent->close) after = newline + "{" + after + "}" + newline;
    } else if (request.operation == "erase" && token != tokens.end() && !token->quoted && token->text == "{") {
        int depth = 0;
        auto close = token;
        for (; close != tokens.end(); ++close) {
            if (close->quoted) continue;
            if (close->text == "{") ++depth;
            else if (close->text == "}" && --depth == 0) break;
        }
        if (close == tokens.end() || close->end != request.end)
            throw std::runtime_error("Select a complete options group to remove");
    } else if (request.operation == "set" || request.operation == "erase") {
        if (token == tokens.end() || token->start != request.start || token->end != request.end ||
            (!token->quoted && (token->text == "{" || token->text == "}" ||
                StructureCommandToken(*token))))
            throw std::runtime_error("Select an editable parameter value");
        after = request.operation == "erase" ? std::string() : EditorValue(request);
    } else if (request.operation == "insert") {
        if (request.start != request.end) throw std::runtime_error("Invalid parameter insertion range");
        for (const auto& value : tokens)
            if (value.start < request.start && value.end > request.start)
                throw std::runtime_error("Cannot insert inside a parameter");
        after = request.kind == "group" ? newline + "{" + newline + newline + "}" + newline :
            ' ' + EditorValue(request) + ' ';
    } else {
        throw std::runtime_error("Unsupported editor operation");
    }
    source.replace(patchStart, request.end - request.start, after);
    return {{patchStart, request.before, std::move(after)}};
}
