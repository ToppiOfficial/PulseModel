#pragma once
#include <cctype>
#include <string>
#include <vector>

// An SFM auto-rigger script, read as text and never run. It supplies the target's
// bone roles: the `boneList` and the options of the final BuildRig(...) call.
struct RigScript {
    std::vector<std::string> bones; // fixed slots, see kRigSlot
    int spines = 1;                 // spine slots in use, 1 to 4
    bool toe = false, collar = false, fingers = false;
};

namespace rigscript {

// Comments dropped; quoted text is kept as written.
inline std::string StripComments(const std::string& text) {
    std::string out;
    char quote = 0;
    for (size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (quote) {
            out += c;
            if (c == quote || c == '\n') quote = 0;
        } else if (c == '#') {
            while (i + 1 < text.size() && text[i + 1] != '\n') ++i;
        } else {
            if (c == '\'' || c == '"') quote = c;
            out += c;
        }
    }
    return out;
}

inline std::string Trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

// The text between a bracket at `open` and its match, split on top-level commas.
inline bool SplitGroup(const std::string& s, size_t open, std::vector<std::string>& items) {
    const char closer = s[open] == '(' ? ')' : ']';
    int depth = 0;
    char quote = 0;
    std::string item;
    for (size_t i = open + 1; i < s.size(); ++i) {
        const char c = s[i];
        if (quote) {
            item += c;
            if (c == quote) quote = 0;
            continue;
        }
        if (c == '\'' || c == '"') quote = c;
        else if (c == '(' || c == '[') ++depth;
        else if ((c == ')' || c == ']') && depth > 0) --depth;
        else if (depth == 0 && c == closer) {
            if (!Trim(item).empty()) items.push_back(Trim(item));
            return true;
        } else if (depth == 0 && c == ',') {
            items.push_back(Trim(item));
            item.clear();
            continue;
        }
        item += c;
    }
    return false;
}

// 'x', "x", u'x' or r'x'; escapes are not supported.
inline bool Unquote(const std::string& item, std::string& out) {
    size_t a = 0;
    while (a < item.size() && (item[a] == 'u' || item[a] == 'U' || item[a] == 'r' || item[a] == 'R')) ++a;
    if (a + 1 >= item.size() || (item[a] != '\'' && item[a] != '"') || item.back() != item[a]) return false;
    out = item.substr(a + 1, item.size() - a - 2);
    return out.find('\\') == std::string::npos && !out.empty();
}

inline bool ParseBool(const std::string& s, bool& out) {
    if (s == "True") out = true;
    else if (s == "False") out = false;
    else return false;
    return true;
}

inline bool ParseList(const std::string& group, std::vector<std::string>& bones) {
    std::vector<std::string> items;
    if (!SplitGroup(group, 0, items)) return false;

    for (const std::string& item : items) {
        if (item == "None") { // an unused slot
            bones.push_back(std::string());
            continue;
        }

        std::string name;
        if (!Unquote(item, name)) return false;
        bones.push_back(name);
    }
    return !bones.empty();
}

// Offset of the last `BuildRig(` call that is not its definition.
inline size_t FindCall(const std::string& s) {
    size_t at = std::string::npos;
    for (size_t p = s.find("BuildRig("); p != std::string::npos; p = s.find("BuildRig(", p + 1)) {
        const bool def = p >= 4 && s.compare(p - 4, 4, "def ") == 0;
        const bool word = p > 0 && (std::isalnum(static_cast<unsigned char>(s[p - 1])) || s[p - 1] == '_');
        if (!def && !word) at = p;
    }
    return at;
}

// Offset of the `[` of `name = [...]`, the last such assignment.
inline size_t FindListAssign(const std::string& s, const std::string& name) {
    size_t at = std::string::npos;
    for (size_t p = s.find(name); p != std::string::npos; p = s.find(name, p + 1)) {
        if (p > 0 && (std::isalnum(static_cast<unsigned char>(s[p - 1])) || s[p - 1] == '_')) continue;
        size_t q = p + name.size();
        while (q < s.size() && (s[q] == ' ' || s[q] == '\t')) ++q;
        if (q >= s.size() || s[q] != '=' || (q + 1 < s.size() && s[q + 1] == '=')) continue;
        ++q;
        while (q < s.size() && std::isspace(static_cast<unsigned char>(s[q]))) ++q;
        if (q < s.size() && s[q] == '[') at = q;
    }
    return at;
}

} // namespace rigscript

// Bone list slots, as the script indexes them: 0 pelvis, 1-4 spine, 5 neck, 6 head,
// 7-10 right thigh/calf/foot/toe, 11 right clavicle, 12-14 right arm/forearm/hand,
// 15-18 and 19-22 the same on the left.
constexpr size_t kRigSlots = 23;

inline bool ParseRigScript(const std::string& raw, RigScript& rig, std::string& err) {
    const std::string s = rigscript::StripComments(raw);
    const size_t call = rigscript::FindCall(s);
    std::vector<std::string> args;
    if (call == std::string::npos || !rigscript::SplitGroup(s, call + 8, args) || args.size() != 7) {
        err = "no BuildRig(num, toe, collar, bones, footroll, fingers, handpreset) call in the rig script";
        return false;
    }

    int num = 0;
    if (args[0].size() != 1 || args[0][0] < '1' || args[0][0] > '4') {
        err = "rig script: the spine count must be a literal from 1 to 4";
        return false;
    }
    num = args[0][0] - '0';
    if (!rigscript::ParseBool(args[1], rig.toe) || !rigscript::ParseBool(args[2], rig.collar) ||
        !rigscript::ParseBool(args[5], rig.fingers)) {
        err = "rig script: toe, collar and finger options must be literal True or False";
        return false;
    }
    rig.spines = num;

    std::string list = args[3];
    if (list.empty() || list[0] != '[') {
        const size_t open = rigscript::FindListAssign(s, list);
        if (open == std::string::npos) {
            err = "rig script: cannot find the bone list \"" + list + "\"";
            return false;
        }
        list = s.substr(open);
    }
    if (!rigscript::ParseList(list, rig.bones) || rig.bones.size() < kRigSlots) {
        err = "rig script: the bone list must be " + std::to_string(kRigSlots) + " quoted names or None";
        return false;
    }
    rig.bones.resize(kRigSlots);
    return true;
}
