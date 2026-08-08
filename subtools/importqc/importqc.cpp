// importqc - stock studiomdl .qc -> .pulseqc (importqc.h).
//
// The rewrite is line-based: a converted command's source lines are replaced
// wholesale by generated text and everything else is copied through, so the
// script keeps its own comments, order and formatting.

#include "importqc.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace pulse::importqc {
namespace {

namespace fs = std::filesystem;

struct Tok {
    std::string text;
    bool quoted = false; // came from "..." - so a quoted `mesh` is a name
    int line = 0;
};

// keyvalues1 lexing, same rules as the compiler's loader: whitespace-separated
// words, "quoted strings", `//` line comments, braces standalone.
bool Tokenize(const std::string& s, const std::string& file, std::vector<Tok>& out,
              std::string* err) {
    int line = 1;
    size_t i = 0;
    while (i < s.size()) {
        const char c = s[i];
        if (c == '\n') {
            line++;
            i++;
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(c))) {
            i++;
            continue;
        }
        if (c == '/' && i + 1 < s.size() && s[i + 1] == '/') {
            while (i < s.size() && s[i] != '\n')
                i++;
            continue;
        }
        if (c == '/' && i + 1 < s.size() && s[i + 1] == '*') {
            i += 2;
            while (i + 1 < s.size() && !(s[i] == '*' && s[i + 1] == '/')) {
                if (s[i] == '\n')
                    line++;
                i++;
            }
            i = i + 1 < s.size() ? i + 2 : s.size();
            continue;
        }
        if (c == '"') {
            Tok t;
            t.line = line;
            t.quoted = true;
            i++;
            while (i < s.size() && s[i] != '"') {
                if (s[i] == '\n')
                    line++;
                t.text.push_back(s[i++]);
            }
            if (i >= s.size()) {
                if (err) *err = file + "(" + std::to_string(t.line) + "): unterminated string";
                return false;
            }
            i++;
            out.push_back(std::move(t));
            continue;
        }
        if (c == '{' || c == '}') {
            out.push_back({std::string(1, c), false, line});
            i++;
            continue;
        }
        Tok t;
        t.line = line;
        while (i < s.size()) {
            const char d = s[i];
            if (std::isspace(static_cast<unsigned char>(d)) || d == '{' || d == '}' || d == '"' ||
                (d == '/' && i + 1 < s.size() && (s[i + 1] == '/' || s[i + 1] == '*')))
                break;
            t.text.push_back(d);
            i++;
        }
        out.push_back(std::move(t));
    }
    return true;
}

std::string Lower(std::string s) {
    for (char& c : s)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

std::string Q(const std::string& s) { return "\"" + s + "\""; }

// Append `lines`, blank-separated wherever the command changes - a $model body
// hoists a long run of them. A comment belongs to the line below it.
void AppendGrouped(std::vector<std::string>& out, const std::vector<std::string>& lines,
                   std::string prev) {
    auto cmdOf = [](const std::string& s) {
        return s.compare(0, 2, "//") == 0 ? std::string() : s.substr(0, s.find(' '));
    };
    std::vector<std::string> key(lines.size());
    for (int i = static_cast<int>(lines.size()) - 1; i >= 0; --i) {
        key[i] = cmdOf(lines[i]);
        if (key[i].empty() && i + 1 < static_cast<int>(lines.size()))
            key[i] = key[i + 1];
    }
    for (size_t i = 0; i < lines.size(); ++i) {
        if (!key[i].empty() && !prev.empty() && key[i] != prev)
            out.push_back("");
        if (!key[i].empty())
            prev = key[i];
        out.push_back(lines[i]);
    }
}

// The converted twin of a script: .qci -> .pulseqci, anything else ->
// .pulseqc. Never the input path itself.
fs::path ConvertedPath(const fs::path& p) {
    const std::string ext = Lower(p.extension().string()) == ".qci" ? ".pulseqci" : ".pulseqc";
    fs::path out = p.parent_path() / (p.stem().string() + ext);
    if (out.filename() == p.filename())
        out = p.parent_path() / (p.stem().string() + ".converted" + ext);
    return out;
}

// The token stream plus the per-line rewrite plan.
struct Conv {
    std::string file;
    fs::path dir;                // the input's directory - $include is relative to it
    std::set<std::string>* seen; // scripts already converted, shared down the include tree
    std::vector<Tok> toks;
    size_t pos = 0;
    std::vector<std::string> src;  // input lines, 0-based
    std::vector<std::string> repl; // generated text, keyed by a command's first line
    std::vector<bool> drop;        // line consumed by a conversion
    std::map<std::string, std::string> meshOf; // lowercased filename -> $rendermesh name
    std::set<std::string> usedNames;           // lowercased, across every $rendermesh
    // every $rendermesh, written as one block above the first group instead of
    // beside the group that uses it
    std::vector<std::string> meshDecls;
    int firstSlot = -1; // first line a conversion replaced; where that block goes
    // $staticprop/$simpleprop become the $modelarchetype written under
    // $modelname, so the flag has to be known before that line is reached
    std::string archetype = "general";
    // the open hitbox set: stock's $hboxset/$hbox are flat lines, .pulseqc
    // encloses them, so the boxes are collected until the set ends
    std::string hboxSet;
    int hboxSlot = -1; // line the finished block replaces
    std::vector<std::string> hboxes;
    bool haveArchetype = false; // the script already writes $modelarchetype
    bool haveModelName = false; // this file has the $modelname the archetype goes under
    bool renderPass = false;    // an $opaque/$mostlyopaque was already converted
    std::vector<fs::path> cdStack; // $pushd, joined - stock's nest, a search dir does not
    std::string err;
    int commands = 0;
};

bool Fail(Conv& c, int line, const std::string& msg) {
    c.err = c.file + "(" + std::to_string(line) + "): " + msg;
    return false;
}

bool More(const Conv& c) { return c.pos < c.toks.size(); }
bool SameLine(const Conv& c, int line) { return More(c) && c.toks[c.pos].line == line; }

// stock's GetToken(false) does not cross lines, so every command argument sits
// on the command's own line.
bool Want(Conv& c, int line, const std::string& where, const char* what, std::string& out) {
    if (!SameLine(c, line))
        return Fail(c, line, where + " expects " + what);
    out = c.toks[c.pos++].text;
    return true;
}

// Replace [first,last] with `out`, one generated line each.
void Emit(Conv& c, int first, int last, const std::vector<std::string>& out) {
    for (int i = first; i <= last && i <= static_cast<int>(c.src.size()); ++i)
        c.drop[i - 1] = true;
    std::string& slot = c.repl[first - 1];
    for (const std::string& s : out)
        slot += s + "\n";
    c.commands++;
}

// The $rendermesh block goes above the first group, so only a group claims the
// slot - a flag rewritten higher up in the file must not.
void ClaimMeshSlot(Conv& c, int line) {
    if (c.firstSlot < 0)
        c.firstSlot = line;
}

// The line the last consumed token sat on - the end of the range Emit replaces.
int LastLine(const Conv& c) { return c.pos > 0 ? c.toks[c.pos - 1].line : 0; }

// The rest of the line, re-quoted as authored.
std::string RestOfLine(Conv& c, int line) {
    std::string s;
    while (SameLine(c, line)) {
        const Tok& t = c.toks[c.pos++];
        s += " " + (t.quoted ? Q(t.text) : t.text);
    }
    return s;
}

// "upper" or "lower" - stock keys the lid off the first letter of its type.
std::string LidType(const std::string& type) {
    return !type.empty() && Lower(type)[0] == 'u' ? "upper" : "lower";
}

// A unique $rendermesh name, preferring `want`.
std::string Reserve(Conv& c, const std::string& want) {
    std::string base;
    for (char ch : want)
        base.push_back(std::isspace(static_cast<unsigned char>(ch)) || ch == '"' ? '_' : ch);
    if (base.empty())
        base = "mesh";
    std::string name = base;
    for (int n = 2; c.usedNames.count(Lower(name)); ++n)
        name = base + "_" + std::to_string(n);
    c.usedNames.insert(Lower(name));
    return name;
}

// The $rendermesh a bodygroup's `studio` line gets. Two entries naming the same
// file share one, since a plain studio entry has nothing private to it.
std::string MeshFor(Conv& c, const std::string& file) {
    const std::string key = Lower(file);
    auto it = c.meshOf.find(key);
    if (it != c.meshOf.end())
        return it->second;
    const std::string name = Reserve(c, fs::path(file).stem().string());
    c.meshOf[key] = name;
    return name;
}

// Post-filename `studio` options (reverse/scale/faces/bias/subd). None has a
// $rendermesh spelling, so each becomes a note in the output.
void StudioOpts(Conv& c, int line, std::vector<std::string>& notes) {
    while (SameLine(c, line)) {
        const Tok& t = c.toks[c.pos];
        if (!t.quoted && t.text == "{")
            break;
        c.pos++;
        const std::string o = Lower(t.text);
        std::string args;
        const int extra = (o == "scale" || o == "bias") ? 1 : (o == "faces" ? 2 : 0);
        for (int i = 0; i < extra && SameLine(c, line); ++i)
            args += " " + c.toks[c.pos++].text;
        notes.push_back("// importqc: dropped `" + t.text + args +
                        "` - $rendermesh has no per-mesh equivalent");
    }
}

// One $model body: everything it declares is top-level and global in .pulseqc,
// except the VTA flex list, which becomes the $rendermesh's $vta block.
struct Body {
    std::string vtaFile;
    std::vector<std::string> vtaFlexes; // `flex "x" frame 3 position 1`
    std::vector<std::string> hoisted;   // $flexcontroller / $eyeball / ... lines
    std::vector<std::string> eyeballs;  // declared so far, for an eyelid with no `eyeball`
    int mouths = 0;                     // $mouth numbers by declaration order
};

// `flex`/`flexpair`/`defaultflex` options: frame/position/split/decay. `split`
// is flexpair's own argument as well as an option, and the option wins.
bool FlexOptions(Conv& c, int line, const std::string& where, const std::string& name,
                 std::string split, Body& b) {
    std::string entry = "flex " + Q(name);
    while (SameLine(c, line)) {
        const std::string o = Lower(c.toks[c.pos].text);
        if (o != "frame" && o != "position" && o != "split" && o != "decay")
            return Fail(c, line, where + ": unknown flex option \"" + c.toks[c.pos].text + "\"");
        c.pos++;
        std::string v;
        if (!Want(c, line, where, "a value", v))
            return false;
        if (o == "split")
            split = v;
        else
            entry += " " + o + " " + v;
    }
    b.vtaFlexes.push_back(entry);
    // stock splits a paired flex into <name>L/<name>R off the mesh balance
    if (!split.empty() && split != "0")
        b.hoisted.push_back("$morphsplitstereo splitfactor " + split + " " + Q(name));
    return true;
}

// eyelid: the three VTA frames become named deltas so $eyelid can address them
// by name, which is how .pulseqc spells lid poses.
bool Eyelid(Conv& c, int line, Body& b) {
    const std::string where = "eyelid";
    std::string type, file;
    if (!Want(c, line, where, "a type", type) || !Want(c, line, where, "a .vta file", file))
        return false;
    if (b.vtaFile.empty())
        b.vtaFile = file;

    static const char* kSlot[3] = {"lowerer", "neutral", "raiser"};
    std::string frame[3], target[3], eyeball, split;
    while (SameLine(c, line)) {
        const Tok& t = c.toks[c.pos++];
        const std::string o = Lower(t.text);
        int slot = -1;
        for (int i = 0; i < 3; ++i)
            if (o == kSlot[i])
                slot = i;
        if (slot >= 0) {
            if (!Want(c, line, where, "a frame", frame[slot]) ||
                !Want(c, line, where, "a target", target[slot]))
                return false;
        } else if (o == "split") {
            if (!Want(c, line, where, "a split", split))
                return false;
        } else if (o == "eyeball") {
            if (!Want(c, line, where, "an eyeball name", eyeball))
                return false;
        } else {
            return Fail(c, line, where + ": unknown option \"" + t.text + "\"");
        }
    }

    std::string cmd = "$eyelid " + LidType(type) + " flexdesc " + Q(type);
    for (int i = 0; i < 3; ++i) {
        const std::string delta = type + "_" + kSlot[i];
        if (frame[i].empty()) {
            cmd += std::string(" ") + kSlot[i] + " - 0";
            continue;
        }
        b.vtaFlexes.push_back("flex " + Q(delta) + " frame " + frame[i]);
        cmd += std::string(" ") + kSlot[i] + " " + Q(delta) + " " + target[i];
    }
    if (eyeball.empty() && b.eyeballs.size() == 1)
        eyeball = b.eyeballs.front();
    if (eyeball.empty()) {
        b.hoisted.push_back("// importqc: this eyelid named no eyeball - $eyelid needs one, so "
                            "add it by hand");
        eyeball = b.eyeballs.empty() ? "?" : b.eyeballs.front();
    }
    if (!split.empty() && split != "0")
        cmd += " split " + split;
    b.hoisted.push_back(cmd + " eyeball " + Q(eyeball));
    return true;
}

// dmxeyelid: already delta-named, so it is a rename plus dropping the mesh file
// (the deltas resolve against whichever body carries them).
bool DmxEyelid(Conv& c, int line, Body& b) {
    const std::string where = "dmxeyelid";
    std::string type, file;
    if (!Want(c, line, where, "a type", type) || !Want(c, line, where, "a mesh file", file))
        return false;
    b.hoisted.push_back("$eyelid " + LidType(type) + RestOfLine(c, line));
    return true;
}

// eyeball: positional in stock, named clauses in .pulseqc. The iris material is
// read and discarded by both.
bool Eyeball(Conv& c, int line, Body& b) {
    const std::string where = "eyeball";
    std::string name, bone, x, y, z, material, diameter, angle, iris, pupil;
    if (!Want(c, line, where, "a name", name) || !Want(c, line, where, "a bone", bone) ||
        !Want(c, line, where, "an origin x", x) || !Want(c, line, where, "an origin y", y) ||
        !Want(c, line, where, "an origin z", z) ||
        !Want(c, line, where, "a material", material) ||
        !Want(c, line, where, "a diameter", diameter) || !Want(c, line, where, "an angle", angle) ||
        !Want(c, line, where, "an iris material", iris) ||
        !Want(c, line, where, "a pupil scale", pupil))
        return false;
    b.eyeballs.push_back(name);
    b.hoisted.push_back("$eyeball " + Q(name) + " bone " + Q(bone) + " origin " + x + " " + y +
                        " " + z + " diameter " + diameter + " angle " + angle + " pupilscale " +
                        pupil + " material " + Q(material));
    return true;
}

bool ConvertFile(const fs::path& in, const fs::path& out, bool bodyMode,
                 std::set<std::string>& seen, std::string* err);

// Convert the script an $include names and give back the lines replacing it -
// .qci becomes .pulseqci and the path re-points at the twin. A file that is not
// there stands as it is, with a note above it: there is nothing to convert.
bool IncludeFile(Conv& c, int line, const std::string& rel, const std::string& flags,
                 bool bodyMode, std::vector<std::string>& out) {
    const fs::path relPath(rel);
    const fs::path full = (relPath.is_absolute() ? relPath : c.dir / relPath).lexically_normal();
    std::error_code ec;
    if (!fs::is_regular_file(full, ec)) {
        out.push_back("// importqc: \"" + rel +
                      "\" is missing - not converted, this path still points at stock QC");
        out.push_back("$include " + Q(rel) + flags);
        return true;
    }

    const fs::path canon = fs::weakly_canonical(full, ec);
    const std::string key = Lower((ec ? full : canon).string());
    if (c.seen->insert(key).second &&
        !ConvertFile(full, ConvertedPath(full), bodyMode, *c.seen, &c.err))
        return false;

    // keep the path as authored, only the filename changes
    const fs::path newRel = relPath.parent_path() / ConvertedPath(relPath).filename();
    out.push_back("$include " + Q(newRel.generic_string()) + flags);
    return true;
}

// One $model body option. Everything it writes is a top-level .pulseqc command,
// so it goes to `hoisted` and the caller decides where that lands.
bool BodyOption(Conv& c, const Tok& t, const std::string& where, Body& b) {
    {
        const int line = t.line;
        const std::string o = Lower(t.text);

        if (o == "eyeball") {
            if (!Eyeball(c, line, b))
                return false;
        } else if (o == "eyelid") {
            if (!Eyelid(c, line, b))
                return false;
        } else if (o == "dmxeyelid") {
            if (!DmxEyelid(c, line, b))
                return false;
        } else if (o == "mouth") {
            // the index is declaration order in .pulseqc, so it comes off here
            std::string index;
            if (!Want(c, line, "mouth", "an index", index))
                return false;
            if (index != std::to_string(b.mouths))
                b.hoisted.push_back("// importqc: mouth index " + index + " was written " +
                                    std::to_string(b.mouths) + "th - $mouth numbers by order");
            b.mouths++;
            b.hoisted.push_back("$mouth" + RestOfLine(c, line));
        } else if (o == "flexcontroller") {
            b.hoisted.push_back("$flexcontroller" + RestOfLine(c, line));
        } else if (o == "localvar") {
            b.hoisted.push_back("$flexlocalvar" + RestOfLine(c, line));
        } else if (!t.quoted && t.text.size() > 1 && t.text[0] == '%') {
            // %<morph> = <expr>. A trailing backslash token continues the rule
            // on the next line - Valve's own scripts write it as `\\`.
            std::string expr;
            int at = line;
            for (;;) {
                expr += RestOfLine(c, at);
                size_t end = expr.size();
                while (end > 0 && expr[end - 1] == '\\')
                    end--;
                if (end == expr.size() || end == 0 || expr[end - 1] != ' ')
                    break;
                expr.resize(end - 1);
                if (!More(c))
                    break;
                at = c.toks[c.pos].line;
            }
            b.hoisted.push_back("$flexrule " + Q(t.text.substr(1)) + expr);
        } else if (o == "flexfile") {
            if (!Want(c, line, "flexfile", "a .vta file", b.vtaFile))
                return false;
        } else if (o == "flex" || o == "flexpair" || o == "defaultflex") {
            std::string name = "default", split;
            if (o != "defaultflex" && !Want(c, line, o, "a flex name", name))
                return false;
            if (o == "flexpair" && !Want(c, line, o, "a split", split))
                return false;
            if (!FlexOptions(c, line, o, name, split, b))
                return false;
        } else if (o == "vcafile") {
            std::string file;
            if (!Want(c, line, "vcafile", "a .vca file", file))
                return false;
            b.hoisted.push_back("// importqc: `vcafile " + file +
                                "` is a $rendermesh $vca option - move it there by hand");
        } else if (o == "noautodmxrules") {
            // nothing to write: .pulseqc imports a DMX flex rig only when a
            // $datamodelflexes asks for it
        } else if (o == "attachment" || o == "spherenormals") {
            b.hoisted.push_back("// importqc: dropped `" + t.text + RestOfLine(c, line) +
                                "` - no .pulseqc equivalent");
        } else if (o == "$include") {
            // stock splices the file's tokens into the body, so it holds body
            // options - convert it as a body and $include the twin at top level
            std::string rel;
            if (!Want(c, line, "$include", "a script path", rel))
                return false;
            if (!IncludeFile(c, line, rel, RestOfLine(c, line), /*bodyMode=*/true, b.hoisted))
                return false;
        } else {
            return Fail(c, line, where + ": unknown model option \"" + t.text + "\"");
        }
    }
    return true;
}

// The body of a $model, or a whole file of body options - one option per source
// line, replaced by what it hoists.
bool ModelBody(Conv& c, const std::string& where, Body& b) {
    c.pos++; // '{'
    for (;;) {
        if (!More(c))
            return Fail(c, LastLine(c), where + " is missing '}'");
        const Tok t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            return true;
        if (!BodyOption(c, t, where, b))
            return false;
    }
}

// $bodygroup <name> { studio <file> | blank ... }
bool Bodygroup(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    std::string name;
    if (!Want(c, cmd.line, "$bodygroup", "a name", name))
        return false;
    if (!More(c) || c.toks[c.pos].quoted || c.toks[c.pos].text != "{")
        return Fail(c, cmd.line, "$bodygroup \"" + name + "\" expects '{'");
    c.pos++;

    std::vector<std::string> group;
    for (;;) {
        if (!More(c))
            return Fail(c, cmd.line, "$bodygroup \"" + name + "\" is missing '}'");
        const Tok t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        const std::string o = Lower(t.text);
        if (o == "blank") {
            group.push_back("    blank");
            continue;
        }
        if (o != "studio")
            return Fail(c, t.line, "$bodygroup \"" + name + "\": unknown option \"" + t.text +
                                       "\" (expected studio, blank or '}')");
        std::string file;
        if (!Want(c, t.line, "studio", "a source filename", file))
            return false;
        StudioOpts(c, t.line, c.meshDecls);
        const std::string mesh = MeshFor(c, file);
        c.meshDecls.push_back("$rendermesh " + Q(mesh) + " " + Q(file));
        group.push_back("    mesh " + Q(mesh));
    }

    ClaimMeshSlot(c, cmd.line);
    std::vector<std::string> out;
    out.push_back("$modelgroup " + Q(name) + " {");
    out.insert(out.end(), group.begin(), group.end());
    out.push_back("}");
    Emit(c, cmd.line, LastLine(c), out);
    return true;
}

// $bodygrouppreset <name> { <group> <choice index> ... } - the same shape under
// its new name, so only the command word and the quoting change.
bool BodygroupPreset(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    std::string name;
    if (!Want(c, cmd.line, "$bodygrouppreset", "a name", name))
        return false;
    if (!More(c) || c.toks[c.pos].quoted || c.toks[c.pos].text != "{")
        return Fail(c, cmd.line, "$bodygrouppreset \"" + name + "\" expects '{'");
    c.pos++;

    std::vector<std::string> out{"$modelgrouppreset " + Q(name) + " {"};
    for (;;) {
        if (!More(c))
            return Fail(c, cmd.line, "$bodygrouppreset \"" + name + "\" is missing '}'");
        const Tok t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        std::string choice;
        if (!Want(c, t.line, "$bodygrouppreset \"" + name + "\"", "a choice index", choice))
            return false;
        out.push_back("    " + Q(t.text) + " " + choice);
    }
    out.push_back("}");
    Emit(c, cmd.line, LastLine(c), out);
    return true;
}

// $model <name> <file> [studio opts] [{ body }]  /  $body <name> <file>
// Both are one bodypart with one choice, so both become a $rendermesh plus the
// inline $modelgroup form.
bool Model(Conv& c, bool bodied) {
    const Tok cmd = c.toks[c.pos++];
    const std::string what = cmd.text;
    std::string name, file;
    if (!Want(c, cmd.line, what, "a name", name) ||
        !Want(c, cmd.line, what, "a source filename", file))
        return false;

    StudioOpts(c, cmd.line, c.meshDecls);

    Body b;
    const std::string where = what + " \"" + name + "\"";
    if (bodied && More(c) && !c.toks[c.pos].quoted && c.toks[c.pos].text == "{") {
        if (!ModelBody(c, where, b))
            return false;
    }

    // named after the model, never shared: a $model's mesh carries its own
    // morph import and options, so another reference to the file gets its own
    const std::string mesh = Reserve(c, name);

    const std::string decl = "$rendermesh " + Q(mesh) + " " + Q(file);
    if (b.vtaFlexes.empty()) {
        c.meshDecls.push_back(decl);
    } else {
        c.meshDecls.push_back(decl + " {");
        c.meshDecls.push_back("    $vta " + Q(b.vtaFile) + " {");
        for (const std::string& f : b.vtaFlexes)
            c.meshDecls.push_back("        " + f);
        c.meshDecls.push_back("    }");
        c.meshDecls.push_back("}");
    }

    ClaimMeshSlot(c, cmd.line);
    std::vector<std::string> out;
    out.push_back("$modelgroup " + Q(name) + " " + Q(mesh));
    AppendGrouped(out, b.hoisted, "$modelgroup");
    Emit(c, cmd.line, LastLine(c), out);
    return true;
}

// $texturegroup [<name>] { { <row> } ... } - stock lists a full material row per
// family and pairs them with row 0 by position. Each $set names both materials
// instead, so only the slots that differ are written. An unchanged family still
// gets an empty $set: dropping it would renumber every skin after it.
bool TextureGroup(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    const std::string where = "$texturegroup";
    if (More(c) && c.toks[c.pos].text != "{")
        c.pos++; // the group name, which .pulseqc does not take
    if (!More(c) || c.toks[c.pos].quoted || c.toks[c.pos].text != "{")
        return Fail(c, cmd.line, where + " expects '{'");
    c.pos++;

    std::vector<std::vector<std::string>> rows;
    for (;;) {
        if (!More(c))
            return Fail(c, cmd.line, where + " is missing '}'");
        const Tok t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        if (t.quoted || t.text != "{")
            return Fail(c, t.line, where + ": expected a material row or '}', got \"" + t.text +
                                       "\"");
        std::vector<std::string> row;
        for (;;) {
            if (!More(c))
                return Fail(c, t.line, where + ": a material row is missing '}'");
            const Tok m = c.toks[c.pos++];
            if (!m.quoted && m.text == "}")
                break;
            row.push_back(m.text);
        }
        if (!rows.empty() && row.size() != rows.front().size())
            return Fail(c, t.line, where + ": every row must name the same materials as the "
                                           "first, which lists " +
                                       std::to_string(rows.front().size()));
        rows.push_back(std::move(row));
    }

    std::vector<std::string> out;
    if (rows.size() > 1) {
        out.push_back("$texturegroup {");
        for (size_t fam = 1; fam < rows.size(); ++fam) {
            out.push_back("    $set {");
            for (size_t i = 0; i < rows[fam].size(); ++i)
                if (rows[fam][i] != rows[0][i])
                    out.push_back("        material " + Q(rows[0][i]) + " " + Q(rows[fam][i]));
            out.push_back("    }");
        }
        out.push_back("}");
    }
    Emit(c, cmd.line, LastLine(c), out);
    return true;
}

// $collisionmodel / $collisionjoints <source> [{ ... }] -> $physicsmodel.
// .pulseqc has no single-body/ragdoll split - the compile stage counts the bones
// the geometry resolves to and picks - so both land on the same command. The
// geometry options move onto the $physicsshape and the per-joint commands become
// $physicsmarkup / $physicsjoint / $physicscollide.
bool CollisionModel(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    const std::string where = cmd.text;
    std::string file;
    if (!Want(c, cmd.line, where, "a source filename", file))
        return false;

    std::vector<std::string> shape, head, pairs, tail, files{file};

    // Everything per-bone is grouped under the bone it belongs to rather than
    // run together per command, the way a decompile writes it - a ragdoll is
    // read and edited bone by bone.
    struct BoneEdits {
        std::vector<std::string> markup, axes;
    };
    std::vector<std::string> order; // first appearance
    std::map<std::string, BoneEdits> byBone;
    auto boneOf = [&](const std::string& bone) -> BoneEdits& {
        if (byBone.find(bone) == byBone.end())
            order.push_back(bone);
        return byBone[bone];
    };
    // stock let a later $joint* line silently overwrite an earlier one for the
    // same bone and field/axis; .pulseqc rejects the repeat, so both are written
    // out and tagged - we cannot know which one was meant.
    auto firstWord = [](const std::string& s) { return s.substr(0, s.find(' ')); };
    auto addLine = [&](std::vector<std::string>& lines, std::string line) {
        const std::string key = firstWord(line);
        for (const std::string& e : lines)
            if (firstWord(e) == key) {
                line += " // duplicate \"" + key + "\"";
                break;
            }
        lines.push_back(std::move(line));
    };
    auto markup = [&](const std::string& bone, const std::string& field) {
        addLine(boneOf(bone).markup, field);
    };

    if (More(c) && !c.toks[c.pos].quoted && c.toks[c.pos].text == "{") {
        c.pos++;
        for (;;) {
            if (!More(c))
                return Fail(c, cmd.line, where + " is missing '}'");
            const Tok t = c.toks[c.pos++];
            const int line = t.line;
            if (!t.quoted && t.text == "}")
                break;
            const std::string o = Lower(t.text);
            const std::string sub = where + " " + t.text;
            std::string a, b;
            auto arg = [&](const char* what, std::string& dst) {
                return Want(c, line, sub, what, dst);
            };

            if (o == "$mass") {
                if (!arg("a mass in kg", a))
                    return false;
                head.push_back("    $mass " + a);
            } else if (o == "$automass" || o == "$calculatemass") {
                head.push_back("    $automass");
            } else if (o == "$inertia" || o == "$damping" || o == "$rotdamping" || o == "$drag" ||
                       o == "$weldposition" || o == "$weldnormal") {
                if (!arg("a value", a))
                    return false;
                head.push_back("    " + o + " " + a);
            } else if (o == "$noselfcollisions" || o == "$assumeworldspace") {
                head.push_back("    " + o);
            } else if (o == "$rootbone") {
                if (!arg("a bone name", a))
                    return false;
                head.push_back("    $rootbone " + Q(a));
            } else if (o == "$masscenter") {
                head.push_back("    $masscenter" + RestOfLine(c, line));
            } else if (o == "$animatedfriction") {
                tail.push_back("    $animatedfriction" + RestOfLine(c, line));
            } else if (o == "$concave" || o == "$concaveperjoint") {
                shape.push_back("        concave");
            } else if (o == "$remove2d") {
                shape.push_back("        remove2d");
            } else if (o == "$maxconvexpieces") {
                if (!arg("a piece count", a))
                    return false;
                shape.push_back("        maxconvex " + a);
            } else if (o == "$addconvexsrc") {
                if (!arg("a source filename", a))
                    return false;
                files.push_back(a);
            } else if (o == "$jointskip") {
                if (!arg("a bone name", a))
                    return false;
                markup(a, "skip");
            } else if (o == "$jointmerge") {
                if (!arg("a bone name", a) || !arg("a second bone name", b))
                    return false;
                markup(b, "mergeinto " + Q(a)); // the second bone is the one folded away
            } else if (o == "$jointinertia" || o == "$jointdamping" || o == "$jointrotdamping" ||
                       o == "$jointmassbias") {
                if (!arg("a bone name", a) || !arg("a value", b))
                    return false;
                markup(a, o.substr(6) + " " + b); // past the "$joint" prefix
            } else if (o == "$jointcollide" || o == "$jointnocollide") {
                if (!arg("a bone name", a) || !arg("a second bone name", b))
                    return false;
                // a pair names two bones, so it belongs to neither bone's group
                pairs.push_back("    " +
                                std::string(o == "$jointcollide" ? "$physicscollide"
                                                                 : "$physicsnocollide") +
                                " " + Q(a) + " " + Q(b));
            } else if (o == "$jointcollidealltoall") {
                if (!More(c) || c.toks[c.pos].quoted || c.toks[c.pos].text != "{")
                    return Fail(c, line, sub + " expects '{'");
                c.pos++;
                std::vector<std::string> bones;
                for (;;) {
                    if (!More(c))
                        return Fail(c, line, sub + " is missing '}'");
                    const Tok n = c.toks[c.pos++];
                    if (!n.quoted && n.text == "}")
                        break;
                    bones.push_back(n.text);
                }
                // $physicscollide is symmetric, so each pair is written once
                for (size_t i = 0; i < bones.size(); ++i)
                    for (size_t j = i + 1; j < bones.size(); ++j)
                        pairs.push_back("    $physicscollide " + Q(bones[i]) + " " + Q(bones[j]));
            } else if (o == "$jointconstrain") {
                // stock reads the limits positionally whatever the type is, so
                // free/fixed only keep their spelling when they match what they
                // would have written anyway
                std::string axis, type, lo, hi, fr = "1.0";
                if (!arg("a bone name", a) || !arg("an axis", axis) ||
                    !arg("free, limit or fixed", type) || !arg("a min angle", lo) ||
                    !arg("a max angle", hi))
                    return false;
                if (SameLine(c, line) && !arg("a friction value", fr))
                    return false;
                const std::string t2 = Lower(type);
                std::string axisLine = Lower(axis);
                bool fixed = false;
                if (t2 == "free" && std::atof(lo.c_str()) == -360.0 &&
                    std::atof(hi.c_str()) == 360.0) {
                    axisLine += " free";
                } else if (t2 == "fixed" && std::atof(lo.c_str()) == 0.0 &&
                           std::atof(hi.c_str()) == 0.0) {
                    axisLine += " fixed";
                    fixed = true;
                } else {
                    axisLine += " limit " + lo + " " + hi;
                }
                // a fixed axis zeroes its own friction, and an omitted one is
                // 1.0 on both sides - so neither is worth writing down
                if (!fixed && std::atof(fr.c_str()) != 1.0)
                    axisLine += " " + fr;
                addLine(boneOf(a).axes, axisLine);
            } else {
                head.push_back("// importqc: dropped `" + t.text + RestOfLine(c, line) +
                               "` - no $physicsmodel equivalent");
            }
        }
    }

    std::vector<std::string> out{"$physicsmodel {"};
    for (const std::string& f : files) {
        // a name that matches a $rendermesh reuses that mesh instead of loading
        // the file a second time
        const std::string from =
            c.usedNames.count(Lower(f)) ? "fromrendermesh " + Q(f) : "fromfile " + Q(f);
        out.push_back("    $physicsshape " + from + (shape.empty() ? "" : " {"));
        out.insert(out.end(), shape.begin(), shape.end());
        if (!shape.empty())
            out.push_back("    }");
    }
    out.insert(out.end(), head.begin(), head.end());

    // one blank-separated group per bone: its markup, then its constraint
    for (const std::string& bone : order) {
        const BoneEdits& e = byBone[bone];
        out.push_back("");
        if (!e.markup.empty()) {
            out.push_back("    $physicsmarkup " + Q(bone) + " {");
            for (const std::string& f : e.markup)
                out.push_back("        " + f);
            out.push_back("    }");
        }
        if (!e.axes.empty()) {
            out.push_back("    $physicsjoint " + Q(bone) + " {");
            for (const std::string& x : e.axes)
                out.push_back("        " + x);
            out.push_back("    }");
        }
    }
    for (const std::vector<std::string>* block : {&pairs, &tail}) {
        if (block->empty())
            continue;
        out.push_back("");
        out.insert(out.end(), block->begin(), block->end());
    }
    out.push_back("}");
    Emit(c, cmd.line, LastLine(c), out);
    return true;
}

// $origin <x> <y> <z> [rot] -> $transformmodel. The optional fourth value is a
// z rotation, which is the roll of an `angles` clause - both add the built-in 90.
bool Origin(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    std::string x, y, z;
    if (!Want(c, cmd.line, "$origin", "an X offset", x) ||
        !Want(c, cmd.line, "$origin", "a Y offset", y) ||
        !Want(c, cmd.line, "$origin", "a Z offset", z))
        return false;
    std::string out = "$transformmodel origin " + x + " " + y + " " + z;
    if (SameLine(c, cmd.line))
        out += " angles 0 0 " + c.toks[c.pos++].text;
    Emit(c, cmd.line, LastLine(c), {out});
    return true;
}

// $scale <float> -> $transformmodel scale.
bool Scale(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    std::string s;
    if (!Want(c, cmd.line, "$scale", "a scale", s))
        return false;
    Emit(c, cmd.line, LastLine(c),
         {"// importqc: PulseModel scales the whole model - procedural (VRD) bones and",
          "// eyeballs included - which stock studiomdl leaves at their authored size.",
          "$transformmodel scale " + s});
    return true;
}

// Write the collected set into the slot its first line claimed.
void FlushHboxSet(Conv& c) {
    if (c.hboxSlot < 0)
        return;
    std::string& slot = c.repl[c.hboxSlot - 1];
    slot += "$hboxset " + Q(c.hboxSet) + " {\n";
    for (const std::string& h : c.hboxes)
        slot += "    " + h + "\n";
    slot += "}\n";
    c.hboxSlot = -1;
    c.hboxes.clear();
}

// $hboxset <name> - opens a set; every $hbox until the next one belongs to it.
void HboxSet(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    FlushHboxSet(c);
    std::string name;
    if (SameLine(c, cmd.line))
        name = c.toks[c.pos++].text;
    c.hboxSet = name;
    c.hboxSlot = cmd.line;
    Emit(c, cmd.line, LastLine(c), {});
}

// $hbox <group> <bone> <min xyz> <max xyz> [p y r] [radius] [name]. The trailing
// optionals are positional in stock and named clauses in .pulseqc.
bool Hbox(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    static const char* kWhat[8] = {"a hit group", "a bone name", "a min X", "a min Y",
                                   "a min Z",     "a max X",     "a max Y", "a max Z"};
    std::string line = "$hbox";
    for (int i = 0; i < 8; ++i) {
        std::string v;
        if (!Want(c, cmd.line, "$hbox", kWhat[i], v))
            return false;
        line += " " + (i == 1 ? Q(v) : v);
    }

    std::vector<std::string> extra;
    while (SameLine(c, cmd.line)) {
        const Tok& t = c.toks[c.pos++];
        extra.push_back(t.quoted ? Q(t.text) : t.text);
    }
    // a script that already names its clauses keeps them as authored
    const std::string first = extra.empty() ? std::string() : Lower(extra[0]);
    if (first == "angles" || first == "radius" || first == "name") {
        for (const std::string& e : extra)
            line += " " + e;
        extra.clear();
    }
    if (extra.size() >= 3)
        line += " angles " + extra[0] + " " + extra[1] + " " + extra[2];
    if (extra.size() >= 4)
        line += " radius " + extra[3];
    if (extra.size() >= 5)
        line += " name " + (extra[4][0] == '"' ? extra[4] : Q(extra[4]));

    // a bare $hbox with no set open is stock's implicit "default" set
    if (c.hboxSlot < 0) {
        c.hboxSet = "default";
        c.hboxSlot = cmd.line;
    }
    if (extra.size() < 3 && !extra.empty())
        c.hboxes.push_back("// importqc: dropped a trailing value - stock reads the "
                           "angle offset as three");
    if (extra.size() > 5)
        c.hboxes.push_back("// importqc: dropped everything past the hitbox name");
    c.hboxes.push_back(line);
    Emit(c, cmd.line, LastLine(c), {});
    return true;
}

// $include "file.qci" [localdir] [optional]
bool Include(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    std::string rel;
    if (!Want(c, cmd.line, "$include", "a script path", rel))
        return false;
    const std::string flags = RestOfLine(c, cmd.line); // localdir / optional, as authored
    std::vector<std::string> out;
    if (!IncludeFile(c, cmd.line, rel, flags, /*bodyMode=*/false, out))
        return false;
    Emit(c, cmd.line, LastLine(c), out);
    return true;
}

// $modelname <path>, plus the $modelarchetype the prop flags became - it always
// sits directly under the name.
bool ModelName(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    std::string path;
    if (!Want(c, cmd.line, "$modelname", "a model path", path))
        return false;
    std::vector<std::string> out{"$modelname " + Q(path)};
    if (!c.haveArchetype)
        out.push_back("$modelarchetype " + c.archetype);
    Emit(c, cmd.line, LastLine(c), out);
    return true;
}

// $pushd <dir> / $popd -> $addsearchdir. There is no cd stack in .pulseqc, so
// each pushed dir is registered instead - joined through the stack, since a
// nested $pushd is relative to the one below it. $popd cannot unregister one.
bool PushD(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    std::string dir;
    if (!Want(c, cmd.line, "$pushd", "a directory", dir))
        return false;
    const fs::path p(dir);
    const fs::path joined = (p.is_absolute() || c.cdStack.empty() ? p : c.cdStack.back() / p)
                                .lexically_normal();
    c.cdStack.push_back(joined);
    Emit(c, cmd.line, LastLine(c), {"$addsearchdir " + Q(joined.generic_string())});
    return true;
}

void PopD(Conv& c) {
    const Tok cmd = c.toks[c.pos++];
    std::vector<std::string> out;
    if (c.cdStack.empty())
        out.push_back("// importqc: $popd with nothing pushed");
    else
        c.cdStack.pop_back();
    Emit(c, cmd.line, cmd.line, out);
}

// $opaque / $mostlyopaque -> $renderpass. The render pass is one value, so a
// second flag is dropped rather than written over the first.
void RenderPass(Conv& c, const std::string& pass) {
    const Tok cmd = c.toks[c.pos++];
    std::vector<std::string> out;
    if (!c.renderPass)
        out.push_back("$renderpass " + pass);
    c.renderPass = true;
    Emit(c, cmd.line, cmd.line, out);
}

// What the prop flags and an existing $modelarchetype say, before any line is
// rewritten - $modelname is usually written above both.
void ScanFlags(Conv& c) {
    for (const Tok& t : c.toks) {
        if (t.quoted)
            continue;
        const std::string o = Lower(t.text);
        if (o == "$staticprop")
            c.archetype = "static";
        else if (o == "$simpleprop")
            c.archetype = "simple";
        else if (o == "$modelarchetype")
            c.haveArchetype = true;
        else if (o == "$modelname")
            c.haveModelName = true;
    }
}

// A whole file of $model body options, which is what a .qci $included from
// inside a body holds. Each option is replaced by the top-level commands it
// hoists, in place.
bool RunBody(Conv& c) {
    Body b;
    while (More(c)) {
        const Tok t = c.toks[c.pos++];
        const size_t was = b.hoisted.size();
        if (!BodyOption(c, t, c.file, b))
            return false;
        Emit(c, t.line, LastLine(c),
             {b.hoisted.begin() + static_cast<std::ptrdiff_t>(was), b.hoisted.end()});
    }
    // a VTA flex list belongs to the $rendermesh of the $model that included
    // this file, and nothing here can reach across into it
    if (!b.vtaFlexes.empty())
        c.repl[0].insert(0, "// importqc: this file's flex list is a $rendermesh $vta block - move "
                            "it onto the mesh by hand\n");
    return true;
}

bool Run(Conv& c) {
    ScanFlags(c);
    while (More(c)) {
        const Tok& t = c.toks[c.pos];
        const std::string o = t.quoted ? std::string() : Lower(t.text);
        bool ok = true;
        if (o == "$bodygroup")
            ok = Bodygroup(c);
        else if (o == "$bodygrouppreset")
            ok = BodygroupPreset(c);
        else if (o == "$model")
            ok = Model(c, /*bodied=*/true);
        else if (o == "$body")
            ok = Model(c, /*bodied=*/false);
        else if (o == "$modelname")
            ok = ModelName(c);
        else if (o == "$texturegroup")
            ok = TextureGroup(c);
        else if (o == "$collisionmodel" || o == "$collisionjoints")
            ok = CollisionModel(c);
        else if (o == "$opaque" || o == "$mostlyopaque")
            RenderPass(c, o.substr(1));
        else if (o == "$include")
            ok = Include(c);
        else if (o == "$pushd")
            ok = PushD(c);
        else if (o == "$popd")
            PopD(c);
        else if (o == "$origin")
            ok = Origin(c);
        else if (o == "$scale")
            ok = Scale(c);
        else if (o == "$hboxset" || o == "$hitboxset")
            HboxSet(c);
        else if (o == "$hbox")
            ok = Hbox(c);
        else if (o == "$skinnedlods") {
            // dropped: .pulseqc always takes a LOD mesh's own weights
            Emit(c, t.line, t.line, {});
            c.pos++;
        }
        else if (o == "$staticprop" || o == "$simpleprop") {
            // normally the $modelarchetype under $modelname - but a .qci has no
            // $modelname to sit under, so there it is written in place
            std::vector<std::string> out;
            if (!c.haveModelName && !c.haveArchetype) {
                out.push_back("$modelarchetype " + c.archetype);
                c.haveArchetype = true;
            }
            Emit(c, t.line, t.line, out);
            c.pos++;
        }
        else
            c.pos++; // not ours: the line passes through untouched
        if (!ok)
            return false;
    }
    FlushHboxSet(c);
    return true;
}

bool ConvertFile(const fs::path& in, const fs::path& out, bool bodyMode,
                 std::set<std::string>& seen, std::string* err) {
    std::ifstream f(in, std::ios::binary);
    if (!f) {
        if (err) *err = "cannot open \"" + in.string() + "\"";
        return false;
    }
    std::ostringstream buf;
    buf << f.rdbuf();
    std::string text = buf.str();
    // the loader only tolerates a UTF-8 BOM at offset 0, and the header line
    // below would push this one into the middle of the file
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF)
        text.erase(0, 3);

    Conv c;
    c.file = in.string();
    c.dir = in.parent_path();
    c.seen = &seen;
    if (!Tokenize(text, c.file, c.toks, err))
        return false;

    std::istringstream lines(text);
    for (std::string line; std::getline(lines, line);) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        c.src.push_back(line);
    }
    c.repl.resize(c.src.size() + 1);
    c.drop.assign(c.src.size() + 1, false);

    if (!(bodyMode ? RunBody(c) : Run(c))) {
        if (err) *err = c.err;
        return false;
    }

    // the whole rendermeshlist goes above the first group, not beside the group
    // that happens to use it
    if (c.firstSlot > 0 && !c.meshDecls.empty()) {
        std::string block;
        for (const std::string& s : c.meshDecls)
            block += s + "\n";
        c.repl[c.firstSlot - 1].insert(0, block + "\n");
    }

    std::ofstream o(out);
    if (!o) {
        if (err) *err = "cannot write \"" + out.string() + "\"";
        return false;
    }
    o << "// importqc version " << PULSEMODEL_VERSION << "\n";
    o << "// converted from " << in.filename().string() << "\n\n";
    for (size_t i = 0; i < c.src.size(); ++i) {
        o << c.repl[i];
        if (!c.drop[i])
            o << c.src[i] << "\n";
    }
    if (!o) {
        if (err) *err = "write failed for \"" + out.string() + "\"";
        return false;
    }
    std::printf("importqc: rewrote %d command%s -> %s\n", c.commands, c.commands == 1 ? "" : "s",
                out.string().c_str());
    return true;
}

} // namespace

std::string DefaultOutput(const std::string& in) { return ConvertedPath(in).string(); }

bool Convert(const std::string& in, const std::string& out, std::string* err) {
    std::error_code ec;
    const fs::path canon = fs::weakly_canonical(in, ec);
    std::set<std::string> seen{Lower((ec ? fs::path(in) : canon).string())};
    return ConvertFile(in, out, /*bodyMode=*/false, seen, err);
}

} // namespace pulse::importqc
