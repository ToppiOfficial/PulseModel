// qcloader.cpp - keyvalues1 compile-script loader. See qcloader.h.

#include "qcloader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "dmxloader.h"
#include "dmxrig.h"
#include "facemarkup.h"
#include "flexreg.h"
#include "meshedit.h"
#include "fbxloader.h"
#include "smdloader.h"

namespace pulse::loader {

namespace cm = pulse::compile;
namespace lim = pulse::limits;
namespace pm = pulse::math;
namespace fs = std::filesystem;

namespace {

// animation/sequence flags (format/mdl.h, mirrored like pulseloader.cpp does)
constexpr int kStudioLooping = 0x0001;
constexpr int kStudioSnap = 0x0002;
constexpr int kStudioDelta = 0x0004;
constexpr int kStudioAutoplay = 0x0008;
constexpr int kStudioPost = 0x0010;
constexpr int kStudioRealtime = 0x0100;
constexpr int kStudioLocal = 0x0200;
constexpr int kStudioHidden = 0x0400;
constexpr int kStudioOverride = 0x0800;
constexpr int kStudioCyclePose = 0x0080;
constexpr int kStudioWorld = 0x4000;
constexpr int kStudioNoForceLoop = 0x8000;
constexpr int kStudioWorldAndRelative = 0x20000;
constexpr int kStudioRootXform = 0x40000;
// BSP contents flags, the subset $contents names (mirrored likewise)
constexpr int kContentsSolid = 0x1;
constexpr int kContentsGrate = 0x8;
constexpr int kContentsMonster = 0x2000000;
constexpr int kContentsDebris = 0x4000000;
constexpr int kContentsLadder = 0x20000000;
// autolayer flags (STUDIO_AL_*)
constexpr int kStudioAlSpline = 0x0040;
constexpr int kStudioAlXfade = 0x0080;
constexpr int kStudioAlNoblend = 0x0200;
constexpr int kStudioAlLocal = 0x1000;
constexpr int kStudioAlPose = 0x4000;

// ---------------------------------------------------------------------------
// Tokenizer
// ---------------------------------------------------------------------------

struct Token {
    std::string text;
    int line = 0;
    bool quoted = false; // came from "..." - so a quoted "loop" is a filename
    // the text came out of a $name$ expansion (a variable or a macro
    // parameter). Only conditions care: an expanded operand is a literal value,
    // never the name of another variable to look up.
    bool expanded = false;
};

// keyvalues1 lexing: whitespace-separated words, "quoted strings", `//` line
// comments, and braces as standalone tokens even when jammed against a word.
bool Tokenize(const std::string& s, const std::string& file,
              std::vector<Token>& out, std::string* err) {
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
        if (c == '"') {
            Token t;
            t.line = line;
            t.quoted = true;
            i++;
            while (i < s.size() && s[i] != '"') {
                if (s[i] == '\n')
                    line++;
                t.text.push_back(s[i++]);
            }
            if (i >= s.size()) {
                if (err) *err = file + "(" + std::to_string(t.line) +
                                "): unterminated string";
                return false;
            }
            i++; // closing quote
            out.push_back(std::move(t));
            continue;
        }
        if (c == '{' || c == '}') {
            out.push_back({std::string(1, c), line, false});
            i++;
            continue;
        }
        Token t;
        t.line = line;
        while (i < s.size()) {
            const char d = s[i];
            if (std::isspace(static_cast<unsigned char>(d)) || d == '{' || d == '}' ||
                d == '"' || (d == '/' && i + 1 < s.size() && s[i + 1] == '/'))
                break;
            t.text.push_back(d);
            i++;
        }
        out.push_back(std::move(t));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Parse context
// ---------------------------------------------------------------------------

struct Ctx {
    cm::CompileInput& in;
    fs::path scriptDir; // root script's directory; every source path is relative to it
    std::string file;
    std::vector<Token> toks;
    size_t pos = 0;
    std::string* err = nullptr;
    fs::path curDir; // dir of the file currently being parsed ($include is relative to it)
    std::vector<std::string> includeStack; // cycle guard
    std::vector<fs::path> includeDirs; // $addincludesearchdir
    std::vector<fs::path> searchDirs;  // $addsearchdir, for source files - never mixed with includeDirs
    std::map<std::string, source::Source*> rendermeshes; // $rendermesh name -> loaded source
    // filename -> shared source. A bodied $rendermesh is excluded: its edits must not leak to other refs.
    std::map<std::string, source::Source*> sourceCache;
    source::MaterialTable physMats; // collision-only materials, kept out of the model's texture table
    std::map<std::string, int> namedAnims; // $animation name -> index into in.anims
    std::map<std::string, std::vector<Token>> cmdlists; // $cmdlist name -> body tokens
    std::map<std::string, std::string> variables; // $definevariable, case-sensitive, shared across $include
    struct Macro {
        std::vector<std::string> params;
        std::vector<Token> body;
    };
    std::map<std::string, Macro> macros; // keyed lowercased
    int macroExpansions = 0; // runaway-recursion guard
    std::vector<std::string> lockedVars; // -defvar names; locked against $definevariable
    bool scriptBreak = false; // $break, sticky through every $include parent

    float defaultFps = 30.0f;
    float defaultFadeIn = 0.2f;
    float defaultFadeOut = 0.2f;
    bool lcaseSequences = false;
    std::vector<std::string> allowedActivities; // $allowactivityname, exact-case
    bool unlockDefineBones = false;

    // $attachment, held raw: its matrix needs the model's final default rotation,
    // not known until every source and $transformmodel angle has been seen
    struct PendingAttachment {
        std::string name;
        std::string bone; // empty = model space, anchored to root bone
        pm::Vector3 origin{};
        pm::Vector3 anglesDeg{};
        bool hasAngles = false;
        int type = 0;  // kAttachIs* bits
        int flags = 0; // kAttachFlag* bits, written to disk
        bool filled = true; // false = $declareattachment slot awaiting its definition
        bool noscale = false; // $illumposition: origin taken raw, not scaled like a vertex
        // vertex selectors: origin becomes an offset from their averaged position
        std::vector<std::string> flexgroups;
        std::vector<std::string> flexmorphs;
        std::vector<std::string> materials;
    };
    std::vector<PendingAttachment> attachments;

    // top-level $wrinklescale: merged into every $rendermesh loaded after it
    std::vector<source::WrinkleScaleOption> wrinkleScales;

    // $flexcontroller/$flexlocalvar/$flexrule/$flexcorrective, held until the
    // bodygroup list is final - registration order can't run command by command
    ManualFlex manual;

    // $eyeball/$mouth/$eyelid, held for the same reason; one list across all
    // three because their relative order is load-bearing (indices, flexdescs)
    FaceMarkup face;

    bool Eof() const { return pos >= toks.size(); }
    const Token& Cur() const { return toks[pos]; }

    // True when -defvar owns this variable name (case-sensitive, like the
    // variable table itself).
    bool IsLockedVar(const std::string& name) const {
        for (const std::string& v : lockedVars)
            if (v == name)
                return true;
        return false;
    }

    bool Fail(int line, const std::string& msg) {
        if (err) *err = file + "(" + std::to_string(line) + "): " + msg;
        return false;
    }

    // Next token, or null at end of file.
    const Token* Next() { return Eof() ? nullptr : &toks[pos++]; }

    // True when the next token starts a new command (or the file ended) - the
    // terminator for an unbraced option list.
    bool AtCommand() const { return Eof() || (!Cur().quoted && Cur().text[0] == '$'); }

    // A required value token. Rejects a following $command so a missing
    // argument reports itself instead of eating the next line.
    bool Want(const char* what, const Token& cmd, std::string& value) {
        if (AtCommand())
            return Fail(cmd.line, cmd.text + " expects " + what);
        value = toks[pos++].text;
        return true;
    }

    bool WantFloat(const char* what, const Token& cmd, float& value) {
        std::string s;
        if (!Want(what, cmd, s))
            return false;
        try {
            size_t used = 0;
            const float v = std::stof(s, &used);
            if (used != s.size())
                throw std::invalid_argument("trailing characters");
            value = v;
        } catch (const std::exception&) {
            return Fail(cmd.line, cmd.text + " expects " + what + ", got \"" + s + "\"");
        }
        return true;
    }

    bool WantInt(const char* what, const Token& cmd, int& value) {
        std::string s;
        if (!Want(what, cmd, s))
            return false;
        try {
            size_t used = 0;
            const int v = std::stoi(s, &used);
            if (used != s.size())
                throw std::invalid_argument("trailing characters");
            value = v;
        } catch (const std::exception&) {
            return Fail(cmd.line, cmd.text + " expects " + what + ", got \"" + s + "\"");
        }
        return true;
    }

    // Like WantInt but accepts studiomdl's "." placeholder as -1 (the ikrule
    // `range` frame markers).
    bool WantFrame(const char* what, const Token& cmd, int& value) {
        std::string s;
        if (!Want(what, cmd, s))
            return false;
        if (s == ".") { value = -1; return true; }
        try {
            size_t used = 0;
            const int v = std::stoi(s, &used);
            if (used != s.size())
                throw std::invalid_argument("trailing characters");
            value = v;
        } catch (const std::exception&) {
            return Fail(cmd.line, cmd.text + " expects " + what + ", got \"" + s + "\"");
        }
        return true;
    }

    // True when the next token parses cleanly as a base-10 integer - used for
    // studiomdl's optional trailing activity weight.
    bool NextIsInt() const {
        if (Eof() || Cur().quoted) return false;
        const std::string& s = Cur().text;
        try {
            size_t used = 0;
            (void)std::stoi(s, &used);
            return used == s.size();
        } catch (const std::exception&) {
            return false;
        }
    }
};

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

// "\n    <path>" per candidate, for a "looked in:" error
std::string LookedIn(const std::vector<fs::path>& tried) {
    std::string s;
    for (const fs::path& p : tried)
        s += "\n    " + p.string();
    return s;
}

// Where a source file (.dmx/.smd/.vrd) actually is: the ROOT script's
// directory first, then each $addsearchdir dir in registration order (stock
// searches cddir then g_addSearchDirs, joining the name as written - no
// basename flattening, unlike $include). Empty return = nowhere, and `tried`
// then holds every candidate for the error message.
fs::path FindSourceFile(const Ctx& c, const std::string& filename,
                        std::vector<fs::path>* tried) {
    const fs::path rel(filename);
    std::vector<fs::path> cands;
    if (rel.is_absolute()) {
        cands.push_back(rel);
    } else {
        cands.push_back(c.scriptDir / rel);
        for (const fs::path& dir : c.searchDirs)
            cands.push_back(dir / rel);
    }
    for (fs::path& p : cands) {
        p = p.lexically_normal().make_preferred();
        std::error_code ec;
        if (fs::is_regular_file(p, ec))
            return p;
    }
    if (tried) *tried = std::move(cands);
    return {};
}

// editors on Windows add one unasked
void StripUtf8Bom(std::string& text) {
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF)
        text.erase(0, 3);
}

std::string Lower(std::string s) {
    for (char& ch : s)
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return s;
}

std::string StripExtension(const std::string& s) {
    const size_t dot = s.find_last_of('.');
    const size_t slash = s.find_last_of("/\\");
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
        return s.substr(0, dot);
    return s;
}

bool HasExtension(const std::string& s) {
    const size_t dot = s.find_last_of('.');
    const size_t slash = s.find_last_of("/\\");
    return dot != std::string::npos && (slash == std::string::npos || dot > slash);
}

// $datamodeljoints/$datamodelflexes read the datamodel directly, so their
// extensionless reference is always .dmx - never probed for an .smd.
std::string WithDmxExtension(const std::string& s) {
    return HasExtension(s) ? s : s + ".dmx";
}

// keyvalues1 scripts conventionally drop the source extension. Resolve it by
// probing the search dirs in preference order - .dmx wins over a .smd or .fbx
// of the same name. Nothing found falls back to .dmx so the not-found error
// names the primary format.
std::string WithSourceExtension(const Ctx& c, const std::string& s) {
    if (HasExtension(s))
        return s;
    for (const char* ext : {".dmx", ".smd", ".fbx"})
        if (!FindSourceFile(c, s + ext, nullptr).empty())
            return s + ext;
    return s + ".dmx";
}

// legacy SMD is picked purely by extension (.smd/.sma/.phys, matching the
// reference studiomdl loader table); everything else routes through the DMX
// reader. An extensionless reference got its extension from
// WithSourceExtension.
bool IsSmdPath(const std::string& s) {
    auto ends = [&](const char* ext) {
        const size_t n = std::strlen(ext);
        if (s.size() < n) return false;
        return _stricmp(s.c_str() + s.size() - n, ext) == 0;
    };
    return ends(".smd") || ends(".sma") || ends(".phys");
}

bool IsFbxPath(const std::string& s) {
    return s.size() >= 4 && _stricmp(s.c_str() + s.size() - 4, ".fbx") == 0;
}

// The edits one $rendermesh { } body asks for (meshedit.h).
struct MeshEdit {
    std::string name; // the $rendermesh alias, for the load print
    source::MeshFilter filter;
    source::SkinnedBoneCull boneCull = source::SkinnedBoneCull::None;
    bool noMorph = false;
    std::string vta; // $vta: a legacy morph file for an SMD mesh
    int vtaLine = 0;
    std::vector<source::VtaFlexOption> vtaFlexes;
    std::string vca, vcaName; // $vca: the same file played back as one NWAY run
    int vcaLine = 0;
};

// Load-or-reuse a source file, keyed by filename. `edit` marks the load as a
// render mesh, always PRIVATE (never cached, never reused) since edits must
// not leak to another reference and a render mesh is never an animation
// source. `kind` (source::LoadKind) gates both how much gets read and what a
// cached entry may satisfy - a cache hit needs kind >= what's requested, and
// the store only ever raises a key's kind.
source::Source* LoadSource(Ctx& c, const std::string& filename, int line,
                           bool morphSource = false, MeshEdit* edit = nullptr,
                           source::LoadKind kind = source::LoadKind::Model) {
    const bool priv = edit != nullptr;
    const bool animOnly = kind == source::LoadKind::Animation;
    // a collision hull is never drawn, so keep its materials out of the model's
    // texture table (the indices still have to exist - Source groups faces by
    // material - they just index a scratch table nothing reads)
    source::MaterialTable& mats =
        kind == source::LoadKind::Collision ? c.physMats : c.in.mats;

    const std::string key = Lower(filename);
    if (!priv) {
        auto it = c.sourceCache.find(key);
        if (it != c.sourceCache.end() && it->second->kind >= kind)
            return it->second;
    }

    std::vector<fs::path> tried;
    const fs::path full = FindSourceFile(c, filename, &tried);
    if (full.empty()) {
        c.Fail(line, "cannot find \"" + filename + "\" - looked in:" + LookedIn(tried));
        return nullptr;
    }
    std::string loadErr;

    // one line per file actually read - a cache reuse above prints nothing.
    // a render mesh names its alias and prints the basename only (the same file
    // loads once per $rendermesh); errors keep the full path. A DMX prints after
    // the read so the line can carry its "<format> <ver>, <encoding> <ver>".
    const std::string head =
        priv ? "loading rendermesh \"" + edit->name + "\": " + full.filename().string()
             : std::string("loading ") +
                   (kind == source::LoadKind::Animation     ? "animation"
                    : kind == source::LoadKind::Collision   ? "collision"
                                                            : "model") +
                   ": " + full.string();

    auto src = std::make_unique<source::Source>();
    // the name AS WRITTEN, not where it was found - it is the cache key above,
    // and the SMD path derives cdtextures from it
    src->filename = filename;
    src->kind = kind;

    if (IsSmdPath(filename)) {
        std::printf("%s\n", head.c_str());
        // SMD triangles are grouped by material, not by named mesh - there is
        // nothing for an $exceptionlist to name
        if (edit && !edit->filter.empty()) {
            c.Fail(line, "$exceptionlist needs a DMX source - an SMD has no named meshes");
            return nullptr;
        }
        if (!source::LoadSmdSource(full.string(), *src, mats, c.in.scale, &loadErr,
                                   morphSource, animOnly)) {
            c.Fail(line, "cannot load \"" + full.string() + "\": " + loadErr);
            return nullptr;
        }
    } else if (IsFbxPath(filename)) {
        std::printf("%s\n", head.c_str());
        if (!source::LoadFbxSource(full.string(), *src, mats, c.in.scale, &loadErr, morphSource,
                                   edit ? &edit->filter : nullptr, animOnly)) {
            c.Fail(line, "cannot load \"" + full.string() + "\": " + loadErr);
            return nullptr;
        }
        // an inclusive filter naming a mesh the file does not have would silently
        // drop the geometry it was written to keep
        if (edit) {
            if (const std::string* miss = edit->filter.Unmatched()) {
                c.Fail(line, "$exceptionlist names \"" + *miss + "\", which is not a mesh in \"" +
                                 filename + "\"");
                return nullptr;
            }
        }
    } else {
        auto dm = pulse::dmx::Datamodel::Load(full.string().c_str(), &loadErr);
        if (!dm) {
            c.Fail(line, "cannot load \"" + full.string() + "\": " + loadErr);
            return nullptr;
        }
        std::printf("%s (%s %d, %s %d)\n", head.c_str(), dm->format.c_str(), dm->format_version,
                    dm->encoding.c_str(), dm->encoding_version);
        if (!source::LoadDmxSource(*dm, *src, mats, c.in.scale, &loadErr, morphSource,
                                   edit ? &edit->filter : nullptr, animOnly)) {
            c.Fail(line, "cannot parse \"" + full.string() + "\": " + loadErr);
            return nullptr;
        }
        // an inclusive filter naming a mesh the file does not have would silently
        // drop the geometry it was written to keep
        if (edit) {
            if (const std::string* miss = edit->filter.Unmatched()) {
                c.Fail(line, "$exceptionlist names \"" + *miss + "\", which is not a mesh in \"" +
                                 filename + "\"");
                return nullptr;
            }
        }
    }

    if (edit)
        source::CullUnskinnedBones(*src, edit->boneCull);

    c.in.sources.push_back(std::move(src));
    source::Source* loaded = c.in.sources.back().get();
    if (!priv) {
        source::Source*& cached = c.sourceCache[key];
        if (!cached || cached->kind < kind) // raise only - see the cache rule above
            cached = loaded;
    }
    return loaded;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

bool CmdModelName(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a model name", cmd, name))
        return false;
    c.in.outname = StripExtension(name);
    if (c.in.outname.empty())
        return c.Fail(cmd.line, "$modelname is empty");
    return true;
}

// $wrinklescale <morph> <scale> - bake wrinkle onto that morph wherever it
// appears. Applied once every source is loaded, so it is position-free and a
// mesh that does not have the morph is skipped.
bool CmdWrinkleScale(Ctx& c, const Token& cmd) {
    source::WrinkleScaleOption w;
    w.line = cmd.line;
    if (!c.Want("a morph name", cmd, w.shape) || !c.WantFloat("a wrinkle scale", cmd, w.scale))
        return false;
    c.wrinkleScales.push_back(std::move(w));
    return true;
}

// $rendermesh <name> "file.dmx" [{ ... }] - the rendermeshlist entry. Bones,
// geometry and the raw morph deltas, nothing else: the flex rig behind the deltas
// comes from $datamodelflexes and the DME bone markup from $datamodeljoints.
//
// The optional body edits what comes out of the file, as if the model had been
// re-exported for this one entry (meshedit.h). Every $rendermesh loads its own
// Source whether or not it has a body - see LoadSource.
bool CmdRenderMesh(Ctx& c, const Token& cmd) {
    std::string name, file;
    if (!c.Want("a name", cmd, name) || !c.Want("a source filename", cmd, file))
        return false;
    if (c.rendermeshes.count(name))
        return c.Fail(cmd.line, "duplicate $rendermesh \"" + name + "\"");

    MeshEdit edit;
    edit.name = name;
    if (!c.Eof() && !c.Cur().quoted && c.Cur().text == "{") {
        ++c.pos;
        const std::string where = "$rendermesh \"" + name + "\"";
        for (;;) {
            const Token* t = c.Next();
            if (!t)
                return c.Fail(cmd.line, where + " is missing '}'");
            if (t->text == "}")
                break;
            const std::string o = Lower(t->text);

            if (o == "$exceptionlist") {
                if (!edit.filter.empty())
                    return c.Fail(t->line, where + ": $exceptionlist written twice");
                const Token* n = c.Next();
                if (!n)
                    return c.Fail(t->line, "$exceptionlist expects '{'");
                if (!n->quoted && Lower(n->text) == "exclusive") {
                    edit.filter.exclusive = true;
                    n = c.Next();
                } else if (!n->quoted && Lower(n->text) == "inclusive") {
                    n = c.Next();
                }
                if (!n || n->text != "{")
                    return c.Fail(t->line, "$exceptionlist expects exclusive, inclusive or '{'");
                for (;;) {
                    const Token* m = c.Next();
                    if (!m)
                        return c.Fail(t->line, "$exceptionlist is missing '}'");
                    if (m->text == "}")
                        break;
                    edit.filter.names.push_back({m->text, false});
                }
                if (edit.filter.names.empty())
                    return c.Fail(t->line, "$exceptionlist is empty");
                continue;
            }

            if (o == "$skinnedbonecull") {
                const Token* n = c.Next();
                if (!n)
                    return c.Fail(t->line, "$skinnedbonecull expects aggressive or tree");
                const std::string m = Lower(n->text);
                if (m == "aggressive")
                    edit.boneCull = source::SkinnedBoneCull::Aggressive;
                else if (m == "tree")
                    edit.boneCull = source::SkinnedBoneCull::Tree;
                else
                    return c.Fail(n->line, "$skinnedbonecull expects aggressive or tree, got \"" +
                                               n->text + "\"");
                continue;
            }

            if (o == "$nomorph" || o == "$nofacial") {
                edit.noMorph = true;
                continue;
            }

            // $vta "file.vta" { flex "<name>" frame <N> [position <f>] [decay <f>] ... }
            // SMD-only: a DMX carries its delta states itself. A .vta names
            // nothing (what follows `#` on a `time` line is a comment), so each
            // morph is named here and picked by frame index - see smdloader.h.
            if (o == "$vta") {
                if (!edit.vta.empty())
                    return c.Fail(t->line, where + ": $vta written twice");
                if (!c.Want("a .vta filename", *t, edit.vta))
                    return false;
                edit.vtaLine = t->line;
                if (c.Eof() || c.Cur().quoted || c.Cur().text != "{")
                    return c.Fail(t->line, where + ": $vta expects '{' - list the frames to import");
                ++c.pos;
                for (;;) {
                    const Token* m = c.Next();
                    if (!m)
                        return c.Fail(t->line, where + ": $vta is missing '}'");
                    if (m->text == "}")
                        break;
                    if (m->quoted || Lower(m->text) != "flex")
                        return c.Fail(m->line,
                                      "$vta: expected flex or '}', got \"" + m->text + "\"");
                    source::VtaFlexOption fo;
                    bool sawFrame = false;
                    if (!c.Want("a flex name", *m, fo.name))
                        return false;
                    while (!c.Eof() && !c.Cur().quoted) {
                        const std::string k = Lower(c.Cur().text);
                        if (k == "frame") {
                            ++c.pos;
                            if (!c.WantInt("a frame index", *m, fo.frame))
                                return false;
                            sawFrame = true;
                        } else if (k == "position" || k == "decay") {
                            ++c.pos;
                            if (!c.WantFloat(k == "position" ? "a position" : "a decay", *m,
                                             k == "position" ? fo.position : fo.decay))
                                return false;
                        } else {
                            break;
                        }
                    }
                    if (!sawFrame)
                        return c.Fail(m->line, "$vta: flex \"" + fo.name +
                                                   "\" needs a frame <N> - a .vta names nothing");
                    edit.vtaFlexes.push_back(std::move(fo));
                }
                if (edit.vtaFlexes.empty())
                    return c.Fail(t->line, where + ": $vta is empty - it imports nothing");
                continue;
            }

            // $vca "file.vca" [<controller name>] - vertex cache animation. Same
            // file format as $vta; the frames are a playback sequence, not named
            // shapes, so nothing is listed. The controller name defaults to the
            // file's basename (reference Option_VertexCacheAnimationFile).
            if (o == "$vca") {
                if (!edit.vca.empty())
                    return c.Fail(t->line, where + ": $vca written twice");
                if (!c.Want("a .vca filename", *t, edit.vca))
                    return false;
                edit.vcaLine = t->line;
                if (!c.AtCommand() && !(!c.Cur().quoted && c.Cur().text == "}"))
                    edit.vcaName = c.Next()->text;
                if (edit.vcaName.empty())
                    edit.vcaName = StripExtension(fs::path(edit.vca).filename().string());
                continue;
            }

            return c.Fail(t->line, where + ": expected $exceptionlist, $skinnedbonecull, "
                                           "$nomorph, $vta, $vca or '}', got \"" + t->text + "\"");
        }
    }

    // Always passes the edit, even an empty one: that is what marks the load as a
    // render mesh, and a render mesh always gets its own Source. $nomorph is the
    // load gate itself - with no morph parse there are no delta shapes, flexkeys
    // or balance data to drop later.
    source::Source* src = LoadSource(c, WithSourceExtension(c, file), cmd.line,
                                     /*morphSource=*/!edit.noMorph, &edit);
    if (!src)
        return false;

    // $vta / $vca: both read a VTA-format file against the mesh just loaded, and
    // both are SMD-only - a DMX mesh carries its own delta states.
    if (!edit.vta.empty() || !edit.vca.empty()) {
        const std::string where = "$rendermesh \"" + name + "\"";
        const bool isVca = !edit.vca.empty();
        const std::string& file = isVca ? edit.vca : edit.vta;
        const char* cmdName = isVca ? "$vca" : "$vta";
        const int line = isVca ? edit.vcaLine : edit.vtaLine;

        if (!edit.vta.empty() && !edit.vca.empty())
            return c.Fail(edit.vcaLine,
                          where + ": $vca must be the only flex source, but $vta is also set");
        if (edit.noMorph)
            return c.Fail(line, where + ": " + cmdName + " and $nomorph contradict each other");
        if (!IsSmdPath(src->filename))
            return c.Fail(line, where + ": " + cmdName +
                                    " is SMD-only - a DMX mesh carries its own delta states");
        std::vector<fs::path> tried;
        const fs::path full = FindSourceFile(c, file, &tried);
        if (full.empty())
            return c.Fail(line, "cannot find \"" + file + "\" - looked in:" + LookedIn(tried));
        std::string loadErr;
        const bool ok =
            isVca ? source::LoadVcaMorphs(full.string(), *src, c.in.scale, edit.vcaName, &loadErr)
                  : source::LoadVtaMorphs(full.string(), *src, c.in.scale, edit.vtaFlexes, &loadErr);
        if (!ok)
            return c.Fail(line, "cannot load \"" + full.string() + "\": " + loadErr);
    }

    c.rendermeshes[name] = src;
    return true;
}

// Fuse the render meshes one `mesh` line names into a single drawable Source,
// registered like any other so the LOD commands can name it by `studio`.
source::Source* MergeRenderMeshes(Ctx& c, const std::vector<source::Source*>& refs,
                                  const std::string& studio, int line) {
    auto merged = std::make_unique<source::Source>();
    merged->filename = studio;
    std::string mergeErr;
    if (!source::MergeSources(refs, *merged, &mergeErr)) {
        c.Fail(line, "mesh \"" + studio + "\": " + mergeErr);
        return nullptr;
    }
    c.in.sources.push_back(std::move(merged));
    return c.in.sources.back().get();
}

// $modelgroup <name> { mesh [<display>] <rendermesh>... [name <display>] | blank ... }
// $modelgroup <name> <rendermesh>... [name <display>]
//
// The second form is the shorthand for a single, non-swappable bodypart: one
// choice, drawn from the meshes on the line, named after the group by default.
//
// `studio` is an accepted spelling of `mesh`, for $bodygroup muscle memory.
//
// One `mesh` line is one bodygroup choice. Listing several $rendermesh names
// draws them as ONE model, so a script mixes and matches meshes it already has
// instead of exporting a combined file per combination. The choice's studio name
// - what SFM shows - is `name <display>` (anywhere on the line) or a leading
// word that is not a $rendermesh, else the render-mesh names joined with '_'
// (compile::ChoiceName). A display name that collides with a $rendermesh name
// has to use `name`.
bool CmdModelGroup(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a name", cmd, name))
        return false;
    // duplicates are ambiguous for $modelgrouppreset and for a decompile, so
    // they are a hard error rather than two groups the tools pick between
    for (const auto& b : c.in.bodyparts)
        if (b.name == name)
            return c.Fail(cmd.line, "$modelgroup \"" + name + "\" already exists");
    const std::string where = "$modelgroup \"" + name + "\"";
    cm::CompileInput::InBodyPart part;
    part.name = name;

    // One choice: the $rendermesh names to the end of `line`, plus the optional
    // `name <display>`. `defName` is the display name when the line gives none.
    auto meshLine = [&](int line, const std::vector<std::string>& defName) -> bool {
        std::string studio;
        bool named = false;

        // the render meshes this choice draws, to the end of the line - the
        // next `mesh`/`studio`/`blank`/'}' still ends the list, so an old
        // one-per-word line keeps parsing the way it always did
        std::vector<source::Source*> refs;
        std::vector<std::string> refNames;
        while (!c.Eof() && c.Cur().line == line) {
            const Token& r = c.Cur();
            // a one-choice group with nothing to draw is meaningless, so `blank`
            // is an error inline instead of the terminator it is in the block
            if (!r.quoted && !defName.empty() && _stricmp(r.text.c_str(), "blank") == 0)
                return c.Fail(r.line, where + ": blank needs the { } form - an inline "
                                              "$modelgroup is a single choice");
            if (!r.quoted && (r.text == "}" || _stricmp(r.text.c_str(), "mesh") == 0 ||
                              _stricmp(r.text.c_str(), "studio") == 0 ||
                              _stricmp(r.text.c_str(), "blank") == 0))
                break;
            ++c.pos;
            // `name <display>`, anywhere on the line
            if (!r.quoted && _stricmp(r.text.c_str(), "name") == 0) {
                if (named)
                    return c.Fail(r.line, where + ": mesh is named twice");
                if (c.Eof() || c.Cur().line != line ||
                    (!c.Cur().quoted && c.Cur().text == "}"))
                    return c.Fail(r.line, where + ": mesh name expects a display name");
                if (!c.Want("a display name", r, studio))
                    return false;
                named = true;
                continue;
            }
            auto it = c.rendermeshes.find(r.text);
            if (it == c.rendermeshes.end()) {
                // a leading word that is not a $rendermesh is the display
                // name; a name that collides with one needs `name`
                if (!named && refs.empty() && defName.empty()) {
                    studio = r.text;
                    named = true;
                    continue;
                }
                return c.Fail(r.line,
                              where + " references unknown rendermesh \"" + r.text + "\"");
            }
            if (std::find(refs.begin(), refs.end(), it->second) != refs.end())
                return c.Fail(r.line, where + ": mesh \"" + r.text + "\" is listed twice");
            refs.push_back(it->second);
            refNames.push_back(r.text);
        }
        if (refs.empty())
            return c.Fail(line, where + ": mesh expects at least one $rendermesh name");

        cm::CompileInput::InModel model;
        model.name = named ? studio
                           : cm::ChoiceName(defName.empty() ? refNames : defName);
        if (model.name.empty())
            return c.Fail(line, where + ": mesh name is empty");
        // mstudiomodel_t::name is a 64-byte inline field, so the writer cuts
        // anything longer - say so here, where the line number is known
        if (model.name.size() > 63)
            std::fprintf(stderr,
                         "warning: %s line %d: model name \"%s\" is %zu chars, "
                         "truncated to \"%.63s\" - use `name <display>` to shorten it\n",
                         c.file.c_str(), line, model.name.c_str(), model.name.size(),
                         model.name.c_str());
        // one render mesh is drawn as it loaded; several are fused into a
        // private Source of their own (meshedit.h)
        model.source = refs.size() == 1 ? refs[0]
                                        : MergeRenderMeshes(c, refs, model.name, line);
        if (!model.source)
            return false;
        part.models.push_back(std::move(model));
        return true;
    };

    const Token* brace = c.Next();
    if (!brace)
        return c.Fail(cmd.line, "$modelgroup expects '{' or a $rendermesh name after the name");
    if (brace->text != "{") {
        // inline form - one choice on the command's own line, named after the group
        --c.pos;
        if (!meshLine(brace->line, {name}))
            return false;
        c.in.bodyparts.push_back(std::move(part));
        return true;
    }

    for (;;) {
        const Token* t = c.Next();
        if (!t)
            return c.Fail(cmd.line, where + " is missing '}'");
        if (t->text == "}")
            break;

        if (!t->quoted && _stricmp(t->text.c_str(), "blank") == 0) {
            cm::CompileInput::InModel model;
            model.name = "blank";
            model.source = nullptr;
            part.models.push_back(std::move(model));
            continue;
        }
        if (!t->quoted && (_stricmp(t->text.c_str(), "mesh") == 0 ||
                           _stricmp(t->text.c_str(), "studio") == 0)) {
            if (!meshLine(t->line, {}))
                return false;
            continue;
        }
        return c.Fail(t->line,
                      where + ": expected mesh, studio, blank or '}', got \"" + t->text + "\"");
    }
    c.in.bodyparts.push_back(std::move(part));
    return true;
}

// $modelgrouppreset <name> { <modelgroup> <choice index> ... }
//
// A named body value the engine can switch to (Cmd_BodygroupPreset). The
// group names are resolved in Compile, not here, so the presets may sit above
// the $modelgroup blocks they name.
bool CmdModelGroupPreset(Ctx& c, const Token& cmd) {
    cm::CompileInput::InBodyGroupPreset preset;
    if (!c.Want("a name", cmd, preset.name))
        return false;
    for (const auto& p : c.in.bodygrouppresets)
        if (p.name == preset.name)
            return c.Fail(cmd.line, "$modelgrouppreset \"" + preset.name + "\" already exists");

    const Token* brace = c.Next();
    if (!brace || brace->text != "{")
        return c.Fail(cmd.line, "$modelgrouppreset expects '{' after the name");

    const std::string where = "$modelgrouppreset \"" + preset.name + "\"";
    for (;;) {
        const Token* t = c.Next();
        if (!t)
            return c.Fail(cmd.line, where + " is missing '}'");
        if (!t->quoted && t->text == "}")
            break;
        int choice = 0;
        if (!c.WantInt("a choice index", cmd, choice))
            return false;
        preset.choices.emplace_back(t->text, choice);
    }
    if (preset.choices.empty())
        return c.Fail(cmd.line, where + ": expects at least one $modelgroup name and index");
    c.in.bodygrouppresets.push_back(std::move(preset));
    return true;
}

// ---------------------------------------------------------------------------
// $datamodeljoints / $datamodelflexes: import rig markup / flex rig from a DMX
// (may be a different file than the mesh - borrows another character's rig).
// Neither builds skeleton or geometry; an imported entry with no matching bone
// is dropped like a hand-written one, EXCEPT a hitbox, which is still a hard
// $hitboxset error. Empty import list = import everything. First definition
// wins on duplicates, except hitbox sets: any set already declared makes the
// whole imported list a no-op (model-global, all-or-nothing).
// ---------------------------------------------------------------------------

// Comma-join the categories a $datamodel* actually imports, for the load print.
std::string RigImportList(std::initializer_list<std::pair<bool, const char*>> opts) {
    std::string s;
    for (const std::pair<bool, const char*>& o : opts) {
        if (!o.first)
            continue;
        if (!s.empty())
            s += ", ";
        s += o.second;
    }
    return s;
}

// Locate a rig file for a $datamodel* command. Empty = not found (already
// reported).
fs::path FindRigFile(Ctx& c, const Token& cmd, const std::string& filename) {
    std::vector<fs::path> tried;
    const fs::path full = FindSourceFile(c, filename, &tried);
    if (full.empty())
        c.Fail(cmd.line, "cannot find \"" + filename + "\" - looked in:" + LookedIn(tried));
    return full;
}

// Open a DMX for its rig alone. It never becomes a Source, so nothing in it
// reaches the skeleton, the bodygroups or the material table.
std::unique_ptr<pulse::dmx::Datamodel> LoadRigDmx(Ctx& c, const Token& cmd,
                                                  const std::string& filename,
                                                  const std::string& what) {
    const fs::path full = FindRigFile(c, cmd, filename);
    if (full.empty())
        return nullptr;
    std::string loadErr;
    auto dm = pulse::dmx::Datamodel::Load(full.string().c_str(), &loadErr);
    if (!dm) {
        c.Fail(cmd.line, "cannot load \"" + full.string() + "\": " + loadErr);
        return nullptr;
    }
    std::printf("loading %s: %s (%s %d, %s %d)\n", what.c_str(), full.filename().string().c_str(),
                dm->format.c_str(), dm->format_version, dm->encoding.c_str(),
                dm->encoding_version);
    return dm;
}

// $datamodeljoints <file> [jigglebones] [proceduralbones] [hitboxes]
//                        [attachments]
bool CmdDataModelJoints(Ctx& c, const Token& cmd) {
    std::string file;
    if (!c.Want("a source filename", cmd, file))
        return false;

    bool jiggle = false, procedural = false, hitboxes = false, attachments = false;
    while (!c.AtCommand()) {
        const Token t = c.toks[c.pos++];
        const std::string o = Lower(t.text);
        if (o == "jigglebones")
            jiggle = true;
        else if (o == "proceduralbones")
            procedural = true;
        else if (o == "hitboxes")
            hitboxes = true;
        else if (o == "attachments")
            attachments = true;
        else
            return c.Fail(t.line, "$datamodeljoints: expected jigglebones, proceduralbones, "
                                  "hitboxes or attachments, got \"" + t.text + "\"");
    }
    if (!jiggle && !procedural && !hitboxes && !attachments)
        jiggle = procedural = hitboxes = attachments = true; // no list = the whole rig

    const std::string resolved = WithDmxExtension(file);
    const std::string what = "joints (" + RigImportList({{jiggle, "jigglebones"},
                                                         {procedural, "proceduralbones"},
                                                         {hitboxes, "hitboxes"},
                                                         {attachments, "attachments"}}) + ")";
    std::string err;
    auto dm = LoadRigDmx(c, cmd, resolved, what);
    if (!dm)
        return false;
    if (!LoadDmxJoints(*dm, c.in, jiggle, procedural, hitboxes, attachments, &err))
        return c.Fail(cmd.line, "$datamodeljoints \"" + file + "\": " + err);
    return true;
}

// Merge one loaded rig into the script-wide one. Raw control indices are
// rebased onto the merged control list; a control or remap a PREVIOUS
// $datamodelflexes already contributed is reused rather than duplicated, while
// this file's own list is taken as it stands (its indices are its own).
void MergeFlexRig(source::FlexRig& dst, const source::FlexRig& src, bool controllers,
                  bool correctives, bool dominators, bool rules) {
    std::vector<int> rebase(src.controls.size(), -1);
    if (controllers) {
        const size_t oldControls = dst.controls.size();
        const size_t oldRemaps = dst.remaps.size();
        for (size_t i = 0; i < src.controls.size(); ++i) {
            int found = -1;
            for (size_t j = 0; j < oldControls; ++j) {
                if (_stricmp(dst.controls[j].name.c_str(), src.controls[i].name.c_str()) == 0) {
                    found = static_cast<int>(j);
                    break;
                }
            }
            if (found < 0) {
                dst.controls.push_back(src.controls[i]);
                found = static_cast<int>(dst.controls.size()) - 1;
            }
            rebase[i] = found;
        }
        for (const source::ControllerRemap& r : src.remaps) {
            bool dup = false;
            for (size_t j = 0; j < oldRemaps; ++j) {
                if (_stricmp(dst.remaps[j].name.c_str(), r.name.c_str()) == 0) {
                    dup = true;
                    break;
                }
            }
            if (!dup)
                dst.remaps.push_back(r);
        }
    }

    if (correctives) {
        for (const source::FlexRig::Corrective& cor : src.correctives) {
            source::FlexRig::Corrective out;
            out.delta = cor.delta;
            for (int k : cor.combination)
                out.combination.push_back(rebase[k]);
            if (dominators) {
                for (const std::vector<int>& dom : cor.dominators) {
                    std::vector<int> mapped;
                    for (int k : dom)
                        mapped.push_back(rebase[k]);
                    out.dominators.push_back(std::move(mapped));
                }
            }
            dst.correctives.push_back(std::move(out));
        }
    }

    if (rules) {
        dst.hasRules = dst.hasRules || src.hasRules;
        for (const source::SrcFlexRule& r : src.rules)
            dst.rules.push_back(r);
    }
}

// $datamodelflexes <file> [flexcontroller] [flexcorrective] [flexdominator]
//                        [flexrule]
bool CmdDataModelFlexes(Ctx& c, const Token& cmd) {
    std::string file;
    if (!c.Want("a source filename", cmd, file))
        return false;

    bool controllers = false, correctives = false, dominators = false, rules = false;
    while (!c.AtCommand()) {
        const Token t = c.toks[c.pos++];
        const std::string o = Lower(t.text);
        if (o == "flexcontroller")
            controllers = true;
        else if (o == "flexcorrective")
            correctives = true;
        else if (o == "flexdominator")
            dominators = true;
        else if (o == "flexrule")
            rules = true;
        else
            return c.Fail(t.line, "$datamodelflexes: expected flexcontroller, flexcorrective, "
                                  "flexdominator or flexrule, got \"" + t.text + "\"");
    }
    if (!controllers && !correctives && !dominators && !rules)
        controllers = correctives = dominators = rules = true; // no list = the whole rig

    // A corrective FETCHes the controllers it combines and a dominator is only
    // ever a tail on a corrective's rule, so neither stands alone.
    if (correctives && !controllers)
        return c.Fail(cmd.line, "$datamodelflexes: flexcorrective needs flexcontroller - a "
                                "corrective rule fetches the controllers it combines");
    if (dominators && !correctives)
        return c.Fail(cmd.line, "$datamodelflexes: flexdominator needs flexcorrective - a "
                                "domination is a tail on a corrective's rule");

    const std::string resolved = WithDmxExtension(file);
    const std::string what = "flexes (" + RigImportList({{controllers, "flexcontroller"},
                                                         {correctives, "flexcorrective"},
                                                         {dominators, "flexdominator"},
                                                         {rules, "flexrule"}}) + ")";
    source::FlexRig rig;
    std::string err;
    auto dm = LoadRigDmx(c, cmd, resolved, what);
    if (!dm)
        return false;
    if (!source::LoadDmxFlexRig(*dm, rig, &err))
        return c.Fail(cmd.line, "$datamodelflexes \"" + file + "\": " + err);
    MergeFlexRig(c.manual.datamodel, rig, controllers, correctives, dominators, rules);
    return true;
}

// ---------------------------------------------------------------------------
// $animation / $sequence
//
// Unlike the rest of .pulseqc, these two keep studiomdl's exact authoring
// shape: $animation declares a named clip, $sequence
// references clips by name - or names a file inline, loaded as an implied
// "@<seq>" animation - and carries the playback options. Every option that maps
// to a field the compile backend already consumes is wired up; a real studiomdl
// option with no backend yet is a hard error that names itself, so a script
// never silently loses behavior. The unsupported options are staged follow-on
// work, not permanent gaps.
// ---------------------------------------------------------------------------

// ACT_* activity shorthand ("ACT_IDLE 1"), matched without a case-sensitive
// prefix compare so it stays portable off MSVC.
bool HasActPrefix(const std::string& o) {
    return o.size() >= 4 && (o[0] == 'A' || o[0] == 'a') &&
           (o[1] == 'C' || o[1] == 'c') && (o[2] == 'T' || o[2] == 't') &&
           o[3] == '_';
}


const char* UnsupportedAnimOption(const std::string& o) {
    if (_stricmp(o.c_str(), "if") == 0)
        return "use $if / $switch instead";
    return nullptr;
}

int LookupControl(const std::string& s); // defined below, used by the align family
bool ParseIkRule(Ctx& c, const Token& cmd, std::vector<cm::CompileInput::InIkRule>& out);

// Apply one animation-option token to `a` (studiomdl ParseAnimationToken, the
// backend-supported subset). The option keyword must already be consumed - `t`
// is that token; any arguments are read from the stream here. Returns:
//    1 - consumed as an option
//    0 - not an animation option (caller treats it as a filename / seq ref)
//   -1 - error, c.err set (includes a known-but-unsupported option)
int ApplyAnimOption(Ctx& c, const Token& t, cm::CompileInput::InAnim& a) {
    if (t.quoted)
        return 0; // a quoted token is always a filename
    const std::string& o = t.text;

    if (_stricmp(o.c_str(), "fps") == 0) {
        if (!c.WantFloat("a frame rate", t, a.fps))
            return -1;
        if (a.fps <= 0.0f) {
            c.Fail(t.line, "fps must be > 0");
            return -1;
        }
        return 1;
    }
    if (_stricmp(o.c_str(), "loop") == 0) { a.flags |= kStudioLooping; return 1; }
    if (_stricmp(o.c_str(), "snap") == 0) { a.flags |= kStudioSnap; return 1; }
    if (_stricmp(o.c_str(), "reverse") == 0) {
        a.cmds.push_back({cm::CompileInput::InAnim::InCmd::Reverse});
        return 1;
    }
    if (_stricmp(o.c_str(), "post") == 0) { a.flags |= kStudioPost; return 1; }
    if (_stricmp(o.c_str(), "noforceloop") == 0) {
        a.flags |= kStudioNoForceLoop;
        return 1;
    }
    if (_stricmp(o.c_str(), "ignorescale") == 0) { a.ignorescale = true; return 1; }
    // noautoik/autoik: the compile stage auto-adds an IK_RELEASE rule for every
    // chain this animation moves but never references; noautoik opts out.
    if (_stricmp(o.c_str(), "ikrule") == 0) return ParseIkRule(c, t, a.ikrules) ? 1 : -1;
    // ikfixup: same syntax as ikrule, but the correction is baked into this
    // clip's keyframes here instead of shipped as engine-side error deltas.
    if (_stricmp(o.c_str(), "ikfixup") == 0) {
        std::vector<cm::CompileInput::InIkRule> one;
        if (!ParseIkRule(c, t, one))
            return -1;
        if (_stricmp(one[0].type.c_str(), "footstep") != 0) {
            c.Fail(t.line, "ikfixup: only the footstep type bakes a correction "
                           "(got \"" + one[0].type + "\") - use ikrule instead");
            return -1;
        }
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::IkFixup;
        cmd.ikfixup = std::move(one[0]);
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    if (_stricmp(o.c_str(), "noautoik") == 0) { a.noAutoIK = true; return 1; }
    if (_stricmp(o.c_str(), "autoik") == 0) { a.noAutoIK = false; return 1; }
    // nocull: keep this animation whatever $animationcullmethod says, both as
    // unreferenced and as a duplicate of another one
    if (_stricmp(o.c_str(), "nocull") == 0) { a.nocull = true; return 1; }
    // ignoretransformbone <angles|position>: roll back that category of
    // $transformbone bind-pose edits while converting this clip
    if (_stricmp(o.c_str(), "ignoretransformbone") == 0) {
        std::string which;
        if (!c.Want("angles or position", t, which))
            return -1;
        if (_stricmp(which.c_str(), "angles") == 0) {
            a.ignoreTransformAngles = true;
        } else if (_stricmp(which.c_str(), "position") == 0) {
            a.ignoreTransformPosition = true;
        } else {
            c.Fail(t.line, "ignoretransformbone: expected angles or position, got \"" +
                               which + "\"");
            return -1;
        }
        return 1;
    }

    // demand loading, all no-ops without $animblocksize
    if (_stricmp(o.c_str(), "noanimblock") == 0) { a.disableAnimblocks = true; return 1; }
    if (_stricmp(o.c_str(), "noanimblockstall") == 0) {
        a.isFirstSectionLocal = true;
        return 1;
    }
    if (_stricmp(o.c_str(), "nostallframes") == 0) {
        if (!c.WantFloat("a frame count", t, a.numNostallFrames))
            return -1;
        return 1;
    }

    // loop fixups, all three of which imply the animation loops
    if (_stricmp(o.c_str(), "fudgeloop") == 0) {
        a.fudgeloop = true;
        a.flags |= kStudioLooping;
        return 1;
    }
    if (_stricmp(o.c_str(), "startloop") == 0) {
        if (a.looprestartpercent != 0.0f) {
            c.Fail(t.line, "startloop: percentstartloop is already set");
            return -1;
        }
        if (!c.WantInt("a start frame", t, a.looprestart))
            return -1;
        a.flags |= kStudioLooping;
        return 1;
    }
    if (_stricmp(o.c_str(), "percentstartloop") == 0) {
        if (a.looprestart != 0) {
            c.Fail(t.line, "percentstartloop: startloop is already set");
            return -1;
        }
        if (!c.WantFloat("a percentage", t, a.looprestartpercent))
            return -1;
        a.flags |= kStudioLooping;
        return 1;
    }

    // clip trim. `framestart` sets only the start, leaving the end at the
    // source clip's (ParseAnimationToken's bUseDefaultEndFrame); both ends are
    // clamped to the clip in the compile stage.
    // blockname <name>: sample a specific clip out of a multi-clip source. The
    // reference also resets the frame range to that clip's, which the compile
    // stage does by clamping to whichever clip is picked.
    if (_stricmp(o.c_str(), "blockname") == 0) {
        if (!c.Want("a source clip name", t, a.blockname))
            return -1;
        return 1;
    }
    if (_stricmp(o.c_str(), "framestart") == 0) {
        if (!c.WantInt("a start frame", t, a.startframe))
            return -1;
        return 1;
    }
    if (_stricmp(o.c_str(), "frame") == 0) {
        if (!c.WantInt("a start frame", t, a.startframe) ||
            !c.WantInt("an end frame", t, a.endframe))
            return -1;
        if (a.endframe < a.startframe) {
            c.Fail(t.line, "end frame before start frame");
            return -1;
        }
        return 1;
    }
    // numframes <n> (CMD_NUMFRAMES): force the clip to n frames, padding with
    // copies of the last frame or dropping the tail. Runs in command order.
    if (_stricmp(o.c_str(), "numframes") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::NumFrames;
        if (!c.WantInt("a frame count", t, cmd.numframes))
            return -1;
        if (cmd.numframes < 1 || cmd.numframes > lim::kMaxAnimFrames) {
            c.Fail(t.line, "numframes must be 1.." + std::to_string(lim::kMaxAnimFrames));
            return -1;
        }
        a.cmds.push_back(std::move(cmd));
        return 1;
    }

    // per-animation transform. These replace the model-wide origin/rotation/
    // scale for this animation only.
    if (_stricmp(o.c_str(), "origin") == 0) {
        if (!c.WantFloat("an X offset", t, a.adjust.x) ||
            !c.WantFloat("a Y offset", t, a.adjust.y) ||
            !c.WantFloat("a Z offset", t, a.adjust.z))
            return -1;
        a.adjustSet = true;
        return 1;
    }
    if (_stricmp(o.c_str(), "rotate") == 0) {
        // z only, and the +90 composes exactly like $transformmodel angles
        float deg = 0.0f;
        if (!c.WantFloat("a rotation in degrees", t, deg))
            return -1;
        a.rotation = pm::RadianEuler{0.0f, 0.0f, (deg + 90.0f) * pm::kDeg2Rad};
        a.rotationSet = true;
        return 1;
    }
    if (_stricmp(o.c_str(), "angles") == 0) {
        float p = 0.0f, y = 0.0f, r = 0.0f;
        if (!c.WantFloat("a pitch", t, p) || !c.WantFloat("a yaw", t, y) ||
            !c.WantFloat("a roll", t, r))
            return -1;
        a.rotation = pm::RadianEuler{p * pm::kDeg2Rad, y * pm::kDeg2Rad,
                                     (r + 90.0f) * pm::kDeg2Rad};
        a.rotationSet = true;
        return 1;
    }
    if (_stricmp(o.c_str(), "scale") == 0) {
        if (!c.WantFloat("a scale", t, a.scale))
            return -1;
        return 1;
    }
    if (_stricmp(o.c_str(), "weightlist") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::Weights;
        if (!c.Want("a weightlist name", t, cmd.name))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    if (_stricmp(o.c_str(), "subtract") == 0 || _stricmp(o.c_str(), "presubtract") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::Subtract;
        cmd.presubtract = (_stricmp(o.c_str(), "presubtract") == 0);
        if (!c.Want("an animation name", t, cmd.name) ||
            !c.WantInt("a frame", t, cmd.frame))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // fixuploop <start> <end>  (CMD_FIXUP): spread the difference between the
    // overlapping first/last frames over <start> frames back from the end and
    // <end> frames in from the start. `start` is negative.
    if (_stricmp(o.c_str(), "fixuploop") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::FixupLoop;
        if (!c.WantInt("a start frame", t, cmd.fixupStart) ||
            !c.WantInt("an end frame", t, cmd.fixupEnd))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // walkframe <frame> <controls...>              (CMD_MOTION)
    // walkalignto <frame> <anim> <srcframe> <controls...> (CMD_REFMOTION)
    // Extract the root bone's travel up to <frame> into a movement key.
    if (_stricmp(o.c_str(), "walkframe") == 0 ||
        _stricmp(o.c_str(), "walkalignto") == 0) {
        const bool ref = (_stricmp(o.c_str(), "walkalignto") == 0);
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = ref ? cm::CompileInput::InAnim::InCmd::RefMotion
                       : cm::CompileInput::InAnim::InCmd::Motion;
        if (!c.WantInt("an end frame", t, cmd.motionEndFrame))
            return -1;
        cmd.srcframe = cmd.motionEndFrame;
        if (ref) {
            if (!c.Want("an animation name", t, cmd.name) ||
                !c.WantInt("a reference frame", t, cmd.frame))
                return -1;
        }
        for (;;) {
            if (c.AtCommand() || c.Eof() || c.Cur().quoted)
                break;
            const int ctrl = LookupControl(c.Cur().text);
            if (ctrl == -1)
                break;
            cmd.motiontype |= ctrl;
            c.pos++;
        }
        if (cmd.motiontype == 0) {
            c.Fail(t.line, "\"" + o + "\": missing motion controls");
            return -1;
        }
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // motionrollback <seconds>: how far before the last frame whole-animation
    // motion extraction stops, so a non-looping clip does not pop
    // Inert unless the clip also has a motion control.
    if (_stricmp(o.c_str(), "motionrollback") == 0) {
        if (!c.WantFloat("a rollback in seconds", t, a.motionrollback))
            return -1;
        return 1;
    }
    // a bare motion control on an $animation sets the clip's motiontype
    // (ParseAnimationToken's lookupControl fallthrough)
    {
        const int ctrl = LookupControl(o);
        if (ctrl != -1) {
            a.motiontype |= ctrl;
            return 1;
        }
    }
    // match <anim>  (CMD_MATCH): rotate every frame so this clip's first frame
    // lines up with the reference animation's first frame.
    if (_stricmp(o.c_str(), "match") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::Match;
        if (!c.Want("an animation name", t, cmd.name))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // matchblend <anim> <srcframe> <destframe> <pre> <post> (CMD_MATCHBLEND):
    // like `match`, but the correction is only applied around <destframe>,
    // ramped in over <pre> frames before and <post> frames after.
    if (_stricmp(o.c_str(), "matchblend") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::MatchBlend;
        if (!c.Want("an animation name", t, cmd.name) ||
            !c.WantInt("a source frame", t, cmd.srcframe) ||
            !c.WantInt("a destination frame", t, cmd.destframe) ||
            !c.WantInt("a pre-blend frame count", t, cmd.matchPre) ||
            !c.WantInt("a post-blend frame count", t, cmd.matchPost))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // transformbone <bone> [origin x y z] [angles p y r]: offset one bone across
    // every frame of the clip, relative to its parent (world for a root bone).
    // Not a stock studiomdl option. Both clauses are optional and independent,
    // so `transformbone <bone>` alone is a no-op. They must sit on the command's
    // own line - `origin` and `angles` are per-animation options in their own
    // right, and a greedy scan would swallow the next line's.
    if (_stricmp(o.c_str(), "transformbone") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::TransformBone;
        cm::AnimBoneTransform& bt = cmd.xform;
        if (!c.Want("a bone name", t, bt.bone))
            return -1;
        for (;;) {
            if (c.AtCommand() || c.Eof() || c.Cur().quoted || c.Cur().line != t.line)
                break;
            const std::string n = c.Cur().text;
            if (_stricmp(n.c_str(), "origin") == 0) {
                if (bt.originSet) {
                    c.Fail(c.Cur().line, "transformbone: origin written twice");
                    return -1;
                }
                c.pos++;
                if (!c.WantFloat("an X offset", t, bt.origin.x) ||
                    !c.WantFloat("a Y offset", t, bt.origin.y) ||
                    !c.WantFloat("a Z offset", t, bt.origin.z))
                    return -1;
                bt.originSet = true;
            } else if (_stricmp(n.c_str(), "angles") == 0) {
                if (bt.anglesSet) {
                    c.Fail(c.Cur().line, "transformbone: angles written twice");
                    return -1;
                }
                c.pos++;
                float p = 0.0f, y = 0.0f, r = 0.0f;
                if (!c.WantFloat("a pitch", t, p) || !c.WantFloat("a yaw", t, y) ||
                    !c.WantFloat("a roll", t, r))
                    return -1;
                // plain pitch/yaw/roll, like $definebone - no rootxform +90
                bt.angles = pm::RadianEuler{p * pm::kDeg2Rad, y * pm::kDeg2Rad,
                                            r * pm::kDeg2Rad};
                bt.anglesSet = true;
            } else {
                break;
            }
        }
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // copypose <anim|seq|bindpose> <srcframe> <start> <end> [fadein <f>]
    // [fadeout <f>] [position|rotation]: hold one frame of another clip across a
    // frame range of this one. Not a stock studiomdl option. The trailing
    // clauses are optional and may be written in any order; naming neither
    // channel copies both. The source pose is taken exactly as it stands - a
    // `scale` or `ignorescale` mismatch between the two clips is copied through,
    // not compensated for.
    // `bindpose` is a reserved source: the compile's own bind pose, read as data
    // rather than as a clip, so no other command can have edited it first.
    if (_stricmp(o.c_str(), "copypose") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::CopyPose;
        if (!c.Want("an animation or sequence name", t, cmd.name) ||
            !c.WantInt("a source frame", t, cmd.srcframe) ||
            !c.WantInt("a start frame", t, cmd.destframe) ||
            !c.WantInt("an end frame", t, cmd.copyEndFrame))
            return -1;
        if (_stricmp(cmd.name.c_str(), "bindpose") == 0) {
            cmd.copyBindPose = true;
            cmd.name.clear();
        }
        if (cmd.srcframe < 0) {
            c.Fail(t.line, "copypose: source frame must be >= 0");
            return -1;
        }
        // the bind pose is one pose, so any frame but 0 is a mistake rather
        // than a clamp
        if (cmd.copyBindPose && cmd.srcframe != 0) {
            c.Fail(t.line, "copypose: bindpose has only frame 0");
            return -1;
        }
        if (cmd.copyEndFrame < cmd.destframe) {
            c.Fail(t.line, "copypose: end frame before start frame");
            return -1;
        }
        // The optional clauses must sit on the command's own line: `fadein` and
        // `fadeout` are $sequence options too, so a greedy scan would silently
        // eat the next line's `fadeout 0.2` as a fade width in frames.
        bool pos = false, rot = false, fin = false, fout = false;
        for (;;) {
            if (c.AtCommand() || c.Eof() || c.Cur().quoted || c.Cur().line != t.line)
                break;
            const std::string n = c.Cur().text;
            if (_stricmp(n.c_str(), "fadein") == 0) {
                if (fin) {
                    c.Fail(c.Cur().line, "copypose: fadein written twice");
                    return -1;
                }
                c.pos++;
                if (!c.WantFloat("a fade-in length in frames", t, cmd.fadeIn))
                    return -1;
                fin = true;
            } else if (_stricmp(n.c_str(), "fadeout") == 0) {
                if (fout) {
                    c.Fail(c.Cur().line, "copypose: fadeout written twice");
                    return -1;
                }
                c.pos++;
                if (!c.WantFloat("a fade-out length in frames", t, cmd.fadeOut))
                    return -1;
                fout = true;
            } else if (_stricmp(n.c_str(), "position") == 0) {
                c.pos++;
                pos = true;
            } else if (_stricmp(n.c_str(), "rotation") == 0) {
                c.pos++;
                rot = true;
            } else {
                break;
            }
        }
        if (cmd.fadeIn < 0.0f || cmd.fadeOut < 0.0f) {
            c.Fail(t.line, "copypose: fade lengths must be >= 0");
            return -1;
        }
        cmd.copyPos = pos || !rot;
        cmd.copyRot = rot || !pos;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // worldspaceblend <anim>              (CMD_WORLDSPACEBLEND)
    // worldspaceblendloop <anim> <frame>
    // Blend this clip toward the reference in WORLD space, per bone, weighted by
    // the weightlist. The loop form samples the reference by cycle from <frame>
    // instead of holding one frame.
    if (_stricmp(o.c_str(), "worldspaceblend") == 0 ||
        _stricmp(o.c_str(), "worldspaceblendloop") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::WorldspaceBlend;
        cmd.worldLoops = (_stricmp(o.c_str(), "worldspaceblendloop") == 0);
        if (!c.Want("an animation name", t, cmd.name))
            return -1;
        if (cmd.worldLoops && !c.WantInt("a start frame", t, cmd.srcframe))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // align family (CMD_AO): move this clip so one bone lands where the
    // reference animation has it.
    //   alignto    <anim>                          - root bone, X|Y, frame 0/0
    //   align      <anim> <controls...> <sf> <df>
    //   alignboneto <bone> <anim>
    //   alignbone   <bone> <anim> <controls...> <sf> <df>
    if (_stricmp(o.c_str(), "alignto") == 0 || _stricmp(o.c_str(), "align") == 0 ||
        _stricmp(o.c_str(), "alignboneto") == 0 ||
        _stricmp(o.c_str(), "alignbone") == 0) {
        const bool named = (_stricmp(o.c_str(), "alignboneto") == 0 ||
                            _stricmp(o.c_str(), "alignbone") == 0);
        const bool wholeClip = (_stricmp(o.c_str(), "alignto") == 0 ||
                                _stricmp(o.c_str(), "alignboneto") == 0);
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::Align;
        if (named && !c.Want("a bone name", t, cmd.alignBone))
            return -1;
        if (!c.Want("an animation name", t, cmd.name))
            return -1;
        if (wholeClip) {
            cmd.motiontype = 0x0001 | 0x0002; // STUDIO_X | STUDIO_Y
        } else {
            // motion controls run until a token that is not one; the next two
            // are then the source and destination frames
            for (;;) {
                if (c.AtCommand() || c.Eof() || c.Cur().quoted)
                    break;
                const int ctrl = LookupControl(c.Cur().text);
                if (ctrl == -1)
                    break;
                cmd.motiontype |= ctrl;
                c.pos++;
            }
            if (cmd.motiontype == 0) {
                c.Fail(t.line, "\"" + o + "\": missing motion controls "
                               "(expected X/Y/Z/XR/YR/ZR)");
                return -1;
            }
            if (!c.WantInt("a source frame", t, cmd.srcframe) ||
                !c.WantInt("a destination frame", t, cmd.destframe))
                return -1;
        }
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // appendanim <anim>  (CMD_APPENDANIM): tack another clip's frames onto the
    // end of this one.
    if (_stricmp(o.c_str(), "appendanim") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::AppendAnim;
        if (!c.Want("an animation name", t, cmd.name))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // bonedriver <bone> [axis x|y|z] [value <f>] [range <s> <p> <t> <e>]
    // (CMD_BONEDRIVER): overwrite one position axis of a bone. Without `range`
    // every frame gets the value; with it the value is smoothstep-ramped in.
    // This is NOT the procedural-bone $driverbone - it just edits frames.
    if (_stricmp(o.c_str(), "bonedriver") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::BoneDriver;
        if (!c.Want("a bone name", t, cmd.alignBone))
            return -1;
        // each clause is optional, and stock checks them in this fixed order
        if (!c.AtCommand() && !c.Cur().quoted &&
            _stricmp(c.Cur().text.c_str(), "axis") == 0) {
            c.pos++;
            std::string axis;
            if (!c.Want("an axis (x/y/z)", t, axis))
                return -1;
            if (_stricmp(axis.c_str(), "x") == 0) cmd.driverAxis = 0;
            else if (_stricmp(axis.c_str(), "y") == 0) cmd.driverAxis = 1;
            else if (_stricmp(axis.c_str(), "z") == 0) cmd.driverAxis = 2;
            else {
                c.Fail(t.line, "unknown bonedriver axis \"" + axis + "\"");
                return -1;
            }
        }
        if (!c.AtCommand() && !c.Cur().quoted &&
            _stricmp(c.Cur().text.c_str(), "value") == 0) {
            c.pos++;
            if (!c.WantFloat("a value", t, cmd.driverValue))
                return -1;
        }
        if (!c.AtCommand() && !c.Cur().quoted &&
            _stricmp(c.Cur().text.c_str(), "range") == 0) {
            c.pos++;
            cmd.driverAll = false;
            if (!c.WantInt("a start frame", t, cmd.driverStart) ||
                !c.WantInt("a peak frame", t, cmd.driverPeak) ||
                !c.WantInt("a tail frame", t, cmd.driverTail) ||
                !c.WantInt("an end frame", t, cmd.driverEnd))
                return -1;
        }
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // rotateto <degrees>  (CMD_ANGLE): rotate the whole clip about Z so it ends
    // up facing the given yaw.
    if (_stricmp(o.c_str(), "rotateto") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::Angle;
        if (!c.WantFloat("an angle in degrees", t, cmd.angle))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    if (_stricmp(o.c_str(), "cmdlist") == 0) {
        std::string name;
        if (!c.Want("a $cmdlist name", t, name))
            return -1;
        auto it = c.cmdlists.find(name);
        if (it == c.cmdlists.end()) {
            c.Fail(t.line, "unknown cmdlist \"" + name + "\"");
            return -1;
        }
        // splice the stored body in where the option stood - it is then parsed
        // exactly as if written inline, so a cmdlist supports whatever the
        // option parser does and grows with it. $cmdlist rejects nesting, so
        // this cannot expand forever.
        c.toks.insert(c.toks.begin() + c.pos, it->second.begin(), it->second.end());
        return 1;
    }
    if (const char* why = UnsupportedAnimOption(o)) {
        c.Fail(t.line, "\"" + o + "\": " + why);
        return -1;
    }
    return 0;
}

// $cmdlist <name> { <animation options> ... }  (Cmd_Cmdlist). The body is kept
// as raw tokens, not parsed here - `cmdlist <name>` splices it into an
// $animation / $sequence option stream verbatim.
bool CmdCmdList(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a name", cmd, name))
        return false;
    if (c.cmdlists.count(name))
        return c.Fail(cmd.line, "duplicate $cmdlist \"" + name + "\"");

    std::vector<Token> body;
    bool braced = false;
    if (!c.Eof() && !c.Cur().quoted && c.Cur().text == "{") {
        braced = true;
        c.pos++;
    }
    for (;;) {
        if (braced) {
            if (c.Eof())
                return c.Fail(cmd.line, "$cmdlist \"" + name + "\" is missing '}'");
            if (!c.Cur().quoted && c.Cur().text == "}") { c.pos++; break; }
        } else if (c.AtCommand()) {
            break;
        }
        const Token t = c.toks[c.pos++];
        if (!t.quoted && _stricmp(t.text.c_str(), "cmdlist") == 0)
            return c.Fail(t.line, "$cmdlist \"" + name + "\" cannot nest a cmdlist");
        body.push_back(t);
    }
    c.cmdlists[name] = std::move(body);
    return true;
}

// The option body of an $animation: a { } block, or options inline until the
// next $command. Shared with $append / $prepend / $continue, which re-open a
// finished clip and run more options into it (studiomdl ParseAnimation).
bool ParseAnimBody(Ctx& c, const Token& cmd, const std::string& name,
                   cm::CompileInput::InAnim& a) {
    bool braced = false;
    if (!c.Eof() && !c.Cur().quoted && c.Cur().text == "{") {
        braced = true;
        c.pos++;
    }
    for (;;) {
        if (braced) {
            if (c.Eof())
                return c.Fail(cmd.line, cmd.text + " \"" + name + "\" is missing '}'");
            if (!c.Cur().quoted && c.Cur().text == "}") { c.pos++; break; }
        } else if (c.AtCommand()) {
            break;
        }
        const Token t = c.toks[c.pos++];
        const int r = ApplyAnimOption(c, t, a);
        if (r < 0)
            return false;
        if (r == 0)
            return c.Fail(t.line, cmd.text + " \"" + name +
                                  "\": unknown option \"" + t.text + "\"");
    }
    return true;
}

// $animation <name> <file> [options]   (studiomdl Cmd_Animation + ParseAnimation)
// $bindposeanimation <name> [options]: the same command without the file - the
// clip is the compile's own bind pose (loaded skeletons + $definebone), one
// frame unless `numframes` asks for more.
bool CmdAnimationCommon(Ctx& c, const Token& cmd, bool bindpose) {
    std::string name, file;
    if (!c.Want("a name", cmd, name))
        return false;
    if (!bindpose && !c.Want("a source filename", cmd, file))
        return false;
    if (c.namedAnims.count(name))
        return c.Fail(cmd.line, "duplicate animation name \"" + name + "\"");

    cm::CompileInput::InAnim a;
    a.name = name;
    a.fps = c.defaultFps;
    if (bindpose) {
        a.bindpose = true;
        a.endframe = 0; // one frame unless `numframes` says otherwise
    } else {
        a.source = LoadSource(c, WithSourceExtension(c, file), cmd.line, /*morphSource=*/false,
                              /*edit=*/nullptr, source::LoadKind::Animation);
        if (!a.source)
            return false;
    }

    if (!ParseAnimBody(c, cmd, name, a))
        return false;

    c.namedAnims[name] = static_cast<int>(c.in.anims.size());
    c.in.anims.push_back(std::move(a));
    return true;
}

bool CmdAnimation(Ctx& c, const Token& cmd) { return CmdAnimationCommon(c, cmd, false); }
bool CmdBindPoseAnimation(Ctx& c, const Token& cmd) { return CmdAnimationCommon(c, cmd, true); }

// $declareanimation <name>  (studiomdl Cmd_DeclareAnimation): an empty
// STUDIO_OVERRIDE slot, filled in later by an $includemodel's animation. Must
// appear before anything that references it, matching stock file order.
bool CmdDeclareAnimation(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a name", cmd, name))
        return false;
    if (c.namedAnims.count(name))
        return c.Fail(cmd.line, "duplicate animation name \"" + name + "\"");
    cm::CompileInput::InAnim a;
    a.name = name;
    a.isDeclare = true;
    a.flags |= kStudioOverride;
    c.namedAnims[name] = static_cast<int>(c.in.anims.size());
    c.in.anims.push_back(std::move(a));
    return true;
}

// $declaresequence <name>  (studiomdl Cmd_DeclareSequence): empty override slot.
bool CmdDeclareSequence(Ctx& c, const Token& cmd) {
    cm::CompileInput::InSequence seq;
    seq.fadeintime = c.defaultFadeIn;
    seq.fadeouttime = c.defaultFadeOut;
    if (!c.Want("a name", cmd, seq.name))
        return false;
    seq.isDeclare = true;
    seq.flags |= kStudioOverride;
    c.in.sequences.push_back(std::move(seq));
    return true;
}

// $includemodel "path/name.mdl"  (studiomdl Cmd_IncludeModel): borrow an
// already-compiled model's sequences. Stored "models/"-prefixed and nothing is
// read at compile time - the engine links it on load. Suppresses the automatic
// "reference" bind-pose sequence a script with no $sequence would otherwise get.
bool CmdIncludeModel(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a compiled .mdl path", cmd, name))
        return false;
    // must name a .mdl - a source .dmx/.smd or a bare name would compile fine
    // and leave a reference the engine silently fails to resolve
    if (name.size() < 4 || _stricmp(name.c_str() + name.size() - 4, ".mdl") != 0)
        return c.Fail(cmd.line, cmd.text + ": \"" + name + "\" must end in .mdl");
    if (c.in.includeModels.size() >= static_cast<size_t>(lim::kMaxIncludeModels))
        return c.Fail(cmd.line, cmd.text + ": too many entries (max " +
                                std::to_string(lim::kMaxIncludeModels) + ")");
    c.in.includeModels.push_back("models/" + name);
    return true;
}

// ikrule <chain> <type> [type-args] [options]  (studiomdl Option_IKRule subset).
// Fills InIkRule, whose chain/type are resolved later in the compile stage.
bool ParseIkRule(Ctx& c, const Token& cmd, std::vector<cm::CompileInput::InIkRule>& out) {
    cm::CompileInput::InIkRule rule;
    // stock Option_IKRule leaves the ramp frames at calloc's 0 (only `range`
    // or autosteps move them) and sets contact to -1. Override the struct's
    // -1 frame defaults so an unspecified range matches the reference.
    rule.startframe = rule.peakframe = rule.tailframe = rule.endframe = 0;
    rule.contact = -1;
    if (!c.Want("an ik chain name", cmd, rule.chain))
        return false;

    std::string type;
    if (!c.Want("an ik rule type", cmd, type))
        return false;
    if (_stricmp(type.c_str(), "touch") == 0) {
        rule.type = "touch";
        if (!c.Want("a touch bone name", cmd, rule.touchBone))
            return false;
    } else if (_stricmp(type.c_str(), "footstep") == 0) {
        rule.type = "footstep";
    } else if (_stricmp(type.c_str(), "attachment") == 0) {
        rule.type = "attachment";
        if (!c.Want("an attachment name", cmd, rule.attachment))
            return false;
    } else if (_stricmp(type.c_str(), "release") == 0) {
        rule.type = "release";
    } else {
        return c.Fail(cmd.line, "ikrule type \"" + type + "\": not yet supported "
                                "(expected touch/footstep/attachment/release)");
    }

    // trailing options, in any order (Option_IKRule stops at the first token it
    // does not recognize - so does this)
    for (;;) {
        if (c.AtCommand())
            break;
        if (!c.Cur().quoted && c.Cur().text == "}")
            break;
        if (c.Cur().quoted)
            break;
        const std::string o = c.Cur().text;
        if (_stricmp(o.c_str(), "height") == 0) {
            c.pos++;
            if (!c.WantFloat("a height", cmd, rule.height)) return false;
            rule.heightSet = true;
        } else if (_stricmp(o.c_str(), "floor") == 0) {
            c.pos++;
            if (!c.WantFloat("a floor", cmd, rule.floor)) return false;
            rule.floorSet = true;
        } else if (_stricmp(o.c_str(), "pad") == 0) {
            c.pos++;
            float pad = 0.0f;
            if (!c.WantFloat("a pad", cmd, pad)) return false;
            rule.radius = pad / 2.0f;
            rule.radiusSet = true;
        } else if (_stricmp(o.c_str(), "radius") == 0) {
            c.pos++;
            if (!c.WantFloat("a radius", cmd, rule.radius)) return false;
            rule.radiusSet = true;
        } else if (_stricmp(o.c_str(), "contact") == 0) {
            c.pos++;
            if (!c.WantInt("a contact frame", cmd, rule.contact)) return false;
        } else if (_stricmp(o.c_str(), "range") == 0) {
            c.pos++;
            if (!c.WantFrame("a start frame", cmd, rule.startframe) ||
                !c.WantFrame("a peak frame", cmd, rule.peakframe) ||
                !c.WantFrame("a tail frame", cmd, rule.tailframe) ||
                !c.WantFrame("an end frame", cmd, rule.endframe))
                return false;
        } else if (_stricmp(o.c_str(), "usesequence") == 0) {
            c.pos++;
            rule.usesequence = true;
            rule.usesource = false;
        } else if (_stricmp(o.c_str(), "usesource") == 0) {
            c.pos++;
            rule.usesource = true;
            rule.usesequence = false;
        } else if (_stricmp(o.c_str(), "fakeorigin") == 0) {
            c.pos++;
            if (!c.WantFloat("an X offset", cmd, rule.fakeorigin.x) ||
                !c.WantFloat("a Y offset", cmd, rule.fakeorigin.y) ||
                !c.WantFloat("a Z offset", cmd, rule.fakeorigin.z))
                return false;
            rule.fakeoriginSet = true;
        } else if (_stricmp(o.c_str(), "fakerotate") == 0) {
            c.pos++;
            if (!c.WantFloat("a pitch", cmd, rule.fakerotate.x) ||
                !c.WantFloat("a yaw", cmd, rule.fakerotate.y) ||
                !c.WantFloat("a roll", cmd, rule.fakerotate.z))
                return false;
            rule.fakerotateSet = true;
        } else {
            break; // an unrecognized token belongs to the enclosing sequence
        }
    }

    out.push_back(std::move(rule));
    return true;
}

// event <name> <frame> [options]  (studiomdl Option_Event). `<name>` is either
// a numeric id or an event name; `[options]` is the free-form string the game
// reads (a sound name, a particle name, ...).
bool ParseEvent(Ctx& c, const Token& cmd, cm::CompileInput::InSequence& seq) {
    cm::SeqEvent ev;
    if (!c.Want("an event name or id", cmd, ev.name) ||
        !c.WantInt("an event frame", cmd, ev.frame))
        return false;
    if (ev.frame < 0)
        return c.Fail(cmd.line, "event \"" + ev.name + "\" frame must be >= 0");

    // stock reads the options token only when it is still on the event's line
    // (TokenAvailable), so an option-less event does not eat the next line
    if (!c.Eof() && c.Cur().line == cmd.line &&
        !(!c.Cur().quoted && c.Cur().text == "}"))
        ev.options = c.toks[c.pos++].text;
    // options is a char[64] on disk, null included
    if (ev.options.size() > 63)
        return c.Fail(cmd.line, "event \"" + ev.name + "\" options are longer "
                                "than 63 characters");

    seq.events.push_back(std::move(ev));
    if (seq.events.size() > static_cast<size_t>(pulse::limits::kMaxEvents))
        return c.Fail(cmd.line, "too many events in \"" + seq.name + "\"");
    return true;
}

// studiomdl lookupControl. The plain controls name a component to read
// (calcblend, align); the L-prefixed ones are motion-extraction channels, and
// LM/LQ pick how the extracted movement interpolates.
int LookupControl(const std::string& s) {
    static const struct { const char* name; int bit; } kControls[] = {
        {"X", 0x0001}, {"Y", 0x0002}, {"Z", 0x0004},
        {"XR", 0x0008}, {"YR", 0x0010}, {"ZR", 0x0020},
        {"LX", 0x0040}, {"LY", 0x0080}, {"LZ", 0x0100},
        {"LXR", 0x0200}, {"LYR", 0x0400}, {"LZR", 0x0800},
        {"LM", 0x1000}, {"LQ", 0x2000},
    };
    for (const auto& ctrl : kControls)
        if (_stricmp(s.c_str(), ctrl.name) == 0)
            return ctrl.bit;
    return -1;
}

// studiomdl LookupXNode: transition node ids are 1-based and an unknown name
// silently declares a new node.
int LookupXNode(Ctx& c, const std::string& name) {
    for (size_t i = 0; i < c.in.xnodes.size(); i++)
        if (_stricmp(c.in.xnodes[i].c_str(), name.c_str()) == 0)
            return static_cast<int>(i) + 1;
    c.in.xnodes.push_back(name);
    return static_cast<int>(c.in.xnodes.size());
}

// $skiptransition <node> <node> [<node> ...]  (Cmd_Skiptransition): every
// ordered pair of the named nodes gets a direct route, so the graph does not
// route them through an intermediate node.
bool CmdSkipTransition(Ctx& c, const Token& cmd) {
    std::vector<int> list;
    while (!c.AtCommand())
        list.push_back(LookupXNode(c, c.toks[c.pos++].text));
    if (list.empty())
        return c.Fail(cmd.line, "$skiptransition expects at least one node name");
    for (size_t i = 0; i < list.size(); i++)
        for (size_t j = 0; j < list.size(); j++)
            if (list[i] != list[j])
                c.in.xnodeskips.emplace_back(list[i], list[j]);
    return true;
}

// $calctransitions  (Cmd_CalcTransitions): fill in multi-stage routes between
// nodes that have no direct transition.
bool CmdCalcTransitions(Ctx& c, const Token&) {
    c.in.multistageGraph = true;
    return true;
}

// keyvalues { ... }  (studiomdl Option_KeyValues), shared by the $keyvalues
// command and the $sequence option. The block between braces is re-emitted as
// text and copied into the .mdl unchanged; nesting is preserved and tokens
// inside a nested block come back out quoted, exactly like the reference.
bool ParseKeyValues(Ctx& c, const Token& cmd, std::string& out) {
    const Token* brace = c.Next();
    if (!brace || brace->quoted || brace->text != "{")
        return c.Fail(cmd.line, cmd.text + " expects '{'");

    int level = 1;
    for (;;) {
        const Token* t = c.Next();
        if (!t)
            return c.Fail(cmd.line, cmd.text + ": keyvalue block missing matching braces");
        if (!t->quoted && t->text == "}") {
            if (--level <= 0)
                break;
            out += " }\n";
        } else if (!t->quoted && t->text == "{") {
            out += "{\n";
            level++;
        } else if (level > 1) {
            out += "\"" + t->text + "\" ";
        } else {
            out += t->text + " ";
        }
    }
    return true;
}

// $boneflexdriver <bone> <tx|ty|tz> <flexcontroller> <min> <max>
// (Cmd_BoneFlexDriver): a bone's translation along one axis drives a flex
// controller. Both the bone and the controller are find-or-create, so repeating
// the command for the same bone adds another control rather than replacing it.
bool CmdBoneFlexDriver(Ctx& c, const Token& cmd) {
    std::string bone, component, controller;
    if (!c.Want("a bone name", cmd, bone) ||
        !c.Want("a bone component (tx/ty/tz)", cmd, component) ||
        !c.Want("a flex controller name", cmd, controller))
        return false;

    // stock matches on prefix, so "tx"/"ty"/"tz" and anything starting with them
    int nComponent = -1;
    static const char* kComponents[] = {"tx", "ty", "tz"};
    for (int i = 0; i < 3; i++)
        if (component.size() >= 2 && _strnicmp(component.c_str(), kComponents[i], 2) == 0) {
            nComponent = i;
            break;
        }
    if (nComponent == -1)
        return c.Fail(cmd.line, "$boneflexdriver: invalid bone component \"" +
                                component + "\", must be one of tx/ty/tz");

    cm::BoneFlexDriverControl ctrl;
    ctrl.controllername = controller;
    ctrl.component = nComponent;
    if (!c.WantFloat("a min value", cmd, ctrl.min) ||
        !c.WantFloat("a max value", cmd, ctrl.max))
        return false;

    cm::BoneFlexDriver* driver = nullptr;
    for (auto& d : c.in.boneflexdrivers)
        if (_stricmp(d.bonename.c_str(), bone.c_str()) == 0) { driver = &d; break; }
    if (!driver) {
        c.in.boneflexdrivers.emplace_back();
        driver = &c.in.boneflexdrivers.back();
        driver->bonename = bone;
    }
    // find-or-create on the control too: the same controller named twice for one
    // bone updates that control in place
    for (auto& existing : driver->controls)
        if (_stricmp(existing.controllername.c_str(), controller.c_str()) == 0) {
            existing = ctrl;
            return true;
        }
    driver->controls.push_back(std::move(ctrl));
    return true;
}

// $keyvalues { ... }  (Cmd_KeyValues): the model-level block.
bool CmdKeyValues(Ctx& c, const Token& cmd) {
    return ParseKeyValues(c, cmd, c.in.keyvalues);
}

// ---------------------------------------------------------------------------
// Flex / morph. Unlike stock these are TOP-LEVEL and GLOBAL, not $model options
// - there is no $model/$body here. Every mesh a $modelgroup actually uses gets
// the whole rig; a $rendermesh with delta states that no $modelgroup references
// contributes nothing. Everything is stashed on Ctx::manual and registered in
// one pass at the end of the script, since the registration ORDER (and so the
// on-disk table indices) depends on the finished bodygroup list.
// ---------------------------------------------------------------------------

// $flexcontroller <group> [range <min> <max>] <name> [<name>...]
// (Option_Flexcontroller). `range` applies to every name AFTER it, so one line
// can mix ranges. Redefining an existing controller warns and keeps the first.
bool CmdFlexController(Ctx& c, const Token& cmd) {
    std::string group;
    if (!c.Want("a controller group", cmd, group))
        return false;

    float rangeMin = 0.0f;
    float rangeMax = 1.0f;
    int names = 0;
    while (!c.AtCommand()) {
        const Token& t = *c.Next();
        if (!t.quoted && _stricmp(t.text.c_str(), "range") == 0) {
            if (!c.WantFloat("a range min", cmd, rangeMin) ||
                !c.WantFloat("a range max", cmd, rangeMax))
                return false;
            continue;
        }
        ManualFlex::Controller ctrl;
        ctrl.name = t.text;
        ctrl.group = group;
        ctrl.min = rangeMin;
        ctrl.max = rangeMax;
        c.manual.controllers.push_back(std::move(ctrl));
        names++;
    }
    if (names == 0)
        return c.Fail(cmd.line, "$flexcontroller expects at least one controller name after "
                                "the group \"" + group + "\"");
    return true;
}

// $flexlocalvar <name> [<name>...]  (the `localvar` model option): reserves a
// flexdesc that drives no geometry, so $flexrule can share intermediate values.
bool CmdFlexLocalVar(Ctx& c, const Token& cmd) {
    int names = 0;
    while (!c.AtCommand()) {
        ManualFlex::Rule rule;
        rule.name = c.Next()->text;
        rule.localvar = true;
        c.manual.rules.push_back(std::move(rule));
        names++;
    }
    if (names == 0)
        return c.Fail(cmd.line, "$flexlocalvar expects at least one name");
    return true;
}

// $flexrule <result> = <expression>   (the `%rule` model option, Option_Flexrule)
// The expression runs to the next $command. It is re-joined into one string and
// handed to the shunting-yard parser, which does its own tokenizing - so
// "(1 - %x)" and "(1-%x)" parse the same however this file's lexer split them.
bool CmdFlexRule(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a morph name", cmd, name))
        return false;

    std::string eq;
    if (!c.Want("'=' after the morph name", cmd, eq))
        return false;
    if (eq != "=")
        return c.Fail(cmd.line, "$flexrule expects '=' after \"" + name + "\", got \"" + eq +
                                "\" (write it as its own token: $flexrule " + name +
                                " = <expression>)");

    ManualFlex::Rule rule;
    rule.name = name;
    while (!c.AtCommand()) {
        if (!rule.expr.empty())
            rule.expr += ' ';
        rule.expr += c.Next()->text;
    }
    if (rule.expr.empty())
        return c.Fail(cmd.line, "$flexrule \"" + name + "\" has no expression");

    c.manual.rules.push_back(std::move(rule));
    return true;
}

// ---------------------------------------------------------------------------
// Face markup. Top-level and global like the flex commands, and for the same
// reason - there is no $model to hang them on. ORDER IS SIGNIFICANT across all
// three: $eyeball order fixes the index an $eyelid names, and $mouth/$eyelid
// both append to the global flexdesc table. One shared list preserves it.
// ---------------------------------------------------------------------------

// $eyeball <name> bone <b> origin <x y z> material <m> [diameter <d>]
//          [angle <deg>] [pupilscale <s>] [center] (Option_Eyeball). Named
// clauses, unlike QC's positional form. Binds to every body whose mesh uses
// `material`; matching no body at all is a hard error, same as stock.
// `center` puts the eyeball at the bbox center of the material's verts and
// makes `origin` an offset from it, in final (post-$translatemodel, post-90-Z)
// axes - so it is optional there.
bool CmdEyeball(Ctx& c, const Token& cmd) {
    FaceMarkup::Entry entry;
    entry.kind = FaceMarkup::Kind::Eyeball;
    if (!c.Want("an eyeball name", cmd, entry.eyeball.name))
        return false;

    bool haveBone = false, haveMaterial = false, haveOrigin = false;
    while (!c.AtCommand()) {
        const Token& t = *c.Next();
        const std::string o = Lower(t.text);
        if (o == "bone") {
            if (!c.Want("a bone name", cmd, entry.eyeball.bonename)) return false;
            haveBone = true;
        } else if (o == "material") {
            if (!c.Want("a material name", cmd, entry.eyeball.material)) return false;
            haveMaterial = true;
        } else if (o == "origin") {
            if (!c.WantFloat("an origin x", cmd, entry.eyeball.origin.x) ||
                !c.WantFloat("an origin y", cmd, entry.eyeball.origin.y) ||
                !c.WantFloat("an origin z", cmd, entry.eyeball.origin.z))
                return false;
            haveOrigin = true;
        } else if (o == "diameter") {
            if (!c.WantFloat("a diameter", cmd, entry.eyeball.diameter)) return false;
        } else if (o == "angle") {
            if (!c.WantFloat("an angle in degrees", cmd, entry.eyeball.angle)) return false;
        } else if (o == "pupilscale") {
            if (!c.WantFloat("a pupil scale", cmd, entry.eyeball.pupilscale)) return false;
        } else if (o == "center") {
            entry.eyeball.center = true;
        } else {
            return c.Fail(t.line, "$eyeball: unknown option \"" + t.text +
                                  "\" (expected bone/origin/material/diameter/angle/"
                                  "pupilscale/center)");
        }
    }
    if (!haveBone)
        return c.Fail(cmd.line, "$eyeball \"" + entry.eyeball.name + "\": missing `bone`");
    if (!haveMaterial)
        return c.Fail(cmd.line, "$eyeball \"" + entry.eyeball.name + "\": missing `material`");
    if (!haveOrigin && !entry.eyeball.center)
        return c.Fail(cmd.line, "$eyeball \"" + entry.eyeball.name + "\": missing `origin`");

    c.face.entries.push_back(std::move(entry));
    return true;
}

// $mouth <controller> <bone> <forward x y z>  (Option_Mouth). The index is
// implicit - declaration order. The controller registers a flexdesc and the
// bone is force-kept, because the mouth shader reads it.
bool CmdMouth(Ctx& c, const Token& cmd) {
    FaceMarkup::Entry entry;
    entry.kind = FaceMarkup::Kind::Mouth;
    if (!c.Want("a flex controller name", cmd, entry.mouth.controller) ||
        !c.Want("a bone name", cmd, entry.mouth.bonename) ||
        !c.WantFloat("a forward x", cmd, entry.mouth.forward.x) ||
        !c.WantFloat("a forward y", cmd, entry.mouth.forward.y) ||
        !c.WantFloat("a forward z", cmd, entry.mouth.forward.z))
        return false;
    c.face.entries.push_back(std::move(entry));
    return true;
}

// $eyelid <upper|lower> [flexdesc <name>] lowerer <delta> <target>
//         neutral <delta> <target> raiser <delta> <target>
//         (righteyeball <name> lefteyeball <name> | eyeball <name>)
//
// Poses are addressed by delta name, not VTA frame index, and resolve against
// whichever body carries one; "-" means the pose has no vertex data. The
// righteyeball/lefteyeball form covers both eyes with one desc pair; `eyeball`
// with `flexdesc` is the per-eye form, one line per eye. Both need the $eyeball
// lines they name to come first.
bool CmdEyelid(Ctx& c, const Token& cmd) {
    FaceMarkup::Entry entry;
    entry.kind = FaceMarkup::Kind::Eyelid;

    std::string type;
    if (!c.Want("\"upper\" or \"lower\"", cmd, type))
        return false;
    if (_stricmp(type.c_str(), "upper") == 0) {
        entry.eyelid.upper = true;
    } else if (_stricmp(type.c_str(), "lower") == 0) {
        entry.eyelid.upper = false;
    } else {
        return c.Fail(cmd.line, "$eyelid: type must be \"upper\" or \"lower\", got \"" +
                                type + "\"");
    }

    static const char* kSuffix[3] = {"lowerer", "neutral", "raiser"};
    bool haveSlot[3] = {false, false, false};
    while (!c.AtCommand()) {
        const Token& t = *c.Next();
        const std::string o = Lower(t.text);
        int slot = -1;
        for (int i = 0; i < 3; ++i)
            if (o == kSuffix[i]) { slot = i; break; }
        if (slot >= 0) {
            if (!c.Want("a delta state name or \"-\"", cmd, entry.eyelid.delta[slot]) ||
                !c.WantFloat("a lid target", cmd, entry.eyelid.target[slot]))
                return false;
            if (entry.eyelid.delta[slot] == "-")
                entry.eyelid.delta[slot].clear();
            // targets are scaled at parse time (`* g_currentscale`)
            entry.eyelid.target[slot] *= c.in.scale;
            haveSlot[slot] = true;
        } else if (o == "flexdesc") {
            if (!c.Want("a flexdesc name", cmd, entry.eyelid.basedesc)) return false;
        } else if (o == "eyeball") {
            if (!c.Want("an eyeball name", cmd, entry.eyelid.eyeball)) return false;
        } else if (o == "righteyeball") {
            if (!c.Want("an eyeball name", cmd, entry.eyelid.righteyeball)) return false;
        } else if (o == "lefteyeball") {
            if (!c.Want("an eyeball name", cmd, entry.eyelid.lefteyeball)) return false;
        } else {
            return c.Fail(t.line, "$eyelid: unknown option \"" + t.text +
                                  "\" (expected lowerer/neutral/raiser/flexdesc/eyeball/"
                                  "righteyeball/lefteyeball)");
        }
    }
    for (int i = 0; i < 3; ++i)
        if (!haveSlot[i])
            return c.Fail(cmd.line, std::string("$eyelid ") + type + ": missing `" +
                                    kSuffix[i] + " <delta> <target>`");
    const bool mono = !entry.eyelid.eyeball.empty();
    if (mono) {
        if (!entry.eyelid.righteyeball.empty() || !entry.eyelid.lefteyeball.empty())
            return c.Fail(cmd.line, "$eyelid " + type +
                                    ": `eyeball` is the per-eye form, it cannot be mixed with "
                                    "`righteyeball`/`lefteyeball`");
        if (entry.eyelid.basedesc.empty())
            return c.Fail(cmd.line, "$eyelid " + type +
                                    ": the `eyeball` form needs `flexdesc <name>` for its lid "
                                    "flexdesc");
    } else {
        if (!entry.eyelid.basedesc.empty())
            return c.Fail(cmd.line, "$eyelid " + type +
                                    ": `flexdesc` only applies to the `eyeball` form; the "
                                    "paired form names its descs after the type");
        if (entry.eyelid.righteyeball.empty())
            return c.Fail(cmd.line, "$eyelid " + type + ": missing `righteyeball <name>`");
        if (entry.eyelid.lefteyeball.empty())
            return c.Fail(cmd.line, "$eyelid " + type + ": missing `lefteyeball <name>`");
    }

    c.face.entries.push_back(std::move(entry));
    return true;
}

// $flexcorrective <morph> <control> <control> [<control>...]
// The hand-authored form of a DMX combination operator's corrective: the morph
// fires only when every listed control is up. Bare name = a flex controller,
// %name = another morph. Compiles to one fetch per control, then COMBO n - the
// same op shape AddBodyFlexRuleForKey builds.
bool CmdFlexCorrective(Ctx& c, const Token& cmd) {
    ManualFlex::Rule rule;
    if (!c.Want("a morph name", cmd, rule.name))
        return false;

    while (!c.AtCommand())
        rule.combo.push_back(c.Next()->text);

    if (rule.combo.size() < 2)
        return c.Fail(cmd.line, "$flexcorrective \"" + rule.name + "\" expects at least two "
                                "controls to combine (for one, use $flexrule " + rule.name +
                                " = <control>)");

    c.manual.rules.push_back(std::move(rule));
    return true;
}

// $flexdominate <morph> <dominator> [<dominator>...]
// The hand-authored form of a DMX domination rule: the listed controls suppress
// the morph as they rise. Bare name = a flex controller, %name = another morph.
// Appended to the morph's rule after every rule exists (ApplyDominations), so it
// needs the morph to already have one - a $flexcorrective or $flexrule.
bool CmdFlexDominate(Ctx& c, const Token& cmd) {
    ManualFlex::Domination dom;
    if (!c.Want("a morph name", cmd, dom.name))
        return false;

    while (!c.AtCommand())
        dom.dominators.push_back(c.Next()->text);

    if (dom.dominators.empty())
        return c.Fail(cmd.line, "$flexdominate \"" + dom.name + "\" expects at least one "
                                "dominating control");

    c.manual.dominations.push_back(std::move(dom));
    return true;
}

// $morphsplitstereo [splitfactor <f>] <morph> [<morph>...]
// Split an already-encoded delta state into its <name>L/<name>R desc pair, the
// same split a stereo combination control would have made. There is no
// controller side to this: the engine blends the pair off the mesh's per-vertex
// balance. `splitfactor` synthesizes that balance from the base vertex X for a
// mesh that carries none (a primitive midline threshold, not a painted DMX
// weight); without it the mesh must have real balance data or nothing drives the
// halves apart (WarnStereoWithoutBalance). Like $flexcontroller's `range` it
// applies to every name AFTER it, so one line can mix factors. 0 is the default
// and means "no synthesis" - pass it explicitly to go back to the mesh's own
// balance for the names that follow.
bool CmdMorphSplitStereo(Ctx& c, const Token& cmd) {
    float factor = 0.0f;
    int names = 0;
    while (!c.AtCommand()) {
        const Token& t = *c.Next();
        if (!t.quoted && _stricmp(t.text.c_str(), "splitfactor") == 0) {
            if (!c.WantFloat("a split factor", cmd, factor))
                return false;
            continue;
        }
        c.manual.stereoSplits.push_back({t.text, factor});
        names++;
    }
    if (names == 0)
        return c.Fail(cmd.line, "$morphsplitstereo expects at least one morph name");
    return true;
}

// $flexcullmethod <name> - how hard the flex tables are pruned once everything
// is registered. Replaces studiomdl's -cullmorphs and -cullflex launch options,
// which are both folded into "aggressive". See CullFlex() in compile.cpp.
bool CmdFlexCullMethod(Ctx& c, const Token& cmd) {
    std::string m;
    if (!c.Want("a cull method name", cmd, m))
        return false;
    if (_stricmp(m.c_str(), "aggressive") == 0)
        c.in.flexCullMethod = cm::FlexCullMethod::Aggressive;
    else if (_stricmp(m.c_str(), "rules_only") == 0)
        c.in.flexCullMethod = cm::FlexCullMethod::RulesOnly;
    else if (_stricmp(m.c_str(), "duplicates") == 0)
        c.in.flexCullMethod = cm::FlexCullMethod::Duplicates;
    else if (_stricmp(m.c_str(), "none") == 0)
        c.in.flexCullMethod = cm::FlexCullMethod::None;
    else
        return c.Fail(cmd.line, "unknown $flexcullmethod \"" + m +
                                "\" (expected aggressive/rules_only/duplicates/none)");
    return true;
}

// $animationcullmethod <name> - which $animations survive to the .mdl. Replaces
// studiomdl's -cullanims launch option, whose cull is "aggressive". Never
// removes anything but an animation. See CullAnimations() in compile.cpp.
bool CmdAnimationCullMethod(Ctx& c, const Token& cmd) {
    std::string m;
    if (!c.Want("a cull method name", cmd, m))
        return false;
    if (_stricmp(m.c_str(), "aggressive") == 0)
        c.in.animCullMethod = cm::AnimCullMethod::Aggressive;
    else if (_stricmp(m.c_str(), "duplicates") == 0)
        c.in.animCullMethod = cm::AnimCullMethod::Duplicates;
    else if (_stricmp(m.c_str(), "none") == 0)
        c.in.animCullMethod = cm::AnimCullMethod::None;
    else
        return c.Fail(cmd.line, "unknown $animationcullmethod \"" + m +
                                "\" (expected aggressive/duplicates/none)");
    return true;
}

// studiomdl sequence options we deliberately do not accept. Returns the reason,
// or null if `o` is not one of them.
const char* UnsupportedSeqOption(const std::string& o) {
    // `deform` never shipped: the only call site is inside a comment block and
    // Option_Deform does not exist.
    if (_stricmp(o.c_str(), "deform") == 0)
        return "never implemented in studiomdl either";
    return nullptr;
}

// $sequence <name> <animref...> [options] or $sequence <name> { ... }
// (studiomdl Cmd_Sequence + ParseSequence). $bindposesequence takes the same
// options minus animation refs - its one animation is the bind pose, so a
// non-option token is an error, not a file to load. `blends` collects
// discovered animation indices for the caller's post-processing. `isAppend`
// ($append/$prepend/$continue reopening a finished sequence) still applies
// options but adds no further blend animation.
bool ParseSeqBody(Ctx& c, const Token& cmd, cm::CompileInput::InSequence& seq,
                  std::vector<int>& blends, bool bindpose, bool isAppend) {
    bool braced = false;
    if (!c.Eof() && !c.Cur().quoted && c.Cur().text == "{") {
        braced = true;
        c.pos++;
    }

    // nested braces inside the body, stock's ParseSequence `depth`. This is what
    // makes the conventional `{ event 5004 5 "sound" }` wrapper legal.
    int depth = 0;

    for (;;) {
        if (braced) {
            if (c.Eof())
                return c.Fail(cmd.line, cmd.text + " \"" + seq.name + "\" is missing '}'");
            if (!c.Cur().quoted && c.Cur().text == "}") {
                c.pos++;
                if (depth > 0) { depth--; continue; }
                break;
            }
        } else if (c.AtCommand()) {
            break;
        }

        const Token t = c.toks[c.pos++];
        const std::string& o = t.text;

        if (!t.quoted && o == "{") { depth++; continue; }
        if (!t.quoted && o == "}") {
            // in a braced body the closing brace is handled above, so reaching
            // here means an unbraced sequence saw one
            if (depth == 0)
                return c.Fail(t.line, cmd.text + " \"" + seq.name + "\": unexpected '}'");
            depth--;
            continue;
        }

        // --- sequence-level options (checked before animation options, exactly
        //     like stock ParseSequence) ---
        if (!t.quoted) {
            if (_stricmp(o.c_str(), "activity") == 0 || HasActPrefix(o)) {
                if (_stricmp(o.c_str(), "activity") == 0) {
                    if (!c.Want("an activity name", t, seq.activityname))
                        return false;
                } else {
                    seq.activityname = o; // ACT_* shorthand: the token is the name
                }
                // $allowactivityname whitelist, checked where stock checks it
                if (!c.allowedActivities.empty()) {
                    bool ok = false;
                    for (const std::string& n : c.allowedActivities)
                        if (n == seq.activityname) { ok = true; break; }
                    if (!ok)
                        return c.Fail(t.line, "unknown sequence activity \"" +
                                              seq.activityname + "\" in \"" + seq.name +
                                              "\" (not in $allowactivityname)");
                }
                // optional integer weight; absent -> -1 (Option_Activity)
                if (c.NextIsInt()) {
                    if (!c.WantInt("an activity weight", t, seq.actweight))
                        return false;
                    if (seq.actweight == 0)
                        return c.Fail(t.line, "activity \"" + seq.activityname +
                                              "\" weight must be a nonzero integer");
                } else {
                    seq.actweight = -1;
                }
                continue;
            }
            if (_stricmp(o.c_str(), "snap") == 0) { seq.flags |= kStudioSnap; continue; }
            if (_stricmp(o.c_str(), "autoplay") == 0) { seq.flags |= kStudioAutoplay; continue; }
            if (_stricmp(o.c_str(), "hidden") == 0) { seq.flags |= kStudioHidden; continue; }
            if (_stricmp(o.c_str(), "realtime") == 0) { seq.flags |= kStudioRealtime; continue; }
            if (_stricmp(o.c_str(), "post") == 0) { seq.flags |= kStudioPost; continue; }
            if (_stricmp(o.c_str(), "delta") == 0) {
                seq.flags |= kStudioDelta | kStudioPost;
                continue;
            }
            if (_stricmp(o.c_str(), "predelta") == 0) { seq.flags |= kStudioDelta; continue; }
            if (_stricmp(o.c_str(), "worldspace") == 0) {
                seq.flags |= kStudioWorld | kStudioPost;
                continue;
            }
            if (_stricmp(o.c_str(), "worldrelative") == 0) {
                seq.flags |= kStudioWorldAndRelative | kStudioPost;
                continue;
            }
            // rootdriver <bone>: the engine derives a root transform from this
            // bone's motion (STUDIO_ROOTXFORM)
            if (_stricmp(o.c_str(), "rootdriver") == 0) {
                seq.flags |= kStudioRootXform;
                if (!c.Want("a bone name", t, seq.rootdriverBone))
                    return false;
                continue;
            }
            if (_stricmp(o.c_str(), "exitphase") == 0) {
                if (!c.WantFloat("a phase", t, seq.exitphase)) return false;
                continue;
            }
            // posecycle <param>: the sequence's cycle is driven by a pose
            // parameter instead of time. The parameter is resolved (and
            // auto-created) in the compile stage, like `blend`.
            if (_stricmp(o.c_str(), "posecycle") == 0) {
                seq.flags |= kStudioCyclePose;
                if (!c.Want("a pose parameter name", t, seq.posecycle))
                    return false;
                continue;
            }
            if (_stricmp(o.c_str(), "fadein") == 0) {
                if (!c.WantFloat("a time", t, seq.fadeintime)) return false;
                continue;
            }
            if (_stricmp(o.c_str(), "fadeout") == 0) {
                if (!c.WantFloat("a time", t, seq.fadeouttime)) return false;
                continue;
            }
            if (_stricmp(o.c_str(), "blendwidth") == 0) {
                if (!c.WantInt("a blend width", t, seq.blendwidth)) return false;
                continue;
            }
            if (_stricmp(o.c_str(), "blend") == 0) {
                if (seq.numblendparams >= 2)
                    return c.Fail(t.line, cmd.text + " \"" + seq.name +
                                          "\": more than 2 blend parameters");
                auto& bp = seq.blendparams[seq.numblendparams++];
                if (!c.Want("a pose parameter name", t, bp.parameter) ||
                    !c.WantFloat("a start value", t, bp.min) ||
                    !c.WantFloat("an end value", t, bp.max))
                    return false;
                continue;
            }
            // calcblend <param> <attachment> <control>: derive this axis's pose
            // range from how the attachment moves across the grid.
            if (_stricmp(o.c_str(), "calcblend") == 0) {
                if (seq.numblendparams >= 2)
                    return c.Fail(t.line, cmd.text + " \"" + seq.name +
                                          "\": more than 2 blend parameters");
                const int slot = seq.numblendparams;
                auto& bp = seq.blendparams[seq.numblendparams++];
                std::string control;
                if (!c.Want("a pose parameter name", t, bp.parameter) ||
                    !c.Want("an attachment name", t, seq.paramattachment[slot]) ||
                    !c.Want("a motion control", t, control))
                    return false;
                const int ctrl = LookupControl(control);
                if (ctrl == -1)
                    return c.Fail(t.line, "unknown calcblend control \"" + control +
                                          "\" (expected X/Y/Z/XR/YR/ZR)");
                bp.calc = true; // range comes from the attachment, not min/max
                seq.paramcontrol[slot] = ctrl;
                continue;
            }
            if (_stricmp(o.c_str(), "blendref") == 0) {
                if (!c.Want("an animation name", t, seq.paramanim)) return false;
                continue;
            }
            if (_stricmp(o.c_str(), "blendcomp") == 0) {
                if (!c.Want("an animation name", t, seq.paramcompanim)) return false;
                continue;
            }
            if (_stricmp(o.c_str(), "blendcenter") == 0) {
                if (!c.Want("an animation name", t, seq.paramcenter)) return false;
                continue;
            }
            if (_stricmp(o.c_str(), "addlayer") == 0) {
                cm::CompileInput::InAutoLayer al;
                if (!c.Want("a sequence name", t, al.sequence))
                    return false;
                while (!c.AtCommand() && !c.Cur().quoted && c.Cur().text != "}" &&
                       _stricmp(c.Cur().text.c_str(), "local") == 0) {
                    al.flags |= kStudioAlLocal;
                    seq.flags |= kStudioLocal;
                    c.pos++;
                }
                seq.autolayers.push_back(std::move(al));
                continue;
            }
            if (_stricmp(o.c_str(), "blendlayer") == 0) {
                cm::CompileInput::InAutoLayer al;
                if (!c.Want("a sequence name", t, al.sequence) ||
                    !c.WantFloat("a start frame", t, al.start) ||
                    !c.WantFloat("a peak frame", t, al.peak) ||
                    !c.WantFloat("a tail frame", t, al.tail) ||
                    !c.WantFloat("an end frame", t, al.end))
                    return false;
                for (;;) {
                    if (c.AtCommand() || c.Cur().quoted || c.Cur().text == "}")
                        break;
                    const std::string f = c.Cur().text;
                    if (_stricmp(f.c_str(), "xfade") == 0) { al.flags |= kStudioAlXfade; c.pos++; }
                    else if (_stricmp(f.c_str(), "spline") == 0) { al.flags |= kStudioAlSpline; c.pos++; }
                    else if (_stricmp(f.c_str(), "noblend") == 0) { al.flags |= kStudioAlNoblend; c.pos++; }
                    else if (_stricmp(f.c_str(), "local") == 0) {
                        al.flags |= kStudioAlLocal;
                        seq.flags |= kStudioLocal;
                        c.pos++;
                    } else if (_stricmp(f.c_str(), "poseparameter") == 0) {
                        c.pos++;
                        al.flags |= kStudioAlPose;
                        if (!c.Want("a pose parameter name", t, al.poseparameter))
                            return false;
                    } else {
                        break;
                    }
                }
                seq.autolayers.push_back(std::move(al));
                continue;
            }
            if (_stricmp(o.c_str(), "iklock") == 0) {
                cm::IkLock lock;
                if (!c.Want("an ik chain name", t, lock.name) ||
                    !c.WantFloat("a position weight", t, lock.flPosWeight) ||
                    !c.WantFloat("a rotation weight", t, lock.flLocalQWeight))
                    return false;
                seq.iklocks.push_back(std::move(lock));
                continue;
            }
            if (_stricmp(o.c_str(), "event") == 0) {
                if (!ParseEvent(c, t, seq))
                    return false;
                continue;
            }
            // animtag <name> <cycle>  (Option_AnimTag): a named marker at a
            // cycle fraction, resolved to a tag id at runtime.
            if (_stricmp(o.c_str(), "animtag") == 0) {
                cm::SeqAnimTag tag;
                if (!c.Want("a tag name", t, tag.name) ||
                    !c.WantFloat("a cycle", t, tag.cycle))
                    return false;
                seq.animtags.push_back(std::move(tag));
                if (seq.animtags.size() > static_cast<size_t>(pulse::limits::kMaxTags))
                    return c.Fail(t.line, "too many animtags in \"" + seq.name + "\"");
                continue;
            }
            if (_stricmp(o.c_str(), "keyvalues") == 0) {
                if (!ParseKeyValues(c, t, seq.keyvalues))
                    return false;
                continue;
            }
            // transition graph. `node` is the both-ends form; `transition` goes
            // one way; `rtransition` is the same but reversible (nodeflags 1).
            if (_stricmp(o.c_str(), "node") == 0) {
                std::string n;
                if (!c.Want("a node name", t, n))
                    return false;
                seq.entrynode = seq.exitnode = LookupXNode(c, n);
                continue;
            }
            if (_stricmp(o.c_str(), "transition") == 0 ||
                _stricmp(o.c_str(), "rtransition") == 0) {
                std::string from, to;
                if (!c.Want("an entry node name", t, from) ||
                    !c.Want("an exit node name", t, to))
                    return false;
                seq.entrynode = LookupXNode(c, from);
                seq.exitnode = LookupXNode(c, to);
                if (_stricmp(o.c_str(), "rtransition") == 0)
                    seq.nodeflags |= 1;
                continue;
            }
            // activitymodifier <name> | activitymodifier { <name> ... }
            // (Option_ActivityModifier). Names are lowercased.
            if (_stricmp(o.c_str(), "activitymodifier") == 0 ||
                _stricmp(o.c_str(), "actmod") == 0) {
                std::string first;
                if (!c.Want("an activity modifier name", t, first))
                    return false;
                auto add = [&seq](std::string n) {
                    for (char& ch : n)
                        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
                    seq.activitymodifiers.push_back(std::move(n));
                };
                if (first == "{") {
                    for (;;) {
                        const Token* m = c.Next();
                        if (!m)
                            return c.Fail(t.line, "activitymodifier block is missing '}'");
                        if (!m->quoted && m->text == "}")
                            break;
                        add(m->text);
                    }
                } else {
                    add(std::move(first));
                }
                if (seq.activitymodifiers.size() >
                    static_cast<size_t>(pulse::limits::kMaxActivityModifiers))
                    return c.Fail(t.line, "too many activity modifiers in \"" +
                                          seq.name + "\"");
                continue;
            }
            if (const char* why = UnsupportedSeqOption(o))
                return c.Fail(t.line, "\"" + o + "\": " + why);
        }

        // --- animation options apply to blend anim 0, once one exists (stock
        //     routes ParseAnimationToken to animations[0]) ---
        if (!blends.empty()) {
            const int r = ApplyAnimOption(c, t, c.in.anims[blends[0]]);
            if (r < 0)
                return false;
            if (r == 1)
                continue;
        }

        // a bind-pose sequence already has its animation, and an append re-opens
        // a sequence whose blend grid is already fixed - in neither case can a
        // token here be an animation reference, so it is just a bad option
        if (bindpose || isAppend)
            return c.Fail(t.line, cmd.text + " \"" + seq.name +
                                  "\": unknown option \"" + t.text + "\"");

        // --- otherwise an animation reference: a named $animation, else a
        //     filename loaded as an implied "@<seq>" animation ---
        auto named = c.namedAnims.find(o);
        if (named != c.namedAnims.end()) {
            blends.push_back(named->second);
        } else {
            cm::CompileInput::InAnim a;
            // implied name is always "@<seq>" - the reference names every
            // implied blend identically; the string table dedups them
            a.name = "@" + seq.name;
            a.fps = c.defaultFps;
            a.source = LoadSource(c, WithSourceExtension(c, o), t.line, /*morphSource=*/false,
                                  /*edit=*/nullptr, source::LoadKind::Animation);
            if (!a.source)
                return false;
            blends.push_back(static_cast<int>(c.in.anims.size()));
            c.in.anims.push_back(std::move(a));
        }
    }
    return true;
}

bool CmdSequenceCommon(Ctx& c, const Token& cmd, bool bindpose) {
    cm::CompileInput::InSequence seq;
    seq.fadeintime = c.defaultFadeIn;
    seq.fadeouttime = c.defaultFadeOut;
    if (!c.Want("a name", cmd, seq.name))
        return false;
    // $lcaseallsequences lowercases the $sequence name only (Cmd_Sequence)
    if (c.lcaseSequences)
        for (char& ch : seq.name)
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));

    // blend animation indices as discovered (into c.in.anims). The first one
    // receives sequence-body animation options, matching stock ParseSequence's
    // ParseAnimationToken(animations[0]).
    std::vector<int> blends;
    // a bind-pose sequence gets its single implied animation up front, so the
    // animation options in the body land on it exactly as they would on an
    // implied file animation
    if (bindpose) {
        cm::CompileInput::InAnim a;
        a.name = "@" + seq.name;
        a.fps = c.defaultFps;
        a.bindpose = true;
        a.endframe = 0;
        blends.push_back(static_cast<int>(c.in.anims.size()));
        c.in.anims.push_back(std::move(a));
    }

    if (!ParseSeqBody(c, cmd, seq, blends, bindpose, /*isAppend=*/false))
        return false;

    if (blends.empty())
        return c.Fail(cmd.line, cmd.text + " \"" + seq.name + "\" has no animations");

    // stock CopyAnimationSettings: implied blend anims after the
    // first inherit anim 0's fps, transform and whole ordered command list.
    // Named animation references keep their own settings.
    for (size_t i = 1; i < blends.size(); i++) {
        cm::CompileInput::InAnim& dst = c.in.anims[blends[i]];
        if (dst.name.empty() || dst.name[0] != '@')
            continue;
        const cm::CompileInput::InAnim& src0 = c.in.anims[blends[0]];
        dst.fps = src0.fps;
        dst.adjust = src0.adjust;
        dst.adjustSet = src0.adjustSet;
        dst.scale = src0.scale;
        dst.rotation = src0.rotation;
        dst.rotationSet = src0.rotationSet;
        dst.cmds.insert(dst.cmds.end(), src0.cmds.begin(), src0.cmds.end());
        dst.ikrules.insert(dst.ikrules.end(), src0.ikrules.begin(), src0.ikrules.end());
    }

    seq.animIndex = blends[0];
    if (blends.size() > 1)
        seq.blendAnims = std::move(blends);
    c.in.sequences.push_back(std::move(seq));
    return true;
}

bool CmdSequence(Ctx& c, const Token& cmd) { return CmdSequenceCommon(c, cmd, false); }
bool CmdBindPoseSequence(Ctx& c, const Token& cmd) { return CmdSequenceCommon(c, cmd, true); }

// $append / $prepend / $continue <name> [options] (studiomdl Cmd_Append/
// Cmd_Prepend/Cmd_Continue). Re-opens a $sequence or $animation - sequence
// looked up first, so a name shared between the two resolves to the sequence.
// $append puts new animation commands after the existing ones, $prepend
// before, $continue is $append with an optional body. A sequence append can
// edit blend animation 0 but not add blend animations (grid fixed at close).
enum class AppendMode { Append, Prepend, Continue };

bool CmdAppendCommon(Ctx& c, const Token& cmd, AppendMode mode) {
    std::string name;
    if (!c.Want("a $sequence or $animation name", cmd, name))
        return false;

    // where the clip's command list ends now - everything the body adds past
    // this point is what $prepend moves to the front
    auto rotateForPrepend = [&](cm::CompileInput::InAnim& a, size_t before) {
        if (mode != AppendMode::Prepend)
            return true;
        if (a.cmds.size() == before)
            return c.Fail(cmd.line, cmd.text + " \"" + name +
                                    "\": no animation command to prepend");
        std::rotate(a.cmds.begin(), a.cmds.begin() + static_cast<ptrdiff_t>(before),
                    a.cmds.end());
        return true;
    };

    for (cm::CompileInput::InSequence& seq : c.in.sequences) {
        if (_stricmp(seq.name.c_str(), name.c_str()) != 0)
            continue;
        if (seq.isDeclare || seq.animIndex < 0)
            return c.Fail(cmd.line, cmd.text + " \"" + name +
                                    "\": a $declaresequence slot has no animation");
        std::vector<int> blends{seq.animIndex};
        const size_t before = c.in.anims[seq.animIndex].cmds.size();
        if (!ParseSeqBody(c, cmd, seq, blends, /*bindpose=*/false, /*isAppend=*/true))
            return false;
        return rotateForPrepend(c.in.anims[seq.animIndex], before);
    }

    // sequence names are compared case-insensitively above, so animations are
    // too rather than have the two disagree
    int animIndex = -1;
    for (const auto& kv : c.namedAnims) {
        if (_stricmp(kv.first.c_str(), name.c_str()) == 0) { animIndex = kv.second; break; }
    }
    if (animIndex < 0)
        return c.Fail(cmd.line, cmd.text + ": no $sequence or $animation named \"" +
                                name + "\"");

    cm::CompileInput::InAnim& a = c.in.anims[animIndex];
    const size_t before = a.cmds.size();
    if (!ParseAnimBody(c, cmd, name, a))
        return false;
    return rotateForPrepend(a, before);
}

bool CmdAppend(Ctx& c, const Token& cmd) { return CmdAppendCommon(c, cmd, AppendMode::Append); }
bool CmdPrepend(Ctx& c, const Token& cmd) { return CmdAppendCommon(c, cmd, AppendMode::Prepend); }
bool CmdContinue(Ctx& c, const Token& cmd) { return CmdAppendCommon(c, cmd, AppendMode::Continue); }

// $modelarchetype <name> - replaces QC's $staticprop / $simpleprop flags.
bool CmdModelArchetype(Ctx& c, const Token& cmd) {
    std::string a;
    if (!c.Want("an archetype name", cmd, a))
        return false;
    if (_stricmp(a.c_str(), "character") == 0 || _stricmp(a.c_str(), "general") == 0)
        c.in.archetype = cm::Archetype::General;
    else if (_stricmp(a.c_str(), "static") == 0)
        c.in.archetype = cm::Archetype::Static;
    else if (_stricmp(a.c_str(), "simple") == 0)
        c.in.archetype = cm::Archetype::Simple;
    else
        return c.Fail(cmd.line, "unknown $modelarchetype \"" + a +
                                "\" (expected character/general/static/simple)");
    return true;
}

// $vtxformat <int> - which .vtx strip/stripgroup layout to write. 0 = legacy
// 27/25-byte headers (TF2/L4D2/GMod/HL2), 1 = full 35/33-byte headers with the
// topology fields (SFM/CS:GO/ASW+). The -vtxformat launch switch overrides it.
bool CmdVtxFormat(Ctx& c, const Token& cmd) {
    if (!c.WantInt("a vtx format (0 or 1)", cmd, c.in.vtxArchetype))
        return false;
    if (c.in.vtxArchetype != 0 && c.in.vtxArchetype != 1)
        return c.Fail(cmd.line, "unknown $vtxformat (expected 0 or 1)");
    return true;
}

// $modelbudget { bones <n> materials <n> } - lower the compile ceilings for
// this model. Braces are optional for a single field. Budgets only ever lower:
// the pulselimits.h value is both the default and the cap.
bool CmdModelBudget(Ctx& c, const Token& cmd) {
    const bool braced = !c.Eof() && !c.Cur().quoted && c.Cur().text == "{";
    if (braced)
        c.pos++;

    do {
        if (c.Eof())
            return c.Fail(cmd.line, cmd.text + (braced ? ": missing '}'"
                                                       : ": expects a budget name"));
        const Token t = c.toks[c.pos++];
        if (braced && !t.quoted && t.text == "}")
            break;
        const std::string o = t.quoted ? std::string() : Lower(t.text);
        const Token sub{cmd.text + " " + t.text, t.line, false};

        int* dst = nullptr;
        int hard = 0;
        if (o == "bones") {
            dst = &c.in.budgetBones;
            hard = lim::kMaxBones;
        } else if (o == "materials") {
            dst = &c.in.budgetMaterials;
            hard = lim::kMaxSkins;
        } else {
            return c.Fail(t.line, cmd.text + ": invalid syntax \"" + t.text + "\"");
        }

        int value = 0;
        if (!c.WantInt("a count", sub, value))
            return false;
        if (value < 1 || value > hard)
            return c.Fail(t.line, cmd.text + " " + o + ": " + std::to_string(value) +
                                  " is out of range (1-" + std::to_string(hard) + ")");
        *dst = value;
    } while (braced);
    return true;
}

// $renderpass <name> - replaces QC's $opaque / $mostlyopaque flags. "none" is
// authorable, so the no-flag default can be written down explicitly.
bool CmdRenderPass(Ctx& c, const Token& cmd) {
    std::string p;
    if (!c.Want("a render pass name", cmd, p))
        return false;
    if (_stricmp(p.c_str(), "none") == 0)
        c.in.renderPass = 0;
    else if (_stricmp(p.c_str(), "opaque") == 0)
        c.in.renderPass = 1;
    else if (_stricmp(p.c_str(), "mostlyopaque") == 0)
        c.in.renderPass = 2;
    else
        return c.Fail(cmd.line, "unknown $renderpass \"" + p +
                                "\" (expected none/opaque/mostlyopaque)");
    return true;
}

// $setbindpose <file> <frame> - bake one frame of a pose file into the rest
// mesh. Loaded as an ANIMATION source (bones + animation only), so the pose file
// contributes no geometry and no materials.
bool CmdSetBindPose(Ctx& c, const Token& cmd) {
    std::string file;
    if (!c.Want("a pose source filename", cmd, file))
        return false;
    int frame = 0;
    if (!c.WantInt("a frame number", cmd, frame))
        return false;
    if (c.in.bindPoseSource)
        return c.Fail(cmd.line, "$setbindpose written twice");
    source::Source* src =
        LoadSource(c, file, cmd.line, /*morphSource=*/false, /*edit=*/nullptr,
                   source::LoadKind::Animation);
    if (!src)
        return false;
    c.in.bindPoseSource = src;
    c.in.bindPoseFrame = frame;
    return true;
}

// $setflex <morph> <strength> - bake a fixed morph amount into the rest mesh.
bool CmdSetFlex(Ctx& c, const Token& cmd) {
    cm::CompileInput::FixedFlex fx;
    if (!c.Want("a morph name", cmd, fx.name))
        return false;
    if (!c.WantFloat("a strength", cmd, fx.value))
        return false;
    if (fx.value < 0.0f || fx.value > 1.0f)
        std::printf("WARNING: %s(%d): $setflex \"%s\" strength %g outside [0,1] - clamped\n",
                    c.file.c_str(), cmd.line, fx.name.c_str(), fx.value);
    fx.line = cmd.line;
    c.in.fixedFlexes.push_back(std::move(fx));
    return true;
}

bool CmdSurfaceProp(Ctx& c, const Token& cmd) {
    return c.Want("a surface property name", cmd, c.in.surfaceprop);
}

// The token list shared by $contents and $jointcontents (reference
// ParseContents). Each token adds bits and may clear others; a bare number
// (decimal or 0x hex) sets raw bits. At least one token is required.
bool ParseContentsTokens(Ctx& c, const Token& cmd, int& add, int& remove) {
    std::string tok;
    if (!c.Want("a contents value", cmd, tok))
        return false;
    add = 0;
    remove = 0;
    for (;;) {
        if (_stricmp(tok.c_str(), "solid") == 0) {
            add |= kContentsSolid;
        } else if (_stricmp(tok.c_str(), "grate") == 0) {
            add |= kContentsGrate;
            remove |= kContentsSolid;
        } else if (_stricmp(tok.c_str(), "ladder") == 0) {
            add |= kContentsLadder;
        } else if (_stricmp(tok.c_str(), "monster") == 0) {
            add |= kContentsMonster;
        } else if (_stricmp(tok.c_str(), "debris") == 0) {
            add |= kContentsDebris;
        } else if (_stricmp(tok.c_str(), "notsolid") == 0) {
            remove |= kContentsSolid;
        } else {
            char* end = nullptr;
            const long v = strtol(tok.c_str(), &end, 0);
            if (end == tok.c_str() || *end != '\0')
                return c.Fail(cmd.line, "unknown " + cmd.text + " value \"" + tok + "\"");
            add |= static_cast<int>(v);
        }
        if (c.AtCommand())
            break;
        tok = c.toks[c.pos++].text;
    }
    return true;
}

// $contents <token>... - what the model counts as for traces. The word starts
// at CONTENTS_SOLID, so "grate" (which removes solid) yields grate alone.
// Repeating the command accumulates onto the running word.
bool CmdContents(Ctx& c, const Token& cmd) {
    int add = 0, remove = 0;
    if (!ParseContentsTokens(c, cmd, add, remove))
        return false;
    c.in.contents = (c.in.contents | add) & ~remove;
    return true;
}

// $jointcontents <bone> <token>... - the same word for one bone and, through
// the parent walk in ApplyJointContents, its descendants. Unlike $contents
// each command starts fresh from CONTENTS_SOLID, and a second command for the
// same bone REPLACES the word (reference Cmd_JointContents).
bool CmdJointContents(Ctx& c, const Token& cmd) {
    std::string bone;
    if (!c.Want("a bone name", cmd, bone))
        return false;
    int add = 0, remove = 0;
    if (!ParseContentsTokens(c, cmd, add, remove))
        return false;
    const int word = (kContentsSolid | add) & ~remove;
    for (auto& jc : c.in.jointContents) {
        if (_stricmp(jc.first.c_str(), bone.c_str()) == 0) {
            jc.second = word;
            return true;
        }
    }
    c.in.jointContents.emplace_back(bone, word);
    return true;
}

// $transformmodel [origin x y z] [angles x y z] [scale <float>]
//
// The whole modelmodifierlist transform in one command: translatemodel,
// rotatemodel and scale. Every clause is optional and they may appear in any
// order, so `$transformmodel origin 0 0 32` is complete on its own.
bool CmdTransformModel(Ctx& c, const Token& cmd) {
    bool any = false;
    while (!c.AtCommand()) {
        const Token& t = c.toks[c.pos++];
        // errors read "$transformmodel origin expects ..."
        const Token sub{cmd.text + " " + t.text, t.line, false};

        if (!t.quoted && _stricmp(t.text.c_str(), "origin") == 0) {
            float x = 0, y = 0, z = 0;
            if (!c.WantFloat("an X offset", sub, x) || !c.WantFloat("a Y offset", sub, y) ||
                !c.WantFloat("a Z offset", sub, z))
                return false;
            c.in.adjust = {x, y, z};
        } else if (!t.quoted && _stricmp(t.text.c_str(), "angles") == 0) {
            // degrees, (pitch, yaw, roll) - the roll composes with the built-in
            // +90 yaw exactly like .pulsemdl's rotatemodel
            float x = 0, y = 0, z = 0;
            if (!c.WantFloat("a pitch", sub, x) || !c.WantFloat("a yaw", sub, y) ||
                !c.WantFloat("a roll", sub, z))
                return false;
            c.in.rotation = {x * pm::kDeg2Rad, y * pm::kDeg2Rad,
                             (z + 90.0f) * pm::kDeg2Rad};
            c.in.rotationSet = true;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "scale") == 0) {
            // scale is baked in as each DMX is read, so a source loaded before
            // this point would silently keep the old scale - refuse instead
            if (!c.in.sources.empty())
                return c.Fail(t.line, "$transformmodel scale must appear before the "
                                      "first $rendermesh or $sequence");
            if (!c.WantFloat("a scale", sub, c.in.scale))
                return false;
        } else {
            return c.Fail(t.line, "$transformmodel: expected origin, angles or scale, "
                                  "got \"" + t.text + "\"");
        }
        any = true;
    }
    if (!any)
        return c.Fail(cmd.line, "$transformmodel expects at least one of "
                                "origin, angles or scale");
    return true;
}

// $upaxis <x|-x|y|-y|z|-z> (Cmd_UpAxis): names the up direction of the art
// space so the compiler can rotate it into Source's Z-up space. It writes the
// same slot as $transformmodel angles (reference g_defaultrotation), so
// whichever comes last wins. -x/-y are untested upstream, and z/-z both just
// leave the built-in +90 yaw.
bool CmdUpAxis(Ctx& c, const Token& cmd) {
    std::string axis;
    if (!c.Want("an axis", cmd, axis))
        return false;

    pm::RadianEuler r{0.0f, 0.0f, pm::kPiF / 2.0f};
    if (_stricmp(axis.c_str(), "x") == 0)
        r.y = pm::kPiF / 2.0f; // 90 about y moves x into z
    else if (_stricmp(axis.c_str(), "-x") == 0)
        r.y = -pm::kPiF / 2.0f;
    else if (_stricmp(axis.c_str(), "y") == 0)
        r.x = pm::kPiF / 2.0f; // 90 about x moves y into z
    else if (_stricmp(axis.c_str(), "-y") == 0)
        r.x = -pm::kPiF / 2.0f;
    else if (_stricmp(axis.c_str(), "z") != 0 && _stricmp(axis.c_str(), "-z") != 0)
        return c.Fail(cmd.line, "unknown $upaxis option: \"" + axis + "\"");

    c.in.rotation = r;
    c.in.rotationSet = true;
    return true;
}

// $attachment <name> [bone <name>] [origin x y z] [angles x y z]
//                    [rigid] [absolute] [world_align]
//                    [flexgroup <n>]... [flexmorph <n>]... [material <n>]...
//
// Also accepts the legacy studiomdl positional form (Cmd_Attachment):
//   $attachment <name> <bone> [<x> <y> <z>] [absolute] [rigid] [rotate p y r]
// telling the two apart is a peek: a token after the name that is not a clause
// keyword is the parent bone. `position` and `rotate` are aliases of `origin`
// and `angles`.
//
// Everything after the name is optional and order-independent. A missing
// origin/angles is (0,0,0). With no `bone` clause the attachment is tied to
// MODEL space: it is flagged absolute and the compile stage anchors it to the
// model's root bone.
//
// The three selector clauses repeat and replace the reference's separate
// $attachmentbyverts: they average the position of every vertex they match,
// origin becomes an offset from that average, and a bone-less attachment takes
// the bone those vertices weight to most (compile.cpp,
// GenerateVertexAveragedAttachments).
bool CmdAttachment(Ctx& c, const Token& cmd) {
    Ctx::PendingAttachment a;
    if (!c.Want("a name", cmd, a.name))
        return false;

    static const char* kClauses[] = {
        "bone",   "origin",   "position",  "angles",    "rotate",  "rigid",
        "absolute", "world_align", "flexgroup", "flexmorph", "material"};
    auto isClause = [](const Token& t) {
        if (t.quoted)
            return false;
        for (const char* k : kClauses)
            if (_stricmp(t.text.c_str(), k) == 0)
                return true;
        return false;
    };

    // legacy positional head: <bone> [<x> <y> <z>], then the same option loop.
    // The offset is optional here so a legacy bone can still take the modern
    // `origin`/`position` clause instead.
    if (!c.AtCommand() && !isClause(c.Cur())) {
        const Token sub{cmd.text + " \"" + a.name + "\"", c.Cur().line, false};
        if (!c.Want("a bone name", sub, a.bone))
            return false;
        if (!c.AtCommand() && !isClause(c.Cur())) {
            if (!c.WantFloat("an X offset", sub, a.origin.x) ||
                !c.WantFloat("a Y offset", sub, a.origin.y) ||
                !c.WantFloat("a Z offset", sub, a.origin.z))
                return false;
        }
    }

    while (!c.AtCommand()) {
        const Token& t = c.toks[c.pos++];
        const Token sub{cmd.text + " " + t.text, t.line, false};

        if (!t.quoted && _stricmp(t.text.c_str(), "bone") == 0) {
            if (!c.Want("a bone name", sub, a.bone))
                return false;
        } else if (!t.quoted && (_stricmp(t.text.c_str(), "origin") == 0 ||
                                 _stricmp(t.text.c_str(), "position") == 0)) {
            if (!c.WantFloat("an X offset", sub, a.origin.x) ||
                !c.WantFloat("a Y offset", sub, a.origin.y) ||
                !c.WantFloat("a Z offset", sub, a.origin.z))
                return false;
        } else if (!t.quoted && (_stricmp(t.text.c_str(), "angles") == 0 ||
                                 _stricmp(t.text.c_str(), "rotate") == 0)) {
            if (!c.WantFloat("a pitch", sub, a.anglesDeg.x) ||
                !c.WantFloat("a yaw", sub, a.anglesDeg.y) ||
                !c.WantFloat("a roll", sub, a.anglesDeg.z))
                return false;
            a.hasAngles = true;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "rigid") == 0) {
            a.type |= cm::kAttachIsRigid;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "absolute") == 0) {
            a.type |= cm::kAttachIsAbsolute;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "world_align") == 0) {
            a.flags |= cm::kAttachFlagWorldAlign;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "flexgroup") == 0) {
            a.flexgroups.emplace_back();
            if (!c.Want("a flex group name", sub, a.flexgroups.back()))
                return false;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "flexmorph") == 0) {
            a.flexmorphs.emplace_back();
            if (!c.Want("a morph name", sub, a.flexmorphs.back()))
                return false;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "material") == 0) {
            a.materials.emplace_back();
            if (!c.Want("a material name", sub, a.materials.back()))
                return false;
        } else {
            return c.Fail(t.line, "$attachment \"" + a.name +
                                  "\": expected bone, origin/position, "
                                  "angles/rotate, rigid, absolute, world_align, "
                                  "flexgroup, flexmorph or material, got \"" +
                                  t.text + "\"");
        }
    }

    // no bone -> the transform is model space, which is exactly what the
    // absolute flag means (kAttachIsAbsolute: "local is in model space").
    // Selectors are the exception: they derive a bone from the vertices.
    const bool byVerts =
        !a.flexgroups.empty() || !a.flexmorphs.empty() || !a.materials.empty();
    if (a.bone.empty() && !byVerts)
        a.type |= cm::kAttachIsAbsolute;

    // fill a $declareattachment slot of the same name in place so its pinned
    // index holds; otherwise append (reference Cmd_Attachment)
    for (Ctx::PendingAttachment& slot : c.attachments) {
        if (!slot.filled && _stricmp(slot.name.c_str(), a.name.c_str()) == 0) {
            slot = std::move(a);
            return true;
        }
    }

    c.attachments.push_back(std::move(a));
    return true;
}

// $declareattachment <name>  (studiomdl Cmd_DeclareAttachment): reserve an
// ordering slot so this attachment's final index is pinned here, the same idea
// as $declaresequence. A later $attachment of the same name, or a DMX source
// attachment imported by $datamodeljoints, fills the slot in place - that is
// the only way to sit a DMX attachment between two scripted ones. Useful when
// a game or mod addresses attachments by index instead of by name. A slot
// nothing ever fills is dropped with a warning.
bool CmdDeclareAttachment(Ctx& c, const Token& cmd) {
    Ctx::PendingAttachment a;
    if (!c.Want("a name", cmd, a.name))
        return false;
    // the bit never reaches the output - a slot is either replaced by its
    // definition or dropped, so `filled` is what the rest of the code tests
    a.type |= cm::kAttachIsDeclared;
    a.filled = false;
    c.attachments.push_back(std::move(a));
    return true;
}

// Build the attachment matrices once the script is fully read. Clause order in
// the matrix mirrors the .pulsemdl loader exactly: absolute seeds local from
// the inverse model rotation, an explicit angles overrides the rotation part,
// and the translation is stuffed in last.
void FinishAttachments(Ctx& c) {
    std::vector<cm::Attachment> scripted;
    const pm::RadianEuler defaultRot =
        c.in.rotationSet
            ? c.in.rotation
            : (c.in.upAxisY ? pm::RadianEuler{pm::kPiF / 2.0f, 0.0f, pm::kPiF / 2.0f}
                            : pm::RadianEuler{0.0f, 0.0f, pm::kPiF / 2.0f});

    for (const Ctx::PendingAttachment& p : c.attachments) {
        // a $declareattachment nothing scripted claimed: adopt the DMX source
        // attachment of that name, lifting it out of the floating tail so it
        // lands at the pinned index instead of after every scripted one
        if (!p.filled) {
            auto it = std::find_if(c.in.attachments.begin(), c.in.attachments.end(),
                                   [&p](const cm::Attachment& a) {
                                       return _stricmp(a.name.c_str(), p.name.c_str()) == 0;
                                   });
            if (it == c.in.attachments.end()) {
                std::fprintf(stderr,
                             "warning: $declareattachment \"%s\": never defined, dropping\n",
                             p.name.c_str());
                continue;
            }
            scripted.push_back(std::move(*it));
            c.in.attachments.erase(it);
            continue;
        }

        cm::Attachment att;
        att.name = p.name;
        att.bonename = p.bone;
        att.type = p.type;
        att.flags = p.flags;
        att.flexgroups = p.flexgroups;
        att.flexmorphs = p.flexmorphs;
        att.materials = p.materials;

        if (att.type & cm::kAttachIsAbsolute)
            pm::AngleIMatrix(defaultRot, att.local);
        if (p.hasAngles)
            pm::AngleMatrixDeg(p.anglesDeg, att.local);

        // position is scaled like a vertex, and goes in after the rotation.
        // With selectors this is the offset from the vertex average, not the
        // final position (GenerateVertexAveragedAttachments adds the average).
        const float s = p.noscale ? 1.0f : c.in.scale;
        att.local.m[0][3] = p.origin.x * s;
        att.local.m[1][3] = p.origin.y * s;
        att.local.m[2][3] = p.origin.z * s;

        scripted.push_back(std::move(att));
    }

    // $datamodeljoints may already have appended DMX-sourced attachments;
    // scripted ones always come first (reference ReorderSourceAttachmentsLast)
    c.in.attachments.insert(c.in.attachments.begin(), scripted.begin(), scripted.end());
}

// ---------------------------------------------------------------------------
// Bone structure - $definebone and the collapse markup, stock syntax 1:1.
// ---------------------------------------------------------------------------

// $bonecullmethod <name> - bone_cull_type. Replaces QC's $nocollapsebones
// (= none) and its "onlyweights" variant, and names the default so it can be
// written down explicitly. $alwayscollapse overrules "none" per bone.
bool CmdBoneCullMethod(Ctx& c, const Token& cmd) {
    std::string m;
    if (!c.Want("a cull method name", cmd, m))
        return false;
    if (_stricmp(m.c_str(), "aggressive") == 0)
        c.in.boneCullType = cm::BoneCullType::Aggressive;
    else if (_stricmp(m.c_str(), "leafonly") == 0)
        c.in.boneCullType = cm::BoneCullType::LeafOnly;
    else if (_stricmp(m.c_str(), "none") == 0)
        c.in.boneCullType = cm::BoneCullType::None;
    else
        return c.Fail(cmd.line, "unknown $bonecullmethod \"" + m +
                                "\" (expected aggressive/leafonly/none)");
    return true;
}

// $realignbones (Cmd_RealignBones): realign every single-child bone chain so
// each bone points at its child. Skips $definebone bones given a realign pair.
bool CmdRealignBones(Ctx& c, const Token&) {
    c.in.realignBones = true;
    return true;
}

// $lockbonelengths (Cmd_LockBoneLengths): stop source animation from stretching
// bones - every frame gets the bind-pose local translation back, and each ik
// chain is re-solved so its end bone stays where the animation put it.
bool CmdLockBoneLengths(Ctx& c, const Token&) {
    c.in.lockBoneLengths = true;
    return true;
}

// $transformbone <bone> [options]  (Cmd_TransformBindPoseBone)
//
// Edits a bone's bind pose late in the pipeline, after the global bone table is
// built - the merged replacement for stock's $rotatebone + $movebone. The
// options are order-independent:
//   angles <p y r>                          rotate the bind orientation
//   position <x y z>                        translate the bind position
//   worldangles / worldposition             use world axes instead of the bone's
//   transformweights <residualbone> [f] [s] re-skin ramp onto residualbone
//   offset <x y z>                          shift the transformweights ramp end
//   transformverts                          carry the rigged verts at rest
//   transformchildren                       move the bone's whole subtree with it
//   ignoreanimation                         never let the edit reach a clip
//   ignorehitbox                            keep this bone's $hbox in place
// Position-independent: the whole list is applied in one batch by the compile
// stage, so the bone is resolved there, not here.
bool CmdTransformBone(Ctx& c, const Token& cmd) {
    cm::BoneTransformEdit e;
    e.line = cmd.line;
    if (!c.Want("a bone name", cmd, e.name))
        return false;

    // an optional trailing number: consumed only when the token parses whole as
    // a float, so a following keyword is left for the option loop
    auto optFloat = [&](float& out) {
        if (c.AtCommand() || c.Cur().quoted)
            return;
        const std::string& s = c.Cur().text;
        try {
            size_t used = 0;
            const float v = std::stof(s, &used);
            if (used != s.size())
                return;
            out = std::min(std::max(v, 0.0f), 1.0f);
            c.pos++;
        } catch (const std::exception&) {
        }
    };

    while (!c.AtCommand()) {
        const Token& t = c.toks[c.pos++];
        const Token sub{cmd.text + " " + t.text, t.line, false};
        const char* o = t.text.c_str();

        if (!t.quoted && _stricmp(o, "angles") == 0) {
            if (!c.WantFloat("a pitch", sub, e.angles.x) ||
                !c.WantFloat("a yaw", sub, e.angles.y) ||
                !c.WantFloat("a roll", sub, e.angles.z))
                return false;
            e.hasAngles = true;
        } else if (!t.quoted && _stricmp(o, "position") == 0) {
            if (!c.WantFloat("an X offset", sub, e.pos.x) ||
                !c.WantFloat("a Y offset", sub, e.pos.y) ||
                !c.WantFloat("a Z offset", sub, e.pos.z))
                return false;
            e.hasPosition = true;
        } else if (!t.quoted && _stricmp(o, "worldangles") == 0) {
            e.worldAngles = true;
        } else if (!t.quoted && _stricmp(o, "worldposition") == 0) {
            e.worldPosition = true;
        } else if (!t.quoted && _stricmp(o, "transformweights") == 0) {
            if (!c.Want("a residual bone name", sub, e.residualbone))
                return false;
            e.hasMoveWeight = true;
            optFloat(e.moveWeightFactor);    // ramp band width, default 1.0
            optFloat(e.moveWeightSmoothing); // S-curve easing, default 0.0
        } else if (!t.quoted && _stricmp(o, "offset") == 0) {
            if (!c.WantFloat("an X offset", sub, e.moveWeightOffset.x) ||
                !c.WantFloat("a Y offset", sub, e.moveWeightOffset.y) ||
                !c.WantFloat("a Z offset", sub, e.moveWeightOffset.z))
                return false;
            e.hasMoveWeightOffset = true;
        } else if (!t.quoted && _stricmp(o, "transformverts") == 0) {
            e.transformVerts = true;
        } else if (!t.quoted && _stricmp(o, "transformchildren") == 0) {
            e.transformChildren = true;
        } else if (!t.quoted && _stricmp(o, "ignoreanimation") == 0) {
            e.ignoreAnimation = true;
        } else if (!t.quoted && _stricmp(o, "ignorehitbox") == 0) {
            e.ignoreHitbox = true;
        } else {
            return c.Fail(t.line, "$transformbone \"" + e.name +
                                  "\": expected angles, position, worldangles, "
                                  "worldposition, transformweights, offset, "
                                  "transformverts, transformchildren, "
                                  "ignoreanimation or ignorehitbox, "
                                  "got \"" + t.text + "\"");
        }
    }

    if (e.transformVerts && e.hasMoveWeight)
        return c.Fail(cmd.line,
                      "$transformbone: transformverts and transformweights are "
                      "mutually exclusive");
    if (e.hasMoveWeightOffset && !e.hasMoveWeight)
        return c.Fail(cmd.line, "$transformbone: offset requires transformweights");
    if (!e.hasAngles && !e.hasPosition)
        return c.Fail(cmd.line, "$transformbone: requires at least one of angles or position");

    c.in.boneTransformEdits.push_back(std::move(e));
    if (c.in.boneTransformEdits.size() >
        static_cast<size_t>(pulse::limits::kMaxBoneTransformEdits))
        return c.Fail(cmd.line, "too many $transformbone entries");
    return true;
}

// $root <bone> (Cmd_Root) - PLACEHOLDER: parsed and stored, nothing reads it.
bool CmdRoot(Ctx& c, const Token& cmd) {
    return c.Want("a bone name", cmd, c.in.primaryRootBone);
}

// $unlockdefinebones (Cmd_UnlockDefineBones): flips the default lock state for
// every $definebone after it. An unlocked bone is appended AFTER the source
// union instead of replacing a source bone's bind pose.
bool CmdUnlockDefineBones(Ctx& c, const Token&) {
    c.unlockDefineBones = true;
    return true;
}

// $definebone <name> <parent> [locked|unlocked] <x y z> <p y r>
//                             [<x y z> <p y r>]                (Cmd_DefineBone)
// The optional second sextet is the realign transform; supplying it marks the
// bone pre-aligned, so RealignBones leaves it alone.
bool CmdDefineBone(Ctx& c, const Token& cmd) {
    cm::ImportBone ib;
    if (!c.Want("a bone name", cmd, ib.name) ||
        !c.Want("a parent name (\"\" for a root bone)", cmd, ib.parent))
        return false;

    ib.bUnlocked = c.unlockDefineBones;
    if (!c.AtCommand() && !c.Cur().quoted) {
        if (_stricmp(c.Cur().text.c_str(), "unlocked") == 0) {
            ib.bUnlocked = true;
            c.pos++;
        } else if (_stricmp(c.Cur().text.c_str(), "locked") == 0) {
            ib.bUnlocked = false;
            c.pos++;
        }
    }

    // position + (pitch, yaw, roll) degrees -> a bind-pose matrix
    auto sextet = [&](pm::matrix3x4& out) {
        pm::Vector3 pos, rotDeg;
        if (!c.WantFloat("an X position", cmd, pos.x) ||
            !c.WantFloat("a Y position", cmd, pos.y) ||
            !c.WantFloat("a Z position", cmd, pos.z) ||
            !c.WantFloat("a pitch", cmd, rotDeg.x) ||
            !c.WantFloat("a yaw", cmd, rotDeg.y) ||
            !c.WantFloat("a roll", cmd, rotDeg.z))
            return false;
        pm::AngleMatrixDeg(rotDeg, out);
        out.m[0][3] = pos.x;
        out.m[1][3] = pos.y;
        out.m[2][3] = pos.z;
        return true;
    };

    if (!sextet(ib.rawLocal))
        return false;
    if (!c.AtCommand()) {
        ib.bPreAligned = true;
        if (!sextet(ib.srcRealign))
            return false;
    }

    c.in.importbones.push_back(std::move(ib));
    if (c.in.importbones.size() > static_cast<size_t>(pulse::limits::kMaxBones))
        return c.Fail(cmd.line, "too many $definebone entries");
    return true;
}

// The markup entries share one BoneMarkup per bone, so $bonemerge and
// $donotcollapse can both name it.
cm::BoneMarkup& FindOrAddMarkup(Ctx& c, const std::string& name) {
    for (auto& bm : c.in.bonemarkups)
        if (_stricmp(bm.name.c_str(), name.c_str()) == 0)
            return bm;
    c.in.bonemarkups.push_back(cm::BoneMarkup{name});
    return c.in.bonemarkups.back();
}

// $bonemerge <bone> (Cmd_BoneMerge): tag BONE_USED_BY_BONE_MERGE so the bone
// survives the cull and a bonemerged child model can find it.
bool CmdBoneMerge(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a bone name", cmd, name))
        return false;
    FindOrAddMarkup(c, name).isBonemerge = true;
    return true;
}

// $donotcollapse <bone> (Cmd_DoNotCollapse): force-keep the bone. Beats
// $alwayscollapse.
bool CmdDoNotCollapse(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a bone name", cmd, name))
        return false;
    FindOrAddMarkup(c, name).doNotCollapse = true;
    return true;
}

// $alwayscollapse <bone> (Cmd_AlwaysCollapse): force-collapse the bone even
// when something would normally keep it.
bool CmdAlwaysCollapse(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a bone name", cmd, name))
        return false;
    c.in.alwaysCollapse.push_back(std::move(name));
    return true;
}

// $hierarchy <child> <parent> (Cmd_ForcedHierarchy, "$heirarchy" is stock's own
// misspelling and works too): reparent a bone. Applied once the global bone
// table exists, so it can join two source skeletons that were exported apart.
// A quoted empty parent makes the child a root bone.
bool CmdHierarchy(Ctx& c, const Token& cmd) {
    cm::CompileInput::ForcedHierarchy fh;
    if (!c.Want("a bone name", cmd, fh.child) ||
        !c.Want("a parent bone name", cmd, fh.parent))
        return false;
    c.in.forcedHierarchy.push_back(std::move(fh));
    return true;
}

// ---------------------------------------------------------------------------
// Procedural bones - $jigglebone, $driverbone and the legacy $proceduralbones
// VRD file. The driven bone is tagged BONE_ALWAYS_PROCEDURAL by the compile
// stage: the engine owns its pose, so it survives the collapse and carries no
// animation data.
// ---------------------------------------------------------------------------

// The reference converts through DOUBLE (`verify_atof(token) * M_PI / 180.0f`).
// Rounding a float pi/180 instead is off by 1 ULP on values like 35 deg, and
// that lands in the .mdl.
float DegToRad(float deg) {
    return static_cast<float>(deg * 3.14159265358979323846 / 180.0);
}

bool WantOpenBrace(Ctx& c, const Token& cmd, const std::string& what) {
    if (c.AtCommand() || c.Cur().quoted || c.Cur().text != "{")
        return c.Fail(cmd.line, what + ": missing '{'");
    c.pos++;
    return true;
}

bool WantDegrees(Ctx& c, const Token& cmd, float& outRad) {
    float deg = 0.0f;
    if (!c.WantFloat("an angle in degrees", cmd, deg))
        return false;
    outRad = DegToRad(deg);
    return true;
}

// reference ParseJiggleStiffness: stiffness AND damping both clamp to [0,1000]
// (ParseJiggleDamping's tighter range is never actually called)
bool WantStiffness(Ctx& c, const Token& cmd, float& out) {
    float v = 0.0f;
    if (!c.WantFloat("a stiffness value", cmd, v))
        return false;
    out = v < 0.0f ? 0.0f : (v > 1000.0f ? 1000.0f : v);
    return true;
}

// ParseCommonJiggle - the options every subsection but `is_boing` accepts.
// 1 = consumed, 0 = not a common option, -1 = parse error.
int ParseCommonJiggle(Ctx& c, const Token& cmd, const std::string& o, cm::JiggleBone& jb) {
    auto num = [&](const char* what, float& dst) { return c.WantFloat(what, cmd, dst) ? 1 : -1; };
    if (o == "tip_mass") return num("a tip mass", jb.tipMass);
    if (o == "length") return num("a length", jb.length);
    if (o == "yaw_friction") return num("a yaw friction", jb.yawFriction);
    if (o == "yaw_bounce") return num("a yaw bounce", jb.yawBounce);
    if (o == "pitch_friction") return num("a pitch friction", jb.pitchFriction);
    if (o == "pitch_bounce") return num("a pitch bounce", jb.pitchBounce);
    if (o == "angle_constraint") {
        jb.flags |= cm::kJiggleHasAngleConstraint;
        return WantDegrees(c, cmd, jb.angleLimit) ? 1 : -1;
    }
    if (o == "yaw_constraint") {
        jb.flags |= cm::kJiggleHasYawConstraint;
        return WantDegrees(c, cmd, jb.minYaw) && WantDegrees(c, cmd, jb.maxYaw) ? 1 : -1;
    }
    if (o == "pitch_constraint") {
        jb.flags |= cm::kJiggleHasPitchConstraint;
        return WantDegrees(c, cmd, jb.minPitch) && WantDegrees(c, cmd, jb.maxPitch) ? 1 : -1;
    }
    return 0;
}

// one is_flexible / is_rigid / has_base_spring / is_boing block. The flags the
// section itself implies are set by the caller, before the '{'.
bool ParseJiggleSection(Ctx& c, const Token& cmd, const std::string& sect, cm::JiggleBone& jb) {
    const std::string where = "$jigglebone:" + sect;
    if (!WantOpenBrace(c, cmd, where))
        return false;

    const bool boing = sect == "is_boing";
    const bool flexible = sect == "is_flexible";
    const bool baseSpring = sect == "has_base_spring";

    while (true) {
        if (c.AtCommand())
            return c.Fail(cmd.line, where + ": missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            return true;
        const std::string o = Lower(t.text);
        const Token sub{where + " " + t.text, t.line, false};

        bool handled = true;
        if (flexible) {
            if (o == "yaw_stiffness") { if (!WantStiffness(c, sub, jb.yawStiffness)) return false; }
            else if (o == "yaw_damping") { if (!WantStiffness(c, sub, jb.yawDamping)) return false; }
            else if (o == "pitch_stiffness") { if (!WantStiffness(c, sub, jb.pitchStiffness)) return false; }
            else if (o == "pitch_damping") { if (!WantStiffness(c, sub, jb.pitchDamping)) return false; }
            else if (o == "along_stiffness") { if (!WantStiffness(c, sub, jb.alongStiffness)) return false; }
            else if (o == "along_damping") { if (!WantStiffness(c, sub, jb.alongDamping)) return false; }
            else if (o == "allow_length_flex") jb.flags &= ~cm::kJiggleHasLengthConstraint;
            else handled = false;
        } else if (baseSpring) {
            if (o == "stiffness") { if (!WantStiffness(c, sub, jb.baseStiffness)) return false; }
            else if (o == "damping") { if (!WantStiffness(c, sub, jb.baseDamping)) return false; }
            else if (o == "base_mass") { if (!c.WantFloat("a base mass", sub, jb.baseMass)) return false; }
            else if (o == "left_constraint") {
                if (!c.WantFloat("a minimum", sub, jb.baseMinLeft) ||
                    !c.WantFloat("a maximum", sub, jb.baseMaxLeft)) return false;
            } else if (o == "left_friction") {
                if (!c.WantFloat("a friction", sub, jb.baseLeftFriction)) return false;
            } else if (o == "up_constraint") {
                if (!c.WantFloat("a minimum", sub, jb.baseMinUp) ||
                    !c.WantFloat("a maximum", sub, jb.baseMaxUp)) return false;
            } else if (o == "up_friction") {
                if (!c.WantFloat("a friction", sub, jb.baseUpFriction)) return false;
            } else if (o == "forward_constraint") {
                if (!c.WantFloat("a minimum", sub, jb.baseMinForward) ||
                    !c.WantFloat("a maximum", sub, jb.baseMaxForward)) return false;
            } else if (o == "forward_friction") {
                if (!c.WantFloat("a friction", sub, jb.baseForwardFriction)) return false;
            } else handled = false;
        } else if (boing) {
            if (o == "impact_speed") { if (!c.WantFloat("a speed", sub, jb.boingImpactSpeed)) return false; }
            else if (o == "impact_angle") { // stored as a cosine
                float deg = 0.0f;
                if (!c.WantFloat("an angle in degrees", sub, deg)) return false;
                jb.boingImpactAngle = std::cos(DegToRad(deg));
            }
            else if (o == "damping_rate") { if (!c.WantFloat("a damping rate", sub, jb.boingDampingRate)) return false; }
            else if (o == "frequency") { if (!c.WantFloat("a frequency", sub, jb.boingFrequency)) return false; }
            else if (o == "amplitude") { if (!c.WantFloat("an amplitude", sub, jb.boingAmplitude)) return false; }
            else handled = false;
        } else {
            handled = false; // is_rigid takes only the common options
        }

        // is_boing is the one section that does NOT fall through to the common
        // options (reference ParseBoingJiggle errors instead)
        if (!handled && !boing) {
            const int r = ParseCommonJiggle(c, sub, o, jb);
            if (r < 0) return false;
            handled = r > 0;
        }
        if (!handled)
            return c.Fail(t.line, where + ": invalid syntax \"" + t.text + "\"");
    }
}

// $jigglebone <bone> { is_flexible|is_rigid|has_base_spring|is_boing { ... } }
// (Cmd_JiggleBone). Re-declaring a bone overwrites its definition in place.
bool CmdJiggleBone(Ctx& c, const Token& cmd) {
    cm::JiggleBone jb;
    if (!c.Want("a bone name", cmd, jb.bonename))
        return false;
    if (!WantOpenBrace(c, cmd, "$jigglebone"))
        return false;

    while (true) {
        if (c.AtCommand())
            return c.Fail(cmd.line, "$jigglebone: missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        const std::string sect = Lower(t.text);
        if (sect == "is_flexible")
            jb.flags |= cm::kJiggleIsFlexible | cm::kJiggleHasLengthConstraint;
        else if (sect == "is_rigid")
            jb.flags |= cm::kJiggleIsRigid | cm::kJiggleHasLengthConstraint;
        else if (sect == "has_base_spring")
            jb.flags |= cm::kJiggleHasBaseSpring;
        else if (sect == "is_boing") {
            jb.flags |= cm::kJiggleIsBoing;
            // seeded only here, matching where ParseBoingJiggle seeds them - a
            // non-boing jigglebone writes zeros. 0.7071f is the reference's
            // literal, NOT cos(45 deg), and the two differ in the .mdl bytes.
            jb.boingImpactSpeed = 100.0f;
            jb.boingImpactAngle = 0.7071f;
            jb.boingDampingRate = 0.25f;
            jb.boingFrequency = 30.0f;
            jb.boingAmplitude = 0.35f;
        } else
            return c.Fail(t.line, "$jigglebone: invalid syntax \"" + t.text + "\"");

        if (!ParseJiggleSection(c, cmd, sect, jb))
            return false;
    }

    for (cm::JiggleBone& prev : c.in.jigglebones) {
        if (_stricmp(prev.bonename.c_str(), jb.bonename.c_str()) == 0) {
            prev = std::move(jb); // last definition wins, in place
            return true;
        }
    }
    c.in.jigglebones.push_back(std::move(jb));
    if (c.in.jigglebones.size() > static_cast<size_t>(pulse::limits::kMaxJiggleBones))
        return c.Fail(cmd.line, "too many jigglebones");
    return true;
}

// Append one trigger. Rotations arrive in degrees, the position in model units.
// The writer stores 1/tolerance, so a zero tolerance would put an inf on disk -
// the reference divides unguarded, we refuse.
bool AddTrigger(Ctx& c, const Token& cmd, cm::ProceduralBone& pb, float tolDeg,
                const pm::Vector3& driverRotDeg, const pm::Vector3& helperRotDeg,
                const pm::Vector3& pos) {
    if (tolDeg <= 0.0f)
        return c.Fail(cmd.line, "trigger tolerance must be > 0");

    cm::ProceduralBoneTrigger tr;
    tr.tolerance = DegToRad(tolDeg);
    pm::AngleQuaternion({DegToRad(driverRotDeg.x), DegToRad(driverRotDeg.y),
                         DegToRad(driverRotDeg.z)}, tr.trigger);
    pm::AngleQuaternion({DegToRad(helperRotDeg.x), DegToRad(helperRotDeg.y),
                         DegToRad(helperRotDeg.z)}, tr.quat);
    // scaled at parse time, where the reference applies g_currentscale
    tr.pos = {pos.x * c.in.scale, pos.y * c.in.scale, pos.z * c.in.scale};

    // hard cap: the .mdl reader walks exactly numtriggers entries. The
    // reference truncates with a warning rather than failing, as does the
    // .pulsemdl loader - both front ends must land in the same place.
    if (pb.triggers.size() >= static_cast<size_t>(pulse::limits::kMaxProceduralTriggers)) {
        std::fprintf(stderr, "animconstraint \"%s\": more than %d triggers; "
                             "dropping the rest\n",
                     pb.helpername.c_str(), pulse::limits::kMaxProceduralTriggers);
        return true;
    }
    pb.triggers.push_back(tr);
    return true;
}

// $driverbone <helper> <driver> [relative|absolute] {
//     basepos <x y z>                                        optional, default 0
//     trigger <tolerance> <driver rot x y z> <helper rot x y z> <helper pos x y z>
//     ...
// }
// The inline replacement for a VRD quatinterp helper. Both rotations are
// RadianEuler degrees (x=roll, y=pitch, z=yaw), NOT a QAngle - they go into
// AngleQuaternion unpermuted, exactly like the <trigger> line they mirror.
//
// `relative` (the default) reads the helper pose as a DELTA from its bind pose,
// which MapProceduralBones folds in once the skeleton is final - so an all-zero
// trigger leaves the bone at rest and basepos is a plain shared offset.
// `absolute` is the VRD's own convention: the pose is parent-relative in full,
// so an all-zero trigger puts the bone AT its parent's origin and basepos has
// to carry the bind pose. It exists so a VRD can be transcribed verbatim.
bool CmdDriverBone(Ctx& c, const Token& cmd) {
    cm::ProceduralBone pb;
    if (!c.Want("a helper bone name", cmd, pb.helpername) ||
        !c.Want("a driver bone name", cmd, pb.drivername))
        return false;

    // anything between the names and the '{' has to be the mode - a typo here
    // would otherwise surface as a confusing "missing '{'"
    if (!c.AtCommand() && !c.Cur().quoted && c.Cur().text != "{") {
        const std::string mode = Lower(c.Cur().text);
        if (mode != "relative" && mode != "absolute")
            return c.Fail(c.Cur().line, "$driverbone: expected relative, absolute "
                                        "or '{', got \"" + c.Cur().text + "\"");
        pb.absolutePose = mode == "absolute";
        c.pos++;
    }
    if (!WantOpenBrace(c, cmd, "$driverbone"))
        return false;

    // held raw until the block closes, so basepos is order-independent
    struct Raw {
        Token at;
        float tol = 0.0f;
        pm::Vector3 driverRot, helperRot, pos;
    };
    std::vector<Raw> raw;
    pm::Vector3 basepos{};

    while (true) {
        if (c.AtCommand())
            return c.Fail(cmd.line, "$driverbone: missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        const std::string o = t.quoted ? std::string() : Lower(t.text);

        if (o == "basepos") {
            const Token sub{"$driverbone basepos", t.line, false};
            if (!c.WantFloat("an X offset", sub, basepos.x) ||
                !c.WantFloat("a Y offset", sub, basepos.y) ||
                !c.WantFloat("a Z offset", sub, basepos.z))
                return false;
            continue;
        }
        if (o != "trigger")
            return c.Fail(t.line, "$driverbone: expected trigger, basepos or "
                                  "'}', got \"" + t.text + "\"");

        const Token sub{"$driverbone trigger", t.line, false};
        Raw r{sub};
        if (!c.WantFloat("a tolerance in degrees", sub, r.tol) ||
            !c.WantFloat("a driver roll", sub, r.driverRot.x) ||
            !c.WantFloat("a driver pitch", sub, r.driverRot.y) ||
            !c.WantFloat("a driver yaw", sub, r.driverRot.z) ||
            !c.WantFloat("a helper roll", sub, r.helperRot.x) ||
            !c.WantFloat("a helper pitch", sub, r.helperRot.y) ||
            !c.WantFloat("a helper yaw", sub, r.helperRot.z) ||
            !c.WantFloat("a helper X offset", sub, r.pos.x) ||
            !c.WantFloat("a helper Y offset", sub, r.pos.y) ||
            !c.WantFloat("a helper Z offset", sub, r.pos.z))
            return false;
        raw.push_back(std::move(r));
    }

    if (raw.empty())
        return c.Fail(cmd.line, "$driverbone \"" + pb.helpername + "\": no triggers");

    // emitted after the block so basepos applies wherever it was written. Added
    // before the scale, like the VRD's `(basepos + pos) * g_currentscale`.
    for (const Raw& r : raw) {
        const pm::Vector3 pos{basepos.x + r.pos.x, basepos.y + r.pos.y,
                              basepos.z + r.pos.z};
        if (!AddTrigger(c, r.at, pb, r.tol, r.driverRot, r.helperRot, pos))
            return false;
    }
    c.in.proceduralbones.push_back(std::move(pb));
    if (c.in.proceduralbones.size() > static_cast<size_t>(pulse::limits::kMaxProceduralBones))
        return c.Fail(cmd.line, "too many procedural bones");
    return true;
}

// $driveraimat <bone> <target> <up x y z> <aim x y z>
//
// The engine rotates <bone> so its aim vector points at <target>, which is an
// attachment name if one matches and otherwise a bone name. The inline form of
// a VRD <aimconstraint>; the parent and the base position both come from the
// skeleton, so neither is asked for.
bool CmdDriverAimAt(Ctx& c, const Token& cmd) {
    cm::AimAtBone ab;
    ab.autobasepos = true;
    if (!c.Want("a bone name", cmd, ab.bonename) ||
        !c.Want("an aim target attachment or bone name", cmd, ab.aimname) ||
        !c.WantFloat("an up vector X", cmd, ab.upvector.x) ||
        !c.WantFloat("an up vector Y", cmd, ab.upvector.y) ||
        !c.WantFloat("an up vector Z", cmd, ab.upvector.z) ||
        !c.WantFloat("an aim vector X", cmd, ab.aimvector.x) ||
        !c.WantFloat("an aim vector Y", cmd, ab.aimvector.y) ||
        !c.WantFloat("an aim vector Z", cmd, ab.aimvector.z))
        return false;
    // unit length on read, like Grab_AimAtBones (a zero vector stays zero -
    // that is what VectorNormalize's epsilon divide is for)
    pm::VectorNormalize(ab.upvector);
    pm::VectorNormalize(ab.aimvector);

    c.in.aimatbones.push_back(std::move(ab));
    if (c.in.aimatbones.size() > static_cast<size_t>(pulse::limits::kMaxProceduralBones))
        return c.Fail(cmd.line, "too many procedural bones");
    return true;
}

// $proceduralbones "file.vrd" (Load_ProceduralBones -> Grab_QuatInterpBones).
// The legacy file form of $driverbone and $driveraimat. Unlike $driverbone the
// quatinterp poses here are ABSOLUTE (parent-relative), so the bind-pose fold
// is skipped downstream.
//
//   <helper>   <bone> <parent> <controlparent> <control>
//   <basepos>  x y z          added to every LATER trigger position
//   <rotateaxis> / <jointorient>  x y z degrees, pre/post-rotate the pose
//   <trigger>  tol  trigX trigY trigZ  angX angY angZ  posX posY posZ
//   <display>  ignored - it is modelling-tool decoration
//
// An <aimconstraint> opens an aim-at block instead ($driveraimat's file form).
// Its <aimvector>/<upvector>/<basepos> lines belong to that block, so <basepos>
// means the aimat's own base position there and the file-scoped one everywhere
// else - the reference gets the same split by handing the file to
// Grab_AimAtBones until it hits a line it does not recognize.
//
//   <aimconstraint> <bone> <parent> <aimname>
//   <aimvector> / <upvector>  x y z    (normalized on read)
//   <basepos>   x y z
bool CmdProceduralBones(Ctx& c, const Token& cmd) {
    std::string filename;
    if (!c.Want("a .vrd file name", cmd, filename))
        return false;
    // the non-.vrd branch of Load_ProceduralBones reads axis-interp bones, a
    // different procedural type with no backend here
    if (_stricmp(fs::path(filename).extension().string().c_str(), ".vrd") != 0)
        return c.Fail(cmd.line, "$proceduralbones expects a .vrd file, got \"" +
                                filename + "\"");

    std::vector<fs::path> tried;
    const fs::path full = FindSourceFile(c, filename, &tried);
    if (full.empty())
        return c.Fail(cmd.line, "cannot find \"" + filename + "\" - looked in:" +
                                LookedIn(tried));
    std::ifstream f(full.string(), std::ios::binary);
    if (!f)
        return c.Fail(cmd.line, "cannot open \"" + full.string() + "\"");

    cm::ProceduralBone* cur = nullptr;
    // set only INSIDE an <aimconstraint> block - see the header comment
    size_t curAim = static_cast<size_t>(-1);
    const size_t aimsBefore = c.in.aimatbones.size();
    // file-scoped, exactly like Grab_QuatInterpBones' locals: they persist
    // across <helper> lines and apply to every trigger that follows
    pm::Vector3 basepos{};
    pm::Vector3 rotateaxisDeg{}, jointorientDeg{};

    std::string line;
    int lineno = 0;
    while (std::getline(f, line)) {
        lineno++;
        // a VRD line is whitespace-delimited; names may be quoted (ScanVrdLine)
        std::vector<std::string> tok;
        for (size_t i = 0; i < line.size();) {
            if (std::isspace(static_cast<unsigned char>(line[i]))) { i++; continue; }
            std::string s;
            if (line[i] == '"') {
                for (i++; i < line.size() && line[i] != '"'; i++) s += line[i];
                if (i < line.size()) i++;
            } else {
                for (; i < line.size() && !std::isspace(static_cast<unsigned char>(line[i])); i++)
                    s += line[i];
            }
            tok.push_back(std::move(s));
        }
        if (tok.empty() || tok[0].empty())
            continue;
        // comments are not part of the format, but the fixtures carry them
        if (tok[0].compare(0, 2, "//") == 0)
            continue;

        const Token at{"$proceduralbones \"" + filename + "\" line " +
                       std::to_string(lineno), cmd.line, false};
        const std::string kw = Lower(tok[0]);
        auto num = [&](size_t i, float& out) {
            try {
                size_t used = 0;
                out = std::stof(tok[i], &used);
                return used == tok[i].size();
            } catch (const std::exception&) { return false; }
        };
        auto vec = [&](size_t i, pm::Vector3& out) {
            return num(i, out.x) && num(i + 1, out.y) && num(i + 2, out.z);
        };

        // only the three aim-at value lines stay inside an <aimconstraint>
        // block; anything else ends it (Grab_AimAtBones returns)
        if (kw != "<aimvector>" && kw != "<upvector>" && kw != "<basepos>")
            curAim = static_cast<size_t>(-1);

        if (kw == "<helper>") {
            if (tok.size() < 5)
                return c.Fail(cmd.line, at.text + ": <helper> expects "
                                        "<bone> <parent> <controlparent> <control>");
            c.in.proceduralbones.push_back(cm::ProceduralBone{});
            cur = &c.in.proceduralbones.back();
            cur->helpername = tok[1];
            cur->helperparentname = tok[2];
            cur->driverparentname = tok[3];
            cur->drivername = tok[4];
            cur->absolutePose = true;
        } else if (kw == "<aimconstraint>") {
            if (tok.size() < 4)
                return c.Fail(cmd.line, at.text + ": <aimconstraint> expects "
                                        "<bone> <parent> <aimname>");
            cm::AimAtBone ab;
            ab.bonename = tok[1];
            ab.parentname = tok[2];
            ab.aimname = tok[3];
            c.in.aimatbones.push_back(std::move(ab));
            if (c.in.aimatbones.size() > static_cast<size_t>(pulse::limits::kMaxProceduralBones))
                return c.Fail(cmd.line, at.text + ": too many procedural bones");
            curAim = c.in.aimatbones.size() - 1;
        } else if (kw == "<aimvector>" || kw == "<upvector>") {
            if (curAim == static_cast<size_t>(-1))
                return c.Fail(cmd.line, at.text + ": " + tok[0] +
                                        " outside an <aimconstraint> block");
            pm::Vector3 v;
            if (tok.size() < 4 || !vec(1, v))
                return c.Fail(cmd.line, at.text + ": " + tok[0] + " expects x y z");
            pm::VectorNormalize(v);
            if (kw == "<aimvector>")
                c.in.aimatbones[curAim].aimvector = v;
            else
                c.in.aimatbones[curAim].upvector = v;
        } else if (kw == "<basepos>") {
            pm::Vector3 v;
            if (tok.size() < 4 || !vec(1, v))
                return c.Fail(cmd.line, at.text + ": <basepos> expects x y z");
            // inside an aim-at block this is that bone's own base position, not
            // the quatinterp trigger offset
            if (curAim != static_cast<size_t>(-1))
                c.in.aimatbones[curAim].basepos = v;
            else
                basepos = v;
        } else if (kw == "<rotateaxis>") {
            if (tok.size() < 4 || !vec(1, rotateaxisDeg))
                return c.Fail(cmd.line, at.text + ": <rotateaxis> expects x y z");
        } else if (kw == "<jointorient>") {
            if (tok.size() < 4 || !vec(1, jointorientDeg))
                return c.Fail(cmd.line, at.text + ": <jointorient> expects x y z");
        } else if (kw == "<display>") {
            // size + distance, modelling-tool decoration - nothing reads it
        } else if (kw == "<trigger>") {
            if (!cur)
                return c.Fail(cmd.line, at.text + ": <trigger> before any <helper>");
            float tol = 0.0f;
            pm::Vector3 driverRot, helperRot, pos;
            if (tok.size() < 11 || !num(1, tol) || !vec(2, driverRot) ||
                !vec(5, helperRot) || !vec(8, pos))
                return c.Fail(cmd.line, at.text + ": <trigger> expects tolerance + "
                                        "9 floats");
            // basepos is added before the scale, like the reference's
            // `(basepos + pos) * g_currentscale`
            pos = {basepos.x + pos.x, basepos.y + pos.y, basepos.z + pos.z};
            if (!AddTrigger(c, at, *cur, tol, driverRot, helperRot, pos))
                return false;

            // rotateaxis pre-rotates the helper pose, jointorient post-rotates it
            pm::Quaternion& q = cur->triggers.back().quat;
            if (rotateaxisDeg.x != 0.0f || rotateaxisDeg.y != 0.0f || rotateaxisDeg.z != 0.0f) {
                pm::Quaternion q1, q2;
                pm::AngleQuaternion({DegToRad(rotateaxisDeg.x), DegToRad(rotateaxisDeg.y),
                                     DegToRad(rotateaxisDeg.z)}, q1);
                pm::QuaternionMult(q1, q, q2);
                q = q2;
            }
            if (jointorientDeg.x != 0.0f || jointorientDeg.y != 0.0f || jointorientDeg.z != 0.0f) {
                pm::Quaternion q1, q2;
                pm::AngleQuaternion({DegToRad(jointorientDeg.x), DegToRad(jointorientDeg.y),
                                     DegToRad(jointorientDeg.z)}, q1);
                pm::QuaternionMult(q, q1, q2);
                q = q2;
            }
        } else {
            return c.Fail(cmd.line, at.text + ": unknown \"" + tok[0] + "\"");
        }
    }

    if (!cur && c.in.aimatbones.size() == aimsBefore)
        return c.Fail(cmd.line, "\"" + filename + "\" has no <helper> or "
                                "<aimconstraint>");
    return true;
}

bool CmdAmbientBoost(Ctx& c, const Token&) {
    c.in.ambientBoost = true;
    return true;
}

// $donotcastshadows (Cmd_DoNotCastShadows): sets
// STUDIOHDR_FLAGS_DO_NOT_CAST_SHADOWS, no arguments.
bool CmdDoNotCastShadows(Ctx& c, const Token&) {
    c.in.doNotCastShadows = true;
    return true;
}

// $forcephonemecrossfade (Cmd_ForcePhonemeCrossfade): sets
// STUDIOHDR_FLAGS_FORCE_PHONEME_CROSSFADE so
// the viseme check always spans two phonemes. No arguments.
bool CmdForcePhonemeCrossfade(Ctx& c, const Token&) {
    c.in.forcePhonemeCrossfade = true;
    return true;
}

// $skipboneinbbox (Cmd_SkipBoneInBBox): clears
// useBoneInBBox, so the auto-generated hitboxes (and the sequence boxes derived
// from them) grow from the bone's vertices alone instead of always containing
// the bone origin. No arguments, no effect when $hitboxset is authored.
bool CmdSkipBoneInBBox(Ctx& c, const Token&) {
    c.in.skipBoneInBBox = true;
    return true;
}

// $bbox <minx miny minz> <maxx maxy maxz> (Cmd_BBox): pins the render/culling
// hull instead of taking sequence 0's box. Six raw floats - the reference
// applies neither $scale nor the model rotation to them.
bool CmdBBox(Ctx& c, const Token& cmd) {
    if (!c.WantFloat("a min x", cmd, c.in.bbox[0].x) ||
        !c.WantFloat("a min y", cmd, c.in.bbox[0].y) ||
        !c.WantFloat("a min z", cmd, c.in.bbox[0].z) ||
        !c.WantFloat("a max x", cmd, c.in.bbox[1].x) ||
        !c.WantFloat("a max y", cmd, c.in.bbox[1].y) ||
        !c.WantFloat("a max z", cmd, c.in.bbox[1].z))
        return false;
    c.in.bboxSet = true;
    return true;
}

// $cbox <minx miny minz> <maxx maxy maxz> (Cmd_CBox): the view clipping box.
// Zero unless given, which is what every stock model ships.
bool CmdCBox(Ctx& c, const Token& cmd) {
    if (!c.WantFloat("a min x", cmd, c.in.cbox[0].x) ||
        !c.WantFloat("a min y", cmd, c.in.cbox[0].y) ||
        !c.WantFloat("a min z", cmd, c.in.cbox[0].z) ||
        !c.WantFloat("a max x", cmd, c.in.cbox[1].x) ||
        !c.WantFloat("a max y", cmd, c.in.cbox[1].y) ||
        !c.WantFloat("a max z", cmd, c.in.cbox[1].z))
        return false;
    c.in.cboxSet = true;
    return true;
}

// $illumposition <x> <y> <z> [bone <name>] (Cmd_Illumposition): the point the
// engine samples the lighting environment at, instead of sequence 0's box
// center. Neither form is scaled by $transformmodel - like $bbox the numbers
// are taken as written.
//
// Bone-less, the point is static and gets the reference's source -> model
// swizzle, which is the default rotatemodel (+90 yaw) applied to it. With a
// bone the numbers are bone space and the command also emits a rigid
// "__illumPosition" attachment, which the header then points at so the
// position follows that bone.
bool CmdIllumPosition(Ctx& c, const Token& cmd) {
    if (c.in.illumpositionSet)
        return c.Fail(cmd.line, "duplicate $illumposition");

    pm::Vector3 pos;
    if (!c.WantFloat("an X position", cmd, pos.x) ||
        !c.WantFloat("a Y position", cmd, pos.y) ||
        !c.WantFloat("a Z position", cmd, pos.z))
        return false;

    std::string bone;
    while (!c.AtCommand()) {
        const Token& t = c.toks[c.pos++];
        if (t.quoted || _stricmp(t.text.c_str(), "bone") != 0)
            return c.Fail(t.line, "$illumposition: expected bone, got \"" +
                                  t.text + "\"");
        const Token sub{cmd.text + " " + t.text, t.line, false};
        if (!c.Want("a bone name", sub, bone))
            return false;
    }

    c.in.illumposition = pos;
    c.in.illumpositionSet = true;

    if (bone.empty()) {
        c.in.illumposition = {-pos.y, pos.x, pos.z};
        return true;
    }

    Ctx::PendingAttachment a;
    a.name = "__illumPosition";
    a.bone = bone;
    a.origin = pos;
    a.type = cm::kAttachIsRigid;
    a.noscale = true;
    c.in.illumpositionAttachment = a.name;
    c.attachments.push_back(std::move(a));
    return true;
}

// $eyeposition <x> <y> <z> (Cmd_Eyeposition): the ideal eye point, used by the
// engine to aim eyes and by tools as the view origin. Same source -> model
// swizzle as the static $illumposition, but this one IS scaled by $scale,
// which the compile stage applies.
bool CmdEyePosition(Ctx& c, const Token& cmd) {
    pm::Vector3 pos;
    if (!c.WantFloat("an X position", cmd, pos.x) ||
        !c.WantFloat("a Y position", cmd, pos.y) ||
        !c.WantFloat("a Z position", cmd, pos.z))
        return false;
    c.in.eyeposition = {-pos.y, pos.x, pos.z};
    return true;
}

// $maxeyedeflection <degrees> (Cmd_MaxEyeDeflection): how far off center the
// eyes may turn before the head takes over. Stored as its cosine; 0 means the
// engine's own cos(30) default.
bool CmdMaxEyeDeflection(Ctx& c, const Token& cmd) {
    float deg;
    if (!c.WantFloat("an angle in degrees", cmd, deg))
        return false;
    c.in.maxEyeDeflection = std::cos(DegToRad(deg));
    return true;
}

// $cdmaterials "path" ["path" ...] - takes every path up to the next command.
bool CmdCdMaterials(Ctx& c, const Token& cmd) {
    std::string first;
    if (!c.Want("a material path", cmd, first))
        return false;
    c.in.cdmaterials.push_back(first);
    while (!c.AtCommand())
        c.in.cdmaterials.push_back(c.toks[c.pos++].text);
    return true;
}

// One $set body: `material <replaced> <replacement>` lines until '}'.
bool ParseTextureSet(Ctx& c, const Token& cmd, std::vector<cm::CompileInput::SkinReplace>& fam) {
    if (!WantOpenBrace(c, cmd, "$texturegroup $set"))
        return false;

    while (true) {
        if (c.Eof())
            return c.Fail(cmd.line, "$texturegroup $set: missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            return true;
        if (t.quoted || _stricmp(t.text.c_str(), "material") != 0)
            return c.Fail(t.line, "$texturegroup $set: expected material or '}', got \"" +
                                  t.text + "\"");

        const Token sub{"$texturegroup $set material", t.line, false};
        // a brace is not a material name - Want would happily eat the '}' and
        // report the miscount somewhere else
        auto wantName = [&](const char* what, std::string& dst) {
            if (!c.AtCommand() && !c.Cur().quoted && (c.Cur().text == "}" || c.Cur().text == "{"))
                return c.Fail(t.line, sub.text + " expects " + what);
            return c.Want(what, sub, dst);
        };

        cm::CompileInput::SkinReplace r;
        if (!wantName("a material to replace", r.from) ||
            !wantName("a replacement material", r.to))
            return false;
        fam.push_back(std::move(r));
    }
}

// ---------------------------------------------------------------------------
// $lod / $shadowlod (reference Cmd_LOD)
// ---------------------------------------------------------------------------

// Path- and extension-stripped name, the comparison form every LOD command
// uses for a model (reference GetModelBaseName).
std::string ModelBaseName(const std::string& s) {
    const std::string noExt = StripExtension(s);
    const size_t slash = noExt.find_last_of("/\\");
    return slash == std::string::npos ? noExt : noExt.substr(slash + 1);
}

// Resolve a model name written in replacemodel/decimatemodel to the source it
// names: a $rendermesh alias first, then any loaded source's base filename.
source::Source* FindModelSource(Ctx& c, const std::string& name) {
    auto it = c.rendermeshes.find(name);
    if (it != c.rendermeshes.end())
        return it->second;
    const std::string want = ModelBaseName(name);
    for (auto& sp : c.in.sources)
        // only a Model-kind source is drawable: an animation source has no
        // geometry at all, and a collision source's materials index a private
        // table, so neither can stand in as LOD geometry
        if (sp->kind == source::LoadKind::Model &&
            _stricmp(ModelBaseName(sp->filename).c_str(), want.c_str()) == 0)
            return sp.get();
    return nullptr;
}

// replacemodel <model> <rendermesh-or-file> - this LOD draws different geometry
bool ParseReplaceModel(Ctx& c, const Token& cmd, cm::ScriptLod& lod) {
    cm::LodReplacement r;
    if (!c.Want("a model name", cmd, r.src) ||
        !c.Want("a replacement mesh", cmd, r.dst))
        return false;

    // "blank" means the LOD system was told to replace nothing - forget the
    // entry entirely (reference Cmd_ReplaceModel)
    if (_stricmp(r.src.c_str(), "blank") == 0)
        return true;

    r.srcSource = FindModelSource(c, r.src);
    if (!r.srcSource)
        return c.Fail(cmd.line, "replacemodel: unknown model \"" + r.src + "\"");

    auto it = c.rendermeshes.find(r.dst);
    r.source = it != c.rendermeshes.end()
                   ? it->second
                   : LoadSource(c, WithSourceExtension(c, r.dst), cmd.line, /*morphSource=*/true);
    if (!r.source)
        return false;

    lod.modelReplacements.push_back(std::move(r));
    return true;
}

// removemodel <model> - this LOD stops drawing the model entirely. Stored as a
// model replacement with no replacement geometry (reference Cmd_RemoveModel).
bool ParseRemoveModel(Ctx& c, const Token& cmd, cm::ScriptLod& lod) {
    cm::LodReplacement r;
    if (!c.Want("a model name", cmd, r.src))
        return false;
    if (_stricmp(r.src.c_str(), "blank") == 0)
        return true;

    r.srcSource = FindModelSource(c, r.src);
    if (!r.srcSource)
        return c.Fail(cmd.line, "removemodel: unknown model \"" + r.src + "\"");

    lod.modelReplacements.push_back(std::move(r));
    return true;
}

// decimatemodel <model> <factor> - auto-generate this LOD from the full-detail
// mesh. factor is the share of triangles kept: 1.0 = all, 0.5 = half.
bool ParseDecimateModel(Ctx& c, const Token& cmd, cm::ScriptLod& lod) {
    cm::LodReplacement r;
    if (!c.Want("a model name", cmd, r.src) ||
        !c.WantFloat("a decimation factor", cmd, r.decimation))
        return false;
    if (r.decimation <= 0.0f || r.decimation > 1.0f)
        return c.Fail(cmd.line, "decimatemodel: factor must be in (0, 1.0]");

    r.srcSource = FindModelSource(c, r.src);
    if (!r.srcSource)
        return c.Fail(cmd.line, "decimatemodel: unknown model \"" + r.src + "\"");

    lod.generateLods.push_back(std::move(r));
    return true;
}

// $lod <switchvalue> { ... } / $shadowlod { ... }
//
// A shadow LOD reserves switch value -1, which is what identifies it to the
// engine, and has facial animation off by default.
bool CmdLod(Ctx& c, const Token& cmd) {
    const bool isShadow = _stricmp(cmd.text.c_str(), "$shadowlod") == 0;

    for (const cm::ScriptLod& prev : c.in.scriptLods)
        if (prev.IsShadow())
            return c.Fail(cmd.line,
                          "$shadowlod must be the last LOD and there can be only one");

    cm::ScriptLod lod;
    if (isShadow) {
        lod.switchValue = -1.0f;
        lod.facialAnimation = false;
    } else {
        if (!c.WantFloat("a switch value", cmd, lod.switchValue))
            return false;
        if (lod.switchValue < 0.0f)
            return c.Fail(cmd.line, "$lod: negative switch values are reserved for $shadowlod");
    }

    // +1 for the implicit root LOD, which no $lod block writes
    if (c.in.scriptLods.size() + 2 > static_cast<size_t>(pulse::limits::kMaxNumLods))
        return c.Fail(cmd.line, "too many $lod blocks");

    if (!WantOpenBrace(c, cmd, cmd.text))
        return false;

    while (true) {
        if (c.Eof())
            return c.Fail(cmd.line, cmd.text + ": missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;

        const std::string opt = Lower(t.text);
        if (opt == "replacemodel") {
            if (!ParseReplaceModel(c, t, lod)) return false;
        } else if (opt == "removemodel") {
            if (!ParseRemoveModel(c, t, lod)) return false;
        } else if (opt == "decimatemodel") {
            if (!ParseDecimateModel(c, t, lod)) return false;
        } else if (opt == "decimateallmodel") {
            if (!c.WantFloat("a decimation factor", t, lod.decimateAllFactor))
                return false;
            if (lod.decimateAllFactor <= 0.0f || lod.decimateAllFactor > 1.0f)
                return c.Fail(t.line, "decimateallmodel: factor must be in (0, 1.0]");
        } else if (opt == "replacebone") {
            cm::LodReplacement r;
            if (!c.Want("a bone name", t, r.src) || !c.Want("a replacement bone", t, r.dst))
                return false;
            lod.boneReplacements.push_back(std::move(r));
        } else if (opt == "bonetreecollapse") {
            cm::LodReplacement r;
            if (!c.Want("a bone name", t, r.src))
                return false;
            lod.boneTreeCollapses.push_back(std::move(r));
        } else if (opt == "replacematerial") {
            cm::LodReplacement r;
            if (!c.Want("a material name", t, r.src) ||
                !c.Want("a replacement material", t, r.dst))
                return false;
            // The replacement is named by string in the .vtx replacement list,
            // never by material id, so it does NOT join the material table. The
            // reference registers it here and then culls it right back out in
            // CullUnusedMaterials - same result, one less step.
            lod.materialReplacements.push_back(std::move(r));
        } else if (opt == "removemesh") {
            cm::LodReplacement r;
            if (!c.Want("a material name", t, r.src))
                return false;
            lod.meshRemovals.push_back(std::move(r));
        } else if (opt == "removemeshword") {
            cm::LodReplacement r;
            if (!c.Want("a word to match", t, r.src))
                return false;
            lod.meshWordRemovals.push_back(std::move(r));
        } else if (opt == "nomorphs" || opt == "nofacial") {
            lod.facialAnimation = false;
        } else if (opt == "facial") {
            if (isShadow)
                return c.Fail(t.line, "$shadowlod: facial animation is not allowed on a shadow LOD");
            lod.facialAnimation = true;
        } else if (opt == "use_shadowlod_materials") {
            // silently ignored on a plain $lod, the way the reference does it -
            // the flag is model-wide and only means anything for the shadow LOD
            if (isShadow)
                lod.useShadowLodMaterials = true;
        } else {
            return c.Fail(t.line, cmd.text + ": unknown option \"" + t.text + "\"");
        }
    }

    c.in.scriptLods.push_back(std::move(lod));
    return true;
}

// $texturegroup { $set { material <replaced> <replacement> ... } ... }
//
// Skin families. Family 0 is the model's own materials; each $set becomes the
// next family in script order, so the first $set is skin 1. Stock's
// Cmd_TextureGroup listed a full row of materials per group and paired them up
// with row 0 by position; here each line names both materials, so a family
// writes only what it changes and the order within a $set does not matter.
// More than one $texturegroup just keeps appending families.
bool CmdTextureGroup(Ctx& c, const Token& cmd) {
    if (!WantOpenBrace(c, cmd, cmd.text))
        return false;

    while (true) {
        if (c.Eof())
            return c.Fail(cmd.line, "$texturegroup: missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        if (t.quoted || _stricmp(t.text.c_str(), "$set") != 0)
            return c.Fail(t.line, "$texturegroup: expected $set or '}', got \"" +
                                  t.text + "\"");

        std::vector<cm::CompileInput::SkinReplace> fam;
        if (!ParseTextureSet(c, t, fam))
            return false;
        c.in.skinFamilies.push_back(std::move(fam));
    }

    // +1 for family 0, which no $set writes
    if (c.in.skinFamilies.size() + 1 > static_cast<size_t>(pulse::limits::kMaxSkinFamilies))
        return c.Fail(cmd.line, "too many $texturegroup $set blocks");
    return true;
}

// ---------------------------------------------------------------------------
// Hitboxes - $hitboxset { $hbox ... }. Stock QC lets a bare $hbox fall into an
// implicit "default" set; here the set is always explicit, so which set a box
// lands in is read off the script instead of inferred from command order.
// ---------------------------------------------------------------------------

// One $hbox inside a $hitboxset body:
//   $hbox <group> <bone> <minx> <miny> <minz> <maxx> <maxy> <maxz>
//         [angles <p> <y> <r>] [radius <r>] [name <hitboxname>]
// The trailing clauses are stock's positional optionals ($hbox ... [angles]
// [radius] [name]) turned into named ones, same treatment as $attachment.
bool ParseHbox(Ctx& c, const Token& cmd, cm::HitboxSet& set) {
    cm::HitBox hb;
    if (!c.WantInt("a hit group", cmd, hb.group) ||
        !c.Want("a bone name", cmd, hb.bonename) ||
        !c.WantFloat("a min X", cmd, hb.bmin.x) ||
        !c.WantFloat("a min Y", cmd, hb.bmin.y) ||
        !c.WantFloat("a min Z", cmd, hb.bmin.z) ||
        !c.WantFloat("a max X", cmd, hb.bmax.x) ||
        !c.WantFloat("a max Y", cmd, hb.bmax.y) ||
        !c.WantFloat("a max Z", cmd, hb.bmax.z))
        return false;

    // a clause list ends at '}' or the next $hbox
    while (!c.AtCommand() && !(!c.Cur().quoted && c.Cur().text == "}")) {
        const Token t = c.toks[c.pos++];
        const std::string o = Lower(t.text);
        const Token sub{cmd.text + " " + t.text, t.line, false};

        if (!t.quoted && o == "angles") {
            if (!c.WantFloat("a pitch", sub, hb.angOffset.x) ||
                !c.WantFloat("a yaw", sub, hb.angOffset.y) ||
                !c.WantFloat("a roll", sub, hb.angOffset.z))
                return false;
        } else if (!t.quoted && o == "radius") {
            if (!c.WantFloat("a capsule radius", sub, hb.capsuleRadius))
                return false;
        } else if (!t.quoted && o == "name") {
            if (!c.Want("a hitbox name", sub, hb.name))
                return false;
        } else {
            return c.Fail(t.line, "$hbox: expected angles, radius or name, got \"" +
                                  t.text + "\"");
        }
    }

    // scaled at parse time, where the reference calls scale_vertex
    hb.bmin = {hb.bmin.x * c.in.scale, hb.bmin.y * c.in.scale, hb.bmin.z * c.in.scale};
    hb.bmax = {hb.bmax.x * c.in.scale, hb.bmax.y * c.in.scale, hb.bmax.z * c.in.scale};

    set.hitboxes.push_back(std::move(hb));
    if (set.hitboxes.size() > static_cast<size_t>(pulse::limits::kMaxHitboxesPerSet))
        return c.Fail(cmd.line, "too many $hbox entries in set \"" + set.name + "\"");
    return true;
}

// $hitboxset <name> { $hbox ... }  (Cmd_HitboxSet + Cmd_Hitbox), also spelled
// $hboxset - stock's name for the same command. Declaring any set turns the
// auto-generated one off entirely: the model gets exactly the sets written
// here, in script order.
bool CmdHitboxSet(Ctx& c, const Token& cmd) {
    cm::HitboxSet set;
    if (!c.Want("a set name", cmd, set.name))
        return false;
    if (!WantOpenBrace(c, cmd, cmd.text))
        return false;

    while (true) {
        if (c.Eof())
            return c.Fail(cmd.line, cmd.text + " \"" + set.name + "\": missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        if (t.quoted || _stricmp(t.text.c_str(), "$hbox") != 0)
            return c.Fail(t.line, cmd.text + " \"" + set.name +
                                  "\": expected $hbox or '}', got \"" + t.text + "\"");
        if (!ParseHbox(c, t, set))
            return false;
    }

    c.in.hitboxsets.push_back(std::move(set));
    if (c.in.hitboxsets.size() > static_cast<size_t>(pulse::limits::kMaxHitboxSets))
        return c.Fail(cmd.line, "too many hitbox sets");
    return true;
}

// A top-level $hbox. Listed in the command table only so it reports what is
// actually wrong instead of "unknown command".
bool CmdHboxOutsideSet(Ctx& c, const Token& cmd) {
    return c.Fail(cmd.line, "$hbox must appear inside a $hitboxset / $hboxset "
                            "{ } block - there is no implicit \"default\" set");
}

// $renamehboxset <target> <newname> - rename an already-declared set.
//
// Acts on the sets declared ABOVE it, so it must be written after the
// $hitboxset it renames; naming a set that does not exist yet is an error
// rather than a silently-deferred rename. The point is reusing one hitbox
// layout under a different set name - a shared block pulled in by (future)
// $include can be renamed at the use site without editing the block.
bool CmdRenameHboxSet(Ctx& c, const Token& cmd) {
    std::string target, newname;
    if (!c.Want("a hitbox set name", cmd, target) ||
        !c.Want("a new name", cmd, newname))
        return false;
    if (newname.empty())
        return c.Fail(cmd.line, "$renamehboxset: the new name cannot be empty");

    for (cm::HitboxSet& s : c.in.hitboxsets) {
        if (_stricmp(s.name.c_str(), target.c_str()) == 0) {
            s.name = std::move(newname);
            return true;
        }
    }
    return c.Fail(cmd.line, "$renamehboxset: no hitbox set named \"" + target +
                            "\" has been declared yet");
}

// ---------------------------------------------------------------------------
// $poseparameter / $ikchain / $ikautoplaylock - stock studiomdl syntax, 1:1.
// These feed the same CompileInput lists the $sequence blend/iklock/ikrule
// options reference by name, so they conventionally precede the sequences.
// ---------------------------------------------------------------------------

// $poseparameter <name> <min> <max> [wrap | loop <value>]  (Cmd_PoseParameter).
// Re-declaring a name updates it in place, matching LookupPoseParameter.
bool CmdPoseParameter(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a name", cmd, name))
        return false;

    cm::PoseParam* pp = nullptr;
    for (auto& e : c.in.poseparams)
        if (_stricmp(e.name.c_str(), name.c_str()) == 0) { pp = &e; break; }
    if (!pp) {
        c.in.poseparams.emplace_back();
        pp = &c.in.poseparams.back();
        pp->name = name;
    }

    // min / max are optional in stock (default 0); read them when present
    if (!c.AtCommand() && !c.WantFloat("a min value", cmd, pp->min))
        return false;
    if (!c.AtCommand() && !c.WantFloat("a max value", cmd, pp->max))
        return false;

    while (!c.AtCommand()) {
        const Token t = c.toks[c.pos++];
        if (!t.quoted && _stricmp(t.text.c_str(), "wrap") == 0) {
            pp->flags |= kStudioLooping;
            pp->loop = pp->max - pp->min;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "loop") == 0) {
            pp->flags |= kStudioLooping;
            if (!c.WantFloat("a loop value", t, pp->loop))
                return false;
        } else {
            return c.Fail(t.line, "$poseparameter \"" + name +
                                  "\": expected wrap or loop, got \"" + t.text + "\"");
        }
    }

    if (c.in.poseparams.size() > static_cast<size_t>(pulse::limits::kMaxPoseParam))
        return c.Fail(cmd.line, "too many pose parameters");
    return true;
}

// $ikchain <name> <endbone> [knee x y z] [height h] [pad p] [floor f]
//          [center x y z]   (Cmd_IKChain). A duplicate name is warned + ignored.
bool CmdIkChain(Ctx& c, const Token& cmd) {
    std::string name, endbone;
    if (!c.Want("a name", cmd, name) || !c.Want("an end bone name", cmd, endbone))
        return false;

    bool dup = false;
    for (const auto& e : c.in.ikchains)
        if (_stricmp(e.name.c_str(), name.c_str()) == 0) { dup = true; break; }

    cm::IkChain chain;
    chain.name = name;
    chain.bonename = endbone;

    while (!c.AtCommand()) {
        const Token t = c.toks[c.pos++];
        const Token sub{cmd.text + " " + t.text, t.line, false};
        if (!t.quoted && _stricmp(t.text.c_str(), "knee") == 0) {
            if (!c.WantFloat("an X", sub, chain.link[0].kneeDir.x) ||
                !c.WantFloat("a Y", sub, chain.link[0].kneeDir.y) ||
                !c.WantFloat("a Z", sub, chain.link[0].kneeDir.z))
                return false;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "height") == 0) {
            if (!c.WantFloat("a height", sub, chain.height)) return false;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "pad") == 0) {
            float pad = 0.0f;
            if (!c.WantFloat("a pad", sub, pad)) return false;
            chain.radius = pad / 2.0f;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "floor") == 0) {
            if (!c.WantFloat("a floor", sub, chain.floor)) return false;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "center") == 0) {
            if (!c.WantFloat("an X", sub, chain.center.x) ||
                !c.WantFloat("a Y", sub, chain.center.y) ||
                !c.WantFloat("a Z", sub, chain.center.z))
                return false;
        } else {
            return c.Fail(t.line, "$ikchain \"" + name + "\": expected knee, height, "
                                  "pad, floor or center, got \"" + t.text + "\"");
        }
    }

    if (dup) {
        fprintf(stderr, "WARNING: duplicate ikchain \"%s\" ignored\n", name.c_str());
        return true;
    }
    c.in.ikchains.push_back(std::move(chain));
    if (c.in.ikchains.size() > static_cast<size_t>(pulse::limits::kMaxIkChains))
        return c.Fail(cmd.line, "too many ik chains");
    return true;
}

// $ikautoplaylock <chain> <position_weight> <rotation_weight>  (Cmd_IKAutoplayLock).
bool CmdIkAutoplayLock(Ctx& c, const Token& cmd) {
    cm::IkLock lock;
    if (!c.Want("an ik chain name", cmd, lock.name) ||
        !c.WantFloat("a position weight", cmd, lock.flPosWeight) ||
        !c.WantFloat("a rotation weight", cmd, lock.flLocalQWeight))
        return false;
    c.in.ikautoplaylocks.push_back(std::move(lock));
    if (c.in.ikautoplaylocks.size() >
        static_cast<size_t>(pulse::limits::kMaxIkAutoplayLocks))
        return c.Fail(cmd.line, "too many ik autoplay locks");
    return true;
}

// Shared body for $weightlist / $defaultweightlist (studiomdl Option_Weightlist),
// inline or braced. Each `<bone> <weight>` adds an entry whose position weight
// defaults to the rotation weight; a following `posweight <val>` overrides the
// last entry's position weight. A nonzero posweight needs a nonzero weight.
bool ParseWeightEntries(Ctx& c, const Token& cmd, const std::string& listname,
                        std::vector<cm::WeightList::Entry>& entries) {
    bool braced = false;
    if (!c.Eof() && !c.Cur().quoted && c.Cur().text == "{") {
        braced = true;
        c.pos++;
    }
    for (;;) {
        if (braced) {
            if (c.Eof())
                return c.Fail(cmd.line, "weightlist \"" + listname + "\" is missing '}'");
            if (!c.Cur().quoted && c.Cur().text == "}") { c.pos++; break; }
        } else if (c.AtCommand()) {
            break;
        }

        const Token t = c.toks[c.pos++];
        if (!t.quoted && _stricmp(t.text.c_str(), "posweight") == 0) {
            if (entries.empty())
                return c.Fail(t.line, "weightlist \"" + listname +
                                      "\": posweight before any bone");
            float pw = 0.0f;
            if (!c.WantFloat("a position weight", t, pw))
                return false;
            entries.back().posweight = pw;
            if (entries.back().weight == 0.0f && pw > 0.0f)
                return c.Fail(t.line, "weightlist \"" + listname + "\" bone \"" +
                                      entries.back().bone +
                                      "\": posweight > 0 needs weight > 0");
        } else {
            cm::WeightList::Entry entry;
            entry.bone = t.text;
            if (!c.WantFloat("a weight", t, entry.weight))
                return false;
            entry.posweight = entry.weight;
            entries.push_back(std::move(entry));
        }
    }
    return true;
}

// $weightlist <name> { <bone> <weight> [posweight <val>] ... }  (Cmd_Weightlist).
bool CmdWeightList(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a name", cmd, name))
        return false;
    for (const auto& e : c.in.weightlists)
        if (_stricmp(e.name.c_str(), name.c_str()) == 0)
            return c.Fail(cmd.line, "duplicate weightlist \"" + name + "\"");
    cm::WeightList wl;
    wl.name = name;
    if (!ParseWeightEntries(c, cmd, name, wl.entries))
        return false;
    c.in.weightlists.push_back(std::move(wl));
    // slot 0 is the default list, so the named lists share kMaxWeightlists - 1
    if (c.in.weightlists.size() > static_cast<size_t>(pulse::limits::kMaxWeightlists) - 1)
        return c.Fail(cmd.line, "too many weightlists");
    return true;
}

// $defaultweightlist { <bone> <weight> [posweight <val>] ... }  (Cmd_DefaultWeightlist).
// Fills weightlist slot 0; re-declaring replaces it (Option_Weightlist resets).
bool CmdDefaultWeightList(Ctx& c, const Token& cmd) {
    c.in.defaultWeights.clear();
    return ParseWeightEntries(c, cmd, "<default>", c.in.defaultWeights);
}

// ---------------------------------------------------------------------------
// Script-wide animation/sequence defaults. Each one takes effect for every
// $animation / $sequence declared after it, matching studiomdl's globals.
// ---------------------------------------------------------------------------

// $defaultfps <fps>  (Cmd_SetDefaultFPS): the fps a new animation starts with.
bool CmdDefaultFps(Ctx& c, const Token& cmd) {
    float fps = 0.0f;
    if (!c.WantFloat("a frame rate", cmd, fps))
        return false;
    if (fps <= 0.0f)
        return c.Fail(cmd.line, "$defaultfps must be > 0");
    c.defaultFps = fps;
    return true;
}

// $defaultfadein / $defaultfadeout <seconds>  (Cmd_SetDefaultFadeIn/OutTime).
bool CmdDefaultFadeIn(Ctx& c, const Token& cmd) {
    return c.WantFloat("a time", cmd, c.defaultFadeIn);
}
bool CmdDefaultFadeOut(Ctx& c, const Token& cmd) {
    return c.WantFloat("a time", cmd, c.defaultFadeOut);
}

// $lcaseallsequences  (Cmd_LCaseAllSequences).
bool CmdLCaseAllSequences(Ctx& c, const Token&) {
    c.lcaseSequences = true;
    return true;
}

// $allowactivityname <name>  (Cmd_AllowActivityName): opt-in whitelist. Declare
// one and every $sequence activity must be on the list, so a typo'd ACT_ name
// fails the compile instead of silently becoming a new activity.
bool CmdAllowActivityName(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("an activity name", cmd, name))
        return false;
    c.allowedActivities.push_back(std::move(name));
    return true;
}

// $sectionframes <frames> <minframelimit>  (Cmd_SectionFrames): animation
// compression sectioning. Stock's defaults are 30 30.
bool CmdSectionFrames(Ctx& c, const Token& cmd) {
    int frames = 0, minLimit = 0;
    if (!c.WantInt("a section length in frames", cmd, frames) ||
        !c.WantInt("a minimum frame limit", cmd, minLimit))
        return false;
    // a zero section length divides by zero downstream
    if (frames <= 0 || minLimit < 0)
        return c.Fail(cmd.line, "$sectionframes expects a length > 0 and a "
                                "minimum frame limit >= 0");
    c.in.sectionFrames = frames;
    c.in.minSectionFrameLimit = minLimit;
    return true;
}

// $animblocksize <size> [nostall] [highres|lowres] [numframes <1..4>]
//                       [cachehighres] [posdelta <f>]   (Cmd_AnimBlockSize)
// Turns on demand-loaded animation: the payload moves to models/<name>.ani in
// chunks of roughly <size> bytes. A value under 1024 is read as kilobytes, the
// way stock does, so "$animblocksize 8" means 8 KB.
bool CmdAnimBlockSize(Ctx& c, const Token& cmd) {
    int size = 0;
    if (!c.WantInt("a block size in bytes", cmd, size))
        return false;
    if (size < 0)
        return c.Fail(cmd.line, "$animblocksize expects a size >= 0");
    if (size < 1024)
        size *= 1024;
    c.in.animblocksize = size;

    while (!c.AtCommand()) {
        const std::string o = c.Next()->text;
        if (_stricmp(o.c_str(), "nostall") == 0) {
            c.in.noAnimblockStall = true;
        } else if (_stricmp(o.c_str(), "highres") == 0) {
            c.in.animblockHighRes = true;
            c.in.animblockLowRes = false;
        } else if (_stricmp(o.c_str(), "lowres") == 0) {
            c.in.animblockLowRes = true;
            c.in.animblockHighRes = false;
        } else if (_stricmp(o.c_str(), "numframes") == 0) {
            int n = 0;
            if (!c.WantInt("a zero-frame count", cmd, n))
                return false;
            c.in.maxZeroFrames = n < 1 ? 1 : (n > 4 ? 4 : n);
        } else if (_stricmp(o.c_str(), "cachehighres") == 0) {
            c.in.zeroFramesHighres = true;
        } else if (_stricmp(o.c_str(), "posdelta") == 0) {
            if (!c.WantFloat("a position delta", cmd, c.in.minZeroFramePosDelta))
                return false;
        } else {
            return c.Fail(cmd.line, "unknown option \"" + o + "\" on $animblocksize");
        }
    }
    return true;
}

// $bonesaveframe <bone> [position] [rotation] [rotation64]  (Cmd_BoneSaveFrame)
// Names a bone whose pose is cached in the .mdl so the model can be posed
// before its .ani block finishes loading. Stating any entry replaces the
// writer's automatic choice for EVERY bone, so a script that uses this must
// list all the bones it wants cached.
bool CmdBoneSaveFrame(Ctx& c, const Token& cmd) {
    cm::BoneSaveFrame bsf;
    if (!c.Want("a bone name", cmd, bsf.name))
        return false;
    while (!c.AtCommand()) {
        const std::string o = c.Next()->text;
        if (_stricmp(o.c_str(), "position") == 0)
            bsf.savePos = true;
        else if (_stricmp(o.c_str(), "rotation") == 0)
            bsf.saveRot = true;
        else if (_stricmp(o.c_str(), "rotation64") == 0)
            bsf.saveRot64 = true;
        else
            return c.Fail(cmd.line, "unknown option \"" + o + "\" on $bonesaveframe : " + bsf.name);
    }
    c.in.boneSaveFrames.push_back(std::move(bsf));
    return true;
}

// ---------------------------------------------------------------------------
// $physicsmodel - the whole .phy in one block.
//
// There is no $collisionmodel / $collisionjoints split: the compile stage
// counts the bones the collision geometry resolves to and picks single body vs
// ragdoll itself.
//
// Two levels of vocabulary. Directly inside $physicsmodel everything is a
// $command, keeping stock studiomdl's spellings ($mass, $rootbone,
// $noselfcollisions, ...); inside the nested $physicsshape / $physicsjoint /
// $physicsmarkup blocks the options are bare words. So the block loops here
// terminate on '}' rather than on Ctx::AtCommand - a '$' no longer means "a new
// top-level command started".
//
// Defaults are never restated: a field is only touched when authored, so the
// PhysicsShape / CompileInput struct defaults are the single source of them and
// both front ends land in the same place.
// ---------------------------------------------------------------------------

bool WantVec3(Ctx& c, const Token& cmd, pm::Vector3& v) {
    return c.WantFloat("an X value", cmd, v.x) &&
           c.WantFloat("a Y value", cmd, v.y) &&
           c.WantFloat("a Z value", cmd, v.z);
}

// $physicsshape fromfile <file> { }, fromrendermesh <$rendermesh> { }, or
// fromrender { }. `kind` and `ref` are set by the caller; fromfile and
// fromrendermesh differ only in where the authored geometry comes from (disk vs
// a $rendermesh), so both are kind FromFile.
//
// The shape name is not authored - a body is named by its BONE everywhere in
// the .phy, so the name is only ever a diagnostic label and is taken from
// whatever identifies the shape (its bone, its rendermesh, its file).
bool ParsePhysShape(Ctx& c, const Token& cmd, const std::string& mode,
                    const std::string& ref, cm::PhysicsShape& sh) {
    const bool fromRender = mode == "fromrender";
    const bool fromMesh   = mode == "fromrendermesh";
    const std::string where = "$physicsshape " + mode;
    // the block is optional - every option in it has a default, so
    // `$physicsshape fromfile "phys.dmx"` alone is a whole shape
    const bool braced = !c.Eof() && !c.Cur().quoted && c.Cur().text == "{";
    if (braced)
        c.pos++;

    // importtype is named rather than numbered here (the .pulsemdl element
    // still takes the 0/1/2 the enum is built on)
    std::string importType = "perjoint";

    while (braced) {
        if (c.Eof())
            return c.Fail(cmd.line, where + ": missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        const std::string o = t.quoted ? std::string() : Lower(t.text);
        const Token sub{where + " " + t.text, t.line, false};

        // shared framing - scale, then rotate, then translate
        if (o == "offsetorigin") {
            if (!WantVec3(c, sub, sh.offsetOrigin)) return false;
        } else if (o == "offsetangles") {
            if (!WantVec3(c, sub, sh.offsetAngles)) return false;
            sh.offsetAnglesSet = true;
        } else if (o == "importscale") {
            if (!c.WantFloat("a scale", sub, sh.importScale)) return false;
        } else if (o == "parentbone") {
            if (!c.Want("a bone name", sub, sh.parentBone)) return false;
        } else if (o == "maxconvex") {
            if (!c.WantInt("a piece count", sub, sh.maxConvex)) return false;
        } else if (!fromRender && o == "importtype") {
            if (!c.Want("perjoint, singlejoint or singlehull", sub, importType))
                return false;
            importType = Lower(importType);
        } else if (!fromRender && o == "concave") {
            sh.concave = true;
        } else if (!fromRender && o == "remove2d") {
            sh.remove2d = true;
        } else if (fromRender && (o == "decimationfactor" || o == "decimatefactor")) {
            if (!c.WantFloat("a factor", sub, sh.decimationFactor)) return false;
        } else if (fromRender && o == "cullweight") {
            if (!c.WantFloat("a weight", sub, sh.cullWeight)) return false;
        } else if (fromRender && o == "concavity") {
            if (!c.WantFloat("a concavity", sub, sh.concavity)) return false;
        } else if (fromRender && o == "maxhulls") {
            if (!c.WantInt("a hull count", sub, sh.maxHulls)) return false;
        } else if (fromRender && o == "extrabone") {
            std::string bone;
            if (!c.Want("a bone name", sub, bone)) return false;
            sh.extraSkinnedBones.push_back(std::move(bone));
        } else if (fromRender && o == "excludemesh") {
            std::string mesh;
            if (!c.Want("a $rendermesh name", sub, mesh)) return false;
            sh.exceptionMeshNames.push_back(std::move(mesh));
        } else {
            return c.Fail(t.line, where + ": invalid syntax \"" + t.text + "\"");
        }
    }

    if (sh.importScale <= 0.0f)
        return c.Fail(cmd.line, where + ": importscale must be greater than 0");

    if (fromRender) {
        // parentbone carves ONE body out of a skinned character. A prop has no
        // bone to cull against, so omitting it means "the whole render mesh",
        // bound to the root - and extrabone/cullweight go unused.
        sh.name = sh.parentBone.empty() ? "generated" : sh.parentBone;
        if (sh.decimationFactor > 1.0f) sh.decimationFactor = 1.0f;
        if (sh.decimationFactor > 0.0f && sh.decimationFactor < 0.1f)
            sh.decimationFactor = 0.1f;
        if (sh.cullWeight < 0.0f) sh.cullWeight = 0.0f;
        if (sh.cullWeight > 1.0f) sh.cullWeight = 1.0f;
        // 0 = "however many the split produced", which still means the safety
        // ceiling - VHACD merges down to whatever it is given.
        if (sh.maxHulls <= 0) {
            sh.maxHulls = pulse::limits::kMaxGeneratedHulls;
        } else if (sh.maxHulls > pulse::limits::kMaxGeneratedHulls) {
            std::printf("WARNING: PhysicsShapeFromRender \"%s\" asks for %d hulls, "
                        "clamped to %d\n",
                        sh.name.c_str(), sh.maxHulls, pulse::limits::kMaxGeneratedHulls);
            sh.maxHulls = pulse::limits::kMaxGeneratedHulls;
        }
        if (sh.extraSkinnedBones.size() >
            static_cast<size_t>(pulse::limits::kMaxExtraSkinnedBones))
            return c.Fail(cmd.line, where + ": too many extrabone entries (max " +
                                    std::to_string(pulse::limits::kMaxExtraSkinnedBones) + ")");
        // the filter excludes rather than selects, so a name that resolves to
        // nothing would silently fail to exclude anything
        for (const std::string& name : sh.exceptionMeshNames) {
            auto it = c.rendermeshes.find(name);
            if (it == c.rendermeshes.end())
                return c.Fail(cmd.line, where + ": filters unknown rendermesh \"" + name + "\"");
            sh.exceptionSources.push_back(it->second);
        }
        return true;
    }

    if (fromMesh) {
        auto it = c.rendermeshes.find(ref);
        if (it == c.rendermeshes.end())
            return c.Fail(cmd.line, where + ": references unknown rendermesh \"" +
                                    ref + "\"");
        sh.source = it->second;
        sh.name = ref;
    } else {
        // a collision-only source, loaded straight from disk with no
        // $rendermesh in front of it. Same loader as everything else, so .smd
        // and .dmx both work; morphSource stays off, so its delta shapes are
        // skipped along with everything else a collision hull has no use for -
        // and LoadKind::Collision keeps its MATERIALS out of the model too. The
        // hull needs the geometry, but only its shape: the physics stage flattens
        // the per-material mesh grouping away (compile.cpp CollectSourceFaces)
        // and surfaceprop comes from the script, never from a mesh material.
        const std::string file = WithSourceExtension(c, ref);
        sh.source = LoadSource(c, file, cmd.line, /*morphSource=*/false, /*edit=*/nullptr,
                               source::LoadKind::Collision);
        if (!sh.source)
            return false;
        sh.name = fs::path(file).stem().string();
    }

    // perjoint = keep the source's own rigging (the only mode that can produce
    // a ragdoll on its own); singlejoint = pin every part onto parentbone
    // whatever it was rigged to; singlehull = keep only parentbone's share and
    // drop the rest.
    if (importType == "perjoint")         sh.importType = cm::PhysicsImportType::Skinned;
    else if (importType == "singlejoint") sh.importType = cm::PhysicsImportType::ToOneBone;
    else if (importType == "singlehull")  sh.importType = cm::PhysicsImportType::OneBoneOnly;
    else
        return c.Fail(cmd.line, where + ": unknown importtype \"" + importType +
                                "\" (expected perjoint/singlejoint/singlehull)");
    if (sh.importType != cm::PhysicsImportType::Skinned && sh.parentBone.empty())
        return c.Fail(cmd.line, where + ": importtype " + importType +
                                " but no parentbone");
    return true;
}

// $physicsjoint <bone> { x limit <min> <max> [friction <f>] / y free / z fixed }
// An omitted axis is LOCKED, not free - the compile stage zero-fills and only
// the axes named here move.
//
// Braces are optional for a single axis: $physicsjoint <bone> x fixed. More than
// one axis needs either a block or one $physicsjoint per axis.
bool ParsePhysJoint(Ctx& c, const Token& cmd, cm::PhysicsJoint& joint) {
    const std::string where = "$physicsjoint \"" + joint.bonename + "\"";
    const bool braced = !c.Eof() && !c.Cur().quoted && c.Cur().text == "{";
    if (braced)
        c.pos++;

    do {
        if (c.Eof())
            return c.Fail(cmd.line, where + (braced ? ": missing '}'"
                                                    : ": expected x, y or z"));
        const Token t = c.toks[c.pos++];
        if (braced && !t.quoted && t.text == "}")
            break;

        cm::PhysicsJointAxis a;
        const std::string ax = t.quoted ? std::string() : Lower(t.text);
        if      (ax == "x") a.axis = 0;
        else if (ax == "y") a.axis = 1;
        else if (ax == "z") a.axis = 2;
        else return c.Fail(t.line, where + ": expected x, y" +
                                   (braced ? ", z or '}'" : " or z") + ", got \"" +
                                   t.text + "\"");

        const Token sub{where + " " + t.text, t.line, false};
        std::string type;
        if (!c.Want("free, limit or fixed", sub, type))
            return false;
        type = Lower(type);
        if      (type == "free")  a.type = 0;
        else if (type == "limit") a.type = 1;
        else if (type == "fixed") a.type = 2;
        else return c.Fail(t.line, where + ": unknown type \"" + type +
                                   "\" (expected free/limit/fixed)");

        if (a.type == 1 && (!c.WantFloat("a min angle", sub, a.min) ||
                            !c.WantFloat("a max angle", sub, a.max)))
            return false;
        // keyword-prefixed so the optional value cannot be confused with the
        // next axis line
        if (!c.Eof() && !c.Cur().quoted && Lower(c.Cur().text) == "friction") {
            c.pos++;
            if (!c.WantFloat("a friction value", sub, a.friction))
                return false;
        }
        joint.axes.push_back(a);
    } while (braced);
    return true;
}

// $physicsmarkup <bone> { ... } - the per-body override. Every field falls back
// to the $physicsmodel setting of the same name, so presence is what counts and
// an authored 0 has to beat a nonzero default.
//
// Braces are optional for a single field: $physicsmarkup <bone> massbias 7. More
// than one field needs either a block or one $physicsmarkup per field.
bool ParsePhysMarkup(Ctx& c, const Token& cmd, cm::PhysicsMarkup& mk) {
    const std::string where = "$physicsmarkup \"" + mk.bonename + "\"";
    const bool braced = !c.Eof() && !c.Cur().quoted && c.Cur().text == "{";
    if (braced)
        c.pos++;

    do {
        if (c.Eof())
            return c.Fail(cmd.line, where + (braced ? ": missing '}'"
                                                    : ": expects a field name"));
        const Token t = c.toks[c.pos++];
        if (braced && !t.quoted && t.text == "}")
            break;
        const std::string o = t.quoted ? std::string() : Lower(t.text);
        const Token sub{where + " " + t.text, t.line, false};

        if (o == "massbias") {
            if (!c.WantFloat("a bias", sub, mk.massBias)) return false;
            mk.massBiasSet = true;
        } else if (o == "inertia") {
            if (!c.WantFloat("an inertia", sub, mk.inertia)) return false;
            mk.inertiaSet = true;
        } else if (o == "damping") {
            if (!c.WantFloat("a damping value", sub, mk.damping)) return false;
            mk.dampingSet = true;
        } else if (o == "rotdamping") {
            if (!c.WantFloat("a rotdamping value", sub, mk.rotdamping)) return false;
            mk.rotdampingSet = true;
        } else if (o == "skip") {
            mk.skip = true;
        } else if (o == "mergeinto") {
            if (!c.Want("a bone name", sub, mk.mergeInto)) return false;
        } else {
            return c.Fail(t.line, where + ": invalid syntax \"" + t.text + "\"");
        }
    } while (braced);

    if (mk.skip && !mk.mergeInto.empty())
        return c.Fail(cmd.line, where + ": sets both skip and mergeinto - pick one");
    return true;
}

bool CmdPhysicsModel(Ctx& c, const Token& cmd) {
    if (!WantOpenBrace(c, cmd, cmd.text))
        return false;

    while (true) {
        if (c.Eof())
            return c.Fail(cmd.line, "$physicsmodel: missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        const std::string o = t.quoted ? std::string() : Lower(t.text);
        const Token sub{t.text, t.line, false};

        if (o == "$physicsshape") {
            std::string mode;
            if (!c.Want("fromfile, fromrendermesh or fromrender", sub, mode))
                return false;
            mode = Lower(mode);
            if (mode != "fromfile" && mode != "fromrendermesh" && mode != "fromrender")
                return c.Fail(t.line, "$physicsshape: expected fromfile, fromrendermesh "
                                      "or fromrender, got \"" + mode + "\"");
            // the geometry reference rides on the command line, not inside the
            // block - fromrender has nothing to name, it takes the whole model
            std::string ref;
            if (mode != "fromrender") {
                if (!c.Want(mode == "fromfile" ? "a source filename" : "a $rendermesh name",
                            sub, ref))
                    return false;
                if (ref == "{") // else the missing ref reports as a missing brace
                    return c.Fail(t.line, "$physicsshape " + mode +
                                          ": expected a name before '{'");
            }
            cm::PhysicsShape sh;
            sh.kind = mode == "fromrender" ? cm::PhysicsShapeKind::FromRender
                                           : cm::PhysicsShapeKind::FromFile;
            if (!ParsePhysShape(c, t, mode, ref, sh))
                return false;
            c.in.physShapes.push_back(std::move(sh));
            if (c.in.physShapes.size() > static_cast<size_t>(pulse::limits::kMaxPhysShapes))
                return c.Fail(t.line, "too many physics shapes (max " +
                                      std::to_string(pulse::limits::kMaxPhysShapes) + ")");
        } else if (o == "$physicsjoint") {
            cm::PhysicsJoint joint;
            if (!c.Want("a bone name", sub, joint.bonename) ||
                !ParsePhysJoint(c, t, joint))
                return false;
            c.in.physJoints.push_back(std::move(joint));
        } else if (o == "$physicsmarkup") {
            cm::PhysicsMarkup mk;
            if (!c.Want("a bone name", sub, mk.bonename) || !ParsePhysMarkup(c, t, mk))
                return false;
            mk.name = mk.bonename; // only ever used in diagnostics
            c.in.physMarkups.push_back(std::move(mk));
        } else if (o == "$physicscollide") {
            // exactly two bones - the pair is symmetric, so a trailing list
            // would make one of them look like the subject
            cm::PhysicsCollidePair pair;
            if (!c.Want("a bone name", sub, pair.a) ||
                !c.Want("a second bone name", sub, pair.b))
                return false;
            c.in.physCollidePairs.push_back(std::move(pair));
        } else if (o == "$mass") {
            if (!c.WantFloat("a mass in kg", sub, c.in.physMass)) return false;
        } else if (o == "$automass") {
            c.in.physAutoMass = true;
        } else if (o == "$masscenter") {
            if (!WantVec3(c, sub, c.in.physMassCenter)) return false;
            c.in.physMassCenterSet = true;
        } else if (o == "$rootbone") {
            if (!c.Want("a bone name", sub, c.in.physRootBone)) return false;
        } else if (o == "$noselfcollisions") {
            c.in.physNoSelfCollisions = true;
        } else if (o == "$damping") {
            if (!c.WantFloat("a damping value", sub, c.in.physDamping)) return false;
        } else if (o == "$rotdamping") {
            if (!c.WantFloat("a rotdamping value", sub, c.in.physRotdamping)) return false;
        } else if (o == "$inertia") {
            if (!c.WantFloat("an inertia", sub, c.in.physInertia)) return false;
        } else if (o == "$drag") {
            if (!c.WantFloat("a drag value", sub, c.in.physDrag)) return false;
        } else if (o == "$weldposition") {
            if (!c.WantFloat("a position epsilon", sub, c.in.physWeldPosition)) return false;
        } else if (o == "$weldnormal") {
            if (!c.WantFloat("a normal epsilon", sub, c.in.physWeldNormal)) return false;
        } else if (o == "$phyname") {
            if (!c.Want("a .phy name", sub, c.in.physName)) return false;
        } else if (o == "$animatedfriction") {
            c.in.physHasAnimatedFriction = true;
            if (!c.WantInt("a minimum friction", sub, c.in.physAnimFrictionMin) ||
                !c.WantInt("a maximum friction", sub, c.in.physAnimFrictionMax) ||
                !c.WantFloat("a ramp-in time", sub, c.in.physAnimFrictionTimeIn) ||
                !c.WantFloat("a ramp-out time", sub, c.in.physAnimFrictionTimeOut) ||
                !c.WantFloat("a hold time", sub, c.in.physAnimFrictionTimeHold))
                return false;
        } else {
            return c.Fail(t.line, "$physicsmodel: invalid syntax \"" + t.text + "\"");
        }
    }

    if (c.in.physMass <= 0.0f && !c.in.physAutoMass)
        return c.Fail(cmd.line, "$physicsmodel: $mass must be positive");
    return true;
}

// The $physicsmodel body commands, written at top level. Listed in the command
// table only so they report what is actually wrong instead of "unknown
// command" - the same reason $hbox is listed.
bool CmdPhysicsOutsideModel(Ctx& c, const Token& cmd) {
    return c.Fail(cmd.line, cmd.text + " must appear inside a $physicsmodel { } block");
}

// the dispatch loop, defined under the command table - $include reenters it
bool RunCommands(Ctx& c);

// $addsearchdir <dir>  (Cmd_AddSearchDir). A fallback directory for SOURCE
// files - the .dmx/.smd a $rendermesh or $animation names, and $proceduralbones'
// .vrd. Registration order, searched after the script's own directory, so only
// dirs registered ABOVE a reference can serve it. A relative dir is relative to
// the ROOT script, the same rule the source paths it serves follow (stock joins
// cddir[0] here for exactly that reason).
//
// This is NOT $addincludesearchdir - source files and $include scripts keep
// separate lists. $pushd/$popd are not coming back; a search dir covers the
// same ground without a mode the rest of the script has to track.
bool CmdAddSearchDir(Ctx& c, const Token& cmd) {
    std::string dir;
    if (!c.Want("a directory", cmd, dir))
        return false;
    const fs::path p = fs::path(dir).is_absolute() ? fs::path(dir) : c.scriptDir / dir;
    c.searchDirs.push_back(p.lexically_normal().make_preferred());
    return true;
}

// $addincludesearchdir <dir>  (scriplib's AddIncludeDir, as a command - there
// is no launch parameter). Registers a fallback directory for $include; the
// list is searched in registration order, so only dirs registered ABOVE an
// $include can serve it. A relative dir is relative to the file registering it,
// the same rule the $include path itself follows, and is resolved here so the
// entry means one place no matter who reads it later.
bool CmdAddIncludeSearchDir(Ctx& c, const Token& cmd) {
    std::string dir;
    if (!c.Want("a directory", cmd, dir))
        return false;
    const fs::path p = fs::path(dir).is_absolute() ? fs::path(dir) : c.curDir / dir;
    c.includeDirs.push_back(p.lexically_normal().make_preferred());
    return true;
}

// $include "file.qci" [localdir] [optional]   (scriplib's
// AttemptConditionalInclude; its nofallbackdir flag is `localdir` here and its
// iffileexist flag is `optional`). The file's commands run inline, where the
// $include stands - the prefab mechanism. Top level only: it is a $command, so
// it cannot appear inside a braced body, whose options are bare words.
//
//   localdir   only look at the primary location, skip the search dirs
//   optional   found nowhere = do nothing, instead of an error
//
// The path is relative to the file the $include is written in (so a nested
// include names its sibling directly), then to each $addincludesearchdir dir.
// Everything else in an included file - source filenames, $proceduralbones -
// stays relative to the ROOT script's directory, matching how stock keeps
// cddir pinned to the top-level script.
bool CmdInclude(Ctx& c, const Token& cmd) {
    std::string rel;
    if (!c.Want("a script path", cmd, rel))
        return false;

    bool localDir = false, optional = false;
    while (!c.AtCommand()) {
        const Token& t = c.toks[c.pos];
        if (!t.quoted && _stricmp(t.text.c_str(), "localdir") == 0)
            localDir = true;
        else if (!t.quoted && _stricmp(t.text.c_str(), "optional") == 0)
            optional = true;
        else
            return c.Fail(t.line, "$include: unknown parameter \"" + t.text +
                                  "\" - localdir, optional");
        c.pos++;
    }

    // primary location first, then the search dirs in registration order. An
    // absolute path is itself and nothing else.
    const fs::path relPath(rel);
    std::vector<fs::path> tries;
    if (relPath.is_absolute()) {
        tries.push_back(relPath);
    } else {
        tries.push_back(c.curDir / relPath);
        if (!localDir)
            for (const fs::path& dir : c.includeDirs) {
                tries.push_back(dir / relPath);
                // last resort: the bare filename in that dir, for a request
                // that carried a relative hierarchy the dir does not have
                if (relPath.has_parent_path())
                    tries.push_back(dir / relPath.filename());
            }
    }

    fs::path full;
    for (fs::path& t : tries) {
        t = t.lexically_normal().make_preferred();
        std::error_code fec;
        if (fs::is_regular_file(t, fec)) {
            full = t;
            break;
        }
    }
    if (full.empty()) {
        if (optional)
            return true;
        return c.Fail(cmd.line, "$include: cannot find \"" + rel +
                                "\" - looked in:" + LookedIn(tries));
    }

    std::error_code ec;
    const fs::path canon = fs::weakly_canonical(full, ec);
    const std::string key = ec ? full.string() : canon.string();
    for (const std::string& open : c.includeStack)
        if (_stricmp(open.c_str(), key.c_str()) == 0)
            return c.Fail(cmd.line, "$include: \"" + rel +
                                    "\" is already open - circular include");

    std::ifstream f(full, std::ios::binary);
    if (!f)
        return c.Fail(cmd.line, "$include: cannot open \"" + full.string() + "\"");
    std::ostringstream buf;
    buf << f.rdbuf();
    std::string text = buf.str();
    StripUtf8Bom(text);

    const std::string name = full.filename().string();
    std::vector<Token> toks;
    if (!Tokenize(text, name, toks, c.err))
        return false;

    // swap the included file in, run it, swap back. An unbraced option list
    // ends at the file boundary - the include cannot leave a command half-read.
    std::vector<Token> savedToks = std::move(c.toks);
    const size_t savedPos = c.pos;
    const std::string savedFile = c.file;
    const fs::path savedDir = c.curDir;

    c.toks = std::move(toks);
    c.pos = 0;
    c.file = name;
    c.curDir = full.parent_path();
    c.includeStack.push_back(key);

    const bool ok = RunCommands(c);

    c.includeStack.pop_back();
    c.toks = std::move(savedToks);
    c.pos = savedPos;
    c.file = savedFile;
    c.curDir = savedDir;
    return ok;
}

// $break [<message>]  (Cmd_Break). Stops reading the script here: everything
// below is ignored, in this file and in every parent that $included it, but the
// model still compiles from what was read so far. The flag is sticky because
// each RunCommands level would otherwise carry on at its own next token.
bool CmdBreak(Ctx& c, const Token& cmd) {
    if (!c.AtCommand()) {
        const Token& msg = c.toks[c.pos++];
        std::printf("$break: %s (%s line %d)\n", msg.text.c_str(), c.file.c_str(),
                    cmd.line);
    } else {
        std::printf("$break: %s line %d - rest of the script not read\n",
                    c.file.c_str(), cmd.line);
    }
    c.scriptBreak = true;
    return true;
}

// $print <message>. Writes the message to the console and carries on.
bool CmdPrint(Ctx& c, const Token& cmd) {
    if (c.AtCommand())
        return c.Fail(cmd.line, "$print needs a message");
    std::printf("%s\n", c.toks[c.pos++].text.c_str());
    return true;
}

// $assert [<message>]. The hard counterpart to $break: reaching this line is a
// compile error, so a $if branch a script never meant to take can say so.
bool CmdAssert(Ctx& c, const Token& cmd) {
    if (!c.AtCommand())
        return c.Fail(cmd.line, "$assert: " + c.toks[c.pos++].text);
    return c.Fail(cmd.line, "$assert failed");
}

// ---------------------------------------------------------------------------
// Script preprocessing - $definevariable / $redefinevariable / $definemacro
// ---------------------------------------------------------------------------

// A "$name$" reference starting at s[i] (s[i] is '$'): the index of the closing
// '$', or npos when this is a plain word like $modelname. Mirrors the reference
// scriplib's ExpandVariableToken - the scan stops at whitespace, so only a
// closed, non-empty pair counts and a $command never looks like a reference.
size_t VarRefEnd(const std::string& s, size_t i) {
    size_t j = i + 1;
    while (j < s.size() && static_cast<unsigned char>(s[j]) > 32 && s[j] != '$')
        j++;
    return (j < s.size() && s[j] == '$' && j > i + 1) ? j : std::string::npos;
}

// Rewrite every $name$ in a token to its $definevariable value.
bool ExpandVars(Ctx& c, Token& t) {
    if (t.text.find('$') == std::string::npos)
        return true;
    std::string out;
    for (size_t i = 0; i < t.text.size();) {
        const size_t e = t.text[i] == '$' ? VarRefEnd(t.text, i) : std::string::npos;
        if (e == std::string::npos) {
            out.push_back(t.text[i++]);
            continue;
        }
        const std::string name = t.text.substr(i + 1, e - i - 1);
        auto it = c.variables.find(name);
        if (it == c.variables.end())
            return c.Fail(t.line, "unknown variable \"$" + name + "$\" - no "
                                  "$definevariable for it above this line");
        out += it->second;
        i = e + 1;
        t.expanded = true;
    }
    t.text = std::move(out);
    return true;
}

// True when the token holds at least one closed $name$ reference.
bool HasVarRef(const std::string& s) {
    for (size_t i = s.find('$'); i != std::string::npos; i = s.find('$', i + 1))
        if (VarRefEnd(s, i) != std::string::npos)
            return true;
    return false;
}

// The same walk against a macro's parameters. An unmatched $name$ is left
// exactly as written so the variable pass gets a shot at it next.
void SubstMacroParams(Token& t, const std::vector<std::string>& params,
                      const std::vector<std::string>& args) {
    if (t.text.find('$') == std::string::npos)
        return;
    std::string out;
    for (size_t i = 0; i < t.text.size();) {
        const size_t e = t.text[i] == '$' ? VarRefEnd(t.text, i) : std::string::npos;
        if (e == std::string::npos) {
            out.push_back(t.text[i++]);
            continue;
        }
        const std::string name = t.text.substr(i + 1, e - i - 1);
        size_t p = 0;
        for (; p < params.size(); p++)
            if (_stricmp(params[p].c_str(), name.c_str()) == 0)
                break;
        if (p == params.size()) {
            out.append(t.text, i, e - i + 1); // not a parameter - leave it
        } else {
            out += args[p];
            t.expanded = true;
        }
        i = e + 1;
    }
    t.text = std::move(out);
}

// One past the last token the command at c.pos will read: the next $command at
// brace depth 0, so a braced body's inner $commands ($hbox, $physicsshape, ...)
// stay inside the run. A $name$ token is a variable reference, not a command.
size_t CommandExtent(const Ctx& c) {
    int depth = 0;
    size_t i = c.pos + 1;
    for (; i < c.toks.size(); i++) {
        const Token& t = c.toks[i];
        if (t.quoted || t.text.empty())
            continue;
        if (t.text == "{")
            depth++;
        else if (t.text == "}") {
            if (depth > 0)
                depth--;
        } else if (depth == 0 && t.text[0] == '$' &&
                 VarRefEnd(t.text, 0) == std::string::npos)
            break;
    }
    return i;
}

// Expand variables in the token run the command at c.pos is about to read. Run
// per command rather than once over the whole file, so a $redefinevariable
// takes effect exactly where it is written. $definemacro is skipped whole: its
// body is stored raw and expands at each invocation instead, and so are $if /
// $switch - only the branch that is taken may expand, and it does so after the
// splice, one command at a time like anything else.
bool ExpandCommandVars(Ctx& c) {
    const char* cmd = c.toks[c.pos].text.c_str();
    if (_stricmp(cmd, "$definemacro") == 0 || _stricmp(cmd, "$if") == 0 ||
        _stricmp(cmd, "$ifdef") == 0 || _stricmp(cmd, "$switch") == 0)
        return true;
    const size_t end = CommandExtent(c);
    for (size_t i = c.pos; i < end; i++)
        if (!ExpandVars(c, c.toks[i]))
            return false;
    return true;
}

// A -defvar name keeps its launch value - overriding the script is the point of
// the switch. Says so on stdout rather than changing a value behind the author's
// back, and is the one place a variable command does not hard-error.
bool PinnedByDefVar(Ctx& c, const Token& cmd, const std::string& name) {
    if (!c.IsLockedVar(name))
        return false;
    std::printf("%s(%d): %s \"%s\" ignored - pinned to \"%s\" by -defvar\n",
                c.file.c_str(), cmd.line, cmd.text.c_str(), name.c_str(),
                c.variables[name].c_str());
    return true;
}

// $definevariable <name> <value>  (Cmd_DefineVariable)
bool CmdDefineVariable(Ctx& c, const Token& cmd) {
    std::string name, value;
    if (!c.Want("a name", cmd, name) || !c.Want("a value", cmd, value))
        return false;
    if (PinnedByDefVar(c, cmd, name))
        return true;
    if (c.variables.count(name))
        return c.Fail(cmd.line, "duplicate $definevariable \"" + name +
                                "\" - use $redefinevariable to change it");
    c.variables[name] = value;
    return true;
}

// $redefinevariable <name> <value>. Replaces a value in place; the variable has
// to exist already, which is the whole difference from $definevariable.
bool CmdRedefineVariable(Ctx& c, const Token& cmd) {
    std::string name, value;
    if (!c.Want("a name", cmd, name) || !c.Want("a value", cmd, value))
        return false;
    if (PinnedByDefVar(c, cmd, name))
        return true;
    auto it = c.variables.find(name);
    if (it == c.variables.end())
        return c.Fail(cmd.line, "$redefinevariable \"" + name + "\" is not "
                                "defined - use $definevariable first");
    it->second = value;
    return true;
}

bool IsCommandName(const std::string& name); // the kCommands lookup, below

// $definemacro <name> [<param> ...] <body> $endmacro  (Cmd_DefineMacro). The
// parameter list ends at the first $command, which is where the body starts -
// no line continuation to keep track of. The body is kept as raw tokens: both
// $param$ and $variable$ inside it resolve at each invocation, not here.
bool CmdDefineMacro(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a name", cmd, name))
        return false;
    if (static_cast<int>(c.macros.size()) >= lim::kMaxMacros)
        return c.Fail(cmd.line, "too many macros (max " +
                                std::to_string(lim::kMaxMacros) + ")");
    std::string key = name;
    for (char& ch : key)
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (c.macros.count(key))
        return c.Fail(cmd.line, "duplicate $definemacro \"" + name + "\"");
    if (IsCommandName("$" + name))
        return c.Fail(cmd.line, "$definemacro \"" + name + "\" shadows the "
                                "built-in $" + name + " - pick another name");

    Ctx::Macro m;
    while (!c.AtCommand()) {
        const std::string& p = c.toks[c.pos++].text;
        for (const std::string& prev : m.params)
            if (_stricmp(prev.c_str(), p.c_str()) == 0)
                return c.Fail(cmd.line, "$definemacro \"" + name +
                                        "\": duplicate parameter \"" + p + "\"");
        m.params.push_back(p);
        if (static_cast<int>(m.params.size()) > lim::kMaxMacroParams)
            return c.Fail(cmd.line, "$definemacro \"" + name + "\": too many "
                                    "parameters (max " +
                                    std::to_string(lim::kMaxMacroParams) + ")");
    }
    // body up to the matching $endmacro - a nested $definemacro takes its own
    for (int depth = 1;;) {
        if (c.Eof())
            return c.Fail(cmd.line, "$definemacro \"" + name +
                                    "\" is missing its $endmacro");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && _stricmp(t.text.c_str(), "$definemacro") == 0) {
            depth++;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "$endmacro") == 0) {
            if (--depth == 0)
                break;
        }
        m.body.push_back(t);
    }
    c.macros[key] = std::move(m);
    return true;
}

// $endmacro only ever closes a $definemacro body; on its own it is a typo.
bool CmdEndMacroOutside(Ctx& c, const Token& cmd) {
    return c.Fail(cmd.line, "$endmacro without a $definemacro");
}

// $<macroname> <arg> ... - splice the body in where the invocation stands, with
// $param$ replaced by the arguments. The spliced tokens are then parsed exactly
// as if written inline, so a macro can hold any commands at all.
bool ExpandMacro(Ctx& c, const Token& cmd, const Ctx::Macro& m) {
    if (++c.macroExpansions > lim::kMaxMacroExpansions)
        return c.Fail(cmd.line, "macro expansion limit reached (" +
                                std::to_string(lim::kMaxMacroExpansions) +
                                ") at \"" + cmd.text + "\" - a macro is recursive");
    std::vector<std::string> args;
    for (const std::string& p : m.params) {
        const std::string what = "an argument for \"" + p + "\"";
        std::string a;
        if (!c.Want(what.c_str(), cmd, a))
            return false;
        args.push_back(std::move(a));
    }
    std::vector<Token> body = m.body;
    for (Token& t : body)
        SubstMacroParams(t, m.params, args);
    c.toks.insert(c.toks.begin() + c.pos, body.begin(), body.end());
    return true;
}

// ---------------------------------------------------------------------------
// Conditionals - $if / $elif / $else and $switch / $case / $default
// ---------------------------------------------------------------------------
//
// Preprocessing, like the two above: the branch that wins is spliced into the
// token stream where the $if stood and runs as if it had been typed there; the
// branches that lose are dropped unread, so an unknown variable inside one is
// not an error. Comparison only - no parentheses, no None()/Not(), no in().

// "0", "false" and the empty string are false; anything else is true.
bool CondTruthy(const std::string& s) {
    return !(s.empty() || s == "0" || _stricmp(s.c_str(), "false") == 0);
}

// Whole-token decimal number, so a comparison of two of them is numeric.
bool CondNumeric(const std::string& s) {
    size_t i = (!s.empty() && (s[0] == '-' || s[0] == '+')) ? 1 : 0;
    bool digits = false;
    for (; i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])); i++)
        digits = true;
    if (i < s.size() && s[i] == '.')
        for (i++; i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])); i++)
            digits = true;
    return digits && i == s.size();
}

// Reversed spellings ("=!", "=>", "=<") are accepted as the same operator.
bool IsCondOp(const std::string& s) {
    return s == "==" || s == "!=" || s == "=!" || s == ">" || s == "<" ||
           s == ">=" || s == "=>" || s == "<=" || s == "=<";
}

// Numeric when both sides are numbers, otherwise a case-SENSITIVE string
// compare - the same rule variable names follow.
bool CondCompare(const std::string& lhs, const std::string& op, const std::string& rhs) {
    if (CondNumeric(lhs) && CondNumeric(rhs)) {
        const double l = std::strtod(lhs.c_str(), nullptr);
        const double r = std::strtod(rhs.c_str(), nullptr);
        if (op == "==") return l == r;
        if (op == "!=" || op == "=!") return l != r;
        if (op == ">") return l > r;
        if (op == "<") return l < r;
        if (op == ">=" || op == "=>") return l >= r;
        return l <= r;
    }
    const int c = lhs.compare(rhs);
    if (op == "==") return c == 0;
    if (op == "!=" || op == "=!") return c != 0;
    if (op == ">") return c > 0;
    if (op == "<") return c < 0;
    if (op == ">=" || op == "=>") return c >= 0;
    return c <= 0;
}

// One condition token. `literal` means the text IS the value (it was quoted or
// came out of a $name$ expansion); a bare word is a variable reference.
struct CondTok {
    std::string text;
    int line = 0;
    bool literal = false;
};

// Consume the '{' that opens a block body.
bool ExpectBrace(Ctx& c, const Token& cmd) {
    if (c.Eof() || c.Cur().quoted || c.Cur().text != "{")
        return c.Fail(cmd.line, cmd.text + " expects a \"{\" block");
    c.pos++;
    return true;
}

// The tokens between a '{' (already consumed) and its matching '}' (dropped),
// kept raw so they expand where they are spliced rather than here.
bool CollectBlock(Ctx& c, const Token& cmd, std::vector<Token>& body) {
    for (int depth = 1;;) {
        if (c.Eof())
            return c.Fail(cmd.line, cmd.text + " is missing its closing \"}\"");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "{") {
            depth++;
        } else if (!t.quoted && t.text == "}" && --depth == 0) {
            break;
        }
        body.push_back(t);
    }
    return true;
}

// Read a condition up to the '{' that opens its block, expanding $name$ as it
// goes. Operators are whitespace-separated tokens: "a == b", not "a==b".
bool ReadCondition(Ctx& c, const Token& cmd, std::vector<CondTok>& out) {
    for (;;) {
        if (c.Eof() || (!c.Cur().quoted && c.Cur().text == "}"))
            return c.Fail(cmd.line, cmd.text + " is missing the \"{\" that opens its block");
        Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "{")
            break;
        if (!ExpandVars(c, t))
            return false;
        out.push_back({t.text, t.line, t.quoted || t.expanded});
    }
    if (out.empty())
        return c.Fail(cmd.line, cmd.text + " has an empty condition");
    return true;
}

// Flat recursive descent over the condition: or -> and -> comparison -> operand.
// Both sides of && and || are always evaluated, so an undefined variable in the
// half that cannot change the answer still reports itself.
struct CondEval {
    Ctx& c;
    const Token& cmd;
    const std::vector<CondTok>& toks;
    size_t pos = 0;
    bool ok = true;

    bool AtEnd() const { return pos >= toks.size(); }
    const std::string& Cur() const { return toks[pos].text; }

    bool Stop(int line, const std::string& msg) {
        c.Fail(line, cmd.text + ": " + msg);
        ok = false;
        return false;
    }

    // An operand: its value text (for a comparison) plus its own truth value.
    bool Operand(std::string& value) {
        value.clear();
        if (AtEnd())
            return Stop(cmd.line, "the condition ends early");
        const CondTok& t = toks[pos++];
        if (t.literal || CondNumeric(t.text)) {
            value = t.text;
            return CondTruthy(value);
        }
        auto it = c.variables.find(t.text);
        if (it == c.variables.end()) {
            if (t.text.find_first_of("=<>&|") != std::string::npos)
                return Stop(t.line, "cannot read \"" + t.text + "\" - an operator "
                                    "needs a space on each side");
            return Stop(t.line, "unknown variable \"" + t.text + "\" - no "
                                "$definevariable for it above this line (a string "
                                "literal goes in quotes)");
        }
        value = it->second;
        return CondTruthy(value);
    }

    bool Comparison() {
        std::string lhs;
        const bool truth = Operand(lhs);
        if (!ok || AtEnd() || !IsCondOp(Cur()))
            return truth;
        const std::string op = toks[pos++].text;
        std::string rhs;
        Operand(rhs);
        return ok ? CondCompare(lhs, op, rhs) : false;
    }

    bool And() {
        bool r = Comparison();
        while (ok && !AtEnd() && Cur() == "&&") {
            pos++;
            const bool rhs = Comparison();
            r = r && rhs;
        }
        return r;
    }

    bool Or() {
        bool r = And();
        while (ok && !AtEnd() && Cur() == "||") {
            pos++;
            const bool rhs = And();
            r = r || rhs;
        }
        return r;
    }
};

bool EvalCondition(Ctx& c, const Token& cmd, const std::vector<CondTok>& toks,
                   bool& result) {
    CondEval e{c, cmd, toks};
    const bool r = e.Or();
    if (!e.ok)
        return false;
    if (e.pos < toks.size())
        return c.Fail(toks[e.pos].line, cmd.text + ": unexpected \"" +
                                        toks[e.pos].text + "\" in the condition");
    result = r;
    return true;
}

// $ifdef <variable> - true when the name has a value, whatever that value is.
// The name is taken bare and unexpanded; anything past it means a comparison
// was written, which belongs under $if.
bool ReadDefName(Ctx& c, const Token& cmd, bool& result) {
    if (c.Eof() || c.Cur().quoted || c.Cur().text == "{" || c.Cur().text == "}")
        return c.Fail(cmd.line, cmd.text + " expects a variable name");
    const Token name = c.toks[c.pos++];
    if (c.Eof() || c.Cur().quoted || c.Cur().text != "{")
        return c.Fail(name.line, cmd.text + " takes one bare variable name - a "
                                 "condition belongs under $if, not $ifdef");
    c.pos++;
    result = c.variables.find(name.text) != c.variables.end();
    return true;
}

// $if <cond> { ... } [$elif <cond> { ... }]... [$else { ... }], and $ifdef with
// the same shape but a name check in every clause. Every clause's condition is
// evaluated even once one has won, so a typo in a later $elif is still
// reported; only the BODIES of the losers go unread.
bool IfChain(Ctx& c, const Token& cmd, bool ifdef) {
    std::vector<Token> chosen;
    bool taken = false;
    for (Token clause = cmd;;) {
        bool val = false;
        std::vector<Token> body;
        if (ifdef) {
            if (!ReadDefName(c, clause, val))
                return false;
        } else {
            std::vector<CondTok> cond;
            if (!ReadCondition(c, clause, cond) || !EvalCondition(c, clause, cond, val))
                return false;
        }
        if (!CollectBlock(c, clause, body))
            return false;
        if (val && !taken) {
            taken = true;
            chosen = std::move(body);
        }
        if (c.Eof() || c.Cur().quoted || _stricmp(c.Cur().text.c_str(), "$elif") != 0)
            break;
        clause = c.toks[c.pos++];
    }
    if (!c.Eof() && !c.Cur().quoted && _stricmp(c.Cur().text.c_str(), "$else") == 0) {
        const Token els = c.toks[c.pos++];
        std::vector<Token> body;
        if (!ExpectBrace(c, els) || !CollectBlock(c, els, body))
            return false;
        if (!taken)
            chosen = std::move(body);
    }
    c.toks.insert(c.toks.begin() + c.pos, chosen.begin(), chosen.end());
    return true;
}

bool CmdIf(Ctx& c, const Token& cmd) { return IfChain(c, cmd, false); }
bool CmdIfdef(Ctx& c, const Token& cmd) { return IfChain(c, cmd, true); }

// $switch <variable> { $case <value> { ... } ... $default { ... } }. The value
// compare is exact - a $case is a label, not a condition.
bool CmdSwitch(Ctx& c, const Token& cmd) {
    if (c.Eof() || (!c.Cur().quoted && c.Cur().text == "{"))
        return c.Fail(cmd.line, "$switch expects a variable name");
    const Token var = c.toks[c.pos++];
    if (HasVarRef(var.text))
        return c.Fail(var.line, "$switch takes the bare variable name, not \"" +
                                var.text + "\"");
    auto it = c.variables.find(var.text);
    if (it == c.variables.end())
        return c.Fail(var.line, "$switch: unknown variable \"" + var.text +
                                "\" - no $definevariable for it above this line");
    const std::string value = it->second;
    if (!ExpectBrace(c, cmd))
        return false;

    std::vector<Token> chosen, fallback;
    bool taken = false, hasDefault = false;
    for (;;) {
        if (c.Eof())
            return c.Fail(cmd.line, "$switch is missing its closing \"}\"");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        if (t.quoted || _stricmp(t.text.c_str(), "$case") != 0) {
            if (t.quoted || _stricmp(t.text.c_str(), "$default") != 0)
                return c.Fail(t.line, "$switch: expected $case or $default, got \"" +
                                      t.text + "\"");
            if (hasDefault)
                return c.Fail(t.line, "$switch has more than one $default");
            hasDefault = true;
            if (!ExpectBrace(c, t) || !CollectBlock(c, t, fallback))
                return false;
            continue;
        }
        if (c.Eof() || (!c.Cur().quoted && (c.Cur().text == "{" || c.Cur().text == "}")))
            return c.Fail(t.line, "$case expects a value");
        Token label = c.toks[c.pos++];
        if (!ExpandVars(c, label))
            return false;
        std::vector<Token> body;
        if (!ExpectBrace(c, t) || !CollectBlock(c, t, body))
            return false;
        if (!taken && label.text == value) {
            taken = true;
            chosen = std::move(body);
        }
    }
    if (!hasDefault)
        return c.Fail(cmd.line, "$switch requires a $default - say what happens "
                                "when no $case matches, even if it is nothing");
    if (!taken)
        chosen = std::move(fallback);
    c.toks.insert(c.toks.begin() + c.pos, chosen.begin(), chosen.end());
    return true;
}

// CmdIf and CmdSwitch consume their own clauses, so one reaching command
// position is standing on its own.
bool CmdElifOutside(Ctx& c, const Token& cmd) {
    return c.Fail(cmd.line, cmd.text + " without an $if above it");
}

bool CmdCaseOutside(Ctx& c, const Token& cmd) {
    return c.Fail(cmd.line, cmd.text + " outside a $switch body");
}

// ---------------------------------------------------------------------------
// Command table - the whole supported surface. Add, rename or drop a line and
// the language changes; nothing else needs touching.
// ---------------------------------------------------------------------------

struct Command {
    const char* name;
    bool (*fn)(Ctx&, const Token&);
};

constexpr Command kCommands[] = {
    {"$include", CmdInclude},
    {"$break", CmdBreak},
    {"$assert", CmdAssert},
    {"$print", CmdPrint},
    {"$definevariable", CmdDefineVariable},
    {"$redefinevariable", CmdRedefineVariable},
    {"$definemacro", CmdDefineMacro},
    {"$endmacro", CmdEndMacroOutside},
    {"$if", CmdIf},
    {"$ifdef", CmdIfdef},
    {"$elif", CmdElifOutside},
    {"$else", CmdElifOutside},
    {"$switch", CmdSwitch},
    {"$case", CmdCaseOutside},
    {"$default", CmdCaseOutside},
    {"$addincludesearchdir", CmdAddIncludeSearchDir},
    {"$addsearchdir", CmdAddSearchDir},
    {"$modelname", CmdModelName},
    {"$rendermesh", CmdRenderMesh},
    {"$modelgroup", CmdModelGroup},
    {"$modelgrouppreset", CmdModelGroupPreset},
    {"$datamodeljoints", CmdDataModelJoints},
    {"$datamodelflexes", CmdDataModelFlexes},
    {"$wrinklescale", CmdWrinkleScale},
    {"$animation", CmdAnimation},
    {"$bindposeanimation", CmdBindPoseAnimation},
    {"$declareanimation", CmdDeclareAnimation},
    {"$sequence", CmdSequence},
    {"$bindposesequence", CmdBindPoseSequence},
    {"$declaresequence", CmdDeclareSequence},
    {"$append", CmdAppend},
    {"$prepend", CmdPrepend},
    {"$continue", CmdContinue},
    {"$includemodel", CmdIncludeModel},
    {"$cmdlist", CmdCmdList},
    {"$defaultfps", CmdDefaultFps},
    {"$defaultfadein", CmdDefaultFadeIn},
    {"$defaultfadeout", CmdDefaultFadeOut},
    {"$lcaseallsequences", CmdLCaseAllSequences},
    {"$allowactivityname", CmdAllowActivityName},
    {"$sectionframes", CmdSectionFrames},
    {"$animblocksize", CmdAnimBlockSize},
    {"$bonesaveframe", CmdBoneSaveFrame},
    {"$poseparameter", CmdPoseParameter},
    {"$ikchain", CmdIkChain},
    {"$ikautoplaylock", CmdIkAutoplayLock},
    {"$weightlist", CmdWeightList},
    {"$defaultweightlist", CmdDefaultWeightList},
    {"$modelarchetype", CmdModelArchetype},
    {"$vtxformat", CmdVtxFormat},
    {"$modelbudget", CmdModelBudget},
    {"$setbindpose", CmdSetBindPose},
    {"$setflex", CmdSetFlex},
    {"$renderpass", CmdRenderPass},
    {"$surfaceprop", CmdSurfaceProp},
    {"$contents", CmdContents},
    {"$jointcontents", CmdJointContents},
    {"$keyvalues", CmdKeyValues},
    {"$boneflexdriver", CmdBoneFlexDriver},
    {"$flexcontroller", CmdFlexController},
    {"$flexlocalvar", CmdFlexLocalVar},
    {"$flexrule", CmdFlexRule},
    {"$flexcorrective", CmdFlexCorrective},
    {"$flexdominate", CmdFlexDominate},
    {"$morphsplitstereo", CmdMorphSplitStereo},
    {"$flexcullmethod", CmdFlexCullMethod},
    {"$animationcullmethod", CmdAnimationCullMethod},
    {"$eyeball", CmdEyeball},
    {"$mouth", CmdMouth},
    {"$eyelid", CmdEyelid},
    {"$skiptransition", CmdSkipTransition},
    {"$calctransitions", CmdCalcTransitions},
    {"$transformmodel", CmdTransformModel},
    {"$upaxis", CmdUpAxis},
    {"$attachment", CmdAttachment},
    {"$declareattachment", CmdDeclareAttachment},
    {"$hitboxset", CmdHitboxSet},
    {"$hboxset", CmdHitboxSet}, // stock's spelling, same command
    {"$hbox", CmdHboxOutsideSet},
    {"$renamehboxset", CmdRenameHboxSet},
    {"$bonecullmethod", CmdBoneCullMethod},
    {"$physicsmodel", CmdPhysicsModel},
    {"$physicsshape", CmdPhysicsOutsideModel},
    {"$physicsjoint", CmdPhysicsOutsideModel},
    {"$physicsmarkup", CmdPhysicsOutsideModel},
    {"$physicscollide", CmdPhysicsOutsideModel},
    {"$jigglebone", CmdJiggleBone},
    {"$driverbone", CmdDriverBone},
    {"$driveraimat", CmdDriverAimAt},
    {"$proceduralbones", CmdProceduralBones},
    {"$realignbones", CmdRealignBones},
    {"$lockbonelengths", CmdLockBoneLengths},
    {"$transformbone", CmdTransformBone},
    {"$root", CmdRoot},
    {"$definebone", CmdDefineBone},
    {"$unlockdefinebones", CmdUnlockDefineBones},
    {"$bonemerge", CmdBoneMerge},
    {"$donotcollapse", CmdDoNotCollapse},
    {"$alwayscollapse", CmdAlwaysCollapse},
    {"$hierarchy", CmdHierarchy},
    {"$heirarchy", CmdHierarchy}, // stock's misspelling, same command
    {"$ambientboost", CmdAmbientBoost},
    {"$donotcastshadows", CmdDoNotCastShadows},
    {"$forcephonemecrossfade", CmdForcePhonemeCrossfade},
    {"$skipboneinbbox", CmdSkipBoneInBBox},
    {"$bbox", CmdBBox},
    {"$cbox", CmdCBox},
    {"$illumposition", CmdIllumPosition},
    {"$eyeposition", CmdEyePosition},
    {"$maxeyedeflection", CmdMaxEyeDeflection},
    {"$cdmaterials", CmdCdMaterials},
    {"$texturegroup", CmdTextureGroup},
    {"$lod", CmdLod},
    {"$shadowlod", CmdLod},
};

std::string SupportedList();

bool IsCommandName(const std::string& name) {
    for (const Command& k : kCommands)
        if (_stricmp(k.name, name.c_str()) == 0)
            return true;
    return false;
}

bool RunCommands(Ctx& c) {
    while (!c.Eof()) {
        if (c.scriptBreak) // $break, here or in a file this one $included
            return true;
        if (!ExpandCommandVars(c))
            return false;

        const Token cmd = c.toks[c.pos];
        if (cmd.quoted || cmd.text[0] != '$')
            return c.Fail(cmd.line, "expected a $command, got \"" + cmd.text + "\"");
        c.pos++;

        const Command* found = nullptr;
        for (const Command& k : kCommands) {
            if (_stricmp(k.name, cmd.text.c_str()) == 0) {
                found = &k;
                break;
            }
        }
        if (!found) {
            // not built in - a $definemacro name is the only other thing a
            // $word at command position can be
            std::string key = cmd.text.substr(1);
            for (char& ch : key)
                ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            auto m = c.macros.find(key);
            if (m != c.macros.end()) {
                if (!ExpandMacro(c, cmd, m->second))
                    return false;
                continue;
            }
            return c.Fail(cmd.line, "unknown command \"" + cmd.text +
                                    "\" - supported: " + SupportedList());
        }
        if (!found->fn(c, cmd))
            return false;
    }
    return true;
}

std::string SupportedList() {
    std::ostringstream os;
    for (size_t i = 0; i < std::size(kCommands); ++i) {
        if (i) os << ", ";
        os << kCommands[i].name;
    }
    return os.str();
}

} // namespace

bool IsQcScriptPath(const char* path) {
    const std::string ext = fs::path(path).extension().string();
    return _stricmp(ext.c_str(), ".pulseqc") == 0 || _stricmp(ext.c_str(), ".qc") == 0;
}

bool LoadQcScript(const char* path, cm::CompileInput& out, std::string* err,
                  const ScriptVars& defvars) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (err) *err = std::string("cannot open \"") + path + "\"";
        return false;
    }
    std::ostringstream buf;
    buf << f.rdbuf();
    std::string text = buf.str();
    StripUtf8Bom(text);

    Ctx c{out};
    c.scriptDir = fs::path(path).parent_path();
    c.curDir = c.scriptDir;
    c.file = fs::path(path).filename().string();
    c.err = err;

    // -defvar: in effect from the first line, and pinned - a later repeat on
    // the command line just wins over the earlier one
    for (const auto& kv : defvars) {
        c.variables[kv.first] = kv.second;
        if (!c.IsLockedVar(kv.first))
            c.lockedVars.push_back(kv.first);
    }

    if (!Tokenize(text, c.file, c.toks, err))
        return false;

    // the root of the $include cycle check
    std::error_code ec;
    const fs::path canon = fs::weakly_canonical(path, ec);
    c.includeStack.push_back(ec ? std::string(path) : canon.string());

    out.mdlPath = path;

    if (!RunCommands(c))
        return false;

    if (out.outname.empty()) {
        if (err) *err = c.file + ": no $modelname";
        return false;
    }
    // animation-only models (.mdl + .ani, no geometry) are legal
    if (out.bodyparts.empty())
        printf("WARNING: %s: no $modelgroup\n", c.file.c_str());

    // set once every source has been read (the DMX loader latches it on the
    // first model carrying an upAxis attribute)
    out.upAxisY = source::DmxUpAxisY();

    // $wrinklescale: every source, including the merged ones a `mesh` line built,
    // so it never depends on where in the script it was written
    if (!c.wrinkleScales.empty()) {
        for (auto& src : c.in.sources)
            source::ApplyWrinkleScales(*src, c.wrinkleScales);
        for (const source::WrinkleScaleOption& w : c.wrinkleScales)
            if (!w.matched)
                return c.Fail(w.line, "$wrinklescale names \"" + w.shape +
                                          "\", which is not a morph in any $rendermesh");
    }

    // flex/morph: the automatic per-body DMX rig plus the $flexcontroller /
    // $flexlocalvar / $flexrule / $flexcorrective block. Needs the finished
    // bodygroup list, so it runs here rather than per command.
    {
        std::string flexErr;
        if (!RegisterFlex(out, c.manual, &flexErr)) {
            if (err) *err = c.file + ": " + flexErr;
            return false;
        }
        // face markup appends to the tables the flex pass just built, so it
        // runs second - the same order the .pulsemdl loader uses
        if (!RegisterFaceMarkup(out, c.face, &flexErr)) {
            if (err) *err = c.file + ": " + flexErr;
            return false;
        }
    }

    // needs the final rotation/scale, so it runs after everything above
    FinishAttachments(c);
    return true;
}

} // namespace pulse::loader
