// qcloader.cpp - keyvalues1 compile-script loader. See qcloader.h.

#include "strcompat.h"
#include "pathcompat.h"

#include "perf.h"
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
// and `/* */` block comments, and braces as standalone tokens even when jammed
// against a word.
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
                d == '"' ||
                (d == '/' && i + 1 < s.size() && (s[i + 1] == '/' || s[i + 1] == '*')))
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

struct Ctx;
// Resolves any $include or conditional ($if/$switch) at the read cursor in place,
// so both are transparent at any nesting depth - a handler reading its own braced
// body gets them resolved too. Defined under the helpers it needs.
void ResolveDirectives(Ctx& c);

struct Ctx {
    cm::CompileInput& in;
    fs::path scriptDir; // root script's directory
    std::string file;
    std::string rootFile; // `file` for the root script; an $include changes `file`, not this
    std::vector<Token> toks;
    size_t pos = 0;
    std::string* err = nullptr;
    fs::path curDir; // dir of the file currently being parsed ($include is relative to it)
    std::vector<std::string> includeStack; // cycle guard, LIFO with includeFrames
    // one open $include region: pos>=endPos restores file/curDir and pops the
    // cycle-guard entry. endPos is an absolute index, kept live by SpliceAt.
    struct IncludeFrame { size_t endPos; std::string savedFile; fs::path savedDir; };
    std::vector<IncludeFrame> includeFrames;
    bool abortErr = false;        // a hard $include error; stops parsing at the next read
    bool resolvingInclude = false; // reentrancy guard for ResolveIncludes
    // set while a block is captured raw ($if/$switch branch, macro or cmdlist
    // body): a $include there is stored verbatim and resolved when executed, not
    // during capture.
    bool rawCollect = false;
    std::vector<fs::path> includeDirs; // $addincludesearchdir
    std::vector<fs::path> sourceDirStack; // root script dir plus nested $pushd dirs
    std::vector<fs::path> searchDirs;  // $addsearchdir, for source files - never mixed with includeDirs
    // -includesearchdir / -filesearchdir: searched only after the script's own
    // dirs, so a script command always wins.
    std::vector<fs::path> launchIncludeDirs;
    std::vector<fs::path> launchSearchDirs;
    std::map<std::string, source::Source*> rendermeshes; // $rendermesh name -> loaded source
    // filename -> shared source. A bodied $rendermesh is excluded: its edits must not leak to other refs.
    std::map<std::string, source::Source*> sourceCache;
    // full path -> parsed DMX. Read-only input, so the same file feeding several
    // loads (each $rendermesh reopens it) is read and parsed once.
    std::map<std::string, std::shared_ptr<pulse::dmx::Datamodel>> dmxCache;
    source::MaterialTable physMats; // collision-only materials, kept out of the model's texture table
    std::map<std::string, int> namedAnims; // $animation name -> index into in.anims
    std::map<std::string, std::vector<Token>> cmdlists; // $cmdlist name -> body tokens
    std::map<std::string, std::string> variables; // $definevariable, case-sensitive, shared across $include
    struct Macro {
        std::vector<std::string> params;
        std::map<std::string, std::string> defaults;
        std::vector<Token> body;
    };
    std::map<std::string, Macro> macros; // keyed lowercased
    int macroExpansions = 0; // runaway-recursion guard
    std::vector<std::string> lockedVars; // -defvar names; locked against $definevariable
    bool scriptBreak = false; // $break, sticky through every $include parent

    float defaultFps = 30.0f;
    float defaultMotionRollback = 0.3f; // seed for a new anim's motionrollback
    float defaultFadeIn = 0.2f;
    float defaultFadeOut = 0.2f;
    bool lcaseSequences = false;
    std::vector<std::string> allowedActivities; // $allowactivityname, exact-case
    bool unlockDefineBones = false;
    int activeHitboxSet = -1;

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

    // $renamemorph, applied once the script is parsed so placement never matters
    struct MorphRename {
        std::string from;
        std::string to;
        int line = 0;
    };
    std::vector<MorphRename> morphRenames;

    // $flexcontroller/$flexlocalvar/$flexrule/$flexcorrective, held until the
    // bodygroup list is final - registration order can't run command by command
    ManualFlex manual;

    // $eyeball/$mouth/$eyelid, held for the same reason; one list across all
    // three because their relative order is load-bearing (indices, flexdescs)
    FaceMarkup face;

    // Insert tokens at `at`, keeping open include-region boundaries in sync so
    // a macro/conditional/include splice inside an included file cannot desync
    // the frame that restores curDir/file when its region ends.
    void SpliceAt(size_t at, const std::vector<Token>& src) {
        toks.insert(toks.begin() + at, src.begin(), src.end());
        for (IncludeFrame& f : includeFrames)
            if (f.endPos > at)
                f.endPos += src.size();
    }

    bool Eof() { ResolveDirectives(*this); return abortErr || pos >= toks.size(); }
    const Token& Cur() { ResolveDirectives(*this); return toks[pos]; }

    // True when -defvar owns this variable name (case-sensitive, like the
    // variable table itself).
    bool IsLockedVar(const std::string& name) const {
        for (const std::string& v : lockedVars)
            if (v == name)
                return true;
        return false;
    }

    bool Fail(int line, const std::string& msg) {
        if (abortErr) return false; // keep the first $include error, not a cascade
        if (err) *err = file + "(" + std::to_string(line) + "): " + msg;
        return false;
    }

    // Next token, or null at end of file.
    const Token* Next() { return Eof() ? nullptr : &toks[pos++]; }

    // True when the next token starts a new command (or the file ended) - the
    // terminator for an unbraced option list.
    bool AtCommand() { return Eof() || (!Cur().quoted && Cur().text[0] == '$'); }

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

    // Like WantInt but any token starting with "." (".", "..", ...) is the
    // unset placeholder -> -1 (the ikrule `range` frame markers).
    bool WantFrame(const char* what, const Token& cmd, int& value) {
        std::string s;
        if (!Want(what, cmd, s))
            return false;
        if (!s.empty() && s[0] == '.') { value = -1; return true; }
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
    bool NextIsInt() {
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

// Where a source file (.dmx/.smd/.vrd) actually is: the current $pushd
// directory first, then each $addsearchdir dir in registration order.
// Empty return = nowhere, and `tried` then holds every candidate.
fs::path FindSourceFile(const Ctx& c, const std::string& filename,
                        std::vector<fs::path>* tried) {
    const fs::path rel = pulse::FilePath(filename);
    std::vector<fs::path> cands;
    if (rel.is_absolute()) {
        cands.push_back(rel);
    } else {
        cands.push_back(c.sourceDirStack.back() / rel);
        for (const fs::path& dir : c.searchDirs)
            cands.push_back(dir / rel);
        for (const fs::path& dir : c.launchSearchDirs)
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
    source::WeldMode weld = source::WeldMode::None;
    bool noMorph = false;
    float inflate = 0.0f;
    bool flipNormals = false;
    float decimate = 1.0f; // $decimate: face-count fraction, 1 = untouched
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
// Parse a DMX once per file. The document is immutable input - the Source each
// caller builds from it is still built fresh and stays private.
std::shared_ptr<pulse::dmx::Datamodel> LoadDmxCached(Ctx& c, const fs::path& full,
                                                     std::string* err) {
    const std::string key = Lower(full.string());
    auto it = c.dmxCache.find(key);
    if (it != c.dmxCache.end())
        return it->second;
    std::shared_ptr<pulse::dmx::Datamodel> dm;
    { PULSE_PERF("load", "dmx parse");
      dm = pulse::dmx::Datamodel::Load(full.string().c_str(), err); }
    if (dm)
        c.dmxCache[key] = dm;
    return dm;
}

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
                                   morphSource, edit ? &edit->filter : nullptr, animOnly)) {
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
        auto dm = LoadDmxCached(c, full, &loadErr);
        if (!dm) {
            c.Fail(line, "cannot load \"" + full.string() + "\": " + loadErr);
            return nullptr;
        }
        std::printf("%s (%s %d, %s %d)\n", head.c_str(), dm->format.c_str(), dm->format_version,
                    dm->encoding.c_str(), dm->encoding_version);
        PULSE_PERF("load", "dmx -> source");
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

    if (edit) {
        source::WeldVertices(*src, edit->weld);
        source::CullUnskinnedBones(*src, edit->boneCull);
    }

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

            // $removemeshword <keyword> - drop every mesh whose material name
            // contains the keyword (case-insensitive substring). Repeatable.
            if (o == "$removemeshword") {
                std::string word;
                if (!c.Want("a material keyword", *t, word))
                    return false;
                edit.filter.removeWords.push_back(word);
                continue;
            }

            if (o == "$inflate") {
                if (!c.WantFloat("an inflation amount", *t, edit.inflate))
                    return false;
                if (!std::isfinite(edit.inflate))
                    return c.Fail(t->line, where + ": $inflate amount must be finite");
                continue;
            }

            if (o == "$flipnormals") {
                edit.flipNormals = true;
                continue;
            }

            if (o == "$weld") {
                edit.weld = source::WeldMode::KeepSeams;
                if (!c.Eof() && !c.Cur().quoted && Lower(c.Cur().text) == "seams") {
                    ++c.pos;
                    edit.weld = source::WeldMode::All;
                }
                continue;
            }

            // $decimate <factor> - simplify this render mesh to `factor` of its
            // face count as it loads, the way $lod decimatemodel does per LOD.
            if (o == "$decimate") {
                if (!c.WantFloat("a factor", *t, edit.decimate))
                    return false;
                if (edit.decimate <= 0.0f || edit.decimate > 1.0f)
                    return c.Fail(t->line, where + ": $decimate factor must be in (0, 1.0]");
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
                    edit.vcaName = StripExtension(pulse::FilePath(edit.vca).filename().string());
                continue;
            }

            return c.Fail(t->line, where + ": expected $exceptionlist, $removemeshword, "
                                           "$skinnedbonecull, $weld, $inflate, $flipnormals, $decimate, $nomorph, $vta, "
                                           "$vca or '}', got \"" + t->text + "\"");
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

    source::InflateVertices(*src, edit.inflate);
    if (edit.flipNormals)
        source::FlipNormals(*src);

    if (edit.decimate < 1.0f)
        source::SimplifyFaces(*src, *src, edit.decimate,
                              c.in.archetype == cm::Archetype::Static);

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

// $modelgroup/$model <name> <refs...> or { mesh <refs...> | blank ... }.
// Refs are render-mesh aliases or explicit .dmx/.smd/.fbx files.
// Use `name <display>` to disambiguate display names from refs.
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
            source::Source* src = it != c.rendermeshes.end() ? it->second : nullptr;
            const std::string ext = Lower(pulse::FilePath(r.text).extension().string());
            if (!src && (ext == ".dmx" || ext == ".smd" || ext == ".fbx")) {
                if (std::find(refNames.begin(), refNames.end(), r.text) != refNames.end())
                    return c.Fail(r.line, where + ": mesh \"" + r.text + "\" is listed twice");
                MeshEdit edit;
                edit.name = r.text;
                src = LoadSource(c, r.text, r.line, /*morphSource=*/true, &edit);
                if (!src)
                    return false;
            }
            if (!src) {
                // A leading non-reference word is the display name.
                if (!named && refs.empty() && defName.empty()) {
                    studio = r.text;
                    named = true;
                    continue;
                }
                return c.Fail(r.line,
                              where + " references unknown rendermesh \"" + r.text +
                              "\"; direct source files require .smd, .dmx or .fbx");
            }
            if (std::find(refs.begin(), refs.end(), src) != refs.end())
                return c.Fail(r.line, where + ": mesh \"" + r.text + "\" is listed twice");
            refs.push_back(src);
            refNames.push_back(r.text);
        }
        if (refs.empty())
            return c.Fail(line, where + ": mesh expects a $rendermesh name or .smd/.dmx/.fbx file");

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
std::shared_ptr<pulse::dmx::Datamodel> LoadRigDmx(Ctx& c, const Token& cmd,
                                                 const std::string& filename,
                                                 const std::string& what) {
    const fs::path full = FindRigFile(c, cmd, filename);
    if (full.empty())
        return nullptr;
    std::string loadErr;
    auto dm = LoadDmxCached(c, full, &loadErr);
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
        const size_t oldCorrectives = dst.correctives.size();
        for (const source::FlexRig::Corrective& cor : src.correctives) {
            bool dup = false; // a corrective a PREVIOUS merge already contributed
            for (size_t j = 0; j < oldCorrectives; ++j)
                if (_stricmp(dst.correctives[j].delta.c_str(), cor.delta.c_str()) == 0) {
                    dup = true;
                    break;
                }
            if (dup)
                continue;
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

// Load a direct file as a private morph Source (reverse/scale applied). For a
// DMX it auto-imports the joints, and the flex rig too unless suppressed
// (legacy noautodmxrules) - the morph deltas stay on the Source either way.
// `file` must already be extension-resolved. null return = failure (reported).
source::Source* LoadStudioSource(Ctx& c, const Token& entry, const std::string& file,
                                 bool flip, float importScale, bool importFlexRig) {
    MeshEdit edit;
    edit.name = file;
    edit.flipNormals = flip;
    const float defaultScale = c.in.scale;
    c.in.scale = importScale;
    source::Source* src = LoadSource(c, file, entry.line, /*morphSource=*/true, &edit);
    c.in.scale = defaultScale;
    if (!src)
        return nullptr;
    if (flip)
        source::FlipNormals(*src);
    if (!IsSmdPath(file) && !IsFbxPath(file)) {
        auto dm = LoadRigDmx(c, entry, file, "studio rig");
        if (!dm)
            return nullptr;
        source::FlexRig rig;
        std::string err;
        if (!LoadDmxJoints(*dm, c.in, true, true, true, true, &err) ||
            (importFlexRig && !source::LoadDmxFlexRig(*dm, rig, &err))) {
            c.Fail(entry.line, "\"" + file + "\": " + err);
            return nullptr;
        }
        if (importFlexRig)
            MergeFlexRig(c.manual.datamodel, rig, true, true, true, true);
    }
    return src;
}

// $body and $bodygroup load private, file-only choices with automatic DMX rig imports.
bool CmdLegacyBody(Ctx& c, const Token& cmd) {
    cm::CompileInput::InBodyPart part;
    if (!c.Want("a name", cmd, part.name))
        return false;
    const std::string where = cmd.text + " \"" + part.name + "\"";
    for (const auto& b : c.in.bodyparts)
        if (b.name == part.name)
            return c.Fail(cmd.line, where + " already exists");

    auto studio = [&](const Token& entry, bool inlineBody) -> bool {
        if (c.AtCommand() || (!c.Cur().quoted &&
            (c.Cur().text == "{" || c.Cur().text == "}")))
            return c.Fail(entry.line, where + ": studio expects a source filename");
        std::string file;
        if (!c.Want("a source filename", entry, file))
            return false;
        file = WithSourceExtension(c, file);
        bool flip = false;
        float importScale = c.in.scale;
        while (inlineBody && !c.AtCommand() && (c.Cur().quoted || c.Cur().text != "}")) {
            const Token option = *c.Next();
            const std::string o = Lower(option.text);
            if (!option.quoted && o == "reverse") {
                flip = true;
            } else if (!option.quoted && o == "scale") {
                if (!c.WantFloat("an import scale", option, importScale))
                    return false;
                if (!std::isfinite(importScale) || importScale <= 0.0f)
                    return c.Fail(option.line, where + ": scale must be finite and greater than 0");
            } else {
                return c.Fail(option.line, where + ": expected reverse or scale <value>; "
                                                   "use $rendermesh and $modelgroup for mesh edits");
            }
        }
        source::Source* src =
            LoadStudioSource(c, entry, file, flip, importScale, /*importFlexRig=*/true);
        if (!src)
            return false;
        cm::CompileInput::InModel model;
        model.name = inlineBody ? part.name : cm::ChoiceName({file});
        model.source = src;
        part.models.push_back(std::move(model));
        return true;
    };

    if (_stricmp(cmd.text.c_str(), "$body") == 0) {
        if (!studio(cmd, true))
            return false;
    } else {
        const Token* brace = c.Next();
        if (!brace || brace->quoted || brace->text != "{")
            return c.Fail(cmd.line, where + " expects '{'");
        for (;;) {
            const Token* t = c.Next();
            if (!t)
                return c.Fail(cmd.line, where + " is missing '}'");
            if (!t->quoted && t->text == "}")
                break;
            if (!t->quoted && _stricmp(t->text.c_str(), "studio") == 0) {
                if (!studio(*t, false))
                    return false;
            } else if (!t->quoted && _stricmp(t->text.c_str(), "blank") == 0) {
                cm::CompileInput::InModel model;
                model.name = "blank";
                model.source = nullptr;
                part.models.push_back(std::move(model));
            } else {
                return c.Fail(t->line, where + ": expected studio <file>, blank or '}'; "
                                              "use $rendermesh and $modelgroup for mesh edits");
            }
        }
    }
    if (part.models.empty())
        return c.Fail(cmd.line, where + " expects at least one choice");
    c.in.bodyparts.push_back(std::move(part));
    return true;
}

// $model <name> <sourcefile> [reverse] [scale <f>] [subd|faces a b|bias x]
//        [ { <face / flex options> } ]
// The legacy single-mesh body part: one direct file (never a $rendermesh), an
// auto DMX rig import like $body, and a block of face/flex markup. noautodmxrules
// drops only the flex-rig import (the deltas stay). Every flex/eye option is
// GLOBAL, as in stock studiomdl - it can reach other bodies' flexes, the
// accepted trade. VTA options (flexfile/flex/flexpair/defaultflex/vcafile and the
// frame-addressed eyelid) are SMD-only; a DMX carries its own delta states.
bool CmdModel(Ctx& c, const Token& cmd) {
    cm::CompileInput::InBodyPart part;
    if (!c.Want("a name", cmd, part.name))
        return false;
    const std::string where = cmd.text + " \"" + part.name + "\"";
    for (const auto& b : c.in.bodyparts)
        if (b.name == part.name)
            return c.Fail(cmd.line, where + " already exists");

    std::string file;
    if (!c.Want("a source filename", cmd, file))
        return false;
    file = WithSourceExtension(c, file);

    // inline studio flags up to the '{' or the next command
    bool flip = false;
    float importScale = c.in.scale;
    while (!c.AtCommand() && !(!c.Cur().quoted && c.Cur().text == "{")) {
        const Token option = *c.Next();
        const std::string o = Lower(option.text);
        if (option.quoted) {
            return c.Fail(option.line, where + ": expected reverse/scale/subd/faces/bias or '{'");
        } else if (o == "reverse") {
            flip = true;
        } else if (o == "scale") {
            if (!c.WantFloat("an import scale", option, importScale))
                return false;
            if (!std::isfinite(importScale) || importScale <= 0.0f)
                return c.Fail(option.line, where + ": scale must be finite and greater than 0");
        } else if (o == "subd") {
            // no quad subdivision in this compiler (matches importqc); accepted
        } else if (o == "faces") {
            float a, b;
            if (!c.WantFloat("a faces arg", option, a) || !c.WantFloat("a faces arg", option, b))
                return false; // consumed and ignored
        } else if (o == "bias") {
            float x;
            if (!c.WantFloat("a bias arg", option, x))
                return false; // consumed and ignored
        } else {
            return c.Fail(option.line, where + ": expected reverse/scale/subd/faces/bias or '{', "
                                               "got \"" + option.text + "\"");
        }
    }

    // A '{' block? Pre-scan it for noautodmxrules before the rig import decides
    // whether to pull the flex rig (the reference does the same lookahead).
    const bool hasBlock = !c.Eof() && !c.Cur().quoted && c.Cur().text == "{";
    bool importFlexRig = true;
    if (hasBlock) {
        int depth = 0;
        for (size_t i = c.pos; i < c.toks.size(); ++i) {
            const Token& t = c.toks[i];
            if (t.quoted)
                continue;
            if (t.text == "{") {
                depth++;
            } else if (t.text == "}") {
                if (--depth == 0)
                    break;
            } else if (Lower(t.text) == "noautodmxrules") {
                importFlexRig = false;
                break;
            }
        }
    }

    source::Source* src = LoadStudioSource(c, cmd, file, flip, importScale, importFlexRig);
    if (!src)
        return false;
    cm::CompileInput::InModel model;
    model.name = part.name;
    model.source = src;
    part.models.push_back(std::move(model));
    c.in.bodyparts.push_back(std::move(part));

    if (!hasBlock)
        return true;
    ++c.pos; // consume '{'

    static const char* kLidSlot[3] = {"lowerer", "neutral", "raiser"};
    std::string flexfile;                                    // current flex VTA
    std::map<std::string, std::vector<source::VtaFlexOption>> vtaByFile; // file -> shapes
    std::string vcaFile, vcaName;
    int vtaLine = 0, vcaLine = 0;

    auto atEnd = [&] { return c.Eof() || (!c.Cur().quoted && c.Cur().text == "}"); };
    auto sameLine = [&](int line) { return !atEnd() && c.Cur().line == line; };

    auto isModelOption = [](const std::string& lo) {
        return lo == "eyeball" || lo == "eyelid" || lo == "dmxeyelid" || lo == "flexfile" ||
               lo == "flex" || lo == "flexpair" || lo == "defaultflex" || lo == "vcafile" ||
               lo == "localvar" || lo == "mouth" || lo == "flexcontroller" ||
               lo == "noautodmxrules" || lo == "attachment" || lo == "spherenormals";
    };
    // A `%name =` at the cursor starts a NEW rule; a bare `%name` is a fetch
    // operand, so a rule expression reads across lines until the next statement.
    auto startsNewRule = [&]() -> bool {
        if (c.Eof() || c.Cur().quoted || c.Cur().text.empty() || c.Cur().text[0] != '%')
            return false;
        const size_t eqAt = c.Cur().text.size() > 1 ? c.pos + 1 : c.pos + 2;
        return eqAt < c.toks.size() && !c.toks[eqAt].quoted && c.toks[eqAt].text == "=";
    };

    // `flexfile "file" { ... }` groups its flex lines in a nested brace, so track
    // depth: the model block ends only when the outer '}' brings it back to 0.
    int depth = 1;
    for (;;) {
        const Token* tp = c.Next();
        if (!tp)
            return c.Fail(cmd.line, where + " is missing '}'");
        if (!tp->quoted && tp->text == "{") { depth++; continue; }
        if (!tp->quoted && tp->text == "}") { if (--depth == 0) break; continue; }
        const Token t = *tp;
        const std::string o = Lower(t.text);

        // %<name> = <expr>  -> a global flex rule (same as $flexrule)
        if (!t.quoted && !t.text.empty() && t.text[0] == '%') {
            ManualFlex::Rule rule;
            rule.name = t.text == "%" ? "" : t.text.substr(1);
            if (rule.name.empty() && !c.Want("a morph name after '%'", t, rule.name))
                return false;
            std::string eq;
            if (!c.Want("'=' after the morph name", t, eq))
                return false;
            if (eq != "=")
                return c.Fail(t.line, where + ": flex rule expects '=' after \"" + rule.name + "\"");
            // multi-line: read until the next statement (new rule, option, '}', command)
            while (!c.Eof()) {
                const Token& cur = c.Cur();
                if (!cur.quoted &&
                    (cur.text == "}" || cur.text == "{" || cur.text[0] == '$' ||
                     startsNewRule() || isModelOption(Lower(cur.text))))
                    break;
                if (!cur.quoted && cur.text == "\\\\") {
                    ++c.pos;
                    continue;
                }
                if (!rule.expr.empty())
                    rule.expr += ' ';
                rule.expr += c.Next()->text;
            }
            if (rule.expr.empty())
                return c.Fail(t.line, where + ": flex rule \"" + rule.name + "\" has no expression");
            c.manual.rules.push_back(std::move(rule));
            continue;
        }

        // eyeball <name> <bone> <x y z> <material> <diameter> <angle> <iris> <pupil>
        if (o == "eyeball") {
            FaceMarkup::Entry e;
            e.kind = FaceMarkup::Kind::Eyeball;
            std::string iris;
            if (!c.Want("an eyeball name", t, e.eyeball.name) ||
                !c.Want("a bone name", t, e.eyeball.bonename) ||
                !c.WantFloat("an origin x", t, e.eyeball.origin.x) ||
                !c.WantFloat("an origin y", t, e.eyeball.origin.y) ||
                !c.WantFloat("an origin z", t, e.eyeball.origin.z) ||
                !c.Want("a material name", t, e.eyeball.material) ||
                !c.WantFloat("a diameter", t, e.eyeball.diameter) ||
                !c.WantFloat("an angle in degrees", t, e.eyeball.angle) ||
                !c.Want("an iris material", t, iris) || // read and discarded, like the reference
                !c.WantFloat("a pupil scale", t, e.eyeball.pupilscale))
                return false;
            c.face.entries.push_back(std::move(e));
            continue;
        }

        // flexcontroller <type> [range min max] <name>...  (same as $flexcontroller)
        if (o == "flexcontroller") {
            std::string group;
            if (!c.Want("a controller group", t, group))
                return false;
            float rMin = 0.0f, rMax = 1.0f;
            int names = 0;
            while (sameLine(t.line)) {
                const Token& n = *c.Next();
                if (!n.quoted && _stricmp(n.text.c_str(), "range") == 0) {
                    if (!c.WantFloat("a range min", t, rMin) ||
                        !c.WantFloat("a range max", t, rMax))
                        return false;
                    continue;
                }
                c.manual.controllers.push_back({n.text, group, rMin, rMax});
                names++;
            }
            if (names == 0)
                return c.Fail(t.line, where + ": flexcontroller \"" + group +
                                          "\" expects at least one controller name");
            continue;
        }

        // localvar <name>...  (same as $flexlocalvar)
        if (o == "localvar") {
            int names = 0;
            while (sameLine(t.line)) {
                ManualFlex::Rule rule;
                rule.name = c.Next()->text;
                rule.localvar = true;
                c.manual.rules.push_back(std::move(rule));
                names++;
            }
            if (names == 0)
                return c.Fail(t.line, where + ": localvar expects at least one name");
            continue;
        }

        // mouth <index> <controller> <bone> <forward x y z>  (index is implicit
        // in this compiler - declaration order - so it is read and ignored)
        if (o == "mouth") {
            int index;
            FaceMarkup::Entry e;
            e.kind = FaceMarkup::Kind::Mouth;
            if (!c.WantInt("a mouth index", t, index) ||
                !c.Want("a flex controller name", t, e.mouth.controller) ||
                !c.Want("a bone name", t, e.mouth.bonename) ||
                !c.WantFloat("a forward x", t, e.mouth.forward.x) ||
                !c.WantFloat("a forward y", t, e.mouth.forward.y) ||
                !c.WantFloat("a forward z", t, e.mouth.forward.z))
                return false;
            c.face.entries.push_back(std::move(e));
            continue;
        }

        // flexfile <file> - the VTA the following flex/flexpair/defaultflex read
        if (o == "flexfile") {
            if (!c.Want("a .vta filename", t, flexfile))
                return false;
            vtaLine = t.line;
            continue;
        }

        // flex/flexpair/defaultflex <name> [<split>] frame <N> [position f] [decay f]
        // Each names one VTA frame; a split routes to the stereo-split machinery.
        if (o == "flex" || o == "flexpair" || o == "defaultflex") {
            if (flexfile.empty())
                return c.Fail(t.line, where + ": " + o + " needs a `flexfile <file>` before it");
            source::VtaFlexOption fo;
            float split = 0.0f;
            bool hasSplit = false;
            if (o == "defaultflex") {
                fo.name = "default";
            } else if (!c.Want("a flex name", t, fo.name)) {
                return false;
            }
            if (o == "flexpair") {
                if (!c.WantFloat("a pair split", t, split))
                    return false;
                hasSplit = true;
            }
            bool sawFrame = false;
            while (sameLine(t.line)) {
                const std::string k = Lower(c.Cur().text);
                if (k == "frame") {
                    ++c.pos;
                    if (!c.WantInt("a frame index", t, fo.frame))
                        return false;
                    sawFrame = true;
                } else if (k == "position") {
                    ++c.pos;
                    if (!c.WantFloat("a position", t, fo.position))
                        return false;
                } else if (k == "decay") {
                    ++c.pos;
                    if (!c.WantFloat("a decay", t, fo.decay))
                        return false;
                } else if (k == "split") {
                    ++c.pos;
                    if (!c.WantFloat("a split", t, split))
                        return false;
                    hasSplit = true;
                } else {
                    break;
                }
            }
            if (!sawFrame)
                return c.Fail(t.line, where + ": " + o + " \"" + fo.name +
                                          "\" needs a frame <N> - a .vta names nothing");
            if (fo.frame == 0)
                continue; // frame 0 is the VTA basis (the rest pose), not a delta
            if (hasSplit && split != 0.0f)
                c.manual.stereoSplits.push_back({fo.name, split});
            vtaByFile[flexfile].push_back(std::move(fo));
            continue;
        }

        // vcafile <file> [controller] - a baked VTA sequence, the mesh's only flex
        if (o == "vcafile") {
            if (!vcaFile.empty())
                return c.Fail(t.line, where + ": vcafile written twice");
            if (!c.Want("a .vca filename", t, vcaFile))
                return false;
            vcaLine = t.line;
            if (sameLine(t.line))
                vcaName = c.Next()->text;
            if (vcaName.empty())
                vcaName = StripExtension(pulse::FilePath(vcaFile).filename().string());
            continue;
        }

        // dmxeyelid <upper|lower> <file> lowerer <d|-> <t> neutral <d|-> <t>
        //           raiser <d|-> <t> [split s] (righteyeball n lefteyeball n | eyeball n)
        // Deltas resolve by name against this model's morphs; the file is the
        // source they came from. Maps straight to the $eyelid registration.
        if (o == "dmxeyelid") {
            std::string type, dmxfile;
            if (!c.Want("\"upper\" or \"lower\"", t, type) ||
                !c.Want("a source filename", t, dmxfile)) // where the deltas came from
                return false;
            FaceMarkup::Entry e;
            e.kind = FaceMarkup::Kind::Eyelid;
            const char dtc = type.empty() ? '?' : static_cast<char>(std::tolower(type[0]));
            if (dtc == 'u')      e.eyelid.upper = true;
            else if (dtc == 'l') e.eyelid.upper = false;
            else return c.Fail(t.line, where + ": dmxeyelid type must be upper or lower");
            bool haveSlot[3] = {false, false, false};
            while (sameLine(t.line)) {
                const Token& k = *c.Next();
                const std::string ko = Lower(k.text);
                int slot = -1;
                for (int i = 0; i < 3; ++i)
                    if (ko == kLidSlot[i]) { slot = i; break; }
                if (slot >= 0) {
                    if (!c.Want("a delta name or \"-\"", t, e.eyelid.delta[slot]) ||
                        !c.WantFloat("a lid target", t, e.eyelid.target[slot]))
                        return false;
                    if (e.eyelid.delta[slot] == "-")
                        e.eyelid.delta[slot].clear();
                    e.eyelid.target[slot] *= c.in.scale;
                    haveSlot[slot] = true;
                } else if (ko == "split") {
                    if (!c.WantFloat("a split distance", t, e.eyelid.split)) return false;
                } else if (ko == "flexdesc") {
                    if (!c.Want("a flexdesc name", t, e.eyelid.basedesc)) return false;
                } else if (ko == "eyeball") {
                    if (!c.Want("an eyeball name", t, e.eyelid.eyeball)) return false;
                } else if (ko == "righteyeball") {
                    if (!c.Want("an eyeball name", t, e.eyelid.righteyeball)) return false;
                } else if (ko == "lefteyeball") {
                    if (!c.Want("an eyeball name", t, e.eyelid.lefteyeball)) return false;
                } else {
                    return c.Fail(k.line, where + ": dmxeyelid unknown option \"" + k.text + "\"");
                }
            }
            for (int i = 0; i < 3; ++i)
                if (!haveSlot[i])
                    return c.Fail(t.line, where + ": dmxeyelid " + type + " missing `" +
                                              kLidSlot[i] + " <delta> <target>`");
            const bool mono = !e.eyelid.eyeball.empty();
            if (!mono && (e.eyelid.righteyeball.empty() || e.eyelid.lefteyeball.empty()))
                return c.Fail(t.line, where +
                                          ": dmxeyelid needs `eyeball` or both "
                                          "`righteyeball`/`lefteyeball`");
            c.face.entries.push_back(std::move(e));
            continue;
        }

        // eyelid <upperN|lowerN> <vtafile> lowerer <frame|-> <t> neutral <frame|-> <t>
        //        raiser <frame|-> <t> [split s] eyeball <name>
        // The frame-addressed VTA form: type is decided by its first char, its full
        // string is the lid desc base. Each present frame loads as a named delta
        // (<type>_lid_<slot>) so it resolves by name like dmxeyelid.
        if (o == "eyelid") {
            std::string type, vtafile;
            if (!c.Want("\"upper\" or \"lower\"", t, type) ||
                !c.Want("a .vta filename", t, vtafile))
                return false;
            FaceMarkup::Entry e;
            e.kind = FaceMarkup::Kind::Eyelid;
            const char etc = type.empty() ? '?' : static_cast<char>(std::tolower(type[0]));
            if (etc == 'u')      e.eyelid.upper = true;
            else if (etc == 'l') e.eyelid.upper = false;
            else return c.Fail(t.line, where + ": eyelid type must start with upper or lower");
            int frame[3] = {-1, -1, -1};
            bool haveSlot[3] = {false, false, false};
            while (sameLine(t.line)) {
                const Token& k = *c.Next();
                const std::string ko = Lower(k.text);
                int slot = -1;
                for (int i = 0; i < 3; ++i)
                    if (ko == kLidSlot[i]) { slot = i; break; }
                if (slot >= 0) {
                    if (c.Cur().text == "-") {
                        ++c.pos;
                        frame[slot] = -1;
                    } else if (!c.WantInt("a frame index", t, frame[slot])) {
                        return false;
                    }
                    if (!c.WantFloat("a lid target", t, e.eyelid.target[slot]))
                        return false;
                    e.eyelid.target[slot] *= c.in.scale;
                    haveSlot[slot] = true;
                } else if (ko == "split") {
                    if (!c.WantFloat("a split distance", t, e.eyelid.split)) return false;
                } else if (ko == "eyeball") {
                    if (!c.Want("an eyeball name", t, e.eyelid.eyeball)) return false;
                } else {
                    return c.Fail(k.line, where + ": eyelid unknown option \"" + k.text + "\"");
                }
            }
            for (int i = 0; i < 3; ++i)
                if (!haveSlot[i])
                    return c.Fail(t.line, where + ": eyelid " + type + " missing `" +
                                              kLidSlot[i] + " <frame> <target>`");
            if (e.eyelid.eyeball.empty())
                return c.Fail(t.line, where + ": the VTA eyelid form needs `eyeball <name>`");
            e.eyelid.basedesc = type; // e.g. "upper_right" - the reference's lid desc base
            for (int i = 0; i < 3; ++i) {
                if (frame[i] <= 0)
                    continue; // "-" or frame 0 (the basis) = a pose with no vertex data
                source::VtaFlexOption fo;
                fo.name = e.eyelid.basedesc + "_lid_" + kLidSlot[i];
                fo.frame = frame[i];
                e.eyelid.delta[i] = fo.name;
                vtaByFile[vtafile].push_back(std::move(fo));
            }
            c.face.entries.push_back(std::move(e));
            continue;
        }

        if (o == "noautodmxrules")
            continue; // handled by the pre-scan above
        if (o == "attachment")
            continue; // no-op inside $model, as in the reference
        if (o == "spherenormals") {
            std::string mat;
            float x, y, z;
            if (!c.Want("a material name", t, mat) || !c.WantFloat("x", t, x) ||
                !c.WantFloat("y", t, y) || !c.WantFloat("z", t, z))
                return false; // consumed and ignored (matches importqc)
            continue;
        }

        return c.Fail(t.line, where + ": unknown model option \"" + t.text + "\"");
    }

    // Load the collected VTA / VCA morphs against the source (SMD-only). The
    // import scale must match the one the mesh was loaded with.
    if (!vcaFile.empty() && !vtaByFile.empty())
        return c.Fail(vcaLine, where + ": vcafile must be the only flex source");
    if ((!vtaByFile.empty() || !vcaFile.empty()) && !IsSmdPath(src->filename))
        return c.Fail(cmd.line, where + ": flexfile/flex/eyelid/vcafile are SMD-only - a DMX "
                                        "mesh carries its own delta states");
    for (auto& kv : vtaByFile) {
        std::vector<fs::path> tried;
        const fs::path full = FindSourceFile(c, kv.first, &tried);
        if (full.empty())
            return c.Fail(vtaLine, "cannot find \"" + kv.first + "\" - looked in:" + LookedIn(tried));
        std::string err;
        if (!source::LoadVtaMorphs(full.string(), *src, importScale, kv.second, &err))
            return c.Fail(vtaLine, "cannot load \"" + full.string() + "\": " + err);
    }
    if (!vcaFile.empty()) {
        std::vector<fs::path> tried;
        const fs::path full = FindSourceFile(c, vcaFile, &tried);
        if (full.empty())
            return c.Fail(vcaLine, "cannot find \"" + vcaFile + "\" - looked in:" + LookedIn(tried));
        std::string err;
        if (!source::LoadVcaMorphs(full.string(), *src, importScale, vcaName, &err))
            return c.Fail(vcaLine, "cannot load \"" + full.string() + "\": " + err);
    }
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
        // autosteps fans one rule into N; the single-rule bake path cannot
        // express that. Use ikrule autosteps for the runtime rule instead.
        if (one[0].autosteps) {
            c.Fail(t.line, "ikfixup: autosteps is not supported here - use ikrule autosteps");
            return -1;
        }
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::IkFixup;
        cmd.ikfixup = std::move(one[0]);
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // localhierarchy <bone> <parent> [range <start> <peak> <tail> <end>]:
    // reparent <bone> to <parent> ("" = worldspace) over that frame range
    if (_stricmp(o.c_str(), "localhierarchy") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::LocalHierarchy;
        if (!c.Want("a bone name", t, cmd.alignBone) ||
            !c.Want("a parent bone name", t, cmd.name))
            return -1;
        cmd.driverStart = cmd.driverPeak = cmd.driverTail = cmd.driverEnd = -1;
        if (!c.AtCommand() && !c.Cur().quoted &&
            _stricmp(c.Cur().text.c_str(), "range") == 0) {
            c.Next();
            if (!c.WantFrame("a start frame", t, cmd.driverStart) ||
                !c.WantFrame("a peak frame", t, cmd.driverPeak) ||
                !c.WantFrame("a tail frame", t, cmd.driverTail) ||
                !c.WantFrame("an end frame", t, cmd.driverEnd))
                return -1;
        }
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // derivative <scale>: replace each frame with its scaled delta from the
    // previous one.
    if (_stricmp(o.c_str(), "derivative") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::Derivative;
        if (!c.WantFloat("a derivative scale", t, cmd.derivativeScale))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // noanimation collapses to one zeroed delta frame; noanim_keepduration sets
    // the same delta flags but keeps the frame count and bone data.
    if (_stricmp(o.c_str(), "noanimation") == 0 ||
        _stricmp(o.c_str(), "noanim_keepduration") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::NoAnim;
        cmd.keepDuration = _stricmp(o.c_str(), "noanim_keepduration") == 0;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // lineardelta subtracts a straight frame0->last baseline; splinedelta eases
    // that baseline with 3s^2-2s^3.
    if (_stricmp(o.c_str(), "lineardelta") == 0 ||
        _stricmp(o.c_str(), "splinedelta") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::LinearDelta;
        cmd.splineDelta = _stricmp(o.c_str(), "splinedelta") == 0;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // compress <frames>: downsample, keeping frame 0 then every Nth frame.
    if (_stricmp(o.c_str(), "compress") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::Compress;
        if (!c.WantInt("a frame skip", t, cmd.compressFrames))
            return -1;
        if (cmd.compressFrames < 1) {
            c.Fail(t.line, "compress frames must be >= 1");
            return -1;
        }
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // counterrotate <bone>: strip the bone's world rotation (target from the
    // default pose). counterrotateto <p> <y> <r> <bone>: explicit target.
    if (_stricmp(o.c_str(), "counterrotate") == 0 ||
        _stricmp(o.c_str(), "counterrotateto") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::CounterRotate;
        if (_stricmp(o.c_str(), "counterrotateto") == 0) {
            cmd.counterHasTarget = true;
            if (!c.WantFloat("a pitch", t, cmd.counterAngles.x) ||
                !c.WantFloat("a yaw", t, cmd.counterAngles.y) ||
                !c.WantFloat("a roll", t, cmd.counterAngles.z))
                return -1;
        }
        if (!c.Want("a bone name", t, cmd.alignBone))
            return -1;
        a.cmds.push_back(std::move(cmd));
        return 1;
    }
    // forceboneposrot <bone> [pos x y z] [rot x y z [local]]: overwrite a bone's
    // local pos and/or rotation every frame. rot is world-space unless `local`
    // is given or the bone is rootless.
    if (_stricmp(o.c_str(), "forceboneposrot") == 0) {
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = cm::CompileInput::InAnim::InCmd::ForceBonePosRot;
        if (!c.Want("a bone name", t, cmd.alignBone))
            return -1;
        if (!c.AtCommand() && !c.Cur().quoted &&
            _stricmp(c.Cur().text.c_str(), "pos") == 0) {
            c.Next();
            cmd.forceDoPos = true;
            if (!c.WantFloat("pos x", t, cmd.forcePos.x) ||
                !c.WantFloat("pos y", t, cmd.forcePos.y) ||
                !c.WantFloat("pos z", t, cmd.forcePos.z))
                return -1;
        }
        if (!c.AtCommand() && !c.Cur().quoted &&
            _stricmp(c.Cur().text.c_str(), "rot") == 0) {
            c.Next();
            cmd.forceDoRot = true;
            if (!c.WantFloat("rot x", t, cmd.forceRot.x) ||
                !c.WantFloat("rot y", t, cmd.forceRot.y) ||
                !c.WantFloat("rot z", t, cmd.forceRot.z))
                return -1;
            if (!c.AtCommand() && !c.Cur().quoted &&
                _stricmp(c.Cur().text.c_str(), "local") == 0) {
                c.Next();
                cmd.forceRotLocal = true;
            }
        }
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
    // walkframe <frame> <controls...>
    // walkalignto <frame> <anim> <controls...>
    // walkalign <frame> <anim> <controls...> <refframe> <srcframe>
    // Extract the root bone's travel up to <frame> into a movement key.
    if (_stricmp(o.c_str(), "walkframe") == 0 ||
        _stricmp(o.c_str(), "walkalignto") == 0 ||
        _stricmp(o.c_str(), "walkalign") == 0) {
        const bool ref = (_stricmp(o.c_str(), "walkframe") != 0);
        const bool explicitFrames = (_stricmp(o.c_str(), "walkalign") == 0);
        cm::CompileInput::InAnim::InCmd cmd;
        cmd.kind = ref ? cm::CompileInput::InAnim::InCmd::RefMotion
                       : cm::CompileInput::InAnim::InCmd::Motion;
        if (!c.WantInt("an end frame", t, cmd.motionEndFrame))
            return -1;
        cmd.srcframe = cmd.motionEndFrame;
        if (ref) {
            if (!c.Want("an animation name", t, cmd.name))
                return -1;
        }
        for (;;) {
            if (c.AtCommand() || c.Eof() || c.Cur().quoted || c.Cur().line != t.line)
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
        if (explicitFrames &&
            (!c.WantInt("a reference frame", t, cmd.destframe) ||
             !c.WantInt("a source frame", t, cmd.srcframe)))
            return -1;
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
        c.SpliceAt(c.pos, it->second);
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
    const bool savedRaw = c.rawCollect;
    c.rawCollect = true;
    bool braced = false;
    if (!c.Eof() && !c.Cur().quoted && c.Cur().text == "{") {
        braced = true;
        c.pos++;
    }
    for (;;) {
        if (braced) {
            if (c.Eof()) {
                c.rawCollect = savedRaw;
                return c.Fail(cmd.line, "$cmdlist \"" + name + "\" is missing '}'");
            }
            if (!c.Cur().quoted && c.Cur().text == "}") { c.pos++; break; }
        } else if (c.AtCommand()) {
            break;
        }
        const Token t = c.toks[c.pos++];
        if (!t.quoted && _stricmp(t.text.c_str(), "cmdlist") == 0) {
            c.rawCollect = savedRaw;
            return c.Fail(t.line, "$cmdlist \"" + name + "\" cannot nest a cmdlist");
        }
        body.push_back(t);
    }
    c.rawCollect = savedRaw;
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
    a.motionrollback = c.defaultMotionRollback;
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
    } else if (_stricmp(type.c_str(), "unlatch") == 0) {
        rule.type = "unlatch";
    } else if (_stricmp(type.c_str(), "autosteps") == 0) {
        rule.type = "footstep";
        rule.autosteps = true;
        if (!c.WantInt("a step count", cmd, rule.autostepsCount) ||
            !c.Want("a foot bone name", cmd, rule.autostepsBone))
            return false;
    } else {
        return c.Fail(cmd.line, "ikrule type \"" + type + "\": not yet supported "
                                "(expected touch/footstep/attachment/release/unlatch/autosteps)");
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
        } else if (_stricmp(o.c_str(), "target") == 0) {
            c.pos++;
            if (!c.WantInt("a target slot", cmd, rule.slot)) return false;
            rule.slotSet = true;
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
bool ParseKeyValues(Ctx& c, const Token& cmd, std::string& out,
                    bool normalizeModelPaths = false) {
    const Token* brace = c.Next();
    if (!brace || brace->quoted || brace->text != "{")
        return c.Fail(cmd.line, cmd.text + " expects '{'");

    int level = 1;
    std::vector<bool> expectKey(2, true);
    std::vector<bool> modelKey(2, false);
    for (;;) {
        if (out.size() > static_cast<size_t>(lim::kMaxKeyValuesBytes))
            return c.Fail(cmd.line, cmd.text + ": keyvalue block exceeds " +
                                    std::to_string(lim::kMaxKeyValuesBytes) + " bytes");
        const Token* t = c.Next();
        if (!t)
            return c.Fail(cmd.line, cmd.text + ": keyvalue block missing matching braces");
        if (!t->quoted && t->text == "}") {
            if (--level <= 0)
                break;
            out += " }\n";
            expectKey.resize(static_cast<size_t>(level) + 1);
            modelKey.resize(static_cast<size_t>(level) + 1);
            expectKey[level] = true;
            modelKey[level] = false;
        } else if (!t->quoted && t->text == "{") {
            out += "{\n";
            expectKey[level] = true;
            modelKey[level] = false;
            level++;
            expectKey.resize(static_cast<size_t>(level) + 1, true);
            modelKey.resize(static_cast<size_t>(level) + 1, false);
        } else if (level > 1) {
            std::string value = t->text;
            if (expectKey[level]) {
                modelKey[level] = Lower(value) == "model";
                expectKey[level] = false;
            } else {
                if (normalizeModelPaths && modelKey[level])
                    std::replace(value.begin(), value.end(), '\\', '/');
                expectKey[level] = true;
                modelKey[level] = false;
            }
            out += "\"" + value + "\" ";
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

bool CmdCollisionText(Ctx& c, const Token& cmd) {
    return ParseKeyValues(c, cmd, c.in.physCollisionText, true);
}

// ---------------------------------------------------------------------------
// Flex / morph commands are global. Only meshes used by bodygroups contribute.
// Ctx::manual is registered after parsing, in finished bodygroup order.
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

    // Accept the studiomdl `%morph = rule` spelling; the '%' may lex separately.
    if (name == "%") {
        if (!c.Want("a morph name after '%'", cmd, name))
            return false;
    } else if (name.size() > 1 && name[0] == '%') {
        name.erase(0, 1);
    }

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
        if (!c.Cur().quoted && c.Cur().text == "\\\\") {
            ++c.pos;
            continue;
        }
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
// Face markup is global. A shared list preserves command order: $eyelid names
// $eyeball indices, and $mouth/$eyelid append to the global flexdesc table.
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
//         neutral <delta> <target> raiser <delta> <target> [split <distance>]
//         (righteyeball <name> lefteyeball <name> | eyeball <name>)
//
// `split` masks the deltas to one side of the midline, so a delta that moves
// both lids can serve one eye per line; the sign picks the side and the
// magnitude is the blend band. Omitted = the whole delta, which is what
// per-side authored DMX deltas want.
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
        } else if (o == "split") {
            if (!c.WantFloat("a split distance", cmd, entry.eyelid.split)) return false;
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
                                  "\" (expected lowerer/neutral/raiser/split/flexdesc/eyeball/"
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

// $renamemorph <morph> <newname> - rename a source delta state. Held until the
// script is parsed (ApplyMorphRenames), so it may sit anywhere: every morph
// reference in the script is rewritten with it, before or after.
bool CmdRenameMorph(Ctx& c, const Token& cmd) {
    Ctx::MorphRename rn;
    rn.line = cmd.line;
    if (!c.Want("a morph name", cmd, rn.from) || !c.Want("a new morph name", cmd, rn.to))
        return false;
    if (_stricmp(rn.from.c_str(), rn.to.c_str()) != 0)
        c.morphRenames.push_back(std::move(rn));
    return true;
}

// Rewrite %name -> %newname in a flex rule expression. A bare name there is a
// controller, never a morph.
std::string RenameExprMorph(const std::string& expr, const std::string& from,
                            const std::string& to) {
    std::string out;
    for (size_t i = 0; i < expr.size();) {
        if (expr[i] != '%') {
            out.push_back(expr[i++]);
            continue;
        }
        size_t j = i + 1;
        while (j < expr.size() && static_cast<unsigned char>(expr[j]) <= 32)
            ++j; // the rule lexer allows "% name"
        const size_t start = j;
        while (j < expr.size() &&
               (isalnum(static_cast<unsigned char>(expr[j])) || expr[j] == '_'))
            ++j;
        const std::string name = expr.substr(start, j - start);
        out += '%' + (_stricmp(name.c_str(), from.c_str()) == 0 ? to : name);
        i = j;
    }
    return out;
}

// $renamemorph: the delta states plus every script reference to them, in one
// pass after parsing. Renames apply in script order, so chains work.
bool ApplyMorphRenames(Ctx& c) {
    for (const Ctx::MorphRename& rn : c.morphRenames) {
        auto swapName = [&](std::string& s) {
            if (_stricmp(s.c_str(), rn.from.c_str()) == 0)
                s = rn.to;
        };
        // operand form: %name is a morph, a bare name is a controller
        auto swapOperand = [&](std::string& s) {
            if (!s.empty() && s[0] == '%' && _stricmp(s.c_str() + 1, rn.from.c_str()) == 0)
                s = '%' + rn.to;
        };

        // The rename is global, so the target must be free everywhere - merging
        // into a name another mesh already uses is never what was meant.
        for (const auto& src : c.in.sources)
            for (const source::SrcMorphAnim& m : src->morphs)
                if (_stricmp(m.name.c_str(), rn.to.c_str()) == 0)
                    return c.Fail(rn.line, "$renamemorph \"" + rn.from + "\" -> \"" + rn.to +
                                               "\": " + src->filename +
                                               " already has a morph \"" + rn.to + "\"");

        bool hit = false;
        for (auto& src : c.in.sources) {
            for (source::SrcMorphAnim& m : src->morphs)
                if (_stricmp(m.name.c_str(), rn.from.c_str()) == 0) {
                    m.name = rn.to;
                    hit = true;
                }
            for (source::SrcFlexKey& k : src->flexkeys)
                if (_stricmp(k.name.c_str(), rn.from.c_str()) == 0) {
                    k.name = rn.to;
                    hit = true;
                }
            for (source::SrcFlexRule& r : src->dmeFlexRules) {
                swapName(r.name);
                r.expr = RenameExprMorph(r.expr, rn.from, rn.to);
            }
        }
        if (!hit)
            return c.Fail(rn.line, "$renamemorph names \"" + rn.from +
                                       "\", which is not a morph in any $rendermesh");

        for (ManualFlex::Rule& r : c.manual.rules) {
            swapName(r.name);
            for (std::string& op : r.combo)
                swapOperand(op);
            r.expr = RenameExprMorph(r.expr, rn.from, rn.to);
        }
        for (ManualFlex::Domination& d : c.manual.dominations) {
            swapName(d.name);
            for (std::string& op : d.dominators)
                swapOperand(op);
        }
        for (ManualFlex::StereoSplit& s : c.manual.stereoSplits)
            swapName(s.name);
        for (source::FlexRig::Corrective& cor : c.manual.datamodel.correctives)
            swapName(cor.delta);
        for (source::SrcFlexRule& r : c.manual.datamodel.rules) {
            swapName(r.name);
            r.expr = RenameExprMorph(r.expr, rn.from, rn.to);
        }
        for (cm::CompileInput::FixedFlex& fx : c.in.fixedFlexes)
            swapName(fx.name);
        for (Ctx::PendingAttachment& a : c.attachments)
            for (std::string& m : a.flexmorphs)
                swapName(m);
        for (FaceMarkup::Entry& e : c.face.entries)
            if (e.kind == FaceMarkup::Kind::Eyelid)
                for (std::string& d : e.eyelid.delta)
                    swapName(d);
        for (source::WrinkleScaleOption& w : c.wrinkleScales)
            swapName(w.shape);
    }
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

        // a sequence-body `ignorescale` fans out to the whole blend grid plus
        // blendref/blendcomp/blendcenter at compile; flag it here (it still
        // lands on blend anim 0 below like any option).
        if (!t.quoted && _stricmp(t.text.c_str(), "ignorescale") == 0)
            seq.ignorescale = true;

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
            a.motionrollback = c.defaultMotionRollback;
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

    for (const auto& s : c.in.sequences)
        if (_stricmp(s.name.c_str(), seq.name.c_str()) == 0)
            return c.Fail(cmd.line, "duplicate sequence name \"" + seq.name + "\"");

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
        a.motionrollback = c.defaultMotionRollback;
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
        dst.motiontype = src0.motiontype;
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

// Legacy bare flags replaced by $modelarchetype; last one wins.
bool CmdStaticProp(Ctx& c, const Token&) { c.in.archetype = cm::Archetype::Static; return true; }
bool CmdSimpleProp(Ctx& c, const Token&) { c.in.archetype = cm::Archetype::Simple; return true; }
bool CmdAutoCenter(Ctx& c, const Token&) { c.in.autoCenter = true; return true; }

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

// $modelbudget { bones <n> materials <n> } - set the compile ceilings for this
// model. Braces are optional for a single field. The pulselimits.h value is the
// cap; bones default to 255 and must be raised here to go past it.
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

// $renderpass <name> selects the model's render pass.
// "none" makes the no-flag default authorable.
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

// Legacy bare flags replaced by $renderpass; last one wins.
bool CmdOpaque(Ctx& c, const Token&) { c.in.renderPass = 1; return true; }
bool CmdMostlyOpaque(Ctx& c, const Token&) { c.in.renderPass = 2; return true; }

// $setbindpose <file> <frame> [meshonly] - bake the rest mesh and skeleton.
// The pose file contributes no geometry or materials.
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
    if (!c.Eof() && c.Cur().line == cmd.line &&
        _stricmp(c.Cur().text.c_str(), "meshonly") == 0) {
        c.in.bindPoseMeshOnly = true;
        ++c.pos;
    }
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

// $jointsurfaceprop <bone> <surfaceprop> - the property for one bone and, via
// the parent walk in ApplyJointSurfaceProps, its descendants. Replaces any
// existing entry for the same bone.
bool CmdJointSurfaceProp(Ctx& c, const Token& cmd) {
    std::string bone, prop;
    if (!c.Want("a bone name", cmd, bone))
        return false;
    if (!c.Want("a surface property name", cmd, prop))
        return false;
    for (auto& js : c.in.jointSurfaceProps) {
        if (_stricmp(js.first.c_str(), bone.c_str()) == 0) {
            js.second = prop;
            return true;
        }
    }
    c.in.jointSurfaceProps.emplace_back(bone, prop);
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

// $origin <x> <y> <z> [z rotation]  (legacy Cmd_Origin)
// Old-QC alias writing the same slots as $transformmodel origin/angles; last of
// the two in the script wins. The optional 4th value is a yaw in degrees,
// composed with the built-in +90 like $transformmodel angles.
bool CmdOrigin(Ctx& c, const Token& cmd) {
    float x = 0, y = 0, z = 0;
    if (!c.WantFloat("an X offset", cmd, x) || !c.WantFloat("a Y offset", cmd, y) ||
        !c.WantFloat("a Z offset", cmd, z))
        return false;
    c.in.adjust = {x, y, z};
    if (!c.AtCommand()) {
        float zrot = 0;
        if (!c.WantFloat("a Z rotation", cmd, zrot))
            return false;
        c.in.rotation = {0, 0, (zrot + 90.0f) * pm::kDeg2Rad};
        c.in.rotationSet = true;
    }
    return true;
}

// $scale <float>  (legacy Cmd_ScaleUp)
// Old-QC alias for $transformmodel scale. Scale is baked as each source loads,
// so sources placed before this keep the previous scale (legacy g_currentscale
// per-file behavior) - warn, but do not stop; last write wins for later ones.
bool CmdScale(Ctx& c, const Token& cmd) {
    if (!c.WantFloat("a scale", cmd, c.in.scale))
        return false;
    if (!c.in.sources.empty())
        std::fprintf(stderr,
                     "warning: %s line %d: $scale after a source - sources loaded "
                     "before it keep the previous scale\n",
                     c.file.c_str(), cmd.line);
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
            // reference reads up to 3 values, breaking early; omitted
            // components keep their 0 default (anglesDeg{}).
            for (float* d : {&a.anglesDeg.x, &a.anglesDeg.y, &a.anglesDeg.z}) {
                if (c.AtCommand())
                    break;
                const std::string& s = c.Cur().text;
                try {
                    size_t used = 0;
                    const float v = std::stof(s, &used);
                    if (used != s.size())
                        break;
                    *d = v;
                    c.pos++;
                } catch (const std::exception&) {
                    break;
                }
            }
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

// $limitrotation <bone> [sequence names...]: the reference accepts the
// sequence names but calculates the alignment from every eligible animation.
bool CmdLimitRotation(Ctx& c, const Token& cmd) {
    std::string bone;
    if (!c.Want("a bone name", cmd, bone))
        return false;
    c.in.limitRotationBones.push_back(std::move(bone));
    while (!c.Eof() && c.Cur().line == cmd.line)
        c.pos++;
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

// $root <bone> (Cmd_Root): the bone motion extraction, $alignto and $angle
// work from. Resolved after the bone table exists; unknown falls back to bone 0.
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

// One or more bone names on the command line - the tagging commands all take a
// list so the same command need not be repeated per bone.
bool WantBoneNames(Ctx& c, const Token& cmd, std::vector<std::string>& names) {
    std::string first;
    if (!c.Want("a bone name", cmd, first))
        return false;
    names.push_back(std::move(first));
    while (!c.AtCommand())
        names.push_back(c.toks[c.pos++].text);
    return true;
}

// $bonemerge <bone>... (Cmd_BoneMerge): tag BONE_USED_BY_BONE_MERGE so the bone
// survives the cull and a bonemerged child model can find it.
bool CmdBoneMerge(Ctx& c, const Token& cmd) {
    std::vector<std::string> names;
    if (!WantBoneNames(c, cmd, names))
        return false;
    for (const std::string& n : names)
        FindOrAddMarkup(c, n).isBonemerge = true;
    return true;
}

// $donotcollapse <bone>... (Cmd_DoNotCollapse): force-keep the bones. Beats
// $alwayscollapse.
bool CmdDoNotCollapse(Ctx& c, const Token& cmd) {
    std::vector<std::string> names;
    if (!WantBoneNames(c, cmd, names))
        return false;
    for (const std::string& n : names)
        FindOrAddMarkup(c, n).doNotCollapse = true;
    return true;
}

// $renamebone <bone> <new name>: rename a bone in the finished output. Applied
// after every other pass, so the rest of the script keeps naming the source bone.
bool CmdRenameBone(Ctx& c, const Token& cmd) {
    std::string from, to;
    if (!c.Want("a bone name", cmd, from) ||
        !c.Want("a replacement bone name", cmd, to))
        return false;
    c.in.boneRenames.emplace_back(std::move(from), std::move(to));
    return true;
}

// $alwayscollapse <bone>... (Cmd_AlwaysCollapse): force-collapse the bones even
// when something would normally keep them.
bool CmdAlwaysCollapse(Ctx& c, const Token& cmd) {
    std::vector<std::string> names;
    if (!WantBoneNames(c, cmd, names))
        return false;
    for (std::string& n : names)
        c.in.alwaysCollapse.push_back(std::move(n));
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

// Only for values headed back through DegToRad - a sampled pose trigger.
float RadToDeg(float rad) {
    return static_cast<float>(rad * 180.0 / 3.14159265358979323846);
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
                const pm::Vector3& pos, bool absolutePose) {
    if (tolDeg <= 0.0f)
        return c.Fail(cmd.line, "trigger tolerance must be > 0");

    cm::ProceduralBoneTrigger tr;
    tr.absolutePose = absolutePose;
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

// $driverbone <helper> <driver> [relative|absolute] [poseanim <file>] {
//     basepos <x y z>                                        optional, default 0
//     trigger <tolerance> <driver rot x y z> <helper rot x y z> <helper pos x y z>
//     ...
// }
// The inline replacement for a VRD quatinterp helper. Both rotations are
// RadianEuler degrees (x=roll, y=pitch, z=yaw), NOT a QAngle - they go into
// AngleQuaternion unpermuted, exactly like the <trigger> line they mirror.
//
// `absolute` (the default, and the VRD's own convention) reads the helper pose
// as parent-relative in full: an all-zero trigger puts the bone AT its parent's
// origin and basepos has to carry the bind pose.
// `relative` reads it as a DELTA from the bind pose, which MapProceduralBones
// folds in once the skeleton is final - an all-zero trigger leaves the bone at
// rest and basepos is a plain shared offset. Governs `trigger` lines only.
//
// Both rotations and the position can also be sampled out of an animation named
// by `poseanim <file>`:
//     posetrigger <tolerance> <frame>
// reads the driver's and the helper's local pose from that frame. A frame is
// parent-relative in full, so a `posetrigger` is always absolute no matter the
// block mode - one block can mix relative `trigger`s with sampled ones.
bool CmdDriverBone(Ctx& c, const Token& cmd) {
    cm::ProceduralBone pb;
    if (!c.Want("a helper bone name", cmd, pb.helpername) ||
        !c.Want("a driver bone name", cmd, pb.drivername))
        return false;

    std::string poseFile;
    int poseLine = cmd.line;
    bool blockAbsolute = true; // governs `trigger` lines only

    // anything between the names and the '{' has to be the mode or `poseanim
    // <file>` - a typo would otherwise surface as a confusing "missing '{'"
    while (!c.AtCommand() && !c.Cur().quoted && c.Cur().text != "{") {
        const Token t = c.toks[c.pos++];
        const std::string o = Lower(t.text);
        if (o == "poseanim") {
            const Token sub{"$driverbone poseanim", t.line, false};
            if (!c.Want("an animation file", sub, poseFile))
                return false;
            poseLine = t.line;
            continue;
        }
        if (o != "relative" && o != "absolute")
            return c.Fail(t.line, "$driverbone: expected relative, absolute, poseanim "
                                  "or '{', got \"" + t.text + "\"");
        blockAbsolute = o == "absolute";
    }
    if (!WantOpenBrace(c, cmd, "$driverbone"))
        return false;

    // held raw until the block closes, so basepos is order-independent
    struct Raw {
        Token at;
        float tol = 0.0f;
        pm::Vector3 driverRot, helperRot, pos;
        int frame = -1; // >= 0: sample the pose file instead
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

        if (o == "posetrigger") {
            const Token sub{"$driverbone posetrigger", t.line, false};
            Raw r{sub};
            if (!c.WantFloat("a tolerance in degrees", sub, r.tol) ||
                !c.WantInt("a frame", sub, r.frame))
                return false;
            if (r.frame < 0)
                return c.Fail(t.line, "$driverbone posetrigger: frame must be >= 0");
            raw.push_back(std::move(r));
            continue;
        }
        if (o == "basepos") {
            const Token sub{"$driverbone basepos", t.line, false};
            if (!c.WantFloat("an X offset", sub, basepos.x) ||
                !c.WantFloat("a Y offset", sub, basepos.y) ||
                !c.WantFloat("a Z offset", sub, basepos.z))
                return false;
            continue;
        }
        if (o != "trigger")
            return c.Fail(t.line, "$driverbone: expected trigger, posetrigger, "
                                  "basepos or '}', got \"" + t.text + "\"");

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

    const bool sampled = std::any_of(raw.begin(), raw.end(),
                                     [](const Raw& r) { return r.frame >= 0; });
    if (sampled) {
        if (poseFile.empty())
            return c.Fail(cmd.line, "$driverbone \"" + pb.helpername +
                                    "\": posetrigger needs a `poseanim <file>` on the command");
        source::Source* ps = LoadSource(c, WithSourceExtension(c, poseFile), poseLine, false,
                                        nullptr, source::LoadKind::Animation);
        if (!ps)
            return false;
        // the named clip carries the frames; "BindPose" is the one-frame fallback
        source::SourceAnim* anim = nullptr;
        for (source::SourceAnim& sa : ps->anims)
            if (_stricmp(sa.name.c_str(), "BindPose") != 0) { anim = &sa; break; }
        if (!anim && !ps->anims.empty())
            anim = &ps->anims.front();
        if (!anim || anim->frames.empty())
            return c.Fail(poseLine, "$driverbone poseanim \"" + poseFile + "\": no animation frames");

        int driver = -1, helper = -1;
        for (size_t i = 0; i < ps->localBone.size(); ++i) {
            if (_stricmp(ps->localBone[i].name.c_str(), pb.drivername.c_str()) == 0)
                driver = static_cast<int>(i);
            if (_stricmp(ps->localBone[i].name.c_str(), pb.helpername.c_str()) == 0)
                helper = static_cast<int>(i);
        }
        if (driver < 0)
            return c.Fail(poseLine, "$driverbone poseanim \"" + poseFile +
                                    "\": no driver bone \"" + pb.drivername + "\"");
        if (helper < 0)
            return c.Fail(poseLine, "$driverbone poseanim \"" + poseFile +
                                    "\": no helper bone \"" + pb.helpername + "\"");

        // the loader pre-scales positions and AddTrigger scales again, so the
        // sampled offset goes back to authored units here
        const float unscale = c.in.scale != 0.0f ? 1.0f / c.in.scale : 1.0f;
        for (Raw& r : raw) {
            if (r.frame < 0)
                continue;
            if (r.frame >= static_cast<int>(anim->frames.size()))
                return c.Fail(r.at.line, "$driverbone posetrigger: \"" + poseFile + "\" has " +
                                         std::to_string(anim->frames.size()) + " frames");
            const auto& f = anim->frames[r.frame];
            if (static_cast<int>(f.size()) <= std::max(driver, helper))
                return c.Fail(r.at.line, "$driverbone posetrigger: frame " +
                                         std::to_string(r.frame) + " of \"" + poseFile +
                                         "\" does not pose every bone");
            const source::SrcBonePose& d = f[driver];
            const source::SrcBonePose& h = f[helper];
            r.driverRot = {RadToDeg(d.rot.x), RadToDeg(d.rot.y), RadToDeg(d.rot.z)};
            r.helperRot = {RadToDeg(h.rot.x), RadToDeg(h.rot.y), RadToDeg(h.rot.z)};
            r.pos = {h.pos.x * unscale, h.pos.y * unscale, h.pos.z * unscale};
        }
    }

    // emitted after the block so basepos applies wherever it was written. Added
    // before the scale, like the VRD's `(basepos + pos) * g_currentscale`.
    for (const Raw& r : raw) {
        const pm::Vector3 pos{basepos.x + r.pos.x, basepos.y + r.pos.y,
                              basepos.z + r.pos.z};
        // posetrigger samples a full parent-relative frame, so it is absolute
        // regardless of the block mode
        if (!AddTrigger(c, r.at, pb, r.tol, r.driverRot, r.helperRot, pos,
                        blockAbsolute || r.frame >= 0))
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
// Bone names here are non-strict (ProceduralBone::strictName): a name may drop
// the skeleton's dotted namespace, so "Bip01_R_Thigh" hits "ValveBiped.Bip01_R_Thigh".
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
    if (_stricmp(pulse::FilePath(filename).extension().string().c_str(), ".vrd") != 0)
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
            cur->strictName = false;
        } else if (kw == "<aimconstraint>") {
            if (tok.size() < 4)
                return c.Fail(cmd.line, at.text + ": <aimconstraint> expects "
                                        "<bone> <parent> <aimname>");
            cm::AimAtBone ab;
            ab.bonename = tok[1];
            ab.parentname = tok[2];
            ab.aimname = tok[3];
            ab.strictName = false;
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
            if (!AddTrigger(c, at, *cur, tol, driverRot, helperRot, pos, true))
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

// $noforcedfade: model never fades out from level/fallback distance settings.
bool CmdNoForcedFade(Ctx& c, const Token&) {
    c.in.noForcedFade = true;
    return true;
}

// $casttextureshadows: static prop casts alpha-channel texture shadows in VRAD.
bool CmdCastTextureShadows(Ctx& c, const Token&) {
    c.in.castTextureShadows = true;
    return true;
}

// $constantdirectionallight <scale>: light dot stored as a 0-255 byte.
bool CmdConstantDirectionalLight(Ctx& c, const Token& cmd) {
    float scale = 0.0f;
    if (!c.WantFloat("a scale", cmd, scale))
        return false;
    c.in.constDirLight = true;
    c.in.constDirLightDot = static_cast<uint8_t>(scale * 255.0f);
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

// $bbox accepts six raw coordinates or declared render mesh names.
// Mesh bounds are calculated in model space during compilation.
bool CmdBBox(Ctx& c, const Token& cmd) {
    c.in.bboxMeshes.clear();
    if (!c.AtCommand()) {
        char* end = nullptr;
        const char* first = c.Cur().text.c_str();
        std::strtof(first, &end);
        if ((c.Cur().quoted && c.rendermeshes.count(c.Cur().text)) ||
            end == first || *end != '\0') {
            while (!c.AtCommand()) {
                const Token mesh = c.toks[c.pos++];
                auto it = c.rendermeshes.find(mesh.text);
                if (it == c.rendermeshes.end())
                    return c.Fail(mesh.line, "$bbox references unknown rendermesh \"" + mesh.text + "\"");
                if (it->second->vertex.empty())
                    return c.Fail(mesh.line, "$bbox references empty rendermesh \"" + mesh.text + "\"");
                c.in.bboxMeshes.push_back(it->second);
            }
            c.in.bboxSet = false;
            return true;
        }
    }
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

// $illumposition <x> <y> <z> [<bone>] (Cmd_Illumposition): the point the
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
    if (!c.AtCommand())
        bone = c.toks[c.pos++].text;
    if (!c.AtCommand())
        return c.Fail(cmd.line, "$illumposition: unexpected \"" + c.Cur().text + "\"");

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

// $eyeposition <x> <y> <z> [autoheight] (Cmd_Eyeposition): the ideal eye point,
// used by the engine to aim eyes and by tools as the view origin. Same source ->
// model swizzle as the static $illumposition, but this one IS scaled by $scale,
// which the compile stage applies. `autoheight` derives Z from the eyeballs
// (rounded mean of their |z|) and treats the authored Z as an offset from it.
bool CmdEyePosition(Ctx& c, const Token& cmd) {
    pm::Vector3 pos;
    if (!c.WantFloat("an X position", cmd, pos.x) ||
        !c.WantFloat("a Y position", cmd, pos.y) ||
        !c.WantFloat("a Z position", cmd, pos.z))
        return false;
    c.in.eyeposition = {-pos.y, pos.x, pos.z};
    if (!c.AtCommand() && !c.Cur().quoted &&
        _stricmp(c.Cur().text.c_str(), "autoheight") == 0) {
        c.pos++;
        c.in.eyepositionAutoHeight = true;
    }
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

// $renamematerial <from> <to> - rewrite one loaded texture's name, so it must
// come after the $body/$model that loaded it. Exact (case-insensitive) match
// first, then a substring fallback, matching the reference.
bool CmdRenameMaterial(Ctx& c, const Token& cmd) {
    std::string from, to;
    if (!c.Want("a material name", cmd, from) ||
        !c.Want("a replacement material", cmd, to))
        return false;

    to = StripExtension(to); // names are stored extension-stripped
    for (auto& tex : c.in.mats.textures) {
        if (_stricmp(tex.name.c_str(), from.c_str()) == 0) {
            tex.name = to;
            return true;
        }
    }
    for (auto& tex : c.in.mats.textures) {
        if (Lower(tex.name).find(Lower(from)) != std::string::npos) {
            std::printf("$renamematerial fell back to partial match: replacing %s with %s.\n",
                        tex.name.c_str(), to.c_str());
            tex.name = to;
            return true;
        }
    }
    return c.Fail(cmd.line, "$renamematerial: \"" + from + "\" is not a material of this model");
}

// $overridematerial <name> - point every loaded texture at one material. The
// entries are not merged, so an N-material model still writes N identical slots.
bool CmdOverrideMaterial(Ctx& c, const Token& cmd) {
    std::string to;
    if (!c.Want("a material name", cmd, to))
        return false;
    to = StripExtension(to);
    std::printf("$overridematerial is replacing ALL material references with %s.\n", to.c_str());
    for (auto& tex : c.in.mats.textures)
        tex.name = to;
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
bool CmdMinLod(Ctx& c, const Token& cmd) {
    if (!c.WantInt("a non-negative LOD index", cmd, c.in.minLod))
        return false;
    if (c.in.minLod < 0)
        return c.Fail(cmd.line, "$minlod must be non-negative");
    return true;
}

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
            lod.facialAnimation = !isShadow; // on is the plain-$lod default; a shadow forces it off
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

// $texturegroup [name] accepts positional material rows or explicit $set blocks.
// Legacy row 0 defines columns; later rows and each $set append skin families.
// Family 0 uses the model's own materials, shared across all groups.
bool CmdTextureGroup(Ctx& c, const Token& cmd) {
    if (!c.AtCommand() && (c.Cur().quoted || c.Cur().text != "{")) {
        std::string name;
        if (!c.Want("a group name", cmd, name))
            return false;
    }
    if (!WantOpenBrace(c, cmd, cmd.text))
        return false;

    const bool legacy = !c.Eof() && !c.Cur().quoted && c.Cur().text == "{";
    std::vector<std::string> base;
    bool haveBase = false;
    while (true) {
        if (c.Eof())
            return c.Fail(cmd.line, "$texturegroup: missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        if (legacy) {
            if (t.quoted || t.text != "{")
                return c.Fail(t.line, "$texturegroup: expected a material row or '}'");
            std::vector<std::string> row;
            while (true) {
                if (c.Eof())
                    return c.Fail(t.line, "$texturegroup: material row is missing '}'");
                if (!c.Cur().quoted && c.Cur().text == "}") {
                    ++c.pos;
                    break;
                }
                if (c.AtCommand() || (!c.Cur().quoted && c.Cur().text == "{"))
                    return c.Fail(c.Cur().line, "$texturegroup: expected a material name or '}'");
                row.push_back(c.toks[c.pos++].text);
            }
            if (!haveBase) {
                base = std::move(row);
                haveBase = true;
                continue;
            }
            if (row.size() != base.size())
                return c.Fail(t.line, "$texturegroup: every material row must have " +
                                      std::to_string(base.size()) + " entries");
            std::vector<cm::CompileInput::SkinReplace> fam;
            for (size_t i = 0; i < base.size(); ++i)
                if (row[i] != base[i])
                    fam.push_back({base[i], row[i]});
            c.in.skinFamilies.push_back(std::move(fam));
            continue;
        }
        if (t.quoted || _stricmp(t.text.c_str(), "$set") != 0)
            return c.Fail(t.line, "$texturegroup: expected $set or '}', got \"" +
                                  t.text + "\"");

        std::vector<cm::CompileInput::SkinReplace> fam;
        if (!ParseTextureSet(c, t, fam))
            return false;
        c.in.skinFamilies.push_back(std::move(fam));
    }

    // Include the base family in the limit.
    if (c.in.skinFamilies.size() + 1 > static_cast<size_t>(pulse::limits::kMaxSkinFamilies))
        return c.Fail(cmd.line, "too many $texturegroup skin families");
    return true;
}

// $meshsortorder also orders the bodyparts, since the renderer's outer draw
// loop is the bodypart array. A part ranks by the last-drawn material it holds.
// The choice list inside a part is never touched, so `mesh`/`studio`/`blank`
// keep the indices SetBodygroup's second argument addresses.
void SortBodyPartsForMeshOrder(Ctx& c) {
    if (c.in.meshSortOrder.empty())
        return;

    auto partRank = [&](const cm::CompileInput::InBodyPart& bp) {
        int rank = -1;
        for (const auto& im : bp.models)
            for (int mi = 0; im.source && mi < im.source->nummeshes; mi++)
                rank = std::max(rank, cm::MeshSortRank(c.in, im.source->meshindex[mi]));
        return rank;
    };

    std::vector<std::string> before;
    for (const auto& bp : c.in.bodyparts)
        before.push_back(bp.name);

    std::stable_sort(c.in.bodyparts.begin(), c.in.bodyparts.end(),
                     [&](const cm::CompileInput::InBodyPart& a,
                         const cm::CompileInput::InBodyPart& b) {
                         return partRank(a) < partRank(b);
                     });

    for (size_t i = 0; i < before.size(); i++) {
        if (before[i] == c.in.bodyparts[i].name)
            continue;
        std::printf("$meshsortorder reordered $modelgroups - the index SetBodygroup's "
                    "first argument takes has changed:\n");
        for (size_t j = 0; j < c.in.bodyparts.size(); j++) {
            const auto it = std::find(before.begin(), before.end(), c.in.bodyparts[j].name);
            std::printf("  %s: %d -> %d\n", c.in.bodyparts[j].name.c_str(),
                        static_cast<int>(it - before.begin()), static_cast<int>(j));
        }
        break;
    }
}

// A $meshsortorder name no drawn mesh carries orders nothing. Silence there
// would hide a typo as "my layering just didn't work".
void CheckMeshSortOrder(Ctx& c) {
    for (const std::string& want : c.in.meshSortOrder) {
        bool found = false;
        for (const auto& bp : c.in.bodyparts)
            for (const auto& im : bp.models)
                for (int mi = 0; im.source && mi < im.source->nummeshes; mi++)
                    found = found || _stricmp(cm::MeshSortMaterialName(
                                                  c.in, im.source->meshindex[mi]).c_str(),
                                              StripExtension(want).c_str()) == 0;
        if (!found)
            std::printf("WARNING: $meshsortorder \"%s\" matches no material on any mesh; "
                        "it orders nothing.\n",
                        want.c_str());
    }
}

// $meshsortorder { <material> ... } - draw order for the meshes of every
// submodel, and for the $modelgroups holding them. The engine never sorts
// meshes, so this is the only handle on which translucent surface lands on top.
bool CmdMeshSortOrder(Ctx& c, const Token& cmd) {
    if (!WantOpenBrace(c, cmd, cmd.text))
        return false;

    while (true) {
        if (c.Eof())
            return c.Fail(cmd.line, "$meshsortorder: missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        c.in.meshSortOrder.push_back(StripExtension(t.text));
    }
    return true;
}

// ---------------------------------------------------------------------------
// Hitboxes - $hitboxset may use a body or select a set for following top-level
// $hbox commands. A bare $hbox falls into an implicit "default" set.
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

// $hitboxset <name> [{ $hbox ... }] also accepts stock's flat, order-dependent
// form. Declaring any set turns automatic hitbox generation off entirely.
bool CmdHitboxSet(Ctx& c, const Token& cmd) {
    cm::HitboxSet set;
    if (!c.Want("a set name", cmd, set.name))
        return false;

    if (!c.AtCommand()) {
        if (c.Cur().quoted || c.Cur().text != "{")
            return c.Fail(c.Cur().line, cmd.text + " \"" + set.name +
                                        "\": expected '{' or the next $command");
        c.pos++;
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
    }

    c.in.hitboxsets.push_back(std::move(set));
    if (c.in.hitboxsets.size() > static_cast<size_t>(pulse::limits::kMaxHitboxSets))
        return c.Fail(cmd.line, "too many hitbox sets");
    c.activeHitboxSet = static_cast<int>(c.in.hitboxsets.size() - 1);
    return true;
}

bool CmdHboxOutsideSet(Ctx& c, const Token& cmd) {
    if (c.activeHitboxSet < 0) {
        std::printf("WARNING: %s(%d): $hbox has no preceding $hboxset; using set \"default\"\n",
                    c.file.c_str(), cmd.line);
        c.in.hitboxsets.push_back({"default", {}});
        if (c.in.hitboxsets.size() > static_cast<size_t>(pulse::limits::kMaxHitboxSets))
            return c.Fail(cmd.line, "too many hitbox sets");
        c.activeHitboxSet = static_cast<int>(c.in.hitboxsets.size() - 1);
    }
    return ParseHbox(c, cmd, c.in.hitboxsets[static_cast<size_t>(c.activeHitboxSet)]);
}

bool CmdHGroup(Ctx& c, const Token& cmd) {
    int group;
    std::string bone;
    if (!c.WantInt("a hit group", cmd, group) ||
        !c.Want("a bone name", cmd, bone))
        return false;
    c.in.hitgroups.emplace_back(std::move(bone), group);
    return true;
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

// $ikchain <name> <endbone> [control value] [knee x y z] [height h]
//          [pad p] [floor f] [center x y z] (Cmd_IKChain).
bool CmdIkChain(Ctx& c, const Token& cmd) {
    std::string name;
    if (!c.Want("a name", cmd, name))
        return false;

    for (const auto& e : c.in.ikchains)
        if (_stricmp(e.name.c_str(), name.c_str()) == 0) {
            while (!c.AtCommand())
                c.pos++;
            fprintf(stderr, "WARNING: duplicate ikchain \"%s\" ignored\n", name.c_str());
            return true;
        }

    std::string endbone;
    if (!c.Want("an end bone name", cmd, endbone))
        return false;

    cm::IkChain chain;
    chain.name = name;
    chain.bonename = endbone;

    while (!c.AtCommand()) {
        const Token t = c.toks[c.pos++];
        const Token sub{cmd.text + " " + t.text, t.line, false};
        if (LookupControl(t.text) != -1) {
            float unused = 0.0f;
            if (!c.WantFloat("a control value", sub, unused))
                return false;
        } else if (_stricmp(t.text.c_str(), "knee") == 0) {
            if (!c.WantFloat("an X", sub, chain.link[0].kneeDir.x) ||
                !c.WantFloat("a Y", sub, chain.link[0].kneeDir.y) ||
                !c.WantFloat("a Z", sub, chain.link[0].kneeDir.z))
                return false;
        } else if (_stricmp(t.text.c_str(), "height") == 0) {
            if (!c.WantFloat("a height", sub, chain.height)) return false;
        } else if (_stricmp(t.text.c_str(), "pad") == 0) {
            float pad = 0.0f;
            if (!c.WantFloat("a pad", sub, pad)) return false;
            chain.radius = pad / 2.0f;
        } else if (_stricmp(t.text.c_str(), "floor") == 0) {
            if (!c.WantFloat("a floor", sub, chain.floor)) return false;
        } else if (_stricmp(t.text.c_str(), "center") == 0) {
            if (!c.WantFloat("an X", sub, chain.center.x) ||
                !c.WantFloat("a Y", sub, chain.center.y) ||
                !c.WantFloat("a Z", sub, chain.center.z))
                return false;
        }
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

// $motionrollback <sec>  (Cmd_MotionExtractionRollBack): the motionrollback a
// new animation starts with. Per-anim `motionrollback` still overrides it.
bool CmdMotionRollback(Ctx& c, const Token& cmd) {
    float sec = 0.0f;
    if (!c.WantFloat("a rollback in seconds", cmd, sec))
        return false;
    if (sec <= 0.0f)
        return c.Fail(cmd.line, "$motionrollback must be > 0");
    c.defaultMotionRollback = sec;
    return true;
} // Not sure what does this do.

// $nosequence  (Cmd_NoSequence): accepted for stock-QC compatibility. A
// sequence-less model already compiles by default (auto reference bind pose),
// so this is a no-op.
bool CmdNoSequence(Ctx&, const Token&) { return true; }

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
// $physicsmodel uses nested shape, joint, and markup commands. Its build type
// remains automatic; the legacy collision commands set it explicitly.
// ---------------------------------------------------------------------------

bool WantVec3(Ctx& c, const Token& cmd, pm::Vector3& v) {
    return c.WantFloat("an X value", cmd, v.x) &&
           c.WantFloat("a Y value", cmd, v.y) &&
           c.WantFloat("a Z value", cmd, v.z);
}

bool LoadPhysicsFileShape(Ctx& c, const Token& cmd, const std::string& ref,
                          cm::PhysicsShape& sh) {
    const std::string file = WithSourceExtension(c, ref);
    sh.source = LoadSource(c, file, cmd.line, false, nullptr, source::LoadKind::Collision);
    if (!sh.source)
        return false;
    sh.name = pulse::FilePath(file).stem().string();
    return true;
}

// $physicsshape fromfile <file> { }, fromrendermesh <$rendermesh> { }, or
// fromrender { }. Render modes share generation options; fromrendermesh
// limits generation to the named source.
//
// The shape name is not authored - a body is named by its BONE everywhere in
// the .phy, so the name is only ever a diagnostic label and is taken from
// whatever identifies the shape (its bone, its rendermesh, its file).
bool ParsePhysShape(Ctx& c, const Token& cmd, const std::string& mode,
                    const std::string& ref, cm::PhysicsShape& sh) {
    const bool fromRender = mode != "fromfile";
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
        } else if (fromRender && o == "maxdepth") {
            if (!c.WantInt("a recursion depth", sub, sh.maxDepth)) return false;
            if (sh.maxDepth < 0 || sh.maxDepth > 10)
                return c.Fail(t.line, where + ": maxdepth must be between 0 and 10");
        } else if (fromRender && o == "extrabone") {
            std::string bone;
            if (!c.Want("a bone name", sub, bone)) return false;
            sh.extraSkinnedBones.push_back(std::move(bone));
        } else if (fromRender && !fromMesh && o == "excludemesh") {
            std::string mesh;
            if (!c.Want("a $rendermesh name", sub, mesh)) return false;
            sh.exceptionMeshNames.push_back(std::move(mesh));
        } else {
            return c.Fail(t.line, where + ": invalid syntax \"" + t.text + "\"");
        }
    }

    if (sh.importScale <= 0.0f)
        return c.Fail(cmd.line, where + ": importscale must be greater than 0");

    if (fromMesh) {
        auto it = c.rendermeshes.find(ref);
        if (it == c.rendermeshes.end())
            return c.Fail(cmd.line, where + ": references unknown rendermesh \"" +
                                    ref + "\"");
        sh.source = it->second;
    }

    if (fromRender) {
        // parentbone carves ONE body out of a skinned character. A prop has no
        // bone to cull against, so omitting it means "the whole render mesh",
        // bound to the root - and extrabone/cullweight go unused.
        sh.name = fromMesh ? ref : (sh.parentBone.empty() ? "generated" : sh.parentBone);
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

    if (!LoadPhysicsFileShape(c, cmd, ref, sh))
        return false;

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

// True when the next token is a bare number - the optional trailing friction,
// as opposed to the next axis letter or '}'.
bool AtNumber(Ctx& c) {
    if (c.Eof() || c.Cur().quoted)
        return false;
    const std::string& s = c.Cur().text;
    try {
        size_t used = 0;
        static_cast<void>(std::stof(s, &used));
        return used == s.size();
    } catch (const std::exception&) {
        return false;
    }
}

// $physicsjoint <bone> { x limit <min> <max> [friction] / y free / z fixed }
// Friction is the optional trailing NUMBER on an axis, not a keyword. Omitting
// it means 1, not 0 - write an explicit 0 for a frictionless joint.
//
// An omitted axis is LOCKED, not free - the compile stage zero-fills and only
// the axes named here move. Braces are optional: without them the axes run to
// the end of the line, and a '{' may not appear at all.
//
// `joint` may already hold axes from an earlier $physicsjoint on the same bone;
// naming an axis twice is an error either way, not a silent last-one-wins.
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
        for (const cm::PhysicsJointAxis& e : joint.axes)
            if (e.axis == a.axis)
                return c.Fail(t.line, where + ": " + ax + " is written more than once");

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
        // the next axis always starts with a letter or '}', so a bare number
        // here can only be the friction
        if (AtNumber(c) && !c.WantFloat("a friction value", sub, a.friction))
            return false;
        joint.axes.push_back(a);
    } while (braced || (!c.Eof() && c.Cur().line == cmd.line));
    return true;
}

// $physicsmarkup <bone> { ... } - the per-body override. Every field falls back
// to the $physicsmodel setting of the same name, so presence is what counts and
// an authored 0 has to beat a nonzero default.
//
// Braces are optional: without them the fields run to the end of the line, and
// a '{' may not appear at all.
//
// `mk` may already hold fields from an earlier $physicsmarkup on the same bone;
// writing a field twice is an error either way, not a silent last-one-wins.
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

        bool already = false;
        if      (o == "massbias")   already = mk.massBiasSet;
        else if (o == "inertia")    already = mk.inertiaSet;
        else if (o == "damping")    already = mk.dampingSet;
        else if (o == "rotdamping") already = mk.rotdampingSet;
        else if (o == "skip")       already = mk.skip;
        else if (o == "mergeinto")  already = !mk.mergeInto.empty();
        if (already)
            return c.Fail(t.line, where + ": " + o + " is written more than once");

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
    } while (braced || (!c.Eof() && c.Cur().line == cmd.line));

    if (mk.skip && !mk.mergeInto.empty())
        return c.Fail(cmd.line, where + ": sets both skip and mergeinto - pick one");
    return true;
}

bool CmdPhysicsModel(Ctx& c, const Token& cmd) {
    if (!WantOpenBrace(c, cmd, cmd.text))
        return false;

    const size_t shapesBefore = c.in.physShapes.size();
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
            sh.kind = mode == "fromfile" ? cm::PhysicsShapeKind::FromFile
                                         : cm::PhysicsShapeKind::FromRender;
            if (!ParsePhysShape(c, t, mode, ref, sh))
                return false;
            c.in.physShapes.push_back(std::move(sh));
            if (c.in.physShapes.size() > static_cast<size_t>(pulse::limits::kMaxPhysShapes))
                return c.Fail(t.line, "too many physics shapes (max " +
                                      std::to_string(pulse::limits::kMaxPhysShapes) + ")");
        } else if (o == "$physicsjoint") {
            // find-or-create by bone: a bone gets one entry however many
            // $physicsjoint lines name it, so the parser sees the axes already
            // written for it and rejects a second one
            std::string bone;
            if (!c.Want("a bone name", sub, bone))
                return false;
            cm::PhysicsJoint* joint = nullptr;
            for (auto& e : c.in.physJoints)
                if (_stricmp(e.bonename.c_str(), bone.c_str()) == 0) { joint = &e; break; }
            if (!joint) {
                c.in.physJoints.emplace_back();
                joint = &c.in.physJoints.back();
                joint->bonename = bone;
            }
            if (!ParsePhysJoint(c, t, *joint))
                return false;
        } else if (o == "$physicsmarkup") {
            // find-or-create by bone, same as $physicsjoint above
            std::string bone;
            if (!c.Want("a bone name", sub, bone))
                return false;
            cm::PhysicsMarkup* mk = nullptr;
            for (auto& e : c.in.physMarkups)
                if (_stricmp(e.bonename.c_str(), bone.c_str()) == 0) { mk = &e; break; }
            if (!mk) {
                c.in.physMarkups.emplace_back();
                mk = &c.in.physMarkups.back();
                mk->bonename = bone;
                mk->name = bone; // only ever used in diagnostics
            }
            if (!ParsePhysMarkup(c, t, *mk))
                return false;
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
        } else if (o == "$physicsnocollide") {
            // Subtracts from the $physicscollide lines ABOVE it - the .phy has
            // an allowed-pair list and no deny key, so there is nothing to
            // cancel that has not been written yet.
            std::string a, b;
            if (!c.Want("a bone name", sub, a) || !c.Want("a second bone name", sub, b))
                return false;
            std::vector<cm::PhysicsCollidePair>& pairs = c.in.physCollidePairs;
            const size_t before = pairs.size();
            pairs.erase(std::remove_if(pairs.begin(), pairs.end(),
                                       [&](const cm::PhysicsCollidePair& p) {
                                           return (_stricmp(p.a.c_str(), a.c_str()) == 0 &&
                                                   _stricmp(p.b.c_str(), b.c_str()) == 0) ||
                                                  (_stricmp(p.a.c_str(), b.c_str()) == 0 &&
                                                   _stricmp(p.b.c_str(), a.c_str()) == 0);
                                       }),
                        pairs.end());
            if (pairs.size() == before)
                std::printf("WARNING: $physicsnocollide \"%s\" \"%s\" matches no "
                            "$physicscollide above it - ignored\n", a.c_str(), b.c_str());
        } else if (o == "$noselfcollisions") {
            c.in.physNoSelfCollisions = true;
        } else if (o == "$assumeworldspace") {
            c.in.physAssumeWorldspace = true;
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

    if (c.in.physShapes.size() == shapesBefore)
        std::fprintf(stderr, "warning: %s line %d: $physicsmodel has no $physicsshape\n",
                     c.file.c_str(), cmd.line);

    // A zero or negative $mass is not rejected - the per-solid mass floor in
    // writephy clamps every body to 1 kg, which is what stock lands on too.
    return true;
}

cm::PhysicsMarkup& LegacyMarkup(Ctx& c, const std::string& bone) {
    for (cm::PhysicsMarkup& mk : c.in.physMarkups)
        if (_stricmp(mk.bonename.c_str(), bone.c_str()) == 0)
            return mk;
    c.in.physMarkups.emplace_back();
    cm::PhysicsMarkup& mk = c.in.physMarkups.back();
    mk.name = mk.bonename = bone;
    return mk;
}

cm::PhysicsJoint& LegacyJoint(Ctx& c, const std::string& bone) {
    for (cm::PhysicsJoint& joint : c.in.physJoints)
        if (_stricmp(joint.bonename.c_str(), bone.c_str()) == 0)
            return joint;
    c.in.physJoints.emplace_back();
    c.in.physJoints.back().bonename = bone;
    return c.in.physJoints.back();
}

void RemovePhysicsPair(Ctx& c, const std::string& a, const std::string& b) {
    std::vector<cm::PhysicsCollidePair>& pairs = c.in.physCollidePairs;
    pairs.erase(std::remove_if(pairs.begin(), pairs.end(),
                               [&](const cm::PhysicsCollidePair& pair) {
                                   return (_stricmp(pair.a.c_str(), a.c_str()) == 0 &&
                                           _stricmp(pair.b.c_str(), b.c_str()) == 0) ||
                                          (_stricmp(pair.a.c_str(), b.c_str()) == 0 &&
                                           _stricmp(pair.b.c_str(), a.c_str()) == 0);
                               }),
                pairs.end());
}

bool CmdLegacyCollision(Ctx& c, const Token& cmd, cm::PhysicsBuildMode mode) {
    std::string file;
    if (!c.Want("a collision source filename", cmd, file))
        return false;
    if (Lower(file) == "blank")
        return c.Fail(cmd.line, cmd.text + " requires a source file; use $physicsmodel for generation");

    cm::PhysicsShape shape;
    shape.maxConvex = 40;
    if (!LoadPhysicsFileShape(c, cmd, file, shape))
        return false;
    const size_t mainShape = c.in.physShapes.size();
    c.in.physShapes.push_back(std::move(shape));
    c.in.physBuildMode = mode;

    const bool braced = !c.Eof() && !c.Cur().quoted && c.Cur().text == "{";
    if (!braced)
        return true;
    c.pos++;

    int maxConvex = 40;
    bool remove2d = false;
    for (;;) {
        if (c.Eof())
            return c.Fail(cmd.line, cmd.text + ": missing '}'");
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "}")
            break;
        const std::string o = t.quoted ? std::string() : Lower(t.text);
        const Token sub{cmd.text + " " + t.text, t.line, false};

        if (o == "$mass") {
            if (!c.WantFloat("a mass in kg", sub, c.in.physMass)) return false;
            c.in.physAutoMass = false;
        } else if (o == "$automass" || o == "$calculatemass") {
            c.in.physAutoMass = true;
        } else if (o == "$inertia") {
            if (!c.WantFloat("an inertia", sub, c.in.physInertia)) return false;
        } else if (o == "$damping") {
            if (!c.WantFloat("a damping value", sub, c.in.physDamping)) return false;
        } else if (o == "$rotdamping") {
            if (!c.WantFloat("a rotdamping value", sub, c.in.physRotdamping)) return false;
        } else if (o == "$drag") {
            if (!c.WantFloat("a drag value", sub, c.in.physDrag)) return false;
        } else if (o == "$maxconvexpieces") {
            if (!c.WantInt("a piece count", sub, maxConvex)) return false;
        } else if (o == "$remove2d") {
            remove2d = true;
        } else if (o == "$concaveperjoint") {
            c.in.physLegacyConcavePerJoint = true;
        } else if (o == "$weldposition") {
            if (!c.WantFloat("a position epsilon", sub, c.in.physWeldPosition)) return false;
        } else if (o == "$weldnormal") {
            if (!c.WantFloat("a normal epsilon", sub, c.in.physWeldNormal)) return false;
        } else if (o == "$concave") {
            c.in.physLegacyConcave = true;
        } else if (o == "$convexhullcountoverride") {
            std::string ignored;
            if (!c.Want("an override value", sub, ignored)) return false;
            c.in.physConvexHullCountOverride = true;
        } else if (o == "$masscenter") {
            if (!WantVec3(c, sub, c.in.physMassCenter)) return false;
            c.in.physMassCenterSet = true;
        } else if (o == "$jointskip") {
            std::string bone;
            if (!c.Want("a bone name", sub, bone)) return false;
            cm::PhysicsMarkup& mk = LegacyMarkup(c, bone);
            mk.skip = true;
            mk.mergeInto.clear();
        } else if (o == "$jointmerge") {
            std::string parent, child;
            if (!c.Want("a parent bone name", sub, parent) ||
                !c.Want("a child bone name", sub, child)) return false;
            cm::PhysicsMarkup& mk = LegacyMarkup(c, child);
            mk.skip = false;
            mk.mergeInto = parent;
        } else if (o == "$rootbone") {
            if (!c.Want("a bone name", sub, c.in.physRootBone)) return false;
        } else if (o == "$jointconstrain") {
            std::string bone, axis, type;
            cm::PhysicsJointAxis a;
            if (!c.Want("a bone name", sub, bone) || !c.Want("an axis", sub, axis) ||
                !c.Want("free, fixed or limit", sub, type) ||
                !c.WantFloat("a minimum angle", sub, a.min) ||
                !c.WantFloat("a maximum angle", sub, a.max)) return false;
            axis = Lower(axis);
            type = Lower(type);
            if      (axis == "x") a.axis = 0;
            else if (axis == "y") a.axis = 1;
            else if (axis == "z") a.axis = 2;
            else return c.Fail(t.line, sub.text + ": expected x, y or z");
            if      (type == "free")  { a.type = 0; a.min = -360.0f; a.max = 360.0f; }
            else if (type == "limit") { a.type = 1; }
            else if (type == "fixed") { a.type = 2; a.min = a.max = 0.0f; }
            else return c.Fail(t.line, sub.text + ": expected free, fixed or limit");
            if (AtNumber(c) && !c.WantFloat("a friction value", sub, a.friction)) return false;
            cm::PhysicsJoint& joint = LegacyJoint(c, bone);
            auto existing = std::find_if(joint.axes.begin(), joint.axes.end(),
                                         [&](const cm::PhysicsJointAxis& value) {
                                             return value.axis == a.axis;
                                         });
            if (existing == joint.axes.end()) joint.axes.push_back(a);
            else *existing = a;
        } else if (o == "$jointinertia" || o == "$jointdamping" ||
                   o == "$jointrotdamping" || o == "$jointmassbias") {
            std::string bone;
            float value = 0.0f;
            if (!c.Want("a bone name", sub, bone) || !c.WantFloat("a value", sub, value))
                return false;
            cm::PhysicsMarkup& mk = LegacyMarkup(c, bone);
            if (o == "$jointinertia") { mk.inertia = value; mk.inertiaSet = true; }
            else if (o == "$jointdamping") { mk.damping = value; mk.dampingSet = true; }
            else if (o == "$jointrotdamping") { mk.rotdamping = value; mk.rotdampingSet = true; }
            else { mk.massBias = value; mk.massBiasSet = true; }
        } else if (o == "$noselfcollisions") {
            c.in.physNoSelfCollisions = true;
        } else if (o == "$jointcollide" || o == "$jointnocollide") {
            std::string a, b;
            if (!c.Want("a bone name", sub, a) || !c.Want("a second bone name", sub, b))
                return false;
            if (o == "$jointcollide") c.in.physCollidePairs.push_back({a, b});
            else RemovePhysicsPair(c, a, b);
        } else if (o == "$jointcollidealltoall") {
            const Token* open = c.Next();
            if (!open || open->quoted || open->text != "{")
                return c.Fail(t.line, sub.text + " expects '{'");
            std::vector<std::string> bones;
            for (;;) {
                const Token* bone = c.Next();
                if (!bone) return c.Fail(t.line, sub.text + ": missing '}'");
                if (!bone->quoted && bone->text == "}") break;
                if (bones.size() < 32) bones.push_back(bone->text);
            }
            
            for (size_t i = 0; i < bones.size(); i++)
                for (size_t j = 0; j < bones.size(); j++)
                    if (i != j)
                        c.in.physCollidePairs.push_back({bones[i], bones[j]});
        } else if (o == "$animatedfriction") {
            c.in.physHasAnimatedFriction = true;
            if (!c.WantInt("a minimum friction", sub, c.in.physAnimFrictionMin) ||
                !c.WantInt("a maximum friction", sub, c.in.physAnimFrictionMax) ||
                !c.WantFloat("a ramp-in time", sub, c.in.physAnimFrictionTimeIn) ||
                !c.WantFloat("a hold time", sub, c.in.physAnimFrictionTimeHold) ||
                !c.WantFloat("a ramp-out time", sub, c.in.physAnimFrictionTimeOut)) return false;
        } else if (o == "$assumeworldspace") {
            c.in.physAssumeWorldspace = true;
        } else if (o == "$addconvexsrc") {
            std::string extra;
            if (!c.Want("a collision source filename", sub, extra)) return false;
            cm::PhysicsShape extraShape;
            extraShape.maxConvex = maxConvex;
            extraShape.remove2d = remove2d;
            if (!LoadPhysicsFileShape(c, sub, extra, extraShape)) return false;
            c.in.physShapes.push_back(std::move(extraShape));
        } else if (o == "$generate" || o == "$generatemodel" || o == "$generatejoint" ||
                   o == "$addgeneratechild") {
            return c.Fail(t.line, t.text + " is not supported here; use $physicsmodel for generation");
        } else {
            return c.Fail(t.line, cmd.text + ": invalid syntax \"" + t.text + "\"");
        }
    }

    for (size_t i = mainShape; i < c.in.physShapes.size(); i++) {
        c.in.physShapes[i].maxConvex = maxConvex;
        c.in.physShapes[i].remove2d = remove2d;
    }
    c.in.physShapes[mainShape].concave =
        mode == cm::PhysicsBuildMode::Single ? c.in.physLegacyConcave
                                             : c.in.physLegacyConcavePerJoint;
    return true;
}

bool CmdCollisionModel(Ctx& c, const Token& cmd) {
    return CmdLegacyCollision(c, cmd, cm::PhysicsBuildMode::Single);
}

bool CmdCollisionJoints(Ctx& c, const Token& cmd) {
    return CmdLegacyCollision(c, cmd, cm::PhysicsBuildMode::Ragdoll);
}

// The $physicsmodel body commands, written at top level. Listed in the command
// table only so they report what is actually wrong instead of "unknown
// command" - the same reason $hbox is listed.
bool CmdPhysicsOutsideModel(Ctx& c, const Token& cmd) {
    return c.Fail(cmd.line, cmd.text + " must appear inside a $physicsmodel { } block");
}

// the dispatch loop, defined under the command table - $include reenters it
bool RunCommands(Ctx& c);

// $pushd <dir> / $popd select the primary directory for source files.
// Relative pushes nest under the current directory. Popping the root is a no-op.
bool CmdPushD(Ctx& c, const Token& cmd) {
    std::string dir;
    if (!c.Want("a directory", cmd, dir))
        return false;
    const fs::path path = pulse::FilePath(dir);
    const fs::path next = path.is_absolute() ? path : c.sourceDirStack.back() / path;
    c.sourceDirStack.push_back(next.lexically_normal().make_preferred());
    return true;
}

bool CmdPopD(Ctx& c, const Token&) {
    if (c.sourceDirStack.size() > 1)
        c.sourceDirStack.pop_back();
    return true;
}

// $addsearchdir <dir>  (Cmd_AddSearchDir). A fallback directory for SOURCE
// files - the .dmx/.smd a $rendermesh or $animation names, and $proceduralbones'
// .vrd. Registration order, searched after the current $pushd directory, so only
// dirs registered ABOVE a reference can serve it. A relative dir is relative to
// the ROOT script, the same rule the source paths it serves follow (stock joins
// cddir[0] here for exactly that reason).
//
// This is NOT $addincludesearchdir - source files and $include scripts keep
// separate lists. -filesearchdir is searched after this list.
bool CmdAddSearchDir(Ctx& c, const Token& cmd) {
    std::string dir;
    if (!c.Want("a directory", cmd, dir))
        return false;
    const fs::path path = pulse::FilePath(dir);
    const fs::path p = path.is_absolute() ? path : c.scriptDir / path;
    c.searchDirs.push_back(p.lexically_normal().make_preferred());
    return true;
}

// $addincludesearchdir <dir>  (scriplib's AddIncludeDir; -includesearchdir is
// the launch-line form, searched after this list). Registers a fallback directory for $include; the
// list is searched in registration order, so only dirs registered ABOVE an
// $include can serve it. A relative dir is relative to the file registering it,
// the same rule the $include path itself follows, and is resolved here so the
// entry means one place no matter who reads it later.
bool CmdAddIncludeSearchDir(Ctx& c, const Token& cmd) {
    std::string dir;
    if (!c.Want("a directory", cmd, dir))
        return false;
    const fs::path path = pulse::FilePath(dir);
    const fs::path p = path.is_absolute() ? path : c.curDir / path;
    c.includeDirs.push_back(p.lexically_normal().make_preferred());
    return true;
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

// $qcassert <boneexists|importboneexists> <bone> [source] <true|false>
bool CmdQcAssert(Ctx& c, const Token& cmd) {
    std::string type, bone;
    if (!c.Want("boneexists or importboneexists", cmd, type) ||
        !c.Want("a bone name", cmd, bone))
        return false;

    bool actual = false;
    std::string line = "QC Assert: " + type + " " + bone;
    if (_stricmp(type.c_str(), "boneexists") == 0) {
        std::string file;
        if (!c.Want("a source filename", cmd, file))
            return false;
        line += " " + file;
        file = WithSourceExtension(c, file);
        source::Source* src = LoadSource(c, file, cmd.line);
        if (!src)
            return false;
        actual = std::any_of(src->localBone.begin(), src->localBone.end(),
                             [&](const source::LocalBone& b) {
                                 return _stricmp(b.name.c_str(), bone.c_str()) == 0;
                             });
    } else if (_stricmp(type.c_str(), "importboneexists") == 0) {
        actual = std::any_of(c.in.importbones.begin(), c.in.importbones.end(),
                             [&](const cm::ImportBone& b) {
                                 return _stricmp(b.name.c_str(), bone.c_str()) == 0;
                             });
    } else {
        return c.Fail(cmd.line, "$qcassert: unknown assertion type \"" + type + "\"");
    }

    std::string expectedText;
    if (!c.Want("true or false", cmd, expectedText))
        return false;
    bool expected = false;
    if (_stricmp(expectedText.c_str(), "true") == 0)
        expected = true;
    else if (_stricmp(expectedText.c_str(), "false") != 0)
        return c.Fail(cmd.line, "$qcassert expects true or false, got \"" + expectedText + "\"");

    line += " " + expectedText + " RESULT: ";
    if (actual != expected)
        return c.Fail(cmd.line, line + "[Fail]");
    std::printf("%s[Success]\n", line.c_str());
    return true;
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

// $include "file.qci" [localdir] [optional]   (scriplib's
// AttemptConditionalInclude; its nofallbackdir flag is `localdir` here, its
// iffileexist flag is `optional`). Splices the file's tokens in where the
// $include stands, so it is transparent at any depth - top level or inside a
// braced body - exactly like stock's streaming GetToken. Flags are read only on
// the $include's own line; a token on a later line is content, not a flag.
//
//   localdir   only look at the primary location, skip the search dirs
//   optional   found nowhere = do nothing, instead of an error
//
// The path is relative to the file the $include is written in, then to the root
// script's directory, then to each $addincludesearchdir dir. Everything else in
// an included file - source filenames, $proceduralbones - stays relative to the
// ROOT script's directory, matching how stock keeps cddir pinned to the top.
bool SpliceInclude(Ctx& c) {
    const Token cmd = c.toks[c.pos]; // the $include token
    size_t p = c.pos + 1;
    if (p >= c.toks.size() || (!c.toks[p].quoted && c.toks[p].text[0] == '$'))
        return c.Fail(cmd.line, "$include expects a script path");

    Token pathTok = c.toks[p++];
    if (!pathTok.expanded && !ExpandVars(c, pathTok))
        return false;
    const std::string rel = pathTok.text;

    bool localDir = false, optional = false;
    while (p < c.toks.size() && !c.toks[p].quoted && c.toks[p].line == cmd.line &&
           c.toks[p].text[0] != '$' && c.toks[p].text != "{" && c.toks[p].text != "}") {
        const std::string& o = c.toks[p].text;
        if (_stricmp(o.c_str(), "localdir") == 0)
            localDir = true;
        else if (_stricmp(o.c_str(), "optional") == 0)
            optional = true;
        else
            return c.Fail(c.toks[p].line, "$include: unknown parameter \"" + o +
                                          "\" - localdir, optional");
        p++;
    }
    const size_t directiveEnd = p; // [c.pos, directiveEnd) is the $include to drop

    // primary location first, then the search dirs in registration order. An
    // absolute path is itself and nothing else.
    const fs::path relPath = pulse::FilePath(rel);
    std::vector<fs::path> tries;
    if (relPath.is_absolute()) {
        tries.push_back(relPath);
    } else {
        tries.push_back(c.curDir / relPath);
        // stock resolves every $include against the root script's dir, so a
        // nested one often carries the whole path down from there
        if (c.curDir != c.scriptDir)
            tries.push_back(c.scriptDir / relPath);
        if (!localDir)
            for (const auto* list : {&c.includeDirs, &c.launchIncludeDirs})
                for (const fs::path& dir : *list) {
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
        c.toks.erase(c.toks.begin() + c.pos, c.toks.begin() + directiveEnd);
        for (Ctx::IncludeFrame& f : c.includeFrames)
            if (f.endPos > c.pos)
                f.endPos -= (directiveEnd - c.pos);
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

    std::printf("$include: Including %s...\n", full.string().c_str());

    const std::string name = full.filename().string();
    std::vector<Token> toks;
    if (!Tokenize(text, name, toks, c.err))
        return false;

    // drop the $include directive, splice the file in its place, and open a
    // frame so pos crossing its end restores file/curDir and the cycle guard.
    c.toks.erase(c.toks.begin() + c.pos, c.toks.begin() + directiveEnd);
    for (Ctx::IncludeFrame& fr : c.includeFrames)
        if (fr.endPos > c.pos)
            fr.endPos -= (directiveEnd - c.pos);
    const size_t at = c.pos;
    c.SpliceAt(at, toks); // extends any enclosing frame over the new content
    c.includeFrames.push_back({at + toks.size(), c.file, c.curDir});
    c.includeStack.push_back(key);
    c.file = name;
    c.curDir = full.parent_path();
    return true;
}

// Conditional handlers, defined below - the hook resolves them at read time.
bool CmdIf(Ctx& c, const Token& cmd);
bool CmdIfdef(Ctx& c, const Token& cmd);
bool CmdIfndef(Ctx& c, const Token& cmd);
bool CmdSwitch(Ctx& c, const Token& cmd);

// Runs the conditional whose command token sits at the cursor, splicing the
// winning branch in place. Returns false only when nothing matched.
bool ResolveConditionalHere(Ctx& c) {
    const char* t = c.toks[c.pos].text.c_str();
    bool (*fn)(Ctx&, const Token&) =
        _stricmp(t, "$if") == 0       ? CmdIf     :
        _stricmp(t, "$ifdef") == 0    ? CmdIfdef  :
        _stricmp(t, "$ifndef") == 0   ? CmdIfndef :
        _stricmp(t, "$switch") == 0   ? CmdSwitch : nullptr;
    if (!fn)
        return false;
    const Token cmd = c.toks[c.pos]; // copy; the handler consumes from c.pos
    c.pos++;
    if (!fn(c, cmd))
        c.abortErr = true; // the handler's Fail already recorded the message
    return true;
}

void ResolveDirectives(Ctx& c) {
    if (c.resolvingInclude || c.abortErr || c.rawCollect)
        return;
    c.resolvingInclude = true;
    auto popFinished = [&] {
        while (!c.includeFrames.empty() && c.pos >= c.includeFrames.back().endPos) {
            c.file = c.includeFrames.back().savedFile;
            c.curDir = c.includeFrames.back().savedDir;
            c.includeStack.pop_back();
            c.includeFrames.pop_back();
        }
    };
    popFinished();
    while (c.pos < c.toks.size() && !c.abortErr && !c.toks[c.pos].quoted) {
        if (_stricmp(c.toks[c.pos].text.c_str(), "$include") == 0) {
            if (!SpliceInclude(c)) { c.abortErr = true; break; }
        } else if (!ResolveConditionalHere(c)) {
            break;
        }
        popFinished(); // an empty include/branch leaves pos at its own end
    }
    c.resolvingInclude = false;
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
        _stricmp(cmd, "$ifdef") == 0 || _stricmp(cmd, "$ifndef") == 0 ||
        _stricmp(cmd, "$switch") == 0)
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

// $definemacro <name> [<param>[=<default>] ...] <body> $endmacro.
// Parameters end at the first $command. Body references resolve on invocation.
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
        const std::string declaration = c.toks[c.pos++].text;
        const size_t equal = declaration.find('=');
        const std::string p = declaration.substr(0, equal);
        if (p.empty())
            return c.Fail(cmd.line, "$definemacro \"" + name + "\": empty parameter name");
        for (const std::string& prev : m.params)
            if (_stricmp(prev.c_str(), p.c_str()) == 0)
                return c.Fail(cmd.line, "$definemacro \"" + name +
                                        "\": duplicate parameter \"" + p + "\"");
        m.params.push_back(p);
        if (equal != std::string::npos) {
            std::string value = declaration.substr(equal + 1);
            if (value.empty() && !c.Eof() && c.Cur().quoted)
                value = c.toks[c.pos++].text;
            m.defaults.emplace(p, std::move(value));
        }
        if (static_cast<int>(m.params.size()) > lim::kMaxMacroParams)
            return c.Fail(cmd.line, "$definemacro \"" + name + "\": too many "
                                    "parameters (max " +
                                    std::to_string(lim::kMaxMacroParams) + ")");
    }
    // body up to the matching $endmacro - a nested $definemacro takes its own
    const bool savedRaw = c.rawCollect;
    c.rawCollect = true;
    for (int depth = 1;;) {
        if (c.Eof()) {
            c.rawCollect = savedRaw;
            return c.Fail(cmd.line, "$definemacro \"" + name +
                                    "\" is missing its $endmacro");
        }
        const Token t = c.toks[c.pos++];
        if (!t.quoted && _stricmp(t.text.c_str(), "$definemacro") == 0) {
            depth++;
        } else if (!t.quoted && _stricmp(t.text.c_str(), "$endmacro") == 0) {
            if (--depth == 0)
                break;
        }
        m.body.push_back(t);
    }
    c.rawCollect = savedRaw;
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
        const auto fallback = m.defaults.find(p);
        if (c.AtCommand() && fallback != m.defaults.end()) {
            args.push_back(fallback->second);
            continue;
        }
        const std::string what = "an argument for \"" + p + "\"";
        std::string a;
        if (!c.Want(what.c_str(), cmd, a))
            return false;
        args.push_back(std::move(a));
    }
    std::vector<Token> body = m.body;
    for (Token& t : body)
        SubstMacroParams(t, m.params, args);
    c.SpliceAt(c.pos, body);
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
    const bool savedRaw = c.rawCollect;
    c.rawCollect = true;
    for (int depth = 1;;) {
        if (c.Eof()) {
            c.rawCollect = savedRaw;
            return c.Fail(cmd.line, cmd.text + " is missing its closing \"}\"");
        }
        const Token t = c.toks[c.pos++];
        if (!t.quoted && t.text == "{") {
            depth++;
        } else if (!t.quoted && t.text == "}" && --depth == 0) {
            break;
        }
        body.push_back(t);
    }
    c.rawCollect = savedRaw;
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

// $ifdef <variable> - true when the name has a value, whatever that value is;
// $ifndef is the same read, negated.
// The name is taken bare and unexpanded; anything past it means a comparison
// was written, which belongs under $if.
bool ReadDefName(Ctx& c, const Token& cmd, bool& result) {
    if (c.Eof() || c.Cur().quoted || c.Cur().text == "{" || c.Cur().text == "}")
        return c.Fail(cmd.line, cmd.text + " expects a variable name");
    const Token name = c.toks[c.pos++];
    if (c.Eof() || c.Cur().quoted || c.Cur().text != "{")
        return c.Fail(name.line, cmd.text + " takes one bare variable name - a "
                                 "condition belongs under $if, not " + cmd.text);
    c.pos++;
    result = c.variables.find(name.text) != c.variables.end();
    return true;
}

// $if <cond> { ... } [$elif <cond> { ... }]... [$else { ... }], and $ifdef /
// $ifndef with the same shape but a name check in every clause. Every clause's
// condition is evaluated even once one has won, so a typo in a later $elif is
// still reported; only the BODIES of the losers go unread.
bool IfChain(Ctx& c, const Token& cmd, bool ifdef, bool negate = false) {
    std::vector<Token> chosen;
    bool taken = false;
    for (Token clause = cmd;;) {
        bool val = false;
        std::vector<Token> body;
        if (ifdef) {
            if (!ReadDefName(c, clause, val))
                return false;
            val = val != negate;
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
    c.SpliceAt(c.pos, chosen);
    return true;
}

bool CmdIf(Ctx& c, const Token& cmd) { return IfChain(c, cmd, false); }
bool CmdIfdef(Ctx& c, const Token& cmd) { return IfChain(c, cmd, true); }
bool CmdIfndef(Ctx& c, const Token& cmd) { return IfChain(c, cmd, true, /*negate=*/true); }

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
    c.SpliceAt(c.pos, chosen);
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

// $forcecapsules: undocumented top-level flag in Valve's survivor QCs.
// No capsule build path exists here, so it is accepted and silently ignored.
bool CmdForceCapsules(Ctx&, const Token&) { return true; }

// ---------------------------------------------------------------------------
// Command table - the whole supported surface. Add, rename or drop a line and
// the language changes; nothing else needs touching.
// ---------------------------------------------------------------------------

struct Command {
    const char* name;
    bool (*fn)(Ctx&, const Token&);
};

constexpr Command kCommands[] = {
    {"$break", CmdBreak},
    {"$assert", CmdAssert},
    {"$qcassert", CmdQcAssert},
    {"$print", CmdPrint},
    {"$definevariable", CmdDefineVariable},
    {"$redefinevariable", CmdRedefineVariable},
    {"$definemacro", CmdDefineMacro},
    {"$endmacro", CmdEndMacroOutside},
    {"$if", CmdIf},
    {"$ifdef", CmdIfdef},
    {"$ifndef", CmdIfndef},
    {"$elif", CmdElifOutside},
    {"$else", CmdElifOutside},
    {"$switch", CmdSwitch},
    {"$case", CmdCaseOutside},
    {"$default", CmdCaseOutside},
    {"$addincludesearchdir", CmdAddIncludeSearchDir},
    {"$pushd", CmdPushD},
    {"$popd", CmdPopD},
    {"$addsearchdir", CmdAddSearchDir},
    {"$modelname", CmdModelName},
    {"$rendermesh", CmdRenderMesh},
    {"$modelgroup", CmdModelGroup},
    {"$body", CmdLegacyBody},
    {"$bodygroup", CmdLegacyBody},
    {"$model", CmdModel},
    {"$modelgrouppreset", CmdModelGroupPreset},
    {"$bodygrouppreset", CmdModelGroupPreset},
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
    {"$motionrollback", CmdMotionRollback},
    {"$nosequence", CmdNoSequence},
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
    {"$staticprop", CmdStaticProp},
    {"$simpleprop", CmdSimpleProp},
    {"$autocenter", CmdAutoCenter},
    {"$vtxformat", CmdVtxFormat},
    {"$modelbudget", CmdModelBudget},
    {"$setbindpose", CmdSetBindPose},
    {"$setflex", CmdSetFlex},
    {"$renderpass", CmdRenderPass},
    {"$opaque", CmdOpaque},
    {"$mostlyopaque", CmdMostlyOpaque},
    {"$surfaceprop", CmdSurfaceProp},
    {"$contents", CmdContents},
    {"$jointcontents", CmdJointContents},
    {"$jointsurfaceprop", CmdJointSurfaceProp},
    {"$keyvalues", CmdKeyValues},
    {"$collisiontext", CmdCollisionText},
    {"$boneflexdriver", CmdBoneFlexDriver},
    {"$flexcontroller", CmdFlexController},
    {"$flexlocalvar", CmdFlexLocalVar},
    {"$flexrule", CmdFlexRule},
    {"$flexcorrective", CmdFlexCorrective},
    {"$flexdominate", CmdFlexDominate},
    {"$morphsplitstereo", CmdMorphSplitStereo},
    {"$renamemorph", CmdRenameMorph},
    {"$flexcullmethod", CmdFlexCullMethod},
    {"$animationcullmethod", CmdAnimationCullMethod},
    {"$eyeball", CmdEyeball},
    {"$mouth", CmdMouth},
    {"$eyelid", CmdEyelid},
    {"$skiptransition", CmdSkipTransition},
    {"$calctransitions", CmdCalcTransitions},
    {"$transformmodel", CmdTransformModel},
    {"$origin", CmdOrigin},
    {"$scale", CmdScale},
    {"$upaxis", CmdUpAxis},
    {"$attachment", CmdAttachment},
    {"$declareattachment", CmdDeclareAttachment},
    {"$hitboxset", CmdHitboxSet},
    {"$hboxset", CmdHitboxSet}, // stock's spelling, same command
    {"$hbox", CmdHboxOutsideSet},
    {"$hgroup", CmdHGroup},
    {"$renamehboxset", CmdRenameHboxSet},
    {"$bonecullmethod", CmdBoneCullMethod},
    {"$physicsmodel", CmdPhysicsModel},
    {"$collisionmodel", CmdCollisionModel},
    {"$collisionjoints", CmdCollisionJoints},
    {"$physicsshape", CmdPhysicsOutsideModel},
    {"$physicsjoint", CmdPhysicsOutsideModel},
    {"$physicsmarkup", CmdPhysicsOutsideModel},
    {"$physicscollide", CmdPhysicsOutsideModel},
    {"$physicsnocollide", CmdPhysicsOutsideModel},
    {"$assumeworldspace", CmdPhysicsOutsideModel},
    {"$jigglebone", CmdJiggleBone},
    {"$driverbone", CmdDriverBone},
    {"$driveraimat", CmdDriverAimAt},
    {"$proceduralbones", CmdProceduralBones},
    {"$realignbones", CmdRealignBones},
    {"$lockbonelengths", CmdLockBoneLengths},
    {"$limitrotation", CmdLimitRotation},
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
    {"$forcecapsules", CmdForceCapsules},
    {"$donotcastshadows", CmdDoNotCastShadows},
    {"$forcephonemecrossfade", CmdForcePhonemeCrossfade},
    {"$noforcedfade", CmdNoForcedFade},
    {"$casttextureshadows", CmdCastTextureShadows},
    {"$constantdirectionallight", CmdConstantDirectionalLight},
    {"$skipboneinbbox", CmdSkipBoneInBBox},
    {"$bbox", CmdBBox},
    {"$cbox", CmdCBox},
    {"$illumposition", CmdIllumPosition},
    {"$eyeposition", CmdEyePosition},
    {"$maxeyedeflection", CmdMaxEyeDeflection},
    {"$cdmaterials", CmdCdMaterials},
    {"$renamebone", CmdRenameBone},
    {"$renamematerial", CmdRenameMaterial},
    {"$overridematerial", CmdOverrideMaterial},
    {"$texturegroup", CmdTextureGroup},
    {"$meshsortorder", CmdMeshSortOrder},
    {"$minlod", CmdMinLod},
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
        if (c.in.stripLods && !c.toks[c.pos].quoted &&
            (_stricmp(c.toks[c.pos].text.c_str(), "$lod") == 0 ||
             _stricmp(c.toks[c.pos].text.c_str(), "$shadowlod") == 0)) {
            c.pos = CommandExtent(c);
            continue;
        }
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
    return !c.abortErr; // a hard $include error surfaces as Eof
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

void PrintCommandNames() {
    for (const Command& k : kCommands)
        std::printf("%s\n", k.name);
}

bool IsQcScriptPath(const char* path) {
    const std::string ext = pulse::FilePath(path).extension().string();
    return _stricmp(ext.c_str(), ".pulseqc") == 0 || _stricmp(ext.c_str(), ".qc") == 0;
}

bool LoadQcScript(const char* rawPath, cm::CompileInput& out, std::string* err,
                  const ScriptVars& defvars, const SearchDirs& includeDirs,
                  const SearchDirs& fileDirs) {
    const std::string normalizedPath = pulse::FilePath(rawPath).string();
    const char* path = normalizedPath.c_str();
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
    c.sourceDirStack.push_back(c.scriptDir);
    c.file = fs::path(path).filename().string();
    c.rootFile = c.file;
    c.err = err;

    // -defvar: in effect from the first line, and pinned - a later repeat on
    // the command line just wins over the earlier one
    for (const auto& kv : defvars) {
        c.variables[kv.first] = kv.second;
        if (!c.IsLockedVar(kv.first))
            c.lockedVars.push_back(kv.first);
    }

    auto seedDirs = [](const SearchDirs& in, std::vector<fs::path>& out) {
        for (const std::string& dir : in) {
            const fs::path path = pulse::FilePath(dir);
            const fs::path p = path.is_absolute() ? path : fs::current_path() / path;
            out.push_back(p.lexically_normal().make_preferred());
        }
    };
    seedDirs(includeDirs, c.launchIncludeDirs);
    seedDirs(fileDirs, c.launchSearchDirs);

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
    // animation-only models (.mdl + .ani, no geometry) are legal; a script with
    // geometry that never groups it is not - the meshes would not ship.
    if (out.bodyparts.empty() && !c.rendermeshes.empty())
        printf("WARNING: %s: $rendermesh but no $modelgroup\n", c.file.c_str());

    // set once every source has been read (the DMX loader latches it on the
    // first model carrying an upAxis attribute)
    out.upAxisY = source::DmxUpAxisY();

    // $renamemorph: before anything below resolves a morph by name
    if (!ApplyMorphRenames(c))
        return false;

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

    // both need every $modelgroup parsed. The sort must run HERE: RegisterFlex
    // below flattens model indices by walking c.in.bodyparts, so permuting them
    // any later leaves every FlexKey::imodel pointing at the wrong model.
    CheckMeshSortOrder(c);
    SortBodyPartsForMeshOrder(c);

    // flex/morph: the automatic per-body DMX rig plus the $flexcontroller /
    // $flexlocalvar / $flexrule / $flexcorrective block. Needs the finished
    // bodygroup list, so it runs here rather than per command.
    {
        std::string flexErr;
        if (!RegisterFlex(out, c.manual, &flexErr, &c.face)) {
            if (err) *err = c.file + ": " + flexErr;
            return false;
        }
    }

    // needs the final rotation/scale, so it runs after everything above
    FinishAttachments(c);
    return true;
}

} // namespace pulse::loader
