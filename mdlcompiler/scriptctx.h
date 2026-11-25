// scriptctx.h - token and loader state shared by the .qc loader and the .mdlasm loader.

#ifndef PULSEMODEL_SCRIPTCTX_H
#define PULSEMODEL_SCRIPTCTX_H

#include <cctype>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "api.h"
#include "compile.h"
#include "diagnostic.h"
#include "dmxloader.h"
#include "facemarkup.h"
#include "flexreg.h"
#include "meshedit.h"
#include "pathcompat.h"
#include "qcloader.h"
#include "skeletonedit.h"
#include "smdloader.h"
#include "source.h"

namespace pulse::loader {

namespace cm = pulse::compile;
namespace lim = pulse::limits;
namespace pm = pulse::math;
namespace fs = std::filesystem;

namespace detail {

struct Token {
    std::string text;
    int line = 0;
    bool quoted = false; // came from "..." - so a quoted "loop" is a filename
    // the text came out of a $name$ expansion (a variable or a macro
    // parameter). Only conditions care: an expanded operand is a literal value,
    // never the name of another variable to look up.
    bool expanded = false;
    bool expression = false; // a $definevariable "( ... )" value, parens included
    int apiSource = -1;
    int column = 0; // 0-based byte offset in its line, for bare words
};

// The edits one $rendermesh { } body asks for (meshedit.h).
struct MeshEdit {
    std::string name; // the $rendermesh alias, for the load print
    source::MeshFilter filter;
    std::vector<std::vector<source::EditMeshOperation>> editBlocks;
    std::vector<std::vector<source::EditSkeletonOperation>> skeletonBlocks;
    std::vector<std::pair<bool, size_t>> editOrder; // (is skeleton, block index), script order
    bool noMorph = false;
    bool skeletalAwareDecimation = false; // $use_skeletalaware_decimation
    // $decimate / $decimatemesh in script order: (factor, face tag or -1 = all)
    std::vector<std::pair<float, int>> decimates;
    std::string vta; // $vta: a legacy morph file for an SMD mesh
    int vtaLine = 0;
    std::vector<source::VtaFlexOption> vtaFlexes;
    std::string vca, vcaName; // $vca: the same file played back as one NWAY run
    int vcaLine = 0;
};

struct Ctx;
// Resolves any $include or conditional ($if/$switch) at the read cursor in place,
// so both are transparent at any nesting depth - a handler reading its own braced
// body gets them resolved too. Defined under the helpers it needs.
void ResolveDirectives(Ctx& c);

struct Ctx {
    cm::CompileInput& in;
    fs::path scriptDir; // root script's directory
    std::string file;
    std::vector<std::string> apiSources;
    const ScriptFiles* unsaved = nullptr;
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
    bool keepGoing = false;       // analyze: skip a failed command and read on
    bool hadError = false;        // keepGoing skipped something; the script cannot compile
    std::vector<Token> options;   // option words of the running command, see MarkOption
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
    // .mdlasm only: embedded source bytes by the virtual name a command refers to them by
    std::map<std::string, std::shared_ptr<const std::string>> memorySources;
    // .mdlasm only: a command has no line, so its "line" indexes this and the label names the node
    std::vector<std::string> nodeLabels;
    // full path -> parsed DMX. Read-only input, so the same file feeding several
    // loads (each $rendermesh reopens it) is read and parsed once.
    std::map<std::string, std::shared_ptr<pulse::dmx::Datamodel>> dmxCache;
    source::MaterialTable physMats; // collision-only materials, kept out of the model's texture table
    bool dataModelPhysics = false;  // a $datamodelphysics supplied the collision shapes
    int shapelessPhysicsModelLine = 0; // $physicsmodel with no $physicsshape, warned at the end
    std::map<std::string, int> namedAnims; // $animation name -> index into in.anims
    std::map<std::string, std::vector<Token>> cmdlists; // $cmdlist name -> body tokens
    std::map<std::string, std::vector<Token>> lodCmdlists; // $lodcmdlist, keyed lowercased
    std::map<std::string, std::vector<Token>> rendermeshCmdlists; // $rendermeshcmdlist, keyed lowercased
    struct Variable {
        std::string text;
        std::vector<Token> tokens;
        bool list = false;
    };
    std::map<std::string, Variable> variables; // case-sensitive, shared across $include
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
        pm::matrix3x4 inheritedLocal;
        bool hasInheritedLocal = false;
        bool hasOriginOverride = false;
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
        if (!err) return false;

        const pulse::diag::Location at = pulse::diag::Resolve({Path(), line});
        *err = Where(line) + ": " + msg;
        pulse::diag::NoteFailure(*err, at, msg);
        return false;
    }

    // "file(line)", or "file: <node>" for a labelled .mdlasm node.
    std::string Where(int line) const {
        if (!nodeLabels.empty() && line >= 0 && static_cast<size_t>(line) < nodeLabels.size())
            return file + ": " + nodeLabels[static_cast<size_t>(line)];
        return file + "(" + std::to_string(line) + ")";
    }

    // Absolute path of the file being parsed.
    std::string Path() const { return pulse::Utf8(fs::absolute(curDir / file).lexically_normal()); }

    // Analyze: the token at `at` is an option word of the running command, so it can be told
    // apart from values. Sent when the command ends; DropOption undoes a mark that turned out a value.
    void MarkOption(size_t at) {
        if (!keepGoing || at >= toks.size()) return;
        const Token& t = toks[at];
        if (t.quoted || t.expanded || t.apiSource < 0 || t.text.empty()) return;
        const unsigned char first = static_cast<unsigned char>(t.text[0]);
        const bool negated = first == '-' && t.text.size() > 1 &&
            std::isalpha(static_cast<unsigned char>(t.text[1]));
        if (std::isalpha(first) || first == '_' || negated)
            options.push_back(t);
    }
    void MarkOption() { MarkOption(pos - 1); }
    void DropOption(const Token& t) {
        if (!options.empty() && options.back().line == t.line && options.back().column == t.column)
            options.pop_back();
    }

    // A failed command's last option word may be the one it rejected, so that one is left out.
    void ReportOptions(bool succeeded) {
        if (!succeeded && !options.empty()) options.pop_back();
        for (const Token& t : options)
            pulse::api::Record("option", "{\"file\":" + pulse::api::String(apiSources[static_cast<size_t>(t.apiSource)]) +
                ",\"line\":" + std::to_string(t.line) + ",\"column\":" + std::to_string(t.column) +
                ",\"text\":" + pulse::api::String(t.text) + "}");
        options.clear();
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

    // A required word from a fixed set, such as a mode name.
    bool WantOption(const char* what, const Token& cmd, std::string& value) {
        if (!Want(what, cmd, value)) return false;
        MarkOption();
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

} // namespace detail
} // namespace pulse::loader

#endif // PULSEMODEL_SCRIPTCTX_H
