// flexreg.cpp - flex / morph registration. See flexreg.h.
//
// Ports of the reference's flex tables: Add_Flexdesc,
// FindOrAddFlexController, the AddCombination normal path,
// AddBodyFlexData/AddFlexControllers/AddBodyFlexRemaps/AddBodyFlexRules,
// AddBodyFlexRule + AddBodyFlexFetchRule and the Option_Flexrule
// shunting-yard parser.
//
// Registration ORDER defines flexdesc/controller/rule indices and the string
// table - it mirrors the QC: per body (bodygroup order): [first use of the
// source: AddCombination] -> AddBodyFlexData -> [last morphed body only: the
// ManualFlex block, standing in for the manual $model flexcontroller/%rule
// block] -> AddBodyFlexRules.

#include "flexreg.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace pulse::loader {

namespace cm = pulse::compile;
namespace lim = pulse::limits;

// reference Add_Flexdesc: find (stricmp) or append
int AddFlexdesc(cm::CompileInput& in, const std::string& name, std::string* err) {
    for (size_t i = 0; i < in.flexdescs.size(); ++i)
        if (_stricmp(in.flexdescs[i].name.c_str(), name.c_str()) == 0)
            return static_cast<int>(i);
    if (in.flexdescs.size() >= static_cast<size_t>(lim::kMaxFlexDesc)) {
        if (err) *err = "too many flex types, max " + std::to_string(lim::kMaxFlexDesc);
        return -1;
    }
    in.flexdescs.push_back({name});
    return static_cast<int>(in.flexdescs.size()) - 1;
}

namespace {

// flex rule opcodes (format/mdl.h values, mirrored to avoid the header dep)
constexpr int kOpConst = 1;
constexpr int kOpFetch1 = 2;
constexpr int kOpFetch2 = 3;
constexpr int kOpAdd = 4;
constexpr int kOpSub = 5;
constexpr int kOpMul = 6;
constexpr int kOpDiv = 7;
constexpr int kOpNeg = 8;
constexpr int kOpExp = 9;
constexpr int kOpOpen = 10;
constexpr int kOpClose = 11;
constexpr int kOpComma = 12;
constexpr int kOpMax = 13;
constexpr int kOpMin = 14;
constexpr int kOp2Way0 = 15;
constexpr int kOp2Way1 = 16;
constexpr int kOpNWay = 17;
constexpr int kOpCombo = 18;
constexpr int kOpDominate = 19;
constexpr int kOpDmeLowerEyelid = 20;
constexpr int kOpDmeUpperEyelid = 21;

int FindFlexControllerByName(const cm::CompileInput& in, const char* name) {
    for (size_t i = 0; i < in.flexcontrollers.size(); ++i)
        if (_stricmp(in.flexcontrollers[i].name.c_str(), name) == 0)
            return static_cast<int>(i);
    return -1;
}

int FindFlexdescByName(const cm::CompileInput& in, const char* name) {
    for (size_t i = 0; i < in.flexdescs.size(); ++i)
        if (_stricmp(in.flexdescs[i].name.c_str(), name) == 0)
            return static_cast<int>(i);
    return -1;
}

// one line per registered rule, column-aligned with the flex controller lines
void PrintFlexRule(const char* kind, const std::string& name, const std::string& detail) {
    std::printf("flex %-12s%-32s %s\n", kind, name.c_str(), detail.c_str());
}

std::string JoinNames(const std::vector<std::string>& names, const char* sep) {
    std::string out;
    for (const std::string& s : names) {
        if (!out.empty()) out += sep;
        out += s;
    }
    return out;
}

// reference FindOrAddFlexController - exact-name find
// (strcmp) with a mismatch warning; used by the AddCombination path
void FindOrAddFlexController(cm::CompileInput& in, const std::string& name,
                             const std::string& type, float flMin, float flMax) {
    for (const cm::FlexController& fc : in.flexcontrollers) {
        if (strcmp(fc.name.c_str(), name.c_str()) == 0) {
            if (strcmp(fc.type.c_str(), type.c_str()) != 0 || fc.min != flMin ||
                fc.max != flMax) {
                std::fprintf(stderr,
                             "warning: flex controller %s defined twice with different params: "
                             "%s, %f %f vs %s, %f %f\n",
                             name.c_str(), type.c_str(), flMin, flMax, fc.type.c_str(), fc.min,
                             fc.max);
            }
            return;
        }
    }
    cm::FlexController fc;
    fc.name = name;
    fc.type = type;
    fc.min = flMin;
    fc.max = flMax;
    in.flexcontrollers.push_back(std::move(fc));
    std::printf("flex controller  %-32s [%s] %.3f..%.3f\n", name.c_str(), type.c_str(), flMin,
                flMax);
}

// GetExprToken tokenizer: identifiers are [alpha_][alnum_]*,
// numbers are [digit.]+, everything else is a single-char token.
struct ExprLexer {
    const char* p;
    explicit ExprLexer(const char* s) : p(s) {}
    // returns false at end of string
    bool Next(std::string& tok) {
        tok.clear();
        while (*p && static_cast<unsigned char>(*p) <= 32)
            ++p;
        if (!*p)
            return false;
        if (isalpha(static_cast<unsigned char>(*p)) || *p == '_') {
            while (isalnum(static_cast<unsigned char>(*p)) || *p == '_')
                tok.push_back(*p++);
        } else if (isdigit(static_cast<unsigned char>(*p)) || *p == '.') {
            while (isdigit(static_cast<unsigned char>(*p)) || *p == '.')
                tok.push_back(*p++);
        } else {
            tok.push_back(*p++);
        }
        return true;
    }
};

// reference Option_Flexrule: infix -> RPN via
// shunting-yard. One rule per flexdesc - a later rule for the same desc is
// skipped (mirrors the LOD/multi-body dedup).
bool ParseMorphRuleExpr(cm::CompileInput& in, const std::string& flexName,
                        const std::string& expr, std::string* err) {
    static const int precedence[32] = {
        0, /*CONST*/ 0, /*FETCH1*/ 0, /*FETCH2*/ 0,
        /*ADD*/ 1, /*SUB*/ 1, /*MUL*/ 2, /*DIV*/ 2,
        /*NEG*/ 4, /*EXP*/ 3, /*OPEN*/ 0, /*CLOSE*/ 0,
        /*COMMA*/ 0, /*MAX*/ 5, /*MIN*/ 5,
    };

    const int flexdesc = FindFlexdescByName(in, flexName.c_str());
    if (flexdesc < 0) {
        if (err) *err = "morph rule for unknown morph \"" + flexName + "\"";
        return false;
    }

    // dedup per desc (the reference drains the expression tokens; a string
    // input can simply be skipped)
    for (const cm::FlexRule& r : in.flexrules)
        if (r.flex == flexdesc) {
            std::fprintf(stderr, "warning: morph \"%s\" already has a rule, ignoring\n",
                         flexName.c_str());
            return true;
        }

    if (in.flexrules.size() >= static_cast<size_t>(lim::kMaxFlexRules)) {
        if (err) *err = "too many flex rules (max " + std::to_string(lim::kMaxFlexRules) + ")";
        return false;
    }

    std::vector<cm::FlexOp> stream;
    ExprLexer lex(expr.c_str());
    std::string tok;
    while (lex.Next(tok)) {
        cm::FlexOp op{};
        if (tok[0] == '(') {
            op.op = kOpOpen;
        } else if (tok[0] == ')') {
            op.op = kOpClose;
        } else if (tok[0] == '+') {
            op.op = kOpAdd;
        } else if (tok[0] == '-') {
            op.op = kOpSub;
            if (!stream.empty()) {
                switch (stream.back().op) {
                    case kOpOpen:
                    case kOpAdd:
                    case kOpSub:
                    case kOpMul:
                    case kOpDiv:
                    case kOpComma:
                        // unary if preceded by "(+-*/,"
                        op.op = kOpNeg;
                        break;
                }
            }
        } else if (tok[0] == '*') {
            op.op = kOpMul;
        } else if (tok[0] == '/') {
            op.op = kOpDiv;
        } else if (isdigit(static_cast<unsigned char>(tok[0]))) {
            op.op = kOpConst;
            op.d.value = static_cast<float>(atof(tok.c_str()));
        } else if (tok[0] == ',') {
            op.op = kOpComma;
        } else if (_stricmp(tok.c_str(), "max") == 0) {
            op.op = kOpMax;
        } else if (_stricmp(tok.c_str(), "min") == 0) {
            op.op = kOpMin;
        } else if (tok[0] == '%') {
            if (!lex.Next(tok)) {
                if (err) *err = "morph rule \"" + flexName + "\": dangling %";
                return false;
            }
            const int k = FindFlexdescByName(in, tok.c_str());
            if (k < 0) {
                if (err) *err = "morph rule \"" + flexName + "\": unknown flex %" + tok;
                return false;
            }
            op.op = kOpFetch2;
            op.d.index = k;
        } else {
            const int k = FindFlexControllerByName(in, tok.c_str());
            if (k < 0) {
                if (err) *err = "morph rule \"" + flexName + "\": unknown controller " + tok;
                return false;
            }
            op.op = kOpFetch1;
            op.d.index = k;
        }
        stream.push_back(op);
        if (stream.size() > static_cast<size_t>(lim::kMaxFlexOps)) {
            if (err) *err = "morph rule expression for \"" + flexName + "\" too complicated";
            return false;
        }
    }

    cm::FlexRule rule;
    rule.flex = flexdesc;

    std::vector<cm::FlexOp> stack;
    for (size_t k = 0; k < stream.size(); ++k) {
        if (stack.size() >= static_cast<size_t>(lim::kMaxFlexOps)) {
            if (err) *err = "morph rule expression for \"" + flexName + "\" too complicated";
            return false;
        }
        switch (stream[k].op) {
            case kOpConst:
            case kOpFetch1:
            case kOpFetch2:
                rule.ops.push_back(stream[k]);
                break;
            case kOpOpen:
                stack.push_back(stream[k]);
                break;
            case kOpClose:
                while (!stack.empty() && stack.back().op != kOpOpen) {
                    rule.ops.push_back(stack.back());
                    stack.pop_back();
                }
                if (stack.empty()) {
                    if (err) *err = "morph rule \"" + flexName + "\": unmatched )";
                    return false;
                }
                stack.pop_back();
                break;
            case kOpComma:
                while (!stack.empty() && stack.back().op != kOpOpen) {
                    rule.ops.push_back(stack.back());
                    stack.pop_back();
                }
                stack.push_back(stream[k]);
                break;
            case kOpAdd:
            case kOpSub:
            case kOpMul:
            case kOpDiv:
                while (!stack.empty() &&
                       precedence[stream[k].op] <= precedence[stack.back().op]) {
                    rule.ops.push_back(stack.back());
                    stack.pop_back();
                }
                stack.push_back(stream[k]);
                break;
            case kOpNeg:
                if (k + 1 < stream.size() && stream[k + 1].op == kOpConst) {
                    // fold the sign into the constant, no op emitted
                    stream[k + 1].d.value = -stream[k + 1].d.value;
                } else {
                    stack.push_back(stream[k]);
                }
                break;
            case kOpMax:
            case kOpMin:
                stack.push_back(stream[k]);
                break;
        }
        if (rule.ops.size() >= static_cast<size_t>(lim::kMaxFlexOps)) {
            if (err) *err = "morph rule expression for \"" + flexName + "\" too complicated";
            return false;
        }
    }
    while (!stack.empty()) {
        rule.ops.push_back(stack.back());
        stack.pop_back();
        if (rule.ops.size() >= static_cast<size_t>(lim::kMaxFlexOps)) {
            if (err) *err = "morph rule expression for \"" + flexName + "\" too complicated";
            return false;
        }
    }

    // reprocess the operands, eating commas for min/max
    int numCommas = 0;
    size_t j = 0;
    for (size_t k = 0; k < rule.ops.size(); ++k) {
        switch (rule.ops[k].op) {
            case kOpMax:
            case kOpMin:
                if (j == 0 || rule.ops[j - 1].op != kOpComma) {
                    if (err) *err = "morph rule \"" + flexName + "\": missing comma";
                    return false;
                }
                numCommas--;
                rule.ops[j - 1] = rule.ops[k];
                break;
            case kOpComma:
                numCommas++;
                rule.ops[j++] = rule.ops[k];
                break;
            default:
                rule.ops[j++] = rule.ops[k];
                break;
        }
    }
    rule.ops.resize(j);
    if (numCommas != 0) {
        if (err) *err = "morph rule \"" + flexName + "\": too many commas";
        return false;
    }

    in.flexrules.push_back(std::move(rule));
    PrintFlexRule("rule", flexName, "= " + expr);
    return true;
}

// resolve one dominator / corrective operand: bare name -> a flex controller
// (FETCH1), %name -> a flexdesc (FETCH2). Same convention as `expr`.
bool ResolveFetch(const cm::CompileInput& in, const std::string& where,
                  const std::string& operand, cm::FlexOp& out, std::string* err) {
    if (operand.empty()) {
        if (err) *err = where + ": empty operand";
        return false;
    }
    if (operand[0] == '%') {
        const std::string descName = operand.substr(1);
        const int k = FindFlexdescByName(in, descName.c_str());
        if (k < 0) {
            if (err) *err = where + ": unknown morph %" + descName;
            return false;
        }
        out.op = kOpFetch2;
        out.d.index = k;
        return true;
    }
    const int k = FindFlexControllerByName(in, operand.c_str());
    if (k < 0) {
        if (err) *err = where + ": unknown controller " + operand;
        return false;
    }
    out.op = kOpFetch1;
    out.d.index = k;
    return true;
}

// $flexcorrective / a hand-authored combination rule: the same op shape
// AddBodyFlexRuleForKey builds from a DMX combination operator - one fetch per
// control, then COMBO n. One rule per flexdesc, first registered wins.
bool RegisterCorrective(cm::CompileInput& in, const ManualFlex::Rule& r, std::string* err) {
    const std::string where = "morph corrective \"" + r.name + "\"";

    const int flexdesc = FindFlexdescByName(in, r.name.c_str());
    if (flexdesc < 0) {
        if (err) *err = where + ": no such morph";
        return false;
    }
    for (const cm::FlexRule& existing : in.flexrules) {
        if (existing.flex == flexdesc) {
            std::fprintf(stderr, "warning: %s already has a rule, ignoring\n", where.c_str());
            return true;
        }
    }
    if (in.flexrules.size() >= static_cast<size_t>(lim::kMaxFlexRules)) {
        if (err) *err = "too many flex rules, max " + std::to_string(lim::kMaxFlexRules);
        return false;
    }

    cm::FlexRule rule;
    rule.flex = flexdesc;
    for (const std::string& operand : r.combo) {
        cm::FlexOp op{};
        if (!ResolveFetch(in, where, operand, op, err))
            return false;
        rule.ops.push_back(op);
    }
    cm::FlexOp combo{};
    combo.op = kOpCombo;
    combo.d.index = static_cast<int>(r.combo.size());
    rule.ops.push_back(combo);

    if (rule.ops.size() > static_cast<size_t>(lim::kMaxFlexOps)) {
        if (err) *err = where + ": too many controls";
        return false;
    }
    in.flexrules.push_back(std::move(rule));
    PrintFlexRule("corrective", r.name, "= " + JoinNames(r.combo, " + "));
    return true;
}

// reference AddCombination normal path: runs once per
// source that carries DmeFlexRules - controllers from the combo controls,
// flexdescs for every rule name, %name delta registration, then the
// expression/passthrough rules.
bool RegisterSourceCombination(cm::CompileInput& in, source::Source* src, std::string* err) {
    if (!src->hasDmeFlexRules)
        return true;

    // controllers from the combination controls; flexgroup fallback here is
    // "default" (AddCombination), NOT the control name
    for (const source::ControllerRemap& remap : src->controllerRemaps) {
        float flMin = remap.hasMinMax ? remap.min : 0.0f;
        float flMax = remap.hasMinMax ? remap.max : 1.0f;
        std::string group = remap.flexgroup.empty() ? "default" : remap.flexgroup;
        if (remap.stereo) {
            FindOrAddFlexController(in, "right_" + remap.name, group, flMin, flMax);
            FindOrAddFlexController(in, "left_" + remap.name, group, flMin, flMax);
        } else {
            FindOrAddFlexController(in, remap.name, group, flMin, flMax);
        }
    }

    // pass 1: a flexdesc for every rule name (localvars included) so
    // %cross-references between rules resolve
    for (const source::SrcFlexRule& rule : src->dmeFlexRules)
        if (AddFlexdesc(in, rule.name, err) < 0)
            return false;

    // pass 2: register %name-referenced delta shapes that exist in the source -
    // DMX-internal deltas become fetchable flexdescs
    for (const source::SrcFlexRule& rule : src->dmeFlexRules) {
        if (rule.isLocalVar || rule.expr.empty())
            continue;
        const char* p = rule.expr.c_str();
        while (*p) {
            if (*p++ != '%')
                continue;
            const char* start = p;
            while (*p && (isalnum(static_cast<unsigned char>(*p)) || *p == '_'))
                ++p;
            if (p == start)
                continue;
            std::string name(start, p);

            bool bIsDelta = false;
            for (const source::SrcFlexKey& key : src->flexkeys)
                if (_stricmp(key.name.c_str(), name.c_str()) == 0) { bIsDelta = true; break; }
            if (bIsDelta) {
                if (AddFlexdesc(in, name, err) < 0)
                    return false;
                continue;
            }
            bool bIsLocalVar = false;
            for (const source::SrcFlexRule& other : src->dmeFlexRules)
                if (_stricmp(other.name.c_str(), name.c_str()) == 0) { bIsLocalVar = true; break; }
            if (bIsLocalVar)
                continue;
            bool bKnown = false;
            for (const cm::FlexDesc& fd : in.flexdescs)
                if (_stricmp(fd.name.c_str(), name.c_str()) == 0) { bKnown = true; break; }
            std::fprintf(stderr,
                         bKnown ? "warning: DMX flex rule '%s': %%%s resolved from the script, "
                                  "not from DMX delta shapes\n"
                                : "warning: DMX flex rule '%s': %%%s not found in DMX delta "
                                  "shapes or the script - compile will fail\n",
                         rule.name.c_str(), name.c_str());
        }
    }

    // pass 3: emit the expression/passthrough rules (localvar = desc only)
    for (const source::SrcFlexRule& rule : src->dmeFlexRules) {
        if (rule.isLocalVar)
            continue;
        if (!ParseMorphRuleExpr(in, rule.name, rule.expr, err))
            return false;
    }
    return true;
}

// reference AddFlexControllers: resolve the
// source's remaps to global controllers, creating right_/left_, multi_ and
// blink controllers as needed; fills the raw->remap and remap->global maps.
void AddFlexControllersForSource(cm::CompileInput& in, source::Source* src) {
    std::vector<int>& r2s = src->rawToRemapSource;
    std::vector<int>& r2l = src->rawToRemapLocal;
    std::vector<int>& l2i = src->leftRemapToGlobal;
    std::vector<int>& r2i = src->rightRemapToGlobal;

    const int nRawControlCount = static_cast<int>(src->combinationControls.size());
    r2s.assign(nRawControlCount, -1);
    r2l.assign(nRawControlCount, -1);

    const int nRemappedControlCount = static_cast<int>(src->controllerRemaps.size());
    l2i.assign(nRemappedControlCount, -1);
    r2i.assign(nRemappedControlCount, -1);

    for (int i = 0; i < nRemappedControlCount; ++i) {
        source::ControllerRemap& remap = src->controllerRemaps[i];

        // raw -> (remap, local) map; the name match is case-SENSITIVE like
        // the reference's CUtlString ==
        for (size_t jj = 0; jj < remap.rawControls.size(); ++jj) {
            for (int k = 0; k < nRawControlCount; ++k) {
                if (remap.rawControls[jj] == src->combinationControls[k].name) {
                    r2s[k] = i;
                    r2l[k] = static_cast<int>(jj);
                    break;
                }
            }
        }

        // controller min/max: DMX-specified range wins; 2WAY/EYELID default
        // to -1..1, everything else 0..1
        auto controllerMin = [&remap]() {
            if (remap.hasMinMax) return remap.min;
            return (remap.type == source::RemapType::TwoWay ||
                    remap.type == source::RemapType::Eyelid) ? -1.0f : 0.0f;
        };
        auto controllerMax = [&remap]() {
            if (remap.hasMinMax) return remap.max;
            return 1.0f;
        };
        // type falls back to the controller NAME when flexgroup is empty
        // (unlike the AddCombination path's "default")
        auto controllerType = [&remap](const std::string& name) {
            return remap.flexgroup.empty() ? name : remap.flexgroup;
        };

        auto addController = [&](const std::string& name) -> int {
            int existing = FindFlexControllerByName(in, name.c_str());
            if (existing >= 0)
                return existing;
            cm::FlexController fc;
            fc.name = name;
            fc.type = controllerType(name);
            fc.min = controllerMin();
            fc.max = controllerMax();
            in.flexcontrollers.push_back(fc);
            std::printf("flex controller  %-32s [%s] %.3f..%.3f\n", fc.name.c_str(),
                        fc.type.c_str(), fc.min, fc.max);
            return static_cast<int>(in.flexcontrollers.size()) - 1;
        };

        if (remap.stereo) {
            remap.rightIndex = addController("right_" + remap.name);
            r2i[i] = remap.rightIndex;
            remap.leftIndex = addController("left_" + remap.name);
            l2i[i] = remap.leftIndex;
        } else {
            remap.index = addController(remap.name);
            r2i[i] = remap.index;
            l2i[i] = remap.index;
        }

        if (remap.type == source::RemapType::NWay || remap.type == source::RemapType::Eyelid) {
            std::string multiName = "multi_" + remap.name;
            int existing = FindFlexControllerByName(in, multiName.c_str());
            if (existing >= 0) {
                remap.multiIndex = existing;
            } else {
                cm::FlexController fc;
                fc.name = multiName;
                fc.type = multiName;
                fc.min = -1.0f;
                fc.max = 1.0f;
                in.flexcontrollers.push_back(fc);
                std::printf("flex controller  %-32s [%s] %.3f..%.3f\n", fc.name.c_str(),
                            fc.type.c_str(), fc.min, fc.max);
                remap.multiIndex = static_cast<int>(in.flexcontrollers.size()) - 1;
            }
        }

        if (remap.type == source::RemapType::Eyelid) {
            int existing = FindFlexControllerByName(in, "blink");
            if (existing >= 0) {
                remap.blinkController = existing;
            } else {
                cm::FlexController fc;
                fc.name = "blink";
                fc.type = "blink";
                fc.min = 0.0f;
                fc.max = 1.0f;
                in.flexcontrollers.push_back(fc);
                std::printf("flex controller  %-32s [%s] %.3f..%.3f\n", fc.name.c_str(),
                            fc.type.c_str(), fc.min, fc.max);
                remap.blinkController = static_cast<int>(in.flexcontrollers.size()) - 1;
            }
        }
    }
}

// reference AddBodyFlexData
bool AddBodyFlexData(cm::CompileInput& in, source::Source* src, int imodel, std::string* err) {
    src->keyStartIndex = static_cast<int>(in.flexkeys.size());

    for (const source::SrcFlexKey& key : src->flexkeys) {
        if (in.flexkeys.size() >= static_cast<size_t>(lim::kMaxFlexKeys)) {
            if (err) *err = "too many flex keys, max " + std::to_string(lim::kMaxFlexKeys) +
                            ", cannot add " + key.name + " from " + src->filename;
            return false;
        }
        cm::FlexKey fk;
        fk.source = src;
        fk.animationname = key.name;
        fk.imodel = imodel;
        fk.target1 = key.target1;
        if (key.stereo) {
            int descL = AddFlexdesc(in, key.name + "L", err);
            int descR = AddFlexdesc(in, key.name + "R", err);
            if (descL < 0 || descR < 0)
                return false;
            fk.flexdesc = descL;
            fk.flexpair = descR;
        } else {
            int desc = AddFlexdesc(in, key.name, err);
            if (desc < 0)
                return false;
            fk.flexdesc = desc;
            fk.flexpair = 0;
        }
        in.flexkeys.push_back(std::move(fk));
    }

    AddFlexControllersForSource(in, src);

    // AddBodyFlexRemaps: append to the global list for flexcontrollerui
    for (const source::ControllerRemap& remap : src->controllerRemaps)
        in.flexControllerRemaps.push_back(remap);

    return true;
}

// reference AddBodyFlexFetchRule
void AddBodyFlexFetchRule(source::Source* src, cm::FlexRule& rule, int rawIndex,
                          const std::vector<int>& remapToGlobal) {
    const int remapSourceIndex = src->rawToRemapSource[rawIndex];
    const int remapLocalIndex = src->rawToRemapLocal[rawIndex];
    const int globalIndex = remapToGlobal[remapSourceIndex];
    const source::ControllerRemap& remap = src->controllerRemaps[remapSourceIndex];

    auto pushOp = [&rule](int op, int index) {
        cm::FlexOp o{};
        o.op = op;
        o.d.index = index;
        rule.ops.push_back(o);
    };
    auto pushConst = [&rule](float value) {
        cm::FlexOp o{};
        o.op = kOpConst;
        o.d.value = value;
        rule.ops.push_back(o);
    };

    switch (remap.type) {
        case source::RemapType::PassThru:
            pushOp(kOpFetch1, globalIndex);
            break;

        case source::RemapType::Eyelid:
            pushConst(remap.eyesUpDownFlexController >= 0
                          ? static_cast<float>(remap.eyesUpDownFlexController) : -1.0f);
            pushConst(remap.blinkController >= 0
                          ? static_cast<float>(remap.blinkController) : -1.0f);
            pushConst(static_cast<float>(globalIndex)); // CloseLid
            pushOp(remapLocalIndex == 0 ? kOpDmeLowerEyelid : kOpDmeUpperEyelid,
                   remap.multiIndex); // CloseLidV
            break;

        case source::RemapType::TwoWay:
            pushOp(remapLocalIndex == 0 ? kOp2Way0 : kOp2Way1, globalIndex);
            break;

        case source::RemapType::NWay: {
            int nRemapCount = static_cast<int>(remap.rawControls.size());
            float flStep = (nRemapCount > 2) ? 2.0f / (nRemapCount - 1) : 0.0f;
            if (remapLocalIndex == 0) {
                pushConst(-11.0f);
                pushConst(-10.0f);
                pushConst(-1.0f);
                pushConst(-1.0f + flStep);
            } else if (remapLocalIndex == nRemapCount - 1) {
                pushConst(1.0f - flStep);
                pushConst(1.0f);
                pushConst(10.0f);
                pushConst(11.0f);
            } else {
                float flPeak = remapLocalIndex * flStep - 1.0f;
                pushConst(flPeak - flStep);
                pushConst(flPeak);
                pushConst(flPeak);
                pushConst(flPeak + flStep);
            }
            pushConst(static_cast<float>(remap.multiIndex));
            pushOp(kOpNWay, globalIndex);
            break;
        }
    }
}

// reference AddBodyFlexRule
bool AddBodyFlexRuleForKey(cm::CompileInput& in, source::Source* src,
                           const source::CombinationRule& srcRule, int nFlexDesc,
                           const std::vector<int>& remapToGlobal, std::string* err) {
    for (const cm::FlexRule& r : in.flexrules)
        if (r.flex == nFlexDesc)
            return true; // one rule per desc

    if (in.flexrules.size() >= static_cast<size_t>(lim::kMaxFlexRules)) {
        if (err) *err = "too many flex rules, max " + std::to_string(lim::kMaxFlexRules);
        return false;
    }

    cm::FlexRule rule;
    rule.flex = nFlexDesc;

    const std::string& descName = in.flexdescs[nFlexDesc].name;
    auto rawNames = [src](const std::vector<int>& raws) {
        std::vector<std::string> names;
        for (int raw : raws)
            names.push_back(src->combinationControls[raw].name);
        return names;
    };

    const int nCombinationCount = static_cast<int>(srcRule.combination.size());
    for (int j = 0; j < nCombinationCount; ++j)
        AddBodyFlexFetchRule(src, rule, srcRule.combination[j], remapToGlobal);
    if (nCombinationCount > 1) {
        cm::FlexOp o{};
        o.op = kOpCombo;
        o.d.index = nCombinationCount;
        rule.ops.push_back(o);
        PrintFlexRule("corrective", descName,
                      "= " + JoinNames(rawNames(srcRule.combination), " + "));
    }

    for (const std::vector<int>& dominator : srcRule.dominators) {
        if (dominator.empty())
            continue;
        for (int raw : dominator)
            AddBodyFlexFetchRule(src, rule, raw, remapToGlobal);
        cm::FlexOp o{};
        o.op = kOpDominate;
        o.d.index = static_cast<int>(dominator.size());
        rule.ops.push_back(o);
        PrintFlexRule("dominator", descName,
                      "suppressed by " + JoinNames(rawNames(dominator), ", "));
    }

    in.flexrules.push_back(std::move(rule));
    return true;
}

// reference AddBodyFlexRules
bool AddBodyFlexRulesForSource(cm::CompileInput& in, source::Source* src, std::string* err) {
    // resolve the EYELID remaps' eyes-updown controller (exact-name match)
    for (source::ControllerRemap& remap : src->controllerRemaps) {
        if (remap.type == source::RemapType::Eyelid && !remap.eyesUpDownFlexName.empty()) {
            for (size_t j = 0; j < in.flexcontrollers.size(); ++j) {
                if (strcmp(in.flexcontrollers[j].name.c_str(),
                           remap.eyesUpDownFlexName.c_str()) == 0) {
                    remap.eyesUpDownFlexController = static_cast<int>(j);
                    break;
                }
            }
        }
    }

    for (const source::CombinationRule& srcRule : src->combinationRules) {
        const cm::FlexKey& flexKey = in.flexkeys[src->keyStartIndex + srcRule.flex];
        if (!AddBodyFlexRuleForKey(in, src, srcRule, flexKey.flexdesc, src->leftRemapToGlobal,
                                   err))
            return false;
        if (flexKey.flexpair != 0) {
            if (!AddBodyFlexRuleForKey(in, src, srcRule, flexKey.flexpair,
                                       src->rightRemapToGlobal, err))
                return false;
        }
    }
    return true;
}

// the ManualFlex block, standing in for a QC $model's manual flex lines:
// controllers first (QC writes flexcontroller lines before %rules), then the
// rules/localvars/correctives in authored order.
bool RegisterManualLists(cm::CompileInput& in, const ManualFlex& manual, std::string* err) {
    for (const ManualFlex::Controller& mc : manual.controllers) {
        // QC Option_Flexcontroller reuses an existing name with a warning and
        // does NOT update it
        if (FindFlexControllerByName(in, mc.name.c_str()) >= 0) {
            std::fprintf(stderr,
                         "warning: morph controller '%s' already defined, reusing existing "
                         "entry\n", mc.name.c_str());
            continue;
        }
        if (in.flexcontrollers.size() >= static_cast<size_t>(lim::kMaxFlexCtrl)) {
            if (err) *err = "too many flex controllers, max " + std::to_string(lim::kMaxFlexCtrl);
            return false;
        }
        cm::FlexController fc;
        fc.name = mc.name;
        fc.type = mc.group;
        fc.min = mc.min;
        fc.max = mc.max;
        in.flexcontrollers.push_back(fc);
        std::printf("flex controller  %-32s [%s] %.3f..%.3f\n", fc.name.c_str(), fc.type.c_str(),
                    fc.min, fc.max);
    }

    for (const ManualFlex::Rule& rule : manual.rules) {
        if (rule.localvar) {
            if (AddFlexdesc(in, rule.name, err) < 0)
                return false;
        } else if (!rule.combo.empty()) {
            if (!RegisterCorrective(in, rule, err))
                return false;
        } else if (!ParseMorphRuleExpr(in, rule.name, rule.expr, err)) {
            return false;
        }
    }
    return true;
}

// reference IsDeltaStateStereo against an
// imported rig: ParseDeltaName splits on '_' and EVERY part must name a raw
// control, then the delta is stereo if any part's input control is stereo.
bool RigDeltaIsStereo(const source::FlexRig& rig, const std::string& deltaName) {
    bool stereo = false;
    size_t start = 0;
    while (start <= deltaName.size()) {
        const size_t end = deltaName.find('_', start);
        const std::string part = (end == std::string::npos)
                                     ? deltaName.substr(start)
                                     : deltaName.substr(start, end - start);
        // first raw-control match in control order, like FindRawControlIndex
        const source::ControllerRemap* owner = nullptr;
        for (const source::ControllerRemap& remap : rig.remaps) {
            for (const std::string& raw : remap.rawControls) {
                if (_stricmp(raw.c_str(), part.c_str()) == 0) { owner = &remap; break; }
            }
            if (owner)
                break;
        }
        if (!owner)
            return false; // ParseDeltaName fails -> no controls, so no side
        stereo = stereo || owner->stereo;
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return stereo;
}

// $datamodelflexes: stamp the imported rig onto every render mesh that carries
// morphs. Correctives arrive keyed by delta-state NAME, so a rig read out of one
// file binds to the flexkeys of another; a corrective naming a morph this mesh
// does not have is simply dropped. A mesh with no morphs contributes no
// flexdescs, so it gets nothing - otherwise its remaps would be registered a
// second time.
void ApplyDataModelFlex(cm::CompileInput& in, const source::FlexRig& rig) {
    if (rig.empty())
        return;
    for (auto& part : in.bodyparts) {
        for (auto& model : part.models) {
            source::Source* src = model.source;
            if (!src || src->flexkeys.empty())
                continue;
            src->combinationControls = rig.controls;
            src->controllerRemaps = rig.remaps;
            src->dmeFlexRules = rig.rules;
            src->hasDmeFlexRules = rig.hasRules;

            // The rig owns the controls, so it owns the L/R delta split too: a
            // delta driven by a stereo control gets the <name>L/<name>R desc
            // pair, matching the left_/right_ controllers the remap creates.
            // The mesh's own combination operator only decides this when no rig
            // was imported (same file -> same answer either way).
            if (!rig.remaps.empty()) {
                for (source::SrcFlexKey& key : src->flexkeys)
                    key.stereo = RigDeltaIsStereo(rig, key.name);
            }

            src->combinationRules.clear();
            for (const source::FlexRig::Corrective& cor : rig.correctives) {
                int flex = -1;
                for (size_t i = 0; i < src->flexkeys.size(); ++i) {
                    if (_stricmp(src->flexkeys[i].name.c_str(), cor.delta.c_str()) == 0) {
                        flex = static_cast<int>(i);
                        break;
                    }
                }
                if (flex < 0)
                    continue;
                source::CombinationRule rule;
                rule.flex = flex;
                rule.combination = cor.combination;
                rule.dominators = cor.dominators;
                src->combinationRules.push_back(std::move(rule));
            }
        }
    }
}

// $morphsplitstereo splitfactor: synthesize the balance a mesh does not carry.
// reference ComputeSideAndScale for a paired flexkey - the
// side comes from the BASE vertex X, smoothstepped across the 2*factor band at
// the midline; a negative factor mirrors it. Unified into one clamped t because
// the reference's two branches are the same formula with flipped bounds.
// OVERWRITES a painted DMX balance for this delta - explicit command wins, the
// caller warns once when the mesh had real balance to lose.
void GenerateBalance(source::Source& src, const std::string& name, float factor) {
    for (source::SrcMorphAnim& morph : src.morphs) {
        if (_stricmp(morph.name.c_str(), name.c_str()) != 0)
            continue;
        for (source::SrcVertAnim& va : morph.vanims) {
            if (va.vertex < 0 || va.vertex >= static_cast<int>(src.vertex.size()))
                continue;
            float t = (factor - src.vertex[va.vertex].position.x) / (2.0f * factor);
            t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
            va.side = 1.0f - (3.0f * t * t - 2.0f * t * t * t);
        }
    }
    src.hasBalanceData = true;
}

// MorphSplitStereo / a manual stereo split: flag the named delta states stereo
// before any registration runs, so AddBodyFlexData emits <name>L/<name>R descs
// and the flexpair. Without a splitfactor no vertex data changes here - only the
// desc pair; the per-vertex `side` (balance) the mesh already carries is written
// regardless of stereo and the engine does the actual L/R blend.
bool ApplyStereoSplits(cm::CompileInput& in, const ManualFlex& manual, std::string* err) {
    // GenerateBalance sets hasBalanceData, so snapshot who painted their own
    // first; the flag is cleared as each source is warned about, once.
    std::vector<char> hadBalance(in.sources.size());
    for (size_t i = 0; i < in.sources.size(); ++i)
        hadBalance[i] = in.sources[i]->hasBalanceData ? 1 : 0;

    for (const ManualFlex::StereoSplit& split : manual.stereoSplits) {
        bool found = false;
        for (size_t i = 0; i < in.sources.size(); ++i) {
            auto& sp = in.sources[i];
            for (source::SrcFlexKey& key : sp->flexkeys) {
                if (_stricmp(key.name.c_str(), split.name.c_str()) != 0)
                    continue;
                found = true;
                if (key.stereo) {
                    std::fprintf(stderr,
                                 "warning: morph stereo split '%s' is already stereo "
                                 "(stereo control), ignoring\n", split.name.c_str());
                } else {
                    key.stereo = true;
                }
                if (split.factor != 0.0f) {
                    if (hadBalance[i]) {
                        hadBalance[i] = 0;
                        std::fprintf(stderr,
                                     "warning: %s: $morphsplitstereo splitfactor overrides the "
                                     "balance data encoded in this mesh\n", sp->filename.c_str());
                    }
                    GenerateBalance(*sp, split.name, split.factor);
                }
            }
        }
        if (!found) {
            if (err) *err = "morph stereo split \"" + split.name +
                            "\" names no morph target in any render mesh";
            return false;
        }
    }
    return true;
}

// per the reference - a stereo split needs per-vertex balance to
// blend against, so without it the L/R descs exist and nothing ever drives them
// apart. Deferred to here because the split can come from the mesh's own
// combination operator, an imported rig, or a manual MorphSplitStereo, and only
// the mesh knows whether it carries balance data. Once per source, and only for
// sources a body actually uses.
void WarnStereoWithoutBalance(source::Source* src) {
    if (src->hasBalanceData)
        return;
    bool bStereo = false;
    for (const source::ControllerRemap& remap : src->controllerRemaps)
        if (remap.stereo) { bStereo = true; break; }
    if (!bStereo) {
        for (const source::SrcFlexKey& key : src->flexkeys)
            if (key.stereo) { bStereo = true; break; }
    }
    if (!bStereo)
        return;
    std::fprintf(stderr,
                 "warning: %s: morphs are split stereo but no balance data is encoded in the "
                 "mesh - left/right flex splitting will not occur\n",
                 src->filename.c_str());
}

// MorphDominationRule: appended once every rule (auto + manual) exists, so the
// ops trail the rule's COMBO exactly like DMX-encoded dominators
// (AddBodyFlexRuleForKey). A stereo morph gets the same ops on both its L and R
// rules - a hand-authored domination has no side.
bool ApplyDominations(cm::CompileInput& in, const ManualFlex& manual, std::string* err) {
    for (const ManualFlex::Domination& dom : manual.dominations) {
        const std::string where = "morph domination rule \"" + dom.name + "\"";

        // resolve the target's flexdesc(s): the morph itself, or its L/R pair
        // when it was split
        std::vector<int> targetDescs;
        for (const std::string& cand : {dom.name, dom.name + "L", dom.name + "R"}) {
            const int d = FindFlexdescByName(in, cand.c_str());
            if (d >= 0)
                targetDescs.push_back(d);
        }
        if (targetDescs.empty()) {
            if (err) *err = where + " names no morph";
            return false;
        }

        // build the dominator ops once - the indices are global
        std::vector<cm::FlexOp> domOps;
        for (const std::string& d : dom.dominators) {
            cm::FlexOp op{};
            if (!ResolveFetch(in, where, d, op, err))
                return false;
            domOps.push_back(op);
        }
        cm::FlexOp dominate{};
        dominate.op = kOpDominate;
        dominate.d.index = static_cast<int>(domOps.size());
        domOps.push_back(dominate);

        bool applied = false;
        for (int desc : targetDescs) {
            for (cm::FlexRule& rule : in.flexrules) {
                if (rule.flex != desc)
                    continue;
                if (rule.ops.size() + domOps.size() > static_cast<size_t>(lim::kMaxFlexOps)) {
                    if (err) *err = where + ": rule too complicated";
                    return false;
                }
                rule.ops.insert(rule.ops.end(), domOps.begin(), domOps.end());
                applied = true;
            }
        }
        if (!applied) {
            // a morph with no rule is driven directly by its controller; there
            // is nothing for a dominator to suppress
            if (err) *err = where + " targets a morph with no rule to suppress";
            return false;
        }
        PrintFlexRule("dominator", dom.name,
                      "suppressed by " + JoinNames(dom.dominators, ", "));
    }
    return true;
}

} // namespace

bool RegisterFlex(cm::CompileInput& in, const ManualFlex& manual, std::string* err) {
    ApplyDataModelFlex(in, manual.datamodel);
    if (!ApplyStereoSplits(in, manual, err))
        return false;

    // the last morphed body in flattened model index space (blanks counted) -
    // where the manual block attaches, matching a QC that writes its flex lines
    // inside the last morphed $model
    int lastMorphed = -1;
    {
        int imodel = 0;
        for (const auto& part : in.bodyparts) {
            for (const auto& model : part.models) {
                if (model.source && !model.source->flexkeys.empty())
                    lastMorphed = imodel;
                ++imodel;
            }
        }
    }

    bool manualDone = false;
    int imodel = 0;
    for (const auto& part : in.bodyparts) {
        for (const auto& model : part.models) {
            source::Source* src = model.source;
            if (src) {
                if (!src->combinationRegistered) {
                    src->combinationRegistered = true;
                    WarnStereoWithoutBalance(src);
                    if (!RegisterSourceCombination(in, src, err))
                        return false;
                }
                if (!AddBodyFlexData(in, src, imodel, err))
                    return false;
                if (getenv("PULSE_FLEXDBG"))
                    std::fprintf(stderr,
                                 "[flexdbg] imodel=%d remaps=%zu flexkeys=%zu "
                                 "descs=%zu controllers=%zu\n",
                                 imodel, src->controllerRemaps.size(), in.flexkeys.size(),
                                 in.flexdescs.size(), in.flexcontrollers.size());
                if (imodel == lastMorphed) {
                    if (!RegisterManualLists(in, manual, err))
                        return false;
                    manualDone = true;
                }
                if (!AddBodyFlexRulesForSource(in, src, err))
                    return false;
            }
            ++imodel;
        }
    }
    // no morphed body at all: register the manual block after the bodies (no
    // auto rules exist, so no ordering interleave is possible)
    if (!manualDone && !RegisterManualLists(in, manual, err))
        return false;

    return ApplyDominations(in, manual, err);
}

} // namespace pulse::loader
