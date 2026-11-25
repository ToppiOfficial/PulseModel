// mdlasmflex.cpp - see mdlasmflex.h.
//
// Each Flex Output or Local Var node becomes one rule of native flex ops, walking the graph from its
// input. The op sequences follow the Model Assembler's emitter node for node.

#include "mdlasmflex.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <optional>
#include <set>
#include <string_view>

#include "diagnostic.h"
#include "flexreg.h"
#include "pulselimits.h"
#include "strcompat.h"

namespace pulse::mdlasm {
namespace {

namespace cm = pulse::compile;
namespace lim = pulse::limits;

// Studio flex op numbers.
namespace flexop {
constexpr int Const = 1, Fetch1 = 2, Fetch2 = 3, Add = 4, Sub = 5, Mul = 6, Div = 7, Neg = 8, Max = 13, Min = 14;
constexpr int TwoWay0 = 15, TwoWay1 = 16, NWay = 17, Combo = 18, Dominate = 19, LowerEyelid = 20, UpperEyelid = 21;
} // namespace flexop

std::string Lowered(std::string_view text) {
    std::string result(text);
    std::ranges::transform(result, result.begin(), [](unsigned char c) { return char(std::tolower(c)); });
    return result;
}

bool SameName(std::string_view a, std::string_view b) { return Lowered(a) == Lowered(b); }

// A name a rule can refer to: [A-Za-z_][A-Za-z0-9_]*, and not min or max.
bool NameUsable(std::string_view name) {
    if (name.empty()) return false;

    const auto first = static_cast<unsigned char>(name.front());
    if (!(std::isalpha(first) || first == '_')) return false;

    const bool plain = std::ranges::all_of(name, [](unsigned char c) { return std::isalnum(c) || c == '_'; });
    const std::string lower = Lowered(name);
    return plain && lower != "min" && lower != "max";
}

// Input and output socket counts of a node type.
struct Sockets {
    int inputs = 0, outputs = 0;
};

Sockets SocketsOf(FlexNodeType type) {
    using Type = FlexNodeType;
    switch (type) {
    case Type::Controller: case Type::Constant: case Type::MorphValue: case Type::LocalVarGet: return {0, 1};
    case Type::Add: case Type::Subtract: case Type::Multiply: case Type::Divide: case Type::Min: case Type::Max:
    case Type::Math: return {2, 1};
    case Type::Negate: case Type::Invert: case Type::Abs: return {1, 1};
    case Type::Clamp: case Type::Mix: return {3, 1};
    case Type::MapRange: return {5, 1};
    case Type::FlexOutput: case Type::LocalVar: return {1, 0};
    case Type::Reroute: return {1, 1};
    case Type::Crossfade: return {3, 2};
    case Type::Combination: case Type::Dominate: return {5, 1};
    case Type::TwoWay: return {1, 2};
    case Type::NWay: return {2, 1};
    case Type::Eyelid: return {3, 2};
    default: return {0, 0};
    }
}

struct Step {
    int op = 0;
    float value = 0.0f;
    std::string name;
    bool controllerIndex = false;
    int count = 0;
};

struct Problem {
    Uid node = 0;
    std::string message;
};

const FlexLink* LinkInto(const Document& document, Uid node, int input) {
    const auto found = std::ranges::find_if(document.flexLinks, [&](const FlexLink& link) {
        return link.to == node && link.toSocket == input;
    });
    return found == document.flexLinks.end() ? nullptr : &*found;
}

// How a step changes the stack depth.
int StackEffect(const cm::FlexOp& step) {
    using namespace flexop;
    switch (step.op) {
    case Const: case Fetch1: case Fetch2: case TwoWay0: case TwoWay1: return 1;
    case Add: case Sub: case Mul: case Div: case Max: case Min: return -1;
    case Combo: return 1 - step.d.index;
    case Dominate: return -step.d.index;
    case NWay: return -4;
    case LowerEyelid: case UpperEyelid: return -2;
    default: return 0;
    }
}

struct Emitter {
    const Document& document;
    std::vector<Problem>& problems;
    std::set<std::string> morphNames;
    // Lowercase names of the controllers and local vars that made it into the rig.
    std::set<std::string> controllers, localvars;
    std::set<Uid> controllerNodes;
    std::set<std::string>* reads = nullptr;
    std::vector<Step>* out = nullptr;
    std::vector<Uid> path;

    void Report(Uid node, std::string message) {
        const bool known = std::ranges::any_of(problems, [&](const Problem& p) { return p.node == node && p.message == message; });
        if (!known) problems.push_back({node, std::move(message)});
    }

    void Op(int code, int count = 0) {
        Step step;
        step.op = code;
        step.count = count;
        out->push_back(std::move(step));
    }

    void Constant(float value) {
        Step step;
        step.op = flexop::Const;
        step.value = value;
        out->push_back(std::move(step));
    }

    // Const holding a controller's index; an empty name is -1, the game's "none".
    void ControllerIndex(const std::string& name) {
        Step step;
        step.op = flexop::Const;
        step.name = name;
        step.controllerIndex = true;
        out->push_back(std::move(step));
    }

    void Named(int code, std::string name) {
        Step step;
        step.op = code;
        step.name = std::move(name);
        out->push_back(std::move(step));
    }

    void Input(const FlexNode& node, int index) {
        const FlexLink* link = LinkInto(document, node.uid, index);
        if (link && document.findFlexNode(link->from)) Value(link->from, link->fromSocket);
        else Constant(node.values[std::size_t(index)]);
    }

    void Value(Uid uid, int socket = 0) {
        const FlexNode* node = document.findFlexNode(uid);
        if (!node) return Constant(0);

        if (std::ranges::find(path, uid) != path.end()) {
            Report(uid, "The graph loops back to this node.");
            return Constant(0);
        }

        path.push_back(uid);
        Build(*node, socket);
        path.pop_back();
    }

    // The controller an input is wired to, directly or through reroutes and muted pass-throughs. Rig ops
    // take a controller, not a computed value.
    std::optional<std::string> ControllerOf(const FlexNode& node, int index) const {
        const FlexLink* link = LinkInto(document, node.uid, index);

        for (int guard = 0; link && guard < 64; ++guard) {
            const FlexNode* source = document.findFlexNode(link->from);
            if (!source) return std::nullopt;

            if (source->type == FlexNodeType::Controller) {
                if (source->muted || !controllerNodes.contains(source->uid)) return std::nullopt;

                return source->stereo ? (link->fromSocket == 1 ? "right_" : "left_") + source->name : source->name;
            }

            const bool passes = source->type == FlexNodeType::Reroute || (source->muted && SocketsOf(source->type).inputs > 0);
            if (!passes) return std::nullopt;

            link = LinkInto(document, source->uid, 0);
        }
        return std::nullopt;
    }

    void Clamp01() {
        Constant(0);
        Op(flexop::Max);
        Constant(1);
        Op(flexop::Min);
    }

    void Build(const FlexNode& node, int socket) {
        using Type = FlexNodeType;
        using namespace flexop;

        if (node.type == Type::Reroute) return LinkInto(document, node.uid, 0) ? Input(node, 0) : Constant(0);

        if (node.muted) {
            const Sockets sockets = SocketsOf(node.type);
            const bool passes = sockets.inputs >= 1 && sockets.outputs > 0 && node.type != Type::Controller;
            return passes ? Input(node, 0) : Constant(0);
        }

        switch (node.type) {
        case Type::Controller:
            if (!controllerNodes.contains(node.uid)) return Constant(0);
            if (!node.stereo) return Named(Fetch1, node.name);

            return Named(Fetch1, (socket == 1 ? "right_" : "left_") + node.name);

        case Type::Constant:
            return Constant(node.values[0]);

        case Type::MorphValue:
            if (!NameUsable(node.name) || !morphNames.contains(Lowered(node.name))) {
                Report(node.uid, node.name.empty() ? "Pick a morph." : "No morph can be read by this name.");
                return Constant(0);
            }
            if (reads) reads->insert(Lowered(node.name));
            return Named(Fetch2, node.name);

        case Type::LocalVarGet:
            if (!localvars.contains(Lowered(node.name))) {
                Report(node.uid, node.name.empty() ? "Pick a local var." : "No local var has this name.");
                return Constant(0);
            }
            if (reads) reads->insert(Lowered(node.name));
            return Named(Fetch2, node.name);

        case Type::Math: {
            static const int operations[] = {Add, Sub, Mul, Div, Min, Max};
            Input(node, 0);
            Input(node, 1);
            return Op(operations[std::clamp(node.mode, 0, 5)]);
        }

        // How far the value is from the first end to the second, 0..1; From is what is left of it.
        case Type::Crossfade:
            if (socket != 1) Constant(1);
            Input(node, 0);
            Input(node, 1);
            Op(Sub);
            Input(node, 2);
            Input(node, 1);
            Op(Sub);
            Op(Div);
            Clamp01();
            if (socket != 1) Op(Sub);
            return;

        case Type::Add: Input(node, 0); Input(node, 1); return Op(Add);
        case Type::Subtract: Input(node, 0); Input(node, 1); return Op(Sub);
        case Type::Multiply: Input(node, 0); Input(node, 1); return Op(Mul);
        case Type::Divide: Input(node, 0); Input(node, 1); return Op(Div);
        case Type::Negate: Input(node, 0); return Op(Neg);
        case Type::Min: Input(node, 0); Input(node, 1); return Op(Min);
        case Type::Max: Input(node, 0); Input(node, 1); return Op(Max);

        case Type::Clamp:
            Input(node, 0);
            Input(node, 1);
            Op(Max);
            Input(node, 2);
            return Op(Min);

        case Type::Invert:
            Constant(1);
            Input(node, 0);
            return Op(Sub);

        case Type::MapRange:
            Input(node, 3);
            Input(node, 0);
            Input(node, 1);
            Op(Sub);
            Input(node, 4);
            Input(node, 3);
            Op(Sub);
            Op(Mul);
            Input(node, 2);
            Input(node, 1);
            Op(Sub);
            Op(Div);
            return Op(Add);

        case Type::Mix:
            Input(node, 1);
            Input(node, 2);
            Input(node, 1);
            Op(Sub);
            Input(node, 0);
            Op(Mul);
            return Op(Add);

        case Type::Abs:
            Input(node, 0);
            Constant(0);
            Input(node, 0);
            Op(Sub);
            return Op(Max);

        // COMBO: the product of its operands. An unlinked input counts only when it is not the neutral 1.
        case Type::Combination: {
            std::vector<int> used;
            for (int i = 0; i < int(node.values.size()); ++i)
                if (LinkInto(document, node.uid, i) || node.values[std::size_t(i)] != 1.0f) used.push_back(i);

            if (used.empty()) return Constant(1);

            for (const int i : used) Input(node, i);
            if (used.size() > 1) Op(Combo, int(used.size()));
            return;
        }

        // DOMINATE: the value falls as the product of the linked dominators rises.
        case Type::Dominate: {
            Input(node, 0);

            int dominators = 0;
            for (int i = 1; i < int(node.values.size()); ++i) {
                if (!LinkInto(document, node.uid, i)) continue;

                Input(node, i);
                ++dominators;
            }
            if (dominators) Op(Dominate, dominators);
            return;
        }

        case Type::TwoWay: {
            const auto controller = ControllerOf(node, 0);
            if (!controller) {
                Report(node.uid, "Link a Controller to Value.");
                return Constant(0);
            }
            return Named(socket == 1 ? TwoWay1 : TwoWay0, *controller);
        }

        // NWAY: the control scaled by a 0..1 ramp of the position controller, peaking at this index of the count.
        case Type::NWay: {
            const int count = int(std::lround(node.values[3])), index = int(std::lround(node.values[2]));
            if (count < 3 || index < 0 || index >= count) {
                Report(node.uid, count < 3 ? "Count must be 3 or more." : "Index must be 0 to count - 1.");
                return Constant(0);
            }

            const auto control = ControllerOf(node, 0), position = ControllerOf(node, 1);
            if (!control || !position) {
                Report(node.uid, "Link a Controller to Control and to Position.");
                return Constant(0);
            }

            const float step = 2.0f / float(count - 1), peak = float(index) * step - 1.0f;
            if (index == 0) {
                for (const float edge : {-11.0f, -10.0f, -1.0f, -1.0f + step}) Constant(edge);
            } else if (index == count - 1) {
                for (const float edge : {1.0f - step, 1.0f, 10.0f, 11.0f}) Constant(edge);
            } else {
                for (const float edge : {peak - step, peak, peak, peak + step}) Constant(edge);
            }

            ControllerIndex(*position);
            return Named(NWay, *control);
        }

        // DME lower and upper eyelid. Inputs: gaze (optional), 0 lower to 1 upper lid, close amount. A lid's weight
        // is its share of the closing, reduced when the gaze moves away from it.
        case Type::Eyelid: {
            const auto closeV = ControllerOf(node, 1), close = ControllerOf(node, 2);
            const auto eyes = ControllerOf(node, 0);
            if (!closeV || !close) {
                Report(node.uid, "Link a Controller to Lid Lower/Upper and to Close Amount.");
                return Constant(0);
            }
            if (!eyes && LinkInto(document, node.uid, 0)) {
                Report(node.uid, "Link a Controller to Gaze Up/Down, or leave it unlinked.");
                return Constant(0);
            }

            ControllerIndex(eyes.value_or(std::string()));
            ControllerIndex({});
            ControllerIndex(*close);
            return Named(socket == 1 ? UpperEyelid : LowerEyelid, *closeV);
        }

        default:
            return Constant(0);
        }
    }
};

struct Pending {
    Uid node = 0;
    std::string target;
    std::vector<Step> steps;
    std::set<std::string> reads;
    bool done = false;
};

struct BuiltRule {
    Uid node = 0;
    std::string target;
    std::vector<Step> steps;
};

template<class Entries>
int IndexOfName(const Entries& entries, std::string_view name) {
    for (std::size_t i = 0; i < entries.size(); ++i)
        if (SameName(entries[i].name, name)) return int(i);
    return -1;
}

bool SameOps(const cm::FlexRule& a, const cm::FlexRule& b) {
    if (a.flex != b.flex || a.ops.size() != b.ops.size()) return false;

    for (std::size_t i = 0; i < a.ops.size(); ++i)
        if (a.ops[i].op != b.ops[i].op || a.ops[i].d.index != b.ops[i].d.index) return false;
    return true;
}

int StackDepth(const cm::FlexRule& rule) {
    int depth = 0, deepest = 0;
    for (const auto& step : rule.ops) {
        depth += StackEffect(step);
        deepest = std::max(deepest, depth);
    }
    return deepest;
}

std::string NodeLabel(const Document& document, Uid uid) {
    const FlexNode* node = document.findFlexNode(uid);
    if (!node) return "Flex Graph";
    return node->name.empty() ? "Flex Graph node" : "Flex Graph '" + node->name + "'";
}

void Warn(const Document& document, const Problem& problem) {
    pulse::diag::WarnAt({}, NodeLabel(document, problem.node) + ": " + problem.message + " The rule was left out.");
}

} // namespace

bool RegisterFlexGraph(const Document& document, cm::CompileInput& in, std::string* err) {
    std::vector<Problem> problems;
    Emitter emitter{document, problems, {}, {}, {}, {}};

    // The descs the model already has are the morphs a graph can read and write.
    for (const auto& desc : in.flexdescs) emitter.morphNames.insert(Lowered(desc.name));

    std::vector<cm::FlexController> controllers;
    for (const auto& node : document.flexNodes) {
        if (node.type != FlexNodeType::Controller) continue;

        if (!NameUsable(node.name)) {
            emitter.Report(node.uid, "Use letters, digits and _, and not min or max.");
            continue;
        }
        // A stereo controller is a pair, right first as the compiler registers them.
        const std::vector<std::string> names = node.stereo ? std::vector<std::string>{"right_" + node.name, "left_" + node.name}
                                                           : std::vector<std::string>{node.name};
        const bool taken = std::ranges::any_of(names, [&](const std::string& name) { return emitter.controllers.contains(Lowered(name)); });
        if (taken) {
            emitter.Report(node.uid, "Another controller has this name.");
            continue;
        }
        if (!(node.max > node.min)) emitter.Report(node.uid, "Max must be above min.");

        emitter.controllerNodes.insert(node.uid);
        for (const auto& name : names) {
            emitter.controllers.insert(Lowered(name));
            cm::FlexController controller;
            controller.name = name;
            controller.type = node.group.empty() ? "default" : node.group;
            controller.min = node.min;
            controller.max = node.max;
            controllers.push_back(std::move(controller));
        }
    }

    // The panel declares local vars; one that cannot be written as a rule name or repeats another is left out.
    std::vector<std::string> localvars;
    for (const auto& name : document.flexLocalVars) {
        if (!NameUsable(name) || emitter.morphNames.contains(Lowered(name))) continue;
        if (!emitter.localvars.insert(Lowered(name)).second) continue;

        localvars.push_back(name);
    }

    std::vector<Pending> pending;
    std::set<std::string> targets;
    for (const auto& node : document.flexNodes) {
        const bool output = node.type == FlexNodeType::FlexOutput;
        if ((!output && node.type != FlexNodeType::LocalVar) || node.muted) continue;

        if (output) {
            if (node.name.empty()) {
                emitter.Report(node.uid, "Pick a morph.");
                continue;
            }
            if (!emitter.morphNames.contains(Lowered(node.name))) {
                emitter.Report(node.uid, "The model has no morph with this name.");
                continue;
            }
        } else if (!emitter.localvars.contains(Lowered(node.name))) {
            emitter.Report(node.uid, node.name.empty() ? "Pick a local var." : "No local var is declared with this name.");
            continue;
        }

        if (!targets.insert(Lowered(node.name)).second) {
            emitter.Report(node.uid, "Another output already writes this name.");
            continue;
        }

        Pending rule;
        rule.node = node.uid;
        rule.target = node.name;
        emitter.reads = &rule.reads;
        emitter.out = &rule.steps;
        emitter.path = {node.uid};
        emitter.Input(node, 0);
        emitter.reads = nullptr;
        emitter.out = nullptr;
        pending.push_back(std::move(rule));
    }

    // Rules run in order, so one that reads a morph or local var must come after the rule that writes it.
    std::set<std::string> waiting;
    for (const auto& rule : pending) waiting.insert(Lowered(rule.target));

    std::vector<BuiltRule> ordered;
    for (bool progress = true; progress;) {
        progress = false;

        for (auto& rule : pending) {
            if (rule.done) continue;

            const bool ready = std::ranges::none_of(rule.reads, [&](const std::string& name) { return waiting.contains(name); });
            if (!ready) continue;

            rule.done = true;
            waiting.erase(Lowered(rule.target));
            ordered.push_back({rule.node, rule.target, std::move(rule.steps)});
            progress = true;
        }
    }
    for (const auto& rule : pending)
        if (!rule.done) emitter.Report(rule.node, "Part of a loop through Morph Value or Local Var nodes.");

    // Register: local vars, controllers, then the rules by name.
    for (const auto& name : localvars)
        if (pulse::loader::AddFlexdesc(in, name, err) < 0)
            return false;
    for (auto& controller : controllers) {
        if (IndexOfName(in.flexcontrollers, controller.name) >= 0) {
            pulse::diag::WarnAt({}, "morph controller '" + controller.name + "' already defined, reusing existing entry");
            continue;
        }
        if (in.flexcontrollers.size() >= static_cast<std::size_t>(lim::kMaxFlexCtrl)) {
            if (err) *err = "too many flex controllers, max " + std::to_string(lim::kMaxFlexCtrl);
            return false;
        }
        in.flexcontrollers.push_back(std::move(controller));
    }

    for (const auto& rule : ordered) {
        cm::FlexRule built;
        built.flex = IndexOfName(in.flexdescs, rule.target);
        if (built.flex < 0) {
            problems.push_back({rule.node, "The model has no morph named " + rule.target + "."});
            continue;
        }

        std::string error;
        for (const auto& step : rule.steps) {
            cm::FlexOp op{};
            op.op = step.op;

            switch (step.op) {
            case flexop::Const:
                op.d.value = step.value;
                if (step.controllerIndex) {
                    const int controller = step.name.empty() ? -1 : IndexOfName(in.flexcontrollers, step.name);
                    if (controller < 0 && !step.name.empty()) error = "No controller is named " + step.name + ".";
                    op.d.value = float(controller);
                }
                break;

            case flexop::Fetch2:
                op.d.index = IndexOfName(in.flexdescs, step.name);
                if (op.d.index < 0) error = "No morph or local var is named " + step.name + ".";
                break;

            case flexop::Combo:
            case flexop::Dominate:
                op.d.index = step.count;
                break;

            case flexop::Fetch1:
            case flexop::TwoWay0:
            case flexop::TwoWay1:
            case flexop::NWay:
            case flexop::LowerEyelid:
            case flexop::UpperEyelid:
                op.d.index = IndexOfName(in.flexcontrollers, step.name);
                if (op.d.index < 0) error = "No controller is named " + step.name + ".";
                break;

            default:
                break;
            }

            if (!error.empty()) break;
            built.ops.push_back(op);
        }

        if (error.empty() && in.flexrules.size() >= std::size_t(lim::kMaxFlexRules))
            error = "Too many flex rules, the limit is " + std::to_string(lim::kMaxFlexRules) + ".";
        if (!error.empty()) {
            problems.push_back({rule.node, error});
            continue;
        }
        if (StackDepth(built) > 32)
            problems.push_back({rule.node, "The expression is too deep: the game's evaluator holds 32 values."});

        // The same rule twice is kept once; a different rule for the same morph stacks as a staged rule.
        if (std::ranges::any_of(in.flexrules, [&](const cm::FlexRule& other) { return SameOps(other, built); })) continue;

        in.flexrules.push_back(std::move(built));
    }

    for (const Problem& problem : problems) Warn(document, problem);
    return true;
}

} // namespace pulse::mdlasm
