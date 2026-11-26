// mdlasmanim.cpp - the Animation Graph of a model assembly, turned into animations, sequences, weight lists and
// pose parameters. A copy of Studio's libs/modeleditor/src/animgraph.cpp emit (the Studio preview runs the same
// code), so the graph means the same thing in the editor and here.

#include "mdlasmanim.h"

#include <algorithm>
#include <cctype>
#include <map>
#include <set>

namespace pulse::mdlasm {
namespace {


// Animation and sequence flags as the compiler reads them (qcloader.cpp:46-71).
constexpr int kLooping = 0x0001;
constexpr int kSnap = 0x0002;
constexpr int kDelta = 0x0004;
constexpr int kAutoplay = 0x0008;
constexpr int kPost = 0x0010;
constexpr int kRealtime = 0x0100;
constexpr int kLocal = 0x0200;
constexpr int kHidden = 0x0400;
constexpr int kCyclePose = 0x0080;
constexpr int kWorld = 0x4000;
constexpr int kNoForceLoop = 0x8000;
constexpr int kWorldAndRelative = 0x20000;
constexpr int kRootXform = 0x40000;
constexpr int kAlSpline = 0x0040;
constexpr int kAlXfade = 0x0080;
constexpr int kAlNoblend = 0x0200;
constexpr int kAlLocal = 0x1000;
constexpr int kAlPose = 0x4000;

constexpr float kDegToRad = 3.14159265358979f / 180.0f;

bool sameName(std::string_view a, std::string_view b)
{
    return std::ranges::equal(a, b, [](unsigned char x, unsigned char y) { return std::tolower(x) == std::tolower(y); });
}

std::string lowered(std::string text)
{
    for (char& ch : text) ch = char(std::tolower(static_cast<unsigned char>(ch)));
    return text;
}

constexpr const char* WalkInputs[9] = {"Center", "N", "NE", "E", "SE", "S", "SW", "W", "NW"};

// The links in the ordered input list; fixed sockets before it are not counted.
int linkedInputCount(const Document& document, Uid uid, int first = 0)
{
    return int(std::ranges::count_if(document.animLinks, [&](const AnimLink& link) { return link.to == uid && link.toSocket >= first; }));
}

// The first input of a Blend's clip list; the inputs before it are its pose axes.
int animOrderedFirst(AnimNodeType type)
{
    return type == AnimNodeType::Blend ? 2 : 0;
}

// The input socket that takes a Pose Param node: `which` is the axis of a Blend or Walk, 0 for the others.
int animPoseSocket(AnimNodeType type, int which = 0)
{
    switch (type) {
    case AnimNodeType::Blend: return which == 0 ? 0 : 1;
    case AnimNodeType::Walk: return which == 0 ? 9 : 10;
    case AnimNodeType::Sequence:
    case AnimNodeType::Layer: return 2;
    default: return -1;
    }
}

const AnimLink* inputLink(const Document& document, Uid uid, int socket)
{
    for (const auto& link : document.animLinks)
        if (link.to == uid && link.toSocket == socket) return &link;
    return nullptr;
}

// The node feeding an input, null when the input is free or its source is muted.
const AnimNode* inputNode(const Document& document, const AnimNode& node, int socket)
{
    const auto* link = inputLink(document, node.uid, socket);
    if (!link) return nullptr;

    const auto* source = document.findAnimNode(link->from);
    return source && !source->muted ? source : nullptr;
}


} // namespace

bool animCommandHasReference(const AnimCommand& command)
{
    switch (command.kind) {
    case AnimCommand::Subtract:
    case AnimCommand::Align:
    case AnimCommand::Match:
    case AnimCommand::MatchBlend:
    case AnimCommand::WorldspaceBlend:
    case AnimCommand::AppendAnim:
    case AnimCommand::RefMotion: return true;
    case AnimCommand::CopyPose: return !command.copyBindPose;
    default: return false;
    }
}

std::vector<Uid> animUpstream(const Document& document, Uid uid)
{
    std::vector<Uid> found;
    std::set<Uid> seen{uid};
    std::vector<Uid> stack{uid};

    while (!stack.empty()) {
        const Uid current = stack.back();
        stack.pop_back();

        for (const auto& link : document.animLinks) {
            if (link.to != current || !seen.insert(link.from).second) continue;
            found.push_back(link.from);
            stack.push_back(link.from);
        }
    }
    return found;
}

namespace {


class Emitter
{
public:
    explicit Emitter(const Document& document) : document_(document) {}

    AnimScript run()
    {
        declareParameters();

        const AnimNode* out = nullptr;
        for (const auto& node : document_.animNodes)
            if (node.type == AnimNodeType::Out) out = &node;
        if (!out) return std::move(script_);

        std::vector<const AnimLink*> listed;
        for (const auto& link : document_.animLinks)
            if (link.to == out->uid) listed.push_back(&link);
        std::ranges::stable_sort(listed, [](const AnimLink* a, const AnimLink* b) { return a->toSocket < b->toSocket; });

        for (const auto* link : listed) {
            const auto* node = document_.findAnimNode(link->from);
            if (!node || node->muted) continue;

            if (node->type == AnimNodeType::Animation) namedAnim(*node);
            else sequenceOf(*node);
        }

        // Layers name other sequences; those are written after the listed ones.
        while (!pending_.empty()) {
            const Uid uid = pending_.back();
            pending_.pop_back();
            if (const auto* node = document_.findAnimNode(uid)) sequenceOf(*node);
        }
        return std::move(script_);
    }

private:
    void problem(Uid node, std::string message) { script_.problems.push_back({node, std::move(message)}); }

    int weightList(const AnimNode& node)
    {
        const auto found = weights_.find(node.uid);
        if (found != weights_.end()) return found->second;

        compile::WeightList list;
        list.name = node.name;
        for (const auto& entry : node.weights.entries) {
            compile::WeightList::Entry out;
            out.bone = entry.bone;
            out.weight = entry.weight;
            out.posweight = entry.posSet ? entry.posWeight : entry.weight;
            list.entries.push_back(std::move(out));
        }
        script_.weightLists.push_back(std::move(list));
        return weights_[node.uid] = int(script_.weightLists.size()) - 1;
    }

    // The Pose panel's parameters come first, in list order, so they get the lowest indices.
    void declareParameters()
    {
        for (const auto& declared : document_.poseParameters) {
            if (declared.name.empty()) continue;

            const bool taken = std::ranges::any_of(script_.poseParams, [&](const compile::PoseParam& existing) {
                return sameName(existing.name, declared.name);
            });
            if (taken) continue;

            compile::PoseParam param;
            param.name = declared.name;
            param.min = declared.min;
            param.max = declared.max;
            script_.poseParams.push_back(std::move(param));
            declared_.insert(lowered(declared.name));
        }
    }

    const AnimPoseParameter* declaredParameter(std::string_view name) const
    {
        const auto found = std::ranges::find_if(document_.poseParameters, [&](const AnimPoseParameter& param) {
            return sameName(param.name, name);
        });
        return found == document_.poseParameters.end() ? nullptr : &*found;
    }

    // The Pose Param node linked to an input, or null.
    const AnimNode* poseSource(const AnimNode& node, int socket) const
    {
        const auto* source = inputNode(document_, node, socket);
        return source && source->type == AnimNodeType::PoseParam ? source : nullptr;
    }

    // The name an input stands for: the linked Pose Param node, otherwise the typed name.
    std::string poseName(const AnimNode& node, int socket, const std::string& typed) const
    {
        const auto* source = poseSource(node, socket);
        return source ? source->name : typed;
    }

    // A linked axis takes its name and range from the declared parameter the node names.
    AnimBlendAxis blendAxis(const AnimNode& node, int index) const
    {
        auto axis = node.blend.axis[index];
        const auto* source = poseSource(node, animPoseSocket(node.type, index));
        if (!source) return axis;

        axis.name = source->name;
        if (const auto* declared = declaredParameter(axis.name)) {
            axis.min = declared->min;
            axis.max = declared->max;
        }
        return axis;
    }

    void declarePose(const AnimNode& node, const AnimBlendAxis& axis)
    {
        if (axis.name.empty() || axis.calc) return;

        compile::PoseParam param;
        param.name = axis.name;
        param.min = axis.min;
        param.max = axis.max;
        if (axis.wrap || axis.loop != 0.0f) {
            param.flags |= kLooping;
            param.loop = axis.wrap ? axis.max - axis.min : axis.loop;
        }

        for (auto& existing : script_.poseParams) {
            if (!sameName(existing.name, param.name)) continue;

            // A declared parameter owns its range; the axis only adds looping when the parameter has none.
            if (declared_.contains(lowered(existing.name))) {
                if ((axis.wrap || axis.loop != 0.0f) && existing.flags == 0) {
                    existing.flags |= kLooping;
                    existing.loop = axis.wrap ? existing.max - existing.min : axis.loop;
                }
                return;
            }

            if (existing.min != param.min || existing.max != param.max || existing.flags != param.flags || existing.loop != param.loop)
                problem(node.uid, "Pose parameter \"" + param.name + "\" is declared with another range elsewhere.");
            return;
        }
        script_.poseParams.push_back(std::move(param));
    }

    static void applyProps(compile::CompileInput::InAnim& anim, const AnimProps& props)
    {
        anim.fps = props.fps;
        if (props.loop) anim.flags |= kLooping;
        if (props.snap) anim.flags |= kSnap;
        if (props.post) anim.flags |= kPost;
        if (props.noForceLoop) anim.flags |= kNoForceLoop;
        anim.ignorescale = props.ignoreScale;
        anim.noAutoIK = props.noAutoIk;
        anim.nocull = props.noCull;

        if (props.fudgeLoop) {
            anim.fudgeloop = true;
            anim.flags |= kLooping;
        }
        if (props.startLoop != 0) {
            anim.looprestart = props.startLoop;
            anim.flags |= kLooping;
        }
        if (props.startLoopPercent != 0.0f) {
            anim.looprestartpercent = props.startLoopPercent;
            anim.flags |= kLooping;
        }

        anim.motiontype = props.motionType;
        anim.motionrollback = props.motionRollback;
        if (props.originSet) {
            anim.adjust = props.origin;
            anim.adjustSet = true;
        }
        if (props.anglesSet) {
            anim.rotation = math::RadianEuler{props.angles.x * kDegToRad, props.angles.y * kDegToRad,
                                              (props.angles.z + 90.0f) * kDegToRad};
            anim.rotationSet = true;
        }
        anim.scale = props.scale;
        anim.disableAnimblocks = props.disableAnimBlocks;
        anim.isFirstSectionLocal = props.firstSectionLocal;
        anim.numNostallFrames = props.noStallFrames;
        anim.ikrules = props.ikRules;
    }

    void checkProps(const AnimNode& node, const AnimProps& props)
    {
        if (props.fps <= 0.0f) problem(node.uid, "The frame rate must be above 0.");
        if (props.startLoop != 0 && props.startLoopPercent != 0.0f) problem(node.uid, "Start loop and percent start loop are both set.");
        if (props.numFrames < 0 || props.numFrames > pulse::limits::kMaxAnimFrames)
            problem(node.uid, "The frame count must be 1 to " + std::to_string(pulse::limits::kMaxAnimFrames) + ".");
    }

    // Walks the commands from `top` back to the clip; returns the clip node and the commands in script order.
    const AnimNode* chain(const AnimNode& top, std::vector<const AnimNode*>& commands)
    {
        const AnimNode* current = &top;
        std::set<Uid> seen;

        while (current && current->type == AnimNodeType::Command) {
            if (!seen.insert(current->uid).second) return nullptr;
            commands.push_back(current);
            current = inputNode(document_, *current, 0);
        }
        std::ranges::reverse(commands);
        return current && current->type == AnimNodeType::Clip ? current : nullptr;
    }

    // The animation a command names as its reference: a named animation, or a clip written as one of its own.
    std::string referenceName(const AnimNode& command)
    {
        const auto* ref = inputNode(document_, command, 1);
        if (!ref) {
            problem(command.uid, "Connect the reference clip.");
            return {};
        }
        if (ref->type == AnimNodeType::Animation) return script_.anims[std::size_t(namedAnim(*ref))].anim.name;

        std::vector<const AnimNode*> commands;
        const auto* clip = chain(*ref, commands);
        if (!clip) {
            problem(command.uid, "The reference clip does not lead back to an Anim Clip.");
            return {};
        }

        const std::string name = clip->name;
        const auto found = referenced_.find(ref->uid);
        if (found != referenced_.end()) return script_.anims[std::size_t(found->second)].anim.name;

        compile::CompileInput::InAnim anim;
        anim.name = name;
        fillClip(anim, *clip);
        for (const auto* inner : commands) anim.cmds.push_back(commandOf(*inner));
        script_.anims.push_back({std::move(anim), clip->uid, clip->clip.bindPose ? 0 : clip->clip.source});
        referenced_[ref->uid] = int(script_.anims.size()) - 1;
        return name;
    }

    static void fillClip(compile::CompileInput::InAnim& anim, const AnimNode& clip)
    {
        anim.bindpose = clip.clip.bindPose;
        anim.blockname = clip.clip.clip;
        anim.startframe = clip.clip.startFrame;
        anim.endframe = clip.clip.bindPose ? 0 : clip.clip.endFrame;
    }

    compile::CompileInput::InAnim::InCmd commandOf(const AnimNode& node)
    {
        auto cmd = node.command;
        if (animCommandHasReference(cmd)) cmd.name = referenceName(node);
        return cmd;
    }

    int namedAnim(const AnimNode& node)
    {
        const auto found = named_.find(node.uid);
        if (found != named_.end()) return found->second;

        compile::CompileInput::InAnim anim;
        anim.name = node.name;
        applyProps(anim, node.animation);
        checkProps(node, node.animation);

        const auto* weights = inputNode(document_, node, 1);
        if (weights && weights->type == AnimNodeType::WeightList) {
            weightList(*weights);
            anim.cmds.push_back({AnimCommand::Weights, weights->name});
        }
        if (node.animation.numFrames > 0) {
            AnimCommand frames;
            frames.kind = AnimCommand::NumFrames;
            frames.numframes = node.animation.numFrames;
            anim.cmds.push_back(frames);
        }

        Uid source = 0;
        const auto* top = inputNode(document_, node, 0);
        if (!top) {
            problem(node.uid, "Connect an Anim Clip.");
        } else {
            std::vector<const AnimNode*> commands;
            const auto* clip = chain(*top, commands);
            if (!clip) {
                problem(node.uid, "The clip does not lead back to an Anim Clip.");
            } else {
                fillClip(anim, *clip);
                source = clip->clip.bindPose ? 0 : clip->clip.source;
                for (const auto* command : commands) anim.cmds.push_back(commandOf(*command));
            }
        }

        for (const auto& existing : script_.anims)
            if (sameName(existing.anim.name, anim.name) && existing.node != node.uid)
                problem(node.uid, "Another animation is named \"" + anim.name + "\".");

        script_.anims.push_back({std::move(anim), node.uid, source});
        return named_[node.uid] = int(script_.anims.size()) - 1;
    }

    int xnode(const std::string& name)
    {
        for (std::size_t i = 0; i < script_.xnodes.size(); ++i)
            if (sameName(script_.xnodes[i], name)) return int(i) + 1;
        script_.xnodes.push_back(name);
        return int(script_.xnodes.size());
    }

    // The Sequence node at the bottom of a layer chain, with the layers on the way, nearest first.
    const AnimNode* rootOf(const AnimNode& top, std::vector<const AnimNode*>& layers)
    {
        const AnimNode* current = &top;
        std::set<Uid> seen;

        while (current && current->type == AnimNodeType::Layer) {
            if (!seen.insert(current->uid).second) return nullptr;
            layers.push_back(current);
            current = inputNode(document_, *current, 0);
        }
        std::ranges::reverse(layers);
        return current && current->type == AnimNodeType::Sequence ? current : nullptr;
    }

    void sequenceOf(const AnimNode& top)
    {
        std::vector<const AnimNode*> layers;
        const auto* root = rootOf(top, layers);
        if (!root) {
            problem(top.uid, "A layer needs a Sequence at the start of its chain.");
            return;
        }
        if (!emitted_.insert(root->uid).second) {
            problem(root->uid, "The sequence is listed more than once.");
            return;
        }
        sequence(*root, layers);
    }

    void sequence(const AnimNode& root, const std::vector<const AnimNode*>& layers)
    {
        const auto& data = root.sequence;
        AnimScriptSequence out;
        auto& seq = out.sequence;
        seq.name = root.name;
        out.node = root.uid;
        out.members = animUpstream(document_, root.uid);
        out.members.push_back(root.uid);

        for (const auto& existing : script_.sequences)
            if (sameName(existing.sequence.name, seq.name)) problem(root.uid, "Another sequence is named \"" + seq.name + "\".");

        seq.fadeintime = data.fadeIn;
        seq.fadeouttime = data.fadeOut;
        seq.exitphase = data.exitPhase;
        if (data.snap) seq.flags |= kSnap;
        if (data.autoPlay) seq.flags |= kAutoplay;
        if (data.hidden) seq.flags |= kHidden;
        if (data.realTime) seq.flags |= kRealtime;
        if (data.post) seq.flags |= kPost;
        if (data.delta == AnimDeltaMode::Delta) seq.flags |= kDelta | kPost;
        if (data.delta == AnimDeltaMode::PreDelta) seq.flags |= kDelta;
        if (data.worldSpace) seq.flags |= kWorld | kPost;
        if (data.worldRelative) seq.flags |= kWorldAndRelative | kPost;
        if (!data.rootDriver.empty()) {
            seq.flags |= kRootXform;
            seq.rootdriverBone = data.rootDriver;
        }
        const auto poseCycle = poseName(root, animPoseSocket(root.type), data.poseCycle);
        if (!poseCycle.empty()) {
            seq.flags |= kCyclePose;
            seq.posecycle = poseCycle;
        }

        if (!data.activity.empty()) {
            seq.activityname = data.activity;
            seq.actweight = data.activityWeight != 0 ? data.activityWeight : -1;
        }
        for (const auto& modifier : data.activityModifiers) seq.activitymodifiers.push_back(lowered(modifier));
        seq.events = data.events;
        seq.animtags = data.animTags;
        seq.iklocks = data.ikLocks;

        if (!data.entryNode.empty() || !data.exitNode.empty()) {
            seq.entrynode = xnode(data.entryNode.empty() ? data.exitNode : data.entryNode);
            seq.exitnode = xnode(data.exitNode.empty() ? data.entryNode : data.exitNode);
            if (data.reversible) seq.nodeflags |= 1;
        }

        cells(root, out);

        for (const auto* layer : layers) {
            compile::CompileInput::InAutoLayer entry;
            const auto& l = layer->layer;

            std::vector<const AnimNode*> inner;
            const auto* target = inputNode(document_, *layer, 1);
            const auto* targetRoot = target ? rootOf(*target, inner) : nullptr;
            if (!targetRoot) {
                problem(layer->uid, "Connect the sequence to layer.");
                continue;
            }
            if (targetRoot->uid == root.uid) problem(layer->uid, "A sequence cannot layer itself.");

            entry.sequence = targetRoot->name;
            if (l.add) {
                if (l.local) {
                    entry.flags |= kAlLocal;
                    seq.flags |= kLocal;
                }
            } else {
                entry.start = l.start;
                entry.peak = l.peak;
                entry.tail = l.tail;
                entry.end = l.end;
                if (l.xfade) entry.flags |= kAlXfade;
                if (l.spline) entry.flags |= kAlSpline;
                if (l.noBlend) entry.flags |= kAlNoblend;
                if (l.local) {
                    entry.flags |= kAlLocal;
                    seq.flags |= kLocal;
                }
                const auto* poseNode = poseSource(*layer, animPoseSocket(layer->type));
                if (l.usePose || poseNode) {
                    entry.flags |= kAlPose;
                    entry.poseparameter = poseName(*layer, animPoseSocket(layer->type), l.poseParameter);
                }
            }
            seq.autolayers.push_back(std::move(entry));
            out.members.push_back(layer->uid);
            for (const Uid uid : animUpstream(document_, layer->uid)) out.members.push_back(uid);

            if (!emitted_.contains(targetRoot->uid)) pending_.push_back(targetRoot->uid);
        }

        std::ranges::sort(out.members);
        out.members.erase(std::unique(out.members.begin(), out.members.end()), out.members.end());
        script_.sequences.push_back(std::move(out));
    }

    // The animations of the sequence: one for a clip, the grid cells for a Blend or Walk node.
    void cells(const AnimNode& root, AnimScriptSequence& out)
    {
        auto& seq = out.sequence;
        const auto* top = inputNode(document_, root, 0);
        if (!top) {
            problem(root.uid, "Connect a clip.");
            return;
        }

        std::vector<const AnimNode*> sources;
        if (top->type == AnimNodeType::Blend || top->type == AnimNodeType::Walk) {
            blendSetup(*top, seq, sources);
        } else {
            sources.push_back(top);
        }

        std::vector<int> blends;
        for (std::size_t i = 0; i < sources.size(); ++i) {
            if (!sources[i]) {
                problem(top->uid, "A cell of the grid has no clip.");
                blends.push_back(-1);
                continue;
            }
            blends.push_back(cell(root, seq, *sources[i], int(i), blends.empty() ? -1 : blends.front()));
        }

        if (blends.empty() || std::ranges::find(blends, -1) != blends.end()) return;

        seq.animIndex = blends.front();
        if (blends.size() > 1) seq.blendAnims = std::move(blends);
    }

    void blendSetup(const AnimNode& node, compile::CompileInput::InSequence& seq, std::vector<const AnimNode*>& sources)
    {
        if (node.type == AnimNodeType::Walk) {
            const auto& walk = node.walk;

            // Move X and Move Y keep their meaning when the axes are swapped; only the grid turns.
            AnimBlendAxis moves[2];
            for (int i = 0; i < 2; ++i) {
                moves[i].name = i == 0 ? walk.xName : walk.yName;
                moves[i].min = walk.min;
                moves[i].max = walk.max;

                if (const auto* source = poseSource(node, animPoseSocket(node.type, i))) {
                    moves[i].name = source->name;
                    if (const auto* declared = declaredParameter(source->name)) {
                        moves[i].min = declared->min;
                        moves[i].max = declared->max;
                    }
                }
            }
            const auto& first = walk.swapAxes ? moves[1] : moves[0];
            const auto& second = walk.swapAxes ? moves[0] : moves[1];

            seq.numblendparams = 2;
            seq.blendparams[0] = {first.name, first.min, first.max, false};
            seq.blendparams[1] = {second.name, second.min, second.max, false};
            seq.blendwidth = 3;

            declarePose(node, moves[0]);
            declarePose(node, moves[1]);

            // Cell (column, row): the first parameter runs along the columns. N is the high end of the first
            // parameter and E the high end of the second.
            constexpr int Column[9] = {1, 2, 2, 1, 0, 0, 0, 1, 2};
            constexpr int Row[9] = {1, 1, 2, 2, 2, 1, 0, 0, 0};
            sources.assign(9, nullptr);
            for (int input = 0; input < 9; ++input) {
                const int column = walk.swapAxes ? Row[input] : Column[input];
                const int row = walk.swapAxes ? Column[input] : Row[input];
                sources[std::size_t(row * 3 + column)] = inputNode(document_, node, input);
            }
            return;
        }

        const auto& blend = node.blend;
        seq.numblendparams = blend.axes;
        for (int i = 0; i < blend.axes; ++i) {
            const auto axis = blendAxis(node, i);
            seq.blendparams[i] = {axis.name, axis.min, axis.max, axis.calc};
            seq.paramattachment[i] = axis.calc ? axis.attachment : std::string();
            seq.paramcontrol[i] = axis.calc ? axis.control : 0;
            declarePose(node, axis);
        }
        seq.blendwidth = blend.blendWidth;
        seq.paramanim = blend.reference;
        seq.paramcompanim = blend.compare;
        seq.paramcenter = blend.center;

        const int first = animOrderedFirst(node.type);
        for (int i = 0; i < linkedInputCount(document_, node.uid, first); ++i) sources.push_back(inputNode(document_, node, first + i));

        const int count = int(sources.size());
        if (blend.blendWidth > 0 && count % blend.blendWidth != 0)
            problem(node.uid, "The clip count is not a multiple of the blend width.");
        if (blend.blendWidth == 0 && count >= 4) {
            int side = 0;
            while (side * side < count) ++side;
            if (side * side != count) problem(node.uid, "The grid is not square; set the blend width.");
        }
        if (blend.axes == 1 && count < 2) problem(node.uid, "Blend two or more clips.");
    }

    int cell(const AnimNode& root, compile::CompileInput::InSequence& seq, const AnimNode& source, int index, int firstIndex)
    {
        if (source.type == AnimNodeType::Animation) return namedAnim(source);

        std::vector<const AnimNode*> commands;
        const auto* clip = chain(source, commands);
        if (!clip) {
            problem(root.uid, "A clip does not lead back to an Anim Clip.");
            return -1;
        }

        compile::CompileInput::InAnim anim;
        fillClip(anim, *clip);
        const Uid sourceUid = clip->clip.bindPose ? 0 : clip->clip.source;

        if (index == 0) {
            anim.name = "@" + seq.name;
            applyProps(anim, root.sequence.anim);
            checkProps(root, root.sequence.anim);

            const auto* weights = inputNode(document_, root, 1);
            if (weights && weights->type == AnimNodeType::WeightList) {
                weightList(*weights);
                anim.cmds.push_back({AnimCommand::Weights, weights->name});
            }
            if (root.sequence.anim.numFrames > 0) {
                AnimCommand frames;
                frames.kind = AnimCommand::NumFrames;
                frames.numframes = root.sequence.anim.numFrames;
                anim.cmds.push_back(frames);
            }
            for (const auto* command : commands) anim.cmds.push_back(commandOf(*command));
        } else if (commands.empty()) {
            // An implied animation after the first inherits its settings and commands, as stock does.
            const auto& base = script_.anims[std::size_t(firstIndex)].anim;
            anim.name = "@" + seq.name;
            anim.fps = base.fps;
            anim.adjust = base.adjust;
            anim.adjustSet = base.adjustSet;
            anim.scale = base.scale;
            anim.rotation = base.rotation;
            anim.rotationSet = base.rotationSet;
            anim.motiontype = base.motiontype;
            anim.cmds = base.cmds;
            anim.ikrules = base.ikrules;
        } else {
            anim.name = seq.name + "_" + std::to_string(index + 1);
            const auto& base = script_.anims[std::size_t(firstIndex)].anim;
            anim.fps = base.fps;
            for (const auto* command : commands) anim.cmds.push_back(commandOf(*command));
        }

        script_.anims.push_back({std::move(anim), clip->uid, sourceUid});
        return int(script_.anims.size()) - 1;
    }

    const Document& document_;
    AnimScript script_;
    std::map<Uid, int> named_, weights_, referenced_;
    std::set<std::string> declared_;
    std::set<Uid> emitted_;
    std::vector<Uid> pending_;
};

} // namespace

AnimScript emitAnimScript(const Document& document)
{
    return Emitter(document).run();
}

} // namespace pulse::mdlasm
