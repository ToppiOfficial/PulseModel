// mdlasmread.cpp - decodes an .mdlasm into a Document. See mdlasmdoc.h.
//
// Record layouts follow the Model Assembler's writer. A record only ever gains fields at its end:
// a reader takes the fields it knows, so optional tails are read when bytes remain.

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>

#include "mdlasmdoc.h"
#include "mdlasmfile.h"

namespace pulse::mdlasm {

namespace limits = pulse::limits;

namespace {

Uid g_nextUid = (Uid(1) << 63);

Uid newUid() { return ++g_nextUid; }


constexpr std::uint32_t tag(char a, char b, char c, char d)
{
    return std::uint32_t(std::uint8_t(a)) | std::uint32_t(std::uint8_t(b)) << 8 |
           std::uint32_t(std::uint8_t(c)) << 16 | std::uint32_t(std::uint8_t(d)) << 24;
}

constexpr std::uint32_t SummaryChunk = tag('S', 'U', 'M', 'M');

constexpr std::uint32_t DefinitionChunk = tag('M', 'D', 'E', 'F');

constexpr std::uint32_t SourcesChunk = tag('S', 'R', 'C', 'S');

constexpr std::uint32_t BlobChunk = tag('B', 'L', 'O', 'B');

constexpr std::uint32_t MeshesChunk = tag('M', 'E', 'S', 'H');

constexpr std::uint32_t GroupsChunk = tag('M', 'G', 'R', 'P');

constexpr std::uint32_t MaterialsChunk = tag('M', 'M', 'R', 'K');

constexpr std::uint32_t BonesChunk = tag('B', 'D', 'E', 'F');

constexpr std::uint32_t RealignsChunk = tag('R', 'L', 'G', 'N');

constexpr std::uint32_t AttachmentsChunk = tag('A', 'T', 'T', 'C');

constexpr std::uint32_t HitboxesChunk = tag('H', 'B', 'O', 'X');

constexpr std::uint32_t HitboxSetsChunk = tag('H', 'S', 'E', 'T');

constexpr std::uint32_t PhysicsSettingsChunk = tag('P', 'H', 'Y', 'S');

constexpr std::uint32_t PhysicsNodesChunk = tag('P', 'H', 'Y', 'N');

constexpr std::uint32_t FoldersChunk = tag('F', 'L', 'D', 'R');

constexpr std::uint32_t BoneMarkupChunk = tag('B', 'M', 'R', 'K');

constexpr std::uint32_t EyeballsChunk = tag('E', 'Y', 'E', 'B');

constexpr std::uint32_t EyelidsChunk = tag('E', 'Y', 'L', 'D');

constexpr std::uint32_t PrefabsChunk = tag('P', 'R', 'F', 'B');

constexpr std::uint32_t LodsChunk = tag('L', 'O', 'D', 'S');

constexpr std::uint32_t LodSettingsChunk = tag('L', 'O', 'D', 'C');

constexpr std::uint32_t FlexNodesChunk = tag('F', 'L', 'X', 'N');

constexpr std::uint32_t FlexLinksChunk = tag('F', 'L', 'X', 'L');

constexpr std::uint32_t FlexWrinklesChunk = tag('F', 'L', 'X', 'W');

constexpr std::uint32_t FlexVarsChunk = tag('F', 'L', 'X', 'V');

constexpr std::uint32_t FlexSplitsChunk = tag('F', 'L', 'X', 'S');

constexpr std::uint32_t AnimNodesChunk = tag('A', 'N', 'O', 'D');

constexpr std::uint32_t AnimLinksChunk = tag('A', 'L', 'N', 'K');
constexpr std::uint32_t PoseParamsChunk = tag('P', 'P', 'R', 'M');

enum class MaterialRecord : std::uint32_t { Directory, Rename, Collapse };

// Enumerations stored as u32 are range checked so a bad value fails its record, not the file.
template<class E>
E readEnum(core::BinaryReader& in, std::uint32_t last)
{
    const auto value = in.read<std::uint32_t>();
    if (value > last) throw std::runtime_error("Invalid model value.");
    return E(value);
}

math::Vector3 readVector(core::BinaryReader& in)
{
    math::Vector3 v;
    v.x = in.read<float>();
    v.y = in.read<float>();
    v.z = in.read<float>();
    return v;
}

RenderDefinition readRenderDefinition(core::BinaryReader& in)
{
    RenderDefinition render;
    render.present = in.read<bool>();
    if (!render.present) return render;

    render.active = in.read<bool>();
    render.editorName = in.text();
    render.pass = readEnum<ModelRenderPass>(in, std::uint32_t(ModelRenderPass::MostlyOpaque));
    render.ambientBoost = in.read<bool>();
    render.doNotCastShadows = in.read<bool>();
    render.castTextureShadows = in.read<bool>();
    render.constantDirectionalLight = in.read<bool>();
    render.directionalLightScale = in.read<float>();
    render.noForcedFade = in.read<bool>();
    render.forcePhonemeCrossfade = in.read<bool>();

    if (!std::isfinite(render.directionalLightScale) || render.directionalLightScale < 0.0f
        || render.directionalLightScale > 1.0f)
        throw std::runtime_error("Invalid directional light scale.");
    return render;
}

// A stored order that is not a permutation of the nodes falls back to the default.
bool validOrder(const std::array<DefinitionNode, DefinitionNodeCount>& order)
{
    std::array<bool, DefinitionNodeCount> seen{};
    for (const auto node : order) {
        if (std::size_t(node) >= DefinitionNodeCount || seen[std::size_t(node)]) return false;
        seen[std::size_t(node)] = true;
    }
    return true;
}

ModelDefinition readDefinition(core::BinaryReader& in)
{
    ModelDefinition d;
    d.name = in.text();
    d.surfaceProp = in.text();
    d.scale = in.read<float>();
    d.origin = readVector(in);
    d.rotationMode = readEnum<RotationMode>(in, std::uint32_t(RotationMode::UpZ));
    d.angles = readVector(in);
    d.archetype = readEnum<compile::Archetype>(in, std::uint32_t(compile::Archetype::Simple));
    d.bboxSet = in.read<bool>();
    d.bbox[0] = readVector(in);
    d.bbox[1] = readVector(in);
    d.cbox[0] = readVector(in);
    d.cbox[1] = readVector(in);
    d.illumSet = in.read<bool>();
    d.illum = readVector(in);
    d.illumBone = in.text();
    d.eyePosition = readVector(in);
    d.eyeAutoHeight = in.read<bool>();

    // Records saved before the opt-in flag existed treat any eye value as the node being present.
    d.eyeEnabled = in.remaining() ? in.read<bool>()
                                  : d.eyeAutoHeight || d.eyePosition.x != 0 || d.eyePosition.y != 0 || d.eyePosition.z != 0;
    d.eyeActive = in.remaining() ? in.read<bool>() : true;
    if (in.remaining()) d.eyeEditorName = in.text();
    if (in.remaining()) d.render = readRenderDefinition(in);
    if (in.remaining() && in.read<bool>()) std::swap(d.nodeOrder[0], d.nodeOrder[1]);
    if (in.remaining()) {
        d.weightFitPoses = in.read<std::int32_t>();
        if (d.weightFitPoses < 0 || d.weightFitPoses > pulse::limits::kMaxWeightFitPoses)
            throw std::runtime_error("Invalid weight fit pose count.");
    }
    if (in.remaining()) {
        const auto cull = in.read<std::uint32_t>();
        if (cull > 1 + std::uint32_t(compile::BoneCullType::Skinned))
            throw std::runtime_error("Invalid bone cull method.");
        if (cull) d.boneCullMethod = compile::BoneCullType(cull - 1);
    }

    // Records saved before the view box had its own switch treat any value as set.
    const bool legacyView = d.cbox[0].x != 0 || d.cbox[0].y != 0 || d.cbox[0].z != 0
                         || d.cbox[1].x != 0 || d.cbox[1].y != 0 || d.cbox[1].z != 0;
    d.cboxSet = in.remaining() ? in.read<bool>() : legacyView;
    if (in.remaining() >= DefinitionNodeCount) {
        auto order = d.nodeOrder;
        for (auto& node : order) node = DefinitionNode(in.read<std::uint8_t>());
        if (validOrder(order)) d.nodeOrder = order;
    }
    // Records saved before the nodes were opt-in keep the ones that were set.
    if (in.remaining() >= 3) {
        d.bboxPresent = in.read<bool>();
        d.cboxPresent = in.read<bool>();
        d.illumPresent = in.read<bool>();
    } else {
        d.bboxPresent = d.bboxSet;
        d.cboxPresent = d.cboxSet;
        d.illumPresent = d.illumSet;
    }
    if (in.remaining() >= sizeof(float)) {
        const auto deflection = in.read<float>();
        if (std::isfinite(deflection)) d.eyeMaxDeflection = std::clamp(deflection, 0.0f, 89.0f);
    }
    return d;
}

// Sources.


SourceFile readSource(core::BinaryReader& in)
{
    SourceFile source;
    source.uid = in.read<Uid>();
    if (!source.uid) throw std::runtime_error("Invalid source id.");
    source.kind = readEnum<SourceKind>(in, std::uint32_t(SourceKind::Joints));
    source.path = in.text();
    source.modified = in.read<std::int64_t>();
    return source;
}

source::EditMeshOperation readOperation(core::BinaryReader& in)
{
    source::EditMeshOperation op;
    op.kind = readEnum<source::MeshOperation>(in, std::uint32_t(source::MeshOperation::SwitchUV));
    op.combine = readEnum<source::SelectionCombine>(in, std::uint32_t(source::SelectionCombine::Intersect));
    op.name = in.text();
    op.residualBone = in.text();
    op.channel = in.read<std::int32_t>();
    op.directionStream = in.text();
    op.weightStream = in.text();
    op.weightChannel = in.read<std::int32_t>();
    op.flipY = in.read<bool>();
    op.closeSeams = in.read<bool>();
    op.offset = readVector(in);
    op.angles = readVector(in);
    for (auto i = in.count(8); i; --i) op.materials.push_back(in.text());
    op.value = in.read<float>();
    op.thicknessClamp = in.read<float>();
    op.replaceSelection = in.read<bool>();
    op.hull = in.read<bool>();
    op.sharpEdges = in.read<bool>();
    op.selectChildren = in.read<bool>();
    op.contains = in.read<bool>();
    op.worldAxes = in.read<bool>();
    op.ignoreSelectWarn = in.read<bool>();
    return op;
}

source::EditSkeletonOperation readSkeletonOperation(core::BinaryReader& in)
{
    source::EditSkeletonOperation op;
    op.kind = readEnum<source::SkeletonOperation>(in, std::uint32_t(source::SkeletonOperation::ScaleBone));
    op.combine = readEnum<source::SelectionCombine>(in, std::uint32_t(source::SelectionCombine::Intersect));
    op.name = in.text();
    op.parent = in.text();
    op.origin = readVector(in);
    op.angles = readVector(in);
    op.scale = readVector(in);
    op.hasOrigin = in.read<bool>();
    op.hasAngles = in.read<bool>();
    op.worldAxes = in.read<bool>();
    op.absolute = in.read<bool>();
    op.affectChildren = in.read<bool>();
    op.affectMeshes = in.read<bool>();
    op.keepParents = in.read<bool>();
    op.selectNew = in.read<bool>();
    op.ignoreSelectWarn = in.read<bool>();
    return op;
}

source::MeshFilter readFilter(core::BinaryReader& in)
{
    source::MeshFilter filter;
    for (auto i = in.count(8); i; --i) filter.names.push_back({in.text(), false});
    filter.exclusive = in.read<bool>();

    for (auto i = in.count(12); i; --i) {
        source::MeshFilter::TagEntry entry;
        entry.name = in.text();
        entry.tag = in.read<std::int32_t>();
        filter.tags.push_back(std::move(entry));
    }

    for (auto i = in.count(16); i; --i) {
        source::MeshFilter::EditStream stream;
        stream.name = in.text();
        stream.component = in.read<std::int32_t>();
        stream.kind = readEnum<source::EditStreamKind>(in, std::uint32_t(source::EditStreamKind::Channel));
        filter.editStreams.push_back(std::move(stream));
    }

    filter.keepMeshIdentity = in.read<bool>();
    filter.selectMeshStreams = in.read<bool>();
    return filter;
}

MeshEntry readMesh(core::BinaryReader& in)
{
    MeshEntry mesh;
    mesh.uid = in.read<Uid>();
    if (!mesh.uid) throw std::runtime_error("Invalid mesh id.");
    mesh.source = in.read<Uid>();

    auto& edit = mesh.edit;
    edit.name = in.text();
    edit.filter = readFilter(in);

    for (auto blocks = in.count(8); blocks; --blocks) {
        auto& block = edit.editBlocks.emplace_back();
        block.skeleton = in.read<bool>();
        if (block.skeleton) {
            for (auto ops = in.count(32); ops; --ops) block.skeletonOperations.push_back(readSkeletonOperation(in));
        } else {
            for (auto ops = in.count(32); ops; --ops) block.operations.push_back(readOperation(in));
        }
    }

    edit.noMorph = in.read<bool>();
    edit.skeletalAwareDecimation = in.read<bool>();

    for (auto i = in.count(8); i; --i) {
        const auto factor = in.read<float>();
        edit.decimates.emplace_back(factor, in.read<std::int32_t>());
    }

    mesh.enabled = in.remaining() ? in.read<bool>() : true;
    if (in.remaining()) {
        for (auto& block : edit.editBlocks) {
            block.uid = in.read<Uid>();
            if (!block.uid) throw std::runtime_error("Invalid mesh edit block id.");
            block.enabled = in.read<bool>();
            block.ignoreSelectWarn = in.read<bool>();
        }
        if (in.remaining())
            for (auto& block : edit.editBlocks) block.editorName = in.text();
    } else {
        for (auto& block : edit.editBlocks) {
            block.uid = newUid();
            block.ignoreSelectWarn = !block.operations.empty() && block.operations.front().ignoreSelectWarn;
        }
    }

    for (auto& block : edit.editBlocks) {
        if (!block.skeleton) continue;
        for (auto& op : block.skeletonOperations) op.ignoreSelectWarn = block.ignoreSelectWarn;
    }
    return mesh;
}

ModelGroup readGroup(core::BinaryReader& in)
{
    ModelGroup group;
    group.uid = in.read<Uid>();
    if (!group.uid) throw std::runtime_error("Invalid group id.");
    group.name = in.text();
    group.enabled = in.read<bool>();
    for (auto choices = in.count(16); choices; --choices) {
        auto& choice = group.choices.emplace_back();
        choice.name = in.text();
        for (auto meshes = in.count(8); meshes; --meshes) choice.meshes.push_back(in.read<Uid>());
    }

    return group;
}

BoneDefinition readBoneDefinition(core::BinaryReader& in)
{
    BoneDefinition bone;
    bone.uid = in.read<Uid>();
    if (!bone.uid) throw std::runtime_error("Invalid bone definition id.");

    bone.legacyName = in.text();
    bone.enabled = in.read<bool>();
    bone.bone = in.text();
    bone.boneMerge = in.read<bool>();
    if (!in.remaining()) return bone;

    auto& bind = bone.bind;
    bind.active = in.read<bool>();
    bind.parent = in.text();
    bind.origin = readVector(in);
    bind.angles = readVector(in);
    bind.unlocked = in.read<bool>();
    bind.preAligned = in.read<bool>();
    bind.realignOrigin = readVector(in);
    bind.realignAngles = readVector(in);

    if (in.remaining()) bone.doNotCollapse = in.read<bool>();
    if (in.remaining()) bone.alwaysCollapse = in.read<bool>();
    if (in.remaining()) bone.noRealign = in.read<bool>();

    for (const auto& v : {bind.origin, bind.angles, bind.realignOrigin, bind.realignAngles})
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
            throw std::runtime_error("Invalid bone transform.");
    return bone;
}

RealignNode readRealign(core::BinaryReader& in)
{
    RealignNode node;
    node.uid = in.read<Uid>();
    if (!node.uid) throw std::runtime_error("Invalid realign id.");

    node.name = in.text();
    node.enabled = in.read<bool>();
    node.bone = in.text();

    auto& edit = node.edit;
    edit.hasOrigin = in.read<bool>();
    edit.hasAngles = in.read<bool>();
    edit.origin = readVector(in);
    edit.angles = readVector(in);
    edit.worldAxes = in.read<bool>();
    edit.absolute = in.read<bool>();
    edit.affectChildren = in.read<bool>();
    edit.ignoreAnimation = in.read<bool>();

    for (const auto& v : {edit.origin, edit.angles})
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
            throw std::runtime_error("Invalid realign transform.");
    return node;
}

Attachment readAttachment(core::BinaryReader& in)
{
    Attachment attachment;
    attachment.uid = in.read<Uid>();
    attachment.name = in.text();
    attachment.enabled = in.read<bool>();
    attachment.bone = in.text();
    attachment.origin = readVector(in);
    attachment.angles = readVector(in);
    attachment.anglesSet = in.read<bool>();
    attachment.absolute = in.read<bool>();
    attachment.rigid = in.read<bool>();
    attachment.worldAlign = in.read<bool>();
    if (!attachment.uid) throw std::runtime_error("Invalid attachment ID.");
    return attachment;
}

EyelidNode readEyelid(core::BinaryReader& in)
{
    EyelidNode eyelid;
    eyelid.uid = in.read<Uid>();
    eyelid.name = in.text();
    eyelid.enabled = in.read<bool>();
    eyelid.type = readEnum<EyelidType>(in, std::uint32_t(EyelidType::Lower));
    eyelid.rightEye = in.text();
    eyelid.leftEye = in.text();
    for (auto& pose : eyelid.poses) {
        pose.morph = in.text();
        pose.height = in.read<float>();
        if (!std::isfinite(pose.height)) throw std::runtime_error("Invalid eyelid value.");
    }
    eyelid.split = in.read<float>();
    if (!eyelid.uid) throw std::runtime_error("Invalid eyelid ID.");
    if (!std::isfinite(eyelid.split)) throw std::runtime_error("Invalid eyelid value.");

    return eyelid;
}

Eyeball readEyeball(core::BinaryReader& in)
{
    Eyeball eyeball;
    eyeball.uid = in.read<Uid>();
    eyeball.name = in.text();
    eyeball.enabled = in.read<bool>();
    eyeball.bone = in.text();
    eyeball.material = in.text();
    eyeball.origin = readVector(in);
    eyeball.center = in.read<bool>();
    eyeball.diameter = in.read<float>();
    eyeball.angle = in.read<float>();
    eyeball.pupilScale = in.read<float>();
    if (!eyeball.uid) throw std::runtime_error("Invalid eyeball ID.");

    // Eyeball records of 27.68.0 carried their lids; lids are their own nodes now, so those are skipped.
    if (in.remaining()) {
        for (int lid = 0; lid < 2; ++lid) {
            for (int pose = 0; pose < 3; ++pose) {
                in.text();
                in.read<float>();
            }
            in.read<float>();
        }
    }

    for (const auto value : {eyeball.diameter, eyeball.angle, eyeball.pupilScale})
        if (!std::isfinite(value)) throw std::runtime_error("Invalid eyeball value.");
    return eyeball;
}

PrefabNode readPrefab(core::BinaryReader& in)
{
    PrefabNode node;
    node.uid = in.read<Uid>();
    node.name = in.text();
    node.enabled = in.read<bool>();
    node.kind = readEnum<PrefabKind>(in, std::uint32_t(PrefabKind::Physics));
    node.source = in.read<Uid>();
    node.jigglebones = in.read<bool>();
    node.proceduralbones = in.read<bool>();
    node.hitboxes = in.read<bool>();
    node.attachments = in.read<bool>();
    if (in.remaining())
        for (auto renames = in.count(25); renames; --renames) {
            HitboxSetRename rename;
            rename.uid = in.read<Uid>();
            rename.enabled = in.read<bool>();
            rename.from = in.text();
            rename.to = in.text();
            node.hitboxSetRenames.push_back(std::move(rename));
        }
    if (!node.uid) throw std::runtime_error("Invalid prefab ID.");
    return node;
}

std::vector<std::string> readStrings(core::BinaryReader& in)
{
    std::vector<std::string> values;
    for (auto count = in.count(8); count; --count) values.push_back(in.text());
    return values;
}

std::vector<MaterialRule> readMaterialRules(core::BinaryReader& in)
{
    std::vector<MaterialRule> rules;
    for (auto count = in.count(16); count; --count) {
        auto& rule = rules.emplace_back();
        rule.from = in.text();
        rule.to = in.text();
    }
    return rules;
}

LodNode readLod(core::BinaryReader& in)
{
    LodNode node;
    node.uid = in.read<Uid>();
    node.name = in.text();
    node.enabled = in.read<bool>();
    node.shadow = in.read<bool>();
    node.switchValue = in.read<float>();
    if (!node.uid) throw std::runtime_error("Invalid LOD ID.");

    for (auto count = in.count(16); count; --count) {
        auto& rule = node.models.emplace_back();
        rule.target = in.text();
        rule.action = readEnum<LodAction>(in, std::uint32_t(LodAction::Remove));
        rule.replacement = in.text();
        rule.factor = in.read<float>();
        if (!std::isfinite(rule.factor)) throw std::runtime_error("Invalid LOD factor.");
    }
    node.decimateAll = in.read<bool>();
    node.decimateAllFactor = in.read<float>();
    node.skeletalAware = in.read<bool>();
    node.removeSmall = in.read<bool>();
    node.smallSize = in.read<float>();
    node.smallMeshes = readStrings(in);

    for (auto count = in.count(16); count; --count) {
        auto& rule = node.boneReplacements.emplace_back();
        rule.bone = in.text();
        rule.replacement = in.text();
    }
    node.boneTreeCollapses = readStrings(in);
    node.collapseJiggle = in.read<bool>();
    node.collapseProcedural = in.read<bool>();

    node.materialReplacements = readMaterialRules(in);
    node.materialSubstringReplacements = readMaterialRules(in);
    node.meshRemovals = readStrings(in);
    node.meshSubstringRemovals = readStrings(in);

    node.facial = in.read<bool>();
    node.shadowMaterials = in.read<bool>();
    for (const auto value : {node.switchValue, node.decimateAllFactor, node.smallSize})
        if (!std::isfinite(value)) throw std::runtime_error("Invalid LOD value.");
    return node;
}

LodSettings readLodSettings(core::BinaryReader& in)
{
    LodSettings settings;
    settings.minLod = std::clamp(in.read<std::int32_t>(), 0, limits::kMaxNumLods);
    settings.allowRootLods = std::clamp(in.read<std::int32_t>(), 0, 255);
    return settings;
}

void requireFinite(std::initializer_list<float> values)
{
    for (const float value : values)
        if (!std::isfinite(value)) throw std::runtime_error("Invalid animation value.");
}

AnimIkRule readIkRule(core::BinaryReader& in)
{
    AnimIkRule rule;
    rule.chain = in.text();
    rule.type = in.text();
    rule.touchBone = in.text();
    rule.attachment = in.text();
    rule.height = in.read<float>();
    rule.floor = in.read<float>();
    rule.radius = in.read<float>();
    rule.heightSet = in.read<bool>();
    rule.floorSet = in.read<bool>();
    rule.radiusSet = in.read<bool>();
    rule.slot = in.read<std::int32_t>();
    rule.slotSet = in.read<bool>();
    rule.contact = in.read<std::int32_t>();
    rule.startframe = in.read<std::int32_t>();
    rule.peakframe = in.read<std::int32_t>();
    rule.tailframe = in.read<std::int32_t>();
    rule.endframe = in.read<std::int32_t>();
    rule.usesequence = in.read<bool>();
    rule.usesource = in.read<bool>();
    rule.fakeorigin = readVector(in);
    rule.fakerotate = readVector(in);
    rule.fakeoriginSet = in.read<bool>();
    rule.fakerotateSet = in.read<bool>();
    rule.autosteps = in.read<bool>();
    rule.autostepsCount = in.read<std::int32_t>();
    rule.autostepsBone = in.text();
    requireFinite({rule.height, rule.floor, rule.radius, rule.fakeorigin.x, rule.fakeorigin.y, rule.fakeorigin.z,
                   rule.fakerotate.x, rule.fakerotate.y, rule.fakerotate.z});
    return rule;
}

AnimProps readAnimProps(core::BinaryReader& in)
{
    AnimProps props;
    props.fps = in.read<float>();
    props.loop = in.read<bool>();
    props.snap = in.read<bool>();
    props.post = in.read<bool>();
    props.noForceLoop = in.read<bool>();
    props.fudgeLoop = in.read<bool>();
    props.noAutoIk = in.read<bool>();
    props.noCull = in.read<bool>();
    props.ignoreScale = in.read<bool>();
    props.startLoop = in.read<std::int32_t>();
    props.startLoopPercent = in.read<float>();
    props.motionType = in.read<std::int32_t>();
    props.motionRollback = in.read<float>();
    props.numFrames = in.read<std::int32_t>();
    props.originSet = in.read<bool>();
    props.origin = readVector(in);
    props.anglesSet = in.read<bool>();
    props.angles = readVector(in);
    props.scale = in.read<float>();
    props.disableAnimBlocks = in.read<bool>();
    props.firstSectionLocal = in.read<bool>();
    props.noStallFrames = in.read<float>();
    // Records only grow at the end, so list slots whose entries are nodes now stay, always empty.
    for (auto remaining = in.count(8); remaining; --remaining) in.text();
    requireFinite({props.fps, props.startLoopPercent, props.motionRollback, props.origin.x, props.origin.y, props.origin.z,
                   props.angles.x, props.angles.y, props.angles.z, props.scale, props.noStallFrames});
    return props;
}

AnimCommand readAnimCommand(core::BinaryReader& in)
{
    AnimCommand cmd;
    cmd.kind = readEnum<AnimCommand::Kind>(in, std::uint32_t(AnimCommand::DiscardChannels));
    cmd.name = in.text();
    cmd.frame = in.read<std::int32_t>();
    cmd.numframes = in.read<std::int32_t>();
    cmd.presubtract = in.read<bool>();
    cmd.fixupStart = in.read<std::int32_t>();
    cmd.fixupEnd = in.read<std::int32_t>();
    cmd.angle = in.read<float>();
    cmd.alignBone = in.text();
    cmd.motiontype = in.read<std::int32_t>();
    cmd.srcframe = in.read<std::int32_t>();
    cmd.destframe = in.read<std::int32_t>();
    cmd.driverAxis = in.read<std::int32_t>();
    cmd.driverValue = in.read<float>();
    cmd.driverAll = in.read<bool>();
    cmd.driverStart = in.read<std::int32_t>();
    cmd.driverPeak = in.read<std::int32_t>();
    cmd.driverTail = in.read<std::int32_t>();
    cmd.driverEnd = in.read<std::int32_t>();
    cmd.matchPre = in.read<std::int32_t>();
    cmd.matchPost = in.read<std::int32_t>();
    cmd.motionEndFrame = in.read<std::int32_t>();
    cmd.worldLoops = in.read<bool>();
    cmd.copyBindPose = in.read<bool>();
    cmd.copyEndFrame = in.read<std::int32_t>();
    cmd.fadeIn = in.read<float>();
    cmd.fadeOut = in.read<float>();
    cmd.copyPos = in.read<bool>();
    cmd.copyRot = in.read<bool>();
    cmd.xform.bone = in.text();
    cmd.xform.origin = readVector(in);
    cmd.xform.angles.x = in.read<float>();
    cmd.xform.angles.y = in.read<float>();
    cmd.xform.angles.z = in.read<float>();
    cmd.xform.originSet = in.read<bool>();
    cmd.xform.anglesSet = in.read<bool>();
    core::parseRecord(in.text(), [&](core::BinaryReader& record) { cmd.ikfixup = readIkRule(record); });
    cmd.derivativeScale = in.read<float>();
    cmd.keepDuration = in.read<bool>();
    cmd.splineDelta = in.read<bool>();
    cmd.compressFrames = in.read<std::int32_t>();
    cmd.counterHasTarget = in.read<bool>();
    cmd.counterAngles = readVector(in);
    cmd.forceDoPos = in.read<bool>();
    cmd.forceDoRot = in.read<bool>();
    cmd.forceRotLocal = in.read<bool>();
    cmd.forcePos = readVector(in);
    cmd.forceRot = readVector(in);
    for (auto remaining = in.count(8); remaining; --remaining) {
        compile::ChannelDiscard discard;
        discard.bone = in.text();
        discard.posAxes = in.read<std::int32_t>();
        discard.rotAxes = in.read<std::int32_t>();
        discard.wholeBone = in.read<bool>();
        cmd.discards.push_back(std::move(discard));
    }
    requireFinite({cmd.angle, cmd.driverValue, cmd.fadeIn, cmd.fadeOut, cmd.xform.origin.x, cmd.xform.origin.y,
                   cmd.xform.origin.z, cmd.xform.angles.x, cmd.xform.angles.y, cmd.xform.angles.z, cmd.derivativeScale,
                   cmd.counterAngles.x, cmd.counterAngles.y, cmd.counterAngles.z, cmd.forcePos.x, cmd.forcePos.y,
                   cmd.forcePos.z, cmd.forceRot.x, cmd.forceRot.y, cmd.forceRot.z});
    return cmd;
}

AnimSequenceData readAnimSequence(core::BinaryReader& in)
{
    AnimSequenceData data;
    data.anim = readAnimProps(in);
    data.activity = in.text();
    data.activityWeight = in.read<std::int32_t>();
    readStrings(in);
    data.hidden = in.read<bool>();
    data.autoPlay = in.read<bool>();
    data.realTime = in.read<bool>();
    data.snap = in.read<bool>();
    data.post = in.read<bool>();
    data.worldSpace = in.read<bool>();
    data.worldRelative = in.read<bool>();
    data.delta = readEnum<AnimDeltaMode>(in, std::uint32_t(AnimDeltaMode::PreDelta));
    data.fadeIn = in.read<float>();
    data.fadeOut = in.read<float>();
    data.exitPhase = in.read<float>();
    data.poseCycle = in.text();
    data.rootDriver = in.text();
    data.entryNode = in.text();
    data.exitNode = in.text();
    data.reversible = in.read<bool>();

    for (auto remaining = in.count(8); remaining; --remaining) {
        compile::SeqEvent event;
        event.name = in.text();
        event.frame = in.read<std::int32_t>();
        event.options = in.text();
        data.events.push_back(std::move(event));
    }
    for (auto remaining = in.count(8); remaining; --remaining) {
        compile::SeqAnimTag tag;
        tag.name = in.text();
        tag.cycle = in.read<float>();
        data.animTags.push_back(std::move(tag));
    }
    for (auto remaining = in.count(8); remaining; --remaining) {
        in.text();
        in.read<float>();
        in.read<float>();
    }

    requireFinite({data.fadeIn, data.fadeOut, data.exitPhase});
    for (const auto& tag : data.animTags) requireFinite({tag.cycle});
    return data;
}

AnimNode readAnimNode(core::BinaryReader& in)
{
    AnimNode node;
    node.uid = in.read<Uid>();
    node.type = readEnum<AnimNodeType>(in, std::uint32_t(AnimNodeType::DeclareSequence));
    node.x = in.read<float>();
    node.y = in.read<float>();
    node.muted = in.read<bool>();
    node.collapsed = in.read<bool>();
    node.name = in.text();
    if (!node.uid) throw std::runtime_error("Invalid animation node ID.");
    requireFinite({node.x, node.y});

    switch (node.type) {
    case AnimNodeType::Out:
        break;
    case AnimNodeType::Clip:
        node.clip.source = in.read<Uid>();
        node.clip.clip = in.text();
        node.clip.bindPose = in.read<bool>();
        node.clip.startFrame = in.read<std::int32_t>();
        node.clip.endFrame = in.read<std::int32_t>();
        break;
    case AnimNodeType::Animation:
        node.animation = readAnimProps(in);
        break;
    case AnimNodeType::Sequence:
        node.sequence = readAnimSequence(in);
        break;
    case AnimNodeType::WeightList:
        for (auto remaining = in.count(8); remaining; --remaining) {
            AnimWeightEntry entry;
            entry.bone = in.text();
            entry.weight = in.read<float>();
            entry.posWeight = in.read<float>();
            entry.posSet = in.read<bool>();
            requireFinite({entry.weight, entry.posWeight});
            node.weights.entries.push_back(std::move(entry));
        }
        break;
    case AnimNodeType::Command:
        node.command = readAnimCommand(in);
        break;
    case AnimNodeType::Blend:
        node.blend.axes = std::clamp(in.read<std::int32_t>(), 1, 2);
        for (auto& axis : node.blend.axis) {
            axis.name = in.text();
            axis.min = in.read<float>();
            axis.max = in.read<float>();
            axis.wrap = in.read<bool>();
            axis.loop = in.read<float>();
            axis.calc = in.read<bool>();
            axis.attachment = in.text();
            axis.control = in.read<std::int32_t>();
            requireFinite({axis.min, axis.max, axis.loop});
        }
        node.blend.blendWidth = in.read<std::int32_t>();
        node.blend.reference = in.text();
        node.blend.compare = in.text();
        node.blend.center = in.text();
        break;
    case AnimNodeType::Walk:
        node.walk.xName = in.text();
        node.walk.yName = in.text();
        node.walk.min = in.read<float>();
        node.walk.max = in.read<float>();
        node.walk.swapAxes = in.read<bool>();
        requireFinite({node.walk.min, node.walk.max});
        break;
    case AnimNodeType::Layer:
        node.layer.add = in.read<bool>();
        node.layer.start = in.read<float>();
        node.layer.peak = in.read<float>();
        node.layer.tail = in.read<float>();
        node.layer.end = in.read<float>();
        node.layer.xfade = in.read<bool>();
        node.layer.spline = in.read<bool>();
        node.layer.noBlend = in.read<bool>();
        node.layer.local = in.read<bool>();
        node.layer.usePose = in.read<bool>();
        node.layer.poseParameter = in.text();
        requireFinite({node.layer.start, node.layer.peak, node.layer.tail, node.layer.end});
        break;
    case AnimNodeType::PoseParam:
    case AnimNodeType::ActivityModifier:
    case AnimNodeType::IncludeModel:
    case AnimNodeType::DeclareSequence:
        break;
    case AnimNodeType::IkLock:
        node.ikLock.name = in.text();
        node.ikLock.flPosWeight = in.read<float>();
        node.ikLock.flLocalQWeight = in.read<float>();
        requireFinite({node.ikLock.flPosWeight, node.ikLock.flLocalQWeight});
        break;
    case AnimNodeType::IkRule:
        core::parseRecord(in.text(), [&](core::BinaryReader& record) { node.ikRule = readIkRule(record); });
        break;
    }
    return node;
}

AnimPoseParameter readPoseParameter(core::BinaryReader& in)
{
    AnimPoseParameter param;
    param.uid = in.read<Uid>();
    param.name = in.text();
    param.min = in.read<float>();
    param.max = in.read<float>();
    if (!param.uid) throw std::runtime_error("Invalid pose parameter ID.");
    requireFinite({param.min, param.max});
    return param;
}

AnimLink readAnimLink(core::BinaryReader& in)
{
    AnimLink link;
    link.from = in.read<Uid>();
    link.fromSocket = in.read<std::int32_t>();
    link.to = in.read<Uid>();
    link.toSocket = in.read<std::int32_t>();
    if (!link.from || !link.to || link.fromSocket < 0 || link.toSocket < 0) throw std::runtime_error("Invalid animation link.");
    return link;
}

FlexNode readFlexNode(core::BinaryReader& in)
{
    FlexNode node;
    node.uid = in.read<Uid>();
    node.type = readEnum<FlexNodeType>(in, std::uint32_t(FlexNodeType::Eyelid));
    node.x = in.read<float>();
    node.y = in.read<float>();
    node.muted = in.read<bool>();
    node.collapsed = in.read<bool>();
    node.parent = in.read<Uid>();
    node.name = in.text();
    node.group = in.text();
    node.min = in.read<float>();
    node.max = in.read<float>();
    for (float& value : node.values) value = in.read<float>();
    node.width = in.read<float>();
    node.height = in.read<float>();
    // Records written before stereo and Math end here.
    if (in.remaining()) {
        node.stereo = in.read<bool>();
        node.mode = std::clamp(in.read<std::int32_t>(), 0, 5);
    }
    if (!node.uid) throw std::runtime_error("Invalid flex node ID.");

    for (const auto value : {node.x, node.y, node.min, node.max, node.width, node.height})
        if (!std::isfinite(value)) throw std::runtime_error("Invalid flex node value.");
    for (const float value : node.values)
        if (!std::isfinite(value)) throw std::runtime_error("Invalid flex node value.");
    return node;
}

FlexLink readFlexLink(core::BinaryReader& in)
{
    FlexLink link;
    link.from = in.read<Uid>();
    link.fromSocket = in.read<std::int32_t>();
    link.to = in.read<Uid>();
    link.toSocket = in.read<std::int32_t>();
    if (!link.from || !link.to || link.fromSocket < 0 || link.toSocket < 0) throw std::runtime_error("Invalid flex link.");
    return link;
}

struct StoredHitbox
{
    Hitbox box;
    std::string setName;
    Uid owner = 0;
};

StoredHitbox readHitbox(core::BinaryReader& in)
{
    StoredHitbox stored;
    auto& hitbox = stored.box;
    hitbox.uid = in.read<Uid>();
    hitbox.name = in.text();
    hitbox.enabled = in.read<bool>();
    hitbox.bone = in.text();
    stored.setName = in.text();
    hitbox.group = in.read<std::int32_t>();
    hitbox.capsule = in.read<bool>();
    hitbox.min = readVector(in);
    hitbox.max = readVector(in);
    hitbox.angles = readVector(in);
    hitbox.radius = in.read<float>();
    if (in.remaining()) stored.owner = in.read<Uid>();
    if (!hitbox.uid || !std::isfinite(hitbox.radius) || hitbox.radius <= 0)
        throw std::runtime_error("Invalid hitbox.");
    for (const auto& value : {hitbox.min, hitbox.max, hitbox.angles})
        if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
            throw std::runtime_error("Invalid hitbox transform.");
    return stored;
}

void attachHitboxes(Document& document, std::vector<StoredHitbox>& hitboxes)
{
    for (auto& entry : hitboxes) {
        auto* owner = entry.owner ? document.findHitboxSet(entry.owner) : nullptr;
        if (!owner) {
            const auto name = entry.setName.empty() ? std::string("default") : std::move(entry.setName);
            const auto found = std::ranges::find(document.hitboxSets, name, &HitboxSet::name);
            if (found == document.hitboxSets.end()) {
                auto& added = document.hitboxSets.emplace_back();
                added.uid = entry.owner ? entry.owner : newUid();
                added.name = name;
                owner = &added;
            } else {
                owner = &*found;
            }
        }
        owner->boxes.push_back(std::move(entry.box));
    }
}

PhysicsSettings readPhysicsSettings(core::BinaryReader& in)
{
    PhysicsSettings settings;
    settings.rootBone = in.text();
    settings.fileName = in.text();
    settings.massCenter = readVector(in);
    settings.mass = in.read<float>();
    settings.damping = in.read<float>();
    settings.rotationDamping = in.read<float>();
    settings.inertia = in.read<float>();
    settings.drag = in.read<float>();
    settings.weldPosition = in.read<float>();
    settings.weldNormal = in.read<float>();
    settings.autoMass = in.read<bool>();
    settings.massCenterSet = in.read<bool>();
    settings.noSelfCollisions = in.read<bool>();
    settings.assumeWorldSpace = in.read<bool>();
    settings.animatedFriction = in.read<bool>();
    settings.frictionMin = in.read<int>();
    settings.frictionMax = in.read<int>();
    settings.frictionIn = in.read<float>();
    settings.frictionOut = in.read<float>();
    settings.frictionHold = in.read<float>();
    return settings;
}

PhysicsShape readPhysicsShape(core::BinaryReader& in)
{
    PhysicsShape shape;
    shape.type = readEnum<PhysicsShapeType>(in, std::uint32_t(PhysicsShapeType::FromRender));
    shape.bone = in.text();
    shape.min = readVector(in);
    shape.max = readVector(in);
    shape.angles = readVector(in);
    shape.center = readVector(in);
    shape.radius = in.read<float>();
    shape.endRadius = in.read<float>();
    shape.segments = in.read<int>();

    shape.source = in.read<Uid>();
    shape.renderMesh = in.text();
    shape.offsetOrigin = readVector(in);
    shape.offsetAngles = readVector(in);
    shape.importScale = in.read<float>();
    shape.importType = readEnum<compile::PhysicsImportType>(in, std::uint32_t(compile::PhysicsImportType::OneBoneOnly));
    shape.maxConvex = std::clamp(in.read<int>(), 1, limits::kMaxConvexPieces);
    shape.concave = in.read<bool>();
    shape.remove2d = in.read<bool>();
    shape.decimationFactor = in.read<float>();
    shape.concavity = in.read<float>();
    shape.cullWeight = in.read<float>();
    shape.maxHulls = std::clamp(in.read<int>(), 0, limits::kMaxConvexPieces);
    shape.maxDepth = in.read<int>();
    for (auto* list : {&shape.extraBones, &shape.excludedMeshes})
        for (auto remaining = in.count(8); remaining; --remaining) list->push_back(in.text());
    if (in.remaining()) shape.merge = in.read<bool>();
    return shape;
}

template<class Settings, class Visit>
void visitJiggleFloats(Settings& jiggle, Visit visit)
{
    for (auto* value : {&jiggle.length, &jiggle.tipMass, &jiggle.yawStiffness, &jiggle.yawDamping,
                        &jiggle.pitchStiffness, &jiggle.pitchDamping, &jiggle.alongStiffness, &jiggle.alongDamping,
                        &jiggle.yawFriction, &jiggle.yawBounce, &jiggle.pitchFriction, &jiggle.pitchBounce,
                        &jiggle.angleLimit, &jiggle.minYaw, &jiggle.maxYaw, &jiggle.minPitch, &jiggle.maxPitch,
                        &jiggle.baseMass, &jiggle.baseStiffness, &jiggle.baseDamping,
                        &jiggle.baseMinLeft, &jiggle.baseMaxLeft, &jiggle.baseLeftFriction,
                        &jiggle.baseMinUp, &jiggle.baseMaxUp, &jiggle.baseUpFriction,
                        &jiggle.baseMinForward, &jiggle.baseMaxForward, &jiggle.baseForwardFriction,
                        &jiggle.boingImpactSpeed, &jiggle.boingImpactAngle, &jiggle.boingDampingRate,
                        &jiggle.boingFrequency, &jiggle.boingAmplitude})
        visit(*value);
}


BoneMarkupNode readBoneMarkup(core::BinaryReader& in)
{
    BoneMarkupNode node;
    node.uid = in.read<Uid>();
    node.bone = in.text();
    node.enabled = in.read<bool>();
    if (!node.uid) throw std::runtime_error("Invalid bone markup ID.");

    switch (readEnum<BoneMarkupKind>(in, std::uint32_t(BoneMarkupKind::Ik))) {
    case BoneMarkupKind::Jiggle: {
        auto& jiggle = node.value.emplace<JiggleSettings>();
        jiggle.style = readEnum<JiggleStyle>(in, std::uint32_t(JiggleStyle::None));
        jiggle.base = readEnum<JiggleBase>(in, std::uint32_t(JiggleBase::Boing));
        jiggle.lengthFlex = in.read<bool>();
        jiggle.angleConstraint = in.read<bool>();
        jiggle.yawConstraint = in.read<bool>();
        jiggle.pitchConstraint = in.read<bool>();
        jiggle.baseLeftConstraint = in.read<bool>();
        jiggle.baseUpConstraint = in.read<bool>();
        jiggle.baseForwardConstraint = in.read<bool>();
        visitJiggleFloats(jiggle, [&](float& value) { value = in.read<float>(); });
        break;
    }
    case BoneMarkupKind::Procedural: {
        auto& procedural = node.value.emplace<ProceduralSettings>();
        procedural.mode = readEnum<ProceduralMode>(in, std::uint32_t(ProceduralMode::AimAt));
        procedural.driver = in.text();
        procedural.local = in.read<bool>();
        procedural.basePosition = readVector(in);
        for (auto remaining = in.count(4 + 3 * 12); remaining; --remaining) {
            ProceduralTrigger trigger;
            trigger.tolerance = in.read<float>();
            trigger.driverAngles = readVector(in);
            trigger.helperAngles = readVector(in);
            trigger.helperPosition = readVector(in);
            procedural.triggers.push_back(trigger);
        }
        procedural.target = in.text();
        procedural.up = readVector(in);
        procedural.aim = readVector(in);

        // Files saved before the influence options end here.
        if (in.remaining() >= 2 * sizeof(float)) {
            procedural.influenceInput = in.read<float>();
            procedural.influenceOutput = in.read<float>();
            if (!(procedural.influenceInput > 0.0f)) procedural.influenceInput = 1.0f;
            if (!(procedural.influenceOutput >= 0.0f)) procedural.influenceOutput = 1.0f;
        }

        // Files saved before the trigger switches end here.
        if (in.remaining() >= procedural.triggers.size())
            for (auto& trigger : procedural.triggers) trigger.enabled = in.read<bool>();

        // Files saved before twist triggers end here.
        if (in.remaining() >= procedural.triggers.size() * 12)
            for (auto& trigger : procedural.triggers) {
                trigger.kind = readEnum<TriggerKind>(in, std::uint32_t(TriggerKind::Twist));
                trigger.twistAxis = readEnum<TwistAxis>(in, std::uint32_t(TwistAxis::Z));
                trigger.twistAngle = in.read<float>();
                if (!std::isfinite(trigger.twistAngle)) throw std::runtime_error("Invalid twist angle.");
            }

        // Files saved before the inverted helper twist end here.
        if (in.remaining() >= procedural.triggers.size())
            for (auto& trigger : procedural.triggers) trigger.invertHelperTwist = in.read<bool>();
        break;
    }
    case BoneMarkupKind::Ik: {
        auto& chain = node.value.emplace<IkSettings>();
        chain.chain = in.text();
        chain.knee = readVector(in);
        chain.center = readVector(in);
        chain.height = in.read<float>();
        chain.pad = in.read<float>();
        chain.floor = in.read<float>();
        chain.autoplayLock = in.read<bool>();
        chain.lockPosition = in.read<float>();
        chain.lockRotation = in.read<float>();
        break;
    }
    }
    return node;
}

struct StoredFolder
{
    Folder folder;
    std::vector<Uid> members;
};

StoredFolder readFolder(core::BinaryReader& in)
{
    StoredFolder stored;
    auto& folder = stored.folder;
    folder.uid = in.read<Uid>();
    folder.name = in.text();
    folder.kind = readEnum<FolderKind>(in, std::uint32_t(FolderKind::BoneMarkup));
    folder.enabled = in.read<bool>();
    if (!folder.uid) throw std::runtime_error("Invalid folder ID.");

    for (auto members = in.count(sizeof(Uid)); members; --members) stored.members.push_back(in.read<Uid>());
    if (in.remaining()) folder.parent = in.read<Uid>();
    return stored;
}

void attachFolders(Document& document, const std::vector<StoredFolder>& stored)
{
    const auto assign = [](auto& nodes, Uid folder, const std::vector<Uid>& members) {
        for (auto& node : nodes)
            if (std::ranges::find(members, node.uid) != members.end()) node.folder = folder;
    };
    for (const auto& [folder, members] : stored) {
        switch (folder.kind) {
        case FolderKind::Mesh: assign(document.meshes, folder.uid, members); break;
        case FolderKind::ModelGroup: assign(document.groups, folder.uid, members); break;
        case FolderKind::BoneDefinition:
            assign(document.bones, folder.uid, members);
            assign(document.realigns, folder.uid, members);
            break;
        case FolderKind::Attachment: assign(document.attachments, folder.uid, members); break;
        case FolderKind::HitboxSet: assign(document.hitboxSets, folder.uid, members); break;
        case FolderKind::Physics: assign(document.physicsNodes, folder.uid, members); break;
        case FolderKind::BoneMarkup: assign(document.boneMarkups, folder.uid, members); break;
        }
    }
}

enum class PhysicsRecord : std::uint32_t { Shape, Joint, Body, Pair };

PhysicsNode readPhysicsNode(core::BinaryReader& in)
{
    PhysicsNode node;
    node.uid = in.read<Uid>();
    node.name = in.text();
    node.enabled = in.read<bool>();

    switch (readEnum<PhysicsRecord>(in, std::uint32_t(PhysicsRecord::Pair))) {
    case PhysicsRecord::Shape:
        node.value = readPhysicsShape(in);
        break;
    case PhysicsRecord::Joint: {
        auto& joint = node.value.emplace<PhysicsJoint>();
        joint.bone = in.text();
        for (auto& axis : joint.axes) {
            axis.mode = readEnum<PhysicsAxisMode>(in, std::uint32_t(PhysicsAxisMode::Fixed));
            axis.min = in.read<float>();
            axis.max = in.read<float>();
            axis.friction = in.read<float>();
        }
        break;
    }
    case PhysicsRecord::Body: {
        auto& body = node.value.emplace<PhysicsBody>();
        body.bone = in.text();
        body.mergeInto = in.text();
        body.mode = readEnum<PhysicsBodyMode>(in, std::uint32_t(PhysicsBodyMode::Merge));
        body.massMode = readEnum<PhysicsMassMode>(in, std::uint32_t(PhysicsMassMode::Bias));
        body.mass = in.read<float>();
        body.massBias = in.read<float>();
        body.inertia = in.read<float>();
        body.damping = in.read<float>();
        body.rotationDamping = in.read<float>();
        body.inertiaSet = in.read<bool>();
        body.dampingSet = in.read<bool>();
        body.rotationDampingSet = in.read<bool>();
        break;
    }
    case PhysicsRecord::Pair:
        node.value = PhysicsPair{in.text(), in.text(), in.read<bool>()};
        break;
    }
    return node;
}

// A chunk holding exactly one record; extra records from a newer version are skipped.
template<class Parse>
void readSingle(core::BinaryReader& in, Parse&& parse)
{
    const auto count = in.count(8);
    if (!count) throw std::runtime_error("Empty model chunk.");

    core::parseRecord(in.text(), parse);
    for (auto i = count - 1; i; --i) in.text();
}

void readMaterialRecord(core::BinaryReader& in, MaterialMarkup& materials)
{
    switch (readEnum<MaterialRecord>(in, std::uint32_t(MaterialRecord::Collapse))) {
    case MaterialRecord::Directory: materials.directories.push_back(in.text()); break;
    case MaterialRecord::Rename: materials.renames.push_back({in.text(), in.text()}); break;
    case MaterialRecord::Collapse: materials.collapses.push_back({in.text(), in.text()}); break;
    }
}


template<class Visit>
void visitFolderMembers(Document& document, Visit&& visit)
{
    for (auto& node : document.meshes) visit(node, FolderKind::Mesh);
    for (auto& node : document.groups) visit(node, FolderKind::ModelGroup);
    for (auto& node : document.bones) visit(node, FolderKind::BoneDefinition);
    for (auto& node : document.realigns) visit(node, FolderKind::BoneDefinition);
    for (auto& node : document.attachments) visit(node, FolderKind::Attachment);
    for (auto& node : document.hitboxSets) visit(node, FolderKind::HitboxSet);
    for (auto& node : document.physicsNodes) visit(node, FolderKind::Physics);
    for (auto& node : document.boneMarkups) visit(node, FolderKind::BoneMarkup);
}

// Reads a count-prefixed list of records. A record that fails stops the compile: Studio writes only
// valid data, so a bad record means the file is not one it saved.
template<class Parse>
void readRecords(core::BinaryReader& in, const char* area, Parse&& parse)
{
    std::uint64_t index = 0;
    for (auto remaining = in.count(8); remaining; --remaining, ++index) {
        const auto bytes = in.text();
        try {
            core::parseRecord(bytes, parse);
        } catch (const std::exception& error) {
            throw std::runtime_error(std::string("The ") + area + " entry " + std::to_string(index + 1) +
                                     " cannot be read (" + error.what() + "). Open and save it in PulseStudio.");
        }
    }
}

std::string tagName(std::uint32_t type)
{
    std::string name;
    for (int i = 0; i < 4; ++i) name.push_back(char((type >> (8 * i)) & 255));
    return name;
}

bool known(std::uint32_t type)
{
    return type == SummaryChunk || type == DefinitionChunk || type == SourcesChunk ||
           type == BlobChunk || type == MeshesChunk || type == GroupsChunk || type == MaterialsChunk ||
           type == BonesChunk || type == AttachmentsChunk || type == HitboxesChunk || type == HitboxSetsChunk ||
           type == PhysicsSettingsChunk || type == PhysicsNodesChunk || type == FoldersChunk ||
           type == BoneMarkupChunk || type == EyeballsChunk || type == EyelidsChunk || type == PrefabsChunk ||
           type == LodsChunk || type == LodSettingsChunk || type == FlexNodesChunk || type == FlexLinksChunk ||
           type == FlexWrinklesChunk || type == FlexVarsChunk || type == FlexSplitsChunk || type == AnimNodesChunk ||
           type == AnimLinksChunk || type == PoseParamsChunk || type == RealignsChunk;
}

Document readDocument(core::ChunkReader& reader)
{
    Document document;
    std::map<Uid, std::string> blobs;
    std::vector<StoredHitbox> hitboxes;
    std::vector<StoredFolder> folders;
    std::set<Uid> hitboxIds;
    const std::uint32_t version = reader.documentVersion();

    if (version < 4)
        throw std::runtime_error("This model assembly uses an older format. Open and save it in PulseStudio.");

    for (const auto& chunk : reader.chunks()) {
        if (!known(chunk.type)) {
            if (chunk.flags & core::ChunkRequired)
                throw std::runtime_error("This model assembly needs a newer PulseModel (section \"" +
                                         tagName(chunk.type) + "\").");
            document.unknownChunks.push_back(tagName(chunk.type));
            continue;
        }
        if (chunk.codec != 1 || chunk.schema != 1)
            throw std::runtime_error("A model section (\"" + tagName(chunk.type) + "\") uses an unsupported encoding.");

        reader.read(chunk, [&](std::istream& stream, std::uint64_t bytes) {
            core::BinaryReader in(stream, bytes);

            if (chunk.type == SummaryChunk) {
                readSingle(in, [&](core::BinaryReader& record) {
                    record.text();
                    document.game = record.text();
                });
                return;
            }
            if (chunk.type == DefinitionChunk) {
                readSingle(in, [&](core::BinaryReader& record) { document.definition = readDefinition(record); });
                in.finish();
                return;
            }
            if (chunk.type == PhysicsSettingsChunk) {
                readSingle(in, [&](core::BinaryReader& record) { document.physics = readPhysicsSettings(record); });
                in.finish();
                return;
            }
            if (chunk.type == LodSettingsChunk) {
                readSingle(in, [&](core::BinaryReader& record) { document.lodSettings = readLodSettings(record); });
                in.finish();
                return;
            }
            if (chunk.type == BlobChunk) {
                readSingle(in, [&](core::BinaryReader& record) {
                    const auto uid = record.read<Uid>();
                    blobs[uid] = record.text();
                });
                return;
            }

            if (chunk.type == SourcesChunk) {
                readRecords(in, "source", [&](core::BinaryReader& record) { document.sources.push_back(readSource(record)); });
            } else if (chunk.type == MeshesChunk) {
                readRecords(in, "mesh", [&](core::BinaryReader& record) { document.meshes.push_back(readMesh(record)); });
            } else if (chunk.type == GroupsChunk) {
                readRecords(in, "model group", [&](core::BinaryReader& record) { document.groups.push_back(readGroup(record)); });
            } else if (chunk.type == MaterialsChunk) {
                readRecords(in, "material markup", [&](core::BinaryReader& record) { readMaterialRecord(record, document.materials); });
            } else if (chunk.type == BonesChunk) {
                readRecords(in, "bone definition", [&](core::BinaryReader& record) { document.bones.push_back(readBoneDefinition(record)); });
            } else if (chunk.type == RealignsChunk) {
                readRecords(in, "realign", [&](core::BinaryReader& record) { document.realigns.push_back(readRealign(record)); });
            } else if (chunk.type == PhysicsNodesChunk) {
                readRecords(in, "physics", [&](core::BinaryReader& record) { document.physicsNodes.push_back(readPhysicsNode(record)); });
            } else if (chunk.type == BoneMarkupChunk) {
                readRecords(in, "bone markup", [&](core::BinaryReader& record) { document.boneMarkups.push_back(readBoneMarkup(record)); });
            } else if (chunk.type == HitboxSetsChunk) {
                readRecords(in, "hitbox set", [&](core::BinaryReader& record) {
                    HitboxSet set;
                    set.uid = record.read<Uid>();
                    set.name = record.text();
                    set.enabled = record.read<bool>();
                    if (!set.uid || document.findHitboxSet(set.uid)) throw std::runtime_error("Invalid hitbox set ID.");
                    document.hitboxSets.push_back(std::move(set));
                });
            } else if (chunk.type == HitboxesChunk) {
                readRecords(in, "hitbox", [&](core::BinaryReader& record) {
                    auto hitbox = readHitbox(record);
                    if (!hitboxIds.insert(hitbox.box.uid).second) throw std::runtime_error("Duplicate hitbox ID.");
                    hitboxes.push_back(std::move(hitbox));
                });
            } else if (chunk.type == FoldersChunk) {
                readRecords(in, "folder", [&](core::BinaryReader& record) {
                    auto folder = readFolder(record);
                    document.folders.push_back(folder.folder);
                    folders.push_back(std::move(folder));
                });
            } else if (chunk.type == AttachmentsChunk) {
                readRecords(in, "attachment", [&](core::BinaryReader& record) { document.attachments.push_back(readAttachment(record)); });
            } else if (chunk.type == EyeballsChunk) {
                readRecords(in, "eyeball", [&](core::BinaryReader& record) { document.eyeballs.push_back(readEyeball(record)); });
            } else if (chunk.type == EyelidsChunk) {
                readRecords(in, "eyelid", [&](core::BinaryReader& record) { document.eyelids.push_back(readEyelid(record)); });
            } else if (chunk.type == PrefabsChunk) {
                readRecords(in, "prefab", [&](core::BinaryReader& record) { document.prefabs.push_back(readPrefab(record)); });
            } else if (chunk.type == LodsChunk) {
                readRecords(in, "LOD", [&](core::BinaryReader& record) { document.lods.push_back(readLod(record)); });
            } else if (chunk.type == FlexNodesChunk) {
                readRecords(in, "flex node", [&](core::BinaryReader& record) { document.flexNodes.push_back(readFlexNode(record)); });
            } else if (chunk.type == FlexLinksChunk) {
                readRecords(in, "flex link", [&](core::BinaryReader& record) { document.flexLinks.push_back(readFlexLink(record)); });
            } else if (chunk.type == FlexVarsChunk) {
                readRecords(in, "local var", [&](core::BinaryReader& record) { document.flexLocalVars.push_back(record.text()); });
            } else if (chunk.type == FlexWrinklesChunk) {
                readRecords(in, "wrinkle scale", [&](core::BinaryReader& record) {
                    FlexWrinkle wrinkle;
                    wrinkle.morph = record.text();
                    wrinkle.scale = record.read<float>();
                    if (!std::isfinite(wrinkle.scale)) throw std::runtime_error("Invalid wrinkle scale.");
                    document.flexWrinkles.push_back(std::move(wrinkle));
                });
            } else if (chunk.type == FlexSplitsChunk) {
                readRecords(in, "morph split", [&](core::BinaryReader& record) {
                    FlexSplit split;
                    split.morph = record.text();
                    split.factor = record.read<float>();
                    if (!std::isfinite(split.factor)) throw std::runtime_error("Invalid morph split factor.");
                    document.flexSplits.push_back(std::move(split));
                });
            } else if (chunk.type == AnimNodesChunk) {
                readRecords(in, "animation node", [&](core::BinaryReader& record) {
                    auto node = readAnimNode(record);
                    if (document.findAnimNode(node.uid)) throw std::runtime_error("Duplicate animation node ID.");
                    document.animNodes.push_back(std::move(node));
                });
            } else if (chunk.type == AnimLinksChunk) {
                readRecords(in, "animation link", [&](core::BinaryReader& record) { document.animLinks.push_back(readAnimLink(record)); });
            } else if (chunk.type == PoseParamsChunk) {
                readRecords(in, "pose parameter", [&](core::BinaryReader& record) {
                    auto param = readPoseParameter(record);
                    if (std::ranges::any_of(document.poseParameters, [&](const auto& other) { return other.uid == param.uid; }))
                        throw std::runtime_error("Duplicate pose parameter ID.");
                    document.poseParameters.push_back(std::move(param));
                });
            }
            in.finish();
        });
    }

    // Morph Split nodes of the first Flex Graph files became per-morph splits; the first node for a morph wins.
    const auto sameMorph = [](std::string_view a, std::string_view b) {
        return std::ranges::equal(a, b, [](unsigned char x, unsigned char y) { return std::tolower(x) == std::tolower(y); });
    };
    for (const auto& node : document.flexNodes) {
        if (node.type != FlexNodeType::MorphSplit || node.muted || node.name.empty()) continue;

        const bool listed = std::ranges::any_of(document.flexSplits, [&](const FlexSplit& split) { return sameMorph(split.morph, node.name); });
        if (!listed) document.flexSplits.push_back({node.name, node.values[0]});
    }
    std::erase_if(document.flexNodes, [](const FlexNode& node) { return node.type == FlexNodeType::MorphSplit; });

    for (auto& source : document.sources) {
        const auto blob = blobs.find(source.uid);
        if (blob == blobs.end())
            throw std::runtime_error("The source \"" + source.path + "\" has no embedded data. Open and save it in PulseStudio.");

        source.bytes = std::make_shared<const std::string>(std::move(blob->second));
    }

    attachHitboxes(document, hitboxes);

    // The stored switch of every node is its own; the folders then decide the effective one.
    const auto keepOwnSwitch = [](auto& nodes) {
        for (auto& node : nodes) node.userEnabled = node.enabled;
    };
    keepOwnSwitch(document.meshes);
    keepOwnSwitch(document.groups);
    keepOwnSwitch(document.bones);
    keepOwnSwitch(document.realigns);
    keepOwnSwitch(document.attachments);
    keepOwnSwitch(document.hitboxSets);
    keepOwnSwitch(document.physicsNodes);
    keepOwnSwitch(document.boneMarkups);
    attachFolders(document, folders);
    document.refreshEnabled();
    return document;
}

} // namespace

const SourceFile* Document::findSource(Uid uid) const
{
    const auto found = std::ranges::find(sources, uid, &SourceFile::uid);
    return found == sources.end() ? nullptr : &*found;
}

const MeshEntry* Document::findMesh(Uid uid) const
{
    const auto found = std::ranges::find(meshes, uid, &MeshEntry::uid);
    return found == meshes.end() ? nullptr : &*found;
}

const Folder* Document::findFolder(Uid uid) const
{
    const auto found = std::ranges::find(folders, uid, &Folder::uid);
    return found == folders.end() ? nullptr : &*found;
}

const AnimNode* Document::findAnimNode(Uid uid) const
{
    const auto found = std::ranges::find(animNodes, uid, &AnimNode::uid);
    return found == animNodes.end() ? nullptr : &*found;
}

const FlexNode* Document::findFlexNode(Uid uid) const
{
    const auto found = std::ranges::find(flexNodes, uid, &FlexNode::uid);
    return found == flexNodes.end() ? nullptr : &*found;
}

HitboxSet* Document::findHitboxSet(Uid uid)
{
    const auto found = std::ranges::find(hitboxSets, uid, &HitboxSet::uid);
    return found == hitboxSets.end() ? nullptr : &*found;
}

bool Document::folderActive(Uid uid) const
{
    for (auto depth = folders.size() + 1; uid && depth; --depth) {
        const auto* folder = findFolder(uid);
        if (!folder) return true;
        if (!folder->enabled) return false;

        uid = folder->parent;
    }
    return true;
}

void Document::refreshEnabled()
{
    visitFolderMembers(*this, [&](auto& node, FolderKind kind) {
        const auto* folder = node.folder ? findFolder(node.folder) : nullptr;
        if (folder && folder->kind != kind) node.folder = 0;
        node.enabled = node.userEnabled && folderActive(node.folder);
    });
}

Document LoadDocument(const std::string& path)
{
    core::ChunkReader reader(path);
    return readDocument(reader);
}

} // namespace pulse::mdlasm
