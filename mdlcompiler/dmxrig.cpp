// dmxrig.cpp - $datamodeljoints DME bone markup reader. See dmxrig.h.

#include "dmxrig.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace pulse::loader {

namespace cm = pulse::compile;
namespace pm = pulse::math;
namespace lim = pulse::limits;

namespace {

// The reference converts through DOUBLE; a float pi/180 is off by 1 ULP on
// values like 35 deg and that lands in the .mdl (same helper as qcloader).
float DegToRad(float deg) {
    return static_cast<float>(deg * 3.14159265358979323846 / 180.0);
}

float Clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

pm::Vector3 ToVec(const dmx::Vector3& v) { return {v.x, v.y, v.z}; }

// DmeJiggleBone. The struct defaults in compile.h already match the reference's
// seed values, so only the attributes the element carries are read.
void AddJiggleBone(const dmx::Element* dag, cm::CompileInput& in) {
    for (const cm::JiggleBone& prev : in.jigglebones)
        if (_stricmp(prev.bonename.c_str(), dag->name.c_str()) == 0)
            return; // first definition wins
    if (in.jigglebones.size() >= static_cast<size_t>(lim::kMaxJiggleBones)) {
        std::fprintf(stderr, "warning: too many jigglebones, ignoring \"%s\"\n",
                     dag->name.c_str());
        return;
    }

    cm::JiggleBone jb;
    jb.bonename = dag->name;

    jb.length = dag->GetFloat("length", 10.0f);
    jb.tipMass = dag->GetFloat("tipMass", 0.0f);
    if (dag->GetBool("lengthConstrained", false))
        jb.flags |= cm::kJiggleHasLengthConstraint;
    if (dag->GetBool("angleConstrained", false))
        jb.flags |= cm::kJiggleHasAngleConstraint;
    jb.angleLimit = DegToRad(dag->GetFloat("angleLimit", 0.0f));

    if (dag->GetBool("yawConstrained", false))
        jb.flags |= cm::kJiggleHasYawConstraint;
    jb.minYaw = DegToRad(dag->GetFloat("yawMin", 0.0f));
    jb.maxYaw = DegToRad(dag->GetFloat("yawMax", 0.0f));
    jb.yawFriction = dag->GetFloat("yawFriction", 0.0f);
    jb.yawBounce = dag->GetFloat("yawBounce", 0.0f);

    jb.minPitch = DegToRad(dag->GetFloat("pitchMin", 0.0f));
    jb.maxPitch = DegToRad(dag->GetFloat("pitchMax", 0.0f));
    jb.pitchFriction = dag->GetFloat("pitchFriction", 0.0f);
    jb.pitchBounce = dag->GetFloat("pitchBounce", 0.0f);

    const bool flexible = dag->GetBool("flexible", false);
    const bool rigid = dag->GetBool("rigid", false);
    if (flexible) {
        if (rigid)
            std::fprintf(stderr, "warning: jigglebone %s: both flexible and rigid set, "
                                 "ignoring rigid\n", jb.bonename.c_str());
        jb.flags |= cm::kJiggleIsFlexible;
        if (dag->GetBool("pitchConstrained", false))
            jb.flags |= cm::kJiggleHasPitchConstraint;

        // stiffness [0,1000], damping [0,10] - the DMX path's own ranges
        // (HandleDmeJiggleBone), tighter than $jigglebone's
        jb.yawStiffness = Clamp(dag->GetFloat("yawStiffness", 100.0f), 0.0f, 1000.0f);
        jb.yawDamping = Clamp(dag->GetFloat("yawDamping", 0.0f), 0.0f, 10.0f);
        jb.pitchStiffness = Clamp(dag->GetFloat("pitchStiffness", 100.0f), 0.0f, 1000.0f);
        jb.pitchDamping = Clamp(dag->GetFloat("pitchDamping", 0.0f), 0.0f, 10.0f);
        jb.alongStiffness = Clamp(dag->GetFloat("alongStiffness", 100.0f), 0.0f, 1000.0f);
        jb.alongDamping = Clamp(dag->GetFloat("alongDamping", 0.0f), 0.0f, 10.0f);
    } else if (rigid) {
        jb.flags |= cm::kJiggleIsRigid | cm::kJiggleHasLengthConstraint;
    }

    if (dag->GetBool("baseSpring", false)) {
        jb.flags |= cm::kJiggleHasBaseSpring;
        jb.baseMass = dag->GetFloat("baseMass", 0.0f);
        jb.baseStiffness = Clamp(dag->GetFloat("baseStiffness", 100.0f), 0.0f, 1000.0f);
        jb.baseDamping = Clamp(dag->GetFloat("baseDamping", 0.0f), 0.0f, 10.0f);

        jb.baseMinLeft = dag->GetFloat("baseYawMin", -100.0f);
        jb.baseMaxLeft = dag->GetFloat("baseYawMax", 100.0f);
        jb.baseLeftFriction = dag->GetFloat("baseYawFriction", 0.0f);

        jb.baseMinUp = dag->GetFloat("basePitchMin", -100.0f);
        jb.baseMaxUp = dag->GetFloat("basePitchMax", 100.0f);
        jb.baseUpFriction = dag->GetFloat("basePitchFriction", 0.0f);

        jb.baseMinForward = dag->GetFloat("baseAlongMin", -100.0f);
        jb.baseMaxForward = dag->GetFloat("baseAlongMax", 100.0f);
        jb.baseForwardFriction = dag->GetFloat("baseAlongFriction", 0.0f);
    }

    if (dag->GetBool("boing", false)) {
        jb.flags |= cm::kJiggleIsBoing;
        jb.boingImpactSpeed = dag->GetFloat("boingImpactSpeed", 100.0f);
        // authored in degrees, stored as a cosine
        jb.boingImpactAngle = std::cos(DegToRad(dag->GetFloat("boingImpactAngle", 45.0f)));
        jb.boingDampingRate = dag->GetFloat("boingDampingRate", 0.25f);
        jb.boingFrequency = dag->GetFloat("boingFrequency", 30.0f);
        jb.boingAmplitude = dag->GetFloat("boingAmplitude", 0.35f);
    }

    in.jigglebones.push_back(std::move(jb));
}

// DmeQuatInterpBone - the DMX form of $driverbone. The dag itself is the helper
// bone; parent names stay empty and are filled from the final skeleton by
// MapProceduralBones. `unlockBones` picks which form the pose is in: set, it is
// a delta from the bind pose and the compile stage folds the bind pose in;
// clear - the usual export - basePos IS the helper's rest position, so the pose
// is already absolute parent-relative and folding again would double the
// helper's offset from its parent.
void AddQuatInterpBone(const dmx::Element* dag, float scale, cm::CompileInput& in) {
    const std::string& name = dag->name;
    for (const cm::ProceduralBone& prev : in.proceduralbones)
        if (_stricmp(prev.helpername.c_str(), name.c_str()) == 0)
            return;

    const std::string* control = dag->GetString("controlBone");
    if (!control || control->empty()) {
        std::fprintf(stderr, "warning: DmeQuatInterpBone \"%s\" has no controlBone, skipping\n",
                     name.c_str());
        return;
    }

    const std::vector<float>* tolerances = dag->GetFloatArray("tolerances");
    const std::vector<dmx::Quaternion>* triggers = dag->GetQuaternionArray("triggerRotations");
    const std::vector<dmx::Vector3>* positions = dag->GetVector3Array("targetPositions");
    const std::vector<dmx::Quaternion>* rotations = dag->GetQuaternionArray("targetRotations");
    const size_t count = triggers ? triggers->size() : 0;
    if (count == 0) {
        std::fprintf(stderr, "warning: DmeQuatInterpBone \"%s\" has no triggers, skipping\n",
                     name.c_str());
        return;
    }
    if (!tolerances || tolerances->size() != count || !positions || positions->size() != count ||
        !rotations || rotations->size() != count) {
        std::fprintf(stderr, "warning: DmeQuatInterpBone \"%s\" trigger arrays are mismatched, "
                             "skipping\n", name.c_str());
        return;
    }
    if (in.proceduralbones.size() >= static_cast<size_t>(lim::kMaxProceduralBones)) {
        std::fprintf(stderr, "warning: too many procedural bones, ignoring \"%s\"\n",
                     name.c_str());
        return;
    }

    cm::ProceduralBone pb;
    pb.helpername = name;
    pb.drivername = *control;
    pb.absolutePose = !dag->GetBool("unlockBones", false);

    const dmx::Vector3 basePos = dag->GetVector3("basePos");
    for (size_t t = 0; t < count; ++t) {
        if (pb.triggers.size() >= static_cast<size_t>(lim::kMaxProceduralTriggers)) {
            std::fprintf(stderr, "animconstraint \"%s\": more than %d triggers; "
                                 "dropping the rest\n",
                         name.c_str(), lim::kMaxProceduralTriggers);
            break;
        }
        float tolDeg = (*tolerances)[t];
        if (tolDeg <= 0.0f) {
            // the writer stores 1/tolerance, so zero would put an inf on disk
            std::fprintf(stderr, "warning: DmeQuatInterpBone \"%s\" trigger %zu has a "
                                 "non-positive tolerance, using 1.0\n", name.c_str(), t);
            tolDeg = 1.0f;
        }
        cm::ProceduralBoneTrigger tr;
        tr.tolerance = DegToRad(tolDeg);
        const dmx::Quaternion& q = (*triggers)[t];
        tr.trigger = {q.x, q.y, q.z, q.w};
        const dmx::Quaternion& tq = (*rotations)[t];
        tr.quat = {tq.x, tq.y, tq.z, tq.w};
        // basePos is added before the scale, like `(basePos + pos) * g_currentscale`
        const dmx::Vector3& p = (*positions)[t];
        tr.pos = {(basePos.x + p.x) * scale, (basePos.y + p.y) * scale,
                  (basePos.z + p.z) * scale};
        pb.triggers.push_back(tr);
    }

    in.proceduralbones.push_back(std::move(pb));
}

// DmeAimAtBone - the DMX form of $driveraimat. basePos is the full base
// position already, so autobasepos stays false and the compile stage must not
// re-add the bone's rest pose on top of it.
void AddAimAtBone(const dmx::Element* dag, float scale, cm::CompileInput& in) {
    const std::string& name = dag->name;
    for (const cm::AimAtBone& prev : in.aimatbones)
        if (_stricmp(prev.bonename.c_str(), name.c_str()) == 0)
            return;

    const std::string* target = dag->GetString("aimTarget");
    if (!target || target->empty()) {
        std::fprintf(stderr, "warning: DmeAimAtBone \"%s\" has no aimTarget, skipping\n",
                     name.c_str());
        return;
    }
    if (in.aimatbones.size() >= static_cast<size_t>(lim::kMaxProceduralBones)) {
        std::fprintf(stderr, "warning: too many procedural bones, ignoring \"%s\"\n",
                     name.c_str());
        return;
    }

    cm::AimAtBone ab;
    ab.bonename = name;
    ab.aimname = *target;
    if (const std::string* parent = dag->GetString("parentBone"))
        ab.parentname = *parent; // empty = derived from the resolved skeleton

    ab.aimvector = ToVec(dag->GetVector3("aimVector", {0.0f, 0.0f, 1.0f}));
    ab.upvector = ToVec(dag->GetVector3("upVector", {1.0f, 0.0f, 0.0f}));
    pm::VectorNormalize(ab.aimvector);
    pm::VectorNormalize(ab.upvector);

    const pm::Vector3 base = ToVec(dag->GetVector3("basePos"));
    ab.basepos = {base.x * scale, base.y * scale, base.z * scale};
    ab.autobasepos = false;

    in.aimatbones.push_back(std::move(ab));
}

// DmeTransform -> matrix, translation scaled like the DMX skeleton loader's.
pm::matrix3x4 DagLocal(const dmx::Element* dag, float scale) {
    const dmx::Element* t = dag->GetElement("transform");
    if (!t)
        return pm::matrix3x4();
    pm::Quaternion rot{0, 0, 0, 1};
    if (const dmx::Attribute* a = t->Get("orientation"))
        if (auto q = std::get_if<dmx::Quaternion>(&a->value))
            rot = {q->x, q->y, q->z, q->w};
    const pm::Vector3 p = ToVec(t->GetVector3("position"));
    return pm::QuaternionMatrix(rot, {p.x * scale, p.y * scale, p.z * scale});
}

// A DMX attachment is a dag whose SHAPE is a DmeAttachment: the shape names the
// attachment. The BONE is the dag itself when the shape hangs on a joint, and
// otherwise the nearest ancestor joint, with the plain-dag chain below it baked
// into the local transform - an exporter that writes attachments as locators
// parented under the joint would otherwise link to a bone that only exists if
// this same DMX also supplies the mesh (every dag becomes one there, which is
// all reference LoadAttachments relies on). The static-prop case needs no
// special handling, because LinkAttachments already rebases onto the surviving
// bone when the named one collapses.
void WalkAttachments(const dmx::Element* dag, float scale, cm::CompileInput& in,
                     const std::string& boneName, const pm::matrix3x4& boneToDag) {
    const bool isJoint = dag->className == "DmeJoint";
    const std::string& bone = isJoint ? dag->name : boneName;
    const pm::matrix3x4 local =
        isJoint ? pm::matrix3x4() : pm::ConcatTransforms(boneToDag, DagLocal(dag, scale));

    const dmx::Element* shape = dag->GetElement("shape");
    if (shape && shape->className == "DmeAttachment") {
        cm::Attachment att;
        att.name = shape->name;
        att.bonename = bone;
        att.local = local;
        att.type = cm::kAttachIsFromSource;
        if (shape->GetBool("isRigid", false))
            att.type |= cm::kAttachIsRigid;
        if (shape->GetBool("isWorldAligned", false))
            att.flags |= cm::kAttachFlagWorldAlign;
        in.attachments.push_back(std::move(att));
    }

    if (auto kids = dag->GetElementArray("children"))
        for (const dmx::Element* c : *kids)
            if (c)
                WalkAttachments(c, scale, in, bone, local);
}

// root."hitboxSetList" -> DmeHitboxSetList."hitboxSetList" -> DmeHitboxSet
// ("hitboxList" of DmeHitbox). Reference LoadDmxHitboxes: hitboxes are
// model-global and first-to-populate wins, so an existing $hitboxset makes the
// whole DMX list a no-op. An EMPTY set is skipped - a stray "default" set would
// suppress the compile stage's auto-generation.
void AddHitboxes(const dmx::Element* root, cm::CompileInput& in) {
    const dmx::Element* list = root->GetElement("hitboxSetList");
    if (!list)
        return;
    const std::vector<dmx::ElementPtr>* sets = list->GetElementArray("hitboxSetList");
    if (!sets || sets->empty())
        return;
    if (!in.hitboxsets.empty()) {
        std::fprintf(stderr, "warning: DMX hitboxSetList ignored - hitbox sets are already "
                             "defined ($hitboxset or an earlier $datamodeljoints)\n");
        return;
    }

    for (const dmx::Element* set : *sets) {
        if (!set) continue;
        const std::vector<dmx::ElementPtr>* boxes = set->GetElementArray("hitboxList");
        if (!boxes || boxes->empty())
            continue;
        if (in.hitboxsets.size() >= static_cast<size_t>(lim::kMaxHitboxSets)) {
            std::fprintf(stderr, "warning: too many hitbox sets, ignoring \"%s\"\n",
                         set->name.c_str());
            break;
        }

        cm::HitboxSet hs;
        hs.name = set->name;
        for (const dmx::Element* box : *boxes) {
            if (!box) continue;
            if (hs.hitboxes.size() >= static_cast<size_t>(lim::kMaxHitboxesPerSet)) {
                std::fprintf(stderr, "warning: too many hitboxes for hitbox set \"%s\", max %d\n",
                             hs.name.c_str(), lim::kMaxHitboxesPerSet);
                break;
            }
            cm::HitBox hb;
            if (const std::string* bone = box->GetString("boneName"))
                hb.bonename = *bone;
            hb.group = box->GetInt("groupId", 0);
            // scaled like Cmd_Hitbox's scale_vertex, so DMX hitboxes respect $scale
            const dmx::Vector3 lo = box->GetVector3("minBounds");
            const dmx::Vector3 hi = box->GetVector3("maxBounds");
            hb.bmin = {lo.x * in.scale, lo.y * in.scale, lo.z * in.scale};
            hb.bmax = {hi.x * in.scale, hi.y * in.scale, hi.z * in.scale};
            hb.capsuleRadius = box->GetFloat("radius", -1.0f); // <= 0 = a box
            hb.angOffset = ToVec(box->GetVector3("orientation"));
            // the DmeHitbox element's own name is not the hitbox name - the
            // reference leaves these unnamed, like an $hbox with no `name`
            hs.hitboxes.push_back(std::move(hb));
        }
        in.hitboxsets.push_back(std::move(hs));
    }
}

// Walk the dag tree the way the skeleton load does (reference AddDagJoint): a
// dag with no transform is not a joint, and neither it nor its subtree counts.
void WalkDag(const dmx::Element* dag, float scale, cm::CompileInput& in, bool jigglebones,
             bool proceduralbones) {
    if (!dag->GetElement("transform"))
        return;

    if (jigglebones && dag->className == "DmeJiggleBone")
        AddJiggleBone(dag, in);
    if (proceduralbones) {
        if (dag->className == "DmeQuatInterpBone")
            AddQuatInterpBone(dag, scale, in);
        else if (dag->className == "DmeAimAtBone")
            AddAimAtBone(dag, scale, in);
    }

    if (auto kids = dag->GetElementArray("children"))
        for (const dmx::Element* c : *kids)
            if (c)
                WalkDag(c, scale, in, jigglebones, proceduralbones);
}

} // namespace

bool LoadDmxJoints(const dmx::Datamodel& dm, cm::CompileInput& in, bool jigglebones,
                   bool proceduralbones, bool hitboxes, bool attachments, std::string* err) {
    const dmx::Element* root = dm.root;
    if (!root) {
        if (err) *err = "DMX has no root element";
        return false;
    }
    const dmx::Element* skeleton = root->GetElement("skeleton");
    if (!skeleton) skeleton = root->GetElement("model");
    if (!skeleton) {
        if (err) *err = "no 'model'/'skeleton' element on root";
        return false;
    }

    // the root dag is never a joint (or an attachment) itself - start at its
    // children
    if (const auto* kids = skeleton->GetElementArray("children")) {
        for (const dmx::Element* c : *kids) {
            if (!c) continue;
            if (jigglebones || proceduralbones)
                WalkDag(c, in.scale, in, jigglebones, proceduralbones);
            if (attachments)
                WalkAttachments(c, in.scale, in, std::string(), pm::matrix3x4());
        }
    }
    if (hitboxes)
        AddHitboxes(root, in);
    return true;
}

} // namespace pulse::loader
