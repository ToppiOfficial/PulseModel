// scriptapply.h - the semantic half of the compile-script commands, shared by the .qc loader
// (which parses tokens into these values) and the .mdlasm loader (which builds them from a document).

#ifndef PULSEMODEL_SCRIPTAPPLY_H
#define PULSEMODEL_SCRIPTAPPLY_H

#include <string>

#include "dmxrig.h"
#include "scriptctx.h"

namespace pulse::loader::detail {

// $rendermesh: loads the source, runs its decimate and edit blocks, registers it by `name`.
bool ApplyRenderMesh(Ctx& c, int startLine, const std::string& name, const std::string& file,
                     MeshEdit& edit);

// One choice of a $modelgroup, appended to `part`.
bool ApplyModelChoice(Ctx& c, int line, const std::string& where,
                      const std::vector<source::Source*>& refs,
                      const std::vector<std::string>& refNames,
                      const std::vector<std::string>& defName, bool named,
                      const std::string& studio, cm::CompileInput::InBodyPart& part);

// $modelname.
bool ApplyModelName(Ctx& c, int line, const std::string& name);
// $illumposition: the position as written plus the bone it follows (empty = static).
bool ApplyIllumPosition(Ctx& c, int line, const pm::Vector3& pos, const std::string& bone);
// $renamematerial and $collapsematerial.
bool ApplyRenameMaterial(Ctx& c, int line, const std::string& from, std::string to);
bool ApplyCollapseMaterial(Ctx& c, int line, std::string from, std::string to);

// Bone commands. FindOrAddMarkup is the shared per-bone entry that $bonemerge, $donotcollapse and
// $norealign write their flags into.
bool ApplyDefineBone(Ctx& c, int line, cm::ImportBone ib);
bool ApplyRealignBone(Ctx& c, int line, const std::string& name, const cm::BoneRealignEdit& edit);
cm::BoneMarkup& FindOrAddMarkup(Ctx& c, const std::string& name);

// Attachments, hitboxes and bone markup. AddTrigger appends one $driverbone trigger (degrees in,
// scaled position).
bool ApplyAttachment(Ctx& c, Ctx::PendingAttachment a);
bool ApplyHbox(Ctx& c, int line, cm::HitboxSet& set, cm::HitBox hb);
bool ApplyHitboxSet(Ctx& c, int line, cm::HitboxSet set);
bool ApplyJiggleBone(Ctx& c, int line, cm::JiggleBone jb);
bool ApplyProceduralBone(Ctx& c, int line, cm::ProceduralBone pb);
bool ApplyAimAtBone(Ctx& c, int line, cm::AimAtBone ab);
bool ApplyIkChain(Ctx& c, int line, cm::IkChain chain);
bool ApplyIkAutoplayLock(Ctx& c, int line, cm::IkLock lock);
bool AddTrigger(Ctx& c, const Token& cmd, cm::ProceduralBone& pb, float tolDeg,
                const pm::Vector3& driverRotDeg, const pm::Vector3& helperRotDeg,
                const pm::Vector3& pos, bool absolutePose);

// Animations and sequences. ApplyAnimation registers a named $animation, AddImpliedAnimation one only a $sequence
// owns (it returns the index into the animation list). ApplySequence takes the indexes of its blend animations;
// `inherit` copies anim 0's settings onto the implied ones after it, as the .qc reader does.
bool ApplyWeightList(Ctx& c, int line, cm::WeightList list);
bool ApplyPoseParameter(Ctx& c, int line, const cm::PoseParam& param);
bool ApplyAnimation(Ctx& c, int line, cm::CompileInput::InAnim anim);
int AddImpliedAnimation(Ctx& c, cm::CompileInput::InAnim anim);
bool ApplySequence(Ctx& c, int line, cm::CompileInput::InSequence seq, std::vector<int> blends, bool inherit);
// Loads `file` (an embedded source's virtual name or a path) as an animation source.
source::Source* LoadAnimationSource(Ctx& c, int line, const std::string& file);

// A $lod / $shadowlod block, and the model lookup its rules use: a render mesh name, a source
// file name or a fused choice by its name.
bool ApplyLod(Ctx& c, int line, cm::ScriptLod lod);
source::Source* FindModelSource(Ctx& c, const std::string& name);

// Physics shapes, and the two rig imports a prefab makes.
bool ApplyPhysShape(Ctx& c, int line, const std::string& mode, const std::string& ref,
                    const std::string& importType, cm::PhysicsShape& sh);
bool ApplyPhysShapeAdd(Ctx& c, int line, cm::PhysicsShape sh);
bool ApplyDataModelJoints(Ctx& c, int line, const std::string& file, bool jiggle, bool procedural,
                          bool hitboxes, bool attachments, const HitboxImport* hitboxImport = nullptr);
bool ApplyDataModelPhysics(Ctx& c, int line, const std::string& file);

// The passes that need the whole script read; run once after the last command.
bool FinishScript(Ctx& c, cm::CompileInput& out, std::string* err);

} // namespace pulse::loader::detail

#endif // PULSEMODEL_SCRIPTAPPLY_H
