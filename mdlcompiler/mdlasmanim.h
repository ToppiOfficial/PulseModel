// mdlasmanim.h - the animations and sequences an Animation Graph writes.

#ifndef PULSEMODEL_MDLASMANIM_H
#define PULSEMODEL_MDLASMANIM_H

#include <string>
#include <vector>

#include "compile.h"
#include "mdlasmdoc.h"

namespace pulse::mdlasm {

struct Problem
{
    Uid node = 0;
    std::string message;
};

struct AnimScriptAnim
{
    compile::CompileInput::InAnim anim;
    Uid node = 0;   // the Animation or Clip node the animation comes from
    Uid source = 0; // the embedded source it reads, 0 for the bind pose
};

struct AnimScriptSequence
{
    compile::CompileInput::InSequence sequence;
    Uid node = 0;               // the Sequence or Declare Sequence node
    std::vector<Uid> members;   // every node the sequence draws on, for the highlight
};

struct AnimScriptInclude
{
    std::string path; // below models/, as `$includemodel` takes it
    Uid node = 0;
};

struct AnimScript
{
    // In Animation Out order, which is the order the included models are merged in.
    std::vector<AnimScriptInclude> includeModels;
    std::vector<compile::WeightList> weightLists;
    std::vector<compile::PoseParam> poseParams;
    // Transition node names; a sequence's entry and exit node are 1-based indexes into this list.
    std::vector<std::string> xnodes;
    std::vector<AnimScriptAnim> anims;
    std::vector<AnimScriptSequence> sequences;
    std::vector<Problem> problems;
};

// The graph as the compiler reads it. Only what Animation Out reaches is written: its order, then what the
// sequences name (layers) after it. A muted node is left out.
AnimScript emitAnimScript(const Document& document);

// The nodes a Sequence or Animation reads, upstream of `uid` (not including it).
std::vector<Uid> animUpstream(const Document& document, Uid uid);

// Whether the command takes a second clip (its reference animation) as an input.
bool animCommandHasReference(const AnimCommand& command);

} // namespace pulse::mdlasm

#endif // PULSEMODEL_MDLASMANIM_H
