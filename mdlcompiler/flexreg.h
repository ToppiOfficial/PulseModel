// flexreg.h - PulseMDL
//
// Flex / morph registration, shared by both front ends. The global flexdesc /
// flexcontroller / flexrule / flexkey tables are built here from two inputs:
//
//   1. every body's source contributes its delta states (morphs -> flexkeys),
//      plus the imported flex rig - a render mesh carries morphs only, and the
//      rig behind them comes from `.pulseqc`'s $datamodelflexes;
//   2. the script-authored ManualFlex block - `.pulsemdl`'s morphcontrollerlist
//      + morphrulelist, or `.pulseqc`'s $flexcontroller / $flexlocalvar /
//      $flexrule / $flexcorrective.
//
// Registration ORDER defines the table indices and the string table, so both
// front ends must go through RegisterFlex() to stay byte-identical.

#ifndef PULSEMDL_FLEXREG_H
#define PULSEMDL_FLEXREG_H

#include <string>
#include <vector>

#include "compile.h"

namespace pulse::loader {

// The script-authored flex block. Both front ends fill this and hand it to
// RegisterFlex(); nothing in here is format-specific.
struct ManualFlex {
    // one $flexcontroller name / MorphController element
    struct Controller {
        std::string name;
        std::string group = "default"; // the .mdl "type" (UI group)
        float min = 0.0f;
        float max = 1.0f;
    };

    // one morphrulelist entry. Exactly one form applies:
    //   localvar      - reserve a flexdesc, no rule ($flexlocalvar)
    //   !combo.empty  - corrective: FETCH each control, then COMBO n
    //   else          - infix expression, shunting-yarded to RPN ($flexrule)
    struct Rule {
        std::string name; // flexdesc the rule drives
        std::string expr;
        std::vector<std::string> combo; // bare = controller, %x = flexdesc
        bool localvar = false;
    };

    // $flexdominate / MorphDominationRule: appended to the target's rule after
    // every other rule exists, so the ops trail its COMBO
    struct Domination {
        std::string name;
        std::vector<std::string> dominators; // bare = controller, %x = flexdesc
    };

    // $morphsplitstereo: flag an encoded delta state stereo. `factor` != 0
    // synthesizes the per-vertex balance the mesh does not carry, off the base
    // vertex X - the primitive equivalent of a painted DMX balance map.
    struct StereoSplit {
        std::string name;
        float factor = 0.0f;
    };

    std::vector<Controller> controllers;
    std::vector<Rule> rules; // ORDER-SIGNIFICANT
    std::vector<StereoSplit> stereoSplits;
    std::vector<Domination> dominations;

    // $datamodelflexes, merged across every one in the script and already
    // filtered by its inline import list. Stamped onto every render mesh that
    // carries a matching morph before any registration runs, so from there on
    // the pass is the same as when a source carried its own rig.
    source::FlexRig datamodel;
};

// Walk in.bodyparts and build the global flex tables on `in`. Only sources
// actually used by a body contribute - a render mesh that carries delta states
// but is never referenced registers nothing. Returns false + err on failure.
bool RegisterFlex(compile::CompileInput& in, const ManualFlex& manual, std::string* err);

// reference Add_Flexdesc: find (stricmp) or append. Exposed for the face-markup
// pass, which registers eyelid/mouth descs of its own after RegisterFlex.
// Returns the index, or -1 + err past kMaxFlexDesc.
int AddFlexdesc(compile::CompileInput& in, const std::string& name, std::string* err);

} // namespace pulse::loader

#endif // PULSEMDL_FLEXREG_H
