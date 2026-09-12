// flexrig.h - rebuilds the DMX combination operator the .mdl's flex rules came
// from: input controls, correctives and domination rules.

#ifndef MDLDECOMPILER_FLEXRIG_H
#define MDLDECOMPILER_FLEXRIG_H

#include <set>
#include <string>
#include <vector>

#include "mdlfile.h"

namespace mdldecompiler {

// One DmeCombinationInputControl. `rawControls` is slot order: one name for a
// passthru, two for a 2-way, n for an n-way.
struct RigControl {
    std::string name;
    std::vector<std::string> rawControls;
    bool stereo = false;
    bool eyelid = false;
    std::string eyesUpDownFlex; // eyelid only, the controller its window tracks
    bool hasRange = false;
    float min = 0.0f, max = 1.0f;
};

// A delta state driven by a combination of raw controls. One raw control is the
// ordinary "this slider drives this shape" case.
struct RigCorrective {
    std::string delta;
    std::vector<std::string> combo;
    std::vector<std::vector<std::string>> dominators; // as read, for verification
};

// DmeCombinationDominationRule: `dominators` suppress any delta whose
// combination contains all of `suppressed`.
struct RigDomination {
    std::vector<std::string> dominators;
    std::vector<std::string> suppressed;
};

struct FlexRig {
    std::vector<RigControl> controls;
    std::vector<RigCorrective> correctives;
    std::vector<RigDomination> dominations;
    std::set<int> controllers; // flex controllers the rig recreates on compile
    std::set<int> descs;       // flexdescs whose rule the rig rebuilds
    int domMismatch = 0;       // correctives whose dominators did not round trip

    bool empty() const { return correctives.empty(); }
};

FlexRig BuildFlexRig(const Mdl& m);

} // namespace mdldecompiler

#endif // MDLDECOMPILER_FLEXRIG_H
