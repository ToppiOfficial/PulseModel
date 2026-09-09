// facemarkup.h - PulseMDL
//
// Eyeball / mouth / eyelid registration, shared by both front ends:
// `.pulsemdl`'s facemarkuplist and `.pulseqc`'s $eyeball / $mouth / $eyelid.
//
// These are TOP-LEVEL and GLOBAL in both formats, unlike the QC $model block
// options they replace. An eyeball binds to EVERY body carrying its material;
// each eyelid delta resolves against whichever body carries it.
//
// ORDER IS SIGNIFICANT across all three kinds, so they share one list: eyeball
// order fixes the eyeball index an Eyelid names, and Mouth and Eyelid both
// append to the global flexdesc table.
//
// QC passes this into RegisterFlex so model-block eyelids and mouths are
// registered before the manual rules that follow them.

#ifndef PULSEMDL_FACEMARKUP_H
#define PULSEMDL_FACEMARKUP_H

#include <string>
#include <vector>

#include "compile.h"

namespace pulse::loader {

struct FaceMarkup {
    // QC `eyeball <name> <bone> <x y z> <material> <diameter> <angle>
    // <irismaterial> <pupilscale>`; the iris material is dropped (the reference
    // parses and discards it).
    struct Eyeball {
        std::string name;
        std::string bonename; // the HEAD bone, not a dedicated eye bone
        std::string material; // matched by basename against the body's meshes
        math::Vector3 origin{};
        bool center = false;  // origin is an offset from the material's bbox center
        float diameter = 0.0f;
        float angle = 0.0f;      // degrees; stored as tan() by the compile stage
        float pupilscale = 0.0f; // stored as its reciprocal
    };

    // QC `mouth <index> <controller> <bone> <x y z>`; the index is implicit -
    // declaration order.
    struct Mouth {
        std::string controller; // registered as a flexdesc
        std::string bonename;
        math::Vector3 forward{};
    };

    // Lid poses are named, never a VTA frame index; the source is not named, so
    // each delta resolves on its own and the three need not share a mesh. An
    // empty delta means that pose has no vertex data.
    //
    // Stereo (righteyeball + lefteyeball) covers both eyes with one
    // <type>_right/<type>_left desc pair. Mono (eyeball + basedesc) is one entry
    // per eye with its own lid desc and targets - what a v44-48 model has.
    struct Eyelid {
        bool upper = true;
        std::string delta[3];  // lowerer, neutral, raiser
        float target[3] = {0.0f, 0.0f, 0.0f}; // scaled by RegisterFaceMarkup
        std::string basedesc; // mono only: the single lid flexdesc
        std::string eyeball;  // mono only
        std::string righteyeball;
        std::string lefteyeball;
        // optional: keep only the deltas on one side of the midline, so one
        // delta can serve both eyes. 0 = whole delta.
        float split = 0.0f;
    };

    enum class Kind { Eyeball, Mouth, Eyelid };

    struct Entry {
        Kind kind = Kind::Eyeball;
        struct Eyeball eyeball;
        struct Mouth mouth;
        struct Eyelid eyelid;
    };

    std::vector<Entry> entries; // ORDER-SIGNIFICANT
};

// Register the face markup onto `in`. Returns false + err on failure.
bool RegisterFaceMarkup(compile::CompileInput& in, const FaceMarkup& markup, std::string* err);

} // namespace pulse::loader

#endif // PULSEMDL_FACEMARKUP_H
