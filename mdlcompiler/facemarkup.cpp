// facemarkup.cpp - eyeball / mouth / eyelid registration. See facemarkup.h.
//
// Ports of Option_Eyeball, Option_Mouth and Option_DmxEyelid.

#include "facemarkup.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "flexreg.h"

namespace pulse::loader {

namespace cm = pulse::compile;
namespace lim = pulse::limits;

namespace {

const char* const kLidSuffix[3] = {"lowerer", "neutral", "raiser"};

// Option_Eyeball: radius is HALF the diameter, zoffset is the raw
// angle (the compile stage takes tan()), iris_scale is the raw pupil scale (the
// compile stage takes the reciprocal). Model scale is applied there too, where
// g_currentscale is known.
bool RegisterEyeball(cm::CompileInput& in, const FaceMarkup::Eyeball& src, std::string* err) {
    if (src.bonename.empty()) {
        if (err) *err = "eyeball \"" + src.name + "\" has no bone";
        return false;
    }
    if (src.material.empty()) {
        if (err) *err = "eyeball \"" + src.name + "\" has no material";
        return false;
    }
    cm::Eyeball eye;
    eye.name = src.name;
    eye.bonename = src.bonename;
    eye.material = src.material;
    eye.org = src.origin;
    eye.center = src.center;
    eye.radius = src.diameter / 2.0f;
    eye.zoffset = src.angle;
    eye.iris_scale = src.pupilscale;
    in.eyeballs.push_back(std::move(eye));
    return true;
}

// Option_Mouth: the controller is registered as a flexdesc.
bool RegisterMouth(cm::CompileInput& in, const FaceMarkup::Mouth& src, std::string* err) {
    if (src.controller.empty()) {
        if (err) *err = "mouth has no controller";
        return false;
    }
    if (src.bonename.empty()) {
        if (err) *err = "mouth \"" + src.controller + "\" has no bone";
        return false;
    }
    cm::Mouth mouth;
    mouth.controller = src.controller;
    mouth.bonename = src.bonename;
    mouth.forward = src.forward;
    mouth.flexdesc = AddFlexdesc(in, mouth.controller, err);
    if (mouth.flexdesc < 0)
        return false;
    in.mouths.push_back(std::move(mouth));
    return true;
}

// Lid poses are addressed by delta NAME, never a VTA frame index. Stereo binds
// both eyeballs to one <type>_right/<type>_left desc pair (Option_DmxEyelid);
// mono gives one eyeball its own lid desc and targets (Option_Eyelid).
bool RegisterEyelid(cm::CompileInput& in, const FaceMarkup::Eyelid& src, std::string* err) {
    const std::string type = src.upper ? "upper" : "lower";

    const bool mono = !src.eyeball.empty();

    std::string deltaName[3];
    float target[3];
    for (int i = 0; i < 3; ++i) {
        deltaName[i] = src.delta[i];
        target[i] = src.target[i];
    }

    // Each delta resolves on its own against whichever body carries it, and the
    // match yields that flexkey's imodel. An empty name is a pose with no vertex
    // data - it still gets its desc and target, just no flexkey.
    source::Source* lidSource[3] = {nullptr, nullptr, nullptr};
    int lidModel[3] = {-1, -1, -1};
    for (int i = 0; i < 3; ++i) {
        if (deltaName[i].empty())
            continue;
        int imodel = 0;
        for (const auto& part : in.bodyparts) {
            for (const auto& model : part.models) {
                if (model.source && !lidSource[i]) {
                    for (const auto& mo : model.source->morphs)
                        if (_stricmp(mo.name.c_str(), deltaName[i].c_str()) == 0) {
                            lidSource[i] = model.source;
                            lidModel[i] = imodel;
                            // keep the SOURCE's spelling of the name, like
                            // GetNewStyleSourceVertexAnim
                            deltaName[i] = mo.name;
                            break;
                        }
                }
                ++imodel;
            }
        }
        if (!lidSource[i]) {
            if (err) *err = "eyelid: no body carries a vertex animation named \"" +
                            deltaName[i] + "\"";
            return false;
        }
    }

    // flexdesc order is fixed and load-bearing. Mono is one base plus its three
    // suffixes; stereo is <type>_right and its three, then <type>_left and its
    // three.
    const std::string rightBase = mono ? src.basedesc : type + "_right";
    const std::string leftBase = mono ? src.basedesc : type + "_left";
    int baseDesc[2] = {-1, -1}; // [0] = left, [1] = right
    int lidDesc[3][2];          // [slot][left/right]
    baseDesc[1] = AddFlexdesc(in, rightBase, err);
    if (baseDesc[1] < 0) return false;
    for (int i = 0; i < 3; ++i) {
        lidDesc[i][1] = AddFlexdesc(in, rightBase + "_" + kLidSuffix[i], err);
        if (lidDesc[i][1] < 0) return false;
    }
    if (mono) {
        baseDesc[0] = baseDesc[1];
        for (int i = 0; i < 3; ++i)
            lidDesc[i][0] = lidDesc[i][1];
    } else {
        baseDesc[0] = AddFlexdesc(in, leftBase, err);
        if (baseDesc[0] < 0) return false;
        for (int i = 0; i < 3; ++i) {
            lidDesc[i][0] = AddFlexdesc(in, leftBase + "_" + kLidSuffix[i], err);
            if (lidDesc[i][0] < 0) return false;
        }
    }

    // one flexkey per slot that has a delta; stereo pairs left=desc/right=pair,
    // mono drives the single base desc
    for (int i = 0; i < 3; ++i) {
        if (!lidSource[i])
            continue;
        if (in.flexkeys.size() >= static_cast<size_t>(lim::kMaxFlexKeys)) {
            if (err) *err = "too many flex keys, max " + std::to_string(lim::kMaxFlexKeys);
            return false;
        }
        cm::FlexKey fk;
        fk.source = lidSource[i];
        fk.animationname = deltaName[i];
        fk.frame = 0; // always 0 for DMX
        fk.imodel = lidModel[i];
        fk.flexdesc = baseDesc[0];
        fk.flexpair = mono ? 0 : baseDesc[1];
        fk.split = src.split;
        fk.decay = 1.0f;
        switch (i) {
            case 0: // lowerer
                fk.target0 = -11.0f;
                fk.target1 = -10.0f;
                fk.target2 = target[0];
                fk.target3 = target[1];
                break;
            case 1: // neutral
                fk.target0 = target[0];
                fk.target1 = target[1];
                fk.target2 = target[1];
                fk.target3 = target[2];
                break;
            default: // raiser
                fk.target0 = target[1];
                fk.target1 = target[2];
                fk.target2 = 10.0f;
                fk.target3 = 11.0f;
                break;
        }
        in.flexkeys.push_back(std::move(fk));
    }

    // bind onto the already-registered eyeballs (hence list order)
    bool rightOk = false, leftOk = mono;
    for (cm::Eyeball& eye : in.eyeballs) {
        int side; // 0 = left, 1 = right
        if (mono) {
            if (_stricmp(src.eyeball.c_str(), eye.name.c_str()) != 0)
                continue;
            side = 1;
            rightOk = true;
        } else if (!src.righteyeball.empty() &&
            _stricmp(src.righteyeball.c_str(), eye.name.c_str()) == 0) {
            side = 1;
            rightOk = true;
        } else if (!src.lefteyeball.empty() &&
                   _stricmp(src.lefteyeball.c_str(), eye.name.c_str()) == 0) {
            side = 0;
            leftOk = true;
        } else {
            std::printf("WARNING: Unknown Eyeball: %s\n", eye.name.c_str());
            continue;
        }

        // radius is still unscaled here (SetupEyeballs scales it), so scale it
        // for the comparison the reference makes
        for (int i = 0; i < 3; ++i) {
            if (std::fabs(target[i]) > eye.radius * in.scale) {
                if (err) *err = "eyelid \"" + type + "\" " + kLidSuffix[i] +
                                " target out of range for eyeball \"" + eye.name + "\"";
                return false;
            }
        }

        int* lidflex = src.upper ? eye.upperflexdesc : eye.lowerflexdesc;
        float* lidtarget = src.upper ? eye.uppertarget : eye.lowertarget;
        (src.upper ? eye.upperlidflexdesc : eye.lowerlidflexdesc) = baseDesc[side];
        for (int i = 0; i < 3; ++i) {
            lidflex[i] = lidDesc[i][side];
            lidtarget[i] = target[i];
        }
    }
    if (!rightOk) {
        if (err) *err = "eyelid: could not find " + std::string(mono ? "" : "right ") +
                        "eyeball \"" + (mono ? src.eyeball : src.righteyeball) + "\"";
        return false;
    }
    if (!leftOk) {
        if (err) *err = "eyelid: could not find left eyeball \"" + src.lefteyeball + "\"";
        return false;
    }
    return true;
}

} // namespace

bool RegisterFaceMarkup(cm::CompileInput& in, const FaceMarkup& markup, std::string* err) {
    for (const FaceMarkup::Entry& e : markup.entries) {
        switch (e.kind) {
            case FaceMarkup::Kind::Eyeball:
                if (!RegisterEyeball(in, e.eyeball, err))
                    return false;
                break;
            case FaceMarkup::Kind::Mouth:
                if (!RegisterMouth(in, e.mouth, err))
                    return false;
                break;
            case FaceMarkup::Kind::Eyelid:
                if (!RegisterEyelid(in, e.eyelid, err))
                    return false;
                break;
        }
    }
    return true;
}

} // namespace pulse::loader
