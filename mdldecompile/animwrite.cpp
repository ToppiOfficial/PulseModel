// animwrite.cpp - rebuilds one animation clip per local animation, as a DMX or
// - on -smdanimation - as an SMD.
//
// The clip is compressed in the .mdl (or demand-loaded from the .ani) in one of
// two encodings, both undone here:
//
//   RLE     - a per-bone chain of mstudio_rle_anim_t. A channel is either a raw
//             constant or an mstudioanimvalue_t run-length stream of int16s,
//             which are offsets from the bone's bind pose scaled by its
//             posscale/rotscale.
//   FRAMEANIM - a per-bone flag byte array, a block of values that never change,
//             then a fixed stride per frame. Values are absolute, no scaling.
//
// Either container holds the same thing: a frame is the parent-relative
// position + rotation that comes out of either encoding, so nothing is lost on
// the way through. DMX is the default; SMD stays for a tool that only reads it.

#include "animwrite.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <system_error>
#include <vector>

#include "dmxwrite.h"
#include "math/compressed.h"

namespace mdldecompile {
namespace {

using Pose = AnimPose;

bool g_smd = false;

// A block of animation data plus the bounds it has to stay inside. Sections may
// live in the .mdl or in the .ani, so both files come through here.
struct Block {
    const char* base = nullptr;
    const char* end = nullptr;

    template <typename T>
    const T* At(const void* from, int32_t off, int count = 1) const {
        const char* p = static_cast<const char*>(from) + off;
        if (p < base || p > end || sizeof(T) * static_cast<size_t>(count) >
                                       static_cast<size_t>(end - p))
            return nullptr;
        return reinterpret_cast<const T*>(p);
    }
};

// studio.h ExtractAnimValue: the stream is (valid, total) headers each followed
// by `valid` int16s. A frame past `valid` but inside `total` holds the last
// value; a frame past `total` moves on to the next header.
float ExtractAnimValue(const Block& blk, const fm::mstudioanimvalue_t* p, int frame) {
    for (int guard = 0; guard < 1024; ++guard) {
        if (!blk.At<fm::mstudioanimvalue_t>(p, 0))
            return 0.0f;
        const int total = p->num.total, valid = p->num.valid;
        if (total == 0)
            return 0.0f;
        if (frame < total) {
            const int k = (valid > frame) ? frame + 1 : valid;
            const fm::mstudioanimvalue_t* v = blk.At<fm::mstudioanimvalue_t>(p, k * 2);
            return v ? static_cast<float>(v->value) : 0.0f;
        }
        frame -= total;
        p += valid + 1;
    }
    return 0.0f;
}

// One RLE section -> `out` for the frames it covers. `local` is the frame
// relative to the section's own start.
void DecodeRle(const Block& blk, const char* sect, const fm::mstudiobone_t* bones, int numbones,
               bool delta, int local, std::vector<Pose>& out) {
    const char* p = sect;
    for (int guard = 0; guard <= numbones; ++guard) {
        const auto* e = blk.At<fm::mstudio_rle_anim_t>(p, 0);
        if (!e)
            return;
        if (e->bone >= numbones) {
            if (e->nextoffset == 0)
                return;
            p += e->nextoffset;
            continue;
        }
        const fm::mstudiobone_t& b = bones[e->bone];
        Pose& o = out[e->bone];
        const char* d = p + sizeof(fm::mstudio_rle_anim_t);

        // rotation always comes first, raw or as a value-pointer block
        const fm::mstudioanim_valueptr_t* rotv = nullptr;
        if (e->flags & fm::STUDIO_ANIM_RAWROT2) {
            if (const auto* q = blk.At<pm::Quaternion64>(d, 0))
                pm::QuaternionAngles(q->Get(), o.rot);
            d += sizeof(pm::Quaternion64);
        } else if (e->flags & fm::STUDIO_ANIM_RAWROT) {
            if (const auto* q = blk.At<pm::Quaternion48>(d, 0))
                pm::QuaternionAngles(q->Get(), o.rot);
            d += sizeof(pm::Quaternion48);
        } else if (e->flags & fm::STUDIO_ANIM_ANIMROT) {
            rotv = blk.At<fm::mstudioanim_valueptr_t>(d, 0);
            d += sizeof(fm::mstudioanim_valueptr_t);
        }

        const fm::mstudioanim_valueptr_t* posv = nullptr;
        if (e->flags & fm::STUDIO_ANIM_RAWPOS) {
            if (const auto* v = blk.At<pm::Vector48>(d, 0))
                o.pos = v->Get();
            d += sizeof(pm::Vector48);
        } else if (e->flags & fm::STUDIO_ANIM_ANIMPOS) {
            posv = blk.At<fm::mstudioanim_valueptr_t>(d, 0);
        }

        // A stream value is an offset from the bone's STORED pos/rot in scale
        // units; a delta clip has no bind pose to offset from, so it seeds at
        // zero. The stored pair, not the poseToBone-derived one LocalPoses
        // hands the script and the DMX rig - an obfuscated file can hold a
        // decoy there, and the engine still decodes against it, so substituting
        // the real pose here contorts every clip.
        for (int k = 0; k < 3 && rotv; ++k) {
            if (!rotv->offset[k])
                continue;
            const auto* s = blk.At<fm::mstudioanimvalue_t>(rotv, rotv->offset[k]);
            if (!s)
                continue;
            (&o.rot.x)[k] = (delta ? 0.0f : (&b.rot.x)[k]) +
                            ExtractAnimValue(blk, s, local) * (&b.rotscale.x)[k];
        }
        for (int k = 0; k < 3 && posv; ++k) {
            if (!posv->offset[k])
                continue;
            const auto* s = blk.At<fm::mstudioanimvalue_t>(posv, posv->offset[k]);
            if (!s)
                continue;
            (&o.pos.x)[k] = (delta ? 0.0f : (&b.pos.x)[k]) +
                            ExtractAnimValue(blk, s, local) * (&b.posscale.x)[k];
        }

        if (e->nextoffset == 0)
            return;
        p += e->nextoffset;
    }
}

// One STUDIO_FRAMEANIM section. Values are absolute, so the bind pose is only
// the fallback for a bone the section says nothing about.
void DecodeFrameAnim(const Block& blk, const char* sect, int numbones, int local,
                     std::vector<Pose>& out) {
    const auto* fa = blk.At<fm::mstudio_frame_anim_t>(sect, 0);
    if (!fa)
        return;
    const uint8_t* flags = blk.At<uint8_t>(fa, sizeof(fm::mstudio_frame_anim_t), numbones);
    if (!flags)
        return;

    auto rot = [&](const char* d, Pose& o, uint8_t f) -> int {
        if (f & fm::STUDIO_FRAME_CONST_ROT2 || f & fm::STUDIO_FRAME_ANIM_ROT2) {
            if (const auto* q = blk.At<pm::Quaternion48S>(d, 0))
                pm::QuaternionAngles(q->Get(), o.rot);
            return sizeof(pm::Quaternion48S);
        }
        if (f & fm::STUDIO_FRAME_CONST_ROT || f & fm::STUDIO_FRAME_ANIM_ROT) {
            if (const auto* q = blk.At<pm::Quaternion48>(d, 0))
                pm::QuaternionAngles(q->Get(), o.rot);
            return sizeof(pm::Quaternion48);
        }
        return 0;
    };
    auto pos = [&](const char* d, Pose& o, uint8_t f) -> int {
        if (f & fm::STUDIO_FRAME_CONST_POS2 || f & fm::STUDIO_FRAME_ANIM_POS2) {
            if (const auto* v = blk.At<pm::Vector3>(d, 0))
                o.pos = *v;
            return sizeof(pm::Vector3);
        }
        if (f & fm::STUDIO_FRAME_CONST_POS || f & fm::STUDIO_FRAME_ANIM_POS) {
            if (const auto* v = blk.At<pm::Vector48>(d, 0))
                o.pos = v->Get();
            return sizeof(pm::Vector48);
        }
        return 0;
    };

    const char* c = reinterpret_cast<const char*>(fa) + fa->constantsoffset;
    for (int j = 0; j < numbones; ++j) {
        const uint8_t f = flags[j] & (fm::STUDIO_FRAME_CONST_ROT | fm::STUDIO_FRAME_CONST_ROT2 |
                                      fm::STUDIO_FRAME_CONST_POS | fm::STUDIO_FRAME_CONST_POS2);
        c += rot(c, out[j], f);
        c += pos(c, out[j], f);
    }

    const char* fr = reinterpret_cast<const char*>(fa) + fa->frameoffset +
                     static_cast<size_t>(fa->framelength) * local;
    for (int j = 0; j < numbones; ++j) {
        const uint8_t f = flags[j] & (fm::STUDIO_FRAME_ANIM_ROT | fm::STUDIO_FRAME_ANIM_ROT2 |
                                      fm::STUDIO_FRAME_ANIM_POS | fm::STUDIO_FRAME_ANIM_POS2);
        fr += rot(fr, out[j], f);
        fr += pos(fr, out[j], f);
    }
}

// All frames of one clip, in the space the .mdl stores them. Returns false when
// a section lives in a .ani that is missing or short.
bool DecodeAnim(const Mdl& m, const fm::mstudioanimdesc_t& a, const fm::mstudiobone_t* bones,
                int numbones, const Block& local, const Block& ext,
                const fm::mstudioanimblock_t* blocks, int numblocks,
                std::vector<std::vector<Pose>>& frames) {
    const int numframes = a.numframes > 0 ? a.numframes : 1;
    const bool delta = (a.flags & fm::STUDIO_DELTA) != 0;
    const bool frameanim = (a.flags & fm::STUDIO_FRAMEANIM) != 0;

    const fm::mstudioanimsections_t* sections =
        a.sectionframes > 0 ? m.At<fm::mstudioanimsections_t>(&a, a.sectionindex,
                                                              numframes / a.sectionframes + 2)
                            : nullptr;

    frames.assign(static_cast<size_t>(numframes),
                  std::vector<Pose>(static_cast<size_t>(numbones)));
    for (int f = 0; f < numframes; ++f) {
        std::vector<Pose>& out = frames[f];
        for (int j = 0; j < numbones; ++j)
            out[j] = delta ? Pose{} : Pose{bones[j].pos, bones[j].rot};
        // STUDIO_ALLZEROS is a clip the compiler found to be nothing but the
        // bind pose; it carries no data at all and rebuilds from the skeleton.
        if (a.flags & fm::STUDIO_ALLZEROS)
            continue;

        int block = a.animblock, index = a.animindex, sect = 0;
        if (sections) {
            sect = f / a.sectionframes;
            block = sections[sect].animblock;
            index = sections[sect].animindex;
        }

        const Block* blk = &local;
        const char* base = nullptr;
        if (block == 0) {
            base = reinterpret_cast<const char*>(&a) + index;
        } else if (ext.base && blocks && block < numblocks) {
            blk = &ext;
            base = ext.base + blocks[block].datastart + index;
        } else {
            return false;
        }
        if (!blk->At<char>(base, 0))
            return false;

        const int localFrame = sections ? f - sect * a.sectionframes : f;
        if (frameanim)
            DecodeFrameAnim(*blk, base, numbones, localFrame, out);
        else
            DecodeRle(*blk, base, bones, numbones, delta, localFrame, out);
    }
    return true;
}

// Undo motion extraction. `walkframe` strips the root's travel out of the frames
// and banks it as a piecewise path, so the SMD has to walk again or a recompile
// re-extracts nothing. Movement key k stores the CUMULATIVE pose at its end
// frame, so the segment's own step is prev^-1 * cur, replayed across the segment
// on the stored velocity ramp. Estimated, not exact: the `LQ` quadratic path
// replays as that same ramp, which matches at the ends and drifts in the middle.
void RestoreMotion(const Mdl& m, const fm::mstudioanimdesc_t& a, const fm::mstudiobone_t* bones,
                   int numbones, std::vector<std::vector<Pose>>& frames) {
    const fm::mstudiomovement_t* mv =
        m.At<fm::mstudiomovement_t>(&a, a.movementindex, a.nummovements);
    if (!mv || a.nummovements <= 0 || frames.empty())
        return;
    const int last = static_cast<int>(frames.size()) - 1;

    pm::matrix3x4 prev, ident;
    pm::AngleMatrix(pm::RadianEuler{0.0f, 0.0f, 0.0f}, pm::Vector3{0.0f, 0.0f, 0.0f}, ident);
    prev = ident;
    std::vector<pm::matrix3x4> adj(frames.size(), ident);

    int start = 0;
    for (int k = 0; k < a.nummovements; ++k) {
        pm::matrix3x4 cur;
        pm::AngleMatrix(pm::RadianEuler{0.0f, 0.0f, mv[k].angle * pm::kPiF / 180.0f},
                        mv[k].position, cur);
        pm::RadianEuler srot;
        pm::Vector3 spos;
        pm::MatrixAngles(pm::ConcatTransforms(pm::MatrixInvert(prev), cur), srot, spos);
        const float slen = std::sqrt(spos.x * spos.x + spos.y * spos.y + spos.z * spos.z);
        const pm::Vector3 dir =
            slen > 0.0f ? pm::Vector3{spos.x / slen, spos.y / slen, spos.z / slen}
                        : pm::Vector3{0.0f, 0.0f, 0.0f};

        const int end = std::min(mv[k].endframe, last);
        const float n = static_cast<float>(end - start);
        for (int f = start; n > 0.0f && f <= end; ++f) {
            const float t = (f - start) / n;
            const float d = mv[k].v0 * t + 0.5f * (mv[k].v1 - mv[k].v0) * t * t;
            pm::matrix3x4 step;
            pm::AngleMatrix(pm::RadianEuler{srot.x * t, srot.y * t, srot.z * t},
                            pm::Vector3{dir.x * d, dir.y * d, dir.z * d}, step);
            adj[static_cast<size_t>(f)] = pm::ConcatTransforms(prev, step);
        }
        prev = cur;
        start = std::min(mv[k].endframe, last);
    }
    // the tail past the last key keeps the final adjustment, as the extractor did
    for (int f = start + 1; f <= last; ++f)
        adj[static_cast<size_t>(f)] = prev;

    for (int f = 0; f <= last; ++f) {
        for (int j = 0; j < numbones; ++j) {
            if (bones[j].parent >= 0)
                continue;
            pm::matrix3x4 bm;
            pm::AngleMatrix(frames[f][j].rot, frames[f][j].pos, bm);
            pm::MatrixAngles(pm::ConcatTransforms(adj[static_cast<size_t>(f)], bm),
                             frames[f][j].rot, frames[f][j].pos);
        }
    }
}

} // namespace

void SetAnimFormat(bool smd) { g_smd = smd; }

const char* AnimExt() { return g_smd ? ".smd" : ".dmx"; }

void WriteAnimationFiles(const Mdl& m, const std::string& mdlPath, const std::string& dir) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioanimdesc_t* descs =
        m.At<fm::mstudioanimdesc_t>(m.buf.data(), h.localanimindex, h.numlocalanim);
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    if (!descs || !bones || h.numlocalanim <= 0 || h.numbones <= 0)
        return;

    // demand-loaded sections live in the sibling .ani, addressed through the
    // block table; without it those clips are skipped rather than half written
    std::vector<char> ani;
    const fm::mstudioanimblock_t* blocks =
        m.At<fm::mstudioanimblock_t>(m.buf.data(), h.animblockindex, h.numanimblocks);
    if (blocks && h.numanimblocks > 1)
        ReadWhole(StripExt(mdlPath) + ".ani", ani);

    const std::vector<std::string> boneNames = BoneNames(m);
    const std::vector<AnimRef> refs = AnimRefs(m);
    const std::string animDir = dir + "/anims";
    std::error_code ec;
    std::filesystem::create_directories(animDir, ec);

    const Block local{m.buf.data(), m.buf.data() + m.buf.size()};
    const Block ext{ani.empty() ? nullptr : ani.data(),
                    ani.empty() ? nullptr : ani.data() + ani.size()};

    // A delta clip is stored already subtracted, but the .pulseqc rebuilds it
    // with `subtract "<base>" 0` - so the SMD has to carry the pose BEFORE the
    // subtraction or the compiler takes it out twice. Decode the base once up
    // front to put it back; where the file has no clip that can serve, the bind
    // pose does, and gets written out as a clip of its own.
    const int baseIndex = SubtractBase(m, refs);
    std::vector<Pose> realBase, bindPose(static_cast<size_t>(h.numbones));
    for (int j = 0; j < h.numbones; ++j)
        bindPose[j] = Pose{bones[j].pos, bones[j].rot};
    if (baseIndex >= 0) {
        std::vector<std::vector<Pose>> f;
        if (DecodeAnim(m, descs[baseIndex], bones, h.numbones, local, ext, blocks,
                       h.numanimblocks, f) &&
            !f.empty())
            realBase = f[0];
    }

    // BuildRawTransforms yaws a source clip's ROOT bones by g_defaultrotation
    // (+90 about Z) on the way in, so the clip goes back out with that removed.
    // Child bones are parent-relative and never see it.
    pm::matrix3x4 unrotate;
    pm::AngleMatrix(pm::RadianEuler{0.0f, 0.0f, -pm::kPiF / 2.0f}, unrotate);
    auto unyawRoots = [&](std::vector<Pose>& fr) {
        for (int j = 0; j < h.numbones; ++j) {
            if (bones[j].parent >= 0)
                continue;
            pm::matrix3x4 mm;
            pm::AngleMatrix(fr[j].rot, fr[j].pos, mm);
            pm::MatrixAngles(pm::ConcatTransforms(unrotate, mm), fr[j].rot, fr[j].pos);
        }
    };

    auto emit = [&](const std::string& name, const std::vector<std::vector<Pose>>& frames,
                    int fps) {
        const std::string path = animDir + "/" + name + AnimExt();
        if (!g_smd) {
            if (!WriteAnimationDmx(m, path, name, fps, frames)) {
                std::printf("  cannot write \"%s\"\n", path.c_str());
                return false;
            }
            std::printf("  wrote %s (%d frame%s, %d bones)\n", path.c_str(),
                        static_cast<int>(frames.size()), frames.size() == 1 ? "" : "s", h.numbones);
            return true;
        }
        std::FILE* f = std::fopen(path.c_str(), "wb");
        if (!f) {
            std::printf("  cannot write \"%s\"\n", path.c_str());
            return false;
        }
        std::fprintf(f, "version 1\nnodes\n");
        for (int j = 0; j < h.numbones; ++j)
            std::fprintf(f, "%d \"%s\" %d\n", j, boneNames[j].c_str(), bones[j].parent);
        std::fprintf(f, "end\nskeleton\n");
        for (size_t fr = 0; fr < frames.size(); ++fr) {
            std::fprintf(f, "time %d\n", static_cast<int>(fr));
            for (int j = 0; j < h.numbones; ++j) {
                const Pose& p = frames[fr][j];
                std::fprintf(f, "%d %s %s %s %s %s %s\n", j, F(p.pos.x).c_str(), F(p.pos.y).c_str(),
                             F(p.pos.z).c_str(), F(p.rot.x).c_str(), F(p.rot.y).c_str(),
                             F(p.rot.z).c_str());
            }
        }
        std::fprintf(f, "end\n");
        std::fclose(f);
        std::printf("  wrote %s (%d frame%s, %d bones)\n", path.c_str(),
                    static_cast<int>(frames.size()), frames.size() == 1 ? "" : "s", h.numbones);
        return true;
    };

    int written = 0, skipped = 0;
    if (NeedsBindPoseAnim(m, baseIndex)) {
        std::vector<std::vector<Pose>> one{bindPose};
        unyawRoots(one[0]);
        if (emit(kBindPoseAnim, one, 30)) // the fps the .pulseqc writes for it
            ++written;
    }
    for (int i = 0; i < h.numlocalanim; ++i) {
        const fm::mstudioanimdesc_t& a = descs[i];
        if (a.flags & fm::STUDIO_OVERRIDE)
            continue; // $declareanimation - the data lives in the $includemodel
        const int numframes = a.numframes > 0 ? a.numframes : 1;

        std::vector<std::vector<Pose>> frames;
        if (!DecodeAnim(m, a, bones, h.numbones, local, ext, blocks, h.numanimblocks, frames)) {
            std::printf("  \"%s\": animation data is in a .ani that is missing or short "
                        "- skipped\n",
                        refs[i].name.c_str());
            ++skipped;
            continue;
        }

        // Undo SubtractBaseAnimations. The script writes `subtract`, which the
        // compiler runs with STUDIO_POST - so the delta is base^-1 * pose and
        // pose - base, whatever the clip's own POST flag says. (`presubtract`
        // is the other direction; nothing here emits it.) A bone the clip's
        // weightlist zeroed was left absolute and is quietly wrong here - rare
        // enough to live with.
        if (a.flags & fm::STUDIO_DELTA) {
            const std::vector<Pose>& subBase =
                (baseIndex >= 0 && baseIndex < i && !realBase.empty()) ? realBase : bindPose;
            for (std::vector<Pose>& fr : frames) {
                for (int j = 0; j < h.numbones; ++j) {
                    pm::Quaternion qbase, qd;
                    pm::AngleQuaternion(subBase[j].rot, qbase);
                    pm::AngleQuaternion(fr[j].rot, qd);
                    pm::QuaternionSMAngles(1.0f, qbase, qd, fr[j].rot);
                    fr[j].pos = {fr[j].pos.x + subBase[j].pos.x, fr[j].pos.y + subBase[j].pos.y,
                                 fr[j].pos.z + subBase[j].pos.z};
                }
            }
        }

        // motion was extracted in the yawed compile space, so it goes back on
        // before the yaw comes off
        RestoreMotion(m, a, bones, h.numbones, frames);

        for (std::vector<Pose>& fr : frames)
            unyawRoots(fr);

        // the clip's own fps, so the DMX key times land on the frames the
        // `fps` the script writes will sample
        const int fps = a.fps > 0.0f ? static_cast<int>(a.fps + 0.5f) : 30;
        if (emit(refs[i].name, frames, fps))
            ++written;
        else
            ++skipped;
    }

    if (written)
        std::printf("%d animation clip%s in %s\n", written, written == 1 ? "" : "s",
                    animDir.c_str());
    if (skipped)
        std::printf("  %d animation%s could not be extracted\n", skipped, skipped == 1 ? "" : "s");
}

} // namespace mdldecompile
