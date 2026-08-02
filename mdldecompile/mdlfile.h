// mdlfile.h - the loaded .mdl plus the small readers every output writer shares.

#ifndef MDLDECOMPILE_MDLFILE_H
#define MDLDECOMPILE_MDLFILE_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

#include "format/mdl.h"
#include "math/math.h"

namespace fm = pulse::format;
namespace pm = pulse::math;

namespace mdldecompile {

// A loaded file plus bounds-checked access. Every offset in a .mdl is
// attacker-controlled once the file is not ours, so nothing dereferences
// without a range check.
struct Mdl {
    std::vector<char> buf;
    const fm::studiohdr_t* hdr = nullptr;

    bool InRange(const void* base, size_t bytes) const {
        const char* p = static_cast<const char*>(base);
        return p >= buf.data() && p + bytes <= buf.data() + buf.size();
    }

    // `base` is what the offset is relative to: the file start for studiohdr_t
    // fields, the owning struct for everything else.
    template <typename T>
    const T* At(const void* base, int32_t off, int count = 1) const {
        if (off == 0 || count <= 0)
            return nullptr;
        const T* p = reinterpret_cast<const T*>(static_cast<const char*>(base) + off);
        return InRange(p, sizeof(T) * static_cast<size_t>(count)) ? p : nullptr;
    }

    const char* Str(const void* base, int32_t off) const {
        const char* p = At<char>(base, off);
        if (!p)
            return "";
        // must terminate inside the file
        for (const char* q = p; q < buf.data() + buf.size(); ++q)
            if (*q == '\0')
                return p;
        return "";
    }
};

inline bool ReadWhole(const std::string& path, std::vector<char>& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f)
        return false;
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (size <= 0) {
        std::fclose(f);
        return false;
    }
    out.resize(static_cast<size_t>(size));
    const size_t got = std::fread(out.data(), 1, out.size(), f);
    std::fclose(f);
    return got == out.size();
}

// Plain decimal, never an exponent - a QC reads better without 1.0488e-15.
// Trailing zeros are trimmed, and float noise below the last place lands on "0".
inline std::string F(float v) {
    char b[64];
    std::snprintf(b, sizeof b, "%.6f", v);
    std::string s = b;
    if (s.find('.') != std::string::npos) {
        s.erase(s.find_last_not_of('0') + 1);
        if (s.back() == '.')
            s.pop_back();
    }
    return (s == "-0") ? "0" : s;
}

inline std::string V3(const pm::Vector3& v) {
    return F(v.x) + " " + F(v.y) + " " + F(v.z);
}

// Material names are matched by basename (MaterialNameMatches strips the
// stored path but not the query's), so a script has to name them bare.
inline std::string BaseName(const std::string& s) {
    const size_t slash = s.find_last_of("/\\");
    return slash == std::string::npos ? s : s.substr(slash + 1);
}

inline std::string StripExt(const std::string& s) {
    const size_t dot = s.find_last_of('.');
    const size_t slash = s.find_last_of("/\\");
    if (dot == std::string::npos || (slash != std::string::npos && dot < slash))
        return s;
    return s.substr(0, dot);
}

// A stored name is only usable in a script when it is printable, quote-free and
// single-line - obfuscators point names into a blob of decoy text.
inline bool CleanName(const std::string& n) {
    return !n.empty() && n.find('"') == std::string::npos &&
           std::all_of(n.begin(), n.end(), [](unsigned char c) { return c >= 0x20 && c < 0x7f; });
}

inline std::vector<std::string> TextureNames(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<std::string> names;
    const fm::mstudiotexture_t* tex =
        m.At<fm::mstudiotexture_t>(m.buf.data(), h.textureindex, h.numtextures);
    for (int i = 0; tex && i < h.numtextures; ++i)
        names.push_back(m.Str(&tex[i], tex[i].sznameindex));
    return names;
}

// Bone names are not trusted as read: obfuscators point them into a junk blob
// of decoy text, usually the same one for every bone, which would recompile
// into merged or unparseable bones. Anything that is not a clean, unique,
// single-line name becomes bone_<index>. Every writer takes its bone names from
// here, so a renamed bone stays consistent across attachments/hitboxes/ik.
inline std::vector<std::string> BoneNames(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<std::string> names;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    std::set<std::string> used;
    for (int i = 0; bones && i < h.numbones; ++i) {
        std::string n = m.Str(&bones[i], bones[i].sznameindex);
        if (!CleanName(n) || !used.insert(n).second) {
            n = "bone_" + std::to_string(i);
            used.insert(n);
        }
        names.push_back(n);
    }
    return names;
}

// The bind pose to trust, per bone. Obfuscators write a constant decoy into
// pos/rot while leaving poseToBone - what the renderer skins with - correct, so
// the stored pair is only kept when it agrees with the matrix. Every writer
// takes the skeleton from here, or the script and the DMX rig end up describing
// different skeletons.
struct LocalPose {
    pm::Vector3 pos{};
    pm::RadianEuler rot{};
    bool rebuilt = false;
};

inline std::vector<LocalPose> LocalPoses(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones =
        m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    std::vector<LocalPose> out;
    for (int i = 0; bones && i < h.numbones; ++i) {
        // poseToBone is model->bone, so its inverse is the bind pose and the
        // parent's poseToBone is already the inverse needed on the other side
        const int32_t p = bones[i].parent;
        const pm::matrix3x4 world = pm::MatrixInvert(bones[i].poseToBone);
        const pm::matrix3x4 local =
            (p >= 0 && p < h.numbones) ? pm::ConcatTransforms(bones[p].poseToBone, world) : world;

        // the round trip loses a little, so only a real disagreement counts
        pm::matrix3x4 stored;
        pm::AngleMatrix(bones[i].rot, bones[i].pos, stored);
        bool agrees = true;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                agrees &= std::fabs(stored.m[r][c] - local.m[r][c]) < 0.01f;

        LocalPose lp;
        if (agrees) {
            lp.pos = bones[i].pos;
            lp.rot = bones[i].rot;
        } else {
            pm::MatrixAngles(local, lp.rot, lp.pos);
            lp.rebuilt = true;
        }
        out.push_back(lp);
    }
    return out;
}

// A local animation's script name, plus whether the script ever named it. An
// animation declared inline in a $sequence is stored as "@<sequence>", and that
// marker is the only record of which form was written; several implied blends in
// one grid share the name, so duplicates take an ordinal. The SMD writer names
// its files from here, so both halves of the decompile agree.
struct AnimRef {
    std::string name;
    bool implied = false;
};

inline std::vector<AnimRef> AnimRefs(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioanimdesc_t* a =
        m.At<fm::mstudioanimdesc_t>(m.buf.data(), h.localanimindex, h.numlocalanim);
    std::vector<AnimRef> refs;
    std::set<std::string> used;
    for (int i = 0; a && i < h.numlocalanim; ++i) {
        std::string n = m.Str(&a[i], a[i].sznameindex);
        const bool implied = !n.empty() && n[0] == '@';
        if (implied)
            n.erase(0, 1);
        if (!CleanName(n))
            n = "anim_" + std::to_string(i);
        std::string unique = n;
        for (int k = 2; !used.insert(unique).second; ++k)
            unique = n + "_" + std::to_string(k);
        refs.push_back({unique, implied});
    }
    return refs;
}

// What a delta was subtracted from is not in the file - only the delta flag
// survives. A script subtracts the bind pose, so the first single-frame,
// non-delta, script-nameable clip stands in for it. The .pulseqc names it in
// `subtract` and the SMD writer undoes the subtraction with it, so both halves
// have to make the same pick.
inline int SubtractBase(const Mdl& m, const std::vector<AnimRef>& refs) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioanimdesc_t* a =
        m.At<fm::mstudioanimdesc_t>(m.buf.data(), h.localanimindex, h.numlocalanim);
    for (int i = 0; a && i < h.numlocalanim; ++i)
        if (a[i].numframes == 1 && !refs[i].implied &&
            !(a[i].flags & (fm::STUDIO_DELTA | fm::STUDIO_OVERRIDE)))
            return i;
    return -1;
}

// The clip a delta subtracts. A real single-frame reference earlier in the table
// is used when there is one; otherwise the decompile writes the bind pose out
// under this name, which is what the source almost certainly subtracted. Without
// a base there is no way to spell a delta at all - the clip recompiles as an
// absolute pose and loses STUDIO_DELTA.
inline constexpr char kBindPoseAnim[] = "a_bindpose";

inline std::string SubtractNameFor(const std::vector<AnimRef>& refs, int base, int i) {
    return (base >= 0 && base < i) ? refs[base].name : std::string(kBindPoseAnim);
}

// True when some delta clip has to fall back on that synthesized bind-pose clip.
inline bool NeedsBindPoseAnim(const Mdl& m, int base) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudioanimdesc_t* a =
        m.At<fm::mstudioanimdesc_t>(m.buf.data(), h.localanimindex, h.numlocalanim);
    for (int i = 0; a && i < h.numlocalanim; ++i)
        if (!(a[i].flags & fm::STUDIO_OVERRIDE) && (a[i].flags & fm::STUDIO_DELTA) &&
            !(base >= 0 && base < i))
            return true;
    return false;
}

// A stereo flex was split at compile time into "<name>L"/"<name>R" descs sharing
// one vertanim set, so the delta the source authored is the desc without its L.
inline std::string DeltaName(const std::string& desc, bool stereo) {
    if (stereo && desc.size() > 1 && desc.back() == 'L')
        return desc.substr(0, desc.size() - 1);
    return desc;
}

// Runs fn(flexdesc, flexpair) over every morph in the file. flexpair is 0 when
// the flex is mono.
template <typename F>
void ForEachFlex(const Mdl& m, F fn) {
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobodyparts_t* parts =
        m.At<fm::mstudiobodyparts_t>(m.buf.data(), h.bodypartindex, h.numbodyparts);
    for (int i = 0; parts && i < h.numbodyparts; ++i) {
        const fm::mstudiomodel_t* models =
            m.At<fm::mstudiomodel_t>(&parts[i], parts[i].modelindex, parts[i].nummodels);
        for (int j = 0; models && j < parts[i].nummodels; ++j) {
            const fm::mstudiomesh_t* meshes =
                m.At<fm::mstudiomesh_t>(&models[j], models[j].meshindex, models[j].nummeshes);
            for (int k = 0; meshes && k < models[j].nummeshes; ++k) {
                const fm::mstudioflex_t* fx =
                    m.At<fm::mstudioflex_t>(&meshes[k], meshes[k].flexindex, meshes[k].numflexes);
                for (int n = 0; fx && n < meshes[k].numflexes; ++n)
                    fn(fx[n].flexdesc, fx[n].flexpair);
            }
        }
    }
}

// Flex desc names get the same treatment as bone names, and for a sharper
// reason: an obfuscator points every desc at one empty string, which collapses
// every morph target onto a single DMX delta state and leaves every $flexrule
// naming "". Junk or duplicated becomes flex<index> - no underscore, a DMX
// importer reads that as the separator between the halves of a corrective.
inline std::vector<std::string> FlexDescNames(const Mdl& m) {
    const fm::studiohdr_t& h = *m.hdr;
    std::vector<std::string> names;
    const fm::mstudioflexdesc_t* d =
        m.At<fm::mstudioflexdesc_t>(m.buf.data(), h.flexdescindex, h.numflexdesc);
    std::set<std::string> used;
    std::vector<bool> renamed;
    for (int i = 0; d && i < h.numflexdesc; ++i) {
        std::string n = m.Str(&d[i], d[i].szFACSindex);
        const bool junk = !CleanName(n) || !used.insert(n).second;
        if (junk) {
            n = "flex" + std::to_string(i);
            used.insert(n);
        }
        renamed.push_back(junk);
        names.push_back(n);
    }

    // A stereo pair is rebuilt from ONE delta as <name>L/<name>R, so a renamed
    // pair has to keep that convention or nothing - no $flexrule, no $eyelid -
    // can name either half again.
    ForEachFlex(m, [&](int32_t desc, int32_t pair) {
        if (pair <= 0 || desc < 0 || desc >= h.numflexdesc || pair >= h.numflexdesc)
            return;
        if (!renamed[desc] || !renamed[pair])
            return;
        names[desc] = "flex" + std::to_string(desc) + "L";
        names[pair] = "flex" + std::to_string(desc) + "R";
    });
    return names;
}

// Which descs a recompile will actually recreate: a mono flex writes its delta
// under the desc's own name, a stereo one under <delta>L/<delta>R. A desc whose
// name does not come back out of that is unreachable, and the $flexrule naming
// it has to be written commented out.
inline std::vector<bool> DescIsReproducible(const Mdl& m, const std::vector<std::string>& descs) {
    std::vector<bool> ok(descs.size(), true);
    ForEachFlex(m, [&](int32_t desc, int32_t pair) {
        const size_t d = static_cast<size_t>(desc), p = static_cast<size_t>(pair);
        if (desc < 0 || d >= descs.size())
            return;
        if (pair <= 0) {
            ok[d] = true; // mono: the delta carries the desc's own name
            return;
        }
        const std::string base = DeltaName(descs[d], true);
        ok[d] = descs[d] == base + "L";
        if (p < descs.size())
            ok[p] = descs[p] == base + "R";
    });
    return ok;
}

} // namespace mdldecompile

#endif // MDLDECOMPILE_MDLFILE_H
