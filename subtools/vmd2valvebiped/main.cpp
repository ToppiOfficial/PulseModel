// vmd2valvebiped - retarget an MMD motion (.vmd) onto a ValveBiped .mdl as a DMX clip.
//
// Retargeting is done in world space: MMD bones rest with identity orientation
// and a different rest pose, so local rotations never transfer directly. Legs
// follow the MMD foot IK through the target model's own $ikchain data.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "dmxwrite.h"
#include "dr_mp3.h"
#include "dr_wav.h"
#include "mdlfile.h"
#include "pulselimits.h"
#include "strcompat.h"

namespace fm = pulse::format;
namespace fs = std::filesystem;
namespace pm = pulse::math;
using mdldecompiler::AnimPose;
using mdldecompiler::Mdl;
using pm::matrix3x4;
using pm::Vector3;

namespace {

// ------------------------------------------------------------------ vector ops

Vector3 Add(const Vector3& a, const Vector3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 Sub(const Vector3& a, const Vector3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vector3 Mul(const Vector3& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
float Dot(const Vector3& a, const Vector3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
float Len(const Vector3& a) { return std::sqrt(Dot(a, a)); }
Vector3 Cross(const Vector3& a, const Vector3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vector3 Origin(const matrix3x4& m) { return {m.m[0][3], m.m[1][3], m.m[2][3]}; }
void SetOrigin(matrix3x4& m, const Vector3& v) {
    m.m[0][3] = v.x;
    m.m[1][3] = v.y;
    m.m[2][3] = v.z;
}
matrix3x4 RotOnly(matrix3x4 m) {
    SetOrigin(m, {});
    return m;
}
matrix3x4 Transpose(const matrix3x4& m) {
    matrix3x4 t;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            t.m[r][c] = m.m[c][r];
    return t;
}

// Rodrigues rotation taking unit `from` onto unit `to` (same as the compiler's).
matrix3x4 RotationBetween(const Vector3& from, const Vector3& to) {
    Vector3 axis = Cross(from, to);
    float s = pm::VectorNormalize(axis);
    float c = Dot(from, to);
    if (s < 1e-6f) {
        if (c > 0.0f)
            return matrix3x4{};
        axis = std::fabs(from.x) < 0.9f ? Cross(from, Vector3{1, 0, 0}) : Cross(from, Vector3{0, 1, 0});
        pm::VectorNormalize(axis);
        s = 0.0f;
        c = -1.0f;
    }
    const float t = 1.0f - c;
    matrix3x4 r;
    r.m[0][0] = t * axis.x * axis.x + c;
    r.m[0][1] = t * axis.x * axis.y - s * axis.z;
    r.m[0][2] = t * axis.x * axis.z + s * axis.y;
    r.m[1][0] = t * axis.x * axis.y + s * axis.z;
    r.m[1][1] = t * axis.y * axis.y + c;
    r.m[1][2] = t * axis.y * axis.z - s * axis.x;
    r.m[2][0] = t * axis.x * axis.z - s * axis.y;
    r.m[2][1] = t * axis.y * axis.z + s * axis.x;
    r.m[2][2] = t * axis.z * axis.z + c;
    return r;
}

void SwingAbout(std::vector<matrix3x4>& w, const Vector3& pivot, Vector3 from, Vector3 to,
                std::initializer_list<int> bones) {
    if (pm::VectorNormalize(from) < 1e-6f || pm::VectorNormalize(to) < 1e-6f)
        return;
    const matrix3x4 r = RotationBetween(from, to);
    for (int b : bones) {
        const Vector3 rel = pm::VectorRotate(Sub(Origin(w[b]), pivot), r);
        w[b] = pm::ConcatTransforms(r, w[b]);
        SetOrigin(w[b], Add(pivot, rel));
    }
}

// Two-bone analytic IK (the compiler's SolveTwoBoneIK) with a bend-plane hint,
// so a leg that starts straight still bends the knee toward `hint`.
void SolveTwoBoneIK(int iThigh, int iKnee, int iFoot, const Vector3& target, Vector3 hint,
                    std::vector<matrix3x4>& w) {
    const Vector3 thigh = Origin(w[iThigh]), knee = Origin(w[iKnee]), foot = Origin(w[iFoot]);
    const float l1 = Len(Sub(knee, thigh)), l2 = Len(Sub(foot, knee));
    Vector3 dir = Sub(target, thigh);
    float dist = pm::VectorNormalize(dir);
    if (l1 < 1e-4f || l2 < 1e-4f || dist < 1e-4f)
        return;
    dist = std::max(dist, std::max(std::fabs(l1 - l2) * 1.01f, 0.01f));
    dist = std::min(dist, (l1 + l2) * 0.999f);

    Vector3 bend = Len(hint) > 1e-4f ? hint : Sub(knee, thigh);
    bend = Sub(bend, Mul(dir, Dot(bend, dir)));
    if (pm::VectorNormalize(bend) < 1e-4f) {
        bend = std::fabs(dir.x) < 0.9f ? Cross(dir, Vector3{1, 0, 0}) : Cross(dir, Vector3{0, 1, 0});
        pm::VectorNormalize(bend);
    }
    const float a = (dist * dist + l1 * l1 - l2 * l2) / (2.0f * dist);
    const float h = std::sqrt(std::max(0.0f, l1 * l1 - a * a));
    const Vector3 newKnee = Add(thigh, Add(Mul(dir, a), Mul(bend, h)));
    const Vector3 newFoot = Add(thigh, Mul(dir, dist));

    SwingAbout(w, thigh, Sub(knee, thigh), Sub(newKnee, thigh), {iThigh, iKnee, iFoot});
    SwingAbout(w, newKnee, Sub(Origin(w[iFoot]), newKnee), Sub(newFoot, newKnee), {iKnee, iFoot});
}

// --------------------------------------------------------------- byte reading

struct Reader {
    const std::vector<char>& b;
    size_t at = 0;
    bool ok = true;

    bool Need(size_t n) {
        if (!ok || n > b.size() - at)
            ok = false;
        return ok;
    }
    void Skip(size_t n) {
        if (Need(n))
            at += n;
    }
    template <typename T>
    T Get() {
        T v{};
        if (Need(sizeof(T))) {
            std::memcpy(&v, b.data() + at, sizeof(T));
            at += sizeof(T);
        }
        return v;
    }
    Vector3 V3() {
        Vector3 v;
        v.x = Get<float>();
        v.y = Get<float>();
        v.z = Get<float>();
        return v;
    }
    // Fixed-width field, cut at the first NUL.
    std::string Fixed(size_t n) {
        if (!Need(n))
            return std::string();
        const char* p = b.data() + at;
        at += n;
        return std::string(p, strnlen(p, n));
    }
    int32_t Index(int size) {
        if (size == 1)
            return Get<int8_t>();
        if (size == 2)
            return Get<int16_t>();
        return Get<int32_t>();
    }
};

// ------------------------------------------------------------------------- vmd

struct Key {
    uint32_t frame = 0;
    Vector3 pos;
    pm::Quaternion rot;
    uint8_t interp[64] = {};
};

// Camera key: `at` is the look-at point, `rot` the orbit euler, `dist` the
// (negative) eye offset. interp holds [x1 x2 y1 y2] per channel: at xyz, rot, dist, fov.
struct CamKey {
    uint32_t frame = 0;
    float dist = 0.0f;
    Vector3 at, rot;
    uint8_t interp[24] = {};
    uint32_t fov = 30;
};

float Curve(float x1, float y1, float x2, float y2, float x) {
    if (x1 == y1 && x2 == y2)
        return x;
    const auto at = [](float p1, float p2, float t) {
        const float u = 1.0f - t;
        return 3.0f * u * u * t * p1 + 3.0f * u * t * t * p2 + t * t * t;
    };
    float lo = 0.0f, hi = 1.0f, t = x;
    for (int i = 0; i < 24; ++i) {
        t = 0.5f * (lo + hi);
        (at(x1, x2, t) < x ? lo : hi) = t;
    }
    return at(y1, y2, t);
}

// VMD bone bezier: channel c (0-2 position xyz, 3 rotation), control points in
// 0..127 at interp[c], [4+c], [8+c], [12+c]. The curve lives on the later key.
float Bezier(const uint8_t* ip, int c, float x) {
    return Curve(ip[c] / 127.0f, ip[4 + c] / 127.0f, ip[8 + c] / 127.0f, ip[12 + c] / 127.0f, x);
}

matrix3x4 AxisRot(int axis, float a) {
    const int i = (axis + 1) % 3, j = (axis + 2) % 3;
    matrix3x4 r;
    r.m[i][i] = r.m[j][j] = std::cos(a);
    r.m[i][j] = -std::sin(a);
    r.m[j][i] = std::sin(a);
    return r;
}

// Camera eye, view axes and vertical fov (degrees) at MMD frame f, in MMD space. Keys
// one frame apart are a cut and hold the earlier key. Orbit is yaw, then roll, then pitch.
void SampleCamera(const std::vector<CamKey>& keys, float f, Vector3& eye, Vector3& fwd, Vector3& up,
                  float& fov) {
    const CamKey* a = &keys.front();
    const CamKey* b = a;
    float x = 0.0f;
    if (f >= keys.back().frame) {
        a = b = &keys.back();
    } else if (f > keys.front().frame) {
        const auto it = std::upper_bound(keys.begin(), keys.end(), f,
                                         [](float v, const CamKey& k) { return v < k.frame; });
        b = &*it;
        a = &*(it - 1);
        x = (f - a->frame) / static_cast<float>(b->frame - a->frame);
        if (b->frame - a->frame <= 1)
            b = a;
    }
    const auto w = [&](int c) {
        const uint8_t* ip = b->interp + 4 * c;
        return Curve(ip[0] / 127.0f, ip[2] / 127.0f, ip[1] / 127.0f, ip[3] / 127.0f, x);
    };
    const auto lerp = [](float p, float q, float t) { return p + (q - p) * t; };
    const Vector3 at{lerp(a->at.x, b->at.x, w(0)), lerp(a->at.y, b->at.y, w(1)), lerp(a->at.z, b->at.z, w(2))};
    const float wr = w(3);
    const Vector3 rot{lerp(a->rot.x, b->rot.x, wr), lerp(a->rot.y, b->rot.y, wr), lerp(a->rot.z, b->rot.z, wr)};
    const float dist = std::fabs(lerp(a->dist, b->dist, w(4)));
    fov = lerp(static_cast<float>(a->fov), static_cast<float>(b->fov), w(5));

    // MMD is left-handed, so every angle turns the opposite way to the RH formulas
    const matrix3x4 r = pm::ConcatTransforms(AxisRot(1, -rot.y),
                                             pm::ConcatTransforms(AxisRot(2, -rot.z), AxisRot(0, -rot.x)));
    eye = Add(at, pm::VectorRotate(Vector3{0, 0, -dist}, r));
    fwd = pm::VectorRotate(Vector3{0, 0, 1}, r);
    up = pm::VectorRotate(Vector3{0, 1, 0}, r);
}

// Morph key: VMD morphs interpolate linearly, there is no curve.
struct MorphKey {
    uint32_t frame = 0;
    float weight = 0.0f;
};
using Morphs = std::map<std::string, std::vector<MorphKey>>;

float SampleMorph(const std::vector<MorphKey>& keys, float f) {
    if (f <= keys.front().frame)
        return keys.front().weight;
    if (f >= keys.back().frame)
        return keys.back().weight;
    const auto it = std::upper_bound(keys.begin(), keys.end(), f,
                                     [](float v, const MorphKey& k) { return v < k.frame; });
    const MorphKey& b = *it;
    const MorphKey& a = *(it - 1);
    return a.weight + (b.weight - a.weight) * (f - a.frame) / static_cast<float>(b.frame - a.frame);
}

// Sorted by frame; a repeated frame keeps the last key written.
template <typename T>
void SortKeys(std::vector<T>& keys) {
    std::stable_sort(keys.begin(), keys.end(), [](const T& a, const T& b) { return a.frame < b.frame; });
    std::vector<T> uniq;
    for (const T& k : keys) {
        if (!uniq.empty() && uniq.back().frame == k.frame)
            uniq.back() = k;
        else
            uniq.push_back(k);
    }
    keys.swap(uniq);
}

void Sample(const std::vector<Key>& keys, float f, Vector3& pos, pm::Quaternion& rot) {
    if (keys.empty())
        return;
    if (f <= keys.front().frame) {
        pos = keys.front().pos;
        rot = keys.front().rot;
        return;
    }
    if (f >= keys.back().frame) {
        pos = keys.back().pos;
        rot = keys.back().rot;
        return;
    }
    const auto it = std::upper_bound(keys.begin(), keys.end(), f,
                                     [](float v, const Key& k) { return v < k.frame; });
    const Key& b = *it;
    const Key& a = *(it - 1);
    const float x = (f - a.frame) / static_cast<float>(b.frame - a.frame);
    pos.x = a.pos.x + (b.pos.x - a.pos.x) * Bezier(b.interp, 0, x);
    pos.y = a.pos.y + (b.pos.y - a.pos.y) * Bezier(b.interp, 1, x);
    pos.z = a.pos.z + (b.pos.z - a.pos.z) * Bezier(b.interp, 2, x);
    pm::QuaternionSlerp(a.rot, b.rot, Bezier(b.interp, 3, x), rot);
}

// Bone and morph keys by raw Shift-JIS name, and the camera keys if the file has any.
bool LoadVmd(const std::string& path, std::map<std::string, std::vector<Key>>& out,
             Morphs& morphs, std::vector<CamKey>& cams, uint32_t& lastFrame, std::string& err) {
    std::vector<char> buf;
    if (!mdldecompiler::ReadWhole(path, buf)) {
        err = "cannot read \"" + path + "\"";
        return false;
    }
    Reader r{buf};
    const std::string magic = r.Fixed(30);
    size_t nameLen = 0;
    if (magic == "Vocaloid Motion Data 0002")
        nameLen = 20;
    else if (magic == "Vocaloid Motion Data file")
        nameLen = 10;
    else {
        err = "\"" + path + "\" is not a .vmd (bad header)";
        return false;
    }
    r.Skip(nameLen);
    const uint32_t count = r.Get<uint32_t>();
    lastFrame = 0;
    for (uint32_t i = 0; i < count && r.ok; ++i) {
        const std::string name = r.Fixed(15);
        Key k;
        k.frame = r.Get<uint32_t>();
        k.pos = r.V3();
        k.rot.x = r.Get<float>();
        k.rot.y = r.Get<float>();
        k.rot.z = r.Get<float>();
        k.rot.w = r.Get<float>();
        if (r.Need(64))
            std::memcpy(k.interp, buf.data() + r.at, 64);
        r.Skip(64);
        if (!r.ok)
            break;
        out[name].push_back(k);
        lastFrame = std::max(lastFrame, k.frame);
    }
    if (!r.ok) {
        err = "\"" + path + "\" is truncated in the bone key section";
        return false;
    }
    for (auto& [name, keys] : out)
        SortKeys(keys);

    // morph then camera sections; older files end before either
    const uint32_t nmorph = r.at < buf.size() ? r.Get<uint32_t>() : 0;
    for (uint32_t i = 0; i < nmorph && r.ok; ++i) {
        const std::string name = r.Fixed(15);
        MorphKey k;
        k.frame = r.Get<uint32_t>();
        k.weight = r.Get<float>();
        if (r.ok)
            morphs[name].push_back(k);
    }
    const uint32_t ncam = r.ok && r.at < buf.size() ? r.Get<uint32_t>() : 0;
    for (uint32_t i = 0; i < ncam && r.ok; ++i) {
        CamKey c;
        c.frame = r.Get<uint32_t>();
        c.dist = r.Get<float>();
        c.at = r.V3();
        c.rot = r.V3();
        if (r.Need(24))
            std::memcpy(c.interp, buf.data() + r.at, 24);
        r.Skip(24);
        c.fov = r.Get<uint32_t>();
        r.Skip(1); // perspective flag
        if (r.ok)
            cams.push_back(c);
    }
    if (!r.ok) {
        err = "\"" + path + "\" is truncated in the morph or camera section";
        return false;
    }
    SortKeys(cams);
    for (auto& [name, keys] : morphs)
        SortKeys(keys);
    return true;
}

// ------------------------------------------------------------ mmd rest skeleton

// Standard MMD bones, parents first. Names are Shift-JIS (VMD) and UTF-8 (PMX)
// escapes, without the side prefix; `pos` is the left side's built-in rest.
struct MmdDef {
    const char* id;
    const char* sj;
    const char* u8;
    bool sided;
    const char* parent;
    Vector3 pos;
};
const MmdDef kMmd[] = {
    {"all", "\x91\x53\x82\xc4\x82\xcc\x90\x65", "\xe5\x85\xa8\xe3\x81\xa6\xe3\x81\xae\xe8\xa6\xaa", false, nullptr, {0, 0, 0}},
    {"center", "\x83\x5a\x83\x93\x83\x5e\x81\x5b", "\xe3\x82\xbb\xe3\x83\xb3\xe3\x82\xbf\xe3\x83\xbc", false, "all", {0, 8.0f, 0}},
    {"groove", "\x83\x4f\x83\x8b\x81\x5b\x83\x75", "\xe3\x82\xb0\xe3\x83\xab\xe3\x83\xbc\xe3\x83\x96", false, "center", {0, 8.2f, 0}},
    {"waist", "\x8d\x98", "\xe8\x85\xb0", false, "groove", {0, 11.2f, 0.2f}},
    {"upper", "\x8f\xe3\x94\xbc\x90\x67", "\xe4\xb8\x8a\xe5\x8d\x8a\xe8\xba\xab", false, "waist", {0, 11.8f, 0}},
    {"upper2", "\x8f\xe3\x94\xbc\x90\x67\x32", "\xe4\xb8\x8a\xe5\x8d\x8a\xe8\xba\xab\x32", false, "upper", {0, 13.2f, 0}},
    {"neck", "\x8e\xf1", "\xe9\xa6\x96", false, "upper2", {0, 15.9f, 0.3f}},
    {"head", "\x93\xaa", "\xe9\xa0\xad", false, "neck", {0, 16.8f, 0.2f}},
    {"lower", "\x89\xba\x94\xbc\x90\x67", "\xe4\xb8\x8b\xe5\x8d\x8a\xe8\xba\xab", false, "waist", {0, 11.8f, 0}},
    {"shoulder", "\x8c\xa8", "\xe8\x82\xa9", true, "upper2", {0.3f, 15.4f, 0.3f}},
    // arm chain and fingers are placed along -armangle in BuiltinArms
    {"arm", "\x98\x72", "\xe8\x85\x95", true, "shoulder", {1.3f, 15.1f, 0.4f}},
    {"armtwist", "\x98\x72\x9d\x80", "\xe8\x85\x95\xe6\x8d\xa9", true, "arm", {}},
    {"elbow", "\x82\xd0\x82\xb6", "\xe3\x81\xb2\xe3\x81\x98", true, "armtwist", {}},
    {"handtwist", "\x8e\xe8\x9d\x80", "\xe6\x89\x8b\xe6\x8d\xa9", true, "elbow", {}},
    {"wrist", "\x8e\xe8\x8e\xf1", "\xe6\x89\x8b\xe9\xa6\x96", true, "handtwist", {}},
    {"thumb0", "\x90\x65\x8e\x77\x82\x4f", "\xe8\xa6\xaa\xe6\x8c\x87\xef\xbc\x90", true, "wrist", {}},
    {"thumb1", "\x90\x65\x8e\x77\x82\x50", "\xe8\xa6\xaa\xe6\x8c\x87\xef\xbc\x91", true, "thumb0", {}},
    {"thumb2", "\x90\x65\x8e\x77\x82\x51", "\xe8\xa6\xaa\xe6\x8c\x87\xef\xbc\x92", true, "thumb1", {}},
    {"index1", "\x90\x6c\x8e\x77\x82\x50", "\xe4\xba\xba\xe6\x8c\x87\xef\xbc\x91", true, "wrist", {}},
    {"index2", "\x90\x6c\x8e\x77\x82\x51", "\xe4\xba\xba\xe6\x8c\x87\xef\xbc\x92", true, "index1", {}},
    {"index3", "\x90\x6c\x8e\x77\x82\x52", "\xe4\xba\xba\xe6\x8c\x87\xef\xbc\x93", true, "index2", {}},
    {"middle1", "\x92\x86\x8e\x77\x82\x50", "\xe4\xb8\xad\xe6\x8c\x87\xef\xbc\x91", true, "wrist", {}},
    {"middle2", "\x92\x86\x8e\x77\x82\x51", "\xe4\xb8\xad\xe6\x8c\x87\xef\xbc\x92", true, "middle1", {}},
    {"middle3", "\x92\x86\x8e\x77\x82\x52", "\xe4\xb8\xad\xe6\x8c\x87\xef\xbc\x93", true, "middle2", {}},
    {"ring1", "\x96\xf2\x8e\x77\x82\x50", "\xe8\x96\xac\xe6\x8c\x87\xef\xbc\x91", true, "wrist", {}},
    {"ring2", "\x96\xf2\x8e\x77\x82\x51", "\xe8\x96\xac\xe6\x8c\x87\xef\xbc\x92", true, "ring1", {}},
    {"ring3", "\x96\xf2\x8e\x77\x82\x52", "\xe8\x96\xac\xe6\x8c\x87\xef\xbc\x93", true, "ring2", {}},
    {"pinky1", "\x8f\xac\x8e\x77\x82\x50", "\xe5\xb0\x8f\xe6\x8c\x87\xef\xbc\x91", true, "wrist", {}},
    {"pinky2", "\x8f\xac\x8e\x77\x82\x51", "\xe5\xb0\x8f\xe6\x8c\x87\xef\xbc\x92", true, "pinky1", {}},
    {"pinky3", "\x8f\xac\x8e\x77\x82\x52", "\xe5\xb0\x8f\xe6\x8c\x87\xef\xbc\x93", true, "pinky2", {}},
    {"leg", "\x91\xab", "\xe8\xb6\xb3", true, "lower", {0.9f, 10.9f, 0.1f}},
    {"knee", "\x82\xd0\x82\xb4", "\xe3\x81\xb2\xe3\x81\x96", true, "leg", {0.9f, 6.0f, -0.1f}},
    {"ankle", "\x91\xab\x8e\xf1", "\xe8\xb6\xb3\xe9\xa6\x96", true, "knee", {0.9f, 1.1f, 0.4f}},
    {"ikparent", "\x91\xab\x49\x4b\x90\x65", "\xe8\xb6\xb3\x49\x4b\xe8\xa6\xaa", true, "all", {0.9f, 0, 0.4f}},
    {"legik", "\x91\xab\x82\x68\x82\x6a", "\xe8\xb6\xb3\xef\xbc\xa9\xef\xbc\xab", true, "ikparent", {0.9f, 1.1f, 0.4f}},
};
const char* const kSideSj[2] = {"\x8d\xb6", "\x89\x45"}; // left, right
const char* const kSideU8[2] = {"\xe5\xb7\xa6", "\xe5\x8f\xb3"};

struct MmdBone {
    std::string key; // "<id>" or "L:<id>" / "R:<id>"
    std::string sj, u8;
    int parent = -1;
    Vector3 rest;
};

struct MmdSkel {
    std::vector<MmdBone> bones;
    std::map<std::string, int> byKey;

    int Find(const std::string& id, int side = -1) const {
        const auto it = byKey.find(side < 0 ? id : std::string(side ? "R:" : "L:") + id);
        return it == byKey.end() ? -1 : it->second;
    }
};

MmdSkel BuildMmdSkel() {
    MmdSkel s;
    for (const MmdDef& d : kMmd) {
        for (int side = 0; side < (d.sided ? 2 : 1); ++side) {
            MmdBone b;
            b.key = d.sided ? std::string(side ? "R:" : "L:") + d.id : d.id;
            b.sj = d.sided ? std::string(kSideSj[side]) + d.sj : d.sj;
            b.u8 = d.sided ? std::string(kSideU8[side]) + d.u8 : d.u8;
            b.rest = {side ? -d.pos.x : d.pos.x, d.pos.y, d.pos.z};
            if (d.parent) {
                const bool sidedParent = s.byKey.count(d.parent) == 0;
                b.parent = s.Find(d.parent, sidedParent ? side : -1);
            }
            s.byKey[b.key] = static_cast<int>(s.bones.size());
            s.bones.push_back(b);
        }
    }
    return s;
}

// A-pose arms: every arm bone on one line from the shoulder joint, drooping
// `deg` below horizontal. Fingers sit one hand length past the wrist.
void BuiltinArms(MmdSkel& s, float deg) {
    const float a = deg * pm::kDeg2Rad;
    for (int side = 0; side < 2; ++side) {
        const Vector3 d{(side ? -1.0f : 1.0f) * std::cos(a), -std::sin(a), 0.0f};
        const Vector3 arm = s.bones[s.Find("arm", side)].rest;
        const auto put = [&](const char* id, float along) {
            s.bones[s.Find(id, side)].rest = Add(arm, Mul(d, along));
        };
        put("armtwist", 1.5f);
        put("elbow", 3.0f);
        put("handtwist", 4.4f);
        put("wrist", 5.8f);
        for (const char* f : {"thumb0", "thumb1", "thumb2", "index1", "index2", "index3", "middle1",
                              "middle2", "middle3", "ring1", "ring2", "ring3", "pinky1", "pinky2",
                              "pinky3"})
            put(f, 6.6f);
    }
}

std::string Utf16ToUtf8(const char* p, size_t bytes) {
    std::string out;
    for (size_t i = 0; i + 1 < bytes; i += 2) {
        const unsigned c = static_cast<unsigned char>(p[i]) | (static_cast<unsigned char>(p[i + 1]) << 8);
        if (c < 0x80) {
            out += static_cast<char>(c);
        } else if (c < 0x800) {
            out += static_cast<char>(0xc0 | (c >> 6));
            out += static_cast<char>(0x80 | (c & 0x3f));
        } else {
            out += static_cast<char>(0xe0 | (c >> 12));
            out += static_cast<char>(0x80 | ((c >> 6) & 0x3f));
            out += static_cast<char>(0x80 | (c & 0x3f));
        }
    }
    return out;
}

// Rest positions from a PMX (2.0/2.1) bone section. A standard bone the model
// lacks sits on its parent, which keeps it inert for rotation.
bool LoadPmxRest(const std::string& path, MmdSkel& s, std::string& err) {
    std::vector<char> buf;
    if (!mdldecompiler::ReadWhole(path, buf)) {
        err = "cannot read \"" + path + "\"";
        return false;
    }
    Reader r{buf};
    if (r.Fixed(4) != "PMX ") {
        err = "\"" + path + "\" is not a .pmx";
        return false;
    }
    r.Skip(4); // version
    const uint8_t nglobals = r.Get<uint8_t>();
    uint8_t g[8] = {};
    for (int i = 0; i < nglobals; ++i) {
        const uint8_t v = r.Get<uint8_t>();
        if (i < 8)
            g[i] = v;
    }
    const bool utf16 = g[0] == 0;
    const int addUv = g[1], vtxSize = g[2], texSize = g[3], boneSize = g[5];
    const auto text = [&]() {
        const int32_t n = r.Get<int32_t>();
        if (n < 0 || !r.Need(static_cast<size_t>(n)))
            return std::string();
        const char* p = buf.data() + r.at;
        r.at += static_cast<size_t>(n);
        return utf16 ? Utf16ToUtf8(p, static_cast<size_t>(n)) : std::string(p, static_cast<size_t>(n));
    };
    for (int i = 0; i < 4; ++i)
        text();

    const int32_t nverts = r.Get<int32_t>();
    for (int32_t i = 0; i < nverts && r.ok; ++i) {
        r.Skip(32 + 16 * static_cast<size_t>(addUv));
        switch (r.Get<uint8_t>()) {
        case 0: r.Skip(boneSize); break;
        case 1: r.Skip(2 * boneSize + 4); break;
        case 2:
        case 4: r.Skip(4 * boneSize + 16); break;
        case 3: r.Skip(2 * boneSize + 4 + 36); break;
        default: r.ok = false;
        }
        r.Skip(4);
    }
    r.Skip(static_cast<size_t>(std::max(0, r.Get<int32_t>())) * vtxSize);
    const int32_t ntex = r.Get<int32_t>();
    for (int32_t i = 0; i < ntex && r.ok; ++i)
        text();
    const int32_t nmats = r.Get<int32_t>();
    for (int32_t i = 0; i < nmats && r.ok; ++i) {
        text();
        text();
        r.Skip(16 + 12 + 4 + 12 + 1 + 16 + 4 + 2 * texSize + 1);
        r.Skip(r.Get<uint8_t>() == 0 ? texSize : 1);
        text();
        r.Skip(4);
    }

    std::map<std::string, Vector3> found;
    const int32_t nbones = r.Get<int32_t>();
    for (int32_t i = 0; i < nbones && r.ok; ++i) {
        const std::string name = text();
        text();
        const Vector3 pos = r.V3();
        r.Skip(boneSize + 4);
        const uint16_t flags = r.Get<uint16_t>();
        r.Skip((flags & 0x1) ? boneSize : 12);
        if (flags & 0x300) r.Skip(boneSize + 4);
        if (flags & 0x400) r.Skip(12);
        if (flags & 0x800) r.Skip(24);
        if (flags & 0x2000) r.Skip(4);
        if (flags & 0x20) {
            r.Skip(boneSize + 8);
            const int32_t links = r.Get<int32_t>();
            for (int32_t k = 0; k < links && r.ok; ++k) {
                r.Skip(boneSize);
                if (r.Get<uint8_t>())
                    r.Skip(24);
            }
        }
        if (r.ok)
            found.emplace(name, pos);
    }
    if (!r.ok) {
        err = "\"" + path + "\" is truncated or not a PMX 2.x file";
        return false;
    }

    std::string missing;
    for (MmdBone& b : s.bones) {
        const auto it = found.find(b.u8);
        if (it != found.end()) {
            b.rest = it->second;
        } else {
            b.rest = b.parent >= 0 ? s.bones[b.parent].rest : Vector3{};
            missing += " " + b.key;
        }
    }
    if (!missing.empty())
        std::printf("pmx: standard bones not in the model, held on their parent:%s\n", missing.c_str());
    return true;
}

// ---------------------------------------------------------------------- target

struct Target {
    int n = 0;
    std::vector<int> parent;
    std::vector<matrix3x4> local, world; // bind pose
    std::vector<std::string> names;

    int Find(const std::string& suffix) const {
        const std::string want = "ValveBiped.Bip01_" + suffix;
        for (int i = 0; i < n; ++i)
            if (_stricmp(names[i].c_str(), want.c_str()) == 0)
                return i;
        return -1;
    }
};

bool LoadMdl(const std::string& path, Mdl& m, std::string& err) {
    if (!mdldecompiler::ReadWhole(path, m.buf) || m.buf.size() < sizeof(fm::studiohdr_t)) {
        err = "cannot read \"" + path + "\"";
        return false;
    }
    m.hdr = reinterpret_cast<const fm::studiohdr_t*>(m.buf.data());
    if (m.hdr->id != fm::kIdStudioHeader) {
        err = "\"" + path + "\" is not a studio model (bad id)";
        return false;
    }
    return true;
}

// Target bone <- MMD bone. Legs are not here: the IK solve places them.
struct MapDef {
    const char* valve;
    const char* mmd;
    bool sided;
};
const MapDef kMap[] = {
    {"Pelvis", "lower", false},   {"Spine", "upper", false},     {"Spine1", "upper", false},
    {"Spine2", "upper2", false},  {"Spine4", "upper2", false},   {"Neck1", "neck", false},
    {"Head1", "head", false},     {"Clavicle", "shoulder", true}, {"UpperArm", "arm", true},
    {"Forearm", "elbow", true},   {"Hand", "wrist", true},       {"Finger0", "thumb0", true},
    {"Finger01", "thumb1", true}, {"Finger02", "thumb2", true},  {"Finger1", "index1", true},
    {"Finger11", "index2", true}, {"Finger12", "index3", true},  {"Finger2", "middle1", true},
    {"Finger21", "middle2", true}, {"Finger22", "middle3", true}, {"Finger3", "ring1", true},
    {"Finger31", "ring2", true},  {"Finger32", "ring3", true},   {"Finger4", "pinky1", true},
    {"Finger41", "pinky2", true}, {"Finger42", "pinky3", true},
};

// MMD morph -> flex controllers, as "<controller>=<weight>" tokens, on Valve names.
// `ops` suits FACS rules (HL2/GMod citizens, L4D2 survivors); its vowels are Valve's
// phonemes.txt presets a=aa i=iy u=uw e=eh o=ao. `visemes` (null = ops) drives one
// controller per vowel, for rigs that normalise each mouth shape by the sum of all.
struct FaceDef {
    const char* id;
    const char* sj;
    const char* u8;
    const char* ops;
    const char* visemes = nullptr;
};
const FaceDef kFace[] = {
    {"a", "\x82\xa0", "\xe3\x81\x82", "jaw_drop=0.8 right_mouth_drop=1 left_mouth_drop=1", "jaw_drop=1"},
    {"i", "\x82\xa2", "\xe3\x81\x84",
     "right_part=1 left_part=1 right_funneler=0.75 left_funneler=0.75 right_stretcher=0.78 "
     "left_stretcher=0.78 jaw_clencher=0.55 jaw_drop=0.3 right_mouth_drop=1 left_mouth_drop=1",
     "right_stretcher=1 left_stretcher=1"},
    {"u", "\x82\xa4", "\xe3\x81\x86", "right_puckerer=0.95 left_puckerer=0.9 jaw_drop=0.4",
     "right_funneler=1.2 left_funneler=1.2"},
    {"e", "\x82\xa6", "\xe3\x81\x88",
     "right_funneler=0.5 left_funneler=0.5 right_stretcher=0.6 left_stretcher=0.6 jaw_drop=0.4 "
     "right_mouth_drop=1 left_mouth_drop=1", "right_part=1 left_part=1"},
    {"o", "\x82\xa8", "\xe3\x81\x8a",
     "right_puckerer=0.5 left_puckerer=0.5 right_funneler=0.5 left_funneler=0.5 jaw_drop=0.75 "
     "right_mouth_drop=0.6 left_mouth_drop=0.6", "right_funneler=2 left_funneler=2"},
    {"blink", "\x82\xdc\x82\xce\x82\xbd\x82\xab", "\xe3\x81\xbe\xe3\x81\xb0\xe3\x81\x9f\xe3\x81\x8d", "blink=1"},
    {"wink", "\x83\x45\x83\x42\x83\x93\x83\x4e", "\xe3\x82\xa6\xe3\x82\xa3\xe3\x83\xb3\xe3\x82\xaf", "left_lid_closer=1"},
    {"winkright", "\x83\x45\x83\x42\x83\x93\x83\x4e\x89\x45",
     "\xe3\x82\xa6\xe3\x82\xa3\xe3\x83\xb3\xe3\x82\xaf\xe5\x8f\xb3", "right_lid_closer=1"},
    {"wink2", "\x83\x45\x83\x42\x83\x93\x83\x4e\x82\x51",
     "\xe3\x82\xa6\xe3\x82\xa3\xe3\x83\xb3\xe3\x82\xaf\xef\xbc\x92", "left_lid_closer=1"},
    {"wink2right", "\xb3\xa8\xdd\xb8\x82\x51\x89\x45",
     "\xef\xbd\xb3\xef\xbd\xa8\xef\xbe\x9d\xef\xbd\xb8\xef\xbc\x92\xe5\x8f\xb3", "right_lid_closer=1"},
    {"laugh", "\x8f\xce\x82\xa2", "\xe7\xac\x91\xe3\x81\x84",
     "right_lid_closer=0.7 left_lid_closer=0.7 right_cheek_raiser=0.5 left_cheek_raiser=0.5"},
    {"jitome", "\x82\xb6\x82\xc6\x96\xda", "\xe3\x81\x98\xe3\x81\xa8\xe7\x9b\xae",
     "right_lid_closer=0.4 left_lid_closer=0.4"},
    {"surprised", "\x82\xd1\x82\xc1\x82\xad\x82\xe8", "\xe3\x81\xb3\xe3\x81\xa3\xe3\x81\x8f\xe3\x82\x8a",
     "right_lid_raiser=1 left_lid_raiser=1"},
    {"smile", "\x82\xc9\x82\xb1\x82\xe8", "\xe3\x81\xab\xe3\x81\x93\xe3\x82\x8a",
     "right_corner_puller=1 left_corner_puller=1"},
    {"frown", "\x81\xc8", "\xe2\x88\xa7", "right_corner_depressor=0.8 left_corner_depressor=0.8"},
    {"troubled", "\x8d\xa2\x82\xe9", "\xe5\x9b\xb0\xe3\x82\x8b", "right_inner_raiser=1 left_inner_raiser=1"},
    {"angry", "\x93\x7b\x82\xe8", "\xe6\x80\x92\xe3\x82\x8a", "right_lowerer=1 left_lowerer=1"},
    {"serious", "\x90\x5e\x96\xca\x96\xda", "\xe7\x9c\x9f\xe9\x9d\xa2\xe7\x9b\xae", "right_lowerer=0.5 left_lowerer=0.5"},
    {"browup", "\x8f\xe3", "\xe4\xb8\x8a",
     "right_inner_raiser=1 left_inner_raiser=1 right_outer_raiser=1 left_outer_raiser=1"},
    {"browdown", "\x89\xba", "\xe4\xb8\x8b", "right_lowerer=0.6 left_lowerer=0.6"},
};

using FaceOps = std::vector<std::pair<std::string, float>>;
using FaceMap = std::map<std::string, FaceOps>; // by raw VMD morph name

bool ParseFaceOps(const std::string& text, FaceOps& ops) {
    std::istringstream in(text);
    for (std::string tok; in >> tok;) {
        const size_t eq = tok.find('=');
        if (eq == 0 || eq == std::string::npos)
            return false;
        char* end = nullptr;
        const float w = std::strtof(tok.c_str() + eq + 1, &end);
        if (end == tok.c_str() + eq + 1 || *end)
            return false;
        ops.emplace_back(tok.substr(0, eq), w);
    }
    return true;
}

FaceMap BuiltinFaceMap(bool visemes) {
    FaceMap map;
    for (const FaceDef& d : kFace)
        ParseFaceOps(visemes && d.visemes ? d.visemes : d.ops, map[d.sj]);
    return map;
}

// One "<morph> <controller>=<weight>..." per line, # comments. <morph> is a kFace
// id, its UTF-8 name, or the raw VMD bytes (a Shift-JIS file names any morph).
bool LoadFaceMap(const std::string& path, FaceMap& map, std::string& err) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        err = "cannot read \"" + path + "\"";
        return false;
    }
    map.clear();
    int line = 0;
    for (std::string s; std::getline(f, s);) {
        if (++line == 1 && s.rfind("\xef\xbb\xbf", 0) == 0)
            s.erase(0, 3);
        s.resize(std::min(s.find('#'), s.size()));
        std::istringstream in(s);
        std::string morph, rest;
        if (!(in >> morph))
            continue;
        for (const FaceDef& d : kFace)
            if (morph == d.id || morph == d.u8) {
                morph = d.sj;
                break;
            }
        std::getline(in, rest);
        if (!ParseFaceOps(rest, map[morph])) {
            err = "\"" + path + "\" line " + std::to_string(line) + ": expected <controller>=<weight>";
            return false;
        }
    }
    return true;
}

// Per-frame controller values on the .mdl's own controllers, clamped to their
// range. Only controllers some keyed morph drives come back.
std::vector<mdldecompiler::FlexTrack> FaceTracks(const Morphs& morphs, const FaceMap& map,
                                                 const std::vector<mdldecompiler::FlexTrack>& ctrls,
                                                 const std::vector<float>& times,
                                                 std::set<std::string>& missing, int& unmapped) {
    std::vector<mdldecompiler::FlexTrack> tracks = ctrls;
    std::vector<bool> used(ctrls.size(), false);
    for (mdldecompiler::FlexTrack& t : tracks)
        t.values.assign(times.size(), 0.0f);
    for (const auto& [name, keys] : morphs) {
        const auto it = map.find(name);
        if (it == map.end()) {
            ++unmapped;
            continue;
        }
        for (const auto& [ctrl, w] : it->second) {
            size_t c = 0;
            while (c < ctrls.size() && _stricmp(ctrls[c].name.c_str(), ctrl.c_str()) != 0)
                ++c;
            if (c == ctrls.size()) {
                missing.insert(ctrl);
                continue;
            }
            used[c] = true;
            for (size_t k = 0; k < times.size(); ++k)
                tracks[c].values[k] += SampleMorph(keys, times[k]) * w;
        }
    }
    std::vector<mdldecompiler::FlexTrack> out;
    for (size_t c = 0; c < tracks.size(); ++c) {
        if (!used[c])
            continue;
        for (float& v : tracks[c].values)
            v = std::clamp(v, tracks[c].min, tracks[c].max);
        out.push_back(std::move(tracks[c]));
    }
    return out;
}

// MMD eye bones: both eyes, then left and right.
const char* const kEyeBones[3] = {"\x97\xbc\x96\xda", "\x8d\xb6\x96\xda", "\x89\x45\x96\xda"};

// Gaze from the eye bones as -1..1 of `range` degrees on eyes_updown/eyes_rightleft, in
// the engine's signs: +1 looks down / toward the model's left. The player scales it to range.
void AddEyeTracks(const std::map<std::string, std::vector<Key>>& keys,
                  const std::vector<mdldecompiler::FlexTrack>& ctrls, const std::vector<float>& times,
                  float range, std::vector<mdldecompiler::FlexTrack>& out) {
    const std::vector<Key>* eye[3] = {};
    for (int i = 0; i < 3; ++i) {
        const auto it = keys.find(kEyeBones[i]);
        eye[i] = it == keys.end() ? nullptr : &it->second;
    }
    const auto ctrlName = [&](const char* n) {
        for (const mdldecompiler::FlexTrack& c : ctrls)
            if (_stricmp(c.name.c_str(), n) == 0)
                return c.name;
        return std::string();
    };
    mdldecompiler::FlexTrack up{ctrlName("eyes_updown"), -1.0f, 1.0f, {}};
    mdldecompiler::FlexTrack side{ctrlName("eyes_rightleft"), -1.0f, 1.0f, {}};
    if ((!eye[0] && !eye[1] && !eye[2]) || (up.name.empty() && side.name.empty()))
        return;

    // the per-side bones add their average on top of the both-eyes bone
    const float sideWeight = eye[1] && eye[2] ? 0.5f : 1.0f;
    for (float f : times) {
        float yaw = 0.0f, pitch = 0.0f;
        for (int i = 0; i < 3; ++i) {
            if (!eye[i])
                continue;
            Vector3 pos;
            pm::Quaternion q;
            Sample(*eye[i], f, pos, q);
            const Vector3 d = pm::VectorRotate(Vector3{0, 0, -1}, pm::QuaternionMatrix(q, Vector3{}));
            const float w = i ? sideWeight : 1.0f;
            yaw += w * std::atan2(d.x, -d.z);
            pitch += w * std::atan2(d.y, std::sqrt(d.x * d.x + d.z * d.z));
        }
        up.values.push_back(std::clamp(-pitch * pm::kRad2Deg / range, -1.0f, 1.0f));
        side.values.push_back(std::clamp(yaw * pm::kRad2Deg / range, -1.0f, 1.0f));
    }
    for (mdldecompiler::FlexTrack* t : {&up, &side})
        if (!t->name.empty())
            out.push_back(std::move(*t));
}

struct Leg {
    int thigh = -1, knee = -1, foot = -1;
    Vector3 kneeDir; // thigh-local, from the $ikchain
    int ik = -1;     // MMD legik bone
    float bendSign = 1.0f; // which side of the hinge the $ikchain knee faces
};

void PrintHeader() {
    std::printf("-------------------------------\n");
    std::printf("PulseModel [VMD to ValveBiped]\n");
    std::printf("version:   %s\n", PULSEMODEL_VERSION);
    std::printf("developer: Toppi\n");
    std::printf("-------------------------------\n");
}

int Usage() {
    std::printf(
        "MMD motion (.vmd) -> ValveBiped DMX animation\n\n"
        "  vmd2valvebiped -vmd <file> [-vmd <file>...] -mdl <file> [options]\n\n"
        "  -vmd <file>      MMD motion to retarget; repeat for several clips, all\n"
        "                   retargeted through the same -mdl and -pmx\n"
        "  -mdl <file>      ValveBiped model giving the proportions; needs leg $ikchains\n"
        "  -pmx <file>      MMD model the motion was made for; its rest pose replaces\n"
        "                   the built-in standard skeleton\n"
        "  -armangle <deg>  built-in rest arm droop below horizontal (default 35)\n"
        "  -o <file>        output .dmx for a single -vmd (default: the .vmd's name,\n"
        "                   beside it)\n"
        "  -compile <name>  also build an animation-only .mdl/.ani with mdlcompiler:\n"
        "                   <name> is its $modelname (e.g. survivors/anim_mmd.mdl);\n"
        "                   clips and .qc go in <name>_src beside the first .vmd\n"
        "  -srcdir <dir>    where -compile writes its clips and .qc (default\n"
        "                   <name>_src beside the first .vmd)\n"
        "  -game <dir>      mod dir the compiled model installs into, under\n"
        "                   <dir>/models (default: beside the first .vmd)\n"
        "  -sound           with -compile: play <vmd name>.wav (or .mp3) from beside\n"
        "                   each .vmd when its sequence starts; converted to a 16-bit\n"
        "                   44.1 kHz wav in <dir>/sound/mmd/\n"
        "  -start <frame>   first MMD frame (default 0)\n"
        "  -end <frame>     last MMD frame (default the motion's last key)\n"
        "  -uncompressanim  keep every frame past the %d-frame limit, for engine\n"
        "                   branches that take more; default resamples to fit\n"
        "  -dmxencoding <enc>\n"
        "                   binary (default) or keyvalues2 text\n"
        "  -dmxmodel <n>    the `format model` version: 15 (default), 1, 18, or 22\n"
        "                   for Source 2 modeldoc\n"
        "  -faceset <name>  built-in face map on Valve controller names: facs (default;\n"
        "                   survivors, HL2 citizens - vowels as Valve phonemes) or\n"
        "                   visemes (one controller per vowel, for rigs whose mouth\n"
        "                   shapes each normalise by the sum of all mouth controls)\n"
        "  -eyerange <deg>  MMD eye-bone angle written as full gaze deflection\n"
        "                   (default 30); eyes go out as -1..1 of the model's range\n"
        "  -facemap <file>  MMD morph -> flex controller map replacing the built-in\n"
        "                   Valve face one: \"<morph> <controller>=<weight>...\" per\n"
        "                   line, <morph> a built-in id (a i u e o blink...), its\n"
        "                   Japanese name in UTF-8, or raw Shift-JIS bytes\n\n"
        "  Face keys from the .vmd, with <vmd name>_face.vmd beside it (a separate\n"
        "  lip sync) replacing any morph both key, go out as flex channels on the\n"
        "  .mdl's controllers, in model 22\n"
        "  only: the clip itself with -dmxmodel 22, and with -compile a model 22 copy\n"
        "  of every clip in <game>/datamodel/flex_anims/.\n\n"
        "  Camera keys from <vmd name>_camera.vmd beside each .vmd, else from the .vmd\n"
        "  itself, go out as a <clip>_camera .smd moving only the root bone\n"
        "  \"mmd_camera\": origin at the eye, +X along the view, +Z up.\n"
        "  -camerafov <mode> also carries MMD's vertical fov (degrees):\n"
        "                   bone - child bone \"mmd_camera_fov\", its local X is the fov\n"
        "                   dmx  - <clip>_camera.dmx, `format pulsecamera 1`, for\n"
        "                          PulseWorkshop; with -compile in\n"
        "                          <game>/datamodel/camera_anims/\n",
        pulse::limits::kMaxAnimFrames);
    return 1;
}

int Fail(const std::string& err) {
    std::fprintf(stderr, "error: %s\n", err.c_str());
    return 1;
}

// Sequence/file name from a path: each run of spaces, punctuation or non-ASCII
// becomes one '_'.
std::string ClipName(const std::string& path) {
    std::string s;
    for (char c : mdldecompiler::StripExt(mdldecompiler::BaseName(path))) {
        const bool keep = std::isalnum(static_cast<unsigned char>(c)) || c == '-';
        if (keep)
            s += c;
        else if (!s.empty() && s.back() != '_')
            s += '_';
    }
    while (!s.empty() && s.back() == '_')
        s.pop_back();
    return s.empty() ? std::string("clip") : s;
}

// The compiler ships one folder up from the subtools.
std::string CompilerPath(const char* argv0) {
    std::error_code ec;
    const fs::path self = fs::absolute(fs::path(argv0), ec);
#ifdef _WIN32
    const char* exe = "mdlcompiler.exe";
#else
    const char* exe = "mdlcompiler";
#endif
    const fs::path p = self.parent_path().parent_path() / exe;
    return fs::exists(p, ec) ? p.string() : std::string(exe);
}

struct Clip {
    std::string name;
    int fps = 30;
    std::string sound; // path under sound/, empty for none
    bool camera = false; // an .smd holding only kCameraBone
};

// Root bone the camera clip animates: origin at the eye, +X along the view, +Z up.
const char* const kCameraBone = "mmd_camera";
// -camerafov bone: child of kCameraBone, local X is the vertical fov in degrees.
const char* const kCameraFovBone = "mmd_camera_fov";

bool WriteCameraSmd(const std::string& path, const std::vector<AnimPose>& frames,
                    const std::vector<float>* fov) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    std::fprintf(f, "version 1\nnodes\n0 \"%s\" -1\n", kCameraBone);
    if (fov)
        std::fprintf(f, "1 \"%s\" 0\n", kCameraFovBone);
    std::fprintf(f, "end\nskeleton\n");
    for (size_t k = 0; k < frames.size(); ++k) {
        std::fprintf(f, "time %zu\n0 %f %f %f %f %f %f\n", k, frames[k].pos.x, frames[k].pos.y,
                     frames[k].pos.z, frames[k].rot.x, frames[k].rot.y, frames[k].rot.z);
        if (fov)
            std::fprintf(f, "1 %f 0 0 0 0 0\n", (*fov)[k]);
    }
    std::fprintf(f, "end\n");
    return std::fclose(f) == 0;
}

// Decodes a .wav or .mp3 and writes it as the PCM wav the engine plays: 16-bit,
// mono or stereo, 44100 Hz unless it already sits at 11025/22050. Returns a
// reason on failure; `note` says what was converted.
std::string ConvertAudio(const std::string& src, const std::string& dst, std::string& note) {
    std::vector<char> buf;
    if (!mdldecompiler::ReadWhole(src, buf))
        return "cannot be read";
    unsigned ch = 0, rate = 0;
    drwav_uint64 frames = 0;
    int16_t* pcm = nullptr;
    const bool mp3 = _stricmp(fs::path(src).extension().string().c_str(), ".mp3") == 0;
    if (mp3) {
        drmp3_config cfg{};
        drmp3_uint64 n = 0;
        pcm = drmp3_open_memory_and_read_pcm_frames_s16(buf.data(), buf.size(), &cfg, &n, nullptr);
        frames = n;
        ch = cfg.channels;
        rate = cfg.sampleRate;
    } else {
        pcm = drwav_open_memory_and_read_pcm_frames_s16(buf.data(), buf.size(), &ch, &rate, &frames, nullptr);
    }
    if (!pcm || !ch || !rate || !frames) {
        mp3 ? drmp3_free(pcm, nullptr) : drwav_free(pcm, nullptr);
        return "could not be decoded";
    }

    // more than two channels folds down to the first two
    const unsigned outCh = std::min(ch, 2u);
    const unsigned outRate = (rate == 11025 || rate == 22050 || rate == 44100) ? rate : 44100;
    const double step = static_cast<double>(rate) / outRate;
    const size_t outFrames = static_cast<size_t>(static_cast<double>(frames) / step);
    std::vector<int16_t> out(outFrames * outCh);
    if (outRate == rate) {
        for (size_t i = 0; i < outFrames; ++i)
            for (unsigned c = 0; c < outCh; ++c)
                out[i * outCh + c] = pcm[i * ch + c];
    } else {
        // Hann-windowed sinc, cut off at the lower Nyquist so a downsample
        // does not alias. 32 taps a side is plenty for music.
        const int taps = 32;
        const double cutoff = std::min(1.0, 1.0 / step);
        for (size_t i = 0; i < outFrames; ++i) {
            const double at = i * step;
            const long centre = static_cast<long>(at);
            for (unsigned c = 0; c < outCh; ++c) {
                double sum = 0.0, wsum = 0.0;
                for (long k = centre - taps + 1; k <= centre + taps; ++k) {
                    if (k < 0 || k >= static_cast<long>(frames))
                        continue;
                    const double x = (at - k) * cutoff;
                    const double sinc = std::fabs(x) < 1e-9 ? 1.0 : std::sin(pm::kPiF * x) / (pm::kPiF * x);
                    const double win = 0.5 + 0.5 * std::cos(pm::kPiF * (at - k) / taps);
                    sum += pcm[k * ch + c] * sinc * win;
                    wsum += sinc * win;
                }
                const double v = wsum != 0.0 ? sum / wsum : 0.0;
                out[i * outCh + c] = static_cast<int16_t>(std::clamp(v, -32768.0, 32767.0));
            }
        }
    }
    mp3 ? drmp3_free(pcm, nullptr) : drwav_free(pcm, nullptr);

    std::FILE* f = std::fopen(dst.c_str(), "wb");
    if (!f)
        return "cannot write \"" + dst + "\"";
    const uint32_t bytes = static_cast<uint32_t>(out.size() * 2);
    const auto u32 = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    const auto u16 = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f);
    u32(36 + bytes);
    std::fwrite("WAVEfmt ", 1, 8, f);
    u32(16);
    u16(1);
    u16(static_cast<uint16_t>(outCh));
    u32(outRate);
    u32(outRate * outCh * 2);
    u16(static_cast<uint16_t>(outCh * 2));
    u16(16);
    std::fwrite("data", 1, 4, f);
    u32(bytes);
    std::fwrite(out.data(), 2, out.size(), f);
    const bool ok = std::fclose(f) == 0;

    note = std::string(mp3 ? "mp3" : "wav") + " " + std::to_string(rate) + " Hz " +
           std::to_string(ch) + "ch -> wav " + std::to_string(outRate) + " Hz 16-bit " +
           std::to_string(outCh) + "ch";
    return ok ? std::string() : "cannot write \"" + dst + "\"";
}

// An animation-only model: the reference skeleton, its bonemerge tags, and one
// $sequence per clip. Joints go out exactly as the decompiler writes them.
bool WriteAnimQc(const std::string& path, const Mdl& m, const std::string& modelname,
                 const std::vector<Clip>& clips, bool fovBone) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        return false;
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones = m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    const std::vector<std::string> names = mdldecompiler::BoneNames(m);
    const std::vector<mdldecompiler::LocalPose> poses = mdldecompiler::LocalPoses(m);
    const auto deg = [](float r) {
        return mdldecompiler::F(static_cast<float>(std::round(r * pm::kRad2Deg * 10000.0) / 10000.0));
    };

    std::fprintf(f, "// vmd2valvebiped: animation-only model, $includemodel it from the character.\n");
    std::fprintf(f, "$modelname \"%s\"\n\n", modelname.c_str());
    for (int i = 0; i < h.numbones; ++i) {
        const int p = bones[i].parent;
        const std::string parent = (p >= 0 && p < h.numbones) ? names[p] : std::string();
        // QAngle order (pitch yaw roll); the identity sextet marks it pre-aligned
        std::fprintf(f, "$definebone \"%s\" \"%s\" %s %s %s %s  0 0 0 0 0 0\n", names[i].c_str(),
                     parent.c_str(), mdldecompiler::V3(poses[i].pos).c_str(), deg(poses[i].rot.y).c_str(),
                     deg(poses[i].rot.z).c_str(), deg(poses[i].rot.x).c_str());
    }
    if (std::any_of(clips.begin(), clips.end(), [](const Clip& c) { return c.camera; })) {
        std::fprintf(f, "$definebone \"%s\" \"\" 0 0 0 0 0 0  0 0 0 0 0 0\n", kCameraBone);
        if (fovBone)
            std::fprintf(f, "$definebone \"%s\" \"%s\" 0 0 0 0 0 0  0 0 0 0 0 0\n", kCameraFovBone,
                         kCameraBone);
    }
    bool merge = false;
    for (int i = 0; i < h.numbones; ++i) {
        if (!(bones[i].flags & fm::BONE_USED_BY_BONE_MERGE))
            continue;
        std::fprintf(f, merge ? "" : "\n");
        std::fprintf(f, "$bonemerge \"%s\"\n", names[i].c_str());
        merge = true;
    }
    std::fprintf(f, "\n$animblocksize 64\n\n");
    for (const Clip& c : clips) {
        std::fprintf(f, "$sequence \"%s\" \"%s.%s\" fps %d", c.name.c_str(), c.name.c_str(),
                     c.camera ? "smd" : "dmx", c.fps);
        // client-side, fires at sequence start; a .wav/.mp3 name skips soundscripts
        if (!c.sound.empty())
            std::fprintf(f, " {\n    event AE_CL_PLAYSOUND 0 \"%s\"\n}", c.sound.c_str());
        std::fputc('\n', f);
    }
    std::fclose(f);
    return true;
}

} // namespace

int main(int argc, char** argv) {
    PrintHeader();
    std::vector<std::string> vmdPaths;
    std::string mdlPath, pmxPath, outPath, compileName, gameDir, srcArg, faceMapPath;
    std::string faceSet = "facs", cameraFov;
    float armAngle = 35.0f, eyeRange = 30.0f;
    long startArg = -1, endArg = -1;
    bool uncompress = false, sound = false;
    std::string dmxEncoding = "binary";
    int dmxModel = 15;
    for (int i = 1; i < argc; ++i) {
        const char* a = argv[i];
        const auto next = [&]() -> const char* { return (i + 1 < argc) ? argv[++i] : ""; };
        if (!_stricmp(a, "-vmd")) vmdPaths.push_back(next());
        else if (!_stricmp(a, "-mdl")) mdlPath = next();
        else if (!_stricmp(a, "-pmx")) pmxPath = next();
        else if (!_stricmp(a, "-o")) outPath = next();
        else if (!_stricmp(a, "-compile")) compileName = next();
        else if (!_stricmp(a, "-srcdir")) srcArg = next();
        else if (!_stricmp(a, "-game")) gameDir = next();
        else if (!_stricmp(a, "-armangle")) armAngle = static_cast<float>(std::atof(next()));
        else if (!_stricmp(a, "-eyerange")) eyeRange = static_cast<float>(std::atof(next()));
        else if (!_stricmp(a, "-start")) startArg = std::atol(next());
        else if (!_stricmp(a, "-end")) endArg = std::atol(next());
        else if (!_stricmp(a, "-uncompressanim")) uncompress = true;
        else if (!_stricmp(a, "-sound")) sound = true;
        else if (!_stricmp(a, "-dmxencoding")) dmxEncoding = next();
        else if (!_stricmp(a, "-dmxmodel")) dmxModel = std::atoi(next());
        else if (!_stricmp(a, "-facemap")) faceMapPath = next();
        else if (!_stricmp(a, "-faceset")) faceSet = next();
        else if (!_stricmp(a, "-camerafov")) cameraFov = next();
        else return Usage();
    }
    if (vmdPaths.empty() || mdlPath.empty())
        return Usage();
    if (!outPath.empty() && (vmdPaths.size() > 1 || !compileName.empty()))
        return Fail("-o names one .dmx; it cannot be used with several -vmd or with -compile");
    if (sound && compileName.empty())
        return Fail("-sound writes a sequence event, so it needs -compile");
    if (const char* e = mdldecompiler::SetDmxOutput(dmxEncoding, dmxModel))
        return Fail(e);
    if (!cameraFov.empty() && cameraFov != "bone" && cameraFov != "dmx")
        return Fail("unknown -camerafov \"" + cameraFov + "\" (bone or dmx)");
    const bool fovBone = cameraFov == "bone";

    std::string err;
    MmdSkel mmd = BuildMmdSkel();
    BuiltinArms(mmd, armAngle);
    if (!pmxPath.empty() && !LoadPmxRest(pmxPath, mmd, err))
        return Fail(err);
    if (faceSet != "facs" && faceSet != "visemes")
        return Fail("unknown -faceset \"" + faceSet + "\" (facs or visemes)");
    if (eyeRange <= 0.0f)
        return Fail("-eyerange must be above 0 degrees");
    FaceMap faceMap = BuiltinFaceMap(faceSet == "visemes");
    if (!faceMapPath.empty() && !LoadFaceMap(faceMapPath, faceMap, err))
        return Fail(err);

    Mdl m;
    if (!LoadMdl(mdlPath, m, err))
        return Fail(err);
    const fm::studiohdr_t& h = *m.hdr;
    const fm::mstudiobone_t* bones = m.At<fm::mstudiobone_t>(m.buf.data(), h.boneindex, h.numbones);
    if (!bones)
        return Fail("\"" + mdlPath + "\" has no readable bones");
    std::vector<mdldecompiler::FlexTrack> ctrls;
    const fm::mstudioflexcontroller_t* fcs =
        m.At<fm::mstudioflexcontroller_t>(m.buf.data(), h.flexcontrollerindex, h.numflexcontrollers);
    for (int i = 0; fcs && i < h.numflexcontrollers; ++i)
        ctrls.push_back({m.Str(&fcs[i], fcs[i].sznameindex), fcs[i].min, fcs[i].max, {}});

    Target t;
    t.n = h.numbones;
    t.names = mdldecompiler::BoneNames(m);
    const std::vector<mdldecompiler::LocalPose> poses = mdldecompiler::LocalPoses(m);
    t.parent.resize(t.n);
    t.local.resize(t.n);
    t.world.resize(t.n);
    for (int i = 0; i < t.n; ++i) {
        const int p = bones[i].parent;
        t.parent[i] = (p >= 0 && p < i) ? p : -1;
        pm::AngleMatrix(poses[i].rot, poses[i].pos, t.local[i]);
        t.world[i] = t.parent[i] >= 0 ? pm::ConcatTransforms(t.world[t.parent[i]], t.local[i]) : t.local[i];
    }

    const int pelvis = t.Find("Pelvis");
    const int lThigh = t.Find("L_Thigh"), rThigh = t.Find("R_Thigh");
    int top = t.Find("Head1");
    if (top < 0) top = t.Find("Neck1");
    if (top < 0) top = t.Find("Spine4");
    if (pelvis < 0 || lThigh < 0 || rThigh < 0 || top < 0)
        return Fail("\"" + mdlPath + "\" is not a ValveBiped skeleton (needs Pelvis, L/R_Thigh, Head1)");

    // A bind that is not upright (Pelvis left at the origin, unrotated, so only the
    // rig's clips stand it) is stood up in those clips' frame - up +Z, left +X,
    // facing -Y - ankles off the floor by a survivor's ankle-to-leg ratio.
    {
        Vector3 bu = Sub(Origin(t.world[top]), Origin(t.world[pelvis]));
        pm::VectorNormalize(bu);
        if (bu.z < 0.9f) {
            Vector3 bl = Sub(Origin(t.world[lThigh]), Origin(t.world[rThigh]));
            bl = Sub(bl, Mul(bu, Dot(bl, bu)));
            pm::VectorNormalize(bl);
            const Vector3 bf = Cross(bl, bu);
            matrix3x4 from, to;
            for (int r = 0; r < 3; ++r) {
                from.m[r][0] = (&bl.x)[r];
                from.m[r][1] = (&bu.x)[r];
                from.m[r][2] = (&bf.x)[r];
            }
            to.m[0][0] = 1; to.m[1][0] = 0;  to.m[2][0] = 0; // left  +X
            to.m[0][1] = 0; to.m[1][1] = 0;  to.m[2][1] = 1; // up    +Z
            to.m[0][2] = 0; to.m[1][2] = -1; to.m[2][2] = 0; // front -Y
            const matrix3x4 stand = pm::ConcatTransforms(to, Transpose(from));
            const auto rebuild = [&]() {
                for (int i = 0; i < t.n; ++i)
                    t.world[i] = t.parent[i] >= 0 ? pm::ConcatTransforms(t.world[t.parent[i]], t.local[i])
                                                  : t.local[i];
            };
            for (int i = 0; i < t.n; ++i)
                if (t.parent[i] < 0)
                    t.local[i] = pm::ConcatTransforms(stand, t.local[i]);
            rebuild();

            const int lf = t.Find("L_Foot"), rf = t.Find("R_Foot"), lc = t.Find("L_Calf");
            if (lf >= 0 && rf >= 0 && lc >= 0) {
                const float leg = Len(Sub(Origin(t.world[lc]), Origin(t.world[lThigh]))) +
                                  Len(Sub(Origin(t.world[lf]), Origin(t.world[lc])));
                const float lift = 0.078f * leg - std::min(Origin(t.world[lf]).z, Origin(t.world[rf]).z);
                for (int i = 0; i < t.n; ++i)
                    if (t.parent[i] < 0)
                        t.local[i].m[2][3] += lift;
                rebuild();
            }
            std::printf("bind pose is not upright - stood up for retargeting\n");
        }
    }

    // Leg chains come from the model's own $ikchain data.
    Leg legs[2];
    const fm::mstudioikchain_t* chains =
        m.At<fm::mstudioikchain_t>(m.buf.data(), h.ikchainindex, h.numikchains);
    for (int c = 0; chains && c < h.numikchains; ++c) {
        if (chains[c].numlinks != 3)
            continue;
        const fm::mstudioiklink_t* link =
            m.At<fm::mstudioiklink_t>(&chains[c], chains[c].linkindex, 3);
        if (!link || link[2].bone < 0 || link[2].bone >= t.n || link[0].bone < 0 ||
            link[0].bone >= t.n || link[1].bone < 0 || link[1].bone >= t.n)
            continue;
        for (int side = 0; side < 2; ++side)
            if (link[2].bone == t.Find(side ? "R_Foot" : "L_Foot"))
                legs[side] = {link[0].bone, link[1].bone, link[2].bone, link[0].kneeDir,
                              mmd.Find("legik", side)};
    }
    if (legs[0].foot < 0 || legs[1].foot < 0)
        return Fail("\"" + mdlPath + "\" has no $ikchain ending at ValveBiped.Bip01_L_Foot and "
                    "_R_Foot - leg IK needs both");

    // MMD -> target basis from each rig's own left/up/forward. MMD is left-handed
    // Y-up facing -Z, so this is a reflection and M*R*Mt stays a proper rotation.
    Vector3 up = Sub(Origin(t.world[top]), Origin(t.world[pelvis]));
    pm::VectorNormalize(up);
    Vector3 left = Sub(Origin(t.world[lThigh]), Origin(t.world[rThigh]));
    left = Sub(left, Mul(up, Dot(left, up)));
    pm::VectorNormalize(left);
    const Vector3 fwd = Cross(left, up);
    matrix3x4 M;
    for (int r = 0; r < 3; ++r) {
        M.m[r][0] = (&left.x)[r];
        M.m[r][1] = (&up.x)[r];
        M.m[r][2] = -(&fwd.x)[r];
    }
    const matrix3x4 Mt = Transpose(M);

    // Bind pose settles which way the hinge bends: the $ikchain knee direction,
    // or the bind knee's own offset when the chain has none.
    for (Leg& g : legs) {
        const Vector3 thigh = Origin(t.world[g.thigh]);
        Vector3 dir = Sub(Origin(t.world[g.foot]), thigh);
        pm::VectorNormalize(dir);
        Vector3 knee = pm::VectorRotate(g.kneeDir, t.world[g.thigh]);
        if (Len(knee) < 1e-4f)
            knee = Sub(Origin(t.world[g.knee]), thigh);
        g.bendSign = Dot(Cross(dir, left), knee) < 0.0f ? -1.0f : 1.0f;
    }

    const auto restOf = [&](const char* id, int side) { return mmd.bones[mmd.Find(id, side)].rest; };
    const float mmdLeg = Len(Sub(restOf("knee", 0), restOf("leg", 0))) +
                         Len(Sub(restOf("ankle", 0), restOf("knee", 0)));
    const float tgtLeg = Len(Sub(Origin(t.world[legs[0].knee]), Origin(t.world[legs[0].thigh]))) +
                         Len(Sub(Origin(t.world[legs[0].foot]), Origin(t.world[legs[0].knee])));
    if (mmdLeg < 1e-4f || tgtLeg < 1e-4f)
        return Fail("zero-length leg, cannot derive the motion scale");
    const float scale = tgtLeg / mmdLeg;

    // Per target bone: its MMD source and the rest alignment O that poses the
    // target's arms into the MMD rest pose. Fingers take the hand's O.
    std::vector<int> src(t.n, -1);
    std::vector<matrix3x4> align(t.n);
    int mapped = 0;
    std::string absent;
    for (const MapDef& d : kMap) {
        for (int side = 0; side < (d.sided ? 2 : 1); ++side) {
            const std::string vn = d.sided ? std::string(side ? "R_" : "L_") + d.valve : d.valve;
            const int i = t.Find(vn);
            if (i < 0) {
                absent += " " + vn;
                continue;
            }
            src[i] = mmd.Find(d.mmd, d.sided ? side : -1);
            ++mapped;
        }
    }
    for (int side = 0; side < 2; ++side) {
        const char* s = side ? "R_" : "L_";
        const auto tb = [&](const char* n) { return t.Find(std::string(s) + n); };
        const auto tdir = [&](int a, int b) {
            return (a < 0 || b < 0) ? Vector3{} : Sub(Origin(t.world[b]), Origin(t.world[a]));
        };
        const auto mdir = [&](const char* a, const char* b) {
            return pm::VectorRotate(Sub(restOf(b, side), restOf(a, side)), M);
        };
        const int cla = tb("Clavicle"), ua = tb("UpperArm"), fa = tb("Forearm"), hand = tb("Hand");
        struct Seg { int bone; Vector3 from, to; };
        Vector3 handT = tdir(hand, tb("Finger2"));
        Vector3 handM = mdir("wrist", "middle1");
        if (Len(handT) < 1e-4f) handT = tdir(fa, hand);
        if (Len(handM) < 1e-4f) handM = mdir("elbow", "wrist");
        const Seg segs[] = {{cla, tdir(cla, ua), mdir("shoulder", "arm")},
                            {ua, tdir(ua, fa), mdir("arm", "elbow")},
                            {fa, tdir(fa, hand), mdir("elbow", "wrist")},
                            {hand, handT, handM}};
        for (Seg g : segs) {
            if (g.bone < 0 || pm::VectorNormalize(g.from) < 1e-4f || pm::VectorNormalize(g.to) < 1e-4f)
                continue;
            align[g.bone] = RotationBetween(g.from, g.to);
        }
        if (hand >= 0)
            for (int i = 0; i < t.n; ++i)
                if (src[i] >= 0 && t.names[i].find(std::string("Bip01_") + s + "Finger") != std::string::npos)
                    align[i] = align[hand];
    }

    const auto toTarget = [&](const matrix3x4& g) {
        return pm::ConcatTransforms(M, pm::ConcatTransforms(RotOnly(g), Mt));
    };

    // -compile: the clips, .qc and compile all go through one source folder.
    std::error_code ec;
    std::string srcDir;
    if (!compileName.empty()) {
        if (gameDir.empty())
            gameDir = fs::path(vmdPaths[0]).parent_path().string();
        srcDir = !srcArg.empty() ? srcArg
                                 : (fs::path(vmdPaths[0]).parent_path() / (ClipName(compileName) + "_src")).string();
        fs::create_directories(srcDir, ec);
    }
    const fs::path dataDir = compileName.empty() ? fs::path() : fs::path(gameDir) / "datamodel";
    if (!dataDir.empty())
        fs::create_directories(dataDir / "flex_anims", ec);
    std::set<std::string> faceMissing;

    const int lower = mmd.Find("lower");
    const Vector3 lowerRest = mmd.bones[lower].rest;
    const long limit = pulse::limits::kMaxAnimFrames;
    // MMD space -> target: the MMD lower-body rest lands on the bind Pelvis
    const Vector3 camAnchor = Sub(Origin(t.world[pelvis]), Mul(pm::VectorRotate(lowerRest, M), scale));
    std::vector<Clip> clips;
    std::vector<matrix3x4> G(mmd.bones.size()), W(t.n), L(t.n);

    for (const std::string& vmdPath : vmdPaths) {
        std::map<std::string, std::vector<Key>> keys;
        Morphs morphs;
        std::vector<CamKey> cams;
        uint32_t lastFrame = 0;
        if (!LoadVmd(vmdPath, keys, morphs, cams, lastFrame, err))
            return Fail(err);
        std::vector<const std::vector<Key>*> mmdKeys(mmd.bones.size(), nullptr);
        int keyed = 0;
        for (size_t b = 0; b < mmd.bones.size(); ++b) {
            const auto it = keys.find(mmd.bones[b].sj);
            if (it != keys.end()) {
                mmdKeys[b] = &it->second;
                ++keyed;
            }
        }
        // camera/light/morph-only motions carry no body keys
        if (!keyed) {
            std::printf("skipped %s - no standard MMD bone is keyed\n", vmdPath.c_str());
            continue;
        }

        // Frame budget: past the engine limit the clip is resampled over the same
        // duration at a lower integer fps, never cut.
        const long first = startArg >= 0 ? startArg : 0;
        const long last = endArg >= 0 ? std::min<long>(endArg, lastFrame) : static_cast<long>(lastFrame);
        if (last < first)
            return Fail("\"" + vmdPath + "\": -start is past -end or the motion's last key");
        const long span = last - first;
        int fps = 30;
        long count = span + 1;
        if (count > limit && !uncompress) {
            fps = std::max(1, static_cast<int>(30 * (limit - 1) / span));
            count = span * fps / 30 + 1;
        }

        std::vector<float> times;
        for (long k = 0; k < count; ++k)
            times.push_back(static_cast<float>(first) + static_cast<float>(k) * 30.0f / fps);

        std::vector<std::vector<AnimPose>> frames;
        frames.reserve(static_cast<size_t>(count));
        Vector3 lastBend[2];
        for (long k = 0; k < count; ++k) {
            const float f = times[static_cast<size_t>(k)];
            for (size_t b = 0; b < mmd.bones.size(); ++b) {
                Vector3 pos;
                pm::Quaternion rot;
                if (mmdKeys[b])
                    Sample(*mmdKeys[b], f, pos, rot);
                const MmdBone& mb = mmd.bones[b];
                const Vector3 off = mb.parent >= 0 ? Sub(mb.rest, mmd.bones[mb.parent].rest) : mb.rest;
                const matrix3x4 lm = pm::QuaternionMatrix(rot, Add(off, pos));
                G[b] = mb.parent >= 0 ? pm::ConcatTransforms(G[mb.parent], lm) : lm;
            }

            for (int i = 0; i < t.n; ++i) {
                const int p = t.parent[i];
                if (src[i] < 0) {
                    L[i] = t.local[i];
                    W[i] = p >= 0 ? pm::ConcatTransforms(W[p], L[i]) : L[i];
                    continue;
                }
                Vector3 pos = p >= 0 ? pm::VectorTransform(Origin(t.local[i]), W[p]) : Origin(t.local[i]);
                if (i == pelvis)
                    pos = Add(Origin(t.world[i]),
                              Mul(pm::VectorRotate(Sub(Origin(G[lower]), lowerRest), M), scale));
                W[i] = pm::ConcatTransforms(toTarget(G[src[i]]),
                                            pm::ConcatTransforms(align[i], RotOnly(t.world[i])));
                SetOrigin(W[i], pos);
                L[i] = p >= 0 ? pm::ConcatTransforms(pm::MatrixInvert(W[p]), W[i]) : W[i];
            }

            for (int side = 0; side < 2; ++side) {
                const Leg& g = legs[side];
                const Vector3 ikRest = mmd.bones[g.ik].rest;
                const Vector3 goal = Add(Origin(t.world[g.foot]),
                                         Mul(pm::VectorRotate(Sub(Origin(G[g.ik]), ikRest), M), scale));
                // The knee hinges on the hip's side axis, as MMD's leg IK does - foot
                // yaw never swings it. A leg lying along that axis keeps last frame's.
                Vector3 dir = Sub(goal, Origin(W[g.thigh]));
                pm::VectorNormalize(dir);
                Vector3 bend = Mul(Cross(dir, pm::VectorRotate(left, toTarget(G[lower]))), g.bendSign);
                if (Len(bend) < 0.3f && Len(lastBend[side]) > 0.0f)
                    bend = lastBend[side];
                pm::VectorNormalize(bend);
                lastBend[side] = bend;
                SolveTwoBoneIK(g.thigh, g.knee, g.foot, goal, bend, W);
                const Vector3 footPos = Origin(W[g.foot]);
                W[g.foot] = pm::ConcatTransforms(toTarget(G[g.ik]), RotOnly(t.world[g.foot]));
                SetOrigin(W[g.foot], footPos);
                for (int b : {g.thigh, g.knee, g.foot}) {
                    const int p = t.parent[b];
                    L[b] = p >= 0 ? pm::ConcatTransforms(pm::MatrixInvert(W[p]), W[b]) : W[b];
                }
            }

            // Roots stay in the stored bind space, the frame the model's own
            // decompiled clips use - no inverse compile yaw.
            std::vector<AnimPose> fr(static_cast<size_t>(t.n));
            for (int i = 0; i < t.n; ++i)
                pm::MatrixAngles(L[i], fr[i].rot, fr[i].pos);
            frames.push_back(std::move(fr));
        }

        // -compile names clips for the script; a repeat takes an ordinal
        std::string path = outPath, clip;
        if (!srcDir.empty()) {
            clip = ClipName(vmdPath);
            const std::string base = clip;
            for (int n = 2; std::any_of(clips.begin(), clips.end(), [&](const Clip& c) { return c.name == clip; }); ++n)
                clip = base + "_" + std::to_string(n);
            path = (fs::path(srcDir) / (clip + ".dmx")).string();
        } else {
            if (path.empty())
                path = mdldecompiler::StripExt(vmdPath) + ".dmx";
            clip = mdldecompiler::StripExt(mdldecompiler::BaseName(path));
        }

        // face: the motion's own morphs, each one <vmd name>_face.vmd also keys taken from there
        const std::string faceVmd = mdldecompiler::StripExt(vmdPath) + "_face.vmd";
        const bool faceFile = fs::exists(faceVmd, ec);
        if (faceFile) {
            std::map<std::string, std::vector<Key>> unusedKeys;
            std::vector<CamKey> unusedCams;
            Morphs faceMorphs;
            uint32_t faceLast = 0;
            if (!LoadVmd(faceVmd, unusedKeys, faceMorphs, unusedCams, faceLast, err))
                return Fail(err);
            for (auto& [name, fk] : faceMorphs)
                morphs[name] = std::move(fk);
            for (const char* eye : kEyeBones)
                if (unusedKeys.count(eye))
                    keys[eye] = std::move(unusedKeys[eye]);
        }
        int unmapped = 0;
        std::vector<mdldecompiler::FlexTrack> face =
            FaceTracks(morphs, faceMap, ctrls, times, faceMissing, unmapped);
        const size_t faceCount = face.size();
        AddEyeTracks(keys, ctrls, times, eyeRange, face);
        const std::vector<mdldecompiler::FlexTrack>* flex = face.empty() ? nullptr : &face;

        if (!mdldecompiler::WriteAnimationDmx(m, path, clip, fps, frames,
                                              dmxModel == 22 && srcDir.empty() ? flex : nullptr))
            return Fail("cannot write \"" + path + "\"");
        clips.push_back({clip, fps, std::string()});

        std::printf("wrote %s\n", path.c_str());
        std::printf("  %ld frame%s at %d fps", count, count == 1 ? "" : "s", fps);
        if (fps != 30)
            std::printf(" (resampled from %ld MMD frames to fit the %ld-frame limit)", span + 1, limit);
        std::printf("\n  %d MMD bones keyed, %d target bones mapped, legs by $ikchain\n", keyed, mapped);
        if (!morphs.empty())
            std::printf("  %zu MMD morphs from %s -> %zu flex controllers, %d morphs unmapped\n",
                        morphs.size(), faceFile ? "the motion + _face.vmd" : "the motion", face.size(),
                        unmapped);
        if (face.size() > faceCount)
            std::printf("  eye bones -> %zu gaze controllers (-1..1 of %g degrees)\n",
                        face.size() - faceCount, eyeRange);
        if (flex && dmxModel != 22 && dataDir.empty())
            std::printf("  face keys need -dmxmodel 22 - not written\n");

        // -compile: the SFM / PulseWorkshop copy, always model 22
        if (!dataDir.empty()) {
            const std::string dataPath = (dataDir / "flex_anims" / (clip + ".dmx")).string();
            mdldecompiler::SetDmxOutput(dmxEncoding, 22);
            const bool ok = mdldecompiler::WriteAnimationDmx(m, dataPath, clip, fps, frames, flex);
            mdldecompiler::SetDmxOutput(dmxEncoding, dmxModel);
            if (!ok)
                return Fail("cannot write \"" + dataPath + "\"");
            std::printf("wrote %s\n", dataPath.c_str());
        }
        const size_t body = clips.size() - 1;

        // camera: <vmd name>_camera.vmd beside the motion, else the motion's own keys,
        // sampled over the body clip's frames so the two stay in sync
        const std::string camVmd = mdldecompiler::StripExt(vmdPath) + "_camera.vmd";
        const bool camFile = fs::exists(camVmd, ec);
        if (camFile) {
            std::map<std::string, std::vector<Key>> unused;
            uint32_t camLast = 0;
            cams.clear();
            Morphs unusedMorphs;
            if (!LoadVmd(camVmd, unused, unusedMorphs, cams, camLast, err))
                return Fail(err);
        }
        if (!cams.empty()) {
            std::vector<AnimPose> cf(static_cast<size_t>(count));
            std::vector<float> fov(static_cast<size_t>(count));
            for (long k = 0; k < count; ++k) {
                const float f = static_cast<float>(first) + static_cast<float>(k) * 30.0f / fps;
                Vector3 eye, look, up;
                SampleCamera(cams, f, eye, look, up, fov[static_cast<size_t>(k)]);
                look = pm::VectorRotate(look, M);
                up = pm::VectorRotate(up, M);
                const Vector3 lv = Cross(up, look);
                matrix3x4 cm;
                for (int r = 0; r < 3; ++r) {
                    cm.m[r][0] = (&look.x)[r];
                    cm.m[r][1] = (&lv.x)[r];
                    cm.m[r][2] = (&up.x)[r];
                }
                SetOrigin(cm, Add(camAnchor, Mul(pm::VectorRotate(eye, M), scale)));
                pm::MatrixAngles(cm, cf[static_cast<size_t>(k)].rot, cf[static_cast<size_t>(k)].pos);
            }
            const std::string camClip = clip + "_camera";
            const std::string camPath = (fs::path(path).parent_path() / (camClip + ".smd")).string();
            if (!WriteCameraSmd(camPath, cf, fovBone ? &fov : nullptr))
                return Fail("cannot write \"" + camPath + "\"");
            clips.push_back({camClip, fps, std::string(), true});
            const auto [fovLo, fovHi] = std::minmax_element(fov.begin(), fov.end());
            std::printf("wrote %s\n  %zu camera keys from %s, fov %g-%g degrees\n", camPath.c_str(),
                        cams.size(), camFile ? camVmd.c_str() : "the motion", *fovLo, *fovHi);
            if (srcDir.empty()) {
                std::printf("  $sequence \"%s\" \"%s.smd\" fps %d - needs $definebone \"%s\" \"\" 0 0 0 0 0 0  0 0 0 0 0 0\n",
                            camClip.c_str(), camClip.c_str(), fps, kCameraBone);
                if (fovBone)
                    std::printf("  and $definebone \"%s\" \"%s\" 0 0 0 0 0 0  0 0 0 0 0 0\n", kCameraFovBone,
                                kCameraBone);
            }
            if (cameraFov == "dmx") {
                const fs::path camDir = dataDir.empty() ? fs::path(camPath).parent_path() : dataDir / "camera_anims";
                fs::create_directories(camDir, ec);
                const std::string camDmx = (camDir / (camClip + ".dmx")).string();
                if (!mdldecompiler::WriteCameraDmx(camDmx, camClip, fps, cf, fov))
                    return Fail("cannot write \"" + camDmx + "\"");
                std::printf("wrote %s\n", camDmx.c_str());
            }
        }

        // -sound: <vmd name>.wav (else .mp3) beside the .vmd, converted into
        // <game>/sound/mmd/<clip>.wav and played from frame 0 of the sequence
        if (sound) {
            std::string audio;
            for (const char* ext : {".wav", ".mp3"})
                if (fs::exists(mdldecompiler::StripExt(vmdPath) + ext, ec)) {
                    audio = mdldecompiler::StripExt(vmdPath) + ext;
                    break;
                }
            if (audio.empty()) {
                std::printf("  no .wav or .mp3 named like the .vmd - no sound\n");
                continue;
            }
            const std::string rel = "mmd/" + clip + ".wav";
            if (rel.size() > 64)
                return Fail("sound path \"" + rel + "\" is over the 64-character event limit - "
                            "rename the .vmd shorter");
            const fs::path dst = fs::path(gameDir) / "sound" / "mmd" / (clip + ".wav");
            fs::create_directories(dst.parent_path(), ec);
            std::string note;
            const std::string why = ConvertAudio(audio, dst.string(), note);
            if (!why.empty())
                return Fail("\"" + audio + "\" " + why);
            clips[body].sound = rel;
            std::printf("  sound %s (%s)\n", dst.string().c_str(), note.c_str());
            if (first > 0)
                std::printf("  warning: -start trims the motion but not the audio - they will not line up\n");
        }
        if (srcDir.empty())
            std::printf("  $sequence \"%s\" \"%s\" fps %d\n", clip.c_str(),
                        mdldecompiler::BaseName(path).c_str(), fps);
    }
    if (clips.empty())
        return Fail("none of the .vmd files keys a standard MMD bone - nothing to write");
    if (!absent.empty())
        std::printf("not on this model:%s\n", absent.c_str());
    if (!faceMissing.empty()) {
        std::printf("flex controllers not on this model:");
        for (const std::string& c : faceMissing)
            std::printf(" %s", c.c_str());
        std::printf("\n");
    }
    if (srcDir.empty())
        return 0;

    const std::string qc = (fs::path(srcDir) / (ClipName(compileName) + ".qc")).string();
    if (!WriteAnimQc(qc, m, compileName, clips, fovBone))
        return Fail("cannot write \"" + qc + "\"");
    std::printf("wrote %s\n\n", qc.c_str());

    std::string cmd = "\"" + CompilerPath(argv[0]) + "\" -game \"" + gameDir + "\" \"" + qc + "\"";
#ifdef _WIN32
    cmd = "\"" + cmd + "\""; // cmd /c strips one outer pair of quotes
#endif
    std::fflush(stdout);
    if (std::system(cmd.c_str()) != 0)
        return Fail("mdlcompiler failed on \"" + qc + "\" - the .qc and clips are kept for a rerun");
    return 0;
}
