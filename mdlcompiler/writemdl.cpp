// writemdl.cpp - .mdl + .vvd writers and the post-write fixup pass.
//
// (WriteModelFiles/WriteBoneInfo/WriteAnimations/WriteSequenceInfo/WriteModel/
// WriteTextures/WriteBoneTransforms/WriteStringTable/WriteVertices +
// FixupToSortedLODVertexes). Block order, alignment macros, string-table
// insertion order and the checksum-before-fixup sequencing are all load-bearing.

#include "writer.h"
#include "perf.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <unordered_map>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#else
#include <sys/mman.h>
#endif

#include "format/mdl.h"
#include "format/vvd.h"
#include "format/vtx.h"
#include "math/compressed.h"
#include "pulselimits.h"

namespace pulse::writer {

namespace fmt = pulse::format;
namespace pm = pulse::math;
namespace cm = pulse::compile;
namespace lim = pulse::limits;

// filled by BuildVtx in writevtx.cpp, printed by WriteModelFiles below
std::vector<std::string> g_vtxReport;
std::vector<std::string> g_writtenFiles;

namespace {

// Starting commit for the .mdl and .ani buffers, whose payload has no cheap
// up-front estimate. Not a cap - Buf grows past it. The .vvd does have an
// estimate, so it is sized exactly - see VvdBufferSize.
constexpr size_t kFileBuffer = 32 * 1024 * 1024;

// 48 bytes of vertex + 16 of tangent each, plus header, fixup table, and
// alignment slack: each block costs one ALIGN16 in the vertex pass and one
// more in the tangent pass, so budget 32 per block and a little over.
size_t VvdBufferSize(size_t numVerts, size_t numBlocks, size_t numFixups) {
    return sizeof(fmt::vertexFileHeader_t) + numFixups * sizeof(fmt::vertexFileFixup_t) +
           numVerts * (sizeof(fmt::mstudiovertex_t) + 4 * sizeof(float)) + (numBlocks + 4) * 32;
}

// Reference console report (plus the nested
// flexcontrollers/ik-pose/eyeballs/flexes lines): one
// byte count per block as the file is laid down. The reference prints these
// inline while writing because it knows the output path up front; this writer
// resolves the path at save time, so the lines are collected here and flushed
// under the "writing <path>:" header instead. Same output, same order.
std::vector<std::string> g_mdlReport;
std::vector<std::string> g_vvdReport;

void Report(std::vector<std::string>& into, const char* fmt, ...) {
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    into.emplace_back(line);
}

void FlushReport(std::vector<std::string>& from) {
    for (const std::string& line : from)
        std::printf("%s\n", line.c_str());
    from.clear();
}

// Address-space backing for Buf. Windows must commit explicitly; POSIX
// anonymous maps are demand-paged, so the whole range is mapped at once.
#ifdef _WIN32
uint8_t* MapReserve(size_t bytes) {
    return static_cast<uint8_t*>(VirtualAlloc(nullptr, bytes, MEM_RESERVE, PAGE_READWRITE));
}
bool MapCommit(uint8_t* base, size_t bytes) {
    return VirtualAlloc(base, bytes, MEM_COMMIT, PAGE_READWRITE) != nullptr;
}
void MapRelease(uint8_t* base, size_t) { VirtualFree(base, 0, MEM_RELEASE); }
#else
uint8_t* MapReserve(size_t bytes) {
    void* p = mmap(nullptr, bytes, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
    return p == MAP_FAILED ? nullptr : static_cast<uint8_t*>(p);
}
bool MapCommit(uint8_t*, size_t) { return true; }
void MapRelease(uint8_t* base, size_t bytes) { munmap(base, bytes); }
#endif

// Linear append buffer with reference ALIGN semantics.
//
// Callers hold raw pointers into it across the whole write, so it can never
// move: it reserves a large range up front and commits more as `pos` advances.
// Pages arrive zeroed either way, matching the old zero-filled vector.
struct Buf {
    static constexpr size_t kReserve = size_t(4) << 30; // address space only
    static constexpr size_t kChunk = 32 * 1024 * 1024;  // commit granularity

    uint8_t* base = nullptr;
    size_t committed = 0;
    size_t cur = 0;
    bool failed = false;

    // The write sites take p() first and only then advance, so the commit has
    // to hang off the advance - hence pos being a proxy rather than a size_t.
    struct PosProxy {
        Buf* b;
        operator size_t() const { return b->cur; }
        PosProxy& operator=(size_t v) {
            b->cur = v;
            b->Commit(v);
            return *this;
        }
        PosProxy& operator+=(size_t n) { return *this = b->cur + n; }
        PosProxy& operator++() { return *this += 1; }
        size_t operator++(int) {
            size_t was = b->cur;
            *this += 1;
            return was;
        }
    };
    PosProxy pos{this}; // pData - pStart

    explicit Buf(size_t initial) {
        base = MapReserve(kReserve);
        failed = base == nullptr;
        Commit(initial);
    }
    ~Buf() {
        if (base)
            MapRelease(base, kReserve);
    }
    Buf(const Buf&) = delete;
    Buf& operator=(const Buf&) = delete;

    // Keep one chunk of headroom past `need` so the memcpy-then-advance sites
    // (strings, keyvalues) write into committed pages.
    void Commit(size_t need) {
        if (failed || need + kChunk <= committed)
            return;
        size_t want = ((need + 2 * kChunk - 1) / kChunk) * kChunk;
        if (want > kReserve) {
            failed = true;
            return;
        }
        failed = !MapCommit(base, want);
        if (!failed)
            committed = want;
    }

    uint8_t* start() { return base; }
    // `bytes` is for the memcpy sites that write before advancing pos, when the
    // blob can be bigger than the headroom Commit leaves.
    uint8_t* p(size_t bytes = 0) {
        if (bytes)
            Commit(cur + bytes);
        return base + cur;
    }
    template <typename T>
    T* Reserve(size_t count = 1) {
        T* r = reinterpret_cast<T*>(p());
        pos += sizeof(T) * count;
        return r;
    }
    void Align4() { pos = (cur + 3) & ~size_t(3); }
    void Align16() { pos = (cur + 15) & ~size_t(15); }
    // Only trips if the reserve is exhausted or the OS refused a commit; a
    // reported error beats shipping a plausible-looking corrupt file.
    bool overflowed() const { return failed || cur > committed; }
};

// reference session string table
struct StringTable {
    struct Entry {
        size_t base;     // offset of owning struct in the buffer
        size_t ptr;      // offset of the int32 fixup slot
        std::string str;
        int dupindex;
        size_t addr = 0; // offset of the written string
    };
    std::vector<Entry> entries;

    void Begin() {
        entries.clear();
        Entry e;
        e.base = 0;
        e.ptr = SIZE_MAX;
        e.str = "";
        e.dupindex = -1;
        entries.push_back(e);
    }

    void Add(Buf& buf, const void* base, int32_t* ptr, const char* str) {
        Entry e;
        e.base = reinterpret_cast<const uint8_t*>(base) - buf.start();
        e.ptr = reinterpret_cast<uint8_t*>(ptr) - buf.start();
        e.str = str ? str : "";
        e.dupindex = -1;
        for (size_t i = 0; i < entries.size(); ++i) {
            if (entries[i].str == e.str) {
                e.dupindex = static_cast<int>(i);
                break;
            }
        }
        entries.push_back(e);
    }

    void Write(Buf& buf) {
        // force null at first address
        entries[0].addr = buf.pos;
        *buf.p() = '\0';
        buf.pos++;

        for (size_t i = 1; i < entries.size(); ++i) {
            Entry& e = entries[i];
            int32_t* fixup = reinterpret_cast<int32_t*>(buf.start() + e.ptr);
            if (e.dupindex == -1) {
                *fixup = static_cast<int32_t>(buf.pos - e.base);
                e.addr = buf.pos;
                memcpy(buf.p(), e.str.c_str(), e.str.size());
                buf.pos += e.str.size();
                *buf.p() = '\0';
                buf.pos++;
            } else {
                *fixup = static_cast<int32_t>(entries[e.dupindex].addr - e.base);
            }
        }
        buf.Align4();
    }
};

StringTable g_strings;
cm::CompiledModel* g_m = nullptr;

// ---------------------------------------------------------------------------
// $animblocksize bookkeeping (reference g_animblock / g_numanimblocks).
// Entry 0 is the pseudo-block meaning "in the .mdl, relative to the animdesc",
// so its `start` is re-pointed at each animdesc as WriteAnimations walks them.
// Entries 1.. are byte ranges of the external .ani buffer. `count` mirrors
// g_numanimblocks: 0 until the first animation with a .ani, then >= 2.
// ---------------------------------------------------------------------------
struct AnimBlockTable {
    struct Block {
        size_t start = 0;
        size_t end = 0;
    };
    std::vector<Block> blocks;
    size_t count = 0;
    std::string name; // "models/<outname>.ani", written to the string table

    void Reset() {
        blocks.assign(1, Block{});
        count = 0;
        name.clear();
    }
    // open the first real block
    void Open(size_t start) {
        blocks.resize(2);
        blocks[1].start = start;
        count = 2;
    }
    // close the current block at `at` and start the next one there
    void Advance(size_t at) {
        blocks[count - 1].end = at;
        blocks.resize(count + 1);
        blocks[count].start = at;
        ++count;
    }
};
AnimBlockTable g_animblocks;

// bone-table-by-name qsort comparator (C qsort, strcmpi - parity)
int BoneNameCompare(const void* elem1, const void* elem2) {
    int index1 = *static_cast<const uint8_t*>(elem1);
    int index2 = *static_cast<const uint8_t*>(elem2);
    return _stricmp(g_m->bones[index1].name.c_str(), g_m->bones[index2].name.c_str());
}

void CopyV3(fmt::Vector3& dst, const pm::Vector3& src) { dst = {src.x, src.y, src.z}; }
void CopyQ(fmt::Quaternion& dst, const pm::Quaternion& src) { dst = {src.x, src.y, src.z, src.w}; }
void CopyM(fmt::matrix3x4& dst, const pm::matrix3x4& src) { dst = src; }

// ---------------------------------------------------------------------------
// WriteBoneInfo
// ---------------------------------------------------------------------------
void WriteBoneInfo(Buf& buf, fmt::studiohdr_t* phdr, cm::CompiledModel& m) {
    int numbones = static_cast<int>(m.bones.size());

    fmt::mstudiobone_t* pbone = reinterpret_cast<fmt::mstudiobone_t*>(buf.p());
    phdr->numbones = numbones;
    phdr->boneindex = static_cast<int32_t>(buf.pos);

    g_strings.Add(buf, phdr, &phdr->surfacepropindex, m.surfaceprop.c_str());
    phdr->contents = m.contents;

    for (int i = 0; i < numbones; i++) {
        cm::Bone& b = m.bones[i];
        g_strings.Add(buf, &pbone[i], &pbone[i].sznameindex, b.name.c_str());
        pbone[i].parent = b.parent;
        pbone[i].flags = b.flags;
        pbone[i].procindex = 0;
        pbone[i].physicsbone = b.physicsbone;
        CopyV3(pbone[i].pos, b.pos);
        CopyV3(pbone[i].rot, b.rot);
        CopyV3(pbone[i].posscale, b.posscale);
        CopyV3(pbone[i].rotscale, b.rotscale);
        CopyM(pbone[i].poseToBone, pm::MatrixInvert(b.boneToPose));
        CopyQ(pbone[i].qAlignment, b.qAlignment);

        pm::Quaternion q;
        pm::AngleQuaternion(b.rot, q);
        pm::Quaternion aligned = q;
        pm::QuaternionAlign(b.qAlignment, q, aligned);
        CopyQ(pbone[i].quat, aligned);

        // per-bone surfaceprop defaults to the model's; contents arrives
        // already resolved by ApplyJointContents (which seeds it from the
        // model word), so a $jointcontents of 0 stays 0
        g_strings.Add(buf, &pbone[i], &pbone[i].surfacepropidx,
                      b.surfaceprop.empty() ? m.surfaceprop.c_str() : b.surfaceprop.c_str());
        pbone[i].contents = b.contents;
    }
    buf.pos += numbones * sizeof(fmt::mstudiobone_t);
    buf.Align4();

    // procedural bones. The reference writes them by kind in a
    // fixed order - axisinterp, quatinterp, jiggle - and each one back-patches
    // its bone's proctype/procindex, where procindex is a byte offset RELATIVE
    // to the owning mstudiobone_t. We produce quatinterp and jiggle.
    if (!m.proceduralbones.empty()) {
        fmt::mstudioquatinterpbone_t* pProc =
            reinterpret_cast<fmt::mstudioquatinterpbone_t*>(buf.p());

        // the whole bone array is claimed and aligned FIRST, then each bone's
        // triggers are appended after it - so the triggers of all bones follow
        // the array rather than interleaving with it
        buf.pos += m.proceduralbones.size() * sizeof(fmt::mstudioquatinterpbone_t);
        buf.Align4();

        for (size_t i = 0; i < m.proceduralbones.size(); i++) {
            const cm::ProceduralBone& pb = m.proceduralbones[i];
            int k = pb.helper;
            pbone[k].procindex = static_cast<int32_t>(
                reinterpret_cast<uint8_t*>(&pProc[i]) - reinterpret_cast<uint8_t*>(&pbone[k]));
            pbone[k].proctype = fmt::STUDIO_PROC_QUATINTERP;

            pProc[i].control = pb.driver;
            pProc[i].numtriggers = static_cast<int32_t>(pb.triggers.size());

            fmt::mstudioquatinterpinfo_t* pTrigger =
                reinterpret_cast<fmt::mstudioquatinterpinfo_t*>(buf.p());
            pProc[i].triggerindex = static_cast<int32_t>(
                reinterpret_cast<uint8_t*>(pTrigger) - reinterpret_cast<uint8_t*>(&pProc[i]));
            buf.pos += pb.triggers.size() * sizeof(fmt::mstudioquatinterpinfo_t);

            for (size_t t = 0; t < pb.triggers.size(); t++) {
                const cm::ProceduralBoneTrigger& tr = pb.triggers[t];
                // computed in DOUBLE, then narrowed, per the reference. The loader
                // guarantees tolerance > 0.
                pTrigger[t].inv_tolerance =
                    static_cast<float>(1.0 / static_cast<double>(tr.tolerance));
                CopyQ(pTrigger[t].trigger, tr.trigger);
                CopyV3(pTrigger[t].pos, tr.pos);
                CopyQ(pTrigger[t].quat, tr.quat);
            }
        }
        // no Align4 here: the reference leaves pData where the last trigger ended
        // mstudioquatinterpinfo_t is 48 bytes off a 4-aligned
        // base, so the next block is aligned regardless.
    }

    if (!m.jigglebones.empty()) {
        fmt::mstudiojigglebone_t* jiggle =
            reinterpret_cast<fmt::mstudiojigglebone_t*>(buf.p());

        for (size_t i = 0; i < m.jigglebones.size(); i++) {
            const cm::JiggleBone& jb = m.jigglebones[i];
            int k = jb.bone;
            pbone[k].procindex = static_cast<int32_t>(
                reinterpret_cast<uint8_t*>(&jiggle[i]) - reinterpret_cast<uint8_t*>(&pbone[k]));
            pbone[k].proctype = fmt::STUDIO_PROC_JIGGLE;

            fmt::mstudiojigglebone_t& d = jiggle[i];
            d.flags = jb.flags;
            d.length = jb.length;
            d.tipMass = jb.tipMass;
            d.yawStiffness = jb.yawStiffness;
            d.yawDamping = jb.yawDamping;
            d.pitchStiffness = jb.pitchStiffness;
            d.pitchDamping = jb.pitchDamping;
            d.alongStiffness = jb.alongStiffness;
            d.alongDamping = jb.alongDamping;
            d.angleLimit = jb.angleLimit;
            d.minYaw = jb.minYaw;
            d.maxYaw = jb.maxYaw;
            d.yawFriction = jb.yawFriction;
            d.yawBounce = jb.yawBounce;
            d.minPitch = jb.minPitch;
            d.maxPitch = jb.maxPitch;
            d.pitchFriction = jb.pitchFriction;
            d.pitchBounce = jb.pitchBounce;
            d.baseMass = jb.baseMass;
            d.baseStiffness = jb.baseStiffness;
            d.baseDamping = jb.baseDamping;
            d.baseMinLeft = jb.baseMinLeft;
            d.baseMaxLeft = jb.baseMaxLeft;
            d.baseLeftFriction = jb.baseLeftFriction;
            d.baseMinUp = jb.baseMinUp;
            d.baseMaxUp = jb.baseMaxUp;
            d.baseUpFriction = jb.baseUpFriction;
            d.baseMinForward = jb.baseMinForward;
            d.baseMaxForward = jb.baseMaxForward;
            d.baseForwardFriction = jb.baseForwardFriction;
            d.boingImpactSpeed = jb.boingImpactSpeed;
            d.boingImpactAngle = jb.boingImpactAngle;
            d.boingDampingRate = jb.boingDampingRate;
            d.boingFrequency = jb.boingFrequency;
            d.boingAmplitude = jb.boingAmplitude;
        }
        buf.pos += m.jigglebones.size() * sizeof(fmt::mstudiojigglebone_t);
        buf.Align4();
    }

    // aim-at bones come last of the procedural kinds
    if (!m.aimatbones.empty()) {
        fmt::mstudioaimatbone_t* pProc =
            reinterpret_cast<fmt::mstudioaimatbone_t*>(buf.p());

        for (size_t i = 0; i < m.aimatbones.size(); i++) {
            const cm::AimAtBone& ab = m.aimatbones[i];
            int k = ab.bone;
            pbone[k].procindex = static_cast<int32_t>(
                reinterpret_cast<uint8_t*>(&pProc[i]) - reinterpret_cast<uint8_t*>(&pbone[k]));
            // an attachment target switches both the proctype and what `aim`
            // indexes into
            pbone[k].proctype = ab.aimAttach == -1 ? fmt::STUDIO_PROC_AIMATBONE
                                                   : fmt::STUDIO_PROC_AIMATATTACH;
            pProc[i].parent = ab.parent;
            pProc[i].aim = ab.aimAttach == -1 ? ab.aimBone : ab.aimAttach;
            CopyV3(pProc[i].aimvector, ab.aimvector);
            CopyV3(pProc[i].upvector, ab.upvector);
            CopyV3(pProc[i].basepos, ab.basepos);
        }
        buf.pos += m.aimatbones.size() * sizeof(fmt::mstudioaimatbone_t);
        buf.Align4();
    }

    // bone controllers: -1 everywhere, zero controllers
    for (int i = 0; i < numbones; i++)
        for (int j = 0; j < 6; j++)
            pbone[i].bonecontroller[j] = -1;

    phdr->numbonecontrollers = 0;
    phdr->bonecontrollerindex = static_cast<int32_t>(buf.pos);
    buf.Align4();

    // attachments (`type` is compile-only, never written)
    fmt::mstudioattachment_t* pattachment =
        reinterpret_cast<fmt::mstudioattachment_t*>(buf.p());
    phdr->numlocalattachments = static_cast<int32_t>(m.attachments.size());
    phdr->localattachmentindex = static_cast<int32_t>(buf.pos);
    for (size_t i = 0; i < m.attachments.size(); i++) {
        const cm::Attachment& att = m.attachments[i];
        pattachment[i].localbone = att.bone;
        g_strings.Add(buf, &pattachment[i], &pattachment[i].sznameindex, att.name.c_str());
        CopyM(pattachment[i].local, att.local);
        pattachment[i].flags = att.flags;
    }
    buf.pos += m.attachments.size() * sizeof(fmt::mstudioattachment_t);
    buf.Align4();

    // hitbox sets
    phdr->numhitboxsets = static_cast<int32_t>(m.hitboxsets.size());
    fmt::mstudiohitboxset_t* hitboxset = reinterpret_cast<fmt::mstudiohitboxset_t*>(buf.p());
    phdr->hitboxsetindex = static_cast<int32_t>(buf.pos);
    buf.pos += m.hitboxsets.size() * sizeof(fmt::mstudiohitboxset_t);
    buf.Align4();

    for (size_t s = 0; s < m.hitboxsets.size(); s++, hitboxset++) {
        cm::HitboxSet& set = m.hitboxsets[s];
        g_strings.Add(buf, hitboxset, &hitboxset->sznameindex, set.name.c_str());
        hitboxset->numhitboxes = static_cast<int32_t>(set.hitboxes.size());
        hitboxset->hitboxindex =
            static_cast<int32_t>(buf.pos - (reinterpret_cast<uint8_t*>(hitboxset) - buf.start()));

        fmt::mstudiobbox_t* pbbox = reinterpret_cast<fmt::mstudiobbox_t*>(buf.p());
        for (size_t i = 0; i < set.hitboxes.size(); i++) {
            pbbox[i].bone = set.hitboxes[i].bone;
            pbbox[i].group = set.hitboxes[i].group;
            CopyV3(pbbox[i].bbmin, set.hitboxes[i].bmin);
            CopyV3(pbbox[i].bbmax, set.hitboxes[i].bmax);
            CopyV3(pbbox[i].angOffsetOrientation, set.hitboxes[i].angOffset);
            pbbox[i].flCapsuleRadius = set.hitboxes[i].capsuleRadius;
            pbbox[i].szhitboxnameindex = 0;
            g_strings.Add(buf, &pbbox[i], &pbbox[i].szhitboxnameindex,
                          set.hitboxes[i].name.c_str());
        }
        buf.pos += set.hitboxes.size() * sizeof(fmt::mstudiobbox_t);
        buf.Align4();
    }

    // bone table by name (qsort)
    uint8_t* pBoneTable = buf.p();
    phdr->bonetablebynameindex = static_cast<int32_t>(buf.pos);
    for (int i = 0; i < numbones; i++)
        pBoneTable[i] = static_cast<uint8_t>(i);
    g_m = &m;
    if (numbones > 0)
        qsort(pBoneTable, numbones, sizeof(uint8_t), BoneNameCompare);
    g_m = nullptr;
    buf.pos += numbones * sizeof(uint8_t);
    buf.Align4();
}

// ---------------------------------------------------------------------------
// WriteRLEAnimationData
// ---------------------------------------------------------------------------
void WriteRLEAnimationData(cm::CompiledModel& m, cm::Anim& srcanim, Buf& buf, int w) {
    int numbones = static_cast<int>(m.bones.size());

    fmt::mstudio_rle_anim_t* destanim = buf.Reserve<fmt::mstudio_rle_anim_t>();
    destanim->bone = 255;

    fmt::mstudio_rle_anim_t* prevanim = nullptr;

    for (int j = 0; j < numbones; j++) {
        destanim->flags = 0;
        cm::AnimChannels& psrcdata = srcanim.anim[w][j];

        if (psrcdata.num[0] + psrcdata.num[1] + psrcdata.num[2] + psrcdata.num[3] +
                psrcdata.num[4] + psrcdata.num[5] == 0) {
            continue; // no animation, skip
        }

        destanim->bone = static_cast<uint8_t>(j);

        if (srcanim.flags & fmt::STUDIO_DELTA)
            destanim->flags |= fmt::STUDIO_ANIM_DELTA;

        if ((srcanim.numframes == 1) ||
            (psrcdata.num[0] <= 2 && psrcdata.num[1] <= 2 && psrcdata.num[2] <= 2 &&
             psrcdata.num[3] <= 2 && psrcdata.num[4] <= 2 && psrcdata.num[5] <= 2)) {
            // constant over the section: raw values
            int iFrame = w * srcanim.sectionframes;
            if (iFrame > srcanim.numframes - 1)
                iFrame = srcanim.numframes - 1;
            if (psrcdata.num[3] != 0 || psrcdata.num[4] != 0 || psrcdata.num[5] != 0) {
                pm::Quaternion q;
                pm::AngleQuaternion(srcanim.sanim[iFrame][j].rot, q);
                pm::Quaternion64 q64;
                q64.Set(q);
                memcpy(buf.p(), &q64, sizeof(q64));
                buf.pos += sizeof(q64);
                destanim->flags |= fmt::STUDIO_ANIM_RAWROT2;
            }
            if (psrcdata.num[0] != 0 || psrcdata.num[1] != 0 || psrcdata.num[2] != 0) {
                pm::Vector48 v48;
                v48.Set(srcanim.sanim[iFrame][j].pos);
                memcpy(buf.p(), &v48, sizeof(v48));
                buf.pos += sizeof(v48);
                destanim->flags |= fmt::STUDIO_ANIM_RAWPOS;
            }
        } else {
            // valueptr blocks: rot valueptr always first
            fmt::mstudioanim_valueptr_t* rotvptr = buf.Reserve<fmt::mstudioanim_valueptr_t>();
            fmt::mstudioanim_valueptr_t* posvptr = nullptr;
            if (psrcdata.num[0] != 0 || psrcdata.num[1] != 0 || psrcdata.num[2] != 0)
                posvptr = buf.Reserve<fmt::mstudioanim_valueptr_t>();

            uint16_t* destanimvalue = reinterpret_cast<uint16_t*>(buf.p());
            uint16_t* cursor = destanimvalue;

            // rotation
            for (int k = 3; k < 6; k++) {
                if (psrcdata.num[k] == 0) {
                    rotvptr->offset[k - 3] = 0;
                } else {
                    rotvptr->offset[k - 3] = static_cast<int16_t>(
                        reinterpret_cast<uint8_t*>(cursor) - reinterpret_cast<uint8_t*>(rotvptr));
                    for (int n = 0; n < psrcdata.num[k]; n++)
                        *cursor++ = psrcdata.data[k][n];
                }
            }
            destanim->flags |= fmt::STUDIO_ANIM_ANIMROT;

            if (posvptr) {
                for (int k = 0; k < 3; k++) {
                    if (psrcdata.num[k] == 0) {
                        posvptr->offset[k] = 0;
                    } else {
                        posvptr->offset[k] = static_cast<int16_t>(
                            reinterpret_cast<uint8_t*>(cursor) -
                            reinterpret_cast<uint8_t*>(posvptr));
                        for (int n = 0; n < psrcdata.num[k]; n++)
                            *cursor++ = psrcdata.data[k][n];
                    }
                }
                destanim->flags |= fmt::STUDIO_ANIM_ANIMPOS;
            }
            buf.pos += (cursor - destanimvalue) * sizeof(uint16_t);
        }

        prevanim = destanim;
        destanim->nextoffset =
            static_cast<int16_t>(buf.p() - reinterpret_cast<uint8_t*>(destanim));
        destanim = buf.Reserve<fmt::mstudio_rle_anim_t>();
    }

    if (prevanim)
        prevanim->nextoffset = 0;

    buf.Align4();
}

// ---------------------------------------------------------------------------
// WriteFrameAnimationData - the frame-major encoding used by
// demand-loaded animation. Instead of one RLE stream per bone, a section is a
// per-bone flag byte array + a block of values that never change + a fixed
// framelength stride repeated per frame, so the engine can seek to a frame
// without decoding the ones before it.
// ---------------------------------------------------------------------------
void WriteFrameAnimationData(cm::CompiledModel& m, cm::Anim& srcanim, Buf& buf, int w) {
    const int numbones = static_cast<int>(m.bones.size());

    fmt::mstudio_frame_anim_t* destframeanim = buf.Reserve<fmt::mstudio_frame_anim_t>();

    uint8_t* flag = buf.p();
    buf.pos += numbones * sizeof(uint8_t);
    buf.Align4();

    destframeanim->constantsoffset =
        static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(destframeanim));
    int framelength = 0;
    int iFrame = w * srcanim.sectionframes;
    if (iFrame > srcanim.numframes - 1)
        iFrame = srcanim.numframes - 1;

    for (int j = 0; j < numbones; j++) {
        const cm::AnimChannels& psrcdata = srcanim.anim[w][j];

        if (psrcdata.num[3] == 0 && psrcdata.num[4] == 0 && psrcdata.num[5] == 0) {
            // no rotation on this bone in this section
        } else if (psrcdata.num[3] <= 2 && psrcdata.num[4] <= 2 && psrcdata.num[5] <= 2) {
            flag[j] |= fmt::STUDIO_FRAME_CONST_ROT2;
            pm::Quaternion q;
            pm::AngleQuaternion(srcanim.sanim[iFrame][j].rot, q);
            pm::Quaternion48S q48s;
            q48s.Set(q);
            memcpy(buf.p(), &q48s, sizeof(q48s));
            buf.pos += sizeof(q48s);
        } else {
            flag[j] |= fmt::STUDIO_FRAME_ANIM_ROT2;
            framelength += static_cast<int>(sizeof(pm::Quaternion48S));
        }

        if (psrcdata.num[0] == 0 && psrcdata.num[1] == 0 && psrcdata.num[2] == 0) {
            // no translation on this bone in this section
        } else if (psrcdata.num[0] <= 2 && psrcdata.num[1] <= 2 && psrcdata.num[2] <= 2) {
            if (m.animblockHighRes) {
                flag[j] |= fmt::STUDIO_FRAME_CONST_POS2;
                CopyV3(*reinterpret_cast<fmt::Vector3*>(buf.p()), srcanim.sanim[iFrame][j].pos);
                buf.pos += sizeof(fmt::Vector3);
            } else {
                flag[j] |= fmt::STUDIO_FRAME_CONST_POS;
                pm::Vector48 v48;
                v48.Set(srcanim.sanim[iFrame][j].pos);
                memcpy(buf.p(), &v48, sizeof(v48));
                buf.pos += sizeof(v48);
            }
        } else {
            if (m.animblockHighRes) {
                flag[j] |= fmt::STUDIO_FRAME_ANIM_POS2;
                framelength += static_cast<int>(sizeof(fmt::Vector3));
            } else {
                flag[j] |= fmt::STUDIO_FRAME_ANIM_POS;
                framelength += static_cast<int>(sizeof(pm::Vector48));
            }
        }
    }

    buf.Align4();

    destframeanim->frameoffset =
        static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(destframeanim));
    destframeanim->framelength = framelength;

    // sections overlap by one frame: the last frame of section w is also the
    // first of section w+1, so a blend across the seam has both ends
    int iStartFrame = 0;
    int iEndFrame = srcanim.numframes - 1;
    if (srcanim.sectionframes > 0) {
        iStartFrame = std::min(w * srcanim.sectionframes, srcanim.numframes - 1);
        iEndFrame = std::min((w + 1) * srcanim.sectionframes, srcanim.numframes - 1);
    }

    for (iFrame = iStartFrame; iFrame <= iEndFrame; iFrame++) {
        for (int j = 0; j < numbones; j++) {
            if (flag[j] & fmt::STUDIO_FRAME_ANIM_ROT2) {
                pm::Quaternion q;
                pm::AngleQuaternion(srcanim.sanim[iFrame][j].rot, q);
                pm::Quaternion48S q48s;
                q48s.Set(q);
                memcpy(buf.p(), &q48s, sizeof(q48s));
                buf.pos += sizeof(q48s);
            }

            if (flag[j] & fmt::STUDIO_FRAME_ANIM_POS) {
                pm::Vector48 v48;
                v48.Set(srcanim.sanim[iFrame][j].pos);
                memcpy(buf.p(), &v48, sizeof(v48));
                buf.pos += sizeof(v48);
            } else if (flag[j] & fmt::STUDIO_FRAME_ANIM_POS2) {
                CopyV3(*reinterpret_cast<fmt::Vector3*>(buf.p()), srcanim.sanim[iFrame][j].pos);
                buf.pos += sizeof(fmt::Vector3);
            }
        }
    }

    buf.Align4();
}

// ---------------------------------------------------------------------------
// WriteIkErrors: rule array + ALIGN4, then per rule the
// compressed error header + streams + raw attachment string. Every offset is
// rule-relative, so this works in either the .mdl or the .ani buffer.
// ---------------------------------------------------------------------------
void WriteIkErrors(cm::Anim& srcanim, Buf& buf) {
    if (srcanim.ikrules.empty())
        return;

    fmt::mstudioikrule_t* pikruledata = reinterpret_cast<fmt::mstudioikrule_t*>(buf.p());
    buf.pos += srcanim.ikrules.size() * sizeof(fmt::mstudioikrule_t);
    buf.Align4();

    for (size_t j = 0; j < srcanim.ikrules.size(); j++) {
        const cm::IkRule& rule = srcanim.ikrules[j];
        fmt::mstudioikrule_t* pikrule = &pikruledata[j];

        pikrule->index = rule.index;
        pikrule->chain = rule.chain;
        pikrule->bone = rule.bone;
        pikrule->type = rule.type;
        pikrule->slot = rule.slot;
        CopyV3(pikrule->pos, rule.pos);
        CopyQ(pikrule->q, rule.q);
        pikrule->height = rule.height;
        pikrule->floor = rule.floor;
        pikrule->radius = rule.radius;

        if (srcanim.numframes > 1) {
            pikrule->start = rule.start / (srcanim.numframes - 1.0f);
            pikrule->peak = rule.peak / (srcanim.numframes - 1.0f);
            pikrule->tail = rule.tail / (srcanim.numframes - 1.0f);
            pikrule->end = rule.end / (srcanim.numframes - 1.0f);
            pikrule->contact = rule.contact / (srcanim.numframes - 1.0f);
        } else {
            pikrule->start = 0.0f;
            pikrule->peak = 0.0f;
            pikrule->tail = 1.0f;
            pikrule->end = 1.0f;
            pikrule->contact = 0.0f;
        }
        pikrule->iStart = rule.start;

        // skip the compressed header if there's no IK data
        int k = 0;
        for (; k < 6; k++)
            if (rule.errorData.numanim[k])
                break;
        if (k == 6)
            continue;

        pikrule->compressedikerrorindex =
            static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(pikrule));
        fmt::mstudiocompressedikerror_t* pCompressed =
            reinterpret_cast<fmt::mstudiocompressedikerror_t*>(buf.p());
        buf.pos += sizeof(fmt::mstudiocompressedikerror_t);

        for (k = 0; k < 6; k++) {
            pCompressed->scale[k] = rule.errorData.scale[k];
            pCompressed->offset[k] =
                static_cast<int16_t>(buf.p() - reinterpret_cast<uint8_t*>(pCompressed));
            size_t size = rule.errorData.numanim[k] * sizeof(uint16_t);
            if (size)
                memcpy(buf.p(), rule.errorData.data[k].data(), size);
            buf.pos += size;
        }

        if (!rule.attachment.empty()) {
            // raw string, NOT the string table
            size_t size = rule.attachment.size() + 1;
            memcpy(buf.p(), rule.attachment.c_str(), size);
            pikrule->szattachmentindex =
                static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(pikrule));
            buf.pos += size;
        }

        buf.Align4();
    }
}

// ---------------------------------------------------------------------------
// WriteAnimationData: route each section of one animation to
// either the .mdl (local) or the .ani (demand loaded), and record where it
// landed in the section table. `ext` is null when there is no .ani at all.
// ---------------------------------------------------------------------------
void WriteAnimationData(cm::CompiledModel& m, cm::Anim& srcanim,
                        fmt::mstudioanimdesc_t* destanimdesc, size_t destanimOff, Buf& local,
                        Buf* ext, fmt::mstudioanimsections_t* pSections) {
    const int animblocksize = m.animblocksize;

    for (int w = 0; w < srcanim.numsections; w++) {
        bool bUseExtData = false;
        Buf* pBuf = &local;

        if (ext && !srcanim.disableAnimblocks &&
            !((w * srcanim.sectionframes < srcanim.numNostallFrames) &&
              srcanim.isFirstSectionLocal)) {
            pBuf = ext;
            bUseExtData = true;
        }

        const size_t startSection = pBuf->pos;

        // a demand-loaded model uses the frame-major encoding throughout -
        // including the clips that stayed local
        if (ext && !m.animblockLowRes) {
            srcanim.flags |= fmt::STUDIO_FRAMEANIM;
            destanimdesc->flags |= fmt::STUDIO_FRAMEANIM;
        }

        if (srcanim.flags & fmt::STUDIO_FRAMEANIM)
            WriteFrameAnimationData(m, srcanim, *pBuf, w);
        else
            WriteRLEAnimationData(m, srcanim, *pBuf, w);

        const size_t sectionBytes = pBuf->pos - startSection;
        if (animblocksize > 0 && sectionBytes > static_cast<size_t>(animblocksize))
            std::printf("WARNING: animation \"%s\" section %d is %zu bytes, over the %d-byte "
                        "block size - use shorter animations or a larger $animblocksize\n",
                        srcanim.name.c_str(), w, sectionBytes, animblocksize);

        if (pSections) {
            if (bUseExtData) {
                if (g_animblocks.count &&
                    pBuf->pos - g_animblocks.blocks[g_animblocks.count - 1].start >
                        static_cast<size_t>(animblocksize))
                    g_animblocks.Advance(startSection);

                pSections[w].animblock = static_cast<int32_t>(g_animblocks.count - 1);
                pSections[w].animindex = static_cast<int32_t>(
                    startSection - g_animblocks.blocks[g_animblocks.count - 1].start);
            } else {
                pSections[w].animblock = 0;
                pSections[w].animindex = static_cast<int32_t>(startSection - destanimOff);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// WriteAnimations
// ---------------------------------------------------------------------------
void WriteAnimations(Buf& buf, fmt::studiohdr_t* phdr, cm::CompiledModel& m, Buf* blockBuf) {
    fmt::mstudioanimdesc_t* panimdesc = reinterpret_cast<fmt::mstudioanimdesc_t*>(buf.p());
    phdr->numlocalanim = static_cast<int32_t>(m.anims.size());
    phdr->localanimindex = static_cast<int32_t>(buf.pos);
    buf.pos += m.anims.size() * sizeof(fmt::mstudioanimdesc_t);
    buf.Align4();

    // the .ani write head. blockBuf->pos is rewound to it per animation and
    // runs ahead as that animation's data is written (reference pBlockData vs
    // pBlockEnd); it only catches up once the animation is fully placed.
    size_t blockData = blockBuf ? blockBuf->pos : 0;

    for (size_t i = 0; i < m.anims.size(); i++) {
        cm::Anim& srcanim = m.anims[i];
        fmt::mstudioanimdesc_t* destanim = &panimdesc[i];

        g_strings.Add(buf, destanim, &destanim->sznameindex, srcanim.name.c_str());

        destanim->baseptr =
            static_cast<int32_t>(buf.start() - reinterpret_cast<uint8_t*>(destanim));
        destanim->fps = srcanim.fps;
        destanim->flags = srcanim.flags;
        destanim->sectionframes = srcanim.sectionframes;
        destanim->numframes = srcanim.numframes;

        fmt::mstudioanimsections_t* pSections = nullptr;
        if (srcanim.numsections > 1) {
            destanim->sectionindex =
                static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(destanim));
            pSections = reinterpret_cast<fmt::mstudioanimsections_t*>(buf.p());
            buf.pos += srcanim.numsections * sizeof(fmt::mstudioanimsections_t);
        }

        // align all animation data to cache line boundaries
        buf.Align16();
        if (blockBuf)
            blockData = (blockData + 15) & ~size_t(15);

        // allocate the first real block on the first animation
        if (blockBuf && g_animblocks.count == 0)
            g_animblocks.Open(blockData);

        // block zero is relative to this animdesc
        const size_t destanimOff = reinterpret_cast<uint8_t*>(destanim) - buf.start();
        g_animblocks.blocks[0].start = destanimOff;

        // pBlockEnd: the .ani cursor for THIS animation, rewound to the block
        // write head each time round
        if (blockBuf)
            blockBuf->pos = blockData;

        size_t pAnimData = 0, pIkData = 0;

        // !blockBuf is redundant with disableAnimblocks (ResolveAnimBlockPolicy
        // sets it for every clip when there is no .ani) - stated so the else
        // branch's blockBuf dereferences are locally provable
        if (!blockBuf || srcanim.disableAnimblocks || srcanim.isFirstSectionLocal) {
            destanim->animblock = 0;
            pAnimData = buf.pos;
            WriteAnimationData(m, srcanim, destanim, destanimOff, buf, blockBuf, pSections);
            pIkData = buf.pos;
            WriteIkErrors(srcanim, buf);
        } else {
            pAnimData = blockBuf->pos;
            WriteAnimationData(m, srcanim, destanim, destanimOff, buf, blockBuf, pSections);
            // sections already advanced the block table past what they wrote,
            // so don't let the tail check move it again
            if (destanim->sectionindex)
                blockData = blockBuf->pos;
            destanim->animblock = static_cast<int32_t>(g_animblocks.count - 1);
            pIkData = blockBuf->pos;
            WriteIkErrors(srcanim, *blockBuf);
        }
        // TODO: localhierarchy. A $sequence/$animation option that reparents a
        // bone to another for a frame range (start/peak/tail/end ramp, pose
        // compressed like IK error). Unparsed, so numlocalhierarchy stays 0.

        if (blockBuf && blockData != blockBuf->pos &&
            blockBuf->pos - g_animblocks.blocks[g_animblocks.count - 1].start >
                static_cast<size_t>(m.animblocksize)) {
            g_animblocks.Advance(blockData);
            destanim->animblock = static_cast<int32_t>(g_animblocks.count - 1);
        }

        destanim->animindex = static_cast<int32_t>(
            pAnimData - g_animblocks.blocks[destanim->animblock].start);

        if (!srcanim.ikrules.empty()) {
            destanim->numikrules = static_cast<int32_t>(srcanim.ikrules.size());
            // block 0 is animdesc-relative, a real block is file-relative, and
            // they live in different header fields
            const int32_t off = static_cast<int32_t>(
                pIkData - g_animblocks.blocks[destanim->animblock].start);
            if (destanim->animblock == 0)
                destanim->ikruleindex = off;
            else
                destanim->animblockikruleindex = off;
        }

        if (g_animblocks.count) {
            g_animblocks.blocks[g_animblocks.count - 1].end = blockBuf->pos;
            blockData = blockBuf->pos;
        }
    }

    // movement keys: one mstudiomovement_t per extracted
    // motion segment. angle is the Z rotation in DEGREES.
    for (size_t i = 0; i < m.anims.size(); i++) {
        const cm::Anim& anim = m.anims[i];
        fmt::mstudioanimdesc_t* pdesc = &panimdesc[i];
        pdesc->nummovements = static_cast<int32_t>(anim.piecewisemove.size());
        if (!pdesc->nummovements)
            continue;
        pdesc->movementindex =
            static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(pdesc));
        fmt::mstudiomovement_t* pmove =
            reinterpret_cast<fmt::mstudiomovement_t*>(buf.p());
        buf.pos += anim.piecewisemove.size() * sizeof(*pmove);
        buf.Align4();
        for (size_t j = 0; j < anim.piecewisemove.size(); j++) {
            const cm::LinearMove& mv = anim.piecewisemove[j];
            pmove[j].endframe = mv.endframe;
            pmove[j].motionflags = mv.flags;
            pmove[j].v0 = mv.v0;
            pmove[j].v1 = mv.v1;
            pmove[j].angle = mv.rot.z * (180.0f / pm::kPiF);
            CopyV3(pmove[j].vector, mv.vector);
            CopyV3(pmove[j].position, mv.pos);
        }
    }

    // zero frames only exist for demand-loaded animation:
    // they stand in for a block that has not arrived yet, so with everything
    // local there is nothing to stand in for.
    if (!blockBuf)
        return;

    // which bones get cached. With no $bonesaveframe the writer picks: roots
    // (whose position IS the model's placement) plus any bone that travels far
    // enough for a frozen position to read as a bug. Rotation is always kept.
    const int numbones = static_cast<int>(m.bones.size());
    if (m.boneSaveFrames.empty()) {
        for (int j = 0; j < numbones; j++) {
            const pm::Vector3& r = m.bones[j].posrange;
            if (m.bones[j].parent == -1 ||
                std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z) >= m.minZeroFramePosDelta)
                m.bones[j].flags |= fmt::BONE_HAS_SAVEFRAME_POS;
            m.bones[j].flags |= m.zeroFramesHighres ? fmt::BONE_HAS_SAVEFRAME_ROT64
                                                    : fmt::BONE_HAS_SAVEFRAME_ROT32;
        }
    } else {
        for (const cm::BoneSaveFrame& bsf : m.boneSaveFrames) {
            int j = -1;
            for (int b = 0; b < numbones; b++)
                if (_stricmp(m.bones[b].name.c_str(), bsf.name.c_str()) == 0) {
                    j = b;
                    break;
                }
            if (j == -1) {
                // the bone was collapsed or never existed - the reference errors
                // out here; a warning keeps the model compiling
                std::printf("WARNING: unknown $bonesaveframe \"%s\"\n", bsf.name.c_str());
                continue;
            }
            if (bsf.savePos)
                m.bones[j].flags |= fmt::BONE_HAS_SAVEFRAME_POS;
            if (bsf.saveRot)
                m.bones[j].flags |= m.zeroFramesHighres ? fmt::BONE_HAS_SAVEFRAME_ROT64
                                                        : fmt::BONE_HAS_SAVEFRAME_ROT32;
            else if (bsf.saveRot64)
                m.bones[j].flags |= fmt::BONE_HAS_SAVEFRAME_ROT64;
        }
    }

    // patch the already-written bone table
    fmt::mstudiobone_t* pbones =
        reinterpret_cast<fmt::mstudiobone_t*>(buf.start() + phdr->boneindex);
    for (int j = 0; j < numbones; j++)
        pbones[j].flags |= m.bones[j].flags;

    buf.Align4();

    for (size_t i = 0; i < m.anims.size(); i++) {
        cm::Anim& anim = m.anims[i];
        fmt::mstudioanimdesc_t* destanim = &panimdesc[i];
        if (destanim->animblock == 0)
            continue;

        destanim->zeroframeindex =
            static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(destanim));

        // sample up to maxZeroFrames poses, spread `zeroframespan` frames
        // apart, over at most the first 27 frames of the clip
        int k = std::min(destanim->numframes - 1, 9);
        if (destanim->flags & fmt::STUDIO_LOOPING)
            k = std::min((destanim->numframes - 1) / 2, k);
        destanim->zeroframespan = static_cast<int16_t>(k);
        if (k > 2)
            destanim->zeroframecount =
                static_cast<int16_t>(std::min((destanim->numframes - 1) / k, 3));
        if (destanim->zeroframecount < 1)
            destanim->zeroframecount = 1;
        destanim->zeroframecount =
            std::min<int16_t>(destanim->zeroframecount, static_cast<int16_t>(m.maxZeroFrames));

        for (int j = 0; j < numbones; j++) {
            const int32_t bflags = m.bones[j].flags;
            if (bflags & fmt::BONE_HAS_SAVEFRAME_POS) {
                for (int n = 0; n < destanim->zeroframecount; n++) {
                    pm::Vector48 v48;
                    v48.Set(anim.sanim[destanim->zeroframespan * n][j].pos);
                    memcpy(buf.p(), &v48, sizeof(v48));
                    buf.pos += sizeof(v48);
                }
            }
            if (bflags & fmt::BONE_HAS_SAVEFRAME_ROT64) {
                for (int n = 0; n < destanim->zeroframecount; n++) {
                    pm::Quaternion q;
                    pm::AngleQuaternion(anim.sanim[destanim->zeroframespan * n][j].rot, q);
                    pm::Quaternion64 q64;
                    q64.Set(q);
                    memcpy(buf.p(), &q64, sizeof(q64));
                    buf.pos += sizeof(q64);
                }
            } else if (bflags & fmt::BONE_HAS_SAVEFRAME_ROT32) {
                for (int n = 0; n < destanim->zeroframecount; n++) {
                    pm::Quaternion q;
                    pm::AngleQuaternion(anim.sanim[destanim->zeroframespan * n][j].rot, q);
                    pm::Quaternion32 q32;
                    q32.Set(q);
                    memcpy(buf.p(), &q32, sizeof(q32));
                    buf.pos += sizeof(q32);
                }
            }
        }
        buf.Align4();

        // the ik rule timings, re-read from wherever the full rules landed -
        // the .mdl for a local animation, the .ani buffer otherwise
        if (destanim->numikrules) {
            fmt::mstudioikrulezeroframe_t* pdestikrule =
                reinterpret_cast<fmt::mstudioikrulezeroframe_t*>(buf.p());
            destanim->ikrulezeroframeindex =
                static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(destanim));
            buf.pos += sizeof(*pdestikrule) * destanim->numikrules;

            const fmt::mstudioikrule_t* psrcikrule =
                destanim->ikruleindex
                    ? reinterpret_cast<const fmt::mstudioikrule_t*>(
                          reinterpret_cast<uint8_t*>(destanim) + destanim->ikruleindex)
                    : reinterpret_cast<const fmt::mstudioikrule_t*>(
                          blockBuf->start() + g_animblocks.blocks[destanim->animblock].start +
                          destanim->animblockikruleindex);

            for (int j = 0; j < destanim->numikrules; j++, psrcikrule++, pdestikrule++) {
                pdestikrule->slot = static_cast<int16_t>(psrcikrule->slot);
                pdestikrule->chain = static_cast<int16_t>(psrcikrule->chain);
                pdestikrule->start.SetFloat(psrcikrule->start);
                pdestikrule->peak.SetFloat(psrcikrule->peak);
                pdestikrule->tail.SetFloat(psrcikrule->tail);
                pdestikrule->end.SetFloat(psrcikrule->end);
            }
        }
        buf.Align4();
    }

    buf.Align4();
}

// ---------------------------------------------------------------------------
// WriteSequenceInfo
// ---------------------------------------------------------------------------
void WriteSequenceInfo(Buf& buf, fmt::studiohdr_t* phdr, cm::CompiledModel& m) {
    int numbones = static_cast<int>(m.bones.size());

    phdr->activitylistversion = 0;
    phdr->eventsindexed = 0;

    fmt::mstudioseqdesc_t* pseqdesc = reinterpret_cast<fmt::mstudioseqdesc_t*>(buf.p());
    phdr->numlocalseq = static_cast<int32_t>(m.sequences.size());
    phdr->localseqindex = static_cast<int32_t>(buf.pos);
    buf.pos += m.sequences.size() * sizeof(fmt::mstudioseqdesc_t);

    size_t prevWeightOffset = 0; // offset of the newest written weight block
    bool havePrevWeights = false;
    std::vector<std::vector<float>> writtenWeights; // per written block
    std::vector<size_t> writtenWeightOffsets;

    for (size_t i = 0; i < m.sequences.size(); i++, pseqdesc++) {
        cm::Sequence& seq = m.sequences[i];
        uint8_t* pSequenceStart = reinterpret_cast<uint8_t*>(pseqdesc);

        g_strings.Add(buf, pseqdesc, &pseqdesc->szlabelindex, seq.name.c_str());
        g_strings.Add(buf, pseqdesc, &pseqdesc->szactivitynameindex, seq.activityname.c_str());

        pseqdesc->baseptr = static_cast<int32_t>(buf.start() - pSequenceStart);
        pseqdesc->flags = seq.flags;
        pseqdesc->numblends = seq.numblends;
        pseqdesc->groupsize[0] = seq.groupsize[0];
        pseqdesc->groupsize[1] = seq.groupsize[1];
        pseqdesc->paramindex[0] = seq.paramindex[0];
        pseqdesc->paramstart[0] = seq.paramstart[0];
        pseqdesc->paramend[0] = seq.paramend[0];
        pseqdesc->paramindex[1] = seq.paramindex[1];
        pseqdesc->paramstart[1] = seq.paramstart[1];
        pseqdesc->paramend[1] = seq.paramend[1];

        // posekeys: only when the grid has a blend axis;
        // groupsize[0] floats from param0 then groupsize[1] from param1
        if (seq.groupsize[0] > 1 || seq.groupsize[1] > 1) {
            pseqdesc->posekeyindex = static_cast<int32_t>(buf.p() - pSequenceStart);
            float* pk = reinterpret_cast<float*>(buf.p());
            buf.pos += (seq.groupsize[0] + seq.groupsize[1]) * sizeof(float);
            int nk = 0;
            for (int j = 0; j < seq.groupsize[0]; j++, nk++)
                pk[nk] = (j < static_cast<int>(seq.param0.size())) ? seq.param0[j] : 0.0f;
            for (int j = 0; j < seq.groupsize[1]; j++, nk++)
                pk[nk] = (j < static_cast<int>(seq.param1.size())) ? seq.param1[j] : 0.0f;
        }

        pseqdesc->activity = seq.activity;
        pseqdesc->actweight = seq.actweight;
        CopyV3(pseqdesc->bbmin, seq.bmin);
        CopyV3(pseqdesc->bbmax, seq.bmax);
        pseqdesc->fadeintime = seq.fadeintime;
        pseqdesc->fadeouttime = seq.fadeouttime;
        // entryphase/exitphase are deliberately NOT written: the reference's
        // assignments are commented out, so stock always leaves
        // both at 0 even when the script sets `exitphase`. Writing ours would
        // break byte parity for no gain - the engine never sees a value here.
        pseqdesc->localentrynode = seq.entrynode;
        pseqdesc->localexitnode = seq.exitnode;
        pseqdesc->nodeflags = seq.nodeflags;

        // events. The frame is stored as a cycle fraction of
        // blend anim 0's last frame; the compile stage already rejected a frame
        // past it. A numeric name is an old-style id, anything else is written
        // by name with NEW_EVENT_STYLE.
        pseqdesc->numevents = static_cast<int32_t>(seq.events.size());
        pseqdesc->eventindex = static_cast<int32_t>(buf.p() - pSequenceStart);
        {
            fmt::mstudioevent_t* pevent =
                reinterpret_cast<fmt::mstudioevent_t*>(buf.p());
            buf.pos += seq.events.size() * sizeof(fmt::mstudioevent_t);
            int lastframe = 0;
            if (!seq.animIndices.empty() && seq.animIndices[0] >= 0)
                lastframe = m.anims[seq.animIndices[0]].numframes - 1;
            for (size_t j = 0; j < seq.events.size(); j++) {
                const cm::SeqEvent& ev = seq.events[j];
                pevent[j].cycle =
                    lastframe > 0 ? ev.frame / static_cast<float>(lastframe) : 0.0f;
                if (isdigit(static_cast<unsigned char>(ev.name[0]))) {
                    pevent[j].event = atoi(ev.name.c_str());
                    pevent[j].type = 0;
                    pevent[j].szeventindex = 0;
                } else {
                    g_strings.Add(buf, &pevent[j], &pevent[j].szeventindex,
                                  ev.name.c_str());
                    pevent[j].type = fmt::NEW_EVENT_STYLE;
                }
                // options is a fixed char[64]; the loader capped the length
                memcpy(pevent[j].options, ev.options.c_str(), ev.options.size() + 1);
            }
        }
        buf.Align4();

        pseqdesc->numikrules = seq.numikrules;

        // autolayers: frame values convert to cycle fractions
        // of THIS sequence's first anim, unless STUDIO_AL_POSE (raw pose units)
        pseqdesc->numautolayers = static_cast<int32_t>(seq.autolayers.size());
        pseqdesc->autolayerindex = static_cast<int32_t>(buf.p() - pSequenceStart);
        {
            fmt::mstudioautolayer_t* pautolayer =
                reinterpret_cast<fmt::mstudioautolayer_t*>(buf.p());
            buf.pos += seq.autolayers.size() * sizeof(fmt::mstudioautolayer_t);
            int panim00frames = 1;
            if (!seq.animIndices.empty() && seq.animIndices[0] >= 0)
                panim00frames = m.anims[seq.animIndices[0]].numframes;
            for (size_t j = 0; j < seq.autolayers.size(); j++) {
                const cm::AutoLayer& al = seq.autolayers[j];
                pautolayer[j].iSequence = static_cast<int16_t>(al.sequence);
                pautolayer[j].iPose = static_cast<int16_t>(al.pose);
                pautolayer[j].flags = al.flags;
                if (!(al.flags & fmt::STUDIO_AL_POSE)) {
                    float denom = static_cast<float>(panim00frames - 1);
                    pautolayer[j].start = al.start / denom;
                    pautolayer[j].peak = al.peak / denom;
                    pautolayer[j].tail = al.tail / denom;
                    pautolayer[j].end = al.end / denom;
                } else {
                    pautolayer[j].start = al.start;
                    pautolayer[j].peak = al.peak;
                    pautolayer[j].tail = al.tail;
                    pautolayer[j].end = al.end;
                }
            }
        }

        // boneweights, deduped against previously written blocks
        {
            size_t matchOffset = 0;
            bool matched = false;
            size_t pweight = 0;
            for (size_t k = 0; k < writtenWeights.size(); k++) {
                if (writtenWeightOffsets[k] > pweight) {
                    pweight = writtenWeightOffsets[k];
                    size_t j = 0;
                    for (; j < static_cast<size_t>(numbones); j++)
                        if (seq.weight[j] != writtenWeights[k][j])
                            break;
                    if (j == static_cast<size_t>(numbones)) {
                        matched = true;
                        matchOffset = writtenWeightOffsets[k];
                        break;
                    }
                }
            }
            if (!matched) {
                float* pw = reinterpret_cast<float*>(buf.p());
                pseqdesc->weightlistindex = static_cast<int32_t>(buf.p() - pSequenceStart);
                size_t off = buf.pos;
                buf.pos += numbones * sizeof(float);
                for (int j = 0; j < numbones; j++)
                    pw[j] = seq.weight[j];
                writtenWeights.push_back(seq.weight);
                writtenWeightOffsets.push_back(off);
            } else {
                pseqdesc->weightlistindex =
                    static_cast<int32_t>(matchOffset - (pSequenceStart - buf.start()));
            }
        }

        // iklocks
        pseqdesc->numiklocks = static_cast<int32_t>(seq.iklocks.size());
        pseqdesc->iklockindex = static_cast<int32_t>(buf.p() - pSequenceStart);
        {
            fmt::mstudioiklock_t* piklock = reinterpret_cast<fmt::mstudioiklock_t*>(buf.p());
            buf.pos += seq.iklocks.size() * sizeof(fmt::mstudioiklock_t);
            buf.Align4();
            for (size_t j = 0; j < seq.iklocks.size(); j++) {
                piklock[j].chain = seq.iklocks[j].chain;
                piklock[j].flPosWeight = seq.iklocks[j].flPosWeight;
                piklock[j].flLocalQWeight = seq.iklocks[j].flLocalQWeight;
            }
        }

        // blend anim index shorts
        int16_t* blends = reinterpret_cast<int16_t*>(buf.p());
        pseqdesc->animindexindex = static_cast<int32_t>(buf.p() - pSequenceStart);
        buf.pos += seq.groupsize[0] * seq.groupsize[1] * sizeof(int16_t);
        buf.Align4();
        for (int j = 0; j < seq.groupsize[0]; j++) {
            for (int k = 0; k < seq.groupsize[1]; k++) {
                int offset = k * seq.groupsize[0] + j;
                int idx = offset < static_cast<int>(seq.animIndices.size())
                              ? seq.animIndices[offset] : -1;
                blends[offset] = static_cast<int16_t>(idx >= 0 ? idx : 0);
            }
        }

        pseqdesc->cycleposeindex = seq.cycleposeindex;

        // seq keyvalues (WriteSeqKeyValues): raw text plus a
        // null terminator counted in the size
        pseqdesc->keyvalueindex = static_cast<int32_t>(buf.p() - pSequenceStart);
        pseqdesc->keyvaluesize = static_cast<int32_t>(seq.keyvalues.size());
        if (pseqdesc->keyvaluesize) {
            memcpy(buf.p(seq.keyvalues.size() + 1), seq.keyvalues.data(), seq.keyvalues.size());
            buf.p()[seq.keyvalues.size()] = 0;
            pseqdesc->keyvaluesize++;
            buf.pos += pseqdesc->keyvaluesize;
        }
        buf.Align4();

        // activity modifiers
        pseqdesc->numactivitymodifiers =
            static_cast<int32_t>(seq.activitymodifiers.size());
        pseqdesc->activitymodifierindex =
            static_cast<int32_t>(buf.p() - pSequenceStart);
        {
            fmt::mstudioactivitymodifier_t* pmod =
                reinterpret_cast<fmt::mstudioactivitymodifier_t*>(buf.p());
            buf.pos += seq.activitymodifiers.size() * sizeof(*pmod);
            for (size_t j = 0; j < seq.activitymodifiers.size(); j++)
                g_strings.Add(buf, &pmod[j], &pmod[j].sznameindex,
                              seq.activitymodifiers[j].c_str());
        }
        buf.Align4();

        // animtags. `tag` stays 0 - it is resolved at runtime.
        pseqdesc->numanimtags = static_cast<int32_t>(seq.animtags.size());
        pseqdesc->animtagindex = static_cast<int32_t>(buf.p() - pSequenceStart);
        {
            fmt::mstudioanimtag_t* ptag =
                reinterpret_cast<fmt::mstudioanimtag_t*>(buf.p());
            buf.pos += seq.animtags.size() * sizeof(*ptag);
            for (size_t j = 0; j < seq.animtags.size(); j++) {
                ptag[j].cycle = seq.animtags[j].cycle;
                g_strings.Add(buf, &ptag[j], &ptag[j].sztagindex,
                              seq.animtags[j].name.c_str());
            }
        }

        pseqdesc->rootDriverIndex = seq.rootDriverBone;
        buf.Align4();
    }

    // transition graph: an int per node name, then the
    // numnodes^2 byte matrix
    const int numxnodes = static_cast<int>(m.xnodenames.size());
    int32_t* pxnodename = reinterpret_cast<int32_t*>(buf.p());
    phdr->localnodenameindex = static_cast<int32_t>(buf.pos);
    buf.pos += numxnodes * sizeof(int32_t);
    buf.Align4();
    for (int i = 0; i < numxnodes; i++)
        g_strings.Add(buf, phdr, &pxnodename[i], m.xnodenames[i].c_str());

    phdr->numlocalnodes = numxnodes;
    phdr->localnodeindex = static_cast<int32_t>(buf.pos);
    if (numxnodes) {
        memcpy(buf.p(m.xnode.size()), m.xnode.data(), m.xnode.size());
        buf.pos += m.xnode.size();
    }
    buf.Align4();
}

// ---------------------------------------------------------------------------
// WriteModel - bodyparts, models, meshes (no flex/eyeballs)
// ---------------------------------------------------------------------------
void WriteModel(Buf& buf, fmt::studiohdr_t* phdr, cm::CompiledModel& m) {
    // the reference's "ik/pose" line covers everything from here through the
    // pose parameters - bodyparts, flex tables, ik chains, mouths
    const size_t ikPoseStart = buf.pos;
    fmt::mstudiobodyparts_t* pbodypart = reinterpret_cast<fmt::mstudiobodyparts_t*>(buf.p());
    phdr->numbodyparts = static_cast<int32_t>(m.bodyparts.size());
    phdr->bodypartindex = static_cast<int32_t>(buf.pos);
    buf.pos += m.bodyparts.size() * sizeof(fmt::mstudiobodyparts_t);

    fmt::mstudiomodel_t* pmodel = reinterpret_cast<fmt::mstudiomodel_t*>(buf.p());
    buf.pos += m.models.size() * sizeof(fmt::mstudiomodel_t);

    {
        int j = 0;
        for (size_t i = 0; i < m.bodyparts.size(); i++) {
            g_strings.Add(buf, &pbodypart[i], &pbodypart[i].sznameindex,
                          m.bodyparts[i].name.c_str());
            pbodypart[i].nummodels = static_cast<int32_t>(m.bodyparts[i].modelIndices.size());
            pbodypart[i].base = m.bodyparts[i].base;
            pbodypart[i].modelindex = static_cast<int32_t>(
                reinterpret_cast<uint8_t*>(&pmodel[j]) -
                reinterpret_cast<uint8_t*>(&pbodypart[i]));
            j += static_cast<int>(m.bodyparts[i].modelIndices.size());
        }
    }
    buf.Align4();

    // write global flex names
    {
        fmt::mstudioflexdesc_t* pflexdesc = reinterpret_cast<fmt::mstudioflexdesc_t*>(buf.p());
        phdr->numflexdesc = static_cast<int32_t>(m.flexdescs.size());
        phdr->flexdescindex = static_cast<int32_t>(buf.pos);
        buf.pos += m.flexdescs.size() * sizeof(fmt::mstudioflexdesc_t);
        buf.Align4();
        for (size_t j = 0; j < m.flexdescs.size(); j++)
            g_strings.Add(buf, &pflexdesc[j], &pflexdesc[j].szFACSindex,
                          m.flexdescs[j].name.c_str());
    }

    // write global flex controllers
    {
        fmt::mstudioflexcontroller_t* pflexcontroller =
            reinterpret_cast<fmt::mstudioflexcontroller_t*>(buf.p());
        phdr->numflexcontrollers = static_cast<int32_t>(m.flexcontrollers.size());
        phdr->flexcontrollerindex = static_cast<int32_t>(buf.pos);
        buf.pos += m.flexcontrollers.size() * sizeof(fmt::mstudioflexcontroller_t);
        buf.Align4();
        for (size_t j = 0; j < m.flexcontrollers.size(); j++) {
            g_strings.Add(buf, &pflexcontroller[j], &pflexcontroller[j].sznameindex,
                          m.flexcontrollers[j].name.c_str());
            g_strings.Add(buf, &pflexcontroller[j], &pflexcontroller[j].sztypeindex,
                          m.flexcontrollers[j].type.c_str());
            pflexcontroller[j].min = m.flexcontrollers[j].min;
            pflexcontroller[j].max = m.flexcontrollers[j].max;
            pflexcontroller[j].localToGlobal = -1; // remapped at load time
        }
        // a computed size, not a delta - the reference reports the table only
        Report(g_mdlReport, "flexcontrollers  %7zu bytes (%d)",
               m.flexcontrollers.size() * sizeof(fmt::mstudioflexcontroller_t),
               static_cast<int>(m.flexcontrollers.size()));
    }

    // write flex rules + inline op arrays
    {
        fmt::mstudioflexrule_t* pflexrule = reinterpret_cast<fmt::mstudioflexrule_t*>(buf.p());
        phdr->numflexrules = static_cast<int32_t>(m.flexrules.size());
        phdr->flexruleindex = static_cast<int32_t>(buf.pos);
        buf.pos += m.flexrules.size() * sizeof(fmt::mstudioflexrule_t);
        buf.Align4();

        for (size_t j = 0; j < m.flexrules.size(); j++) {
            pflexrule[j].flex = m.flexrules[j].flex;
            pflexrule[j].numops = static_cast<int32_t>(m.flexrules[j].ops.size());
            pflexrule[j].opindex = static_cast<int32_t>(
                buf.p() - reinterpret_cast<uint8_t*>(&pflexrule[j]));

            fmt::mstudioflexop_t* pflexop = reinterpret_cast<fmt::mstudioflexop_t*>(buf.p());
            for (size_t i = 0; i < m.flexrules[j].ops.size(); i++) {
                pflexop[i].op = m.flexrules[j].ops[i].op;
                pflexop[i].d.index = m.flexrules[j].ops[i].d.index; // raw union copy
            }
            buf.pos += sizeof(fmt::mstudioflexop_t) * m.flexrules[j].ops.size();
            buf.Align4();
        }
    }

    // write flex controller UI: one entry per remap; leftover
    // controllers pair adjacent right_/left_ names, else simple passthru
    {
        phdr->numflexcontrollerui = 0;
        phdr->flexcontrolleruiindex = static_cast<int32_t>(buf.pos);

        const size_t numControllers = m.flexcontrollers.size();
        std::vector<uint8_t> handled(numControllers, 0);

        // case-SENSITIVE prefix check (StringAfterPrefixCaseSensitive)
        auto afterPrefix = [](const std::string& s, const char* prefix) -> const char* {
            size_t n = strlen(prefix);
            return s.compare(0, n, prefix) == 0 ? s.c_str() + n : nullptr;
        };

        for (size_t j = 0; j < numControllers; ++j) {
            if (handled[j])
                continue;

            fmt::mstudioflexcontrollerui_t* pUI =
                reinterpret_cast<fmt::mstudioflexcontrollerui_t*>(buf.p());
            // offset of a controller entry relative to THIS ui struct
            auto ctrlIndex = [&](int idx) {
                return static_cast<int32_t>(phdr->flexcontrollerindex -
                                            static_cast<int32_t>(buf.pos) +
                                            idx * static_cast<int32_t>(
                                                      sizeof(fmt::mstudioflexcontroller_t)));
            };

            bool found = false;
            for (const source::ControllerRemap& remap : m.flexControllerRemaps) {
                int ji = static_cast<int>(j);
                if (ji != remap.index && ji != remap.leftIndex && ji != remap.rightIndex &&
                    ji != remap.multiIndex)
                    continue;

                g_strings.Add(buf, pUI, &pUI->sznameindex, remap.name.c_str());
                pUI->stereo = remap.stereo ? 1 : 0;
                if (remap.stereo) {
                    pUI->szindex0 = ctrlIndex(remap.leftIndex);
                    handled[remap.leftIndex] = 1;
                    pUI->szindex1 = ctrlIndex(remap.rightIndex);
                    handled[remap.rightIndex] = 1;
                } else {
                    pUI->szindex0 = ctrlIndex(remap.index);
                    handled[remap.index] = 1;
                    pUI->szindex1 = 0;
                }
                pUI->remaptype = static_cast<uint8_t>(remap.type);
                if (remap.type == source::RemapType::NWay ||
                    remap.type == source::RemapType::Eyelid) {
                    pUI->szindex2 = ctrlIndex(remap.multiIndex);
                    handled[remap.multiIndex] = 1;
                } else {
                    pUI->szindex2 = 0;
                }
                found = true;
                break;
            }

            if (!found) {
                pUI->remaptype = fmt::FLEXCONTROLLER_REMAP_PASSTHRU;
                pUI->szindex2 = 0;

                const std::string& name = m.flexcontrollers[j].name;
                const char* right = afterPrefix(name, "right_");
                const char* left = afterPrefix(name, "left_");
                const char* nextLeft =
                    (j + 1 < numControllers)
                        ? afterPrefix(m.flexcontrollers[j + 1].name, "left_") : nullptr;
                const char* prevRight =
                    (j > 0) ? afterPrefix(m.flexcontrollers[j - 1].name, "right_") : nullptr;

                if (right && nextLeft && strcmp(right, nextLeft) == 0) {
                    g_strings.Add(buf, pUI, &pUI->sznameindex, name.c_str() + 6);
                    pUI->stereo = 1;
                    pUI->szindex0 = ctrlIndex(static_cast<int>(j) + 1); // left
                    handled[j + 1] = 1;
                    pUI->szindex1 = ctrlIndex(static_cast<int>(j)); // right
                    handled[j] = 1;
                } else if (left && prevRight && strcmp(left, prevRight) == 0) {
                    g_strings.Add(buf, pUI, &pUI->sznameindex, name.c_str() + 5);
                    pUI->stereo = 1;
                    pUI->szindex0 = ctrlIndex(static_cast<int>(j)); // left
                    handled[j] = 1;
                    pUI->szindex1 = ctrlIndex(static_cast<int>(j) - 1); // right
                    handled[j - 1] = 1;
                } else {
                    g_strings.Add(buf, pUI, &pUI->sznameindex, name.c_str());
                    pUI->stereo = 0;
                    pUI->szindex0 = ctrlIndex(static_cast<int>(j));
                    pUI->szindex1 = 0;
                    handled[j] = 1;
                }
            }

            phdr->numflexcontrollerui++;
            buf.pos += sizeof(fmt::mstudioflexcontrollerui_t);
        }
        buf.Align4();
    }

    // ik chains: chain array + ALIGN4, then per-chain link
    // arrays appended back-to-back (linkindex chain-relative)
    {
        fmt::mstudioikchain_t* pikchain = reinterpret_cast<fmt::mstudioikchain_t*>(buf.p());
        phdr->numikchains = static_cast<int32_t>(m.ikchains.size());
        phdr->ikchainindex = static_cast<int32_t>(buf.pos);
        buf.pos += m.ikchains.size() * sizeof(fmt::mstudioikchain_t);
        buf.Align4();

        for (size_t j = 0; j < m.ikchains.size(); j++) {
            const cm::IkChain& chain = m.ikchains[j];
            g_strings.Add(buf, &pikchain[j], &pikchain[j].sznameindex, chain.name.c_str());
            pikchain[j].numlinks = 3;

            fmt::mstudioiklink_t* piklink = reinterpret_cast<fmt::mstudioiklink_t*>(buf.p());
            pikchain[j].linkindex = static_cast<int32_t>(
                buf.p() - reinterpret_cast<uint8_t*>(&pikchain[j]));
            buf.pos += 3 * sizeof(fmt::mstudioiklink_t);

            for (int i = 0; i < 3; i++) {
                piklink[i].bone = chain.link[i].bone;
                CopyV3(piklink[i].kneeDir, chain.link[i].kneeDir);
            }
        }
    }

    // autoplay locks
    {
        fmt::mstudioiklock_t* piklock = reinterpret_cast<fmt::mstudioiklock_t*>(buf.p());
        phdr->numlocalikautoplaylocks = static_cast<int32_t>(m.ikautoplaylocks.size());
        phdr->localikautoplaylockindex = static_cast<int32_t>(buf.pos);
        buf.pos += m.ikautoplaylocks.size() * sizeof(fmt::mstudioiklock_t);
        buf.Align4();

        for (size_t j = 0; j < m.ikautoplaylocks.size(); j++) {
            piklock[j].chain = m.ikautoplaylocks[j].chain;
            piklock[j].flPosWeight = m.ikautoplaylocks[j].flPosWeight;
            piklock[j].flLocalQWeight = m.ikautoplaylocks[j].flLocalQWeight;
        }
    }

    // mouths
    {
        fmt::mstudiomouth_t* pmouth = reinterpret_cast<fmt::mstudiomouth_t*>(buf.p());
        phdr->nummouths = static_cast<int32_t>(m.mouths.size());
        phdr->mouthindex = static_cast<int32_t>(buf.pos);
        buf.pos += m.mouths.size() * sizeof(fmt::mstudiomouth_t);
        buf.Align4();

        for (size_t j = 0; j < m.mouths.size(); j++) {
            pmouth[j].bone = m.mouths[j].bone;
            CopyV3(pmouth[j].forward, m.mouths[j].forward);
            pmouth[j].flexdesc = m.mouths[j].flexdesc;
        }
    }

    // pose parameters
    {
        fmt::mstudioposeparamdesc_t* ppose =
            reinterpret_cast<fmt::mstudioposeparamdesc_t*>(buf.p());
        phdr->numlocalposeparameters = static_cast<int32_t>(m.poseparams.size());
        phdr->localposeparamindex = static_cast<int32_t>(buf.pos);
        buf.pos += m.poseparams.size() * sizeof(fmt::mstudioposeparamdesc_t);
        buf.Align4();

        for (size_t i = 0; i < m.poseparams.size(); i++) {
            g_strings.Add(buf, &ppose[i], &ppose[i].sznameindex, m.poseparams[i].name.c_str());
            ppose[i].start = m.poseparams[i].min;
            ppose[i].end = m.poseparams[i].max;
            ppose[i].flags = m.poseparams[i].flags;
            ppose[i].loop = m.poseparams[i].loop;
        }
    }
    Report(g_mdlReport, "ik/pose    %7zu bytes", buf.pos - ikPoseStart);

    // ComputeVertAnimFixedPointScale: max |component| over
    // every flexkey vanim; wrinkle contributes only for wrinkle-type keys.
    // Header field + flag are only set when the scale isn't the 1/4096
    // default (so a no-flex model keeps its zero-filled bytes).
    float flVertAnimFixedPointScale;
    {
        float flVertAnimRange = 0.0f;
        for (const cm::FlexKey& key : m.flexkeys) {
            if (key.vanim.empty())
                continue;
            const bool bWrinkleVAnim = (key.vanimtype == fmt::STUDIO_VERT_ANIM_WRINKLE);
            for (const source::SrcVertAnim& va : key.vanim) {
                float c[6] = {va.pos.x, va.pos.y, va.pos.z,
                              va.normal.x, va.normal.y, va.normal.z};
                for (float v : c)
                    if (fabsf(v) > flVertAnimRange)
                        flVertAnimRange = fabsf(v);
                if (bWrinkleVAnim && fabsf(va.wrinkle) > flVertAnimRange)
                    flVertAnimRange = fabsf(va.wrinkle);
            }
        }

        flVertAnimFixedPointScale = 1.0f / 4096.0f;
        if (flVertAnimRange > 0.0f) {
            if (flVertAnimRange > 32767) {
                std::fprintf(stderr, "warning: flex value too large: %.2f, max: 32767\n",
                             flVertAnimRange);
                flVertAnimFixedPointScale = 1.0f;
            } else {
                const float flTmpScale = flVertAnimRange / 32767.0f;
                if (flTmpScale > flVertAnimFixedPointScale)
                    flVertAnimFixedPointScale = flTmpScale;
            }
        }
        if (flVertAnimFixedPointScale != 1.0f / 4096.0f) {
            phdr->flags |= fmt::STUDIOHDR_FLAGS_VERT_ANIM_FIXED_POINT_SCALE;
            phdr->flVertAnimFixedPointScale = flVertAnimFixedPointScale;
        }
    }

    // running byte offsets into the FUTURE .vvd data blocks
    size_t externalVertexIndex = 0;
    size_t externalTangentsIndex = 0;

    // the reference reports eyeball and flex bytes per model as it goes
    size_t cur = buf.pos;

    for (size_t i = 0; i < m.models.size(); i++) {
        cm::Model& model = m.models[i];
        uint8_t* pModelStart = reinterpret_cast<uint8_t*>(&pmodel[i]);

        // the reference writes the model FILENAME (or $rendermesh alias); a
        // blank model has an empty filename -> name stays zeroed
        if (model.source)
            strncpy(pmodel[i].name, model.name.c_str(), sizeof(pmodel[i].name) - 1);

        int numvertices = static_cast<int>(model.vertices.size());
        pmodel[i].numvertices = numvertices;

        // one entry per output mesh - normally one per material, more where
        // BuildOutputMeshes had to split an oversized one
        const int nummeshes = static_cast<int>(model.outMeshes.size());

        fmt::mstudiomesh_t* pmesh = reinterpret_cast<fmt::mstudiomesh_t*>(buf.p());
        pmodel[i].meshindex = static_cast<int32_t>(buf.p() - pModelStart);
        buf.pos += nummeshes * sizeof(fmt::mstudiomesh_t);
        buf.Align4();

        pmodel[i].nummeshes = nummeshes;
        for (int mm = 0; mm < nummeshes; mm++) {
            const cm::OutMesh& om = model.outMeshes[mm];
            // already the post-cull table index (the compile stage remapped it).
            // Two meshes may carry the SAME material when one had to be split.
            pmesh[mm].material = om.material;
            pmesh[mm].modelindex = static_cast<int32_t>(
                pModelStart - reinterpret_cast<uint8_t*>(&pmesh[mm]));
            pmesh[mm].numvertices = om.numvertices;
            pmesh[mm].vertexoffset = om.vertexoffset;
        }

        // expected base offsets into external data
        externalVertexIndex = (externalVertexIndex + 15) & ~size_t(15);
        pmodel[i].vertexindex = static_cast<int32_t>(externalVertexIndex);
        externalVertexIndex += numvertices * sizeof(fmt::mstudiovertex_t);

        externalTangentsIndex = (externalTangentsIndex + 3) & ~size_t(3);
        pmodel[i].tangentsindex = static_cast<int32_t>(externalTangentsIndex);
        externalTangentsIndex += numvertices * sizeof(fmt::Vector3) + numvertices * sizeof(float);

        // eyeballs. sznameindex and texture are deliberately
        // NOT written - the reference never sets them, so they stay zero.
        {
            std::vector<const cm::Eyeball*> mine;
            for (const cm::Eyeball& eye : m.eyeballs)
                if (eye.model == static_cast<int>(i))
                    mine.push_back(&eye);

            fmt::mstudioeyeball_t* peyeball =
                reinterpret_cast<fmt::mstudioeyeball_t*>(buf.p());
            pmodel[i].numeyeballs = static_cast<int32_t>(mine.size());
            pmodel[i].eyeballindex = static_cast<int32_t>(buf.p() - pModelStart);
            buf.pos += mine.size() * sizeof(fmt::mstudioeyeball_t);
            buf.Align4();

            for (size_t j = 0; j < mine.size(); j++) {
                const cm::Eyeball& eye = *mine[j];

                // tag the owning mesh as a custom (eyeball) material; this is
                // what drives MESH_IS_EYES in the .vtx
                if (eye.mesh >= 0 && eye.mesh < nummeshes) {
                    pmesh[eye.mesh].materialtype = 1;
                    pmesh[eye.mesh].materialparam = static_cast<int32_t>(j);
                }

                peyeball[j].bone = eye.bone;
                CopyV3(peyeball[j].org, eye.org);
                peyeball[j].zoffset = eye.zoffset;
                peyeball[j].radius = eye.radius;
                CopyV3(peyeball[j].up, eye.up);
                CopyV3(peyeball[j].forward, eye.forward);
                peyeball[j].iris_scale = eye.iris_scale;

                for (int k = 0; k < 3; k++) {
                    peyeball[j].upperflexdesc[k] = eye.upperflexdesc[k];
                    peyeball[j].lowerflexdesc[k] = eye.lowerflexdesc[k];
                    peyeball[j].uppertarget[k] = eye.uppertarget[k];
                    peyeball[j].lowertarget[k] = eye.lowertarget[k];
                }

                peyeball[j].upperlidflexdesc = eye.upperlidflexdesc;
                peyeball[j].lowerlidflexdesc = eye.lowerlidflexdesc;
            }
            Report(g_mdlReport, "eyeballs   %7zu bytes (%d eyeballs)", buf.pos - cur,
                   pmodel[i].numeyeballs);
        }

        // move flexes into individual meshes
        for (int mm = 0; mm < nummeshes; mm++) {
            std::vector<int> numflexkeys(m.flexkeys.size(), 0);
            pmesh[mm].numflexes = 0;

            // count flex instances per mesh (vanims within the mesh's range)
            for (size_t j = 0; j < m.flexkeys.size(); j++) {
                if (m.flexkeys[j].imodel != static_cast<int>(i))
                    continue;
                for (const source::SrcVertAnim& va : m.flexkeys[j].vanim) {
                    int n = va.vertex - pmesh[mm].vertexoffset;
                    if (n >= 0 && n < pmesh[mm].numvertices) {
                        if (numflexkeys[j]++ == 0)
                            pmesh[mm].numflexes++;
                    }
                }
            }

            if (!pmesh[mm].numflexes)
                continue; // numflexes/flexindex stay zero (calloc parity)

            pmesh[mm].flexindex = static_cast<int32_t>(
                buf.p() - reinterpret_cast<uint8_t*>(&pmesh[mm]));
            fmt::mstudioflex_t* pflex = reinterpret_cast<fmt::mstudioflex_t*>(buf.p());
            buf.pos += pmesh[mm].numflexes * sizeof(fmt::mstudioflex_t);
            buf.Align4();

            for (size_t j = 0; j < m.flexkeys.size(); j++) {
                if (!numflexkeys[j])
                    continue;
                const cm::FlexKey& key = m.flexkeys[j];

                pflex->flexdesc = key.flexdesc;
                pflex->target0 = key.target0;
                pflex->target1 = key.target1;
                pflex->target2 = key.target2;
                pflex->target3 = key.target3;
                pflex->numverts = numflexkeys[j];
                pflex->vertindex = static_cast<int32_t>(
                    buf.p() - reinterpret_cast<uint8_t*>(pflex));
                pflex->flexpair = key.flexpair;
                pflex->vertanimtype = key.vanimtype;

                bool bWrinkleVAnim = (pflex->vertanimtype == fmt::STUDIO_VERT_ANIM_WRINKLE);
                size_t nVAnimDeltaSize = bWrinkleVAnim ? sizeof(fmt::mstudiovertanim_wrinkle_t)
                                                       : sizeof(fmt::mstudiovertanim_t);

                uint8_t* pvertanim = buf.p();
                buf.pos += pflex->numverts * nVAnimDeltaSize;
                buf.Align4();

                for (const source::SrcVertAnim& va : key.vanim) {
                    int n = va.vertex - pmesh[mm].vertexoffset;
                    if (n < 0 || n >= pmesh[mm].numvertices)
                        continue;
                    auto* pva = reinterpret_cast<fmt::mstudiovertanim_t*>(pvertanim);
                    pva->index = static_cast<uint16_t>(n);
                    // float -> byte assignment truncates (reference cast)
                    pva->speed = static_cast<uint8_t>(255.0f * va.speed);
                    pva->side = static_cast<uint8_t>(255.0f * va.side);
                    pva->delta[0].SetFloat(va.pos.x);
                    pva->delta[1].SetFloat(va.pos.y);
                    pva->delta[2].SetFloat(va.pos.z);
                    pva->ndelta[0].SetFloat(va.normal.x);
                    pva->ndelta[1].SetFloat(va.normal.y);
                    pva->ndelta[2].SetFloat(va.normal.z);
                    if (bWrinkleVAnim) {
                        // SetWrinkleFixed (studio.h)
                        int nWrinkleDeltaInt =
                            static_cast<int>(va.wrinkle / flVertAnimFixedPointScale);
                        if (nWrinkleDeltaInt < -32767) nWrinkleDeltaInt = -32767;
                        if (nWrinkleDeltaInt > 32767) nWrinkleDeltaInt = 32767;
                        reinterpret_cast<fmt::mstudiovertanim_wrinkle_t*>(pva)->wrinkledelta =
                            static_cast<int16_t>(nWrinkleDeltaInt);
                    }
                    pvertanim += nVAnimDeltaSize;
                }
                pflex++;
            }
        }
        Report(g_mdlReport, "flexes     %7zu bytes (%d flexes)", buf.pos - cur,
               static_cast<int>(m.flexkeys.size()));
        cur = buf.pos;
    }

    buf.Align4();

    // include models ($includemodel). Only sznameindex is filled - the
    // reference leaves szlabelindex at its calloc'd zero.
    const int numinclude = static_cast<int>(m.includeModels.size());
    phdr->numincludemodels = numinclude;
    phdr->includemodelindex = static_cast<int32_t>(buf.pos);
    auto* pinclude = buf.Reserve<fmt::mstudiomodelgroup_t>(numinclude);
    for (int i = 0; i < numinclude; ++i)
        g_strings.Add(buf, &pinclude[i], &pinclude[i].sznameindex,
                      m.includeModels[i].c_str());

    // animblock group info. Entry 0 is the local pseudo-block
    // and stays zeroed; the rest are byte ranges of the .ani file.
    fmt::mstudioanimblock_t* panimblock =
        reinterpret_cast<fmt::mstudioanimblock_t*>(buf.p());
    phdr->numanimblocks = static_cast<int32_t>(g_animblocks.count);
    phdr->animblockindex = static_cast<int32_t>(buf.pos);
    buf.pos += g_animblocks.count * sizeof(fmt::mstudioanimblock_t);
    buf.Align4();

    for (size_t i = 1; i < g_animblocks.count; i++) {
        panimblock[i].datastart = static_cast<int32_t>(g_animblocks.blocks[i].start);
        panimblock[i].dataend = static_cast<int32_t>(g_animblocks.blocks[i].end);
    }

    g_strings.Add(buf, phdr, &phdr->szanimblocknameindex, g_animblocks.name.c_str());
}

// ---------------------------------------------------------------------------
// WriteTextures
// ---------------------------------------------------------------------------
void WriteTextures(Buf& buf, fmt::studiohdr_t* phdr, cm::CompiledModel& m) {
    source::MaterialTable& mats = *m.mats;
    int nummaterials = static_cast<int>(mats.materialToTexture.size());

    fmt::mstudiotexture_t* ptexture = reinterpret_cast<fmt::mstudiotexture_t*>(buf.p());
    phdr->numtextures = nummaterials;
    phdr->textureindex = static_cast<int32_t>(buf.pos);
    buf.pos += nummaterials * sizeof(fmt::mstudiotexture_t);
    for (int i = 0; i < nummaterials; i++) {
        int j = mats.materialToTexture[i];
        g_strings.Add(buf, &ptexture[i], &ptexture[i].sznameindex,
                      mats.textures[j].name.c_str());
    }
    buf.Align4();

    int32_t* cdtextureoffset = reinterpret_cast<int32_t*>(buf.p());
    phdr->numcdtextures = static_cast<int32_t>(m.cdtextures.size());
    phdr->cdtextureindex = static_cast<int32_t>(buf.pos);
    buf.pos += m.cdtextures.size() * sizeof(int32_t);
    for (size_t i = 0; i < m.cdtextures.size(); i++)
        g_strings.Add(buf, phdr, &cdtextureoffset[i], m.cdtextures[i].c_str());
    buf.Align4();

    // skin table
    phdr->skinindex = static_cast<int32_t>(buf.pos);
    phdr->numskinfamilies = m.numskinfamilies;
    phdr->numskinref = m.numskinref;
    int16_t* pref = reinterpret_cast<int16_t*>(buf.p());
    for (int i = 0; i < m.numskinfamilies; i++)
        for (int j = 0; j < m.numskinref; j++)
            *pref++ = m.skinref[i][j];
    buf.pos += static_cast<size_t>(m.numskinfamilies) * m.numskinref * sizeof(int16_t);
    buf.Align4();
}

// ---------------------------------------------------------------------------
// WriteBoneTransforms
// ---------------------------------------------------------------------------
void WriteBoneTransforms(Buf& buf, fmt::studiohdr2_t* phdr2, fmt::mstudiobone_t* pBone,
                         cm::CompiledModel& m) {
    int numbones = static_cast<int>(m.bones.size());

    // srcbonetransform: one entry per bone where its own OR its parent's
    // srcRealign is non-identity. The identity test is
    // the reference's MatricesAreEqual (tolerance 1e-5) - the unconditional
    // RealignBones noise stays below it, so plain models still emit zero.
    // A procedural bone (jigglebone or animconstraint helper) is skipped
    // outright - the engine drives it, so there is no source transform to
    // record.
    const pm::matrix3x4 identity;
    auto needsTransform = [&](int i) {
        if (m.bones[i].flags & fmt::BONE_ALWAYS_PROCEDURAL)
            return false;
        int nParent = m.bones[i].parent;
        return !(pm::MatricesAreEqual(identity, m.bones[i].srcRealign) &&
                 (nParent < 0 || pm::MatricesAreEqual(identity, m.bones[nParent].srcRealign)));
    };

    int nTransformCount = 0;
    for (int i = 0; i < numbones; i++)
        if (needsTransform(i))
            ++nTransformCount;

    fmt::mstudiosrcbonetransform_t* pSrcBoneTransform =
        reinterpret_cast<fmt::mstudiosrcbonetransform_t*>(buf.p());
    phdr2->numsrcbonetransform = nTransformCount;
    phdr2->srcbonetransformindex = static_cast<int32_t>(buf.pos);
    buf.pos += static_cast<size_t>(nTransformCount) * sizeof(fmt::mstudiosrcbonetransform_t);
    int bt = 0;
    for (int i = 0; i < numbones; i++) {
        if (!needsTransform(i))
            continue;

        // pre = inv(parent realign), post = own realign, so that
        // CtoW = P * C = (P * T) * (T^-1 * C) stays constant
        int nParent = m.bones[i].parent;
        if (nParent >= 0)
            CopyM(pSrcBoneTransform[bt].pretransform, pm::MatrixInvert(m.bones[nParent].srcRealign));
        else
            CopyM(pSrcBoneTransform[bt].pretransform, identity);
        CopyM(pSrcBoneTransform[bt].posttransform, m.bones[i].srcRealign);
        g_strings.Add(buf, &pSrcBoneTransform[bt], &pSrcBoneTransform[bt].sznameindex,
                      m.bones[i].name.c_str());
        ++bt;
    }
    buf.Align4();

    if (numbones > 1) {
        phdr2->linearboneindex = static_cast<int32_t>(
            buf.p() - reinterpret_cast<uint8_t*>(phdr2));
        fmt::mstudiolinearbone_t* pLinearBone = buf.Reserve<fmt::mstudiolinearbone_t>();
        pLinearBone->numbones = numbones;

        auto writeBlock = [&](int32_t* destindex, auto getField, size_t elemSize) {
            *destindex = static_cast<int32_t>(
                buf.p() - reinterpret_cast<uint8_t*>(pLinearBone));
            uint8_t* dst = buf.p();
            buf.pos += numbones * elemSize;
            buf.Align4();
            for (int i = 0; i < numbones; i++)
                getField(dst + i * elemSize, i);
        };

        writeBlock(&pLinearBone->flagsindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].flags, 4); }, 4);
        writeBlock(&pLinearBone->parentindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].parent, 4); }, 4);
        writeBlock(&pLinearBone->posindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].pos, 12); }, 12);
        writeBlock(&pLinearBone->quatindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].quat, 16); }, 16);
        writeBlock(&pLinearBone->rotindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].rot, 12); }, 12);
        writeBlock(&pLinearBone->posetoboneindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].poseToBone, 48); }, 48);
        writeBlock(&pLinearBone->posscaleindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].posscale, 12); }, 12);
        writeBlock(&pLinearBone->rotscaleindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].rotscale, 12); }, 12);
        writeBlock(&pLinearBone->qalignmentindex,
                   [&](uint8_t* d, int i) { memcpy(d, &pBone[i].qAlignment, 16); }, 16);
    }
}

// ---------------------------------------------------------------------------
// WriteBoneFlexDrivers
//
// The driver array comes first as a block, then each driver's controls are
// appended in turn - so m_nControlIndex is relative to its own driver struct
// and the control runs are interleaved after the header block, not before it.
// ---------------------------------------------------------------------------
void WriteBoneFlexDrivers(Buf& buf, fmt::studiohdr2_t* phdr2, cm::CompiledModel& m) {
    buf.Align4();

    phdr2->m_nBoneFlexDriverCount = 0;
    phdr2->m_nBoneFlexDriverIndex = 0;

    const int nDrivers = static_cast<int>(m.boneflexdrivers.size());
    if (nDrivers <= 0)
        return;

    fmt::mstudioboneflexdriver_t* pDriver =
        reinterpret_cast<fmt::mstudioboneflexdriver_t*>(buf.p());
    phdr2->m_nBoneFlexDriverCount = nDrivers;
    phdr2->m_nBoneFlexDriverIndex =
        static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(phdr2));
    buf.pos += static_cast<size_t>(nDrivers) * sizeof(fmt::mstudioboneflexdriver_t);
    buf.Align4();

    for (int i = 0; i < nDrivers; ++i) {
        const cm::BoneFlexDriver& d = m.boneflexdrivers[i];
        const int nControls = static_cast<int>(d.controls.size());

        pDriver[i].m_nBoneIndex = d.bone;
        pDriver[i].m_nControlCount = nControls;
        pDriver[i].m_nControlIndex =
            static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(&pDriver[i]));

        fmt::mstudioboneflexdrivercontrol_t* pControl =
            reinterpret_cast<fmt::mstudioboneflexdrivercontrol_t*>(buf.p());
        buf.pos += static_cast<size_t>(nControls) * sizeof(fmt::mstudioboneflexdrivercontrol_t);
        buf.Align4();

        for (int j = 0; j < nControls; ++j) {
            pControl[j].m_nFlexControllerIndex = d.controls[j].flexControllerIndex;
            pControl[j].m_nBoneComponent = d.controls[j].component;
            pControl[j].m_flMin = d.controls[j].min;
            pControl[j].m_flMax = d.controls[j].max;
        }
    }
}

// WriteBodyGroupPresets
void WriteBodyGroupPresets(Buf& buf, fmt::studiohdr2_t* phdr2, cm::CompiledModel& m) {
    buf.Align4();

    phdr2->m_nBodyGroupPresetCount = static_cast<int32_t>(m.bodygrouppresets.size());
    phdr2->m_nBodyGroupPresetIndex = 0;
    if (m.bodygrouppresets.empty())
        return;

    fmt::mstudiobodygrouppreset_t* ppreset =
        reinterpret_cast<fmt::mstudiobodygrouppreset_t*>(buf.p());
    phdr2->m_nBodyGroupPresetIndex =
        static_cast<int32_t>(buf.p() - reinterpret_cast<uint8_t*>(phdr2));
    buf.pos += m.bodygrouppresets.size() * sizeof(fmt::mstudiobodygrouppreset_t);
    buf.Align4();

    for (size_t i = 0; i < m.bodygrouppresets.size(); ++i) {
        const cm::BodyGroupPreset& p = m.bodygrouppresets[i];
        g_strings.Add(buf, &ppreset[i], &ppreset[i].sznameindex, p.name.c_str());
        ppreset[i].iValue = p.value;
        ppreset[i].iMask = p.mask;
    }
}

// AssignMeshIDs: sequential meshid across bodyparts/models
void AssignMeshIDs(Buf& buf, fmt::studiohdr_t* phdr) {
    int numMeshes = 0;
    for (int i = 0; i < phdr->numbodyparts; i++) {
        fmt::mstudiobodyparts_t* pBodyPart = reinterpret_cast<fmt::mstudiobodyparts_t*>(
            buf.start() + phdr->bodypartindex) + i;
        for (int j = 0; j < pBodyPart->nummodels; j++) {
            fmt::mstudiomodel_t* pModel = reinterpret_cast<fmt::mstudiomodel_t*>(
                reinterpret_cast<uint8_t*>(pBodyPart) + pBodyPart->modelindex) + j;
            for (int mm = 0; mm < pModel->nummeshes; mm++) {
                fmt::mstudiomesh_t* pMesh = reinterpret_cast<fmt::mstudiomesh_t*>(
                    reinterpret_cast<uint8_t*>(pModel) + pModel->meshindex) + mm;
                pMesh->meshid = numMeshes + mm;
            }
            numMeshes += pModel->nummeshes;
        }
    }
}

// ---------------------------------------------------------------------------
// WriteVertices: first-pass .vvd buffer
// ---------------------------------------------------------------------------
std::vector<uint8_t> BuildVvd(cm::CompiledModel& m, int32_t checksum) {
    size_t totalVerts = 0;
    for (const cm::Model& model : m.models)
        totalVerts += model.vertices.size();
    Buf buf(VvdBufferSize(totalVerts, m.models.size(), 0));

    fmt::vertexFileHeader_t* fileHeader = buf.Reserve<fmt::vertexFileHeader_t>();
    fileHeader->id = fmt::kIdVertexFile;
    fileHeader->version = fmt::kVertexFileVersion;
    fileHeader->checksum = checksum;
    fileHeader->numFixups = 0;
    fileHeader->fixupTableStart = 0;
    fileHeader->numLODs = 1;
    fileHeader->numLODVertexes[0] = 0;

    g_vvdReport.clear();
    size_t vcur = buf.pos;

    buf.Align16();
    fileHeader->vertexDataStart = static_cast<int32_t>(buf.pos);
    for (cm::Model& model : m.models) {
        if (model.vertices.empty())
            continue;
        buf.Align16();
        fmt::mstudiovertex_t* pVert = reinterpret_cast<fmt::mstudiovertex_t*>(buf.p());
        buf.pos += model.vertices.size() * sizeof(fmt::mstudiovertex_t);
        for (size_t j = 0; j < model.vertices.size(); j++) {
            const cm::LodVertex& v = model.vertices[j];
            CopyV3(pVert[j].m_vecPosition, v.position);
            CopyV3(pVert[j].m_vecNormal, v.normal);
            pVert[j].m_vecTexCoord = {v.texcoord.x, v.texcoord.y};
            memset(&pVert[j].m_BoneWeights, 0, sizeof(fmt::mstudioboneweight_t));
            pVert[j].m_BoneWeights.numbones = static_cast<uint8_t>(v.boneweight.numbones);
            for (int k = 0; k < v.boneweight.numbones; k++) {
                pVert[j].m_BoneWeights.bone[k] = static_cast<uint8_t>(v.boneweight.bone[k]);
                pVert[j].m_BoneWeights.weight[k] = v.boneweight.weight[k];
            }
        }
        fileHeader->numLODVertexes[0] += static_cast<int32_t>(model.vertices.size());
        Report(g_vvdReport, "vertices   %7zu bytes (%d vertices)", buf.pos - vcur,
               static_cast<int>(model.vertices.size()));
        vcur = buf.pos;
    }

    buf.Align4();
    fileHeader->tangentDataStart = static_cast<int32_t>(buf.pos);
    for (cm::Model& model : m.models) {
        if (model.vertices.empty())
            continue;
        buf.Align4();
        float* pt = reinterpret_cast<float*>(buf.p());
        buf.pos += model.vertices.size() * 4 * sizeof(float);
        for (size_t j = 0; j < model.vertices.size(); j++) {
            pt[j * 4 + 0] = model.vertices[j].tangentS.x;
            pt[j * 4 + 1] = model.vertices[j].tangentS.y;
            pt[j * 4 + 2] = model.vertices[j].tangentS.z;
            pt[j * 4 + 3] = model.vertices[j].tangentS.w;
        }
        Report(g_vvdReport, "tangents   %7zu bytes (%d vertices)", buf.pos - vcur,
               static_cast<int>(model.vertices.size()));
        vcur = buf.pos;
    }
    Report(g_vvdReport, "total      %7zu bytes", buf.pos);

    return std::vector<uint8_t>(buf.start(), buf.start() + buf.cur);
}

// ---------------------------------------------------------------------------
// FixupToSortedLODVertexes in-memory
// ---------------------------------------------------------------------------
struct UsedVertex {
    int meshVertID;
    int finalMeshVertID;
    int vertexOffset;
    int lodFlags;
};

struct LodMeshInfo {
    int offsets[fmt::kMaxNumLods] = {};
    int numVertexes[fmt::kMaxNumLods] = {};
};

struct VertexPool {
    std::vector<UsedVertex> vertexList;
    std::vector<int> vertexMap;
    int numVertexes = 0;
    LodMeshInfo lodMeshInfo;
};

int Log2i(int v) {
    int r = 0;
    while (v > 1) {
        v >>= 1;
        r++;
    }
    return r;
}

int CompareUsedVertexes(const void* a, const void* b) {
    const UsedVertex* va = static_cast<const UsedVertex*>(a);
    const UsedVertex* vb = static_cast<const UsedVertex*>(b);
    int lodA = Log2i(va->lodFlags);
    int lodB = Log2i(vb->lodFlags);
    int sort = lodB - lodA;
    if (sort) return sort;
    sort = va->vertexOffset - vb->vertexOffset;
    if (sort) return sort;
    return va->meshVertID - vb->meshVertID;
}

void FindVertexOffsets(int vertexOffset, int* offsets, int* counts, int numLODs,
                       const std::vector<UsedVertex>& list) {
    // find the runs of the target mesh (identified by vertexOffset) per lod
    for (int n = 0; n < numLODs; n++) {
        offsets[n] = 0;
        counts[n] = 0;
    }
    for (int i = 0; i < static_cast<int>(list.size()); i++) {
        if (list[i].vertexOffset != vertexOffset)
            continue;
        int lod = Log2i(list[i].lodFlags);
        if (!counts[lod])
            offsets[lod] = i;
        counts[lod]++;
    }
}

// vtx buffer accessors (legacy layout aware)
struct VtxNav {
    uint8_t* base;
    bool legacy;
    fmt::vtx::FileHeader_t* hdr() { return reinterpret_cast<fmt::vtx::FileHeader_t*>(base); }
    fmt::vtx::BodyPartHeader_t* bodyPart(int i) {
        return reinterpret_cast<fmt::vtx::BodyPartHeader_t*>(base + hdr()->bodyPartOffset) + i;
    }
    fmt::vtx::ModelHeader_t* model(fmt::vtx::BodyPartHeader_t* bp, int i) {
        return reinterpret_cast<fmt::vtx::ModelHeader_t*>(
                   reinterpret_cast<uint8_t*>(bp) + bp->modelOffset) + i;
    }
    fmt::vtx::ModelLODHeader_t* lod(fmt::vtx::ModelHeader_t* mh, int i) {
        return reinterpret_cast<fmt::vtx::ModelLODHeader_t*>(
                   reinterpret_cast<uint8_t*>(mh) + mh->lodOffset) + i;
    }
    fmt::vtx::MeshHeader_t* mesh(fmt::vtx::ModelLODHeader_t* lh, int i) {
        return reinterpret_cast<fmt::vtx::MeshHeader_t*>(
                   reinterpret_cast<uint8_t*>(lh) + lh->meshOffset) + i;
    }
    // stride-aware strip group; returns pointers to the shared leading fields
    uint8_t* stripGroupRaw(fmt::vtx::MeshHeader_t* mh, int i) {
        size_t stride = legacy ? sizeof(fmt::vtx::LegacyStripGroupHeader_t)
                               : sizeof(fmt::vtx::StripGroupHeader_t);
        return reinterpret_cast<uint8_t*>(mh) + mh->stripGroupHeaderOffset + i * stride;
    }
};

// shared leading fields of both strip group layouts
struct SGCommon {
    int32_t numVerts;
    int32_t vertOffset;
    int32_t numIndices;
    int32_t indexOffset;
    int32_t numStrips;
    int32_t stripOffset;
};

bool FixupBuffers(cm::CompiledModel& m, std::vector<uint8_t>& mdlBuf,
                  std::vector<uint8_t>& vvdBuf, std::vector<uint8_t>& vtxBuf, bool legacyVtx,
                  std::string* err) {
    fmt::studiohdr_t* pStudioHdr = reinterpret_cast<fmt::studiohdr_t*>(mdlBuf.data());
    VtxNav vtx{vtxBuf.data(), legacyVtx};
    int numLODs = vtx.hdr()->numLODs;

    // ---- BuildSortedVertexList ----
    std::vector<VertexPool> pools;
    {
        auto* bodyparts = reinterpret_cast<fmt::mstudiobodyparts_t*>(
            mdlBuf.data() + pStudioHdr->bodypartindex);
        for (int i = 0; i < pStudioHdr->numbodyparts; i++) {
            fmt::mstudiobodyparts_t* bp = &bodyparts[i];
            auto* models = reinterpret_cast<fmt::mstudiomodel_t*>(
                reinterpret_cast<uint8_t*>(bp) + bp->modelindex);
            for (int j = 0; j < bp->nummodels; j++)
                pools.resize(pools.size() + models[j].nummeshes);
        }
    }

    int numVertexPools = 0;
    {
        auto* bodyparts = reinterpret_cast<fmt::mstudiobodyparts_t*>(
            mdlBuf.data() + pStudioHdr->bodypartindex);
        for (int i = 0; i < vtx.hdr()->numBodyParts; i++) {
            fmt::vtx::BodyPartHeader_t* vbp = vtx.bodyPart(i);
            fmt::mstudiobodyparts_t* bp = &bodyparts[i];
            auto* models = reinterpret_cast<fmt::mstudiomodel_t*>(
                reinterpret_cast<uint8_t*>(bp) + bp->modelindex);
            for (int j = 0; j < vbp->numModels; j++) {
                fmt::vtx::ModelHeader_t* vmh = vtx.model(vbp, j);
                fmt::mstudiomodel_t* sm = &models[j];

                int poolStart = numVertexPools;
                int vertexOffset = 0;
                for (int mm = 0; mm < poolStart; mm++)
                    vertexOffset += pools[mm].numVertexes;
                auto* meshes = reinterpret_cast<fmt::mstudiomesh_t*>(
                    reinterpret_cast<uint8_t*>(sm) + sm->meshindex);
                for (int k = 0; k < sm->nummeshes; k++) {
                    int numMeshVertexes = meshes[k].numvertices;
                    VertexPool& pool = pools[numVertexPools];
                    pool.numVertexes = numMeshVertexes;
                    pool.vertexList.resize(numMeshVertexes);
                    pool.vertexMap.resize(numMeshVertexes);
                    for (int n = 0; n < numMeshVertexes; n++) {
                        pool.vertexList[n] = {n, -1, vertexOffset, 0};
                        pool.vertexMap[n] = n;
                    }
                    numVertexPools++;
                    vertexOffset += numMeshVertexes;
                }

                for (int currLod = 0; currLod < numLODs; currLod++) {
                    fmt::vtx::ModelLODHeader_t* vlh = vtx.lod(vmh, currLod);
                    if (vlh->numMeshes != sm->nummeshes) {
                        if (err) *err = "vtx/mdl mesh count mismatch";
                        return false;
                    }
                    for (int k = 0; k < vlh->numMeshes; k++) {
                        fmt::vtx::MeshHeader_t* vmesh = vtx.mesh(vlh, k);
                        for (int mm = 0; mm < vmesh->numStripGroups; mm++) {
                            uint8_t* sgRaw = vtx.stripGroupRaw(vmesh, mm);
                            SGCommon* sg = reinterpret_cast<SGCommon*>(sgRaw);
                            VertexPool& pool = pools[poolStart + k];
                            for (int n = 0; n < sg->numVerts; n++) {
                                auto* v = reinterpret_cast<fmt::vtx::Vertex_t*>(
                                    sgRaw + sg->vertOffset) + n;
                                if (v->origMeshVertID >= pool.numVertexes) {
                                    if (err) *err = "vtx vertex out of range";
                                    return false;
                                }
                                pool.vertexList[v->origMeshVertID].lodFlags |= 1 << currLod;
                            }
                        }
                    }
                }
            }
        }
    }

    // flatten
    std::vector<UsedVertex> vertexList;
    for (VertexPool& pool : pools) {
        for (UsedVertex& v : pool.vertexList)
            if (!v.lodFlags)
                v.lodFlags = 1 << (numLODs - 1);
        vertexList.insert(vertexList.end(), pool.vertexList.begin(), pool.vertexList.end());
    }
    int numVertexes = static_cast<int>(vertexList.size());

    if (numVertexes > 0)
        qsort(vertexList.data(), numVertexes, sizeof(UsedVertex), CompareUsedVertexes);

    // map (vertexOffset, meshVertID) -> sorted index.
    // Indexed rather than scanned: the reference scans vertexList for every
    // vertex, which is O(numVertexes^2) and dominated write time on character
    // models. Building the index k-ascending and keeping the first entry per
    // key reproduces the scan's first-match-wins result exactly.
    {
        std::unordered_map<uint64_t, int> sortedIndex;
        sortedIndex.reserve(static_cast<size_t>(numVertexes) * 2);
        for (int k = 0; k < numVertexes; k++) {
            uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(vertexList[k].vertexOffset))
                            << 32) |
                           static_cast<uint32_t>(vertexList[k].meshVertID);
            sortedIndex.emplace(key, k); // first k wins
        }

        int vertexOffset = 0;
        for (VertexPool& pool : pools) {
            for (int j = 0; j < pool.numVertexes; j++) {
                uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(vertexOffset)) << 32) |
                               static_cast<uint32_t>(j);
                auto it = sortedIndex.find(key);
                if (it != sortedIndex.end())
                    pool.vertexMap[j] = it->second;
            }
            vertexOffset += pool.numVertexes;
        }
    }

    // per-mesh lod runs + final vertex ids
    {
        auto* bodyparts = reinterpret_cast<fmt::mstudiobodyparts_t*>(
            mdlBuf.data() + pStudioHdr->bodypartindex);
        int finalMeshVertID = 0;
        int poolStart = 0;
        for (int i = 0; i < pStudioHdr->numbodyparts; i++) {
            fmt::mstudiobodyparts_t* bp = &bodyparts[i];
            auto* models = reinterpret_cast<fmt::mstudiomodel_t*>(
                reinterpret_cast<uint8_t*>(bp) + bp->modelindex);
            for (int j = 0; j < bp->nummodels; j++) {
                fmt::mstudiomodel_t* sm = &models[j];
                int vertexOffset = 0;
                for (int n = 0; n < poolStart; n++)
                    vertexOffset += pools[n].numVertexes;
                auto* meshes = reinterpret_cast<fmt::mstudiomesh_t*>(
                    reinterpret_cast<uint8_t*>(sm) + sm->meshindex);
                for (int mm = 0; mm < sm->nummeshes; mm++) {
                    int offsets[fmt::kMaxNumLods] = {};
                    int counts[fmt::kMaxNumLods] = {};
                    if (meshes[mm].numvertices != 0)
                        FindVertexOffsets(vertexOffset, offsets, counts, numLODs, vertexList);
                    for (int n = 0; n < numLODs; n++) {
                        if (!counts[n]) offsets[n] = 0;
                        pools[poolStart + mm].lodMeshInfo.offsets[n] = offsets[n];
                        pools[poolStart + mm].lodMeshInfo.numVertexes[n] = counts[n];
                    }
                    int baseMeshVertID = finalMeshVertID;
                    for (int n = numLODs - 1; n >= 0; n--) {
                        for (int p = 0; p < counts[n]; p++) {
                            vertexList[offsets[n] + p].finalMeshVertID =
                                finalMeshVertID - baseMeshVertID;
                            finalMeshVertID++;
                        }
                    }
                    vertexOffset += pools[poolStart + mm].numVertexes;
                }
                poolStart += sm->nummeshes;
            }
        }
    }

    // ---- FixupVVDFile: rebuild the vvd ----
    {
        fmt::vertexFileHeader_t* oldHdr =
            reinterpret_cast<fmt::vertexFileHeader_t*>(vvdBuf.data());

        // count fixups; single mesh / single lod forces zero
        int numFixups = 0;
        int numMeshes = 0;
        {
            auto* bodyparts = reinterpret_cast<fmt::mstudiobodyparts_t*>(
                mdlBuf.data() + pStudioHdr->bodypartindex);
            for (int i = 0; i < pStudioHdr->numbodyparts; i++) {
                fmt::mstudiobodyparts_t* bp = &bodyparts[i];
                auto* models = reinterpret_cast<fmt::mstudiomodel_t*>(
                    reinterpret_cast<uint8_t*>(bp) + bp->modelindex);
                for (int j = 0; j < bp->nummodels; j++) {
                    fmt::mstudiomodel_t* sm = &models[j];
                    auto* meshes = reinterpret_cast<fmt::mstudiomesh_t*>(
                        reinterpret_cast<uint8_t*>(sm) + sm->meshindex);
                    int k = 0;
                    for (k = 0; k < sm->nummeshes; k++) {
                        if (!meshes[k].numvertices)
                            continue;
                        for (int n = numLODs - 1; n >= 0; n--)
                            if (pools[numMeshes + k].lodMeshInfo.numVertexes[n])
                                numFixups++;
                    }
                    numMeshes += k;
                }
            }
        }
        if (numMeshes == 1 || numFixups == 1 || numLODs == 1)
            numFixups = 0;

        Buf nb(VvdBufferSize(static_cast<size_t>(numVertexes), 2,
                             static_cast<size_t>(numFixups)));
        fmt::vertexFileHeader_t* newHdr = nb.Reserve<fmt::vertexFileHeader_t>();
        *newHdr = *oldHdr;
        newHdr->numLODs = numLODs;
        newHdr->numFixups = numFixups;

        nb.Align4();
        fmt::vertexFileFixup_t* fixupTable =
            reinterpret_cast<fmt::vertexFileFixup_t*>(nb.p());
        newHdr->fixupTableStart = static_cast<int32_t>(nb.pos);
        nb.pos += numFixups * sizeof(fmt::vertexFileFixup_t);

        nb.Align16();
        fmt::mstudiovertex_t* newVerts = reinterpret_cast<fmt::mstudiovertex_t*>(nb.p());
        newHdr->vertexDataStart = static_cast<int32_t>(nb.pos);
        nb.pos += numVertexes * sizeof(fmt::mstudiovertex_t);

        nb.Align16();
        float* newTangents = reinterpret_cast<float*>(nb.p());
        newHdr->tangentDataStart = static_cast<int32_t>(nb.pos);
        nb.pos += numVertexes * 4 * sizeof(float);

        uint8_t* oldVertexBase = vvdBuf.data() + oldHdr->vertexDataStart;
        uint8_t* oldTangentBase = vvdBuf.data() + oldHdr->tangentDataStart;

        // aggregate lod vert counts
        int maxCount = -1;
        for (int n = numLODs - 1; n >= 0; n--) {
            int mask = 1 << n;
            for (int p = 0; p < numVertexes; p++)
                if (mask & vertexList[p].lodFlags)
                    if (maxCount < p) maxCount = p;
            newHdr->numLODVertexes[n] = maxCount + 1;
        }
        for (int n = numLODs; n < fmt::kMaxNumLods; n++)
            newHdr->numLODVertexes[n] = newHdr->numLODVertexes[numLODs - 1];

        if (numFixups) {
            int numOutFixups = 0;
            auto* bodyparts = reinterpret_cast<fmt::mstudiobodyparts_t*>(
                mdlBuf.data() + pStudioHdr->bodypartindex);
            numMeshes = 0;
            for (int i = 0; i < pStudioHdr->numbodyparts; i++) {
                fmt::mstudiobodyparts_t* bp = &bodyparts[i];
                auto* models = reinterpret_cast<fmt::mstudiomodel_t*>(
                    reinterpret_cast<uint8_t*>(bp) + bp->modelindex);
                for (int j = 0; j < bp->nummodels; j++) {
                    fmt::mstudiomodel_t* sm = &models[j];
                    auto* meshes = reinterpret_cast<fmt::mstudiomesh_t*>(
                        reinterpret_cast<uint8_t*>(sm) + sm->meshindex);
                    for (int k = 0; k < sm->nummeshes; k++) {
                        if (!meshes[k].numvertices)
                            continue;
                        for (int n = numLODs - 1; n >= 0; n--) {
                            LodMeshInfo& info = pools[numMeshes + k].lodMeshInfo;
                            if (!info.numVertexes[n])
                                continue;
                            fixupTable[numOutFixups].lod = n;
                            fixupTable[numOutFixups].numVertexes = info.numVertexes[n];
                            fixupTable[numOutFixups].sourceVertexID = info.offsets[n];
                            numOutFixups++;
                        }
                    }
                    numMeshes += sm->nummeshes;
                }
            }
        }

        // flat lookup of old verts
        std::vector<const fmt::mstudiovertex_t*> flatVerts;
        std::vector<const float*> flatTangents;
        {
            auto* bodyparts = reinterpret_cast<fmt::mstudiobodyparts_t*>(
                mdlBuf.data() + pStudioHdr->bodypartindex);
            for (int i = 0; i < pStudioHdr->numbodyparts; i++) {
                fmt::mstudiobodyparts_t* bp = &bodyparts[i];
                auto* models = reinterpret_cast<fmt::mstudiomodel_t*>(
                    reinterpret_cast<uint8_t*>(bp) + bp->modelindex);
                for (int j = 0; j < bp->nummodels; j++) {
                    fmt::mstudiomodel_t* sm = &models[j];
                    const fmt::mstudiovertex_t* oldV =
                        reinterpret_cast<const fmt::mstudiovertex_t*>(
                            oldVertexBase + sm->vertexindex);
                    const float* oldT = reinterpret_cast<const float*>(
                        oldTangentBase + sm->tangentsindex);
                    auto* meshes = reinterpret_cast<fmt::mstudiomesh_t*>(
                        reinterpret_cast<uint8_t*>(sm) + sm->meshindex);
                    for (int k = 0; k < sm->nummeshes; k++) {
                        for (int n = 0; n < meshes[k].numvertices; n++) {
                            flatVerts.push_back(&oldV[meshes[k].vertexoffset + n]);
                            flatTangents.push_back(&oldT[(meshes[k].vertexoffset + n) * 4]);
                        }
                    }
                }
            }
        }

        for (int i = 0; i < numVertexes; i++) {
            int oldIndex = vertexList[i].vertexOffset + vertexList[i].meshVertID;
            memcpy(&newVerts[i], flatVerts[oldIndex], sizeof(fmt::mstudiovertex_t));
            memcpy(&newTangents[i * 4], flatTangents[oldIndex], 4 * sizeof(float));
        }

        vvdBuf.assign(nb.start(), nb.start() + nb.cur);
    }

    // ---- FixupVTXFile: remap origMeshVertIDs ----
    {
        int poolStart = 0;
        auto* bodyparts = reinterpret_cast<fmt::mstudiobodyparts_t*>(
            mdlBuf.data() + pStudioHdr->bodypartindex);
        for (int i = 0; i < vtx.hdr()->numBodyParts; i++) {
            fmt::vtx::BodyPartHeader_t* vbp = vtx.bodyPart(i);
            fmt::mstudiobodyparts_t* bp = &bodyparts[i];
            auto* models = reinterpret_cast<fmt::mstudiomodel_t*>(
                reinterpret_cast<uint8_t*>(bp) + bp->modelindex);
            for (int j = 0; j < vbp->numModels; j++) {
                fmt::vtx::ModelHeader_t* vmh = vtx.model(vbp, j);
                fmt::mstudiomodel_t* sm = &models[j];
                for (int currLod = 0; currLod < numLODs; currLod++) {
                    fmt::vtx::ModelLODHeader_t* vlh = vtx.lod(vmh, currLod);
                    for (int k = 0; k < vlh->numMeshes; k++) {
                        fmt::vtx::MeshHeader_t* vmesh = vtx.mesh(vlh, k);
                        for (int mm = 0; mm < vmesh->numStripGroups; mm++) {
                            uint8_t* sgRaw = vtx.stripGroupRaw(vmesh, mm);
                            SGCommon* sg = reinterpret_cast<SGCommon*>(sgRaw);
                            for (int n = 0; n < sg->numVerts; n++) {
                                auto* v = reinterpret_cast<fmt::vtx::Vertex_t*>(
                                    sgRaw + sg->vertOffset) + n;
                                int newID = pools[poolStart + k].vertexMap[v->origMeshVertID];
                                newID = vertexList[newID].finalMeshVertID;
                                v->origMeshVertID = static_cast<uint16_t>(newID);
                            }
                        }
                    }
                }
                poolStart += sm->nummeshes;
            }
        }
    }

    // ---- FixupMDLFile: numLODVertexes + null material ptrs ----
    {
        auto* bodyparts = reinterpret_cast<fmt::mstudiobodyparts_t*>(
            mdlBuf.data() + pStudioHdr->bodypartindex);
        int numMeshes = 0;
        for (int i = 0; i < pStudioHdr->numbodyparts; i++) {
            fmt::mstudiobodyparts_t* bp = &bodyparts[i];
            auto* models = reinterpret_cast<fmt::mstudiomodel_t*>(
                reinterpret_cast<uint8_t*>(bp) + bp->modelindex);
            for (int j = 0; j < bp->nummodels; j++) {
                fmt::mstudiomodel_t* sm = &models[j];
                auto* meshes = reinterpret_cast<fmt::mstudiomesh_t*>(
                    reinterpret_cast<uint8_t*>(sm) + sm->meshindex);
                for (int mm = 0; mm < sm->nummeshes; mm++) {
                    LodMeshInfo& info = pools[numMeshes + mm].lodMeshInfo;
                    int n = 0;
                    for (n = 0; n < numLODs; n++) {
                        int total = 0;
                        for (int p = n; p < numLODs; p++)
                            total += info.numVertexes[p];
                        meshes[mm].vertexdata.numLODVertexes[n] = total;
                    }
                    for (int p = n; p < fmt::kMaxNumLods; p++)
                        meshes[mm].vertexdata.numLODVertexes[p] =
                            meshes[mm].vertexdata.numLODVertexes[numLODs - 1];

                    // fix the flexes: vanim indices remap
                    // from mesh-relative to the final fixed vertex order
                    for (int fx = 0; fx < meshes[mm].numflexes; fx++) {
                        auto* pFlex = reinterpret_cast<fmt::mstudioflex_t*>(
                            reinterpret_cast<uint8_t*>(&meshes[mm]) + meshes[mm].flexindex) + fx;
                        uint8_t* pvanim =
                            reinterpret_cast<uint8_t*>(pFlex) + pFlex->vertindex;
                        size_t nVAnimSizeBytes =
                            (pFlex->vertanimtype == fmt::STUDIO_VERT_ANIM_WRINKLE)
                                ? sizeof(fmt::mstudiovertanim_wrinkle_t)
                                : sizeof(fmt::mstudiovertanim_t);
                        for (int p = 0; p < pFlex->numverts; p++, pvanim += nVAnimSizeBytes) {
                            auto* pVA = reinterpret_cast<fmt::mstudiovertanim_t*>(pvanim);
                            if (pVA->index >= meshes[mm].numvertices)
                                return false;
                            int newMeshVertID =
                                pools[numMeshes + mm].vertexMap[pVA->index];
                            newMeshVertID = vertexList[newMeshVertID].finalMeshVertID;
                            pVA->index = static_cast<uint16_t>(newMeshVertID);
                        }
                    }
                }
                numMeshes += sm->nummeshes;
            }
        }
        // material pointer slots are already zero in our buffer
    }

    return true;
}

bool SaveFile(const std::filesystem::path& path, const void* data, size_t len, std::string* err) {
    std::error_code ec;
    std::filesystem::create_directories(path.parent_path(), ec);
    FILE* f = nullptr;
#ifdef _WIN32
    fopen_s(&f, path.string().c_str(), "wb");
#else
    f = fopen(path.string().c_str(), "wb");
#endif
    if (!f) {
        if (err) *err = "cannot open for write: " + path.string();
        return false;
    }
    fwrite(data, 1, len, f);
    fclose(f);
    g_writtenFiles.push_back(path.string());
    return true;
}

} // namespace

bool WriteModelFiles(cm::CompiledModel& m, const std::string& outDir, bool legacyVtx,
                     std::string* err) {
    Buf buf(kFileBuffer);

    fmt::studiohdr_t* phdr = buf.Reserve<fmt::studiohdr_t>();
    phdr->id = fmt::kIdStudioHeader;
    phdr->version = lim::kStudioVersion;

    std::string outname = m.outname + ".mdl";

    // $animblocksize: a second output buffer holding the demand-loaded
    // animation payload, headed by its own studiohdr.
    g_animblocks.Reset();
    std::unique_ptr<Buf> blockBuf;
    fmt::studiohdr_t* pblockhdr = nullptr;
    if (m.animblocksize != 0) {
        g_animblocks.name = "models/" + m.outname + ".ani";
        blockBuf = std::make_unique<Buf>(kFileBuffer);
        pblockhdr = blockBuf->Reserve<fmt::studiohdr_t>();
        pblockhdr->id = fmt::kIdStudioAnimGroupHeader;
        pblockhdr->version = lim::kStudioVersion;
    }

    CopyV3(phdr->eyeposition, m.eyeposition);
    CopyV3(phdr->illumposition, m.illumposition);

    // hull from sequence 0 unless $bbox
    pm::Vector3 bbox0 = m.bbox[0], bbox1 = m.bbox[1];
    if (!m.bboxset) {
        if (!m.sequences.empty()) {
            bbox0 = m.sequences[0].bmin;
            bbox1 = m.sequences[0].bmax;
        }
        // CollisionModel_ExpandBBox: the render hull has to
        // enclose the collision hull, otherwise the model is culled while its
        // physics is still visible. Only for a derived box - an authored $bbox
        // is written exactly as the script gave it.
        if (m.physCollideBoundsSet) {
            bbox0 = {std::min(bbox0.x, m.physCollideMins.x), std::min(bbox0.y, m.physCollideMins.y),
                     std::min(bbox0.z, m.physCollideMins.z)};
            bbox1 = {std::max(bbox1.x, m.physCollideMaxs.x), std::max(bbox1.y, m.physCollideMaxs.y),
                     std::max(bbox1.z, m.physCollideMaxs.z)};
        }
    }
    pm::Vector3 cbox0{}, cbox1{};
    if (m.cboxset) {
        cbox0 = m.cbox[0];
        cbox1 = m.cbox[1];
    }
    CopyV3(phdr->hull_min, bbox0);
    CopyV3(phdr->hull_max, bbox1);
    CopyV3(phdr->view_bbmin, cbox0);
    CopyV3(phdr->view_bbmax, cbox1);

    phdr->flags = m.gflags;
    phdr->mass = 1;
    phdr->constdirectionallightdot = 0;

    // studiohdr2 immediately follows, always
    phdr->studiohdr2index = static_cast<int32_t>(buf.pos);
    fmt::studiohdr2_t* phdr2 = buf.Reserve<fmt::studiohdr2_t>();
    memset(phdr2, 0, sizeof(*phdr2));
    phdr2->illumpositionattachmentindex = m.illumpositionattachment;
    phdr2->flMaxEyeDeflection = m.maxEyeDeflection;

    g_strings.Begin();
    strncpy(phdr->name, outname.c_str(), sizeof(phdr->name) - 1);
    g_strings.Add(buf, phdr2, &phdr2->sznameindex, outname.c_str());

    // per-block byte accounting, in the reference's own order and wording
    g_mdlReport.clear();
    size_t sec = buf.pos;
    auto section = [&](const char* fmt, ...) {
        char line[256];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(line, sizeof(line), fmt, ap);
        va_end(ap);
        g_mdlReport.emplace_back(line);
        sec = buf.pos;
    };

    WriteBoneInfo(buf, phdr, m);
    section("bones      %7zu bytes (%d)", buf.pos - sec, phdr->numbones);

    // frames/runtime are reported alongside the animation bytes
    int totalframes = 0;
    float totalseconds = 0.0f;
    for (const cm::Anim& anim : m.anims) {
        totalframes += anim.numframes;
        if (anim.fps > 0.0f)
            totalseconds += static_cast<float>(anim.numframes) / anim.fps;
    }

    WriteAnimations(buf, phdr, m, blockBuf.get());
    section("animations %7zu bytes (%d anims) (%d frames) [%d:%02d]", buf.pos - sec,
            static_cast<int>(m.anims.size()), totalframes, static_cast<int>(totalseconds) / 60,
            static_cast<int>(totalseconds) % 60);
    if (g_animblocks.count > static_cast<size_t>(lim::kMaxAnimBlocks)) {
        if (err)
            *err = "the model needs " + std::to_string(g_animblocks.count) +
                   " animation blocks, over the limit of " +
                   std::to_string(lim::kMaxAnimBlocks) + " - raise $animblocksize";
        return false;
    }
    WriteSequenceInfo(buf, phdr, m);
    section("sequences  %7zu bytes (%d seq) ", buf.pos - sec,
            static_cast<int>(m.sequences.size()));

    // WriteModel appends its own nested eyeballs/flexes lines as it goes, so
    // they land ahead of the "models" total on their own
    WriteModel(buf, phdr, m);
    section("models     %7zu bytes", buf.pos - sec);

    WriteTextures(buf, phdr, m);
    section("textures   %7zu bytes", buf.pos - sec);

    // $keyvalues (WriteKeyValues). Unlike the per-sequence
    // block, the model-level text is wrapped in an "mdlkeyvalue { }" block
    // first (CapKeyValues).
    phdr->keyvalueindex = static_cast<int32_t>(buf.pos);
    phdr->keyvaluesize = 0;
    if (!m.keyvalues.empty()) {
        const std::string capped = "mdlkeyvalue\n{\n" + m.keyvalues + "}\n";
        memcpy(buf.p(capped.size() + 1), capped.data(), capped.size());
        buf.p()[capped.size()] = 0;
        phdr->keyvaluesize = static_cast<int32_t>(capped.size()) + 1;
        buf.pos += phdr->keyvaluesize;
    }
    buf.Align4();
    section("keyvalues  %7zu bytes", buf.pos - sec);

    WriteBoneTransforms(buf, phdr2,
                        reinterpret_cast<fmt::mstudiobone_t*>(buf.start() + phdr->boneindex), m);
    section("bone transforms  %7zu bytes", buf.pos - sec);

    WriteBoneFlexDrivers(buf, phdr2, m);
    section("bone flex driver %7zu bytes", buf.pos - sec);

    WriteBodyGroupPresets(buf, phdr2, m);
    section("bodygroup presets %7zu bytes", buf.pos - sec);

    g_strings.Write(buf);

    size_t total = buf.pos;

    // the reference takes this delta AFTER the string table, so it is the
    // collision block's size in the .mdl - always zero, the hulls live in .phy
    section("collision  %7zu bytes", buf.pos - total);

    // checksum over the whole buffer, BEFORE AssignMeshIDs and the fixup pass
    phdr->checksum = 0;
    for (size_t i = 0; i < total; i += 4) {
        phdr->checksum = (phdr->checksum << 1) + ((phdr->checksum & 0x8000000) ? 1 : 0) +
                         *reinterpret_cast<int32_t*>(buf.start() + i);
    }

    AssignMeshIDs(buf, phdr);
    phdr->length = static_cast<int32_t>(total);

    if (buf.overflowed() || (blockBuf && blockBuf->overflowed())) {
        if (err)
            *err = "out of memory writing the .mdl (needed " +
                   std::to_string(size_t(buf.pos) / (1024 * 1024)) + " MB, limit " +
                   std::to_string(Buf::kReserve / (1024 * 1024)) + " MB)";
        return false;
    }

    std::vector<uint8_t> mdlBuf(buf.start(), buf.start() + total);

    using WClock = std::chrono::steady_clock;
    auto wt0 = WClock::now();
    auto wlog = [](const char* what, WClock::time_point a, WClock::time_point b) {
        if (pulse::perf::g_enabled)
            pulse::perf::Record("write", what,
                                std::chrono::duration<double, std::milli>(b - a).count());
    };

    // These stages are silent and slow on a big model, and the per-file reports
    // below only appear once they are all done - say what is running, and flush,
    // because stdout is fully buffered when a GUI front end pipes it.
    auto stage = [](const char* what) {
        std::printf("building %s...\n", what);
        std::fflush(stdout);
    };

    // the reference gates the vertex/strip write on numbodyparts != 0: an
    // animation-only model gets no .vvd/.vtx. Build them anyway for the fixup
    // pass, just don't save two empty files.
    const bool hasGeometry = !m.bodyparts.empty();

    // .vvd first pass
    if (hasGeometry) stage("vertex data (.vvd)");
    std::vector<uint8_t> vvdBuf =
        BuildVvd(m, reinterpret_cast<fmt::studiohdr_t*>(mdlBuf.data())->checksum);
    auto wtVvd = WClock::now();
    wlog("BuildVvd", wt0, wtVvd);

    // .vtx
    if (hasGeometry) stage("strip data (.vtx)");
    std::vector<uint8_t> vtxBuf = BuildVtx(m, mdlBuf, vvdBuf, legacyVtx);
    auto wtVtx = WClock::now();
    wlog("BuildVtx", wtVvd, wtVtx);

    // fixup pass (mutates all three buffers)
    if (hasGeometry) stage("vertex fixups");
    if (!FixupBuffers(m, mdlBuf, vvdBuf, vtxBuf, legacyVtx, err))
        return false;
    wlog("FixupBuffers", wtVtx, WClock::now());

    // .phy - empty when the model has no collision data
    if (!m.physSolids.empty()) stage("collision data (.phy)");
    std::vector<uint8_t> phyBuf =
        BuildPhy(m, reinterpret_cast<fmt::studiohdr_t*>(mdlBuf.data())->checksum);

    // save. Reference hardcodes a "models/" root in the disk path (not in the
    // header name), so $modelname stays prefix-free and the .mdl bytes match.
    std::filesystem::path base =
        (outDir.empty() ? std::filesystem::path(".") : std::filesystem::path(outDir)) / "models";
    std::filesystem::path stem = (base / m.outname).make_preferred();

    // reference console format: a "writing <path>:" line
    // per output file, then its stats. GUI front ends scrape these lines for
    // the full paths, so the separator must be native - $modelname carries
    // forward slashes that make_preferred() flips.
    auto announce = [&](const char* ext) {
        std::string path = stem.string() + ext;
        std::printf("---------------------\n");
        std::printf("writing %s:\n", path.c_str());
        std::fflush(stdout);
        return path;
    };

    std::string mdlPath = announce(".mdl");
    FlushReport(g_mdlReport);
    std::printf("total      %7zu\n", mdlBuf.size());
    if (!SaveFile(mdlPath, mdlBuf.data(), mdlBuf.size(), err)) return false;

    std::string vvdPath, vtxPath;
    if (hasGeometry) {
        vvdPath = announce(".vvd");
        FlushReport(g_vvdReport);
        if (!SaveFile(vvdPath, vvdBuf.data(), vvdBuf.size(), err)) return false;

        // which strip layout went out. SFM reads the Alien Swarm/CS:GO 35-byte
        // strips and crashes on the legacy ones, so make the format explicit.
        std::printf("VTX format: %s\n", legacyVtx ? "0 - TF2/L4D2/GMod/HL2 (legacy 27-byte strips)"
                                                  : "1 - Alien Swarm/CS:GO/SFM (35-byte strips)");
        vtxPath = announce(".dx90.vtx");
        FlushReport(g_vtxReport);
        std::printf("everything (%zu bytes)\n", vtxBuf.size());
        if (!SaveFile(vtxPath, vtxBuf.data(), vtxBuf.size(), err)) return false;
    }

    std::string aniPath, phyPath;

    if (blockBuf) {
        // the .ani sits beside the .mdl under the same "models/" root, and its
        // name is what szanimblocknameindex already recorded
        pblockhdr->length = static_cast<int32_t>(blockBuf->pos);
        aniPath = announce(".ani");
        std::printf("blocks     %7zu\n", g_animblocks.count ? g_animblocks.count - 1 : 0);
        std::printf("total      %7zu\n", blockBuf->cur);
        if (!SaveFile(aniPath, blockBuf->start(), blockBuf->pos, err))
            return false;
    }

    if (!phyBuf.empty()) {
        // the .phy filename may be overridden independently of the .mdl
        std::filesystem::path phyStem =
            m.physName.empty() ? stem : (base / m.physName).make_preferred();
        phyPath = phyStem.string() + ".phy";
        std::printf("---------------------\n");
        std::printf("writing %s:\n", phyPath.c_str());
        // one ragdollconstraint per parented body (writephy.cpp), so a
        // single-body prop reports 0 joints and a ragdoll reports solids-1
        size_t joints = 0;
        for (size_t i = 0; i < m.physSolids.size(); ++i) {
            const cm::PhysicsSolid& s = m.physSolids[i];
            if (s.parentIndex >= 0 && s.parentIndex != static_cast<int>(i))
                ++joints;
        }
        std::printf("solids     %7zu\n", m.physSolids.size());
        std::printf("joints     %7zu%s\n", joints, joints ? " (ragdoll)" : "");
        if (m.physNoSelfCollisions)
            std::printf("collisions %7s\n", "off");
        else if (!m.physCollisionPairs.empty())
            std::printf("pairs      %7zu\n", m.physCollisionPairs.size());
        std::printf("total      %7zu\n", phyBuf.size());
        if (!SaveFile(phyPath, phyBuf.data(), phyBuf.size(), err)) return false;
    }

    // footer: the shared output dir once, then each file by name with its
    // checksum (mstudiohdr_t::checksum, mdl.h - shared by .mdl/.vvd/
    // .dx90.vtx/.phy; the .ani header does not carry one) and size
    const uint32_t checksum =
        static_cast<uint32_t>(reinterpret_cast<fmt::studiohdr_t*>(mdlBuf.data())->checksum);
    auto name = [](const std::string& path) {
        return std::filesystem::path(path).filename().string();
    };
    std::printf("---------------------\n");
    std::printf("output directory: %s\n", stem.parent_path().string().c_str());
    std::printf("  %-20s 0x%08X  %10zu bytes\n", name(mdlPath).c_str(), checksum, mdlBuf.size());
    if (hasGeometry) {
        std::printf("  %-20s 0x%08X  %10zu bytes\n", name(vvdPath).c_str(), checksum,
                    vvdBuf.size());
        std::printf("  %-20s 0x%08X  %10zu bytes\n", name(vtxPath).c_str(), checksum,
                    vtxBuf.size());
    }
    if (!aniPath.empty())
        std::printf("  %-20s %-10s  %10zu bytes\n", name(aniPath).c_str(), "unchecksummed",
                    blockBuf->cur);
    if (!phyPath.empty())
        std::printf("  %-20s 0x%08X  %10zu bytes\n", name(phyPath).c_str(), checksum,
                    phyBuf.size());
    return true;
}

} // namespace pulse::writer
