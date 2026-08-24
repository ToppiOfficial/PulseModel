#include "convex_decompose.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <vector>

#include "vhacd/vhacd_hull.h"

namespace pulse::tool {
namespace {

constexpr std::uint32_t kMagic = 0x44435750u; // "PWCD"
constexpr std::uint32_t kVersion = 2u;

struct Reader {
    const std::uint8_t* at;
    const std::uint8_t* end;

    bool U32(std::uint32_t& out) {
        if (end - at < 4)
            return false;
        std::memcpy(&out, at, 4);
        at += 4;
        return true;
    }

    bool F32(float& out) {
        if (end - at < 4)
            return false;
        std::memcpy(&out, at, 4);
        at += 4;
        return true;
    }

    // A view into the buffer rather than a copy - a character's shape is
    // megabytes of positions and indices.
    const void* Block(std::size_t bytes) {
        if (static_cast<std::size_t>(end - at) < bytes)
            return nullptr;
        const void* p = at;
        at += bytes;
        return p;
    }
};

void Put(std::vector<std::uint8_t>& out, const void* data, std::size_t bytes) {
    const auto* p = static_cast<const std::uint8_t*>(data);
    out.insert(out.end(), p, p + bytes);
}

void PutU32(std::vector<std::uint8_t>& out, std::uint32_t value) {
    Put(out, &value, 4);
}

} // namespace

bool RunDecompose(const std::string& requestPath, const std::string& responsePath,
                  std::string* err) {
    auto fail = [&](const char* message) {
        if (err)
            *err = message;
        return false;
    };

    std::ifstream in(requestPath, std::ios::binary | std::ios::ate);
    if (!in)
        return fail("could not open the request file");
    const std::streamoff size = in.tellg();
    if (size < 12)
        return fail("the request file is too short");
    std::vector<std::uint8_t> buffer(static_cast<std::size_t>(size));
    in.seekg(0);
    in.read(reinterpret_cast<char*>(buffer.data()), size);
    if (!in)
        return fail("could not read the request file");

    Reader r{buffer.data(), buffer.data() + buffer.size()};
    std::uint32_t magic = 0, version = 0, jobCount = 0;
    if (!r.U32(magic) || !r.U32(version) || !r.U32(jobCount))
        return fail("the request file is truncated");
    if (magic != kMagic || version != kVersion)
        return fail("the request file is not a decompose request");

    std::vector<std::uint8_t> out;
    PutU32(out, kMagic);
    PutU32(out, jobCount);

    for (std::uint32_t job = 0; job < jobCount; job++) {
        std::uint32_t vertCount = 0;
        if (!r.U32(vertCount))
            return fail("a job is truncated");
        const auto* verts = static_cast<const float*>(
            r.Block(static_cast<std::size_t>(vertCount) * 3 * sizeof(float)));
        std::uint32_t triCount = 0;
        if (!verts || !r.U32(triCount))
            return fail("a job is truncated");
        const auto* tris = static_cast<const std::int32_t*>(
            r.Block(static_cast<std::size_t>(triCount) * 3 * sizeof(std::int32_t)));
        float concavity = 0.0f, decimate = 0.0f;
        std::uint32_t maxHulls = 0, resolution = 0;
        if (!tris || !r.F32(concavity) || !r.U32(maxHulls) || !r.F32(decimate) ||
            !r.U32(resolution))
            return fail("a job is truncated");

        std::vector<VHACDHull::DecomposedHull> hulls;
        VHACDHull::DecomposeConvex(verts, static_cast<int>(vertCount), tris,
                                   static_cast<int>(triCount), concavity,
                                   static_cast<int>(maxHulls), decimate, hulls,
                                   static_cast<int>(resolution));

        // A job that decomposes to nothing answers with no pieces rather than
        // failing the batch - the caller falls back to one hull for that body.
        PutU32(out, static_cast<std::uint32_t>(hulls.size()));
        for (const VHACDHull::DecomposedHull& hull : hulls) {
            const auto points = static_cast<std::uint32_t>(hull.vertices.size() / 3);
            PutU32(out, points);
            Put(out, hull.vertices.data(), points * 3 * sizeof(float));
        }
    }

    std::ofstream answer(responsePath, std::ios::binary | std::ios::trunc);
    if (!answer)
        return fail("could not open the response file");
    answer.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    if (!answer)
        return fail("could not write the response file");
    return true;
}

} // namespace pulse::tool
