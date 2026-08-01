// compressed.h - PulseMDL
//
// Compressed on-disk value types used by the .mdl animation data (Vector48,
// Quaternion48, Quaternion64) plus the Valve float16 they build on. Bit-exact
// ports of the reference mathlib behavior: encode clamps/truncates exactly the
// same way (no rounding), infinity maps to maxfloat, NaN maps to zero.
//
// NOTE: do not "improve" the conversions (e.g. add rounding or use
// F16C intrinsics) - the writer depends on identical bits.

#ifndef PULSEMDL_MATH_COMPRESSED_H
#define PULSEMDL_MATH_COMPRESSED_H

#include <cstdint>
#include <cmath>
#include <cstring>

#include "math/math.h"

namespace pulse::math {

inline constexpr float kMaxFloat16Bits = 65504.0f;

// Valve float16: 1 sign / 5 exponent / 10 mantissa, truncating conversion.
class float16 {
public:
    uint16_t raw = 0;

    static uint16_t FromFloat(float input) {
        if (input > kMaxFloat16Bits)
            input = kMaxFloat16Bits;
        else if (input < -kMaxFloat16Bits)
            input = -kMaxFloat16Bits;

        uint32_t bits;
        std::memcpy(&bits, &input, 4);
        uint32_t inSign = bits >> 31;
        uint32_t inExp = (bits >> 23) & 0xFF;
        uint32_t inMant = bits & 0x7FFFFF;

        uint32_t outSign = inSign;
        uint32_t outExp = 0;
        uint32_t outMant = 0;

        if (inExp == 0) {
            // zero and denorm both map to zero
        } else if (inExp == 0xFF) {
            if (inMant == 0) {
                // infinity maps to maxfloat
                outMant = 0x3FF;
                outExp = 0x1E;
            } else {
                // NaN maps to zero
            }
        } else {
            int newExp = static_cast<int>(inExp) - 127;
            if (newExp < -24) {
                // maps to zero (mantissa/exponent stay 0)
            }
            if (newExp < -14) {
                // denorm
                outExp = 0;
                unsigned int expVal = static_cast<unsigned int>(-14 - newExp);
                if (expVal > 0 && expVal < 11)
                    outMant = (1u << (10 - expVal)) + (inMant >> (13 + expVal));
            } else if (newExp > 15) {
                // too big, maps to maxfloat
                outMant = 0x3FF;
                outExp = 0x1E;
            } else {
                outExp = static_cast<uint32_t>(newExp + 15);
                outMant = inMant >> 13;
            }
        }
        return static_cast<uint16_t>((outSign << 15) | (outExp << 10) | outMant);
    }

    static float ToFloat(uint16_t input) {
        uint32_t sign = (input >> 15) & 1;
        uint32_t exp = (input >> 10) & 0x1F;
        uint32_t mant = input & 0x3FF;

        if (exp == 31 && mant == 0) // infinity encoding
            return kMaxFloat16Bits * ((sign == 1) ? -1.0f : 1.0f);
        if (exp == 31) // NaN encoding
            return 0.0f;
        if (exp == 0 && mant != 0) {
            const float halfDenorm = 1.0f / 16384.0f; // 2^-14
            float m = static_cast<float>(mant) / 1024.0f;
            float sgn = sign ? -1.0f : 1.0f;
            return sgn * m * halfDenorm;
        }
        uint32_t outMant = mant << (23 - 10);
        uint32_t outExp = ((exp - 15 + 127) * (exp != 0 ? 1u : 0u)) << 23;
        uint32_t bits = outMant | outExp | (sign << 31);
        float out;
        std::memcpy(&out, &bits, 4);
        return out;
    }

    void SetFloat(float f) { raw = FromFloat(f); }
    float GetFloat() const { return ToFloat(raw); }
};
static_assert(sizeof(float16) == 2, "float16 layout");

// clamp helper matching the reference's int clamp
inline int ClampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Vector48 {
    float16 x, y, z;

    void Set(const Vector3& v) {
        x.SetFloat(v.x);
        y.SetFloat(v.y);
        z.SetFloat(v.z);
    }
    Vector3 Get() const { return {x.GetFloat(), y.GetFloat(), z.GetFloat()}; }
};
static_assert(sizeof(Vector48) == 6, "Vector48 layout");

// x:16 y:16 z:15 wneg:1
struct Quaternion48 {
    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t zwneg = 0;

    void Set(const Quaternion& q) {
        x = static_cast<uint16_t>(ClampInt(static_cast<int>(q.x * 32768.0f) + 32768, 0, 65535));
        y = static_cast<uint16_t>(ClampInt(static_cast<int>(q.y * 32768.0f) + 32768, 0, 65535));
        uint16_t z = static_cast<uint16_t>(ClampInt(static_cast<int>(q.z * 16384.0f) + 16384, 0, 32767));
        zwneg = static_cast<uint16_t>(z | ((q.w < 0) ? 0x8000 : 0));
    }
    Quaternion Get() const {
        Quaternion tmp;
        int z = zwneg & 0x7FFF;
        tmp.x = static_cast<float>((static_cast<int>(x) - 32768) * (1 / 32768.5));
        tmp.y = static_cast<float>((static_cast<int>(y) - 32768) * (1 / 32768.5));
        tmp.z = static_cast<float>((z - 16384) * (1 / 16384.5));
        tmp.w = static_cast<float>(sqrt(1.0 - tmp.x * tmp.x - tmp.y * tmp.y - tmp.z * tmp.z));
        if (zwneg & 0x8000) tmp.w = -tmp.w;
        return tmp;
    }
};
static_assert(sizeof(Quaternion48) == 6, "Quaternion48 layout");

// single uint64: x:21 y:21 z:21 wneg:1 (x in the low bits)
struct Quaternion64 {
    uint64_t bits = 0;

    void Set(const Quaternion& q) {
        uint64_t x = static_cast<uint64_t>(ClampInt(static_cast<int>(q.x * 1048576.0f) + 1048576, 0, 2097151));
        uint64_t y = static_cast<uint64_t>(ClampInt(static_cast<int>(q.y * 1048576.0f) + 1048576, 0, 2097151));
        uint64_t z = static_cast<uint64_t>(ClampInt(static_cast<int>(q.z * 1048576.0f) + 1048576, 0, 2097151));
        uint64_t wneg = (q.w < 0) ? 1u : 0u;
        bits = x | (y << 21) | (z << 42) | (wneg << 63);
    }
    Quaternion Get() const {
        Quaternion tmp;
        int x = static_cast<int>(bits & 0x1FFFFF);
        int y = static_cast<int>((bits >> 21) & 0x1FFFFF);
        int z = static_cast<int>((bits >> 42) & 0x1FFFFF);
        tmp.x = (x - 1048576) * (1 / 1048576.5f);
        tmp.y = (y - 1048576) * (1 / 1048576.5f);
        tmp.z = (z - 1048576) * (1 / 1048576.5f);
        tmp.w = static_cast<float>(sqrt(1.0 - tmp.x * tmp.x - tmp.y * tmp.y - tmp.z * tmp.z));
        if (bits >> 63) tmp.w = -tmp.w;
        return tmp;
    }
};
static_assert(sizeof(Quaternion64) == 8, "Quaternion64 layout");

// x:11 y:10 z:10 wneg:1, LSB first. Zero-frame rotation cache.
struct Quaternion32 {
    uint32_t bits = 0;

    void Set(const Quaternion& q) {
        uint32_t x = static_cast<uint32_t>(ClampInt(static_cast<int>(q.x * 1024) + 1024, 0, 2047));
        uint32_t y = static_cast<uint32_t>(ClampInt(static_cast<int>(q.y * 512) + 512, 0, 1023));
        uint32_t z = static_cast<uint32_t>(ClampInt(static_cast<int>(q.z * 512) + 512, 0, 1023));
        uint32_t wneg = (q.w < 0) ? 1u : 0u;
        bits = x | (y << 11) | (z << 21) | (wneg << 31);
    }
};
static_assert(sizeof(Quaternion32) == 4, "Quaternion32 layout");

// needs to fit 2*sqrt(0.5) into 15 bits
inline constexpr float kScale48S = 23168.0f;
inline constexpr int kShift48S = 16384;

// Shifted 48-bit quaternion: the three SMALLEST components are stored and the
// largest is rebuilt by sqrt at load, so the worst-case error is bounded.
// "offset" (2 bits, split across two words so the whole thing packs into 6
// bytes) says which component the triple starts at; `dneg` is the sign of the
// dropped one. Bitfields are LSB-first within each 16-bit word:
//   word0 = a | offsetH<<15, word1 = b | offsetL<<15, word2 = c | dneg<<15
struct Quaternion48S {
    uint16_t w0 = 0, w1 = 0, w2 = 0;

    void Set(const Quaternion& q) {
        const float* p = &q.x;

        // pick the largest component so the sqrt reconstructs it
        int i = 0;
        if (std::fabs(p[i]) < std::fabs(p[1])) i = 1;
        if (std::fabs(p[i]) < std::fabs(p[2])) i = 2;
        if (std::fabs(p[i]) < std::fabs(p[3])) i = 3;

        int offset = (i + 1) % 4; // so that "d" is the largest element
        const int hi = static_cast<int>(kScale48S * 2);
        int a = ClampInt(static_cast<int>(p[offset] * kScale48S) + kShift48S, 0, hi);
        int b = ClampInt(static_cast<int>(p[(offset + 1) % 4] * kScale48S) + kShift48S, 0, hi);
        int c = ClampInt(static_cast<int>(p[(offset + 2) % 4] * kScale48S) + kShift48S, 0, hi);
        int dneg = (p[(offset + 3) % 4] < 0.0f) ? 1 : 0;

        w0 = static_cast<uint16_t>((a & 0x7FFF) | ((offset > 1 ? 1 : 0) << 15));
        w1 = static_cast<uint16_t>((b & 0x7FFF) | ((offset & 1) << 15));
        w2 = static_cast<uint16_t>((c & 0x7FFF) | (dneg << 15));
    }

    // the two top bits carry `offset` split high/low, and the dropped component
    // is the largest one, so it comes back off the unit-length constraint
    Quaternion Get() const {
        const int offset = ((w0 >> 15) << 1) | (w1 >> 15);
        Quaternion q;
        float* p = &q.x;
        p[offset] = (static_cast<int>(w0 & 0x7FFF) - kShift48S) / kScale48S;
        p[(offset + 1) % 4] = (static_cast<int>(w1 & 0x7FFF) - kShift48S) / kScale48S;
        p[(offset + 2) % 4] = (static_cast<int>(w2 & 0x7FFF) - kShift48S) / kScale48S;
        const double rest = static_cast<double>(p[offset]) * p[offset] +
                            static_cast<double>(p[(offset + 1) % 4]) * p[(offset + 1) % 4] +
                            static_cast<double>(p[(offset + 2) % 4]) * p[(offset + 2) % 4];
        float d = static_cast<float>(sqrt(rest < 1.0 ? 1.0 - rest : 0.0));
        p[(offset + 3) % 4] = (w2 & 0x8000) ? -d : d;
        return q;
    }
};
static_assert(sizeof(Quaternion48S) == 6, "Quaternion48S layout");

} // namespace pulse::math

#endif // PULSEMDL_MATH_COMPRESSED_H
