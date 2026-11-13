// compressed.h - PulseMDL
//
// Compressed on-disk value types used by the .mdl animation data (Vector48,
// Quaternion48, Quaternion64, ...) plus the 16-bit float they build on.
//
// NOTE: encoders truncate toward zero and saturate; switching to rounding
// changes the bytes written to every animation.

#ifndef PULSEMDL_MATH_COMPRESSED_H
#define PULSEMDL_MATH_COMPRESSED_H

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <cstring>

#include "math/math.h"

namespace pulse::math {

inline constexpr float kMaxFloat16Bits = 65504.0f;

// IEEE 754 half: 1 sign / 5 exponent / 10 mantissa. Conversion truncates,
// infinity saturates to the largest finite value, NaN becomes zero.
class float16 {
public:
    uint16_t raw = 0;

    static uint16_t FromFloat(float input) {
        if (std::isnan(input))
            return 0;

        const uint16_t sign = std::signbit(input) ? 0x8000 : 0;
        const float mag = std::fmin(std::fabs(input), kMaxFloat16Bits);

        // Below 2^-14 the half is subnormal: a multiple of 2^-24.
        if (mag < 6.103515625e-5f)
            return static_cast<uint16_t>(sign | static_cast<uint16_t>(mag * 16777216.0f));

        uint32_t bits;
        std::memcpy(&bits, &mag, 4);

        const uint32_t exponent = (bits >> 23) - 127 + 15;
        const uint32_t mantissa = (bits >> 13) & 0x3FF;
        return static_cast<uint16_t>(sign | (exponent << 10) | mantissa);
    }

    static float ToFloat(uint16_t input) {
        const float sign = (input & 0x8000) ? -1.0f : 1.0f;
        const int exponent = (input >> 10) & 0x1F;
        const int mantissa = input & 0x3FF;

        if (exponent == 0x1F)
            return mantissa == 0 ? sign * kMaxFloat16Bits : 0.0f;

        if (exponent == 0)
            return sign * std::ldexp(static_cast<float>(mantissa), -24);

        return sign * std::ldexp(static_cast<float>(1024 + mantissa), exponent - 25);
    }

    void SetFloat(float f) { raw = FromFloat(f); }
    float GetFloat() const { return ToFloat(raw); }
};
static_assert(sizeof(float16) == 2, "float16 layout");

inline int ClampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// value * scale, truncated toward zero, offset by bias, clamped to [0, maxValue].
inline int Quantize(float value, float scale, int bias, int maxValue) {
    return ClampInt(static_cast<int>(value * scale) + bias, 0, maxValue);
}

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
        x = static_cast<uint16_t>(Quantize(q.x, 32768.0f, 32768, 65535));
        y = static_cast<uint16_t>(Quantize(q.y, 32768.0f, 32768, 65535));

        const uint16_t z = static_cast<uint16_t>(Quantize(q.z, 16384.0f, 16384, 32767));
        zwneg = static_cast<uint16_t>(z | ((q.w < 0) ? 0x8000 : 0));
    }

    // Components are biased integers over a step of range + 0.5; w is rebuilt from unit length.
    Quaternion Get() const {
        const double cx = (static_cast<int>(x) - 32768) * (1 / 32768.5);
        const double cy = (static_cast<int>(y) - 32768) * (1 / 32768.5);
        const double cz = ((zwneg & 0x7FFF) - 16384) * (1 / 16384.5);
        const double w = std::sqrt(std::max(0.0, 1.0 - cx * cx - cy * cy - cz * cz));

        return {static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(cz),
                static_cast<float>((zwneg & 0x8000) ? -w : w)};
    }
};
static_assert(sizeof(Quaternion48) == 6, "Quaternion48 layout");

// single uint64: x:21 y:21 z:21 wneg:1 (x in the low bits)
struct Quaternion64 {
    uint64_t bits = 0;

    void Set(const Quaternion& q) {
        const uint64_t x = static_cast<uint64_t>(Quantize(q.x, 1048576.0f, 1048576, 2097151));
        const uint64_t y = static_cast<uint64_t>(Quantize(q.y, 1048576.0f, 1048576, 2097151));
        const uint64_t z = static_cast<uint64_t>(Quantize(q.z, 1048576.0f, 1048576, 2097151));
        const uint64_t wneg = (q.w < 0) ? 1u : 0u;

        bits = x | (y << 21) | (z << 42) | (wneg << 63);
    }

    Quaternion Get() const {
        Quaternion tmp;
        const int x = static_cast<int>(bits & 0x1FFFFF);
        const int y = static_cast<int>((bits >> 21) & 0x1FFFFF);
        const int z = static_cast<int>((bits >> 42) & 0x1FFFFF);

        tmp.x = (x - 1048576) * (1 / 1048576.5f);
        tmp.y = (y - 1048576) * (1 / 1048576.5f);
        tmp.z = (z - 1048576) * (1 / 1048576.5f);
        tmp.w = static_cast<float>(std::sqrt(1.0 - tmp.x * tmp.x - tmp.y * tmp.y - tmp.z * tmp.z));

        if (bits >> 63)
            tmp.w = -tmp.w;
        return tmp;
    }
};
static_assert(sizeof(Quaternion64) == 8, "Quaternion64 layout");

// x:11 y:10 z:10 wneg:1, LSB first. Zero-frame rotation cache.
struct Quaternion32 {
    uint32_t bits = 0;

    void Set(const Quaternion& q) {
        const uint32_t x = static_cast<uint32_t>(Quantize(q.x, 1024.0f, 1024, 2047));
        const uint32_t y = static_cast<uint32_t>(Quantize(q.y, 512.0f, 512, 1023));
        const uint32_t z = static_cast<uint32_t>(Quantize(q.z, 512.0f, 512, 1023));
        const uint32_t wneg = (q.w < 0) ? 1u : 0u;

        bits = x | (y << 11) | (z << 21) | (wneg << 31);
    }
};
static_assert(sizeof(Quaternion32) == 4, "Quaternion32 layout");

// The three stored components are each at most sqrt(0.5) in magnitude.
inline constexpr float kScale48S = 23168.0f;
inline constexpr int kShift48S = 16384;

// Shifted 48-bit quaternion: stores the three components that follow the
// largest one (cyclically) and rebuilds the largest from unit length on read.
//   word0 = a | offsetHigh<<15, word1 = b | offsetLow<<15, word2 = c | sign<<15
// "offset" is the index of the first stored component; the sign belongs to
// the dropped component.
struct Quaternion48S {
    uint16_t w0 = 0, w1 = 0, w2 = 0;

    void Set(const Quaternion& q) {
        const float c[4] = {q.x, q.y, q.z, q.w};

        int largest = 0;
        for (int i = 1; i < 4; ++i)
            if (std::fabs(c[i]) > std::fabs(c[largest]))
                largest = i;

        const int offset = (largest + 1) % 4;
        auto stored = [&](int k) {
            return Quantize(c[(offset + k) % 4], kScale48S, kShift48S, 0x7FFF);
        };
        const int dneg = (c[largest] < 0.0f) ? 1 : 0;

        w0 = static_cast<uint16_t>(stored(0) | ((offset >> 1) << 15));
        w1 = static_cast<uint16_t>(stored(1) | ((offset & 1) << 15));
        w2 = static_cast<uint16_t>(stored(2) | (dneg << 15));
    }

    Quaternion Get() const {
        const int offset = ((w0 >> 15) << 1) | (w1 >> 15);
        const uint16_t words[3] = {w0, w1, w2};

        float c[4] = {};
        double sumSquares = 0.0;
        for (int k = 0; k < 3; ++k) {
            const float v = (static_cast<int>(words[k] & 0x7FFF) - kShift48S) / kScale48S;

            c[(offset + k) % 4] = v;
            sumSquares += static_cast<double>(v) * v;
        }

        const float largest = static_cast<float>(std::sqrt(std::max(0.0, 1.0 - sumSquares)));
        c[(offset + 3) % 4] = (w2 & 0x8000) ? -largest : largest;
        return {c[0], c[1], c[2], c[3]};
    }
};
static_assert(sizeof(Quaternion48S) == 6, "Quaternion48S layout");

} // namespace pulse::math

#endif // PULSEMDL_MATH_COMPRESSED_H
