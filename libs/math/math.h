// math.h - PulseMDL
//
// Compiler-side math: the value types and transform helpers the DMX->source
// mapper and the writers need. Kept separate from the POD value types in
// pulse::dmx - the mapper converts at the boundary.
//
// Conventions follow the Source engine's, so bone and animation data matches
// the .mdl format:
//   - Right-handed; quaternions are (x, y, z, w).
//   - matrix3x4 is row-major float[3][4]: the 3x3 rotation lives in columns
//     0..2, the translation in column 3 (matrix[row][3]).
//   - RadianEuler is (x=roll, y=pitch, z=yaw) in radians; QAngle-style degree
//     triples are (pitch, yaw, roll).
//
// NOTE: written from the standard definitions. Intermediates run in double and
// round to float once on store.

#ifndef PULSEMDL_MATH_H
#define PULSEMDL_MATH_H

#include <cmath>

namespace pulse::math {

struct Vector2 { float x = 0, y = 0; };
struct Vector3 { float x = 0, y = 0, z = 0; };
struct Vector4 { float x = 0, y = 0, z = 0, w = 0; };
struct Quaternion { float x = 0, y = 0, z = 0, w = 1; };

// RadianEuler on the wire: three floats, same layout as Vector3.
using RadianEuler = Vector3;

// Row-major 3x4 transform. Defaults to identity.
struct matrix3x4 {
    float m[3][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}};
};

// Degree<->radian conversion factors (float).
inline constexpr float kPiF = static_cast<float>(3.14159265358979323846);
inline constexpr float kRad2Deg = 180.0f / kPiF;
inline constexpr float kDeg2Rad = kPiF / 180.0f;

// Sine and cosine of one angle.
inline void SinCos(float radians, float* sine, float* cosine) {
    *sine = sinf(radians);
    *cosine = cosf(radians);
}

// Build a local-to-parent transform from a rotation quaternion + translation.
matrix3x4 QuaternionMatrix(const Quaternion& q, const Vector3& pos);

// out = a * b  (apply b, then a).
matrix3x4 ConcatTransforms(const matrix3x4& a, const matrix3x4& b);

// Transform a point (applies rotation + translation).
Vector3 VectorTransform(const Vector3& v, const matrix3x4& m);

// Rotate a direction (rotation only, no translation) - for normals/deltas.
Vector3 VectorRotate(const Vector3& v, const matrix3x4& m);

// Transform a point by the inverse of an orthonormal transform.
Vector3 VectorITransform(const Vector3& v, const matrix3x4& m);

// Rotate a direction by the inverse (transpose) of a rotation.
Vector3 VectorIRotate(const Vector3& v, const matrix3x4& m);

// Orthonormal inverse: transpose the rotation, re-express the translation.
matrix3x4 MatrixInvert(const matrix3x4& in);

// RadianEuler -> rotation matrix, optionally with a position.
void AngleMatrix(const RadianEuler& anglesRad, matrix3x4& out);
void AngleMatrix(const RadianEuler& anglesRad, const Vector3& pos, matrix3x4& out);
// QAngle degrees (pitch, yaw, roll) -> matrix.
void AngleMatrixDeg(const Vector3& anglesDeg, matrix3x4& out);
// Inverse (transposed) variants.
void AngleIMatrixDeg(const Vector3& anglesDeg, matrix3x4& out);
void AngleIMatrix(const RadianEuler& anglesRad, matrix3x4& out);

// Element-wise 3x4 compare, default tolerance 1e-5.
bool MatricesAreEqual(const matrix3x4& a, const matrix3x4& b, float tolerance = 1e-5f);

// matrix -> RadianEuler + position.
void MatrixAngles(const matrix3x4& m, RadianEuler& anglesRad, Vector3& pos);
// matrix -> quaternion + position.
void MatrixAngles(const matrix3x4& m, Quaternion& q, Vector3& pos);

// RadianEuler -> quaternion.
void AngleQuaternion(const RadianEuler& anglesRad, Quaternion& outQuat);

// Make q live in the same hemisphere as p (negate q if their dot is negative).
void QuaternionAlign(const Quaternion& p, const Quaternion& q, Quaternion& qt);

// Delta-animation primitives.
void QuaternionScale(const Quaternion& p, float t, Quaternion& q); // scale rotation angle
void QuaternionMult(const Quaternion& p, const Quaternion& q, Quaternion& qt); // q aligned to p
void QuaternionSM(float s, const Quaternion& p, const Quaternion& q, Quaternion& qt); // (s*p)*q
void QuaternionMA(const Quaternion& p, float s, const Quaternion& q, Quaternion& qt); // p*(s*q)
void QuaternionAngles(const Quaternion& q, RadianEuler& angles); // q -> euler via matrix
void QuaternionSMAngles(float s, const Quaternion& p, const Quaternion& q, RadianEuler& angles);
void QuaternionMAAngles(const Quaternion& p, float s, const Quaternion& q, RadianEuler& angles);

// Spherical interpolation. Slerp aligns hemispheres first; nearly-equal
// quaternions fall back to lerp. Used by DMX channel sampling.
void QuaternionSlerp(const Quaternion& p, const Quaternion& q, float t, Quaternion& qt);
void QuaternionSlerpNoAlign(const Quaternion& p, const Quaternion& q, float t, Quaternion& qt);

// Inverse transpose of the 3x3 part with no translation - the normal
// transform of a mesh dag's bind matrix. Returns the input if it is singular.
matrix3x4 MatrixInverseTranspose(const matrix3x4& in);

// In-place normalize, returns the pre-normalization length. A zero-length
// input is left unchanged.
float VectorNormalize(Vector3& v);
float QuaternionNormalize(Quaternion& q);

// --- display / loader helpers -----------------------------------------------

Vector3 Normalize(const Vector3& v);
Quaternion QuaternionNormalized(const Quaternion& q);

// Decompose a matrix's rotation to Valve Euler angles in DEGREES, returned as
// (pitch, yaw, roll). Used for $hbox angle offsets.
Vector3 MatrixAnglesDeg(const matrix3x4& m);

} // namespace pulse::math

#endif // PULSEMDL_MATH_H
