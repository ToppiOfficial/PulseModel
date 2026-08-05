// math.cpp - transform helpers. See math.h for the conventions and the
// NOTE: operation order / float-double promotion here mirrors the
// reference mathlib exactly; do not simplify.

#include "math/math.h"

#include <cfloat>

namespace pulse::math {

matrix3x4 QuaternionMatrix(const Quaternion& q, const Vector3& pos) {
    matrix3x4 r;
    // Reference QuaternionMatrix: the FIRST COLUMN is computed with double
    // constants (double math, truncated on store); the rest in float.
    r.m[0][0] = static_cast<float>(1.0 - 2.0 * q.y * q.y - 2.0 * q.z * q.z);
    r.m[1][0] = static_cast<float>(2.0 * q.x * q.y + 2.0 * q.w * q.z);
    r.m[2][0] = static_cast<float>(2.0 * q.x * q.z - 2.0 * q.w * q.y);

    r.m[0][1] = 2.0f * q.x * q.y - 2.0f * q.w * q.z;
    r.m[1][1] = 1.0f - 2.0f * q.x * q.x - 2.0f * q.z * q.z;
    r.m[2][1] = 2.0f * q.y * q.z + 2.0f * q.w * q.x;

    r.m[0][2] = 2.0f * q.x * q.z + 2.0f * q.w * q.y;
    r.m[1][2] = 2.0f * q.y * q.z - 2.0f * q.w * q.x;
    r.m[2][2] = 1.0f - 2.0f * q.x * q.x - 2.0f * q.y * q.y;

    r.m[0][3] = pos.x;
    r.m[1][3] = pos.y;
    r.m[2][3] = pos.z;
    return r;
}

matrix3x4 ConcatTransforms(const matrix3x4& a, const matrix3x4& b) {
    // Reference ConcatTransforms is SIMD mul/add with RIGHT-associated sums:
    // out = m0 + (m1 + m2), translation added last as a separate step.
    matrix3x4 r;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            r.m[i][j] = a.m[i][0] * b.m[0][j] +
                        (a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j]);
        }
        r.m[i][3] = r.m[i][3] + a.m[i][3];
    }
    return r;
}

Vector3 VectorTransform(const Vector3& v, const matrix3x4& m) {
    // DotProduct(in1, in2[row]) + in2[row][3]
    return {
        (v.x * m.m[0][0] + v.y * m.m[0][1] + v.z * m.m[0][2]) + m.m[0][3],
        (v.x * m.m[1][0] + v.y * m.m[1][1] + v.z * m.m[1][2]) + m.m[1][3],
        (v.x * m.m[2][0] + v.y * m.m[2][1] + v.z * m.m[2][2]) + m.m[2][3],
    };
}

Vector3 VectorRotate(const Vector3& v, const matrix3x4& m) {
    return {
        v.x * m.m[0][0] + v.y * m.m[0][1] + v.z * m.m[0][2],
        v.x * m.m[1][0] + v.y * m.m[1][1] + v.z * m.m[1][2],
        v.x * m.m[2][0] + v.y * m.m[2][1] + v.z * m.m[2][2],
    };
}

Vector3 VectorITransform(const Vector3& v, const matrix3x4& m) {
    float t0 = v.x - m.m[0][3];
    float t1 = v.y - m.m[1][3];
    float t2 = v.z - m.m[2][3];
    return {
        t0 * m.m[0][0] + t1 * m.m[1][0] + t2 * m.m[2][0],
        t0 * m.m[0][1] + t1 * m.m[1][1] + t2 * m.m[2][1],
        t0 * m.m[0][2] + t1 * m.m[1][2] + t2 * m.m[2][2],
    };
}

Vector3 VectorIRotate(const Vector3& v, const matrix3x4& m) {
    return {
        v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0],
        v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1],
        v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2],
    };
}

matrix3x4 MatrixInvert(const matrix3x4& in) {
    matrix3x4 out;
    out.m[0][0] = in.m[0][0];
    out.m[0][1] = in.m[1][0];
    out.m[0][2] = in.m[2][0];
    out.m[1][0] = in.m[0][1];
    out.m[1][1] = in.m[1][1];
    out.m[1][2] = in.m[2][1];
    out.m[2][0] = in.m[0][2];
    out.m[2][1] = in.m[1][2];
    out.m[2][2] = in.m[2][2];

    float tmp[3] = {in.m[0][3], in.m[1][3], in.m[2][3]};
    out.m[0][3] = -(tmp[0] * out.m[0][0] + tmp[1] * out.m[0][1] + tmp[2] * out.m[0][2]);
    out.m[1][3] = -(tmp[0] * out.m[1][0] + tmp[1] * out.m[1][1] + tmp[2] * out.m[1][2]);
    out.m[2][3] = -(tmp[0] * out.m[2][0] + tmp[1] * out.m[2][1] + tmp[2] * out.m[2][2]);
    return out;
}

void AngleMatrixDeg(const Vector3& anglesDeg, matrix3x4& matrix) {
    float sr, sp, sy, cr, cp, cy;
    SinCos(anglesDeg.y * kDeg2Rad, &sy, &cy); // yaw
    SinCos(anglesDeg.x * kDeg2Rad, &sp, &cp); // pitch
    SinCos(anglesDeg.z * kDeg2Rad, &sr, &cr); // roll

    // matrix = (YAW * PITCH) * ROLL
    matrix.m[0][0] = cp * cy;
    matrix.m[1][0] = cp * sy;
    matrix.m[2][0] = -sp;

    matrix.m[0][1] = sr * sp * cy + cr * -sy;
    matrix.m[1][1] = sr * sp * sy + cr * cy;
    matrix.m[2][1] = sr * cp;
    matrix.m[0][2] = (cr * sp * cy + -sr * -sy);
    matrix.m[1][2] = (cr * sp * sy + -sr * cy);
    matrix.m[2][2] = cr * cp;

    matrix.m[0][3] = 0.0f;
    matrix.m[1][3] = 0.0f;
    matrix.m[2][3] = 0.0f;
}

void AngleMatrix(const RadianEuler& anglesRad, matrix3x4& out) {
    // Reference converts RadianEuler -> QAngle degrees -> back to radians
    // inside the degree AngleMatrix. The float round-trip is intentional.
    Vector3 quakeEulerDeg{anglesRad.y * kRad2Deg, anglesRad.z * kRad2Deg,
                          anglesRad.x * kRad2Deg};
    AngleMatrixDeg(quakeEulerDeg, out);
}

void AngleIMatrixDeg(const Vector3& anglesDeg, matrix3x4& matrix) {
    float sr, sp, sy, cr, cp, cy;
    SinCos(anglesDeg.y * kDeg2Rad, &sy, &cy); // yaw
    SinCos(anglesDeg.x * kDeg2Rad, &sp, &cp); // pitch
    SinCos(anglesDeg.z * kDeg2Rad, &sr, &cr); // roll

    // matrix = (YAW * PITCH) * ROLL, transposed (reference AngleIMatrix)
    matrix.m[0][0] = cp * cy;
    matrix.m[0][1] = cp * sy;
    matrix.m[0][2] = -sp;
    matrix.m[1][0] = sr * sp * cy + cr * -sy;
    matrix.m[1][1] = sr * sp * sy + cr * cy;
    matrix.m[1][2] = sr * cp;
    matrix.m[2][0] = (cr * sp * cy + -sr * -sy);
    matrix.m[2][1] = (cr * sp * sy + -sr * cy);
    matrix.m[2][2] = cr * cp;

    matrix.m[0][3] = 0.0f;
    matrix.m[1][3] = 0.0f;
    matrix.m[2][3] = 0.0f;
}

void AngleIMatrix(const RadianEuler& anglesRad, matrix3x4& out) {
    // Same degree round-trip as AngleMatrix (reference mathlib_base.cpp).
    Vector3 quakeEulerDeg{anglesRad.y * kRad2Deg, anglesRad.z * kRad2Deg,
                          anglesRad.x * kRad2Deg};
    AngleIMatrixDeg(quakeEulerDeg, out);
}

bool MatricesAreEqual(const matrix3x4& a, const matrix3x4& b, float tolerance) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (fabs(a.m[i][j] - b.m[i][j]) > tolerance)
                return false;
        }
    }
    return true;
}

void AngleMatrix(const RadianEuler& anglesRad, const Vector3& pos, matrix3x4& out) {
    AngleMatrix(anglesRad, out);
    out.m[0][3] = pos.x;
    out.m[1][3] = pos.y;
    out.m[2][3] = pos.z;
}

// matrix -> (pitch, yaw, roll) degrees; the shared core of the angle paths.
static void MatrixAnglesDegRaw(const matrix3x4& matrix, float angles[3]) {
    float forward[3];
    float left[3];
    float up2;

    forward[0] = matrix.m[0][0];
    forward[1] = matrix.m[1][0];
    forward[2] = matrix.m[2][0];
    left[0] = matrix.m[0][1];
    left[1] = matrix.m[1][1];
    left[2] = matrix.m[2][1];
    up2 = matrix.m[2][2];

    float xyDist = sqrtf(forward[0] * forward[0] + forward[1] * forward[1]);

    if (xyDist > 0.001f) {
        angles[1] = kRad2Deg * atan2f(forward[1], forward[0]); // yaw
        angles[0] = kRad2Deg * atan2f(-forward[2], xyDist);    // pitch
        angles[2] = kRad2Deg * atan2f(left[2], up2);           // roll
    } else {
        angles[1] = kRad2Deg * atan2f(-left[0], left[1]);
        angles[0] = kRad2Deg * atan2f(-forward[2], xyDist);
        angles[2] = 0;
    }
}

void MatrixAngles(const matrix3x4& m, RadianEuler& anglesRad, Vector3& pos) {
    float deg[3];
    MatrixAnglesDegRaw(m, deg);
    // RadianEuler init from degrees is permuted: (roll, pitch, yaw).
    anglesRad.x = deg[2] * kDeg2Rad;
    anglesRad.y = deg[0] * kDeg2Rad;
    anglesRad.z = deg[1] * kDeg2Rad;
    pos = {m.m[0][3], m.m[1][3], m.m[2][3]};
}

void MatrixAngles(const matrix3x4& m, Quaternion& q, Vector3& pos) {
    float trace;
    trace = m.m[0][0] + m.m[1][1] + m.m[2][2] + 1.0f;
    if (trace > 1.0f + FLT_EPSILON) {
        q.x = (m.m[2][1] - m.m[1][2]);
        q.y = (m.m[0][2] - m.m[2][0]);
        q.z = (m.m[1][0] - m.m[0][1]);
        q.w = trace;
    } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
        trace = 1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2];
        q.x = trace;
        q.y = (m.m[1][0] + m.m[0][1]);
        q.z = (m.m[0][2] + m.m[2][0]);
        q.w = (m.m[2][1] - m.m[1][2]);
    } else if (m.m[1][1] > m.m[2][2]) {
        trace = 1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2];
        q.x = (m.m[0][1] + m.m[1][0]);
        q.y = trace;
        q.z = (m.m[2][1] + m.m[1][2]);
        q.w = (m.m[0][2] - m.m[2][0]);
    } else {
        trace = 1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1];
        q.x = (m.m[0][2] + m.m[2][0]);
        q.y = (m.m[2][1] + m.m[1][2]);
        q.z = trace;
        q.w = (m.m[1][0] - m.m[0][1]);
    }
    QuaternionNormalize(q);
    pos = {m.m[0][3], m.m[1][3], m.m[2][3]};
}

void AngleQuaternion(const RadianEuler& anglesRad, Quaternion& outQuat) {
    float sr, sp, sy, cr, cp, cy;
    SinCos(anglesRad.z * 0.5f, &sy, &cy);
    SinCos(anglesRad.y * 0.5f, &sp, &cp);
    SinCos(anglesRad.x * 0.5f, &sr, &cr);

    float sinRollCosPitch = sr * cp, cosRollSinPitch = cr * sp;
    outQuat.x = sinRollCosPitch * cy - cosRollSinPitch * sy; // X
    outQuat.y = cosRollSinPitch * cy + sinRollCosPitch * sy; // Y

    float cosRollCosPitch = cr * cp, sinRollSinPitch = sr * sp;
    outQuat.z = cosRollCosPitch * sy - sinRollSinPitch * cy; // Z
    outQuat.w = cosRollCosPitch * cy + sinRollSinPitch * sy; // W
}

// QuaternionScale (reference mathlib_base.cpp): scale a rotation's angle
// by t. sqrt/sin/asin run in double and truncate on store, as the reference's
// implicit promotions do.
void QuaternionScale(const Quaternion& p, float t, Quaternion& q) {
    float vecMag = static_cast<float>(sqrt(
        static_cast<double>(p.x * p.x + p.y * p.y + p.z * p.z)));
    vecMag = vecMag < 1.0f ? vecMag : 1.0f;

    float scaledSin = static_cast<float>(sin(asin(static_cast<double>(vecMag)) *
                                          static_cast<double>(t)));

    t = scaledSin / (vecMag + FLT_EPSILON);
    q.x = p.x * t;
    q.y = p.y * t;
    q.z = p.z * t;

    // recompute w so the result stays a unit quaternion
    float w2 = 1.0f - scaledSin * scaledSin;
    if (w2 < 0.0f)
        w2 = 0.0f;
    w2 = static_cast<float>(sqrt(static_cast<double>(w2)));

    // stay on the same hemisphere as the input
    q.w = (p.w < 0) ? -w2 : w2;
}

// QuaternionMult (mathlib_base.cpp): qt = p * q with hemisphere align.
void QuaternionMult(const Quaternion& p, const Quaternion& q, Quaternion& qt) {
    if (&p == &qt) {
        Quaternion p2 = p;
        QuaternionMult(p2, q, qt);
        return;
    }

    Quaternion q2;
    QuaternionAlign(p, q, q2);

    qt.x = p.x * q2.w + p.y * q2.z - p.z * q2.y + p.w * q2.x;
    qt.y = -p.x * q2.z + p.y * q2.w + p.z * q2.x + p.w * q2.y;
    qt.z = p.x * q2.y - p.y * q2.x + p.z * q2.w + p.w * q2.z;
    qt.w = -p.x * q2.x - p.y * q2.y - p.z * q2.z + p.w * q2.w;
}

// QuaternionSM: qt = normalize((s * p) * q)
void QuaternionSM(float s, const Quaternion& p, const Quaternion& q, Quaternion& qt) {
    Quaternion p1, q1;
    QuaternionScale(p, s, p1);
    QuaternionMult(p1, q, q1);
    QuaternionNormalize(q1);
    qt = q1;
}

// QuaternionMA: qt = normalize(p * (s * q))
void QuaternionMA(const Quaternion& p, float s, const Quaternion& q, Quaternion& qt) {
    Quaternion p1, q1;
    QuaternionScale(q, s, q1);
    QuaternionMult(p, q1, p1);
    QuaternionNormalize(p1);
    qt = p1;
}

// QuaternionAngles (mathlib_base.cpp): via the matrix round-trip.
void QuaternionAngles(const Quaternion& q, RadianEuler& angles) {
    matrix3x4 matrix = QuaternionMatrix(q, Vector3{0, 0, 0});
    Vector3 dummyPos;
    MatrixAngles(matrix, angles, dummyPos);
}

// Reference wrappers used by delta subtraction.
void QuaternionSMAngles(float s, const Quaternion& p, const Quaternion& q, RadianEuler& angles) {
    Quaternion qt;
    QuaternionSM(s, p, q, qt);
    QuaternionAngles(qt, angles);
}

void QuaternionMAAngles(const Quaternion& p, float s, const Quaternion& q, RadianEuler& angles) {
    Quaternion qt;
    QuaternionMA(p, s, q, qt);
    QuaternionAngles(qt, angles);
}

void QuaternionAlign(const Quaternion& p, const Quaternion& q, Quaternion& qt) {
    const float* pComp = &p.x;
    const float* qComp = &q.x;
    float* outComp = &qt.x;

    // shorter path wins: compare the "same sign" vs "flipped sign" distance
    float distSame = 0;
    float distFlip = 0;
    for (int i = 0; i < 4; i++) {
        distSame += (pComp[i] - qComp[i]) * (pComp[i] - qComp[i]);
        distFlip += (pComp[i] + qComp[i]) * (pComp[i] + qComp[i]);
    }
    if (distSame > distFlip) {
        for (int i = 0; i < 4; i++)
            outComp[i] = -qComp[i];
    } else if (&qt != &q) {
        for (int i = 0; i < 4; i++)
            outComp[i] = qComp[i];
    }
}

void QuaternionSlerpNoAlign(const Quaternion& p, const Quaternion& q, float t, Quaternion& qt) {
    const float* pComp = &p.x;
    const float* qComp = &q.x;
    float* outComp = &qt.x;
    float angle, cosAngle, sinAngle, scaleP, scaleQ;
    int i;

    // t=0 -> p, t=1 -> q
    cosAngle = pComp[0] * qComp[0] + pComp[1] * qComp[1] + pComp[2] * qComp[2] + pComp[3] * qComp[3];

    if ((1.0f + cosAngle) > 0.000001f) {
        if ((1.0f - cosAngle) > 0.000001f) {
            // note: double-precision acos/sin, truncated on store - as the reference does
            angle = static_cast<float>(acos(static_cast<double>(cosAngle)));
            sinAngle = static_cast<float>(sin(static_cast<double>(angle)));
            scaleP = static_cast<float>(sin(static_cast<double>((1.0f - t) * angle))) / sinAngle;
            scaleQ = static_cast<float>(sin(static_cast<double>(t * angle))) / sinAngle;
        } else {
            scaleP = 1.0f - t;
            scaleQ = t;
        }
        for (i = 0; i < 4; i++) {
            outComp[i] = scaleP * pComp[i] + scaleQ * qComp[i];
        }
    } else {
        outComp[0] = -qComp[1];
        outComp[1] = qComp[0];
        outComp[2] = -qComp[3];
        outComp[3] = qComp[2];
        scaleP = static_cast<float>(sin(static_cast<double>((1.0f - t) * (0.5f * kPiF))));
        scaleQ = static_cast<float>(sin(static_cast<double>(t * (0.5f * kPiF))));
        for (i = 0; i < 3; i++) {
            outComp[i] = scaleP * pComp[i] + scaleQ * outComp[i];
        }
    }
}

void QuaternionSlerp(const Quaternion& p, const Quaternion& q, float t, Quaternion& qt) {
    Quaternion q2;
    // pick whichever sign of q gives the shorter interpolation path
    QuaternionAlign(p, q, q2);
    QuaternionSlerpNoAlign(p, q2, t, qt);
}

matrix3x4 MatrixInverseTranspose(const matrix3x4& in) {
    // Embed as 4x4 (bottom row 0 0 0 1), run the reference's Gaussian
    // elimination with partial pivoting, then transpose - exactly
    // MatrixInverseGeneral + MatrixTranspose from the reference vmatrix.
    float src[4][4] = {
        {in.m[0][0], in.m[0][1], in.m[0][2], in.m[0][3]},
        {in.m[1][0], in.m[1][1], in.m[1][2], in.m[1][3]},
        {in.m[2][0], in.m[2][1], in.m[2][2], in.m[2][3]},
        {0.0f, 0.0f, 0.0f, 1.0f},
    };

    float mat[4][8];
    int rowMap[4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++)
            mat[i][j] = src[i][j];
        mat[i][4] = 0.0f;
        mat[i][5] = 0.0f;
        mat[i][6] = 0.0f;
        mat[i][7] = 0.0f;
        mat[i][i + 4] = 1.0f;
        rowMap[i] = i;
    }

    for (int col = 0; col < 4; col++) {
        // pick the largest-magnitude candidate in this column as pivot
        float bestMag = 1e-6f;
        int bestRow = -1;
        for (int cand = col; cand < 4; cand++) {
            float mag = fabsf(mat[rowMap[cand]][col]);
            if (mag > bestMag) {
                bestRow = cand;
                bestMag = mag;
            }
        }
        if (bestRow == -1)
            return in; // singular; reference returns false and leaves dst - callers never hit this

        int swapTmp = rowMap[bestRow];
        rowMap[bestRow] = rowMap[col];
        rowMap[col] = swapTmp;

        float* pivot = mat[rowMap[col]];
        float invPivot = 1.0f / pivot[col];
        for (int j = 0; j < 8; j++)
            pivot[j] *= invPivot;
        pivot[col] = 1.0f; // force exact 1 despite the division's rounding

        for (int i = 0; i < 4; i++) {
            if (i == col)
                continue;
            float* target = mat[rowMap[i]];
            float factor = -target[col];
            for (int j = 0; j < 8; j++)
                target[j] += pivot[j] * factor;
            target[col] = 0.0f; // force exact 0 despite the fma's rounding
        }
    }

    // inverse is on the right side; transpose while extracting the 3x4
    float inv[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            inv[i][j] = mat[rowMap[i]][j + 4];

    matrix3x4 out;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            out.m[i][j] = inv[j][i]; // transpose
    return out;
}

float VectorNormalize(Vector3& v) {
    float radius = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    // FLT_EPSILON added to eliminate the possibility of divide by zero.
    float iradius = 1.f / (radius + FLT_EPSILON);
    v.x *= iradius;
    v.y *= iradius;
    v.z *= iradius;
    return radius;
}

float QuaternionNormalize(Quaternion& q) {
    float radius, iradius;
    radius = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (radius) {
        // double-precision sqrt then truncate, as the reference does
        radius = static_cast<float>(sqrt(static_cast<double>(radius)));
        iradius = 1.0f / radius;
        q.w *= iradius;
        q.z *= iradius;
        q.y *= iradius;
        q.x *= iradius;
    }
    return radius;
}

// --- non-parity helpers ------------------------------------------------------

Vector3 Normalize(const Vector3& v) {
    float len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (len <= 0.0f) return {0, 0, 0};
    float inv = 1.0f / len;
    return {v.x * inv, v.y * inv, v.z * inv};
}

Quaternion QuaternionNormalized(const Quaternion& q) {
    Quaternion r = q;
    QuaternionNormalize(r);
    if (r.x == 0 && r.y == 0 && r.z == 0 && r.w == 0) return {0, 0, 0, 1};
    return r;
}

Vector3 MatrixAnglesDeg(const matrix3x4& m) {
    float deg[3];
    MatrixAnglesDegRaw(m, deg);
    return {deg[0], deg[1], deg[2]};
}

} // namespace pulse::math
