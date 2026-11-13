// math.cpp - transform and rotation helpers. Conventions are in math.h.
// Written from the standard definitions; intermediates run in double and
// round to float once on store.

#include "math/math.h"

#include <algorithm>

namespace pulse::math {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

// |cos(pitch)| below this is treated as gimbal lock.
constexpr double kGimbalCos = 1e-4;

struct Mat3 {
    double m[3][3] = {};
};

struct Quat4 {
    double x = 0, y = 0, z = 0, w = 1;
};

Mat3 Multiply(const Mat3& a, const Mat3& b) {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                r.m[i][j] += a.m[i][k] * b.m[k][j];
    return r;
}

Mat3 Transposed(const Mat3& a) {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = a.m[j][i];
    return r;
}

// Right-handed rotations about one axis.
Mat3 RotateX(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return Mat3{{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
}

Mat3 RotateY(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return Mat3{{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
}

Mat3 RotateZ(double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    return Mat3{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

// Source angle order: yaw about Z, then pitch about Y, then roll about X.
Mat3 EulerToMat(double roll, double pitch, double yaw) {
    return Multiply(Multiply(RotateZ(yaw), RotateY(pitch)), RotateX(roll));
}

// Rotation block only; the translation column is cleared.
void StoreRotation(const Mat3& r, matrix3x4& out) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            out.m[i][j] = static_cast<float>(r.m[i][j]);

        out.m[i][3] = 0.0f;
    }
}

struct Euler {
    double roll, pitch, yaw;
};

Euler MatToEuler(const matrix3x4& m) {
    // The first column is the rotated X axis; pitch and yaw come from it.
    const double fx = m.m[0][0], fy = m.m[1][0], fz = m.m[2][0];
    const double cosPitch = std::hypot(fx, fy);

    Euler e;
    e.pitch = std::atan2(-fz, cosPitch);

    if (cosPitch > kGimbalCos) {
        e.yaw = std::atan2(fy, fx);
        e.roll = std::atan2(m.m[2][1], m.m[2][2]);
    } else {
        // Yaw and roll share one axis here; fold everything into yaw.
        e.yaw = std::atan2(-m.m[0][1], m.m[1][1]);
        e.roll = 0.0;
    }

    return e;
}

Quat4 ToQuat4(const Quaternion& q) {
    return {q.x, q.y, q.z, q.w};
}

Quaternion ToQuaternion(const Quat4& q) {
    return {static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z),
            static_cast<float>(q.w)};
}

// Hamilton product a * b.
Quat4 Hamilton(const Quat4& a, const Quat4& b) {
    return {
        a.w * b.x + b.w * a.x + (a.y * b.z - a.z * b.y),
        a.w * b.y + b.w * a.y + (a.z * b.x - a.x * b.z),
        a.w * b.z + b.w * a.z + (a.x * b.y - a.y * b.x),
        a.w * b.w - (a.x * b.x + a.y * b.y + a.z * b.z),
    };
}

double Dot(const Quat4& a, const Quat4& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

} // namespace

matrix3x4 QuaternionMatrix(const Quaternion& q, const Vector3& pos) {
    const double x = q.x, y = q.y, z = q.z, w = q.w;
    const double xx = x * x, yy = y * y, zz = z * z;
    const double xy = x * y, xz = x * z, yz = y * z;
    const double wx = w * x, wy = w * y, wz = w * z;

    matrix3x4 r;
    r.m[0][0] = static_cast<float>(1.0 - 2.0 * (yy + zz));
    r.m[0][1] = static_cast<float>(2.0 * (xy - wz));
    r.m[0][2] = static_cast<float>(2.0 * (xz + wy));

    r.m[1][0] = static_cast<float>(2.0 * (xy + wz));
    r.m[1][1] = static_cast<float>(1.0 - 2.0 * (xx + zz));
    r.m[1][2] = static_cast<float>(2.0 * (yz - wx));

    r.m[2][0] = static_cast<float>(2.0 * (xz - wy));
    r.m[2][1] = static_cast<float>(2.0 * (yz + wx));
    r.m[2][2] = static_cast<float>(1.0 - 2.0 * (xx + yy));

    r.m[0][3] = pos.x;
    r.m[1][3] = pos.y;
    r.m[2][3] = pos.z;
    return r;
}

matrix3x4 ConcatTransforms(const matrix3x4& a, const matrix3x4& b) {
    matrix3x4 r;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k)
                sum += a.m[i][k] * b.m[k][j];

            // a's translation applies only to the translation column.
            r.m[i][j] = (j == 3) ? sum + a.m[i][3] : sum;
        }
    }
    return r;
}

Vector3 VectorRotate(const Vector3& v, const matrix3x4& m) {
    auto row = [&](int i) { return m.m[i][0] * v.x + m.m[i][1] * v.y + m.m[i][2] * v.z; };
    return {row(0), row(1), row(2)};
}

Vector3 VectorTransform(const Vector3& v, const matrix3x4& m) {
    const Vector3 r = VectorRotate(v, m);
    return {r.x + m.m[0][3], r.y + m.m[1][3], r.z + m.m[2][3]};
}

Vector3 VectorIRotate(const Vector3& v, const matrix3x4& m) {
    auto col = [&](int j) { return m.m[0][j] * v.x + m.m[1][j] * v.y + m.m[2][j] * v.z; };
    return {col(0), col(1), col(2)};
}

Vector3 VectorITransform(const Vector3& v, const matrix3x4& m) {
    return VectorIRotate({v.x - m.m[0][3], v.y - m.m[1][3], v.z - m.m[2][3]}, m);
}

matrix3x4 MatrixInvert(const matrix3x4& in) {
    matrix3x4 out;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out.m[i][j] = in.m[j][i];

    // New translation is -(R^T * t).
    for (int i = 0; i < 3; ++i) {
        out.m[i][3] = -(out.m[i][0] * in.m[0][3] + out.m[i][1] * in.m[1][3] +
                        out.m[i][2] * in.m[2][3]);
    }
    return out;
}

void AngleMatrix(const RadianEuler& anglesRad, matrix3x4& out) {
    StoreRotation(EulerToMat(anglesRad.x, anglesRad.y, anglesRad.z), out);
}

void AngleMatrix(const RadianEuler& anglesRad, const Vector3& pos, matrix3x4& out) {
    AngleMatrix(anglesRad, out);
    out.m[0][3] = pos.x;
    out.m[1][3] = pos.y;
    out.m[2][3] = pos.z;
}

void AngleMatrixDeg(const Vector3& anglesDeg, matrix3x4& out) {
    StoreRotation(EulerToMat(anglesDeg.z * kDegToRad, anglesDeg.x * kDegToRad,
                             anglesDeg.y * kDegToRad),
                  out);
}

void AngleIMatrixDeg(const Vector3& anglesDeg, matrix3x4& out) {
    StoreRotation(Transposed(EulerToMat(anglesDeg.z * kDegToRad, anglesDeg.x * kDegToRad,
                                        anglesDeg.y * kDegToRad)),
                  out);
}

void AngleIMatrix(const RadianEuler& anglesRad, matrix3x4& out) {
    StoreRotation(Transposed(EulerToMat(anglesRad.x, anglesRad.y, anglesRad.z)), out);
}

bool MatricesAreEqual(const matrix3x4& a, const matrix3x4& b, float tolerance) {
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 4; ++j)
            if (std::fabs(a.m[i][j] - b.m[i][j]) > tolerance)
                return false;

    return true;
}

void MatrixAngles(const matrix3x4& m, RadianEuler& anglesRad, Vector3& pos) {
    const Euler e = MatToEuler(m);

    anglesRad = {static_cast<float>(e.roll), static_cast<float>(e.pitch),
                 static_cast<float>(e.yaw)};
    pos = {m.m[0][3], m.m[1][3], m.m[2][3]};
}

Vector3 MatrixAnglesDeg(const matrix3x4& m) {
    const Euler e = MatToEuler(m);

    return {static_cast<float>(e.pitch * kRadToDeg), static_cast<float>(e.yaw * kRadToDeg),
            static_cast<float>(e.roll * kRadToDeg)};
}

void MatrixAngles(const matrix3x4& m, Quaternion& q, Vector3& pos) {
    const auto& r = m.m;
    const double trace = r[0][0] + r[1][1] + r[2][2];
    Quat4 out;

    // Pick the largest of w, x, y, z to divide by, so the sqrt stays well away from zero.
    if (trace > 0.0) {
        const double s = 2.0 * std::sqrt(trace + 1.0);
        out = {(r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s,
               s / 4.0};
    } else if (r[0][0] >= r[1][1] && r[0][0] >= r[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]);
        out = {s / 4.0, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s,
               (r[2][1] - r[1][2]) / s};
    } else if (r[1][1] >= r[2][2]) {
        const double s = 2.0 * std::sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]);
        out = {(r[0][1] + r[1][0]) / s, s / 4.0, (r[1][2] + r[2][1]) / s,
               (r[0][2] - r[2][0]) / s};
    } else {
        const double s = 2.0 * std::sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]);
        out = {(r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, s / 4.0,
               (r[1][0] - r[0][1]) / s};
    }

    q = ToQuaternion(out);
    QuaternionNormalize(q);
    pos = {r[0][3], r[1][3], r[2][3]};
}

void AngleQuaternion(const RadianEuler& anglesRad, Quaternion& outQuat) {
    const double hr = anglesRad.x * 0.5, hp = anglesRad.y * 0.5, hy = anglesRad.z * 0.5;

    const Quat4 roll{std::sin(hr), 0, 0, std::cos(hr)};
    const Quat4 pitch{0, std::sin(hp), 0, std::cos(hp)};
    const Quat4 yaw{0, 0, std::sin(hy), std::cos(hy)};

    outQuat = ToQuaternion(Hamilton(Hamilton(yaw, pitch), roll));
}

void QuaternionAlign(const Quaternion& p, const Quaternion& q, Quaternion& qt) {
    const bool flip = Dot(ToQuat4(p), ToQuat4(q)) < 0.0;

    qt = flip ? Quaternion{-q.x, -q.y, -q.z, -q.w} : q;
}

void QuaternionScale(const Quaternion& p, float t, Quaternion& q) {
    // |xyz| = sin(half angle) and |w| = cos(half angle); scale the half angle by t.
    const double mag = std::sqrt(double(p.x) * p.x + double(p.y) * p.y + double(p.z) * p.z);
    const double halfAngle = std::atan2(mag, std::fabs(double(p.w)));
    const double axisScale = (mag > 1e-12) ? std::sin(halfAngle * t) / mag : double(t);

    // w keeps the input's sign.
    const double wMag = std::fabs(std::cos(halfAngle * t));

    q = ToQuaternion({p.x * axisScale, p.y * axisScale, p.z * axisScale,
                      (p.w < 0.0f) ? -wMag : wMag});
}

void QuaternionMult(const Quaternion& p, const Quaternion& q, Quaternion& qt) {
    Quaternion aligned;
    QuaternionAlign(p, q, aligned);

    qt = ToQuaternion(Hamilton(ToQuat4(p), ToQuat4(aligned)));
}

void QuaternionSM(float s, const Quaternion& p, const Quaternion& q, Quaternion& qt) {
    Quaternion scaled, product;
    QuaternionScale(p, s, scaled);
    QuaternionMult(scaled, q, product);

    QuaternionNormalize(product);
    qt = product;
}

void QuaternionMA(const Quaternion& p, float s, const Quaternion& q, Quaternion& qt) {
    Quaternion scaled, product;
    QuaternionScale(q, s, scaled);
    QuaternionMult(p, scaled, product);

    QuaternionNormalize(product);
    qt = product;
}

void QuaternionAngles(const Quaternion& q, RadianEuler& angles) {
    Vector3 unusedPos;

    MatrixAngles(QuaternionMatrix(q, Vector3{0, 0, 0}), angles, unusedPos);
}

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

void QuaternionSlerpNoAlign(const Quaternion& p, const Quaternion& q, float t, Quaternion& qt) {
    const Quat4 a = ToQuat4(p);
    const Quat4 b = ToQuat4(q);

    const double cosTheta = std::clamp(Dot(a, b), -1.0, 1.0);
    const double theta = std::acos(cosTheta);
    const double sinTheta = std::sin(theta);

    Quat4 end = b;
    double weightA, weightB;

    if (sinTheta > 1e-6) {
        weightA = std::sin((1.0 - t) * theta) / sinTheta;
        weightB = std::sin(t * theta) / sinTheta;
    } else if (cosTheta > 0.0) {
        // Nearly identical: a straight lerp is exact enough.
        weightA = 1.0 - t;
        weightB = t;
    } else {
        // Opposite: sweep 180 degrees through a quaternion perpendicular to a.
        end = {-a.y, a.x, -a.w, a.z};
        weightA = std::cos(kPi * t);
        weightB = std::sin(kPi * t);
    }

    qt = ToQuaternion({weightA * a.x + weightB * end.x, weightA * a.y + weightB * end.y,
                       weightA * a.z + weightB * end.z, weightA * a.w + weightB * end.w});
}

void QuaternionSlerp(const Quaternion& p, const Quaternion& q, float t, Quaternion& qt) {
    Quaternion aligned;

    QuaternionAlign(p, q, aligned);
    QuaternionSlerpNoAlign(p, aligned, t, qt);
}

matrix3x4 MatrixInverseTranspose(const matrix3x4& in) {
    const auto& a = in.m;

    // The cofactor matrix over the determinant is the inverse transpose.
    double cof[3][3];
    for (int i = 0; i < 3; ++i) {
        const int i1 = (i + 1) % 3, i2 = (i + 2) % 3;

        for (int j = 0; j < 3; ++j) {
            const int j1 = (j + 1) % 3, j2 = (j + 2) % 3;
            cof[i][j] = double(a[i1][j1]) * a[i2][j2] - double(a[i1][j2]) * a[i2][j1];
        }
    }

    const double det = a[0][0] * cof[0][0] + a[0][1] * cof[0][1] + a[0][2] * cof[0][2];
    if (std::fabs(det) < 1e-12)
        return in;

    matrix3x4 out;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            out.m[i][j] = static_cast<float>(cof[i][j] / det);

        out.m[i][3] = 0.0f;
    }
    return out;
}

float VectorNormalize(Vector3& v) {
    const double length = std::sqrt(double(v.x) * v.x + double(v.y) * v.y + double(v.z) * v.z);

    if (length > 0.0) {
        v.x = static_cast<float>(v.x / length);
        v.y = static_cast<float>(v.y / length);
        v.z = static_cast<float>(v.z / length);
    }
    return static_cast<float>(length);
}

float QuaternionNormalize(Quaternion& q) {
    const double length = std::sqrt(double(q.x) * q.x + double(q.y) * q.y +
                                    double(q.z) * q.z + double(q.w) * q.w);

    if (length > 0.0) {
        q.x = static_cast<float>(q.x / length);
        q.y = static_cast<float>(q.y / length);
        q.z = static_cast<float>(q.z / length);
        q.w = static_cast<float>(q.w / length);
    }
    return static_cast<float>(length);
}

// --- display / loader helpers ------------------------------------------------

Vector3 Normalize(const Vector3& v) {
    Vector3 r = v;

    VectorNormalize(r);
    return r;
}

Quaternion QuaternionNormalized(const Quaternion& q) {
    Quaternion r = q;

    if (QuaternionNormalize(r) == 0.0f)
        return {0, 0, 0, 1};

    return r;
}

} // namespace pulse::math
