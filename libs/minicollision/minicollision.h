#pragma once
// minicollision - standalone .phy collision builder.
//
// Ported from the original PulseMDL's libs/minicollision. The original implemented
// Valve's IPhysicsCollision virtual interface so it could stand in for vphysics.dll;
// 
// Two opaque types, matching vphysics' model:
//   Convex  - one convex hull.
//   Collide - a compound of one or more hulls; this is what serializes to the
//             per-solid IVP blob inside a .phy.
//
// Deliberately self-contained: it carries its own vector type rather than using
// libs/math.  Callers convert at the boundary, as with libs/meshoptimizer.

#include <string>
#include <cmath>

namespace pulse::phys {

struct Vec3 {
    float x = 0, y = 0, z = 0;

    Vec3() = default;
    Vec3(float ix, float iy, float iz) : x(ix), y(iy), z(iz) {}

    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator/=(float s) { x /= s; y /= s; z /= s; return *this; }

    float Length() const { return std::sqrt(x * x + y * y + z * z); }
};

struct Convex;   // opaque - one convex hull
struct Collide;  // opaque - compound of hulls, one .phy solid

// ---------------------------------------------------------------------------
// Convex hulls
// ---------------------------------------------------------------------------

// Build the convex hull of a point cloud. Returns nullptr when the cloud is
// degenerate (fewer than 4 non-coplanar points), leaving *err empty; returns
// nullptr with *err set when a hull was built but is unusable - not a closed
// 2-manifold, or past IVP's 12-bit triangle / 16-bit point index limits. IVP
// navigates hulls by edge->get_opposite() hops at runtime, so an unpaired
// directed edge sends the in-game solver into garbage memory.
Convex* ConvexFromVerts(const Vec3* verts, int vertCount, std::string* err);

float ConvexVolume(const Convex* convex);
float ConvexSurfaceArea(const Convex* convex);

// Points in the built hull. The caller's input cloud is not this - the hull
// keeps only what its faces reference.
int ConvexVertexCount(const Convex* convex);

// Stashed in the ledge's client_data - the bone index, for ragdolls.
void SetConvexGameData(Convex* convex, unsigned int gameData);

void ConvexFree(Convex* convex);

// ---------------------------------------------------------------------------
// Compound collides
// ---------------------------------------------------------------------------

// TAKES OWNERSHIP of every non-null convex passed in; do not ConvexFree them
// afterwards. Null entries are skipped.
Collide* ConvertConvexToCollide(Convex** convexes, int convexCount);

void CollideFree(Collide* collide);

// Serialized size in bytes. Source's vcollide loader requires the per-solid
// size to equal the blob exactly, with no slack, so this is not padded.
int CollideSize(const Collide* collide);

// Serialize into dest, which must hold at least CollideSize bytes. Returns
// bytes written.
int CollideWrite(char* dest, const Collide* collide);

void CollideGetAABB(const Collide* collide, Vec3* mins, Vec3* maxs);
Vec3 CollideGetMassCenter(const Collide* collide);
void CollideSetMassCenter(Collide* collide, const Vec3& massCenter);
float CollideVolume(const Collide* collide);

} // namespace pulse::phys
