// minicollision - hull mass properties and IVP compact-surface serialization.
//
// The binary layout comes from the original PulseMDL's libs/minicollision. The
// constants and axis permutations are what make vphysics read the blob correctly.

#include "minicollision.h"
#include "convexhull.h"
#include "ivp_compact.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>
#include <unordered_map>

namespace pulse::phys {

// ---------------------------------------------------------------------------
// Implementation types
// ---------------------------------------------------------------------------

// One convex hull, result of ConvexFromVerts.
struct Convex {
    ConvexHullResult hull;
    unsigned int     gameData = 0;  // from SetConvexGameData (bone index for ragdolls)

    float Volume() const;
    float SurfaceArea() const;
    Vec3  MassCenter() const;
    void  GetAABB(Vec3& mins, Vec3& maxs) const;
};

// Compound of one or more convex hulls, result of ConvertConvexToCollide.
struct Collide {
    std::vector<Convex*> hulls;  // owned
    Vec3 massCenter;
    Vec3 naturalCenter;  // volume centroid, kept when massCenter is overridden
    bool massCenterOverridden = false;

    ~Collide() {
        for (Convex* h : hulls) delete h;
    }

    void ComputeMassCenter();
    void GetAABB(Vec3* mins, Vec3* maxs) const;
    float Volume() const;
};

// ---------------------------------------------------------------------------
// Convex helpers
// ---------------------------------------------------------------------------

float Convex::Volume() const
{
    const auto& verts = hull.vertices;
    const auto& idx   = hull.indices;
    float vol = 0.0f;
    for (int i = 0; i < (int)idx.size(); i += 3) {
        const HullVert& v0 = verts[idx[i+0]];
        const HullVert& v1 = verts[idx[i+1]];
        const HullVert& v2 = verts[idx[i+2]];
        // signed volume of tetrahedron from origin
        vol += v0.x*(v1.y*v2.z - v1.z*v2.y)
             + v0.y*(v1.z*v2.x - v1.x*v2.z)
             + v0.z*(v1.x*v2.y - v1.y*v2.x);
    }
    return fabsf(vol) / 6.0f;
}

float Convex::SurfaceArea() const
{
    const auto& verts = hull.vertices;
    const auto& idx   = hull.indices;
    float area = 0.0f;
    for (int i = 0; i < (int)idx.size(); i += 3) {
        const HullVert& a = verts[idx[i+0]];
        const HullVert& b = verts[idx[i+1]];
        const HullVert& c = verts[idx[i+2]];
        float ex = b.x-a.x, ey = b.y-a.y, ez = b.z-a.z;
        float fx = c.x-a.x, fy = c.y-a.y, fz = c.z-a.z;
        float cx = ey*fz - ez*fy;
        float cy = ez*fx - ex*fz;
        float cz = ex*fy - ey*fx;
        area += sqrtf(cx*cx + cy*cy + cz*cz);
    }
    return area * 0.5f;
}

Vec3 Convex::MassCenter() const
{
    const auto& verts = hull.vertices;
    const auto& idx   = hull.indices;
    float cx=0, cy=0, cz=0, totalVol=0;
    for (int i = 0; i < (int)idx.size(); i += 3) {
        const HullVert& v0 = verts[idx[i+0]];
        const HullVert& v1 = verts[idx[i+1]];
        const HullVert& v2 = verts[idx[i+2]];
        float vol = (v0.x*(v1.y*v2.z - v1.z*v2.y)
                   + v0.y*(v1.z*v2.x - v1.x*v2.z)
                   + v0.z*(v1.x*v2.y - v1.y*v2.x)) / 6.0f;
        // centroid of tetrahedron (origin + v0 + v1 + v2) / 4
        cx += vol * (v0.x + v1.x + v2.x) / 4.0f;
        cy += vol * (v0.y + v1.y + v2.y) / 4.0f;
        cz += vol * (v0.z + v1.z + v2.z) / 4.0f;
        totalVol += vol;
    }
    if (fabsf(totalVol) > 1e-10f) { cx/=totalVol; cy/=totalVol; cz/=totalVol; }
    return Vec3(cx, cy, cz);
}

void Convex::GetAABB(Vec3& mins, Vec3& maxs) const
{
    mins.x = mins.y = mins.z =  1e30f;
    maxs.x = maxs.y = maxs.z = -1e30f;
    for (const HullVert& v : hull.vertices) {
        if (v.x < mins.x) mins.x = v.x;
        if (v.y < mins.y) mins.y = v.y;
        if (v.z < mins.z) mins.z = v.z;
        if (v.x > maxs.x) maxs.x = v.x;
        if (v.y > maxs.y) maxs.y = v.y;
        if (v.z > maxs.z) maxs.z = v.z;
    }
}

// ---------------------------------------------------------------------------
// Collide helpers
// ---------------------------------------------------------------------------

void Collide::ComputeMassCenter()
{
    if (massCenterOverridden) return;
    float totalVol = 0;
    massCenter = Vec3();
    for (Convex* h : hulls) {
        float v = h->Volume();
        massCenter += h->MassCenter() * v;
        totalVol   += v;
    }
    if (totalVol > 1e-10f) massCenter /= totalVol;
    naturalCenter = massCenter;
}

void Collide::GetAABB(Vec3* pMins, Vec3* pMaxs) const
{
    pMins->x = pMins->y = pMins->z =  1e30f;
    pMaxs->x = pMaxs->y = pMaxs->z = -1e30f;
    for (Convex* h : hulls) {
        Vec3 lo, hi;
        h->GetAABB(lo, hi);
        if (lo.x < pMins->x) pMins->x = lo.x;
        if (lo.y < pMins->y) pMins->y = lo.y;
        if (lo.z < pMins->z) pMins->z = lo.z;
        if (hi.x > pMaxs->x) pMaxs->x = hi.x;
        if (hi.y > pMaxs->y) pMaxs->y = hi.y;
        if (hi.z > pMaxs->z) pMaxs->z = hi.z;
    }
}

float Collide::Volume() const
{
    float v = 0;
    for (Convex* h : hulls) v += h->Volume();
    return v;
}

// ---------------------------------------------------------------------------
// PHY blob serialization
// ---------------------------------------------------------------------------

// IVP/vphysics stores all spatial quantities in meters. Source game units are
// inches. vphysics applies this conversion internally in ConvexFromVerts and
// CollideWrite; we must replicate it so the blob is read correctly at runtime.
static const float INCHES_TO_METERS    = 0.0254f;
static const float INCHES_TO_METERS_SQ = INCHES_TO_METERS * INCHES_TO_METERS;

// PHY compact surfaces are in IVP coordinate space (Y-DOWN, meters).
// vphysics' ConvertPositionToIVP does:
//   IVP[0] =  Source.x * scale
//   IVP[1] = -Source.z * scale   (Source "up" -> IVP "-Y")
//   IVP[2] =  Source.y * scale
// This has det=1 (pure rotation), so triangle winding is preserved.
static inline void SrcToIVP(float sx, float sy, float sz, float& ox, float& oy, float& oz)
{
    ox =  sx * INCHES_TO_METERS;
    oy = -sz * INCHES_TO_METERS;
    oz =  sy * INCHES_TO_METERS;
}

// Compute IVP compact ledge size in bytes for a single hull
static int LedgeSizeBytes(const Convex* h)
{
    int nTri = (int)h->hull.indices.size() / 3;
    int nPts = (int)h->hull.vertices.size();
    return (1 + nTri + nPts) * 16;
}

// Compute total PHY blob size for a Collide.
//
// The .phy stores each solid as "int32 size" + blob, and Source's vcollide
// loader requires size to equal exactly phycollide_hdr_t(28) + surfaceSize with
// no slack. So the blob size must NOT be padded - any extra bytes make the
// loader reject the whole vcollide ("missing vcollide data").
static int CalcCollideSize(const Collide* col)
{
    int N = (int)col->hulls.size();
    if (N == 0) return 0;
    // blob = header(28) + surface(48) + sum(ledge sizes) + tree_nodes(28*(2N-1))
    int nodeCount = 2 * N - 1;
    int ledgeBytes = 0;
    for (Convex* h : col->hulls) ledgeBytes += LedgeSizeBytes(h);
    return 28 + 48 + nodeCount * 28 + ledgeBytes;
}

// Validate that a hull is a closed 2-manifold within IVP's bitfield limits.
// IVP's runtime navigates hulls with edge->get_opposite() hops; a single
// unpaired or duplicated directed edge sends the solver into garbage memory
// in-game, so reject such hulls up front.
static bool HullIsValidForIVP(const ConvexHullResult& hull)
{
    int nTri = (int)hull.indices.size() / 3;
    int nPts = (int)hull.vertices.size();
    if (nTri < 4 || nTri > 4095) return false;   // tri_index / pierce_index are 12-bit
    if (nPts < 4 || nPts > 65535) return false;  // start_point_index is 16-bit

    std::unordered_map<uint64_t, int> edges;
    edges.reserve(nTri * 6);
    for (int t = 0; t < nTri; t++) {
        for (int e = 0; e < 3; e++) {
            int v0 = hull.indices[t*3 + e];
            int v1 = hull.indices[t*3 + (e+1)%3];
            if (v0 == v1) return false;
            uint64_t key = ((uint64_t)v0 << 32) | (uint32_t)v1;
            if (edges.count(key)) return false;  // non-manifold: edge used twice
            edges[key] = t*3 + e;
        }
    }
    for (const auto& kv : edges) {
        uint64_t rev = (kv.first << 32) | (kv.first >> 32);
        if (!edges.count(rev)) return false;     // open mesh: no reversed twin
    }
    return true;
}

// Serialize one compact ledge into 'dest'. Returns bytes written.
static int WriteCompactLedge(char* dest, const Convex* h, int gameData)
{
    const auto& verts = h->hull.vertices;
    const auto& idx   = h->hull.indices;
    int nTri = (int)idx.size() / 3;
    int nPts = (int)verts.size();

    // Build adjacency: directed edge (v0->v1) -> linear edge slot in this ledge.
    // IVP's get_opposite() does pointer arithmetic in 4-byte edge units and a
    // triangle occupies 4 slots (bitfield dword + 3 edges), so slot = t*4 + e.
    std::unordered_map<uint64_t, int> edgeLinear;
    edgeLinear.reserve(nTri * 6);
    for (int t = 0; t < nTri; t++) {
        for (int e = 0; e < 3; e++) {
            int v0 = idx[t*3 + e];
            int v1 = idx[t*3 + (e+1)%3];
            edgeLinear[((uint64_t)v0 << 32) | (uint32_t)v1] = t*4 + e;
        }
    }

    // Triangle centroids and outward normals for pierce_index computation
    std::vector<HullVert> triCentroid(nTri), triNormal(nTri);
    for (int t = 0; t < nTri; t++) {
        const HullVert& a = verts[idx[t*3+0]];
        const HullVert& b = verts[idx[t*3+1]];
        const HullVert& c = verts[idx[t*3+2]];
        triCentroid[t] = { (a.x+b.x+c.x)/3.0f, (a.y+b.y+c.y)/3.0f, (a.z+b.z+c.z)/3.0f };
        float ex = b.x-a.x, ey = b.y-a.y, ez = b.z-a.z;
        float fx = c.x-a.x, fy = c.y-a.y, fz = c.z-a.z;
        triNormal[t] = { ey*fz - ez*fy, ez*fx - ex*fz, ex*fy - ey*fx };
    }

    // Write ledge header
    ivp_compact_ledge_t hdr;
    ivp_ledge_init(hdr, nTri, nPts);
    hdr.client_data = gameData;
    memcpy(dest, &hdr, sizeof(hdr));
    int off = sizeof(hdr);  // = 16

    // Write triangles
    for (int t = 0; t < nTri; t++) {
        // pierce_index = triangle on the far side of the hull (the one whose
        // centroid lies deepest against this triangle's outward normal); IVP
        // uses it as the start of the pierce walk in minimize_on_other_side().
        int pierce = t;
        float bestDot = 1e30f;
        for (int u = 0; u < nTri; u++) {
            if (u == t) continue;
            float d = triNormal[t].x * triCentroid[u].x
                    + triNormal[t].y * triCentroid[u].y
                    + triNormal[t].z * triCentroid[u].z;
            if (d < bestDot) { bestDot = d; pierce = u; }
        }

        ivp_compact_triangle_t tri;
        ivp_tri_set(tri, t, pierce, 0);
        for (int e = 0; e < 3; e++) {
            int v0 = idx[t*3 + e];
            int v1 = idx[t*3 + (e+1)%3];
            auto it = edgeLinear.find(((uint64_t)v1 << 32) | (uint32_t)v0);
            int16_t opp = 0;
            if (it != edgeLinear.end())
                opp = (int16_t)(it->second - (t*4 + e));
            ivp_edge_set(tri.c_three_edges[e], (uint16_t)v0, opp);
        }
        memcpy(dest + off, &tri, sizeof(tri));
        off += sizeof(tri);
    }

    // Write points: scale to meters (vphysics does its own axis conversion at load time)
    for (const HullVert& v : verts) {
        ivp_compact_poly_point_t pt;
        SrcToIVP(v.x, v.y, v.z, pt.k[0], pt.k[1], pt.k[2]);
        pt.hesse_val = 0.0f;
        memcpy(dest + off, &pt, sizeof(pt));
        off += sizeof(pt);  // 16 bytes
    }

    return off;
}

// Per-axis variance of the solid about 'center' (source axes, in^2), summed over
// every hull as one body.
static void SecondMoments(const Collide* col, const Vec3& center, double out[3])
{
    double moment[3] = {0, 0, 0}, totalVol = 0;
    for (const Convex* h : col->hulls) {
        const auto& verts = h->hull.vertices;
        const auto& idx   = h->hull.indices;
        for (int i = 0; i + 2 < (int)idx.size(); i += 3) {
            double p[3][3];
            for (int k = 0; k < 3; k++) {
                const HullVert& v = verts[idx[i + k]];
                p[k][0] = (double)v.x - center.x;
                p[k][1] = (double)v.y - center.y;
                p[k][2] = (double)v.z - center.z;
            }
            double vol = (p[0][0] * (p[1][1] * p[2][2] - p[1][2] * p[2][1])
                        + p[0][1] * (p[1][2] * p[2][0] - p[1][0] * p[2][2])
                        + p[0][2] * (p[1][0] * p[2][1] - p[1][1] * p[2][0])) / 6.0;
            totalVol += vol;
            for (int a = 0; a < 3; a++) {
                double s = p[0][a] * p[0][a] + p[1][a] * p[1][a] + p[2][a] * p[2][a]
                         + p[0][a] * p[1][a] + p[0][a] * p[2][a] + p[1][a] * p[2][a];
                moment[a] += vol * s / 10.0;
            }
        }
    }
    for (int a = 0; a < 3; a++)
        out[a] = (std::fabs(totalVol) > 1e-12) ? moment[a] / totalVol : 0.0;
}

// Fraction of each bounding-box face covered by the solid's silhouette, sampled on
// a grid of cell centers (source axes, indexed by the axis looked along).
static void DragFractions(const Collide* col, const Vec3& mins, const Vec3& maxs, float out[3])
{
    const float lo[3] = {mins.x, mins.y, mins.z};
    const float hi[3] = {maxs.x, maxs.y, maxs.z};
    float size[3], minFace = -1.0f;
    for (int a = 0; a < 3; a++) {
        size[a] = hi[a] - lo[a];
    }
    for (int a = 0; a < 3; a++) {
        float face = size[(a + 1) % 3] * size[(a + 2) % 3];
        if (minFace < 0.0f || face < minFace) minFace = face;
    }
    float eps = minFace * 1e-4f;
    eps = eps < 0.25f ? 0.25f : (eps > 128.0f ? 128.0f : eps);
    const float baseSide = std::fmax(std::sqrt(eps), 1e-4f);

    for (int axis = 0; axis < 3; axis++) {
        const int u = (axis + 1) % 3, v = (axis + 2) % 3;
        float side = baseSide;
        // bound the grid so a huge flat solid cannot allocate without limit
        while ((double)(size[u] / side) * (double)(size[v] / side) > 1.6e7) side *= 2.0f;
        const float hs = side * 0.5f;
        int nu = 0, nv = 0;
        while (lo[u] + hs + nu * side < hi[u]) nu++;
        while (lo[v] + hs + nv * side < hi[v]) nv++;
        out[axis] = 0.0f;
        if (nu <= 0 || nv <= 0) continue;

        std::vector<uint8_t> hit((size_t)nu * nv, 0);
        for (const Convex* h : col->hulls) {
            const auto& verts = h->hull.vertices;
            const auto& idx   = h->hull.indices;
            for (int i = 0; i + 2 < (int)idx.size(); i += 3) {
                float pu[3], pv[3];
                for (int k = 0; k < 3; k++) {
                    const HullVert& vt = verts[idx[i + k]];
                    const float c[3] = {vt.x, vt.y, vt.z};
                    pu[k] = c[u]; pv[k] = c[v];
                }
                const float area = (pu[1] - pu[0]) * (pv[2] - pv[0]) - (pu[2] - pu[0]) * (pv[1] - pv[0]);
                if (std::fabs(area) < 1e-12f) continue;  // edge-on to the view

                const float minU = std::fmin(pu[0], std::fmin(pu[1], pu[2]));
                const float maxU = std::fmax(pu[0], std::fmax(pu[1], pu[2]));
                const float minV = std::fmin(pv[0], std::fmin(pv[1], pv[2]));
                const float maxV = std::fmax(pv[0], std::fmax(pv[1], pv[2]));
                int i0 = (int)std::ceil((minU - (lo[u] + hs)) / side);
                int i1 = (int)std::floor((maxU - (lo[u] + hs)) / side);
                int j0 = (int)std::ceil((minV - (lo[v] + hs)) / side);
                int j1 = (int)std::floor((maxV - (lo[v] + hs)) / side);
                if (i0 < 0) i0 = 0;
                if (j0 < 0) j0 = 0;
                if (i1 > nu - 1) i1 = nu - 1;
                if (j1 > nv - 1) j1 = nv - 1;

                for (int j = j0; j <= j1; j++) {
                    const float cv = lo[v] + hs + j * side;
                    for (int ii = i0; ii <= i1; ii++) {
                        const float cu = lo[u] + hs + ii * side;
                        float e[3];
                        for (int k = 0; k < 3; k++) {
                            const int n = (k + 1) % 3;
                            e[k] = (pu[n] - pu[k]) * (cv - pv[k]) - (pv[n] - pv[k]) * (cu - pu[k]);
                        }
                        const bool neg = e[0] < 0 || e[1] < 0 || e[2] < 0;
                        const bool pos = e[0] > 0 || e[1] > 0 || e[2] > 0;
                        if (!(neg && pos)) hit[(size_t)j * nu + ii] = 1;
                    }
                }
            }
        }
        size_t hits = 0;
        for (uint8_t b : hit) hits += b;
        out[axis] = (float)hits / (float)hit.size();
    }
}

// Sphere + quantized box for one tree node from its IVP-space bounds. box_sizes
// are the half-extents in 1/250ths of the node radius.
static void FillNodeBounds(ivp_ledgetree_node_t* nd, const float mn[3], const float mx[3])
{
    float half[3];
    for (int a = 0; a < 3; a++) {
        nd->center[a] = mx[a] * 0.5f + mn[a] * 0.5f;
        half[a] = mx[a] - nd->center[a];
    }
    nd->radius = std::sqrt(half[0] * half[0] + half[1] * half[1] + half[2] * half[2]);
    const float work = nd->radius * (1.0f / 250.0f);
    for (int a = 0; a < 3; a++) {
        int q = (work > 0.0f) ? (int)(half[a] / work) + 1 : 0;
        nd->box_sizes[a] = (uint8_t)(q > 255 ? 255 : q);
    }
}

// Bounds of every vertex of the given hulls, in IVP space (meters).
static void IvpBounds(const Convex* const* hulls, int count, float mn[3], float mx[3])
{
    for (int a = 0; a < 3; a++) { mn[a] = 1e30f; mx[a] = -1e30f; }
    for (int k = 0; k < count; k++) {
        for (const HullVert& v : hulls[k]->hull.vertices) {
            float p[3];
            SrcToIVP(v.x, v.y, v.z, p[0], p[1], p[2]);
            for (int a = 0; a < 3; a++) {
                if (p[a] < mn[a]) mn[a] = p[a];
                if (p[a] > mx[a]) mx[a] = p[a];
            }
        }
    }
}

// Write full PHY blob for a Collide into 'dest'. Returns bytes written.
static int WriteCollide(char* dest, const Collide* col)
{
    int N = (int)col->hulls.size();
    if (N == 0) return 0;

    int nodeCount = 2 * N - 1;
    // Pre-compute ledge sizes for offset math
    std::vector<int> ledgeSz(N);
    for (int i = 0; i < N; i++) ledgeSz[i] = LedgeSizeBytes(col->hulls[i]);

    Vec3 mins, maxs;
    col->GetAABB(&mins, &maxs);

    // Whole-collide bounds in IVP space, used by the internal tree nodes
    std::vector<const Convex*> hullPtrs(col->hulls.begin(), col->hulls.end());
    float allMin[3], allMax[3];
    IvpBounds(hullPtrs.data(), N, allMin, allMax);

    // Total IVP data size (surface + ledges + tree)
    int ledgeTotal = 0;
    for (int s : ledgeSz) ledgeTotal += s;
    int surfaceDataSize = 48 + ledgeTotal + nodeCount * 28;

    // rotation_inertia per IVP axis is sqrt(sum of the other two axes' variance^2),
    // taken about the volume centroid even when the mass center is overridden.
    double var[3];
    SecondMoments(col, col->naturalCenter, var);
    const double vx = var[0], vy = var[2], vz = var[1];  // IVP y = Src z, IVP z = Src y
    Vec3 inertia((float)std::sqrt(vy * vy + vz * vz),
                 (float)std::sqrt(vx * vx + vz * vz),
                 (float)std::sqrt(vx * vx + vy * vy));

    float mcx_ivp, mcy_ivp, mcz_ivp;
    SrcToIVP(col->massCenter.x, col->massCenter.y, col->massCenter.z, mcx_ivp, mcy_ivp, mcz_ivp);

    // upper_limit_radius and max_factor_surface_deviation are measured around
    // the mass center (IVP_SurfaceBuilder_Ledge_Soup::insert_radius_in_compact_surface
    // -> IVP_Compact_Ledge_Solver::calc_radius_to_given_center).
    float massRadiusM = 0.0f;
    float massDevM    = 0.0f;
    for (Convex* h : col->hulls) {
        const auto& verts = h->hull.vertices;
        const auto& idx   = h->hull.indices;
        for (int t = 0; t < (int)idx.size() / 3; t++) {
            float p[3][3]; // triangle verts relative to mass center, IVP space
            for (int k = 0; k < 3; k++) {
                const HullVert& v = verts[idx[t*3 + k]];
                SrcToIVP(v.x, v.y, v.z, p[k][0], p[k][1], p[k][2]);
                p[k][0] -= mcx_ivp; p[k][1] -= mcy_ivp; p[k][2] -= mcz_ivp;
            }
            float e1x = p[1][0]-p[0][0], e1y = p[1][1]-p[0][1], e1z = p[1][2]-p[0][2];
            float e2x = p[2][0]-p[0][0], e2y = p[2][1]-p[0][1], e2z = p[2][2]-p[0][2];
            float hx = e1y*e2z - e1z*e2y;
            float hy = e1z*e2x - e1x*e2z;
            float hz = e1x*e2y - e1y*e2x;
            float hLenSq = hx*hx + hy*hy + hz*hz;
            for (int k = 0; k < 3; k++) {
                float qSq = p[k][0]*p[k][0] + p[k][1]*p[k][1] + p[k][2]*p[k][2];
                if (qSq > massRadiusM * massRadiusM) massRadiusM = sqrtf(qSq);
                if (hLenSq > 1e-20f) {
                    // distance of vertex from the line through the mass center
                    // along the face normal = |p x h| / |h|
                    float cxp = p[k][1]*hz - p[k][2]*hy;
                    float cyp = p[k][2]*hx - p[k][0]*hz;
                    float czp = p[k][0]*hy - p[k][1]*hx;
                    float devSq = (cxp*cxp + cyp*cyp + czp*czp) / hLenSq;
                    if (devSq > massDevM * massDevM) massDevM = sqrtf(devSq);
                }
            }
        }
    }
    // IVP_COMPACT_SURFACE_DEVIATION_STEP_SIZE = 1/250
    int devFactor = 0;
    if (massRadiusM > 1e-10f)
        devFactor = (int)(1.0f + massDevM / (massRadiusM * (1.0f / 250.0f)));
    if (devFactor > 255) devFactor = 255;

    // ------------------------------------------------------------------
    // 1. phycollide_hdr_t
    // ------------------------------------------------------------------
    int off = 0;
    phycollide_hdr_t hdr;
    hdr.vphysicsID      = (int32_t)MINICOL_VPHY_ID;
    hdr.version         = MINICOL_VERSION_IVP;
    hdr.modelType       = 0;
    hdr.surfaceSize     = surfaceDataSize;
    // Unitless silhouette coverage (0..1) of the bounding box face along each Source axis
    DragFractions(col, mins, maxs, hdr.dragAxisAreas);
    hdr.axisMapSize     = 0;
    memcpy(dest + off, &hdr, sizeof(hdr)); off += sizeof(hdr); // 28

    // ------------------------------------------------------------------
    // 2. ivp_compact_surface_t
    // ------------------------------------------------------------------
    ivp_compact_surface_t surf;
    memset(&surf, 0, sizeof(surf));
    surf.mass_center[0]       = mcx_ivp;
    surf.mass_center[1]       = mcy_ivp;
    surf.mass_center[2]       = mcz_ivp;
    // inertia is already in IVP axis order
    surf.rotation_inertia[0]  = inertia.x * INCHES_TO_METERS_SQ;
    surf.rotation_inertia[1]  = inertia.y * INCHES_TO_METERS_SQ;
    surf.rotation_inertia[2]  = inertia.z * INCHES_TO_METERS_SQ;
    surf.upper_limit_radius   = massRadiusM;
    surf.bitfield = (uint32_t)devFactor; // max_factor_surface_deviation in bits [0..7]
    ivp_surface_set_byte_size(surf, surfaceDataSize);
    // Valve layout: ledges directly follow the 48-byte surface header (keeps
    // every ledge 16-byte aligned, required by IVP's edge->triangle pointer
    // masking), ledge tree nodes go at the end.
    surf.offset_ledgetree_root = (int32_t)sizeof(ivp_compact_surface_t) + ledgeTotal;
    surf.dummy[0] = 0;
    surf.dummy[1] = 0;
    surf.dummy[2] = (int32_t)MINICOL_IVPS_ID;
    memcpy(dest + off, &surf, sizeof(surf)); off += sizeof(surf); // 48

    // ------------------------------------------------------------------
    // 3. Compact ledges. At runtime vphysics copies the blob starting at the
    //    compact surface into 16-aligned memory, so alignment relative to the
    //    surface start (dest[28]) is what matters: surface header = 48 and
    //    each ledge is a multiple of 16 bytes -> every ledge stays 16-aligned.
    // ------------------------------------------------------------------
    std::vector<int> ledgeAbsOff(N); // offsets from dest[0]
    ledgeAbsOff[0] = off;
    for (int k = 1; k < N; k++) ledgeAbsOff[k] = ledgeAbsOff[k-1] + ledgeSz[k-1];

    for (int k = 0; k < N; k++) {
        int written = WriteCompactLedge(dest + off, col->hulls[k], col->hulls[k]->gameData);
        off += written;
    }

    // ------------------------------------------------------------------
    // 4. Ledge tree nodes: degenerate left-chain layout
    //    Nodes [0 .. N-2] are internal, nodes [N-1 .. 2N-2] are leaves.
    //    Left child of internal node i = next in memory (node i+1).
    //    Right child of internal node i = node N+i.
    // ------------------------------------------------------------------
    int nodeBase = off; // offset of node array start from dest[0]

    // Write all 2N-1 nodes (fill positions first with zeros, fix offsets after)
    int nodeAreaSize = nodeCount * (int)sizeof(ivp_ledgetree_node_t);
    memset(dest + off, 0, nodeAreaSize);

    // Helper: get node pointer by index
    auto NodeAt = [&](int ni) -> ivp_ledgetree_node_t* {
        return reinterpret_cast<ivp_ledgetree_node_t*>(dest + nodeBase + ni * 28);
    };

    // Fill internal nodes (0 .. N-2)
    for (int i = 0; i < N-1; i++) {
        ivp_ledgetree_node_t* nd = NodeAt(i);
        // Left child = node i+1 (automatically next in memory, no offset needed)
        // Right child = node N+i
        int rightNodeAbsOff = nodeBase + (N+i) * 28;
        int thisNodeAbsOff  = nodeBase + i * 28;
        nd->offset_right_node    = rightNodeAbsOff - thisNodeAbsOff;
        nd->offset_compact_ledge = 0; // internal
        FillNodeBounds(nd, allMin, allMax);
    }

    // Fill leaf nodes (N-1 .. 2N-2), leaf k -> hull k
    for (int k = 0; k < N; k++) {
        int leafNodeIdx   = N - 1 + k;
        int leafAbsOff    = nodeBase + leafNodeIdx * 28;
        int ledgeAbsOffK  = ledgeAbsOff[k];

        const Convex* h = col->hulls[k];
        float hMin[3], hMax[3];
        IvpBounds(&h, 1, hMin, hMax);

        ivp_ledgetree_node_t* nd = NodeAt(leafNodeIdx);
        nd->offset_right_node    = 0; // leaf
        nd->offset_compact_ledge = ledgeAbsOffK - leafAbsOff;
        FillNodeBounds(nd, hMin, hMax);
    }

    off += nodeAreaSize;

    return off;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

Convex* ConvexFromVerts(const Vec3* verts, int vertCount, std::string* err)
{
    if (err) err->clear();
    if (!verts || vertCount < 1) return nullptr;

    std::vector<float> fpts;
    fpts.reserve(vertCount * 3);
    for (int i = 0; i < vertCount; i++) {
        fpts.push_back(verts[i].x);
        fpts.push_back(verts[i].y);
        fpts.push_back(verts[i].z);
    }

    ConvexHullResult res = BuildConvexHull(fpts.data(), vertCount);
    // Degenerate is not an error - the caller decides whether a piece it could
    // not hull is fatal, so *err stays empty here.
    if (res.degenerate || res.indices.empty())
        return nullptr;

    if (!HullIsValidForIVP(res)) {
        if (err) {
            *err = "convex hull (" + std::to_string(res.vertices.size()) + " verts, " +
                   std::to_string(res.indices.size() / 3) +
                   " tris) is not a closed manifold or exceeds IVP limits";
        }
        return nullptr;
    }

    Convex* c = new Convex();
    c->hull = std::move(res);
    return c;
}

float ConvexVolume(const Convex* convex)
{
    return convex ? convex->Volume() : 0.0f;
}

float ConvexSurfaceArea(const Convex* convex)
{
    return convex ? convex->SurfaceArea() : 0.0f;
}

int ConvexVertexCount(const Convex* convex)
{
    return convex ? static_cast<int>(convex->hull.vertices.size()) : 0;
}

void SetConvexGameData(Convex* convex, unsigned int gameData)
{
    if (convex) convex->gameData = gameData;
}

void ConvexFree(Convex* convex)
{
    delete convex;
}

Collide* ConvertConvexToCollide(Convex** convexes, int convexCount)
{
    if (!convexes || convexCount < 1) return nullptr;
    Collide* col = new Collide();
    for (int i = 0; i < convexCount; i++) {
        if (!convexes[i]) continue;
        col->hulls.push_back(convexes[i]);  // transfer ownership
    }
    col->ComputeMassCenter();
    return col;
}

void CollideFree(Collide* collide)
{
    delete collide;
}

int CollideSize(const Collide* collide)
{
    return collide ? CalcCollideSize(collide) : 0;
}

int CollideWrite(char* dest, const Collide* collide)
{
    if (!dest || !collide) return 0;
    return WriteCollide(dest, collide);
}

void CollideGetAABB(const Collide* collide, Vec3* mins, Vec3* maxs)
{
    if (!collide) {
        *mins = *maxs = Vec3();
        return;
    }
    collide->GetAABB(mins, maxs);
}

Vec3 CollideGetMassCenter(const Collide* collide)
{
    return collide ? collide->massCenter : Vec3();
}

void CollideSetMassCenter(Collide* collide, const Vec3& massCenter)
{
    if (!collide) return;
    collide->massCenter = massCenter;
    collide->massCenterOverridden = true;
}

float CollideVolume(const Collide* collide)
{
    return collide ? collide->Volume() : 0.0f;
}

} // namespace pulse::phys
