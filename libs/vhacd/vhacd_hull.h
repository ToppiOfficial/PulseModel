// Plain convex hull entry point backed by VHACD's hull builder.
//
// VHACD.h declares its hull classes only under ENABLE_VHACD_IMPLEMENTATION, which
// exactly one TU may define (vhacd_impl.cpp). This header lets callers reach the
// hull without pulling the 8k-line single-header library in.

#ifndef VHACD_HULL_H
#define VHACD_HULL_H

#include <vector>

namespace VHACDHull
{

struct HullResult
{
	std::vector<float> vertices;  // xyz triples, only points the faces reference
	std::vector<int>   indices;   // triangle list, CCW from outside
};

// Convex hull of a point cloud. rawPts is numPts xyz float triples.
// maxVerts caps the output vertex count (0 = no meaningful cap).
// Returns false if the cloud is degenerate (fewer than 4 non-coplanar points).
bool ComputeHull( const float *rawPts, int numPts, int maxVerts, HullResult &out );

// One piece of a convex decomposition. Only the point cloud is kept - the
// caller re-hulls it, so VHACD's own face list would be thrown away.
struct DecomposedHull
{
	std::vector<float> vertices; // xyz triples
};

// Convex decomposition of a closed triangle mesh. verts is numVerts xyz float
// triples, tris is numTris index triples into it.
//
//   concavity  [0..1] how much volume error to tolerate. Lower = tighter fit
//              and more pieces.
//   maxHulls   piece ceiling. VHACD merges down to it if it made more; it
//              never pads up, so this is a cap, not a target. 0 (or less)
//              lifts the cap entirely - keep whatever the split produced.
//   decimate   (0..1] fraction of each finished hull's *natural* vert count to
//              keep, applied per hull afterwards. <=0 skips the pass.
//   resolution voxels the shape is captured at. This is the whole running cost -
//              it is flat in triangle count - so a preview lowers it. 0 takes
//              the compiler's own scaling off maxHulls.
//
// Hulls are ceilinged at pulse::limits::kMaxHullVerts.
//
// Returns false if the mesh is unusable or nothing survived.
bool DecomposeConvex( const float *verts, int numVerts,
                      const int *tris, int numTris,
                      float concavity, int maxHulls, float decimate,
                      std::vector<DecomposedHull> &out, int resolution = 0 );

} // namespace VHACDHull

#endif // VHACD_HULL_H
