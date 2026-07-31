// The ONE translation unit that compiles the VHACD single-header library.
//
// It exists so more than one consumer can use VHACD without each defining
// ENABLE_VHACD_IMPLEMENTATION (which would duplicate every symbol at link time):
// minicollision wants the hull builder, the compile stage wants the decomposition
// API behind PhysicsShapeFromRender. Both get declarations from vhacd_hull.h and
// their definitions from here.
//
// Keeping DecomposeConvex in this TU (rather than one that only includes the
// declarations) means the decomposer is the single place the compiler talks to
// VHACD's decomposition API - swapping VHACD for another decomposer only touches
// this file.

#include "vhacd_hull.h"
#include "pulselimits.h"

#include <cstdint>
#include <vector>

#define ENABLE_VHACD_IMPLEMENTATION 1
#include "VHACD.h"

namespace VHACDHull
{

bool ComputeHull( const float *rawPts, int numPts, int maxVerts, HullResult &out )
{
	out.vertices.clear();
	out.indices.clear();

	if ( !rawPts || numPts < 4 )
		return false;

	std::vector<VHACD::Vertex> cloud;
	cloud.reserve( numPts );
	for ( int i = 0; i < numPts; i++ )
		cloud.emplace_back( rawPts[i*3], rawPts[i*3+1], rawPts[i*3+2] );

	// The uncapped value must be INT_MAX, not UINT_MAX: ConvexHull's
	// maxVertexCount is an int (VHACD.h), so 0xFFFFFFFF arrives as -1 and
	// the build loop (VHACD.h) exits before adding a fifth point - every
	// hull comes back a tetrahedron.
	VHACD::QuickHull qh;
	uint32_t nTris = qh.ComputeConvexHull( cloud, maxVerts > 0 ? (uint32_t)maxVerts : 0x7FFFFFFFu );
	if ( nTris < 4 )
		return false;

	// VHACD hands back its working vertex pool, which can hold points no face
	// references. Emit only the referenced ones, so the caller's point count
	// matches the hull (IVP sizes its ledge from it).
	const std::vector<VHACD::Vertex>   &verts = qh.GetVertices();
	const std::vector<VHACD::Triangle> &tris  = qh.GetIndices();

	std::vector<int> remap( verts.size(), -1 );
	auto Emit = [&]( uint32_t vi ) -> int
	{
		if ( vi >= remap.size() )
			return -1;
		if ( remap[vi] < 0 )
		{
			remap[vi] = (int)( out.vertices.size() / 3 );
			out.vertices.push_back( (float)verts[vi].mX );
			out.vertices.push_back( (float)verts[vi].mY );
			out.vertices.push_back( (float)verts[vi].mZ );
		}
		return remap[vi];
	};

	out.indices.reserve( tris.size() * 3 );
	for ( const VHACD::Triangle &t : tris )
	{
		int a = Emit( t.mI0 ), b = Emit( t.mI1 ), c = Emit( t.mI2 );
		if ( a < 0 || b < 0 || c < 0 )
			return false;
		out.indices.push_back( a );
		out.indices.push_back( b );
		out.indices.push_back( c );
	}

	return out.vertices.size() >= 4 * 3 && out.indices.size() >= 4 * 3;
}

namespace
{
	// Swallow VHACD's progress/log chatter so a compile stays quiet.
	class NullLogger : public VHACD::IVHACD::IUserLogger
	{
	public:
		void Log( const char * /*msg*/ ) override {}
	};
}

bool DecomposeConvex( const float *verts, int numVerts,
                      const int *tris, int numTris,
                      float concavity, int maxHulls, float decimate,
                      std::vector<DecomposedHull> &out )
{
	out.clear();

	if ( !verts || !tris || numVerts < 4 || numTris < 1 )
		return false;

	std::vector<double> points;
	points.reserve( (size_t)numVerts * 3 );
	for ( int i = 0; i < numVerts * 3; i++ )
		points.push_back( verts[i] );

	// Drop any triangle that indexes outside the cloud rather than trusting the
	// caller - VHACD reads these unchecked.
	std::vector<uint32_t> indices;
	indices.reserve( (size_t)numTris * 3 );
	for ( int i = 0; i < numTris; i++ )
	{
		int a = tris[i*3], b = tris[i*3+1], c = tris[i*3+2];
		if ( a < 0 || b < 0 || c < 0 || a >= numVerts || b >= numVerts || c >= numVerts )
			continue;
		indices.push_back( (uint32_t)a );
		indices.push_back( (uint32_t)b );
		indices.push_back( (uint32_t)c );
	}
	if ( indices.empty() )
		return false;

	NullLogger logger;
	VHACD::IVHACD::Parameters params;
	params.m_logger     = &logger;
	params.m_shrinkWrap = true;

	// High enough that it never shapes the result - it only stops a hull from
	// exceeding what IVP can index.
	params.m_maxNumVerticesPerCH = (uint32_t)pulse::limits::kMaxHullVerts;

	// Map the caller's [0..1] concavity onto VHACD's allowed volume error (a
	// percent), banded ~0.1% (very tight) .. ~10% (very coarse).
	float c = concavity < 0.0f ? 0.0f : ( concavity > 1.0f ? 1.0f : concavity );
	params.m_minimumVolumePercentErrorAllowed = 0.1 + (double)c * ( 10.0 - 0.1 );

	// maxHulls <= 0 means "keep whatever the split produced". VHACD only runs
	// its merge pass when the hull count EXCEEDS m_maxConvexHulls, so a ceiling
	// nothing can reach disables merging outright - there is no separate off
	// switch for it.
	const bool unlimitedHulls = ( maxHulls < 1 );
	params.m_maxConvexHulls = unlimitedHulls ? 0xFFFFFFFFu : (uint32_t)maxHulls;

	// The voxel grid decides how faithfully the shape is captured and where the
	// split planes land, so it runs at full detail and scales up with hull
	// count - more pieces need a finer grid to separate cleanly. Uncapped means
	// "however many it finds", which wants the finest grid on offer.
	uint32_t res = 1000000u;
	if ( !unlimitedHulls )
	{
		res = 400000u + (uint32_t)( maxHulls - 1 ) * 100000u;
		if ( res > 1000000u ) res = 1000000u;
	}
	params.m_resolution = res;

	VHACD::IVHACD *pVHACD = VHACD::CreateVHACD();
	if ( !pVHACD )
		return false;

	if ( pVHACD->Compute( points.data(), (uint32_t)( points.size() / 3 ),
	                      indices.data(), (uint32_t)( indices.size() / 3 ), params ) )
	{
		const uint32_t nHulls = pVHACD->GetNConvexHulls();
		for ( uint32_t h = 0; h < nHulls; h++ )
		{
			VHACD::IVHACD::ConvexHull ch;
			if ( !pVHACD->GetConvexHull( h, ch ) || ch.m_points.size() < 4 )
				continue;

			std::vector<float> pts;
			pts.reserve( ch.m_points.size() * 3 );
			for ( const VHACD::Vertex &p : ch.m_points )
			{
				pts.push_back( (float)p.mX );
				pts.push_back( (float)p.mY );
				pts.push_back( (float)p.mZ );
			}

			// Reduce against THIS hull's own vert count, not a flat budget, so a
			// dense blob is cut hard while an already-simple piece is left alone.
			// Floor of 8 (a box): 4 would rebuild a box as a tetrahedron and lose
			// a third of its volume. A failed reduce keeps the full cloud - too
			// dense beats degenerate.
			const int nNatural = (int)ch.m_points.size();
			if ( decimate > 0.0f && decimate < 1.0f )
			{
				int nTarget = (int)( decimate * (float)nNatural + 0.5f );
				if ( nTarget < 8 ) nTarget = 8;
				if ( nTarget < nNatural )
				{
					HullResult reduced;
					if ( ComputeHull( pts.data(), nNatural, nTarget, reduced ) &&
					     reduced.vertices.size() >= 4 * 3 )
						pts.swap( reduced.vertices );
				}
			}

			DecomposedHull hull;
			hull.vertices.swap( pts );
			out.push_back( std::move( hull ) );
		}
	}

	pVHACD->Clean();
	pVHACD->Release();

	return !out.empty();
}

} // namespace VHACDHull
