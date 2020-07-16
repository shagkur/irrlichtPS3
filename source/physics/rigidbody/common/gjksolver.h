/*
 * gjksolver.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef GJKSOLVER_H_
#define GJKSOLVER_H_

#include "base/common.h"
#include "base/simdfunc.h"
#include "rigidbody/common/simplexsolver.h"

#define GJK_EPSILON					1e-04f
#define GJK_MARGIN					0.025f
#define GJK_ITERATION_MAX			10
#define EPA_ITERATION_MAX			10

/*
	Memo
	pool 13280
	stack 5232 sizeof(PfxGJKSolver) 13856
	*/

///////////////////////////////////////////////////////////////////////////////
// Support Function

typedef void (*GetSupportVertexFunc)(void *shape, Vector3 seperatingAxis, Vector3& supportVertex);

///////////////////////////////////////////////////////////////////////////////
// GJK

class SimplexSolver;

class GJKSolver
{
private:
	GJKSolver() {}

	static const s32   MAX_VERTS = 128;
	static const s32   MAX_EDGES = 128;
	static const s32   MAX_FACETS = 64;

	SimplexSolver mSimplex;

	struct GJKFacet
	{
		Vector3 normal;
		Vector3 closest;
#ifdef __SPU__
		vec_uint4  obsolete;
		vec_float4 distSqr;
		vec_int4 v;
		vec_int4 j;
		GJKFacet *adj[3];
#else
		uint32_t obsolete;
		f32 distSqr;
		s32 v[3];
		s32 j[3];
		GJKFacet *adj[3];
#endif
	};

	struct GJKEdge
	{
		GJKFacet *f;
		s32 i;
		GJKEdge() {}
		GJKEdge(GJKFacet *f_, s32 i_)
		{
			f = f_;
			i= i_;
		}
	};

	#ifndef __SPU__
	Vector3 g_vertsP[MAX_VERTS];
	Vector3 g_vertsQ[MAX_VERTS];
	Vector3 g_vertsW[MAX_VERTS];
	GJKFacet g_facets[MAX_FACETS];
	GJKFacet *g_facetsHead[MAX_FACETS];
	GJKEdge  g_edges[MAX_EDGES];
	#endif

	ATTRIBUTE_ALIGNED16(Vector3 *vertsP);
	ATTRIBUTE_ALIGNED16(Vector3 *vertsQ);
	ATTRIBUTE_ALIGNED16(Vector3 *vertsW);
	ATTRIBUTE_ALIGNED16(GJKFacet *facets);
	ATTRIBUTE_ALIGNED16(GJKFacet **facetsHead);
	ATTRIBUTE_ALIGNED16(GJKEdge *edges);

	s32 numVerts;
	s32 numEdges;
	s32 numFacets;
	s32 numFacetsHead;

#ifdef __SPU__
	inline GJKFacet *addFacet(vec_int4 v);
#else
	inline GJKFacet *addFacet(s32 v1, s32 v2, s32 v3);
#endif

	inline void linkFacets(GJKFacet *f1, s32 e1, GJKFacet *f2, s32 e2);
	void silhouette(GJKFacet *facet, s32 i, Vector3 w);

	inline bool originInTetrahedron(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3);

	f32 detectPenetrationDepth(const Transform3& transformA, const Transform3& transformB, Vector3& pA, Vector3& pB, Vector3& normal);

	void *shapeA;
	void *shapeB;
	GetSupportVertexFunc getSupportVertexShapeA;
	GetSupportVertexFunc getSupportVertexShapeB;

public:
	GJKSolver(void *sA, void *sB, GetSupportVertexFunc fA, GetSupportVertexFunc fB);
	~GJKSolver();

	f32 collide(Vector3& normal, Point3& pointA, Point3& pointB, const Transform3& transformA, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);
};

inline
#ifdef __SPU__
GJKSolver::GJKFacet *GJKSolver::addFacet(vec_int4 v)
#else
GJKSolver::GJKFacet *GJKSolver::addFacet(s32 v1, s32 v2, s32 v3)
#endif
{
	if(UNLIKELY(numFacets == MAX_FACETS))
		return NULL;

	GJKFacet& facet = facets[numFacets];
#ifdef __SPU__
	Vector3 V1 = vertsW[spu_extract(v, 0)];
	Vector3 V2 = vertsW[spu_extract(v, 1)];
	Vector3 V3 = vertsW[spu_extract(v, 2)];
	facet.obsolete = spu_insert(0, facet.obsolete, 0);
	facet.v = v;
#else
	Vector3 V1 = vertsW[v1];
	Vector3 V2 = vertsW[v2];
	Vector3 V3 = vertsW[v3];
	facet.obsolete = 0;
	facet.v[0] = v1;
	facet.v[1] = v2;
	facet.v[2] = v3;
#endif

#ifdef __SPU__
	Vector3 vec0(V3 - V1);
	Vector3 vec1(V2 - V1);
	Vector3 normal(
				   ((vec0[1]*vec1[2]) - (vec0[2]*vec1[1])),
				   ((vec0[2]*vec1[0]) - (vec0[0]*vec1[2])),
				   ((vec0[0]*vec1[1]) - (vec0[1]*vec1[0]))
				  );
#else
	Vector3 normal = cross(V3 - V1, V2 - V1);
#endif

	f32 l = lengthSqr(normal);

	if(l < GJK_EPSILON*GJK_EPSILON)
		return NULL;

	normal /= sqrtf(l);
	facet.closest = dot(V1, normal)*normal;
	facet.normal = normal;

#ifdef __SPU__
	facet.distSqr = spu_insert(lengthSqr(facet.closest), facet.distSqr, 0);
#else
	facet.distSqr = lengthSqr(facet.closest);
#endif

	facetsHead[numFacetsHead++] = &facet;
	numFacets++;

	return &facet;
}

inline void GJKSolver::linkFacets(GJKFacet *f1, s32 e1, GJKFacet *f2, s32 e2)
{
	f1->adj[e1] = f2;
	f2->adj[e2] = f1;
#ifdef __SPU__
	f1->j = spu_insert(e2, f1->j, e1);
	f2->j = spu_insert(e1, f2->j, e1);
#else
	f1->j[e1] = e2;
	f2->j[e2] = e1;
#endif
}

inline bool GJKSolver::originInTetrahedron(const Vector3& p0, const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
    Vector3 n0 = cross((p1 - p0), (p2 - p0));
    Vector3 n1 = cross((p2 - p1), (p3 - p1));
    Vector3 n2 = cross((p3 - p2), (p0 - p2));
    Vector3 n3 = cross((p0 - p3), (p1 - p3));

#ifdef __SPU__
	const vec_uchar16 vmask1 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80});
	const vec_uchar16 vmask2 = ((vec_uchar16){0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13});
	vec_float4 v0 = vec_dot3(n0.get128(), p0.get128());
	vec_float4 v1 = vec_dot3(n1.get128(), p1.get128());
	vec_float4 v2 = vec_dot3(n2.get128(), p2.get128());
	vec_float4 v3 = vec_dot3(n3.get128(), p3.get128());
	vec_float4 v4 = vec_dot3(n0.get128(), (p3 - p0).get128());
	vec_float4 v5 = vec_dot3(n1.get128(), (p0 - p1).get128());
	vec_float4 v6 = vec_dot3(n2.get128(), (p1 - p2).get128());
	vec_float4 v7 = vec_dot3(n3.get128(), (p2 - p3).get128());
	vec_float4 va = spu_or(spu_shuffle(v0, v1, vmask1), spu_shuffle(v2, v3, vmask2));
	vec_float4 vb = spu_or(spu_shuffle(v4, v5, vmask1), spu_shuffle(v6, v7, vmask2));
	return spu_extract(spu_gather(spu_cmpgt(spu_splats(0.0f), spu_mul(va, vb))), 0) != 0;
#else
    return
		dot(n0, p0)*dot(n0, p3 - p0) < 0.0f &&
		dot(n1, p1)*dot(n1, p0 - p1) < 0.0f &&
		dot(n2, p2)*dot(n2, p1 - p2) < 0.0f &&
		dot(n3, p3)*dot(n3, p2 - p3) < 0.0f;
#endif
}

#endif /* GJKSOLVER_H_ */
