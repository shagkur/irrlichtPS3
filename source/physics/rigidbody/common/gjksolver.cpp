/*
 * gjksolver.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "base/simplestack.h"
#include "rigidbody/common/collprim.h"
#include "rigidbody/common/intersectfunction.h"
#include "rigidbody/common/gjksolver.h"

#ifdef __SPU__
	#include "base/heapmanager.h"
	extern HeapManager gPool;
#endif

///////////////////////////////////////////////////////////////////////////////
// Allocate Buffers

GJKSolver::GJKSolver(void *sA, void *sB, GetSupportVertexFunc fA, GetSupportVertexFunc fB)
{
	shapeA = sA;
	shapeB = sB;
	getSupportVertexShapeA = fA;
	getSupportVertexShapeB = fB;
#ifdef __SPU__
	vertsP = (Vector3*)gPool.allocate(sizeof(Vector3)*MAX_VERTS);
	vertsQ = (Vector3*)gPool.allocate(sizeof(Vector3)*MAX_VERTS);
	vertsW = (Vector3*)gPool.allocate(sizeof(Vector3)*MAX_VERTS);
	facets = (GJKFacet*)gPool.allocate(sizeof(GJKFacet)*MAX_FACETS);
	facetsHead = (GJKFacet**)gPool.allocate(sizeof(GJKFacet*)*MAX_FACETS);
	edges = (GJKEdge*)gPool.allocate(sizeof(GJKEdge)*MAX_EDGES);
#else
	vertsP = g_vertsP;
	vertsQ = g_vertsQ;
	vertsW = g_vertsW;
	facets = g_facets;
	facetsHead = g_facetsHead;
	edges = g_edges;
#endif
}

GJKSolver::~GJKSolver()
{
#ifdef __SPU__
	gPool.deallocate(edges);
	gPool.deallocate(facetsHead);
	gPool.deallocate(facets);
	gPool.deallocate(vertsW);
	gPool.deallocate(vertsQ);
	gPool.deallocate(vertsP);
#endif
}

///////////////////////////////////////////////////////////////////////////////
// Construct Silhouette

void GJKSolver::silhouette(GJKFacet *facet, s32 i, Vector3 w)
{
	SimpleStack<GJKEdge> gs;

	gs.push(GJKEdge(facet, i));

	do {
		GJKEdge stk = gs.pop();
		GJKFacet *ft = stk.f;

#ifdef __SPU__
		if(spu_extract(ft->obsolete, 0) == 0) {
#else
		if(ft->obsolete == 0) {
#endif
			if(dot(ft->normal, w-ft->closest) < 0.0f) {
				ASSERT(numEdges <= MAX_FACETS);
				edges[numEdges] = stk;
				numEdges++;
			} else {
#ifdef __SPU__
				ft->obsolete = spu_insert(1, ft->obsolete, 0);

				gs.push(GJKEdge(ft->adj[(stk.i + 2)%3], spu_extract(ft->j, (stk.i + 2)%3)));
				gs.push(GJKEdge(ft->adj[(stk.i + 1)%3], spu_extract(ft->j, (stk.i + 1)%3)));
#else
				ft->obsolete = 1;

				gs.push(GJKEdge(ft->adj[(stk.i + 2)%3], ft->j[(stk.i + 2)%3]));
				gs.push(GJKEdge(ft->adj[(stk.i + 1)%3], ft->j[(stk.i + 1)%3]));
#endif
			}
		}
	} while(UNLIKELY(!gs.isEmpty()));
}

///////////////////////////////////////////////////////////////////////////////
// Detect Penetration Depth (EPA)

f32 GJKSolver::detectPenetrationDepth(const Transform3& transformA, const Transform3& transformB, Vector3& pA, Vector3& pB, Vector3& normal)
{
	Matrix3 invRotA = transpose(transformA.getUpper3x3());
	Matrix3 invRotB = transpose(transformB.getUpper3x3());

	s32 epaIterationCount = 0;
	f32 distance = FLT_MAX;

	numFacets = 0;
	numFacetsHead = 0;

	// 初期状態の判定
	if(mSimplex.numVertices <= 1)
		return distance;
	else if(mSimplex.numVertices == 2) {
		Vector3 v0 = mSimplex.W[0];
		Vector3 v1 = mSimplex.W[1];
		Vector3 dir = normalize(v1 - v0);
		Matrix3 rot = Matrix3::rotation(2.0943951023932f, dir);//120 deg
		s32 axis;
		if(dir[0] < dir[1]) {
			if(dir[0] < dir[2])
				axis = 0;
			else
				axis = 2;
		} else {
			if(dir[1] < dir[2])
				axis = 1;
			else
				axis = 2;
		}
        Vector3 vec(0.0f);
		vec[axis] = 1.0f;

		Vector3 aux[3];
		aux[0] = cross(dir,vec);
		aux[1] = rot * aux[0];
		aux[2] = rot * aux[1];

		Vector3 p[3],q[3],w[3];

		for(s32 i=0;i < 3;i++) {
			Vector3 pInA, qInB;
			getSupportVertexShapeA(shapeA, invRotA*aux[i], pInA);
			getSupportVertexShapeB(shapeB, invRotB*(-aux[i]), qInB);
			p[i] = transformA.getTranslation() + transformA.getUpper3x3()*pInA;
			q[i] = transformB.getTranslation() + transformB.getUpper3x3()*qInB;
			w[i] = p[i] - q[i];
			vertsP[i] = p[i];
			vertsQ[i] = q[i];
			vertsW[i] = w[i];
		}

		if(originInTetrahedron(w[0], w[1], w[2], v0)) {
			vertsP[3] = mSimplex.P[0];
			vertsQ[3] = mSimplex.Q[0];
			vertsW[3] = mSimplex.W[0];
			numVerts = 4;
		} else if(originInTetrahedron(w[0], w[1], w[2], v1)) {
			vertsP[3] = mSimplex.P[1];
			vertsQ[3] = mSimplex.Q[1];
			vertsW[3] = mSimplex.W[1];
			numVerts = 4;
		} else
			return distance;
	} else if(mSimplex.numVertices == 3) {
		numVerts = 3;
		for(s32 i=0;i < numVerts;i++) {
			vertsP[i] = mSimplex.P[i];
			vertsQ[i] = mSimplex.Q[i];
			vertsW[i] = mSimplex.W[i];
		}

		Vector3 p[2], q[2], w[2];
		{
			Vector3 v = cross(vertsW[2] - vertsW[0], vertsW[1] - vertsW[0]);
			Vector3 pInA, qInB;
			getSupportVertexShapeA(shapeA, invRotA*v, pInA);
			getSupportVertexShapeB(shapeB, invRotB*(-v), qInB);
			p[0] = transformA.getTranslation() + transformA.getUpper3x3()*pInA;
			q[0] = transformB.getTranslation() + transformB.getUpper3x3()*qInB;
			w[0] = p[0] - q[0];
			getSupportVertexShapeA(shapeA, invRotA*(-v), pInA);
			getSupportVertexShapeB(shapeB, invRotB*v, qInB);
			p[1] = transformA.getTranslation() + transformA.getUpper3x3()*pInA;
			q[1] = transformB.getTranslation() + transformB.getUpper3x3()*qInB;
			w[1] = p[1] - q[1];
		}

		if(originInTetrahedron(vertsW[0], vertsW[1], vertsW[2], w[0])) {
			vertsP[3] = p[0];
			vertsQ[3] = q[0];
			vertsW[3] = w[0];
			numVerts = 4;
		} else if(originInTetrahedron(vertsW[0], vertsW[1], vertsW[2], w[1])) {
			vertsP[3] = p[1];
			vertsQ[3] = q[1];
			vertsW[3] = w[1];
			numVerts = 4;
		} else
			return distance;
	} else {
		numVerts = 4;
		for(s32 i=0;i < numVerts;i++) {
			vertsP[i] = mSimplex.P[i];
			vertsQ[i] = mSimplex.Q[i];
			vertsW[i] = mSimplex.W[i];
		}
	}

	ASSERT(numVerts == 4);

	if(UNLIKELY(!originInTetrahedron(vertsW[0], vertsW[1], vertsW[2], vertsW[3])))
		return distance;

	if(dot(-vertsW[0], cross(vertsW[2] - vertsW[0], vertsW[1] - vertsW[0])) > 0.0f) {
		Vector3 vertsP1, vertsQ1, vertsW1;
		Vector3 vertsP3, vertsQ3, vertsW3;
		vertsQ1 = vertsQ[1]; vertsW1 = vertsW[1]; vertsP1 = vertsP[1];
		vertsQ3 = vertsQ[3]; vertsW3 = vertsW[3]; vertsP3 = vertsP[3];
		vertsQ[1] =vertsQ3; vertsW[1] = vertsW3; vertsP[1] = vertsP3;
		vertsQ[3] =vertsQ1; vertsW[3] = vertsW1; vertsP[3] = vertsP1;
	}

	{
#ifdef __SPU__
		GJKFacet *f0 = addFacet(((vec_int4){0, 1, 2, 0}));
		GJKFacet *f1 = addFacet(((vec_int4){0, 3, 1, 0}));
		GJKFacet *f2 = addFacet(((vec_int4){0, 2, 3, 0}));
		GJKFacet *f3 = addFacet(((vec_int4){1, 3, 2, 0}));
#else
		GJKFacet *f0 = addFacet(0, 1, 2);
		GJKFacet *f1 = addFacet(0, 3, 1);
		GJKFacet *f2 = addFacet(0, 2, 3);
		GJKFacet *f3 = addFacet(1, 3, 2);
#endif

		if(UNLIKELY(!f0 || !f1 || !f2 || !f3)) return distance;

		linkFacets(f0, 0, f1, 2);
		linkFacets(f0, 1, f3, 2);
		linkFacets(f0, 2, f2, 0);
		linkFacets(f1, 0, f2, 2);
		linkFacets(f1, 1, f3, 0);
		linkFacets(f2, 1, f3, 1);
	}

	GJKFacet *facetMin = NULL;

	do {
		s32 minFacetIdx = 0;
		{
#ifdef __SPU__
			vec_float4 vMinDistSqr = spu_splats(FLT_MAX);
			qword vFacetMinPtr = (qword)spu_splats(0);
			qword vMinFacetIdx = (qword)spu_splats(0);
			for(s32 i=0;i < numFacetsHead;i++) {
				vec_float4 distSqr = facetsHead[i]->distSqr;
				vec_uint4 retEq = spu_cmpeq(facetsHead[i]->obsolete, 0);
				vec_uint4 retGt = spu_cmpgt(vMinDistSqr, distSqr);
				vec_uint4 retAd = spu_and(retEq, retGt);
				vMinDistSqr = spu_sel(vMinDistSqr, distSqr, retAd);
				vFacetMinPtr = si_selb(vFacetMinPtr, si_from_ptr(facetsHead[i]), (qword)retAd);
				vMinFacetIdx = si_selb(vMinFacetIdx, si_from_int(i), (qword)retAd);
			}
			facetMin = (GJKFacet*)si_to_ptr(vFacetMinPtr);
			minFacetIdx = si_to_int(vMinFacetIdx);
#else
			f32 minDistSqr = FLT_MAX;
			for(s32 i=0;i < numFacetsHead;i++) {
				if(facetsHead[i]->obsolete == 0 && facetsHead[i]->distSqr < minDistSqr) {
					minDistSqr = facetsHead[i]->distSqr;
					facetMin = facetsHead[i];
					minFacetIdx = i;
				}
			}
#endif
		}

		facetsHead[minFacetIdx] = facetsHead[--numFacetsHead];

		Vector3 pInA(0.0f), qInB(0.0f);
		getSupportVertexShapeA(shapeA, invRotA*facetMin->normal, pInA);
		getSupportVertexShapeB(shapeB, invRotB*(-facetMin->normal), qInB);
		Vector3 p = transformA.getTranslation() + transformA.getUpper3x3()*pInA;
		Vector3 q = transformB.getTranslation() + transformB.getUpper3x3()*qInB;
		Vector3 w = p - q;
		Vector3 v = facetMin->closest;

		f32 l0 = length(v);
		f32 l1 = dot(facetMin->normal, w);

		if((l1 - l0) < GJK_EPSILON)
			break;

		{
			ASSERT(numVerts < MAX_VERTS);
			s32 vId = numVerts++;
			vertsP[vId] = p;
			vertsQ[vId] = q;
			vertsW[vId] = w;

#ifdef __SPU__
				facetMin->obsolete = spu_insert(1, facetMin->obsolete, 0);
#else
				facetMin->obsolete = 1;
#endif
			numEdges = 0;
#ifdef __SPU__
			silhouette(facetMin->adj[0], spu_extract(facetMin->j, 0), w);
			silhouette(facetMin->adj[1], spu_extract(facetMin->j, 1), w);
			silhouette(facetMin->adj[2], spu_extract(facetMin->j, 2), w);
#else
			silhouette(facetMin->adj[0], facetMin->j[0], w);
			silhouette(facetMin->adj[1], facetMin->j[1], w);
			silhouette(facetMin->adj[2], facetMin->j[2], w);
#endif
			if(UNLIKELY(numEdges == 0)) break;

			bool edgeCheck = true;
			GJKFacet *firstFacet, *lastFacet;
			{
				GJKEdge& edge = edges[0];
#ifdef __SPU__
				s32 v0 = spu_extract(edge.f->v, (edge.i + 1)%3);
				s32 v1 = spu_extract(edge.f->v, edge.i);
				firstFacet = addFacet(((vec_int4){v0, v1, vId, 0}));
#else
				s32 v0 = edge.f->v[(edge.i + 1)%3];
				s32 v1 = edge.f->v[edge.i];
				firstFacet = addFacet(v0, v1, vId);
#endif
				if(UNLIKELY(!firstFacet)) {
					edgeCheck = false;
					break;
				}
				linkFacets(edge.f, edge.i, firstFacet, 0);
				lastFacet = firstFacet;
			}

			if(UNLIKELY(!edgeCheck)) break;

			for(s32 e=1;e < numEdges;e++) {
				GJKEdge& edge = edges[e];
#ifdef __SPU__
				s32 v0 = spu_extract(edge.f->v, (edge.i + 1)%3);
				s32 v1 = spu_extract(edge.f->v ,edge.i);
				GJKFacet *f = addFacet(((vec_int4){v0, v1, vId, 0}));
#else
				s32 v0 = edge.f->v[(edge.i + 1)%3];
				s32 v1 = edge.f->v[edge.i];
				GJKFacet *f = addFacet(v0, v1, vId);
#endif
				if(UNLIKELY(!f)) {edgeCheck = false; break;}
				linkFacets(edge.f, edge.i, f, 0);
				linkFacets(f, 2, lastFacet, 1);
				lastFacet = f;
			}
			if(UNLIKELY(!edgeCheck)) break;

			linkFacets(lastFacet, 1, firstFacet, 2);
		}

		epaIterationCount++;
		if(UNLIKELY(epaIterationCount > EPA_ITERATION_MAX || numFacetsHead == 0))
			break;
	} while(1);

#ifdef __SPU__
	s32 v1 = spu_extract(facetMin->v, 0);
	s32 v2 = spu_extract(facetMin->v, 1);
	s32 v3 = spu_extract(facetMin->v, 2);
#else
	s32 v1 = facetMin->v[0];
	s32 v2 = facetMin->v[1];
	s32 v3 = facetMin->v[2];
#endif
	Vector3 p0 = vertsW[v2]-vertsW[v1];
	Vector3 p1 = vertsW[v3]-vertsW[v1];
	Vector3 p2 = facetMin->closest-vertsW[v1];

	Vector3 v = cross(p0, p1);
	Vector3 crS = cross(v, p0);
	Vector3 crT = cross(v, p1);
	f32 d0 = dot(crT, p0);
	f32 d1 = dot(crS, p1);

	if(fabsf(d0) < GJK_EPSILON || fabsf(d1) < GJK_EPSILON) return distance;

	f32 lamda1 = dot(crT, p2)/d0;
	f32 lamda2 = dot(crS, p2)/d1;

	pA = vertsP[v1] + lamda1*(vertsP[v2] - vertsP[v1]) + lamda2*(vertsP[v3] - vertsP[v1]);
	pB = vertsQ[v1] + lamda1*(vertsQ[v2] - vertsQ[v1]) + lamda2*(vertsQ[v3] - vertsQ[v1]);

	f32 lenSqr = lengthSqr(pB - pA);
	normal = normalize(pB - pA);

	return -sqrtf(lenSqr);
}

///////////////////////////////////////////////////////////////////////////////
// GJK

f32 GJKSolver::collide(Vector3& normal, Point3 &pointA, Point3 &pointB, const Transform3& transformA, const Transform3& transformB, f32 distanceThreshold)
{
	(void) distanceThreshold;

	int gjkIterationCount = 0;

	mSimplex.reset();

	Transform3 cTransformA = transformA;
	Transform3 cTransformB = transformB;
	Matrix3 invRotA = transpose(cTransformA.getUpper3x3());
	Matrix3 invRotB = transpose(cTransformB.getUpper3x3());

	Vector3 offset = (cTransformA.getTranslation() + cTransformB.getTranslation())*0.5f;
	cTransformA.setTranslation(cTransformA.getTranslation() - offset);
	cTransformB.setTranslation(cTransformB.getTranslation() - offset);

	Vector3 separatingAxis(-cTransformA.getTranslation());
	if(lengthSqr(separatingAxis) < 0.000001f) separatingAxis = Vector3(1, 0, 0);
	f32 squaredDistance = FLT_MAX;
	f32 delta = 0.0f;
	f32 distance = FLT_MAX;

	for(;;) {
		Vector3 pInA, qInB;

		getSupportVertexShapeA(shapeA, invRotA*(-separatingAxis), pInA);
		getSupportVertexShapeB(shapeB, invRotB*separatingAxis, qInB);

		Vector3 p = cTransformA.getTranslation() + cTransformA.getUpper3x3()*pInA;
		Vector3 q = cTransformB.getTranslation() + cTransformB.getUpper3x3()*qInB;
		Vector3 w = p - q;

		delta = dot(separatingAxis, w);

		if(UNLIKELY(delta > 0.0f)) {
			normal = separatingAxis;
			return distance;
		}

		if(UNLIKELY(mSimplex.inSimplex(w)))
			break;

		f32 f0 = squaredDistance - delta;
		f32 f1 = squaredDistance*GJK_EPSILON;

		if(UNLIKELY(f0 <= f1))
			break;

		mSimplex.addVertex(w, p, q);

		if(UNLIKELY(!mSimplex.closest(separatingAxis))) {
			normal = separatingAxis;
			return distance;
		}

		squaredDistance = lengthSqr(separatingAxis);

		if(UNLIKELY(gjkIterationCount >= GJK_ITERATION_MAX || mSimplex.fullSimplex()))
			break;

		gjkIterationCount++;
	}

	Vector3 pA(0.0f), pB(0.0f);

	f32 dist = detectPenetrationDepth(cTransformA, cTransformB, pA, pB, normal);

	if(dist < 0.0f) {
		pA += normal*GJK_MARGIN;
		pB -= normal*GJK_MARGIN;
		dist = dot(normal, pA - pB);
		pointA = orthoInverse(transformA)*Point3(pA + offset);
		pointB = orthoInverse(transformB)*Point3(pB + offset);
	}

	return dist;
}
