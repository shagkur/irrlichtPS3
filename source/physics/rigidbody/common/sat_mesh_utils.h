/*
 * sat_mesh_utils.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef SAT_MESH_UTILS_H_
#define SAT_MESH_UTILS_H_

#include "base/common.h"

struct Plane {
	Vector3 N; // normal
	Vector3 Q; // a point on plane

	Plane(const Vector3& n, const Vector3& q) {
		N = n;
		Q = q;
	}

	f32 onPlane(const Vector3& p) const {
		return dot((p - Q), N);
	}
};

struct ClosestPoints {
	Vector3 pA[4], pB[4];
	f32 distSqr[4];
	f32 closestDistSqr;
	s32 numPoints;

	ClosestPoints()
	{
		numPoints = 0;
		closestDistSqr = FLT_MAX;
	}

	void set(s32 i, const Vector3& pointA, const Vector3& pointB, f32 d)
	{
		pA[i] = pointA;
		pB[i] = pointB;
		distSqr[i] = d;
	}

	void add(const Vector3& pointA, const Vector3& pointB, f32 d)
	{
		const f32 epsilon = 0.00001f;
		if(closestDistSqr < d) return;

		closestDistSqr = d + epsilon;

		s32 replaceId = -1;
		f32 distMax = -FLT_MAX;
		for(s32 i=0;i < numPoints;i++) {
			if(lengthSqr(pA[i] - pointA) < epsilon)
				return;

			if(distMax < distSqr[i]) {
				distMax = distSqr[i];
				replaceId = i;
			}
		}

		replaceId = (numPoints < 4) ? (numPoints++) : replaceId;

		set(replaceId, pointA, pointB, d);
	}
};


SIMD_FORCE_INLINE void gatherVerts(const TriMesh *mesh, const u8 *selFacets, u8 numSelFacets, u8 *selVerts, u8& numSelVerts)
{
	numSelVerts = 0;

	u32 vertBitflags[(NUMMESHVERTICES + 31)/32] = {0};

#ifdef _TRY_SIMD
	vec_uint4 v_numSelVerts = spu_splats((u32)0);

	for(s32 f=0;f < (s32)numSelFacets;f++) {
		const MeshFacet& facet = mesh->facets[selFacets[f]];

		vec_uint4 v_index = ((vec_uint4){facet.vertIndices[0], facet.vertIndices[1], facet.vertIndices[2], 0});
		vec_uint4 v_bflagIndex = spu_rlmask(v_index, 5);
		vec_uint4 v_mask = spu_sl(((vec_uint4){1, 1, 1, 0}), spu_and(v_index, ((vec_uint4){31, 31, 31, 0})));
		vec_uint4 v_bflag = ((vec_uint4){
			vertBitflags[spu_extract(v_bflagIndex, 0)],
			vertBitflags[spu_extract(v_bflagIndex, 1)],
			vertBitflags[spu_extract(v_bflagIndex, 2)],
			0});

		vec_uint4 v_result = spu_and(v_bflag, v_mask);

		vertBitflags[spu_extract(v_bflagIndex, 0)] |= spu_extract(v_mask, 0);
		vertBitflags[spu_extract(v_bflagIndex, 1)] |= spu_extract(v_mask, 1);
		vertBitflags[spu_extract(v_bflagIndex, 2)] |= spu_extract(v_mask, 2);

		selVerts[spu_extract(v_numSelVerts, 0)] = spu_extract(v_index, 0);
		v_numSelVerts = spu_sel(v_numSelVerts + spu_splats(1), v_numSelVerts, spu_splats(spu_extract(v_result, 0)));
		selVerts[spu_extract(v_numSelVerts, 0)] = spu_extract(v_index, 1);
		v_numSelVerts = spu_sel(v_numSelVerts + spu_splats(1), v_numSelVerts, spu_splats(spu_extract(v_result, 1)));
		selVerts[spu_extract(v_numSelVerts, 0)] = spu_extract(v_index,2);
		v_numSelVerts = spu_sel(v_numSelVerts + spu_splats(1), v_numSelVerts, spu_splats(spu_extract(v_result, 2)));
	}

	numSelVerts = spu_extract(v_numSelVerts, 0);
#else
	for(s32 f=0;f < (s32)numSelFacets;f++) {
		const MeshFacet& facet = mesh->facets[selFacets[f]];

		for(s32 i=0;i < 3;i++) {
			u32 mask, index;

			index = facet.vertIndices[i];
			mask = 1 << (index&31);
			if((vertBitflags[index>>5]&mask) == 0) {
				vertBitflags[index>>5] |= mask;
				selVerts[numSelVerts++] = index;
			}
		}
	}
#endif
}

SIMD_FORCE_INLINE void gatherEdges(const TriMesh *mesh, const u8 *selFacets, u8 numSelFacets, u8 *selEdges, u8& numSelEdges)
{
	u32 edgeBitflags[(NUMMESHEDGES + 31)/32] = {0};
	numSelEdges = 0;

#ifdef _TRY_SIMD
	vec_uint4 v_numSelEdges = spu_splats((u32)0);

	for(u8 f=0;f < numSelFacets;f++) {
		const MeshFacet &facet = mesh->facets[selFacets[f]];

		vec_uint4 v_index = ((vec_uint4){facet.edgeIndices[0], facet.edgeIndices[1], facet.edgeIndices[2], 0});
		vec_uint4 v_bflagIndex = spu_rlmask(v_index, 5);
		vec_uint4 v_mask = spu_sl(((vec_uint4){1, 1, 1, 0}), spu_and(v_index, ((vec_uint4){31, 31, 31, 0})));
		vec_uint4 v_bflag = ((vec_uint4){
			edgeBitflags[spu_extract(v_bflagIndex, 0)],
			edgeBitflags[spu_extract(v_bflagIndex, 1)],
			edgeBitflags[spu_extract(v_bflagIndex, 2)],
			0});
		vec_uint4 v_convex = ((vec_uint4){
			mesh->edges[spu_extract(v_index, 0)].angle,
			mesh->edges[spu_extract(v_index, 1)].angle,
			mesh->edges[spu_extract(v_index, 2)].angle,
			0});

		vec_uint4 v_result = spu_sel(((vec_uint4){1,1,1,1}), spu_and(v_bflag, v_mask), spu_cmpeq(v_convex, EDGE_CONVEX));

		edgeBitflags[spu_extract(v_bflagIndex, 0)] |= spu_extract(v_mask, 0);
		edgeBitflags[spu_extract(v_bflagIndex, 1)] |= spu_extract(v_mask, 1);
		edgeBitflags[spu_extract(v_bflagIndex, 2)] |= spu_extract(v_mask, 2);

		selEdges[spu_extract(v_numSelEdges, 0)] = spu_extract(v_index, 0);
		v_numSelEdges = spu_sel(v_numSelEdges + spu_splats(1), v_numSelEdges, spu_splats(spu_extract(v_result, 0)));
		selEdges[spu_extract(v_numSelEdges, 0)] = spu_extract(v_index, 1);
		v_numSelEdges = spu_sel(v_numSelEdges + spu_splats(1), v_numSelEdges, spu_splats(spu_extract(v_result, 1)));
		selEdges[spu_extract(v_numSelEdges, 0)] = spu_extract(v_index, 2);
		v_numSelEdges = spu_sel(v_numSelEdges + spu_splats(1), v_numSelEdges, spu_splats(spu_extract(v_result, 2)));
	}
	numSelEdges = spu_extract(v_numSelEdges, 0);
#else
	for(s32 f=0;f < (s32)numSelFacets;f++) {
		const MeshFacet& facet = mesh->facets[selFacets[f]];
		for(s32 i=0;i < 3;i++) {
			u32 mask, index;
			index = facet.edgeIndices[i];
			if(mesh->edges[index].angleType == EDGE_CONVEX) {
				mask = 1<<(index&31);
				if((edgeBitflags[index>>5]&mask) == 0) {
					edgeBitflags[index>>5] |= mask;
					selEdges[numSelEdges++] = index;
				}
			}
		}
	}
#endif
}

SIMD_FORCE_INLINE void gatherEdges2(const TriMesh *mesh, const u8 *selFacets, u8 numSelFacets, u8 *selEdges, u8& numSelEdges)
{
	u32 edgeGroup[(NUMMESHEDGES + 31)/32] = {0};
	u32 edgeBitflags[(NUMMESHEDGES + 31)/32] = {0};
	numSelEdges = 0;

	for(s32 f=0;f < (s32)numSelFacets;f++) {
		const MeshFacet& facet = mesh->facets[selFacets[f]];

		for(s32 i=0;i < 3;i++) {
			u32 mask, groupId, edgeId;

			edgeId = facet.edgeIndices[i];
			MeshEdge edge = mesh->edges[edgeId];

			if(edge.angleType != EDGE_CONVEX) continue;

			mask = 1<<(edgeId&31);
			if((edgeBitflags[edgeId>>5]&mask) != 0) continue;
			edgeBitflags[edgeId>>5] |= mask;

			groupId = edge.dirGroup;
			mask = 1<<(groupId&31);
			if((edgeGroup[groupId>>5]&mask) != 0) continue;
			edgeGroup[groupId>>5] |= mask;

			selEdges[numSelEdges++] = edgeId;
		}
	}
}

SIMD_FORCE_INLINE void gatherFacets(const TriMesh *mesh, const f32 *aabbHalf, const Vector3& offsetPos, const Matrix3& offsetRot, u8 *selFacets, u8& numSelFacets)
{
	Matrix3 absOffsetRot = absPerElem(offsetRot);

#ifdef TRY_SIMD
	Vector3 v_aabbHalf(read_Vector3(aabbHalf));
	vec_uint4 v_numSelFacets = spu_splats((u32)0);
	s32 f = 0;
	for(;f < (s32)mesh->numFacets - 4;f+=4) {
		const MeshFacet& facet0 = mesh->facets[f + 0];
		const MeshFacet& facet1 = mesh->facets[f + 1];
		const MeshFacet& facet2 = mesh->facets[f + 2];
		const MeshFacet& facet3 = mesh->facets[f + 3];

		Vector3 facetCenter0 = absPerElem(offsetPos + offsetRot*read_Vector3(facet0.center));
		Vector3 facetCenter1 = absPerElem(offsetPos + offsetRot*read_Vector3(facet1.center));
		Vector3 facetCenter2 = absPerElem(offsetPos + offsetRot*read_Vector3(facet2.center));
		Vector3 facetCenter3 = absPerElem(offsetPos + offsetRot*read_Vector3(facet3.center));

		Vector3 half0 = absOffsetRot*read_Vector3(facet0.half);
		Vector3 half1 = absOffsetRot*read_Vector3(facet1.half);
		Vector3 half2 = absOffsetRot*read_Vector3(facet2.half);
		Vector3 half3 = absOffsetRot*read_Vector3(facet3.half);

		u32 isCheck0 = !cmpgtAny(facetCenter0, half0 + v_aabbHalf);
		u32 isCheck1 = !cmpgtAny(facetCenter1, half1 + v_aabbHalf);
		u32 isCheck2 = !cmpgtAny(facetCenter2, half2 + v_aabbHalf);
		u32 isCheck3 = !cmpgtAny(facetCenter3, half3 + v_aabbHalf);

		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)(f + 0);
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck0), 0));
		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)(f + 1);
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck1), 0));
		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)(f + 2);
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck2), 0));
		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)(f + 3);
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck3), 0));
	}
	for(;f < mesh->numFacets;f++) {
		const MeshFacet& facet = mesh->facets[f];
		Vector3 facetCenter = absPerElem(offsetPos + offsetRot*read_Vector3(facet.center));
		Vector3 facetHalf = absOffsetRot*read_Vector3(facet.half);

		u32 isCheck = !cmpgtAny(facetCenter, facetHalf + v_aabbHalf);

		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)f;
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck), 0));
	}
	numSelFacets = spu_extract(v_numSelFacets, 0);
#else
	for(s32 f=0;f < (s32)mesh->numFacets;f++) {
		const MeshFacet& facet = mesh->facets[f];

		Vector3 facetCenter = absPerElem(offsetPos + offsetRot*read_Vector3(facet.center));
		Vector3 halfBA = absOffsetRot*read_Vector3(facet.half);

		if(facetCenter[0] > (halfBA[0] + aabbHalf[0])) continue;
		if(facetCenter[1] > (halfBA[1] + aabbHalf[1])) continue;
		if(facetCenter[2] > (halfBA[2] + aabbHalf[2])) continue;

		selFacets[numSelFacets++] = (u8)f;
	}
#endif
}

SIMD_FORCE_INLINE void gatherFacets(const TriMesh *mesh, const f32 *aabbHalf, const Vector3& offsetPos, const Matrix3& offsetRot, u8 *selFacets, u8& numSelFacets, u8 *selVerts, u8& numSelVerts)
{
	u32 vertBitflags[(NUMMESHVERTICES + 31)/32] = {0};
	Matrix3 absOffsetRot = absPerElem(offsetRot);

//#ifdef TRY_SIMD
#ifdef __SPU__
	Vector3 v_aabbHalf(read_Vector3(aabbHalf));
	vec_uint4 v_numSelFacets = spu_splats((u32)0);
	s32 f = 0;
	for(;f < (s32)mesh->numFacets - 4;f+=4) {
		const MeshFacet& facet0 = mesh->facets[f + 0];
		const MeshFacet& facet1 = mesh->facets[f + 1];
		const MeshFacet& facet2 = mesh->facets[f + 2];
		const MeshFacet& facet3 = mesh->facets[f + 3];

		Vector3 facetCenter0 = absPerElem(offsetPos + offsetRot*read_Vector3(facet0.center));
		Vector3 facetCenter1 = absPerElem(offsetPos + offsetRot*read_Vector3(facet1.center));
		Vector3 facetCenter2 = absPerElem(offsetPos + offsetRot*read_Vector3(facet2.center));
		Vector3 facetCenter3 = absPerElem(offsetPos + offsetRot*read_Vector3(facet3.center));

		Vector3 half0 = absOffsetRot*read_Vector3(facet0.half);
		Vector3 half1 = absOffsetRot*read_Vector3(facet1.half);
		Vector3 half2 = absOffsetRot*read_Vector3(facet2.half);
		Vector3 half3 = absOffsetRot*read_Vector3(facet3.half);

		u32 isCheck0 = !cmpgtAny(facetCenter0, half0 + v_aabbHalf);
		u32 isCheck1 = !cmpgtAny(facetCenter1, half1 + v_aabbHalf);
		u32 isCheck2 = !cmpgtAny(facetCenter2, half2 + v_aabbHalf);
		u32 isCheck3 = !cmpgtAny(facetCenter3, half3 + v_aabbHalf);

		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)(f + 0);
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck0), 0));
		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)(f + 1);
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck1), 0));
		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)(f + 2);
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck2), 0));
		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)(f + 3);
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck3), 0));
	}
	for(;f < (s32)mesh->numFacets;f++) {
		const MeshFacet& facet = mesh->facets[f];
		Vector3 facetCenter = absPerElem(offsetPos + offsetRot*read_Vector3(facet.center));
		Vector3 facetHalf = absOffsetRot*read_Vector3(facet.half);

		u32 isCheck = !cmpgtAny(facetCenter, facetHalf + v_aabbHalf);

		selFacets[spu_extract(v_numSelFacets, 0)] = (u8)f;
		v_numSelFacets = spu_sel(v_numSelFacets, v_numSelFacets + spu_splats(1), spu_cmpgt(spu_splats(isCheck), 0));
	}
	numSelFacets = spu_extract(v_numSelFacets, 0);

	for(f=0;f < (s32)numSelFacets;f++) {
		const MeshFacet& facet = mesh->facets[selFacets[f]];

		for(s32 i=0;i < 3;i++) {
			u32 mask, index;

			index = facet.vertIndices[i];
			mask = 1<<(index&31);
			if((vertBitflags[index>>5]&mask) == 0) {
				vertBitflags[index>>5] |= mask;
				selVerts[numSelVerts++] = index;
			}
		}
	}
#else
	for(s32 f=0;f < (s32)mesh->numFacets;f++) {
		const MeshFacet& facet = mesh->facets[f];

		Vector3 facetCenter = absPerElem(offsetPos + offsetRot*Vector3(facet.center[0], facet.center[1], facet.center[2]));
		Vector3 halfBA = absOffsetRot*Vector3(facet.half[0], facet.half[1], facet.half[2]);

		if(facetCenter[0] > (halfBA[0] + aabbHalf[0])) continue;
		if(facetCenter[1] > (halfBA[1] + aabbHalf[1])) continue;
		if(facetCenter[2] > (halfBA[2] + aabbHalf[2])) continue;

		selFacets[numSelFacets++] = (u8)f;

		for(s32 i=0;i<3;i++) {
			u32 mask, index;

			index = facet.vertIndices[i];
			mask = 1<<(index&31);
			if((vertBitflags[index>>5]&mask) == 0) {
				vertBitflags[index>>5] |= mask;
				selVerts[numSelVerts++] = index;
			}
		}
	}
#endif
}

SIMD_FORCE_INLINE void gatherFacetsFromFacetGroup(u8 groupId, const TriMesh *mesh, const u8 *selFacets, u8 numSelFacets, u8 *selGpFacets, u8& numSelGpFacets)
{
#ifdef TRY_SIMD
	vec_uint4 v_numSelGpFacets = ((vec_uint4){0, 0, 0, 0});
	vec_uint4 v_groupId = spu_splats((u32)groupId);

	s32 f=0;
	for(;f < (s32)numSelFacets - 4;f+=4) {
		vec_uint4 v_curindex = ((vec_uint4){selFacets[f + 0], selFacets[f + 1], selFacets[f + 2], selFacets[f + 3]});

		const MeshFacet& f0 = mesh->facets[spu_extract(v_curindex, 0)];
		const MeshFacet& f1 = mesh->facets[spu_extract(v_curindex, 1)];
		const MeshFacet& f2 = mesh->facets[spu_extract(v_curindex, 2)];
		const MeshFacet& f3 = mesh->facets[spu_extract(v_curindex, 3)];

		vec_uint4 v_curgroup = ((vec_uint4){f0.dirGroup, f1.dirGroup, f2.dirGroup, f3.dirGroup}9;
		vec_uint4 v_result = spu_cmpeq(v_curgroup, v_groupId);

		selGpFacets[spu_extract(v_numSelGpFacets, 0)] = spu_extract(v_curindex, 0);
		v_numSelGpFacets = spu_sel(v_numSelGpFacets, v_numSelGpFacets + spu_splats(1), spu_splats(spu_extract(v_result, 0)));
		selGpFacets[spu_extract(v_numSelGpFacets, 0)] = spu_extract(v_curindex, 1);
		v_numSelGpFacets = spu_sel(v_numSelGpFacets, v_numSelGpFacets + spu_splats(1), spu_splats(spu_extract(v_result, 1)));
		selGpFacets[spu_extract(v_numSelGpFacets, 0)] = spu_extract(v_curindex, 2);
		v_numSelGpFacets = spu_sel(v_numSelGpFacets, v_numSelGpFacets + spu_splats(1),spu_splats(spu_extract(v_result, 2)));
		selGpFacets[spu_extract(v_numSelGpFacets, 0)] = spu_extract(v_curindex, 3);
		v_numSelGpFacets = spu_sel(v_numSelGpFacets, v_numSelGpFacets + spu_splats(1),spu_splats(spu_extract(v_result, 3)));
	}
	for(;f < (s32)numSelFacets;f++) {
		vec_uint4 v_curindex = ((vec_unit4){selFacets[f], 0, 0, 0});

		const MeshFacet& facet = mesh->facets[spu_extract(v_curindex, 0)];

		vec_uint4 v_curgroup = ((vec_uint4){facet.dirGroup, 0, 0, 0});
		vec_uint4 v_result = spu_cmpeq(v_curgroup, v_groupId);

		selGpFacets[spu_extract(v_numSelGpFacets, 0)] = spu_extract(v_curindex, 0);
		v_numSelGpFacets = spu_sel(v_numSelGpFacets, v_numSelGpFacets + spu_splats(1),spu_splats(spu_extract(v_result, 0)));
	}
	numSelGpFacets = spu_extract(v_numSelGpFacets, 0);
#else
	for(s32 f=0;f < (s32)numSelFacets;f++) {
		const MeshFacet& facet = mesh->facets[selFacets[f]];
		if(groupId == facet.dirGroup)
			selGpFacets[numSelGpFacets++] = selFacets[f];
	}
#endif
}

SIMD_FORCE_INLINE void gatherFacetsFromVertId(u8 vertId, const TriMesh *mesh, const u8 *selFacets, u8 numSelFacets, u8 *selGpFacets, u8& numSelGpFacets)
{
	for(s32 f=0;f < (s32)numSelFacets;f++) {
		const MeshFacet& facet = mesh->facets[selFacets[f]];
		if(vertId == facet.vertIndices[0] || vertId == facet.vertIndices[1] || vertId == facet.vertIndices[2])
			selGpFacets[numSelGpFacets++] = selFacets[f];
	}
}

SIMD_FORCE_INLINE void gatherEdgesFromGroup(u8 groupId, const TriMesh *mesh, const u8 *selFacets, u8 numSelFacets, u8 *selEdges, u8& numSelEdges)
{
	u32 edgeBitflags[(NUMMESHEDGES + 31)/32] = {0};

	for(s32 f=0;f < (s32)numSelFacets;f++) {
		const MeshFacet& facet = mesh->facets[selFacets[f]];
		for(s32 i=0;i < 3;i++) {
			u32 mask, index;
			index = facet.edgeIndices[i];
			if(mesh->edges[index].dirGroup == groupId && mesh->edges[index].angleType == EDGE_CONVEX) {
				mask = 1<<(index&31);
				if((edgeBitflags[index>>5]&mask) == 0) {
					edgeBitflags[index>>5] |= mask;
					selEdges[numSelEdges++] = index;
				}
			}
		}
	}
}

static inline void getProjPlane(const Vector3 *verts, const u8 *vertIds, u8 numVerts, const Plane& plane, f32 &distMin)
{
//#ifdef TRY_SIMD
#ifdef __SPU__
	vec_float4 v_distMin = ((vec_float4){FLT_MAX, 0, 0, 0});
	s32 v=0;
	for(;v < (s32)numVerts - 4;v+=4) {
		vec_float4 p0 = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v + 0]]));
		vec_float4 p1 = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v + 1]]));
		vec_float4 p2 = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v + 2]]));
		vec_float4 p3 = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v + 3]]));

		vec_float4 pmin = vec_min(vec_min(p0, p1), vec_min(p2, p3));
		v_distMin = vec_min(v_distMin, pmin);
	}
	for(;v < (s32)numVerts;v++) {
		vec_float4 p = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v]]));
		v_distMin = vec_min(v_distMin, p);
	}
	distMin = spu_extract(v_distMin, 0);
#else
	distMin = FLT_MAX;
	for(s32 v=0;v < (s32)numVerts;v++) {
		f32 p = plane.onPlane(verts[vertIds[v]]);
		if(p < distMin)
			distMin = p;
	}
#endif
}

static inline void getProjPlane(const Vector3 *verts, const u8 *vertIds, u8 numVerts, const Plane& plane, f32& distMin,f32& distMax)
{
//#ifdef TRY_SIMD
#ifdef __SPU__
	vec_float4 v_distMin = ((vec_float4){FLT_MAX, 0, 0, 0});
	vec_float4 v_distMax = ((vec_float4){-FLT_MAX, 0, 0, 0});
	s32 v=0;
	for(;v < (s32)numVerts - 4;v+=4) {
		vec_float4 p0 = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v + 0]]));
		vec_float4 p1 = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v + 1]]));
		vec_float4 p2 = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v + 2]]));
		vec_float4 p3 = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v + 3]]));

		vec_float4 pmin = vec_min(vec_min(p0, p1), vec_min(p2, p3));
		v_distMin = vec_min(v_distMin, pmin);

		vec_float4 pmax = vec_max(vec_max(p0, p1), vec_max(p2, p3));
		v_distMax = vec_max(v_distMax, pmax);
	}
	for(;v < (s32)numVerts;v++) {
		vec_float4 p = (vec_float4)(qword)si_from_float(plane.onPlane(verts[vertIds[v]]));
		v_distMin = vec_min(v_distMin, p);
		v_distMax = vec_max(v_distMax, p);
	}
	distMin = spu_extract(v_distMin, 0);
	distMax = spu_extract(v_distMax, 0);
#else
	distMin = FLT_MAX;
	distMax = -FLT_MAX;
	for(s32 v=0;v < (s32)numVerts;v++) {
		f32 p = plane.onPlane(verts[vertIds[v]]);
		if(p < distMin) distMin = p;
		if(p > distMax) distMax = p;
	}
#endif
}

static inline void getProjAxis(const Vector3 *verts, const u8 *vertIds, u8 numVerts, const Vector3& axis, f32& distMin,f32& distMax)
{
#ifdef TRY_SIMD
	vec_float4 v_distMin = ((vec_float4){FLT_MAX, 0, 0, 0});
	vec_float4 v_distMax = ((vec_float4){-FLT_MAX, 0, 0, 0});
	s32 v=0;
	for(;v < (s32)numVerts - 4;v+=4) {
		vec_float4 p0 = (vec_float4)(qword)si_from_float(dot(axis,verts[vertIds[v + 0]]));
		vec_float4 p1 = (vec_float4)(qword)si_from_float(dot(axis,verts[vertIds[v + 1]]));
		vec_float4 p2 = (vec_float4)(qword)si_from_float(dot(axis,verts[vertIds[v + 2]]));
		vec_float4 p3 = (vec_float4)(qword)si_from_float(dot(axis,verts[vertIds[v + 3]]));

		vec_float4 pmin = vec_min(vec_min(p0, p1), vec_min(p2, p3));
		v_distMin = vec_min(v_distMin, pmin);

		vec_float4 pmax = vec_max(vec_max(p0, p1), vec_max(p2, p3));
		v_distMax = vec_max(v_distMax, pmax);
	}
	for(;v < (s32)numVerts;v++) {
		vec_float4 p = (vec_float4)(qword)si_from_float(dot(axis,verts[vertIds[v]]));
		v_distMin = vec_min(v_distMin, p);
		v_distMax = vec_max(v_distMax, p);
	}
	distMin = spu_extract(v_distMin, 0);
	distMax = spu_extract(v_distMax, 0);
#else
	distMin = FLT_MAX;
	distMax = -FLT_MAX;
	for(s32 v=0;v < (s32)numVerts;v++) {
		f32 p = dot(axis, verts[vertIds[v]]);
		if(p < distMin) distMin = p;
		if(p > distMax) distMax = p;
	}
#endif
}

static inline void getProjAxisPnts6(const Vector3 *verts, const Vector3& axis, f32& distMin, f32& distMax)
{
#ifdef __SPU__
	vec_float4 p0 = (vec_float4)(qword)si_from_float(dot(axis, verts[0]));
	vec_float4 p1 = (vec_float4)(qword)si_from_float(dot(axis, verts[1]));
	vec_float4 p2 = (vec_float4)(qword)si_from_float(dot(axis, verts[2]));
	vec_float4 p3 = (vec_float4)(qword)si_from_float(dot(axis, verts[3]));
	vec_float4 p4 = (vec_float4)(qword)si_from_float(dot(axis, verts[4]));
	vec_float4 p5 = (vec_float4)(qword)si_from_float(dot(axis, verts[5]));

	vec_float4 v_distMin = vec_min(vec_min(vec_min(vec_min(vec_min(p0, p1), p2), p3), p4), p5);
	vec_float4 v_distMax = vec_max(vec_max(vec_max(vec_max(vec_max(p0, p1), p2), p3), p4), p5);

	distMin = spu_extract(v_distMin, 0);
	distMax = spu_extract(v_distMax, 0);
#else
	f32 p0 = dot(axis, verts[0]);
	f32 p1 = dot(axis, verts[1]);
	f32 p2 = dot(axis, verts[2]);
	f32 p3 = dot(axis, verts[3]);
	f32 p4 = dot(axis, verts[4]);
	f32 p5 = dot(axis, verts[5]);
	distMin = MIN(p5, MIN(p4, MIN(p3, MIN(p2, MIN(p0, p1)))));
	distMax = MAX(p5, MAX(p4, MAX(p3, MAX(p2, MAX(p0, p1)))));
#endif
}

static inline void getProjAxisPnts3(const Vector3 *verts, const Vector3& axis, f32& distMin, f32& distMax)
{
//#ifdef TRY_SIMD
#ifdef __SPU__
	vec_float4 p0 = (vec_float4)(qword)si_from_float(dot(axis,verts[0]));
	vec_float4 p1 = (vec_float4)(qword)si_from_float(dot(axis,verts[1]));
	vec_float4 p2 = (vec_float4)(qword)si_from_float(dot(axis,verts[2]));

	vec_float4 v_distMin = vec_min(vec_min(p0, p1), p2);
	vec_float4 v_distMax = vec_max(vec_max(p0, p1), p2);

	distMin = spu_extract(v_distMin, 0);
	distMax = spu_extract(v_distMax, 0);
#else
	f32 p0 = dot(axis, verts[0]);
	f32 p1 = dot(axis, verts[1]);
	f32 p2 = dot(axis, verts[2]);
	distMin = MIN(p2, MIN(p0, p1));
	distMax = MAX(p2, MAX(p0, p1));
#endif
}

static inline void getProjAxisPnts2(const Vector3 *verts, const Vector3& axis, f32& distMin,f32& distMax)
{
//#ifdef TRY_SIMD
#ifdef __SPU__
	vec_float4 p0 = (vec_float4)(qword)si_from_float(dot(axis,verts[0]));
	vec_float4 p1 = (vec_float4)(qword)si_from_float(dot(axis,verts[1]));

	vec_float4 v_distMin = vec_min(p0, p1);
	vec_float4 v_distMax = vec_max(p0, p1);

	distMin = spu_extract(v_distMin, 0);
	distMax = spu_extract(v_distMax, 0);
#else
	f32 p0 = dot(axis, verts[0]);
	f32 p1 = dot(axis, verts[1]);
	distMin = MIN(p0, p1);
	distMax = MAX(p0, p1);
#endif
}

static inline u8 getClosestPointProjPlane(const Vector3 *verts, const u8 *vertIds, u8 numVerts, const Plane& plane)
{
	u8 minId = 0;
	f32 distMin = FLT_MAX;
	for(s32 v=0;v < (s32)numVerts;v++) {
		f32 p = plane.onPlane(verts[vertIds[v]]);
		if(p < distMin) {
			distMin = p;
			minId = vertIds[v];
		}
	}
	return minId;
}

///////////////////////////////////////////////////////////////////////////////
// ２つのベクトルの向きをチェック

static inline bool isSameDirection(const Vector3& vecA, const Vector3& vecB)
{
#ifdef TRY_SIMD
	vec_float4 vec0 = vecA.get128();
	vec_float4 vec1 = vecB.get128();
	vec_float4 tmp;
	tmp = spu_mul(vec0, vec1);
	tmp = spu_madd(spu_rlqwbyte(vec0, 4), spu_rlqwbyte(vec1, 4), tmp);
	tmp = spu_madd(spu_rlqwbyte(vec0, 8), spu_rlqwbyte(vec1, 8), tmp);
	vec_uint4 result = spu_cmpgt(fabsf4(tmp), spu_splats(0.9999f));
	return spu_extract(result, 0) > 0;
#else
	return fabsf(dot(vecA, vecB)) > 0.9999f;
#endif
}

#endif /* SAT_MESH_UTILS_H_ */
