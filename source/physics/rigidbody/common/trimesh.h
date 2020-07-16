/*
 * trimesh.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef TRIMESH_H_
#define TRIMESH_H_

#include "base/common.h"
#include "base/aabb16.h"
#include "base/vecint3.h"
#include "base/simdfunc.h"

#define NUMMESHFACETS		64
#define NUMMESHEDGES		192
#define NUMMESHVERTICES		128

#define LARGEMESH_SIZE		1000.0f

#define EDGE_FLAT    		0
#define EDGE_CONVEX  		1
#define EDGE_CONCAVE 		2

struct MeshEdge
{
	u8 vertIndex[2];
	u8 angleType;
	union {
		u8 dirGroup; // 同じ方向を持つエッジを同グループとする（SAT判定の高速化のため）
		u8 tilt;
	};

	bool operator==(const MeshEdge& in) const
	{
	    return (in.vertIndex[0] == vertIndex[0] && in.vertIndex[1] == vertIndex[1]) ||
	    	   (in.vertIndex[1] == vertIndex[0] && in.vertIndex[0] == vertIndex[1]);
	}
};

struct MeshFacet
{
	f32 normal[3];
	f32 thickness;
	u8 dirGroup;
	u8 vertIndices[3];
	u8 edgeIndices[3];
	f32 half[3];
	f32 center[3];
};

ATTRIBUTE_ALIGNED16(struct) TriMesh
{
	u8 numVerts;
	u8 numEdges;
	u8 numFacets;
	u8 reserved;
	f32 half[3];
	MeshFacet facets[NUMMESHFACETS];
	MeshEdge edges[NUMMESHEDGES];
	Vector3 verts[NUMMESHVERTICES];

	TriMesh()
	{
		numVerts = numEdges = numFacets = 0;
	}

	bool isEdgeUnique(MeshEdge& edge, u8& id) const
	{
		for(u8 i=0;i < numEdges;i++) {
			if(edges[i] == edge) {
				id = i;
				return false;
			}
		}
		return true;
	}

	void updateAABB()
	{
		Vector3 halfMax(0.0f);

		for(u8 i=0;i < numFacets;i++) {
			Vector3 pnts[6] = {
				verts[facets[i].vertIndices[0]],
				verts[facets[i].vertIndices[1]],
				verts[facets[i].vertIndices[2]],
				verts[facets[i].vertIndices[0]] - facets[i].thickness*read_Vector3(facets[i].normal),
				verts[facets[i].vertIndices[1]] - facets[i].thickness*read_Vector3(facets[i].normal),
				verts[facets[i].vertIndices[2]] - facets[i].thickness*read_Vector3(facets[i].normal)
			};

			Vector3 facetAABBmin, facetAABBmax, facetHalf, facetCenter;
			facetAABBmin = minPerElem(pnts[5], minPerElem(pnts[4], minPerElem(pnts[3], minPerElem(pnts[2], minPerElem(pnts[0], pnts[1])))));
			facetAABBmax = maxPerElem(pnts[5], maxPerElem(pnts[4], maxPerElem(pnts[3], maxPerElem(pnts[2], maxPerElem(pnts[0], pnts[1])))));
			facetHalf = 0.5f*(facetAABBmax - facetAABBmin);
			facetCenter = 0.5f*(facetAABBmax + facetAABBmin);
			store_Vector3(facetHalf, facets[i].half);
			store_Vector3(facetCenter, facets[i].center);
			halfMax = maxPerElem(facetCenter + facetHalf, halfMax);
		}

		store_Vector3(halfMax, half);
	}

	void updateIslandAABB(Vector3& aabbMin, Vector3& aabbMax)
	{
		aabbMin = Vector3(FLT_MAX);
		aabbMax = Vector3(-FLT_MAX);

		for(u32 i=0;i < numFacets;i++) {
			aabbMin = minPerElem(aabbMin, read_Vector3(facets[i].center) - read_Vector3(facets[i].half));
			aabbMax = maxPerElem(aabbMax, read_Vector3(facets[i].center) + read_Vector3(facets[i].half));
		}
	}

	Vector3 getAABB(const Matrix3& rotation)
	{
		return absPerElem(rotation)*read_Vector3(half);
	}
};

ATTRIBUTE_ALIGNED16(struct) ConvexMesh
{
	u8 numVerts;
	u8 numIndices;
	f32 half[3];
	u16 indices[NUMMESHFACETS*3];
	Vector3 verts[NUMMESHVERTICES];

	ConvexMesh()
	{
		numVerts = numIndices = 0;
	}

	void updateAABB()
	{
		Vector3 halfMax(0.0f);
		for(u8 i=0;i < numVerts;i++) {
			halfMax = maxPerElem(absPerElem(verts[i]), halfMax);
		}
		store_Vector3(halfMax, half);
	}

	Vector3 getAABB(const Matrix3& rotation)
	{
		return absPerElem(rotation)*read_Vector3(half);
	}
};

ATTRIBUTE_ALIGNED16(struct) LargeTriMesh
{
	u8 numIslands;
	u8 maxIslands;

	ATTRIBUTE_PTR32(AABB16 *aabbList);

	ATTRIBUTE_PTR32(TriMesh *islands);

	LargeTriMesh()
	{
		numIslands = maxIslands = 0;
		islands = NULL;
		aabbList = NULL;
	}

	bool testAABB(s32 islandIndex, const Vector3& center, const Vector3& half) const
	{
		VecInt3 aabbMinL = getLocalPosition(center - half);
		VecInt3 aabbMaxL = getLocalPosition(center + half);

		if(aabbMaxL.getX() < getXMin(aabbList[islandIndex]) || aabbMinL.getX() > getXMax(aabbList[islandIndex])) return false;
		if(aabbMaxL.getY() < getYMin(aabbList[islandIndex]) || aabbMinL.getY() > getYMax(aabbList[islandIndex])) return false;
		if(aabbMaxL.getZ() < getZMin(aabbList[islandIndex]) || aabbMinL.getZ() > getZMax(aabbList[islandIndex])) return false;

		return true;
	}

	VecInt3 getLocalPosition(const Vector3& worldPos) const
	{
		Vector3 lmhalf(LARGEMESH_SIZE*0.5f);
		Vector3 sz(65535.0f);
		Vector3 tmp = divPerElem(worldPos + lmhalf, 2.0f*lmhalf);
		tmp = mulPerElem(sz, minPerElem(maxPerElem(tmp, Vector3(0.0f)), Vector3(1.0f))); // clamp 0.0 - 1.0
		return VecInt3(tmp);
	}

	VecInt3 getLocalPositionFloor(const Vector3& worldPos) const
	{
		Vector3 lmhalf(LARGEMESH_SIZE*0.5f);
		Vector3 sz(65535.0f);
		Vector3 tmp = divPerElem(worldPos + lmhalf, 2.0f*lmhalf);
		tmp = mulPerElem(sz,minPerElem(maxPerElem(tmp, Vector3(0.0f)), Vector3(1.0f))); // clamp 0.0 - 1.0
		return VecInt3(floorf(tmp[0]), floorf(tmp[1]), floorf(tmp[2]));
	}

	VecInt3 getLocalPositionCeil(const Vector3& worldPos) const
	{
		Vector3 lmhalf(LARGEMESH_SIZE*0.5f);
		Vector3 sz(65535.0f);
		Vector3 tmp = divPerElem(worldPos + lmhalf, 2.0f*lmhalf);
		tmp = mulPerElem(sz,minPerElem(maxPerElem(tmp, Vector3(0.0f)), Vector3(1.0f))); // clamp 0.0 - 1.0
		return VecInt3(ceilf(tmp[0]), ceilf(tmp[1]), ceilf(tmp[2]));
	}

	Vector3 getWorldPosition(const VecInt3& localPos) const
	{
		Vector3 lmhalf(LARGEMESH_SIZE*0.5f);
		Vector3 sz(65535.0f);
		Vector3 tmp = divPerElem((Vector3)localPos, sz);
		return mulPerElem(tmp, 2.0f*lmhalf) - lmhalf;
	}
};

#endif /* TRIMESH_H_ */
