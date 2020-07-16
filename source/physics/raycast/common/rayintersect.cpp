/*
 * rayintersect.cpp
 *
 *  Created on: Jun 7, 2013
 *      Author: mike
 */

#include "base/common.h"

#include "rigidbody/common/collobject.h"
#include "rigidbody/common/intersectfunction.h"
#include "rigidbody/common/subdata.h"

#include "raycast/common/ray.h"

#ifdef __SPU__
	#include "base/heapmanager.h"
	#include "base/prefetchiterator.h"

	#define AABB_PREFETCH_NUM 		32

	extern HeapManager gPool;
#endif

#define RAYINTERSECTFUNC(funcName) \
		bool funcName(\
			const CollPrim& prim,\
			const Transform3& transform,\
			Ray& ray, f32& t);

typedef bool (*RayIntersect)(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t);

RAYINTERSECTFUNC(rayIntersectDummy)
RAYINTERSECTFUNC(rayIntersectSphere)
RAYINTERSECTFUNC(rayIntersectBox)
RAYINTERSECTFUNC(rayIntersectCapsule)

RayIntersect funcTbl_rayIntersect[PRIM_COUNT] =
{
	rayIntersectSphere,
	rayIntersectBox,
	rayIntersectCapsule
};

bool rayIntersectDummy(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t)
{
	(void) prim;
	(void) transform;
	(void) ray;
	(void) t;

	return false;
}

bool rayIntersectSphere(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t)
{
	const Sphere& sphere = prim.getSphere();

	Vector3 v = ray.startPos - transform.getTranslation();

	f32 a = dot(ray.rayDir, ray.rayDir);
	f32 b = dot(v, ray.rayDir);
	f32 c = dot(v, v) - sphere.radius*sphere.radius;

	if(c < 0.0f) return false;

	f32 d = b*b - a*c;

	if(d < 0.0f || fabs(a) < 0.00001f) return false;

	f32 tt = (-b - sqrtf(d))/a;

	if(tt < 0.0f || tt > 1.0f) return false;

	if(tt < t) {
		t = tt;
		ray.contactPoint = ray.startPos + t*ray.rayDir;
		ray.contactNormal = normalize(ray.contactPoint - transform.getTranslation());
		ray.subData.type = SubData::SubDataNone;
		return true;
	}

	return false;
}

bool rayIntersectBox(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t)
{
	const Box& box = prim.getBox();

	Transform3 transformBox = orthoInverse(transform);
	Vector3 startPosL = transformBox.getUpper3x3()*ray.startPos + transformBox.getTranslation();
	Vector3 rayDirL = transformBox.getUpper3x3()*ray.rayDir;

	f32 cur_t = 0.0f;
	Vector3 cur_nml(0.0f);
	if(rayIntersectAABB(box.half, Vector3(0.0f), startPosL, rayDirL, cur_t, cur_nml) && cur_t > 0.0f && cur_t < t) {
		t = cur_t;
		ray.contactPoint = ray.startPos + t*ray.rayDir;
		ray.contactNormal = transform.getUpper3x3()*cur_nml;
		ray.subData.type = SubData::SubDataNone;
		return true;
	}

	return false;
}

bool rayIntersectCapsule(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t)
{
	const Capsule &capsule = prim.getCapsule();

	Transform3 transformCapsule = orthoInverse(transform);
	Vector3 startPosL = transformCapsule.getUpper3x3()*ray.startPos + transformCapsule.getTranslation();
	Vector3 rayDirL = transformCapsule.getUpper3x3()*ray.rayDir;

	f32 radSqr = capsule.radius*capsule.radius;

	{
		f32 h = fabsf(startPosL[0]);
		if(h > capsule.hLength) h = capsule.hLength;
		Vector3 Px(t, 0, 0);
		f32 sqrLen = lengthSqr(startPosL - Px);
		if(sqrLen <= radSqr) return false;
	}

	do {
		Vector3 P(startPosL);
		Vector3 D(rayDirL);

		P[0] = 0.0f;
		D[0] = 0.0f;

		f32 a = dot(D, D);
		f32 b = dot(P, D);
		f32 c = dot(P, P) - radSqr;

		f32 d = b*b - a*c;

		if(d < 0.0f || fabs(a) < 0.00001f) return false;

		f32 tt = (-b - sqrtf(d))/a;

		if(tt < 0.0f)
			break;
		else if(tt > 1.0f)
			return false;

		if(tt < t) {
			Vector3 cp = startPosL + tt*rayDirL;

			if(fabsf(cp[0]) <= capsule.hLength) {
				t = tt;
				ray.contactPoint = Vector3(transform*Point3(cp));
				cp[0] = 0.0f;
				ray.contactNormal = transform.getUpper3x3()*normalize(cp);
				ray.subData.type = SubData::SubDataNone;
				return true;
			}
		}
	} while(0);

	f32 a = dot(rayDirL, rayDirL);
	if(fabs(a) < 0.00001f) return false;

	do {
		Vector3 center(capsule.hLength, 0.0f, 0.0f);
		Vector3 v = startPosL - center;

		f32 b = dot(v, rayDirL);
		f32 c = dot(v, v) - radSqr;

		f32 d = b*b - a*c;

		if(d < 0.0f) break;

		f32 tt = (-b - sqrtf(d))/a;

		if(tt < 0.0f || tt > 1.0f) break;

		if(tt < t) {
			t = tt;
			Vector3 cp = startPosL + t*rayDirL;
			ray.contactPoint = ray.startPos + t*ray.rayDir;
			ray.contactNormal = transform.getUpper3x3()*normalize(cp - center);
			ray.subData.type = SubData::SubDataNone;
			return true;
		}
	} while(0);

	{
		Vector3 center(-capsule.hLength, 0.0f, 0.0f);
		Vector3 v = startPosL - center;

		f32 b = dot(v, rayDirL);
		f32 c = dot(v, v) - radSqr;

		f32 d = b*b - a*c;

		if(d < 0.0f) return false;

		f32 tt = (-b - sqrtf(d))/a;

		if(tt < 0.0f || tt > 1.0f) return false;

		if(tt < t) {
			t = tt;
			Vector3 cp = startPosL + t*rayDirL;
			ray.contactPoint = ray.startPos + t*ray.rayDir;
			ray.contactNormal = transform.getUpper3x3()*normalize(cp - center);
			ray.subData.type = SubData::SubDataNone;
			return true;
		}
	}

	return false;
}

bool rayIntersectConvex(const CollPrim& prim, const Transform3& transform, Ray& ray,f32 &t)
{
	ConvexMesh *mesh;

#ifdef __SPU__
	mesh = (ConvexMesh*)gPool.allocate(sizeof(ConvexMesh), HeapManager::ALIGN16);
	prim.getConvexMesh(mesh);
#else
	mesh = prim.getConvexMesh();
#endif

	Transform3 transformConvexMesh = orthoInverse(transform);
	Vector3 rayStartL = transformConvexMesh.getUpper3x3()*ray.startPos + transformConvexMesh.getTranslation();
	Vector3 rayDirL = transformConvexMesh.getUpper3x3()*ray.rayDir;

	f32 cur_t = 0.0f;
	Vector3 cur_nml(0.0f);
	bool ret = false;
	for(u8 f=0;f < mesh->numIndices/3;f++) {
		Vector3 facetPnts[3] =
		{
			mesh->verts[mesh->indices[f*3 + 0]],
			mesh->verts[mesh->indices[f*3 + 1]],
			mesh->verts[mesh->indices[f*3 + 2]]
		};

		if(rayIntersectTriangleWithoutBackFace(facetPnts, rayStartL, rayDirL, cur_t) && cur_t < t) {
			t = cur_t;
			ray.contactPoint = ray.startPos + t*ray.rayDir;
			ray.contactNormal = transform.getUpper3x3()*normalize(cross(facetPnts[2] - facetPnts[1], facetPnts[0] - facetPnts[1]));
			ray.subData.type = SubData::SubDataNone;
			ret = true;
		}
	}

#ifdef __SPU__
	gPool.deallocate(mesh);
#endif

	return ret;
}

inline bool rayIntersectTriMesh(const TriMesh *mesh, const Vector3& rayStartL, const Vector3& rayDirL, u8 facetType, f32& t, Vector3& nml, SubData& subData)
{
	f32 cur_t;
	Vector3 cur_nml;
	bool ret = false;
	u32 nearFacetId = 0;

	for(u32 f=0;f < mesh->numFacets;f++) {
		const MeshFacet& facet = mesh->facets[f];

		Vector3 facetCenter = read_Vector3(facet.center);
		Vector3 facetHalf = read_Vector3(facet.half);

		if(!rayIntersectAABBFast(facetHalf, facetCenter, rayStartL, rayDirL, cur_t))
			continue;

		if(t <= cur_t) continue;

		Vector3 facetPnts[3] =
		{
			mesh->verts[facet.vertIndices[0]],
			mesh->verts[facet.vertIndices[1]],
			mesh->verts[facet.vertIndices[2]]
		};

		if(facetType == FacetTypeFront && rayIntersectTriangleWithoutBackFace(facetPnts, rayStartL, rayDirL, cur_t) && cur_t < t) {
			nearFacetId = f;
			t = cur_t;
			nml = read_Vector3(facet.normal);
			ret = true;
		}
		else if(facetType == FacetTypeBack && rayIntersectTriangleWithoutFrontFace(facetPnts, rayStartL, rayDirL, cur_t) && cur_t < t) {
			nearFacetId = f;
			t = cur_t;
			nml = read_Vector3(facet.normal);
			ret = true;
		}
		else if(facetType == FacetTypeBoth && rayIntersectTriangle(facetPnts, rayStartL, rayDirL, cur_t) && cur_t < t) {
			nearFacetId = f;
			t = cur_t;
			nml = read_Vector3(facet.normal);
			ret = true;
		}
	}

	if(ret) {
		const MeshFacet& facet = mesh->facets[nearFacetId];
		Vector3 facetPnts[3] =
		{
			mesh->verts[facet.vertIndices[0]],
			mesh->verts[facet.vertIndices[1]],
			mesh->verts[facet.vertIndices[2]]
		};
		f32 fs, ft;
		Vector3 P = rayStartL + t*rayDirL;
		get_ST(fs, ft, facetPnts[1] - facetPnts[0], facetPnts[2] - facetPnts[0], P - facetPnts[0]);
		subData.type = SubData::SubDataFacetLocal;
		subData.setFacetLocalS(fs);
		subData.setFacetLocalT(ft);
		subData.setFacetIndex(nearFacetId);
	}

	return ret;
}

#ifdef __SPU__

inline u32 dmaGetBuffer(void* ls, u32 ea, s32 sz, u32 tag, u32 endEA)
{
	s32 sz_tmp = (ea + sz <= endEA) ? sz : endEA - ea;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_large_get(ls, ea, sz_tmp, tag, 0, 0);
	return sz_tmp;
}

bool rayIntersectLargeMesh(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t)
{
	Transform3 transformMesh = orthoInverse(transform);
	Vector3 startPosL = transformMesh.getUpper3x3()*ray.startPos + transformMesh.getTranslation();
	Vector3 rayDirL = transformMesh.getUpper3x3()*ray.rayDir;

	LargeTriMesh largeMesh;
	prim.getLargeMesh(&largeMesh);

	VecInt3 s, e, aabbMinL, aabbMaxL;

	s = largeMesh.getLocalPosition(startPosL);
	e = largeMesh.getLocalPosition(startPosL + rayDirL);

	aabbMinL = minPerElem(s, e);
	aabbMaxL = maxPerElem(s, e);

#ifdef __SPU__
	vec_uint4 ptn_mask0 = ((vec_uint4){0x02031213, 0x06071617, 0x0A0B1A1B, 0x80808080});
	vec_ushort8 vecAabbA;
	vecAabbA = spu_shuffle((vec_ushort8)aabbMinL.get128(), (vec_ushort8)aabbMaxL.get128(), (vec_uchar16)ptn_mask0);
#endif

	f32 cur_t;
	Vector3 cur_nml(0.0f);
	bool ret = false;

	{
		ReadOnlyPrefetchForwardIterator<AABB16> itrAABB(
			&gPool,
			(u32)largeMesh.aabbList,
			(u32)(largeMesh.aabbList + largeMesh.numIslands),
			AABB_PREFETCH_NUM, 21);

		u32 numIslands = largeMesh.numIslands;
		for(u32 i=0;i < numIslands;i++, ++itrAABB) {
			AABB16 aabbB = *itrAABB;

#ifdef __SPU__
			vec_ushort8 vecAabbB = (vec_ushort8)aabbB;
			vec_uint4 ptn_mask1 = ((vec_uint4){0x10110001, 0x14150405, 0x18190809, 0x80808080});
			vec_ushort8 vecMin = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask1);
			vec_ushort8 vecMax = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask0);
			vec_ushort8 isGt = spu_cmpgt(vecMin, vecMax);
			if(spu_extract(spu_gather(isGt), 0) > 0) continue;
#else
			if(aabbMaxL.getX() < getXMin(aabbB) || aabbMinL.getX() > getXMax(aabbB)) continue;
			if(aabbMaxL.getY() < getYMin(aabbB) || aabbMinL.getY() > getYMax(aabbB)) continue;
			if(aabbMaxL.getZ() < getZMin(aabbB) || aabbMinL.getZ() > getZMax(aabbB)) continue;
#endif

			Vector3 aabbMin, aabbMax;
			aabbMin = largeMesh.getWorldPosition(VecInt3(getXMin(aabbB), getYMin(aabbB), getZMin(aabbB)));
			aabbMax = largeMesh.getWorldPosition(VecInt3(getXMax(aabbB), getYMax(aabbB), getZMax(aabbB)));

			if(!rayIntersectAABBFast((aabbMax - aabbMin)*0.5f, (aabbMin + aabbMax)*0.5f, startPosL, rayDirL, cur_t))
				continue;

			if(t <= cur_t) continue;

			TriMesh *island = (TriMesh*)gPool.allocate(sizeof(TriMesh), HeapManager::ALIGN16);
			spu_dma_get(island, (u64)(largeMesh.islands + i), sizeof(TriMesh), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			SubData subData;
			if(rayIntersectTriMesh(island, startPosL, rayDirL, ray.facetType, t, cur_nml, subData)) {
				ray.contactPoint = ray.startPos + t*ray.rayDir;
				ray.contactNormal = transform.getUpper3x3()*cur_nml;
				ray.subData = subData;
				ray.subData.setIslandIndex(i);
				ret = true;
			}

			gPool.deallocate(island);
		}
	}

	return ret;
}

#else // __SPU__

bool rayIntersectLargeMesh(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t)
{
	Transform3 transformMesh = orthoInverse(transform);
	Vector3 startPosL = transformMesh.getUpper3x3()*ray.startPos + transformMesh.getTranslation();
	Vector3 rayDirL = transformMesh.getUpper3x3()*ray.rayDir;

	LargeTriMesh *largeMesh = prim.getLargeMesh();

	VecInt3 s, e, aabbMinL, aabbMaxL;

	s = largeMesh->getLocalPosition(startPosL);
	e = largeMesh->getLocalPosition(startPosL + rayDirL);

	aabbMinL = minPerElem(s, e);
	aabbMaxL = maxPerElem(s, e);

	f32 cur_t = 0.0f;
	Vector3 cur_nml(0.0f);
	bool ret = false;

	for(u8 i=0;i < largeMesh->numIslands;i++) {
		AABB16 aabbB = largeMesh->aabbList[i];
		if(aabbMaxL.getX() < getXMin(aabbB) || aabbMinL.getX() > getXMax(aabbB)) continue;
		if(aabbMaxL.getY() < getYMin(aabbB) || aabbMinL.getY() > getYMax(aabbB)) continue;
		if(aabbMaxL.getZ() < getZMin(aabbB) || aabbMinL.getZ() > getZMax(aabbB)) continue;

		Vector3 aabbMin, aabbMax;
		aabbMin = largeMesh->getWorldPosition(VecInt3((s32)getXMin(aabbB), (s32)getYMin(aabbB), (s32)getZMin(aabbB)));
		aabbMax = largeMesh->getWorldPosition(VecInt3((s32)getXMax(aabbB), (s32)getYMax(aabbB), (s32)getZMax(aabbB)));

		if(!rayIntersectAABBFast((aabbMax - aabbMin)*0.5f, (aabbMin + aabbMax)*0.5f, startPosL, rayDirL, cur_t))
			continue;

		if(t <= cur_t) continue;

		TriMesh *island = &largeMesh->islands[i];
		SubData subData;

		if(rayIntersectTriMesh(island, startPosL, rayDirL, ray.facetType, t, cur_nml, subData)) {
			ray.contactPoint = ray.startPos + t*ray.rayDir;
			ray.contactNormal = transform.getUpper3x3()*cur_nml;
			ray.subData = subData;
			ray.subData.setIslandIndex(i);
			ret = true;
		}
	}

	return ret;
}

#endif // __SPU__

bool rayIntersect(const CollObject& obj, const Transform3& transform, Ray& ray, f32& t)
{
	bool intersectFlag = false;

	PrimIterator itrPrim(obj);
	for(s32 p=0;p < obj.getNumPrims();p++, ++itrPrim) {
		const CollPrim& prim = *itrPrim;
		Transform3 primTransform = transform*prim.getObjectRelTransform();

		if(funcTbl_rayIntersect[prim.getType()](prim, primTransform, ray, t)) {
			ray.primIdx = p;
			intersectFlag = true;
		}
	}

	return intersectFlag;
}
