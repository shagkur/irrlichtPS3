/*
 * rayintersectheightfield.cpp
 *
 *  Created on: Jun 10, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/heightfield.h"

#include "raycast/common/ray.h"

#define HFIELD_RAY_CACHE		4
#define HEPSILON 				0.00001f

#define HFIELD_IDX(i, j)		((j)*HEIGHTFIELD_X + (i))
#define HFIELD_VEC3(i, j, k)	(((j)*(HEIGHTFIELD_X) + (i))*3 + (k))
#define HFIELD_VEC2(i, j, k)	(((j)*(HEIGHTFIELD_X) + (i))*2 + (k))

#ifdef __SPU__
	#define RAY_HEIGHTFIELD_CACHE_ENABLE

	#include "base/heapmanager.h"
	extern HeapManager gPool;
#endif

HeightField *_heightfield = NULL;

#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE

struct cacheFlag
{
	u8 flag;// 0:none 1:exist 2:keep
	u8 idx;
};

struct HeightFieldCache
{
	bool	firstBlock;

	s32		curBi, curBj;
	s32		curBlockCount;
	s32		curBlockBi[HFIELD_RAY_CACHE];
	s32		curBlockBj[HFIELD_RAY_CACHE];

	ATTRIBUTE_ALIGNED16_PTR32(cacheFlag	*cacheTable);
	ATTRIBUTE_ALIGNED16_PTR32(HeightFieldBlock *cacheData);
} hfc;

static void  initializeHeightFieldCache();
static void  releaseHeightFieldCache();
static void  initBlockData(s32 bi, s32 bj, s32 stepX, s32 stepZ);
static void  updateBlockData(s32 bi, s32 bj, s32 stepX, s32 stepZ);
static f32 getCachedFieldData(s32 i, s32 j);

#endif

///////////////////////////////////////////////////////////////////////////////
// Cache Function

#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE

static void  initializeHeightFieldCache()
{
	hfc.firstBlock = true;
	hfc.cacheTable = (cacheFlag*)gPool.allocate(_heightfield->fieldData.numBlockX*_heightfield->fieldData.numBlockZ*sizeof(cacheFlag), HeapManager::ALIGN16);
	hfc.cacheData = (HeightFieldBlock*)gPool.allocate(sizeof(HeightFieldBlock)*HFIELD_RAY_CACHE, HeapManager::ALIGN128);
}

static void  releaseHeightFieldCache()
{
	gPool.deallocate(hfc.cacheData);
	gPool.deallocate(hfc.cacheTable);
}

static void  initBlockData(s32 bi, s32 bj, s32 stepX, s32 stepZ)
{
	s32 nextBi = bi + stepX;
	s32 nextBj = bj + stepZ;

	s32 nextBlockCount = 0;
	s32 nextBlockBi[HFIELD_RAY_CACHE];
	s32 nextBlockBj[HFIELD_RAY_CACHE];

	{
		nextBlockBi[nextBlockCount] = bi;
		nextBlockBj[nextBlockCount] = bj;
		nextBlockCount++;
	}

	if(nextBi >= 0 && nextBi < _heightfield->fieldData.numBlockX) {
		nextBlockBi[nextBlockCount] = nextBi;
		nextBlockBj[nextBlockCount] = bj;
		nextBlockCount++;
	}

	if(nextBj >= 0 && nextBj < _heightfield->fieldData.numBlockZ) {
		nextBlockBi[nextBlockCount] = bi;
		nextBlockBj[nextBlockCount] = nextBj;
		nextBlockCount++;
	}

	if((nextBi >= 0 && nextBi < _heightfield->fieldData.numBlockX) && (nextBj >= 0 && nextBj < _heightfield->fieldData.numBlockZ)) {
		nextBlockBi[nextBlockCount] = nextBi;
		nextBlockBj[nextBlockCount] = nextBj;
		nextBlockCount++;
	}

	__builtin_memset(hfc.cacheTable, 0 ,_heightfield->fieldData.numBlockX*_heightfield->fieldData.numBlockZ*sizeof(cacheFlag));

	for(s32 i=0;i < nextBlockCount;i++) {
		s32 idx = nextBlockBj[i]*_heightfield->fieldData.numBlockX + nextBlockBi[i];
#ifdef __SPU__
		spu_dma_large_get(&hfc.cacheData[i], (u64)&_heightfield->fieldData.blocks[idx], sizeof(HeightFieldBlock), 1, 0, 0);
#else
		memcpy(&hfc.cacheData[i], _heightfield->fieldData.blocks[idx], sizeof(HeightFieldBlock));
#endif
		hfc.cacheTable[idx].idx = i;
		hfc.cacheTable[idx].flag = 1;
	}

#ifdef __SPU__
	spu_dma_wait_tag_status_all(1<<1);
#endif

	hfc.curBi = bi;
	hfc.curBj = bj;
	hfc.curBlockCount = nextBlockCount;

	__builtin_memcpy(hfc.curBlockBi, nextBlockBi, sizeof(s32)*nextBlockCount);
	__builtin_memcpy(hfc.curBlockBj, nextBlockBj, sizeof(s32)*nextBlockCount);
}

static void  updateBlockData(s32 bi, s32 bj, s32 stepX, s32 stepZ)
{
	s32 nextBi = bi + stepX;
	s32 nextBj = bj + stepZ;

	s32	emptyBuff[HFIELD_RAY_CACHE] = {-1};

	s32 nextBlockCount = 0;
	s32 nextBlockBi[HFIELD_RAY_CACHE];
	s32 nextBlockBj[HFIELD_RAY_CACHE];

	{
		s32 idx = bj*_heightfield->fieldData.numBlockX + bi;
		if(hfc.cacheTable[idx].flag > 0)
			hfc.cacheTable[idx].flag = 2;
		nextBlockBi[nextBlockCount] = bi;
		nextBlockBj[nextBlockCount] = bj;
		nextBlockCount++;
	}

	if(nextBi >= 0 && nextBi < _heightfield->fieldData.numBlockX) {
		int idx = bj*_heightfield->fieldData.numBlockX + nextBi;
		if(hfc.cacheTable[idx].flag > 0)
			hfc.cacheTable[idx].flag = 2;
		nextBlockBi[nextBlockCount] = nextBi;
		nextBlockBj[nextBlockCount] = bj;
		nextBlockCount++;
	}

	if(nextBj >= 0 && nextBj < _heightfield->fieldData.numBlockZ) {
		int idx = nextBj*_heightfield->fieldData.numBlockX + bi;
		if(hfc.cacheTable[idx].flag > 0)
			hfc.cacheTable[idx].flag = 2;
		nextBlockBi[nextBlockCount] = bi;
		nextBlockBj[nextBlockCount] = nextBj;
		nextBlockCount++;
	}

	if((nextBi >= 0 && nextBi < _heightfield->fieldData.numBlockX) && (nextBj >= 0 && nextBj < _heightfield->fieldData.numBlockZ)) {
		int idx = nextBj*_heightfield->fieldData.numBlockX + nextBi;
		if(hfc.cacheTable[idx].flag > 0)
			hfc.cacheTable[idx].flag = 2;
		nextBlockBi[nextBlockCount] = nextBi;
		nextBlockBj[nextBlockCount] = nextBj;
		nextBlockCount++;
	}

	s32 emptyCount = 0;
	for(s32 i=0;i < hfc.curBlockCount;i++) {
		s32 idx = hfc.curBlockBj[i]*_heightfield->fieldData.numBlockX + hfc.curBlockBi[i];
		if(hfc.cacheTable[idx].flag != 2) {
			emptyBuff[emptyCount++] = hfc.cacheTable[idx].idx;
			hfc.cacheTable[idx].flag = 0;
			hfc.cacheTable[idx].idx = 0;
		}
	}

	s32 ibuff = 0;
	for(s32 i=0;i < nextBlockCount;i++) {
		s32 idx = nextBlockBj[i]*_heightfield->fieldData.numBlockX + nextBlockBi[i];
		if(hfc.cacheTable[idx].flag == 2)
			hfc.cacheTable[idx].flag = 1;
		else if(hfc.cacheTable[idx].flag == 0) {
			ASSERT(ibuff < emptyCount);

#ifdef __SPU__
			spu_dma_large_get(&hfc.cacheData[emptyBuff[ibuff]], (u64)&_heightfield->fieldData.blocks[idx], sizeof(HeightFieldBlock), 1, 0, 0);
#else
			memcpy(hfc.cacheData[emptyBuff[ibuff]], _heightfield->fieldData.blocks[idx], sizeof(HeightFieldBlock));
#endif

			hfc.cacheTable[idx].flag = 1;
			hfc.cacheTable[idx].idx = emptyBuff[ibuff];

			ibuff++;
		}
	}

	hfc.curBi = bi;
	hfc.curBj = bj;
	hfc.curBlockCount = nextBlockCount;
	__builtin_memcpy(hfc.curBlockBi, nextBlockBi, sizeof(s32)*nextBlockCount);
	__builtin_memcpy(hfc.curBlockBj, nextBlockBj, sizeof(s32)*nextBlockCount);
}

static f32 getCachedFieldData(s32 i, s32 j)
{
	s32 ci = i - hfc.curBi*BLOCK_SIZE;
	s32 cj = j - hfc.curBj*BLOCK_SIZE;
	s32 idx = hfc.curBj*_heightfield->fieldData.numBlockX + hfc.curBi;
	return hfc.cacheData[hfc.cacheTable[idx].idx].heightBuf[cj*BLOCK_SIZE_B + ci];
}

#endif

static inline bool intersectTriangle(Vector3& p1, Vector3& p2, Vector3& p3, Vector3& n, Vector3& rayStart, Vector3& rayDir, f32& t)
{
	f32 v, w, d;
	Vector3 p1p2, p1p3;
	p1p2 = p2 - p1;
	p1p3 = p3 - p1;
	n = cross(p1p2, p1p3);

	d = dot(-rayDir, n);
	if(d <= 0.0f) return false;

	Vector3 pr = rayStart - p1;
	t = dot(pr, n);
	if(t < 0.0f || t > d) return false;

	Vector3 e = cross(-rayDir, pr);
	v = dot(p1p3, e);
	if(v < 0.0f || v > d) return false;

	w = -dot(p1p2, e);
	if(w < 0.0f || v + w > d) return false;

	t /= d;

	return true;
}

static bool setStartPosition(Vector3& startPosL, Vector3& endPosL, Vector3& rayDirL)
{
	f32 tx = 0.0f, tz = 0.0f;

	if(startPosL[0] < 0.0f) {
		if(fabsf(rayDirL[0]) < HEPSILON) return false;
		tx = (0 - startPosL[0])/rayDirL[0];
		if(tx < 0.0f || tx > 1.0f) return false;
	}
	else if(startPosL[0] > _heightfield->getFieldWidth()) {
		if(fabsf(rayDirL[0]) < HEPSILON) return false;
		tx = (_heightfield->getFieldWidth() - startPosL[0])/rayDirL[0];
		if(tx < 0.0f || tx > 1.0f) return false;
	}

	if(startPosL[2] < 0.0f) {
		if(fabsf(rayDirL[2]) < HEPSILON) return false;
		tz = (0 - startPosL[2])/rayDirL[2];
		if(tz < 0.0f || tz > 1.0f) return false;
	}
	else if(startPosL[2] > _heightfield->getFieldDepth()) {
		if(fabsf(rayDirL[2]) < HEPSILON) return false;
		tz = (_heightfield->getFieldDepth() - startPosL[2])/rayDirL[2];
		if(tz < 0.0f || tz > 1.0f) return false;
	}

	if(tx > tz) {
		startPosL = startPosL + tx*rayDirL;
		if(startPosL[2] < 0.0f || startPosL[2] > _heightfield->getFieldDepth())
			return false;
	} else {
		startPosL = startPosL + tz*rayDirL;
		if(startPosL[0] < 0.0f || startPosL[0] > _heightfield->getFieldWidth())
			return false;
	}

	rayDirL = endPosL-startPosL;

	return true;
}


#ifndef RAY_HEIGHTFIELD_CACHE_ENABLE

static bool detectIntersect(s32 xIdx, s32 zIdx, Vector3& startInCell, Vector3& endInCell, Vector3& startPosL, Vector3& endPosL, Vector3& rayDirL, f32& t, Vector3& nml)
{
	(void) endPosL;

	f32 h1 = getFieldData(_heightfield->fieldData, xIdx + 0, zIdx + 0);
	f32 h2 = getFieldData(_heightfield->fieldData, xIdx + 1, zIdx + 0);
	f32 h3 = getFieldData(_heightfield->fieldData, xIdx + 0, zIdx + 1);
	f32 h4 = getFieldData(_heightfield->fieldData, xIdx + 1, zIdx + 1);

	f32 hmin = MIN(MIN(MIN(h1, h2), h3), h4);
	f32 hmax = MAX(MAX(MAX(h1, h2), h3), h4);

	f32 rmin = MIN(startInCell[1], endInCell[1]);
	f32 rmax = MAX(startInCell[1], endInCell[1]);

	if(rmin > hmax || rmax < hmin)
		return false;

	{
		Vector3 p[4] =
		{
			Vector3(xIdx + 0, h1, zIdx + 0),
			Vector3(xIdx + 1, h2, zIdx + 0),
			Vector3(xIdx + 0, h3, zIdx + 1),
			Vector3(xIdx + 1, h4, zIdx + 1)
		};

		Vector3 n[2];
		f32 tt = 0.0f;

		if(intersectTriangle(p[0], p[2], p[1], n[0], startPosL, rayDirL, tt) && tt < t) {
			t = tt;

			Vector3 p1, p2, p3;
			p1 = mulPerElem(p[0], _heightfield->getScale());
			p2 = mulPerElem(p[2], _heightfield->getScale());
			p3 = mulPerElem(p[1], _heightfield->getScale());
			nml = normalize(cross(p2 - p1, p3 - p1));

			return true;
		}

		if(intersectTriangle(p[3], p[1], p[2], n[1], startPosL, rayDirL, tt) && tt < t) {
			t = tt;

			Vector3 p1, p2, p3;
			p1 = mulPerElem(p[3], _heightfield->getScale());
			p2 = mulPerElem(p[1], _heightfield->getScale());
			p3 = mulPerElem(p[2], _heightfield->getScale());
			nml = normalize(cross(p2 - p1, p3 - p1));

			return true;
		}
	}

	return false;
}

#else

static bool detectIntersect(s32 xIdx, s32 zIdx, Vector3& startInCell, Vector3& endInCell, Vector3& startPosL, Vector3& endPosL, Vector3& rayDirL, f32& t, Vector3& nml, s32 stepX, s32 stepZ)
{
	(void) endPosL;

	s32 bi = xIdx/BLOCK_SIZE;
	s32 bj = zIdx/BLOCK_SIZE;

	if(hfc.firstBlock) {
		initBlockData(bi, bj, stepX, stepZ);
		hfc.firstBlock = false;
	} else if(bi != hfc.curBi || bj != hfc.curBj) {
		spu_dma_wait_tag_status_all(1<<1);
		updateBlockData(bi, bj, stepX, stepZ);
	}

	f32 h1 = getCachedFieldData(xIdx + 0, zIdx + 0);
	f32 h2 = getCachedFieldData(xIdx + 1, zIdx + 0);
	f32 h3 = getCachedFieldData(xIdx + 0, zIdx + 1);
	f32 h4 = getCachedFieldData(xIdx + 1, zIdx + 1);

	f32 hmin = MIN(MIN(MIN(h1, h2), h3), h4);
	f32 hmax = MAX(MAX(MAX(h1, h2), h3), h4);

	f32 rmin = MIN(startInCell[1], endInCell[1]);
	f32 rmax = MAX(startInCell[1], endInCell[1]);

	if(rmin > hmax || rmax < hmin)
		return false;

	{
		Vector3 p[4] =
		{
			Vector3(xIdx + 0, h1, zIdx + 0),
			Vector3(xIdx + 1, h2, zIdx + 0),
			Vector3(xIdx + 0, h3, zIdx + 1),
			Vector3(xIdx + 1, h4, zIdx + 1)
		};

		Vector3 n[2];
		f32 tt;

		if(intersectTriangle(p[0], p[2], p[1], n[0], startPosL, rayDirL, tt) && tt < t) {
			t = tt;

			Vector3 p1, p2, p3;
			p1 = mulPerElem(p[0], _heightfield->getScale());
			p2 = mulPerElem(p[2], _heightfield->getScale());
			p3 = mulPerElem(p[1], _heightfield->getScale());
			nml = normalize(cross(p2 - p1, p3 - p1));

			return true;
		}

		if(intersectTriangle(p[3], p[1], p[2], n[1], startPosL, rayDirL, tt) && tt < t) {
			t = tt;

			Vector3 p1, p2, p3;
			p1 = mulPerElem(p[3], _heightfield->getScale());
			p2 = mulPerElem(p[1], _heightfield->getScale());
			p3 = mulPerElem(p[2], _heightfield->getScale());
			nml = normalize(cross(p2 - p1, p3 - p1));

			return true;
		}
	}

	return false;
}

#endif


bool rayIntersectHeightField(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t)
{
#ifdef __SPU__
	HeightField heightfield;
	prim.getHeightField(&heightfield);
	_heightfield = &heightfield;

	initializeHeightFieldCache();
#else
	_heightfield = const_cast<HeightField*>(prim.getHeightField());
#endif

	ASSERT(_heightfield);

	Transform3 ti = orthoInverse(transform);

	Vector3 startPosL_org = _heightfield->worldToLocalPosition(ti.getTranslation() + ti*ray.startPos);
	Vector3 endPosL_org = _heightfield->worldToLocalPosition(ti.getTranslation() + ti*ray.endPos);
	Vector3 rayDirL_org = endPosL_org - startPosL_org;
	Vector3 startPosL = startPosL_org;
	Vector3 endPosL = endPosL_org;
	Vector3 rayDirL = endPosL - startPosL;

	s32 xIdx, zIdx;
	s32 xIdxLimit, zIdxLimit;
	s32 stepX, stepZ;
	f32 tMaxX, tMaxZ;
	f32 tDeltaX, tDeltaZ;
	f32 rayDirInvX, rayDirInvZ;

	if(!setStartPosition(startPosL, endPosL, rayDirL)) {
#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE
		releaseHeightFieldCache();
#endif
		return false;
	}

	xIdx = (s32)floorf(startPosL[0]);
	zIdx = (s32)floorf(startPosL[2]);

	stepX = rayDirL[0] > 0.0f ? 1 : -1;
	stepZ = rayDirL[2] > 0.0f ? 1 : -1;

	xIdxLimit = (s32)floorf(endPosL[0]) + stepX*2;
	zIdxLimit = (s32)floorf(endPosL[2]) + stepZ*2;

	rayDirInvX = fabsf(rayDirL[0]) < HEPSILON ? stepX*9999.0f : 1.0f/rayDirL[0];
	rayDirInvZ = fabsf(rayDirL[2]) < HEPSILON ? stepZ*9999.0f : 1.0f/rayDirL[2];

	tDeltaX = stepX*rayDirInvX;
	tDeltaZ = stepZ*rayDirInvZ;

	tMaxX = stepX > 0 ? (floorf(startPosL[0] + 1.0f) - startPosL[0])*rayDirInvX : (floorf(startPosL[0]) - startPosL[0])*rayDirInvX;
	tMaxZ = stepZ > 0 ? (floorf(startPosL[2] + 1.0f) - startPosL[2])*rayDirInvZ : (floorf(startPosL[2]) - startPosL[2])*rayDirInvZ;

	Vector3 startInCell, endInCell;
	startInCell = startPosL;
	if(tMaxX < tMaxZ)
		endInCell = startPosL + tMaxX*rayDirL;
	else
		endInCell = startPosL + tMaxZ*rayDirL;

	bool ret =false;

	while(1) {
		Vector3 nml;
#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE
		if(detectIntersect(xIdx, zIdx, startInCell, endInCell, startPosL_org, endPosL_org, rayDirL_org, t, nml, stepX, stepZ)) {
#else
		if(detectIntersect(xIdx, zIdx, startInCell, endInCell, startPosL_org, endPosL_org, rayDirL_org, t, nml)) {
#endif
			ray.contactPoint = ray.startPos + t*ray.rayDir;
			ray.contactNormal = transform.getUpper3x3()*nml;
			ret = true;
			break;
		}

		if(tMaxX < tMaxZ) {
			startInCell = startPosL + tMaxX*rayDirL;
			tMaxX += tDeltaX;
			endInCell = startPosL + tMaxX*rayDirL;
			xIdx += stepX;
			if(xIdx < 0 || xIdx >= _heightfield->getFieldWidth() || xIdx == xIdxLimit )
				break;
		} else {
			startInCell = startPosL + tMaxZ*rayDirL;
			tMaxZ += tDeltaZ;
			endInCell = startPosL + tMaxZ*rayDirL;
			zIdx += stepZ;
			if(zIdx < 0 || zIdx >= _heightfield->getFieldDepth() || zIdx == zIdxLimit )
				break;
		}
	}

#ifdef RAY_HEIGHTFIELD_CACHE_ENABLE
	releaseHeightFieldCache();
#endif

	return ret;

}
