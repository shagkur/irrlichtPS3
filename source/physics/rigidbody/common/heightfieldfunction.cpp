/*
 * heightfieldfunction.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/heightfieldfunction.h"

#ifdef __SPU__

#include "base/heapmanager.h"
extern HeapManager gPool;

struct HeightFieldCache
{
	s32	blockIndicator;
	s32 numCache;
	ATTRIBUTE_ALIGNED16(HeightFieldBlock *cachedBlock);
	ATTRIBUTE_ALIGNED16(s32 *blockIdx);
} hfc;

static f32 getCachedFieldData(const HeightFieldData& fieldData, s32 i, s32 j);

#endif

///////////////////////////////////////////////////////////////////////////////
// Internal Function

static inline f32 interpolate(f32 a, f32 b, f32 x)
{
	return a*(1.0f - x) + b*x;
}

#ifdef __SPU__

static f32 getCachedFieldData(const HeightFieldData& fieldData, s32 i, s32 j)
{
	i = MIN(fieldData.numFieldX - 1, i);
	j = MIN(fieldData.numFieldZ - 1, j);
	s32 bi = i/BLOCK_SIZE;
	s32 bj = j/BLOCK_SIZE;
	s32 ci = i - bi*BLOCK_SIZE;
	s32 cj = j - bj*BLOCK_SIZE;
	s32 idx = bj*fieldData.numBlockX + bi;

	for(s32 k=0;k < hfc.numCache;k++) {
		if(hfc.blockIdx[k] == idx)
			return hfc.cachedBlock[k].heightBuf[cj*BLOCK_SIZE_B + ci];
	}

	{
		spu_dma_large_get(&hfc.cachedBlock[hfc.blockIndicator],	(u64)&fieldData.blocks[idx], sizeof(HeightFieldBlock), 1, 0, 0);

		s32 k = hfc.blockIndicator;
		hfc.blockIdx[hfc.blockIndicator] = idx;
		hfc.blockIndicator = (hfc.blockIndicator + 1)%hfc.numCache;

		spu_dma_wait_tag_status_all(1<<1);

		return hfc.cachedBlock[k].heightBuf[cj*BLOCK_SIZE_B + ci];
	}
}

#endif

///////////////////////////////////////////////////////////////////////////////
// Public Function

bool getHeight(const HeightField& heightfield, f32 x, f32 z, f32& h)
{
	if(x < 0 || x > heightfield.fieldData.fieldWidth || z < 0 || z > heightfield.fieldData.fieldDepth)
		return false;

	s32 i = (s32)x;
	s32 j = (s32)z;

	f32 xx = x - i;
	f32 zz = z - j;

	// 面の高さを取得
#ifdef __SPU__
	f32 h1 = getCachedFieldData(heightfield.fieldData, i + 0, j + 0);
	f32 h2 = getCachedFieldData(heightfield.fieldData, i + 1, j + 0);
	f32 h3 = getCachedFieldData(heightfield.fieldData, i + 0, j + 1);
	f32 h4 = getCachedFieldData(heightfield.fieldData, i + 1, j + 1);
#else
	f32 h1 = getFieldData(heightfield.fieldData, i + 0, j + 0);
	f32 h2 = getFieldData(heightfield.fieldData, i + 1, j + 0);
	f32 h3 = getFieldData(heightfield.fieldData, i + 0, j + 1);
	f32 h4 = getFieldData(heightfield.fieldData, i + 1, j + 1);
#endif

	if(xx < zz) {
		h2 = (h1 + h4)*0.5f;
		h2 += h2 - h3;
	} else {
		h3 = (h1 + h4)*0.5f;
		h3 += h3 - h2;
	}

	h = interpolate(interpolate(h1, h2, xx), interpolate(h3, h4, xx), zz);

	return true;
}

bool getNormal(const HeightField& heightfield, f32 x, f32 z, Vector3& nml)
{
	s32 i = (s32)x;
	s32 j = (s32)z;

	f32 xx = x - i;
	f32 zz = z - j;

	if(i < 0 || i > heightfield.fieldData.fieldWidth || j < 0 || j > heightfield.fieldData.fieldDepth)
		return false;

#ifdef __SPU__
	f32 h1 = getCachedFieldData(heightfield.fieldData, i + 0, j + 0);
	f32 h2 = getCachedFieldData(heightfield.fieldData, i + 1, j + 0);
	f32 h3 = getCachedFieldData(heightfield.fieldData, i + 0, j + 1);
	f32 h4 = getCachedFieldData(heightfield.fieldData, i + 1, j + 1);
#else
	f32 h1 = getFieldData(heightfield.fieldData, i + 0, j + 0);
	f32 h2 = getFieldData(heightfield.fieldData, i + 1, j + 0);
	f32 h3 = getFieldData(heightfield.fieldData, i + 0, j + 1);
	f32 h4 = getFieldData(heightfield.fieldData, i + 1, j + 1);
#endif

	Vector3 p[4] =
	{
		Vector3(i + 0, h1, j + 0),
		Vector3(i + 1, h2, j + 0),
		Vector3(i + 0, h3, j + 1),
		Vector3(i + 1, h4, j + 1)
	};

	if(xx + zz < 1.0f) {
		p[0] = mulPerElem(p[0], heightfield.getScale());
		p[2] = mulPerElem(p[2], heightfield.getScale());
		p[1] = mulPerElem(p[1], heightfield.getScale());
		nml = normalize(cross(p[2] - p[0], p[1] - p[0]));
	} else {
		p[3] = mulPerElem(p[3], heightfield.getScale());
		p[1] = mulPerElem(p[1], heightfield.getScale());
		p[2] = mulPerElem(p[2], heightfield.getScale());
		nml = normalize(cross(p[1] - p[3], p[2] - p[3]));
	}

	return true;
}

#ifdef __SPU__

void initializeHeightFieldCache()
{
	hfc.numCache = HEIGHTFIELD_CACHE_COUNT;
	hfc.cachedBlock = (HeightFieldBlock*)gPool.allocate(sizeof(HeightFieldBlock)*hfc.numCache, HeapManager::ALIGN128);
	hfc.blockIdx = (s32*)gPool.allocate(sizeof(s32)*hfc.numCache, HeapManager::ALIGN128);

	hfc.blockIndicator = 0;
	for(s32 i=0;i < hfc.numCache;i++)
		hfc.blockIdx[i] = -1;
}

void  releaseHeightFieldCache()
{
	gPool.deallocate(hfc.blockIdx);
	gPool.deallocate(hfc.cachedBlock);
}

#endif
