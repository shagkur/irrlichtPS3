/*
 * main.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/heapmanager.h"
#include "base/prefetchiterator.h"
#include "base/safedma.h"

#include "heightfluid/common/heightfluidconfig.h"
#include "heightfluid/common/heightfluidio.h"

#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/forces.h"

#define DMA_TAG1				1
#define DMA_TAG2				2

#define PREFETCH_NUM 			32

#define FIELD_LIMIT_SIZE 		(64*1024)

#define HEAP_BYTES 				(96*1024)

#define IFIELD(a, b) 			((b)*io.fieldWidth + (a))

#define ALLOCATE(align,size) 	gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16);
#define DEALLOCATE(ptr) 		if(ptr) {gPool.deallocate(((void*)ptr)); ptr = NULL;}

ATTRIBUTE_ALIGNED128(u32 commonBuff[32]);
ATTRIBUTE_ALIGNED128(u32 lockBuffer[32]);

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool, HEAP_BYTES);

HeightFluidIO io;

u32 taskId;
u32 barrier = 0;

void sync()
{
	while(mars_task_barrier_try_notify(barrier) != MARS_SUCCESS) {}
	while(mars_task_barrier_try_wait(barrier) != MARS_SUCCESS) {}
}

void lock(u32 *lsBuff, u32 eaBuff)
{
	s32 ch = 1;
	do {
		spu_dma_getllar(lsBuff, (u64)eaBuff, 0, 0);
		spu_dma_wait_atomic_status();
		spu_dsync();

		if(UNLIKELY(lsBuff[0] == 1))
			continue;

		lsBuff[0] = 1; // LOCKED

		spu_dsync();
		spu_dma_putllc(lsBuff, (u64)eaBuff, 0, 0);
		ch = spu_dma_wait_atomic_status();
	} while(UNLIKELY(ch != 0));
}

void unlock(u32 *lsBuff, u32 eaBuff)
{
	s32 ch = 1;
	do {
		spu_dma_getllar(lockBuffer, (u64)eaBuff, 0, 0);
		spu_dma_wait_atomic_status();
		spu_dsync();

		lsBuff[0] = 0; // UNLOCKED

		spu_dsync();
		spu_dma_putllc(lsBuff, (u64)eaBuff, 0, 0);
		ch = spu_dma_wait_atomic_status();
	} while(UNLIKELY(ch != 0));
}

class LockedField
{
private:
	FieldPoint *localFields;
	s32 is, ie, js, je, w, d;

	struct FieldAccess
	{
		u8 lock;
		u8 reserved0;
		u8 reserved1;
		u8 reserved2;
		s16 is;
		s16 ie;
		s16 js;
		s16 je;
	};

	bool checkAccess()
	{
		for(u32 i=0;i < io.numSPU;i++) {
			FieldAccess *fa = (FieldAccess*)(&commonBuff[1] + 3*i);
			if(fa->lock == 1) {
				if(fa->ie < is || ie < fa->is) continue;
				if(fa->je < js || je < fa->js) continue;
				return false;
			}
		}
		return true;
	}

	void lockField()
	{
		w = ie - is + 1;
		d = je - js + 1;

		{
			bool canAccess = false;
			do {
				lock(commonBuff, io.applyForces.commonAddr);

				if(checkAccess()) {
					canAccess = true;
					FieldAccess *fa = (FieldAccess*)(&commonBuff[1] + 3*taskId);
					fa->lock = 1;
					fa->is = is;
					fa->ie = ie;
					fa->js = js;
					fa->je = je;
				}

				unlock(commonBuff, io.applyForces.commonAddr);
			} while(!canAccess);
		}

		ASSERT((w*d*sizeof(FieldPoint)) < FIELD_LIMIT_SIZE);

		localFields = (FieldPoint*)gPool.allocate(sizeof(FieldPoint)*w*d);

		for(s32 j=js;j <= je;j++)
			spu_dma_get(localFields + w*(j - js), io.fieldAddr + sizeof(FieldPoint)*(j*io.fieldWidth + is), sizeof(FieldPoint)*w, DMA_TAG1, 0, 0);

		spu_dma_wait_tag_status_all(1<<DMA_TAG1);
	}

	void unlockField()
	{
		for(s32 j=js;j<=je;j++)
			spu_dma_put(localFields + w*(j - js), io.fieldAddr + sizeof(FieldPoint)*(j*io.fieldWidth + is), sizeof(FieldPoint)*w, DMA_TAG1, 0, 0);

		spu_dma_wait_tag_status_all(1<<DMA_TAG1);

		{
			lock(commonBuff, io.applyForces.commonAddr);
			FieldAccess *fa = (FieldAccess*)(&commonBuff[1] + 3*taskId);
			fa->lock = 0;
			unlock(commonBuff, io.applyForces.commonAddr);
		}

		gPool.deallocate(localFields);
	}

public:
	LockedField(s32 iStart, s32 iEnd, s32 jStart, s32 jEnd)
	{
		is = (s16)iStart;
		ie = (s16)iEnd;
		js = (s16)jStart;
		je = (s16)jEnd;

		lockField();
	}

	~LockedField()
	{
		unlockField();
	}

	FieldPoint& getField(s32 i, s32 j)
	{
		ASSERT(j >= js && j <= je);
		ASSERT(i >= is && i <= ie);
		return localFields[(j - js)*w + (i - is)];
	}
};

void getHeightFluid(u32 addrHeightFluidIO)
{
	spu_dma_get(&io, addrHeightFluidIO, sizeof(HeightFluidIO), DMA_TAG1, 0, 0);
	spu_dma_wait_tag_status_all(1<<DMA_TAG1);
}

void putHeightFluid(u32 addrHeightFluidIO)
{
	spu_dma_put(&io, addrHeightFluidIO, sizeof(HeightFluidIO), DMA_TAG1, 0, 0);
	spu_dma_wait_tag_status_all(1<<DMA_TAG1);
}

inline Vector3 localToWorldPosition(const Vector3& localPosition)
{
	return io.fieldPosition + rotate(io.fieldOrientation, mulPerElem(io.fieldScale, (localPosition - 0.5f*Vector3(io.fieldWidth, 0.0f, io.fieldDepth))));
}

inline Vector3 worldToLocalPosition(const Vector3& worldPosition)
{
	return divPerElem(rotate(conj(io.fieldOrientation), (worldPosition - io.fieldPosition)), io.fieldScale) + 0.5f*Vector3(io.fieldWidth, 0.0f, io.fieldDepth);
}

inline Vector3 localToWorldVector(const Vector3& localVector)
{
	return rotate(io.fieldOrientation, mulPerElem(io.fieldScale, localVector));
}

inline Vector3 worldToLocalVector(const Vector3& worldVector)
{
	return divPerElem(rotate(conj(io.fieldOrientation), worldVector), io.fieldScale);
}

class FieldMgr
{
private:
	ATTRIBUTE_ALIGNED16(const s32 mDmaTag);
	ATTRIBUTE_ALIGNED16(HeapManager *mPool);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchNum);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchSize);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchSizeMinus2);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchSizeMinus4);
	ATTRIBUTE_ALIGNED16(s32 mCurrentBuffer);

	ATTRIBUTE_ALIGNED16(u32 sEA0);
	ATTRIBUTE_ALIGNED16(u32 sEA1);
	ATTRIBUTE_ALIGNED16(u32 sEA2);
	ATTRIBUTE_ALIGNED16(u32 eEA0);
	ATTRIBUTE_ALIGNED16(u32 eEA1);
	ATTRIBUTE_ALIGNED16(u32 eEA2);

	ATTRIBUTE_ALIGNED16(s32 mIndex);

	ATTRIBUTE_ALIGNED16(FieldPoint *scanLine0[2]);
	ATTRIBUTE_ALIGNED16(FieldPoint *scanLine1[2]);
	ATTRIBUTE_ALIGNED16(FieldPoint *scanLine2[2]);

public:
	FieldMgr(HeapManager *pool, s32 line, s32 numData, s32 prefetchNum, s32 tag)
		:mDmaTag(tag),mPool(pool),mPrefetchNum(prefetchNum),mCurrentBuffer(0),mIndex(0)
	{
		mPrefetchSize = sizeof(FieldPoint)*prefetchNum;
		mPrefetchSizeMinus2 = sizeof(FieldPoint)*(prefetchNum - 2);
		mPrefetchSizeMinus4 = sizeof(FieldPoint)*(prefetchNum - 4);

		scanLine0[0] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine0[1] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine1[0] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine1[1] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine2[0] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine2[1] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);

		sEA0 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(0      , line - 1);
		sEA1 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(0      , line    );
		sEA2 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(0      , line + 1);
		eEA0 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(numData, line - 1);
		eEA1 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(numData, line    );
		eEA2 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(numData, line + 1);

		dmaGetBuffer(scanLine0[0], sEA0, eEA0, mPrefetchSize, mDmaTag);
		dmaGetBuffer(scanLine1[0], sEA1, eEA1, mPrefetchSize, mDmaTag);
		dmaGetBuffer(scanLine2[0], sEA2, eEA2, mPrefetchSize, mDmaTag);
		spu_dma_wait_tag_status_all(1<<mDmaTag);

		dmaGetBuffer(scanLine0[1],sEA0 + mPrefetchSizeMinus2, eEA0, mPrefetchSize, mDmaTag);
		dmaGetBuffer(scanLine1[1],sEA1 + mPrefetchSizeMinus2, eEA1, mPrefetchSize, mDmaTag);
		dmaGetBuffer(scanLine2[1],sEA2 + mPrefetchSizeMinus2, eEA2, mPrefetchSize, mDmaTag);

		mIndex = 2;
		sEA0 += sizeof(FieldPoint)*2;
		sEA1 += sizeof(FieldPoint)*2;
		sEA2 += sizeof(FieldPoint)*2;
	}

	~FieldMgr()
	{
		dmaPutBuffer(scanLine1[mCurrentBuffer], sEA1 - (mIndex*sizeof(FieldPoint)), eEA1, (mIndex*sizeof(FieldPoint)), mDmaTag);
		spu_dma_wait_tag_status_all(1<<mDmaTag);

		mPool->deallocate(scanLine2[1]);
		mPool->deallocate(scanLine2[0]);
		mPool->deallocate(scanLine1[1]);
		mPool->deallocate(scanLine1[0]);
		mPool->deallocate(scanLine0[1]);
		mPool->deallocate(scanLine0[0]);
	}

	void fetch()
	{
		sEA0 += sizeof(FieldPoint);
		sEA1 += sizeof(FieldPoint);
		sEA2 += sizeof(FieldPoint);
		mIndex++;
		if(mIndex>=mPrefetchNum) {
			spu_dma_wait_tag_status_all(1<<mDmaTag);

			scanLine1[1 - mCurrentBuffer][0] = scanLine1[mCurrentBuffer][mPrefetchNum - 2];

			dmaPutBuffer(scanLine1[mCurrentBuffer], sEA1 - mPrefetchSize, eEA1, mPrefetchSizeMinus2, mDmaTag);
			dmaGetBufferb(scanLine0[mCurrentBuffer], sEA0 + mPrefetchSizeMinus4, eEA0, mPrefetchSize, mDmaTag);
			dmaGetBuffer(scanLine1[mCurrentBuffer], sEA1 + mPrefetchSizeMinus4, eEA1, mPrefetchSize, mDmaTag);
			dmaGetBuffer(scanLine2[mCurrentBuffer], sEA2 + mPrefetchSizeMinus4, eEA2, mPrefetchSize, mDmaTag);
			mCurrentBuffer = 1 - mCurrentBuffer;
			mIndex = 2;
		}
	}

	void calc()
	{
		FieldPoint fp01 = scanLine0[mCurrentBuffer][mIndex - 1]; // i  ,j-1
		FieldPoint fp21 = scanLine2[mCurrentBuffer][mIndex - 1]; // i  ,j+1
		FieldPoint fp10 = scanLine1[mCurrentBuffer][mIndex - 2]; // i-1,j
		FieldPoint& fp11 = scanLine1[mCurrentBuffer][mIndex - 1]; // i  ,j
		FieldPoint fp12 = scanLine1[mCurrentBuffer][mIndex]; // i+1,j

		if(fp11.flag != 0) return;

		f32 timeStep = io.calcWave.timeStep;
		f32 kConst = io.calcWave.kConst;
		f32 cConst = io.calcWave.cConst;
		f32 deltaX = io.calcWave.deltaX;
		f32 heightMin = io.calcWave.heightMin;
		f32 heightMax = io.calcWave.heightMax;

		fp11.height[2] = 2.0f*fp11.height[1] - fp11.height[0] - kConst*timeStep*(fp11.height[1] - fp11.height[0]) +
						 ((cConst*cConst*timeStep*timeStep)/(deltaX*deltaX)) *
						 (-4.0f*fp11.height[1] + fp12.height[1] + fp10.height[1] + fp21.height[1] + fp01.height[1]);

		fp11.height[2] = MIN(MAX(fp11.height[2], heightMin), heightMax);
	}
};

void calcWave()
{
	// Phase1
	for(u32 j=1;j < io.fieldDepth - 1;j+=2) {
		if((j>>1)%io.numSPU == taskId) {
			FieldMgr fieldMgr(&gPool, j, io.fieldWidth, PREFETCH_NUM, 10);
			for(u32 i=1;i < io.fieldWidth - 1;i++, fieldMgr.fetch())
				fieldMgr.calc();
		}
	}

	sync();

	// Phase2
	for(u32 j=2;j < io.fieldDepth - 1;j+=2) {
		if((j>>1)%io.numSPU == taskId) {
			FieldMgr fieldMgr(&gPool, j, io.fieldWidth, PREFETCH_NUM, 10);
			for(u32 i=1;i < io.fieldWidth - 1;i++, fieldMgr.fetch())
				fieldMgr.calc();
		}
	}

	sync();
}

#define FIELD_HEIGHT(fp, amp) 			(((fp.offset/255.0f) - 0.5f)*amp + fp.height[2])

class SurfaceMgr
{
private:
	const s32 mDmaTag;
	HeapManager *mPool;
	s32 mPrefetchNum;
	s32 mPrefetchSize;
	s32 mPrefetchSizeMinus2;
	s32 mPrefetchSizeMinus4;
	s32 mVec3PrefetchSize;
	s32 mVec3PrefetchSizeMinus2;
	s32 mVec3PrefetchSizeMinus4;
	s32 mCurrentBuffer;

	ATTRIBUTE_ALIGNED16(u32 sFldEA0);
	ATTRIBUTE_ALIGNED16(u32 sFldEA1);
	ATTRIBUTE_ALIGNED16(u32 sFldEA2);
	ATTRIBUTE_ALIGNED16(u32 eFldEA0);
	ATTRIBUTE_ALIGNED16(u32 eFldEA1);
	ATTRIBUTE_ALIGNED16(u32 eFldEA2);

	ATTRIBUTE_ALIGNED16(u32 sVtxEA);
	ATTRIBUTE_ALIGNED16(u32 eVtxEA);
	ATTRIBUTE_ALIGNED16(u32 sNmlEA);
	ATTRIBUTE_ALIGNED16(u32 eNmlEA);

	ATTRIBUTE_ALIGNED16(s32 mIndex);

	ATTRIBUTE_ALIGNED16(FieldPoint *scanLine0[2]);
	ATTRIBUTE_ALIGNED16(FieldPoint *scanLine1[2]);
	ATTRIBUTE_ALIGNED16(FieldPoint *scanLine2[2]);

	ATTRIBUTE_ALIGNED16(Vector3 *vtxLine[2]);
	ATTRIBUTE_ALIGNED16(Vector3 *nmlLine[2]);

public:
	SurfaceMgr(HeapManager *pool, s32 line, s32 numData, s32 prefetchNum, s32 tag)
		:mDmaTag(tag),mPool(pool),mPrefetchNum(prefetchNum),mCurrentBuffer(0),mIndex(0)
	{
		mPrefetchSize = sizeof(FieldPoint)*prefetchNum;
		mPrefetchSizeMinus2 = sizeof(FieldPoint)*(prefetchNum - 2);
		mPrefetchSizeMinus4 = sizeof(FieldPoint)*(prefetchNum - 4);

		mVec3PrefetchSize = sizeof(Vector3)*prefetchNum;
		mVec3PrefetchSizeMinus2 = sizeof(Vector3)*(prefetchNum - 2);
		mVec3PrefetchSizeMinus4 = sizeof(Vector3)*(prefetchNum - 4);

		scanLine0[0] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine0[1] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine1[0] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine1[1] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine2[0] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		scanLine2[1] = (FieldPoint*)mPool->allocate(mPrefetchSize, HeapManager::ALIGN128);
		vtxLine[0] = (Vector3*)mPool->allocate(mVec3PrefetchSize, HeapManager::ALIGN128);
		vtxLine[1] = (Vector3*)mPool->allocate(mVec3PrefetchSize, HeapManager::ALIGN128);
		nmlLine[0] = (Vector3*)mPool->allocate(mVec3PrefetchSize, HeapManager::ALIGN128);
		nmlLine[1] = (Vector3*)mPool->allocate(mVec3PrefetchSize, HeapManager::ALIGN128);

		sFldEA0 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(0      , line - 1);
		sFldEA1 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(0      , line    );
		sFldEA2 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(0      , line + 1);
		eFldEA0 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(numData, line - 1);
		eFldEA1 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(numData, line    );
		eFldEA2 = io.fieldAddr + sizeof(FieldPoint)*IFIELD(numData, line + 1);

		sVtxEA = io.calcSurface.vtxAddr + sizeof(Vector3)*IFIELD(0      , line);
		eVtxEA = io.calcSurface.vtxAddr + sizeof(Vector3)*IFIELD(numData, line);
		sNmlEA = io.calcSurface.nmlAddr + sizeof(Vector3)*IFIELD(0      , line);
		eNmlEA = io.calcSurface.nmlAddr + sizeof(Vector3)*IFIELD(numData, line);

		dmaGetBuffer(scanLine0[0], sFldEA0, eFldEA0, mPrefetchSize, mDmaTag);
		dmaGetBuffer(scanLine1[0], sFldEA1, eFldEA1, mPrefetchSize, mDmaTag);
		dmaGetBuffer(scanLine2[0], sFldEA2, eFldEA2, mPrefetchSize, mDmaTag);
		dmaGetBuffer(vtxLine[0], sVtxEA, eVtxEA, mVec3PrefetchSize, mDmaTag);
		dmaGetBuffer(nmlLine[0], sNmlEA, eNmlEA, mVec3PrefetchSize, mDmaTag);
		spu_dma_wait_tag_status_all(1<<mDmaTag);

		dmaGetBuffer(scanLine0[1], sFldEA0 + mPrefetchSizeMinus2, eFldEA0, mPrefetchSize, mDmaTag);
		dmaGetBuffer(scanLine1[1], sFldEA1 + mPrefetchSizeMinus2, eFldEA1, mPrefetchSize, mDmaTag);
		dmaGetBuffer(scanLine2[1], sFldEA2 + mPrefetchSizeMinus2, eFldEA2, mPrefetchSize, mDmaTag);
		dmaGetBuffer(vtxLine[1], sVtxEA + mVec3PrefetchSizeMinus2, eVtxEA, mVec3PrefetchSize, mDmaTag);
		dmaGetBuffer(nmlLine[1], sNmlEA + mVec3PrefetchSizeMinus2, eNmlEA, mVec3PrefetchSize, mDmaTag);

		mIndex += 2;
		sFldEA0 += sizeof(FieldPoint)*2;
		sFldEA1 += sizeof(FieldPoint)*2;
		sFldEA2 += sizeof(FieldPoint)*2;
		sVtxEA += sizeof(Vector3)*2;
		sNmlEA += sizeof(Vector3)*2;
	}

	~SurfaceMgr()
	{
		dmaPutBuffer(vtxLine[mCurrentBuffer], sVtxEA - (mIndex*sizeof(Vector3)), eVtxEA, (mIndex*sizeof(Vector3)), mDmaTag);
		dmaPutBuffer(nmlLine[mCurrentBuffer], sNmlEA - (mIndex*sizeof(Vector3)), eNmlEA, (mIndex*sizeof(Vector3)), mDmaTag);
		spu_dma_wait_tag_status_all(1<<mDmaTag);

		mPool->deallocate(nmlLine[1]);
		mPool->deallocate(nmlLine[0]);
		mPool->deallocate(vtxLine[1]);
		mPool->deallocate(vtxLine[0]);
		mPool->deallocate(scanLine2[1]);
		mPool->deallocate(scanLine2[0]);
		mPool->deallocate(scanLine1[1]);
		mPool->deallocate(scanLine1[0]);
		mPool->deallocate(scanLine0[1]);
		mPool->deallocate(scanLine0[0]);
	}

	void fetch()
	{
		sFldEA0 += sizeof(FieldPoint);
		sFldEA1 += sizeof(FieldPoint);
		sFldEA2 += sizeof(FieldPoint);
		sVtxEA += sizeof(Vector3);
		sNmlEA += sizeof(Vector3);
		mIndex++;
		if(mIndex >= mPrefetchNum) {
			spu_dma_wait_tag_status_all(1<<mDmaTag);

			vtxLine[1-mCurrentBuffer][0] = vtxLine[mCurrentBuffer][mPrefetchNum - 2];
			nmlLine[1-mCurrentBuffer][0] = nmlLine[mCurrentBuffer][mPrefetchNum - 2];

			dmaPutBuffer(vtxLine[mCurrentBuffer], sVtxEA - mVec3PrefetchSize, eVtxEA, mVec3PrefetchSizeMinus2, mDmaTag);
			dmaPutBuffer(nmlLine[mCurrentBuffer], sNmlEA - mVec3PrefetchSize, eNmlEA, mVec3PrefetchSizeMinus2, mDmaTag);
			dmaGetBufferb(scanLine0[mCurrentBuffer], sFldEA0 + mPrefetchSizeMinus4, eFldEA0, mPrefetchSize, mDmaTag);
			dmaGetBuffer(scanLine1[mCurrentBuffer], sFldEA1 + mPrefetchSizeMinus4, eFldEA1, mPrefetchSize, mDmaTag);
			dmaGetBuffer(scanLine2[mCurrentBuffer], sFldEA2 + mPrefetchSizeMinus4, eFldEA2, mPrefetchSize, mDmaTag);
			dmaGetBuffer(vtxLine[mCurrentBuffer], sVtxEA + mVec3PrefetchSizeMinus4, eVtxEA, mVec3PrefetchSize, mDmaTag);
			dmaGetBuffer(nmlLine[mCurrentBuffer], sNmlEA + mVec3PrefetchSizeMinus4, eNmlEA, mVec3PrefetchSize, mDmaTag);

			mCurrentBuffer = 1 - mCurrentBuffer;
			mIndex = 2;
		}
	}

	void calc()
	{
		FieldPoint fp01 = scanLine0[mCurrentBuffer][mIndex - 1]; // i  ,j-1
		FieldPoint fp02 = scanLine0[mCurrentBuffer][mIndex    ]; // i+1,j-1

		FieldPoint fp21 = scanLine2[mCurrentBuffer][mIndex - 1]; // i  ,j+1
		FieldPoint fp20 = scanLine2[mCurrentBuffer][mIndex - 2]; // i-1,j+1

		FieldPoint fp10 = scanLine1[mCurrentBuffer][mIndex - 2]; // i-1,j
		FieldPoint fp11 = scanLine1[mCurrentBuffer][mIndex - 1]; // i  ,j
		FieldPoint fp12 = scanLine1[mCurrentBuffer][mIndex    ]; // i+1,j

		if(fp11.flag != 0) return;

		Vector3& vtx = vtxLine[mCurrentBuffer][mIndex - 1];
		Vector3& nml = nmlLine[mCurrentBuffer][mIndex - 1];

		f32 amp = io.calcSurface.amplitude;

		vtx[1] = io.fieldScale[1]*FIELD_HEIGHT(fp11, amp);

		Vector3 p  = Vector3( 0, FIELD_HEIGHT(fp11, amp),  0);
		Vector3 v1 = Vector3( 1, FIELD_HEIGHT(fp12, amp),  0) - p;
		Vector3 v2 = Vector3( 1, FIELD_HEIGHT(fp02, amp), -1) - p;
		Vector3 v3 = Vector3( 0, FIELD_HEIGHT(fp01, amp), -1) - p;
		Vector3 v4 = Vector3(-1, FIELD_HEIGHT(fp10, amp),  0) - p;
		Vector3 v5 = Vector3(-1, FIELD_HEIGHT(fp20, amp),  1) - p;
		Vector3 v6 = Vector3( 0, FIELD_HEIGHT(fp21, amp),  1) - p;

		nml = normalize(
						normalize(cross(v1, v2)) +
						normalize(cross(v2, v3)) +
						normalize(cross(v3, v4)) +
						normalize(cross(v4, v5)) +
						normalize(cross(v5, v6)) +
						normalize(cross(v6, v1))
					   );
	}
};

void calcPositionAndNormal()
{
	for(u32 j=1;j < io.fieldDepth - 1;j++) {
		if(j%io.numSPU == taskId) {
			SurfaceMgr surfaceMgr(&gPool, j, io.fieldWidth, PREFETCH_NUM, 10);
			for(u32 i=1;i < io.fieldWidth - 1;i++, surfaceMgr.fetch())
				surfaceMgr.calc();
		}
	}
}

void setMotionStage1()
{
	for(u32 j=0;j < io.fieldDepth;j++) {
		if(j%io.numSPU == taskId) {
			PrefetchForwardIterator<FieldPoint> itrField(
				&gPool,
				(u32)io.fieldAddr + sizeof(FieldPoint)*IFIELD(0, j),
				(u32)io.fieldAddr + sizeof(FieldPoint)*IFIELD(io.fieldWidth, j),
				PREFETCH_NUM, 10);

			for(u32 i=0;i < io.fieldWidth;i++, ++itrField) {
				FieldPoint& fp = *itrField;
				io.setMotion.reducedWater += fp.height[2];
			}
		}
	}
}

void setMotionStage2()
{
	for(u32 j=0;j < io.fieldDepth;j++) {
		if(j%io.numSPU == taskId) {
			PrefetchForwardIterator<FieldPoint> itrField(
				&gPool,
				(u32)io.fieldAddr + sizeof(FieldPoint)*IFIELD(0, j),
				(u32)io.fieldAddr + sizeof(FieldPoint)*IFIELD(io.fieldWidth, j),
				PREFETCH_NUM, 10);

			for(u32 i=0;i < io.fieldWidth;i++, ++itrField) {
				FieldPoint& fp = *itrField;
				fp.height[2] += io.setMotion.reducedWater;
				fp.height[0] = fp.height[1];
				fp.height[1] = fp.height[2];
			}
		}
	}
}

void applyForces()
{
	ReadOnlyPrefetchForwardIterator<TrbState> itrState(
		&gPool,
		io.applyForces.statesAddr + sizeof(TrbState)*io.applyForces.startState,
		io.applyForces.statesAddr + sizeof(TrbState)*(io.applyForces.startState + io.applyForces.numStates),
		PREFETCH_NUM, 10);

	PrefetchForwardIterator<Forces> itrForces(
		&gPool,
		io.applyForces.forcesAddr + sizeof(Forces)*io.applyForces.startState,
		io.applyForces.forcesAddr + sizeof(Forces)*(io.applyForces.startState + io.applyForces.numStates),
		PREFETCH_NUM, 11);

	CollObject coll;

	for(u32 rb=0;rb < io.applyForces.numStates;rb++, ++itrState, ++itrForces) {
		const TrbState& state = (*itrState);
		Forces& forces = (*itrForces);

		u8 mask = 1<<MoveTypeFixed;
		if(UNLIKELY(state.getMoveTypeBits()&mask)) continue;
		if(UNLIKELY(state.isDeleted())) continue;

		spu_dma_get(&coll, io.applyForces.collsAddr + sizeof(CollObject)*state.trbBodyIdx, sizeof(CollObject), 0, 0, 0);
		spu_dma_wait_tag_status_all(1);

		Vector3 bodyRadius = divPerElem(Vector3(coll.getHalf()), io.fieldScale); // Bounding Sphere

		Vector3 bodyPos = worldToLocalPosition(state.getPosition());

		// Bounding Sphere
		if(bodyPos[1] - bodyRadius[1] > 0.5f*io.applyForces.amplitude) continue;
		if(bodyPos[0] + bodyRadius[0] < 1.0f || bodyPos[0] - bodyRadius[0] > io.fieldWidth) continue;
		if(bodyPos[2] + bodyRadius[2] < 1.0f || bodyPos[2] - bodyRadius[2] > io.fieldDepth) continue;

		PrimIterator itrPrim(coll);
		for(s32 p=0;p < coll.getNumPrims();p++, ++itrPrim) {
			const CollPrim& prim = *itrPrim;
			Vector3 primPosW = state.getPosition() + prim.getObjectRelTransform().getTranslation();
			f32 primRadiusScalar = 0.0f;

			if(prim.getType() == SPHERE )
				primRadiusScalar = prim.getSphere().radius;
			else if(prim.getType() == BOX )
				primRadiusScalar = length(prim.getBox().half);
			else if(prim.getType() == CAPSULE )
				primRadiusScalar = prim.getCapsule().hLength + prim.getCapsule().radius;
			else if(prim.getType() == CONVEXMESH ) {
				ConvexMesh *mesh = (ConvexMesh*)gPool.allocate(sizeof(ConvexMesh), HeapManager::ALIGN16);
				prim.getConvexMesh(mesh);
				primRadiusScalar = length(Vector3(mesh->half[0], mesh->half[1], mesh->half[2]));
				gPool.deallocate(mesh);
			}

			Vector3 primPosL = worldToLocalPosition(primPosW);
			Vector3 primRadius = divPerElem(Vector3(primRadiusScalar), io.fieldScale);

			Vector3 floorL[5] =
			{
				primPosL + mulPerElem(primRadius,Vector3(-0.7f, -0.7f, -0.7f)),
				primPosL + mulPerElem(primRadius,Vector3( 0.7f, -0.7f,  0.7f)),
				primPosL + mulPerElem(primRadius,Vector3(-0.7f, -0.7f,  0.7f)),
				primPosL + mulPerElem(primRadius,Vector3( 0.7f, -0.7f, -0.7f)),
				primPosL + mulPerElem(primRadius,Vector3( 0.0f, -0.7f,  0.0f)),
			};

			if(primPosL[0] + primRadius[0] < 0.0f || primPosL[0] - primRadius[0] > io.fieldWidth) continue;
			if(primPosL[1] - primRadius[1] > 2.0f) continue;
			if(primPosL[2] + primRadius[2] < 0.0f || primPosL[2] - primRadius[2] > io.fieldDepth) continue;

			s32 iStart = (s32)MAX(primPosL[0] - primRadius[0], 1);
			s32 iEnd   = (s32)MIN(primPosL[0] + primRadius[0], io.fieldWidth - 2);
			s32 jStart = (s32)MAX(primPosL[2] - primRadius[2], 1);
			s32 jEnd   = (s32)MIN(primPosL[2] + primRadius[2], io.fieldDepth - 2);

			if(iEnd - iStart <= 0 || jEnd - jStart <= 0) continue;

			LockedField lockField(iStart, iEnd, jStart, jEnd);

			Vector3 velL = worldToLocalVector(state.getLinearVelocity());
			f32 waveRatio = MAX(-1.0f, MIN((velL[1] > 0.0f ? 1.0f : -1.0f)*length(velL), 1.0f));
			if(waveRatio > 0.0f) waveRatio *= 0.1f;

			for(s32 i=0;i < 5;i++) {
				s32 ci = MIN(MAX(floorL[i][0], iStart), iEnd - 1);
				s32 cj = MIN(MAX(floorL[i][2], jStart), jEnd - 1);

				if(ci < 0 || ci >= (s32)io.fieldWidth - 2 || cj < 0 || cj >= (s32)io.fieldDepth - 2)
					continue;

				FieldPoint& fp0 = lockField.getField(ci + 0, cj + 0);
				FieldPoint& fp1 = lockField.getField(ci + 1, cj + 0);
				FieldPoint& fp2 = lockField.getField(ci + 0, cj + 1);
				FieldPoint& fp3 = lockField.getField(ci + 1, cj + 1);

				f32 h0 = FIELD_HEIGHT(fp0, io.applyForces.amplitude);
				f32 h1 = FIELD_HEIGHT(fp1, io.applyForces.amplitude);
				f32 h2 = FIELD_HEIGHT(fp2, io.applyForces.amplitude);
				f32 h3 = FIELD_HEIGHT(fp3, io.applyForces.amplitude);

				f32 surfDepth = 0.25f*(h0 + h1 + h2 + h3);

				if(floorL[i][1] < surfDepth) {
					f32 depthDiff = surfDepth - floorL[i][1];
					Vector3 buoForce = depthDiff*Vector3(0.0f, io.applyForces.buoyPower, 0.0f);
					buoForce = localToWorldVector(buoForce);

					Vector3 fvec = floorL[i] - primPosL;
					fvec = rotate(io.fieldOrientation, mulPerElem(io.fieldScale, fvec));
					Vector3 vel = state.getLinearVelocity() + cross(state.getAngularVelocity(), fvec);
					Vector3 force = buoForce - io.applyForces.buoyDamping*vel;

					forces.force += force;
					forces.torque += cross(fvec, force);
				}
			}

			{
				s32 ci = MIN(MAX(primPosL[0], iStart), iEnd - 1);
				s32 cj = MIN(MAX(primPosL[2], jStart), jEnd - 1);

				if(LIKELY(!(ci < 1 || ci >= (s32)io.fieldWidth - 2 || cj < 1 || cj >= (s32)io.fieldDepth - 2))) {
					FieldPoint& fp0 = lockField.getField(ci + 0, cj + 0);
					FieldPoint& fp1 = lockField.getField(ci + 1, cj + 0);
					FieldPoint& fp2 = lockField.getField(ci + 0, cj + 1);
					FieldPoint& fp3 = lockField.getField(ci + 1, cj + 1);

					f32 h0 = FIELD_HEIGHT(fp0, io.applyForces.amplitude);
					f32 h1 = FIELD_HEIGHT(fp1, io.applyForces.amplitude);
					f32 h2 = FIELD_HEIGHT(fp2, io.applyForces.amplitude);
					f32 h3 = FIELD_HEIGHT(fp3, io.applyForces.amplitude);

					f32 centerDepth = primPosL[1] - primRadius[1];

					f32 surfDepth = 0.25f*(h0 + h1 + h2 + h3);

					if(LIKELY(centerDepth < surfDepth)) {
						if(fp0.flag == 0 && fp0.height[2] > io.applyForces.limitHeight) fp0.height[2] += waveRatio*io.applyForces.downCurrWave;
						if(fp0.flag == 0 && fp0.height[1] > io.applyForces.limitHeight) fp0.height[1] += waveRatio*io.applyForces.downPrevWave;

						if(fp1.flag == 0 && fp1.height[2] > io.applyForces.limitHeight) fp1.height[2] += waveRatio*io.applyForces.downCurrWave;
						if(fp1.flag == 0 && fp1.height[1] > io.applyForces.limitHeight) fp1.height[1] += waveRatio*io.applyForces.downPrevWave;

						if(fp2.flag == 0 && fp2.height[2] > io.applyForces.limitHeight) fp2.height[2] += waveRatio*io.applyForces.downCurrWave;
						if(fp2.flag == 0 && fp2.height[1] > io.applyForces.limitHeight) fp2.height[1] += waveRatio*io.applyForces.downPrevWave;

						if(fp3.flag == 0 && fp3.height[2] > io.applyForces.limitHeight) fp3.height[2] += waveRatio*io.applyForces.downCurrWave;
						if(fp3.flag == 0 && fp3.height[1] > io.applyForces.limitHeight) fp3.height[1] += waveRatio*io.applyForces.downPrevWave;
					}
				}
			}

			for(s32 j=jStart;j <= jEnd;j++) {
				for(s32 i=iStart;i <= iEnd;i++) {
					FieldPoint& fp = lockField.getField(i, j);

					f32 dx = i - primPosL[0];
					f32 dz = j - primPosL[2];
					f32 ddx = (dx*dx)/(primRadius[0]*primRadius[0]);
					f32 ddz = (dz*dz)/(primRadius[2]*primRadius[2]);
					f32 d = ddx + ddz;
					if(UNLIKELY(d > 0.9999f)) continue;

					d = primRadius[1]*sqrtf(1.0f - ddx - ddz);
					f32 bodyDepth = primPosL[1] - d;
					f32 bodyCeil  = primPosL[1] + d;
					if(bodyDepth > fp.height[2] || bodyCeil < fp.height[2]) continue;

					if(fp.flag == 0 && fp.height[2] > io.applyForces.limitHeight) fp.height[2] += waveRatio*io.applyForces.downCurrWave;
					if(fp.flag == 0 && fp.height[1] > io.applyForces.limitHeight) fp.height[1] += waveRatio*io.applyForces.downPrevWave;
				}
			}
		}
	}
}

int mars_task_main(const struct mars_task_args *task_args)
{
	u32 inQueueEa = task_args->type.u32[2];
	u32 outQueueEa = task_args->type.u32[3];

	barrier = task_args->type.u32[1];

	ATTRIBUTE_ALIGNED16(u32 taskbuff[8]);
	while(mars_task_queue_try_pop_begin(inQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_pop_end(inQueueEa, 0);

	taskId = taskbuff[4];

	u32 addrIo = taskbuff[0];
	u32 function = taskbuff[1];

	getHeightFluid(addrIo);

	switch(function) {
		case HEIGHTFLUID_CALCWAVE:
			calcWave();
			break;
		case HEIGHTFLUID_SETMOTION_STAGE1:
			setMotionStage1();
			break;
		case HEIGHTFLUID_SETMOTION_STAGE2:
			setMotionStage2();
			break;
		case HEIGHTFLUID_CALCMESH:
			calcPositionAndNormal();
			break;
		case HEIGHTFLUID_APPLY_FORCES:
			applyForces();
			break;
	}

	putHeightFluid(addrIo);

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}
