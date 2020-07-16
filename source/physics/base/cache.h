/*
 * cache.h
 *
 *  Created on: Jun 1, 2013
 *      Author: mike
 */

#ifndef CACHE_H_
#define CACHE_H_

#include "common.h"
#include "heapmanager.h"

template< class T >
class Cache
{
private:
	ATTRIBUTE_ALIGNED16(HeapManager *mPool);
	ATTRIBUTE_ALIGNED16(const s32 mTag);
	ATTRIBUTE_ALIGNED16(const s32 mNumBuffer);
	ATTRIBUTE_ALIGNED16(const s32 mTotal);

	ATTRIBUTE_ALIGNED16(u32 mStartEA);
	ATTRIBUTE_ALIGNED16(u32 mSwap);
	ATTRIBUTE_ALIGNED16(T *mBuffer);
	ATTRIBUTE_ALIGNED16(T *mNextBuffer);
	ATTRIBUTE_ALIGNED16(u32 *mEATable);
	ATTRIBUTE_ALIGNED16(u8 *mIdTable);
	ATTRIBUTE_ALIGNED16(spu_dma_list_element *mDmaList[2]);
	ATTRIBUTE_ALIGNED16(u32 mNumList[2]);
	ATTRIBUTE_ALIGNED16(u32 mNumBatch);
	ATTRIBUTE_ALIGNED16(u32 mNumData);

public:
	Cache(HeapManager *pool, u32 startEA, u32 numBuffer, u32 total, u32 batch, s32 tag = 0)
	: mPool(pool), mTag(tag), mNumBuffer(numBuffer), mTotal(total), mStartEA(startEA), mSwap(0), mNumBatch(batch)
	{
		mBuffer = (T*)mPool->allocate(sizeof(T)*mNumBuffer, HeapManager::ALIGN128);
		memset(mBuffer, 0xff, sizeof(T)*mNumBuffer);
		mEATable = (u32*)mPool->allocate(sizeof(uint32_t)*mNumBuffer);
		mIdTable = (u8*)mPool->allocate(sizeof(u8)*mTotal);
		memset(mIdTable,0xff,sizeof(u8)*mTotal);
		mDmaList[0] = (spu_dma_list_element*)mPool->allocate(sizeof(spu_dma_list_element)*mNumBatch);
		mDmaList[1] = (spu_dma_list_element*)mPool->allocate(sizeof(spu_dma_list_element)*mNumBatch);
		for(u32 i=0;i < mNumBatch;i++) {
			mDmaList[0][i].notify = mDmaList[1][i].notify = 0;
			mDmaList[0][i].size = mDmaList[1][i].size = (u64)sizeof(T);
		}

		mNextBuffer = mBuffer;
		mNumData = 0;
		mNumList[0] = mNumList[1] = 0;
	}

	~Cache()
	{
		mPool->deallocate(mDmaList[1]);
		mPool->deallocate(mDmaList[0]);
		mPool->deallocate(mIdTable);
		mPool->deallocate(mEATable);
		mPool->deallocate(mBuffer);
	}

	T& get(u32 id)
	{
		ASSERT(mIdTable[id] < 0xff);
		return mBuffer[mIdTable[id]];
	}

	void fetch(u32 id)
	{
		if(mIdTable[id] == 0xff) {
			u32 ea = mStartEA + sizeof(T)*id;
			mEATable[mNumData] = ea;
			mDmaList[mSwap][mNumList[mSwap]].eal = (u64)ea;
			mIdTable[id] = mNumData;
			mNumList[mSwap]++;
			mNumData++;
		}
	}

	void swap()
	{
		spu_dma_list_get(mNextBuffer, 0, mDmaList[mSwap], sizeof(spu_dma_list_element)*mNumList[mSwap], mTag, 0, 0);
		mNextBuffer += mNumList[mSwap];
		mSwap = mSwap^0x01;
		mNumList[mSwap] = 0;
	}

	void putAll()
	{
		ATTRIBUTE_ALIGNED16(spu_dma_list_element *dmaList) = (spu_dma_list_element*)mPool->allocate(sizeof(spu_dma_list_element)*mNumData);
		for(u32 i=0;i < mNumData;i++) {
			dmaList[i].notify = 0;
			dmaList[i].eal = mEATable[i];
			dmaList[i].size = sizeof(T);
		}
		spu_dma_list_put(mBuffer, 0, dmaList, sizeof(spu_dma_list_element)*mNumData, mTag, 0, 0);
		spu_dma_wait_tag_status_all(1<<mTag);
		mPool->deallocate(dmaList);
	}
};

#endif /* CACHE_H_ */
