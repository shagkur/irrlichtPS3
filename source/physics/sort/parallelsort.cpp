/*
 * parallelsort.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/heapmanager.h"
#include "base/prefetchiterator.h"
#include "base/sortcommon.h"

#include "sort/bitonicsort.h"
#include "sort/hybridsort.h"
#include "sort/parallelsort.h"

#define PARALLEL_SORT_BATCH_MIN 			96
#define PARALLEL_SORT_BATCH_MAX 			768
#define MERGE_PREFETCH_NUM 					32

extern HeapManager gPool;
extern u32 barrier;

///////////////////////////////////////////////////////////////////////////////
// sync

static void sync()
{
	while(mars_task_barrier_try_notify(barrier) != MARS_SUCCESS) {}
	while(mars_task_barrier_try_wait(barrier) != MARS_SUCCESS) {}
}

///////////////////////////////////////////////////////////////////////////////
// Single Sort

SIMD_FORCE_INLINE void singlesort(SortData *d, SortData *buff, s32 n)
{
	(void) buff;

	if(n > 1) {
		SortData *data = (SortData*)gPool.allocate(n*sizeof(SortData));
		SortData *temp = (SortData*)gPool.allocate(n*sizeof(SortData));

		spu_dma_large_get(data, (u64)d, n*sizeof(SortData), 0, 0, 0);
		spu_dma_wait_tag_status_all(1);

		hybridsort(data, temp, n);

		spu_dma_large_put(data, (u64)d, n*sizeof(SortData), 0, 0, 0);
		spu_dma_wait_tag_status_all(1);

		gPool.deallocate(temp);
		gPool.deallocate(data);
	}
}

///////////////////////////////////////////////////////////////////////////////
// Merge Data

void mergeAreaOdd(SortData *inBuff1, u32 n1, SortData *inBuff2, u32 n2, SortData *outBuff)
{
	s32 count = (n1 + n2)/2;
	s32 i = 0, j = 0, c = 0;

    ReadOnlyPrefetchForwardIterator<SortData> itrIn1(
		&gPool,
		(u32)inBuff1,
		(u32)(inBuff1 + n1),
		MERGE_PREFETCH_NUM, 10);

	ReadOnlyPrefetchForwardIterator<SortData> itrIn2(
		&gPool,
		(u32)inBuff2,
		(u32)(inBuff2 + n2),
		MERGE_PREFETCH_NUM, 11);

	WriteOnlyPrefetchForwardIterator<SortData> itrOut(
		&gPool,
		(u32)outBuff,
		(u32)(outBuff + count),
		MERGE_PREFETCH_NUM, 12);

	while(i < (s32)n1 && j< (s32)n2 && c < count) {
		if(getKey(*itrIn1) < getKey(*itrIn2))	{
			*itrOut = *itrIn1;
			++itrIn1;
			i++;
		} else {
			*itrOut = *itrIn2;
			++itrIn2;
			j++;
		}
		c++;
		++itrOut;
    }

	if(c < count) {
		if(i < (s32)n1) {
			while(i < (s32)n1 && c < count) {
				*itrOut = *itrIn1;
				++itrIn1;
				i++;
				c++;
				++itrOut;
			}
		} else if(j < (s32)n2) {
			while(j < (s32)n2 && c < count) {
				*itrOut = *itrIn2;
				++itrIn2;
				j++;
				c++;
				++itrOut;
			}
		}
	}
}

void mergeAreaEven(SortData *inBuff1, u32 n1, SortData *inBuff2, u32 n2, SortData *outBuff)
{
	s32 odd_count = (n1 + n2)/2;
	s32 count = (n1 + n2) - odd_count;
	s32 i = n1, j = n2, c = 0;

    ReadOnlyPrefetchBackwardIterator<SortData> itrIn1(
		&gPool,
		(u32)(inBuff1 + n1),
		(u32)inBuff1,
		MERGE_PREFETCH_NUM, 10);

	ReadOnlyPrefetchBackwardIterator<SortData> itrIn2(
		&gPool,
		(u32)(inBuff2 + n2),
		(u32)inBuff2,
		MERGE_PREFETCH_NUM, 11);

	WriteOnlyPrefetchBackwardIterator<SortData> itrOut(
		&gPool,
		(u32)(outBuff + (n1+n2)),
		(u32)(outBuff + odd_count),
		MERGE_PREFETCH_NUM, 12);

	i--;
	j--;

	--itrIn1;
	--itrIn2;
	--itrOut;

	while(i >= 0 && j >= 0 && c < count) {
		if(getKey(*itrIn1) < getKey(*itrIn2)) {
			*itrOut = *itrIn2;
			--itrIn2;
			j--;
		} else {
			*itrOut = *itrIn1;
			--itrIn1;
			i--;
		}
		c++;
		--itrOut;
	}

	if(c < count) {
		if(i >= 0) {
			while(i >= 0 && c < count) {
				*itrOut = *itrIn1;
				--itrIn1;
				i--;
				c++;
				--itrOut;
			}
		} else if(j >= 0) {
			while(j >= 0 && c < count) {
				*itrOut = *itrIn2;
				--itrIn2;
				j--;
				c++;
				--itrOut;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Copy Data

void copyArea(SortData *inBuff, SortData *outBuff, u32 n)
{
    ReadOnlyPrefetchForwardIterator<SortData> itrIn(
		&gPool,
		(u32)inBuff,
		(u32)(inBuff + n),
		MERGE_PREFETCH_NUM, 10);

	WriteOnlyPrefetchForwardIterator<SortData> itrOut(
		&gPool,
		(u32)outBuff,
		(u32)(outBuff + n),
		MERGE_PREFETCH_NUM, 11);

	for(u32 i=0;i < n;i++, ++itrIn, ++itrOut)
		*itrOut=*itrIn;
}

///////////////////////////////////////////////////////////////////////////////
// Parallel sort

void parallelsort(SortData *data, SortData *buff, u32 numData, u32 taskId, u32 numTasks)
{
	u32 numBatch = numData/(numTasks*2);
	numBatch = CLAMP(numBatch, PARALLEL_SORT_BATCH_MIN, PARALLEL_SORT_BATCH_MAX);

	u32 numArea = (numData + numBatch - 1)/numBatch;

	for(u32 i=0;i < numArea;i++) {
		if(i%numTasks != taskId) continue;

		SortData *d = data + i*numBatch;
		SortData *b = buff + i*numBatch;
		u32 n = MIN(numData - i*numBatch, numBatch);

		singlesort(d, b, n);
	}
	sync();

	SortData *tmpBuff[2] = {data, buff};

	u32 btI = 0;
	u32 btO = 1;
	u32 numMergeArea = numArea;
	u32 numMergeBatch = numBatch;

	while(numMergeArea > 1) {
		for(u32 i=0;i < numMergeArea;i++) {
			if(i%numTasks != taskId) continue;

			if((i&0x01) == 1) {
				// Merge Odd
				SortData *inBuff1 = tmpBuff[btI] + (i - 1)*numMergeBatch;
				SortData *inBuff2 = tmpBuff[btI] + (i    )*numMergeBatch;
				SortData *outBuff = tmpBuff[btO] + (i - 1)*numMergeBatch;
				u32 n1 = MIN(numData - (i - 1)*numMergeBatch, numMergeBatch);
				u32 n2 = MIN(numData - (i    )*numMergeBatch, numMergeBatch);

				mergeAreaOdd(inBuff1, n1, inBuff2, n2, outBuff);
			} else if(i < numMergeArea - 1) {
				// Merge Even
				SortData *inBuff1 = tmpBuff[btI] + (i    )*numMergeBatch;
				SortData *inBuff2 = tmpBuff[btI] + (i + 1)*numMergeBatch;
				SortData *outBuff = tmpBuff[btO] + (i    )*numMergeBatch;
				u32 n1 = MIN(numData - (i    )*numMergeBatch, numMergeBatch);
				u32 n2 = MIN(numData - (i + 1)*numMergeBatch, numMergeBatch);

				mergeAreaEven(inBuff1, n1, inBuff2, n2, outBuff);
			} else {
				// Copy
				SortData *inBuff1 = tmpBuff[btI] + i*numMergeBatch;
				SortData *outBuff = tmpBuff[btO] + i*numMergeBatch;
				u32 n1 = MIN(numData - i*numMergeBatch, numMergeBatch);

				copyArea(inBuff1,outBuff,n1);
			}
		}
		btI = 1 - btI;
		btO = 1 - btO;
		numMergeArea = (numMergeArea + 1)/2;
		numMergeBatch = numMergeBatch*2;
		sync();
	}

	if(btO == 0) {
		for(u32 i=0;i < numArea;i++) {
			if(i%numTasks != taskId) continue;

			SortData *inBuff  = buff + i*numBatch;
			SortData *outBuff = data + i*numBatch;
			u32 n = MIN(numData - i*numBatch, numBatch);

			copyArea(inBuff, outBuff, n);
		}
		sync();
	}
}
