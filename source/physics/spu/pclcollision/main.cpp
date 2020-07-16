/*
 * main.cpp
 *
 *  Created on: Jan 27, 2014
 *      Author: mike
 */

#include "base/common.h"
#include "base/heapmanager.h"
#include "base/safedma.h"

#include "particle/common/particleconfig.h"
#include "particle/common/particleio.h"
#include "particle/common/pclcontact.h"
#include "particle/common/pclstate.h"
#include "particle/common/closestparticle.h"

#include "rigidbody/common/collobject.h"
#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/contact.h"

#define STATIC_MEM					0
#define DYNAMIC_MEM					(64*1024)
#define HEAP_BYTES 					(STATIC_MEM + DYNAMIC_MEM)

IOParamCollisionPcl io;

u32 taskId;
u32 barrier = 0;

ATTRIBUTE_ALIGNED128(u32 lockBuffer[32]);

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool,HEAP_BYTES);

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

void detectCollisionPcl(u32 startSortAddr, u32 numBatchSorts, s32 batchUnit = 16)
{
	const s32 tag1 = 8;

	u32 numBatch = batchUnit;
	u32 numBatchx2 = numBatch*2;
	u32 numBatchx4 = numBatch*4;

	SortData *sortBuffer[2];
	PclContactPair *pairBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];

	PclState *stateBuffer[2];
	spu_dma_list_element *stateDmaListBuf[2];

	sortBuffer[0] = (SortData*)gPool.allocate(sizeof(SortData)*numBatchx4, HeapManager::ALIGN128);
	sortBuffer[1] = (SortData*)gPool.allocate(sizeof(SortData)*numBatchx4, HeapManager::ALIGN128);
	pairBuffer[0] = (PclContactPair*)gPool.allocate(sizeof(PclContactPair)*numBatch, HeapManager::ALIGN128);
	pairBuffer[1] = (PclContactPair*)gPool.allocate(sizeof(PclContactPair)*numBatch, HeapManager::ALIGN128);

	stateBuffer[0] = (PclState*)gPool.allocate(sizeof(PclState)*numBatchx2, HeapManager::ALIGN128);
	stateBuffer[1] = (PclState*)gPool.allocate(sizeof(PclState)*numBatchx2, HeapManager::ALIGN128);

	pairDmaListBuf[0] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	pairDmaListBuf[1] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	pairDmaListBuf[2] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);

	stateDmaListBuf[0] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatchx2, HeapManager::ALIGN128);
	stateDmaListBuf[1] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatchx2, HeapManager::ALIGN128);

	memset(pairDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[2], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(stateDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatchx2);
	memset(stateDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatchx2);

	//PRINTF("SPU : task %d allocated %d bytes\n",taskId,gPool.getAllocated());

	//uint32_t startSortAddr = io.sortsAddr + sizeof(SortData) * io.startBatch;

	// 終端アドレスをセット
	u32 endEA = startSortAddr + sizeof(SortData)*numBatchSorts;

	s32 sortDataBatchSize = sizeof(SortData)*numBatch * 4;
	s32 batchFlag = 0;
	s32 currSort = 0;
	s32 currData = 0;
	u32 numPairs[2] = {0};
	u32 numSorts[2] = {0};
	u32 j,n;

	// あらかじめサイズをセット
	for(j=0,n=0;j < numBatch;j++,n++) {
		pairDmaListBuf[0][n].size = sizeof(PclContactPair);
		pairDmaListBuf[1][n].size = sizeof(PclContactPair);
		stateDmaListBuf[0][n*2 + 0].size = sizeof(PclState);
		stateDmaListBuf[0][n*2 + 1].size = sizeof(PclState);
		stateDmaListBuf[1][n*2 + 0].size = sizeof(PclState);
		stateDmaListBuf[1][n*2 + 1].size = sizeof(PclState);
	}

	// 初期状態
	//PRINTF("sortData dmaGetBuffer ls:0x%x ea:0x%x sz:%d\n",sortBuffer[0],io.sortsAddr,sortDataBatchSize);
	numSorts[0] = dmaGetBuffer(sortBuffer[0], startSortAddr, endEA, sortDataBatchSize, tag1);
	numSorts[0] /= sizeof(SortData);
	//PRINTF("numSorts %d\n",numSorts[0]);

	spu_dma_wait_tag_status_all(1<<tag1);

	for(j=0,n=0;j < numBatch && j < numSorts[0];j++,n++) {
		SortData& sort = sortBuffer[0][j];
		//PRINTF("    get key %d A %d B %d pair %d\n",Key(sort),StateA(sort),StateB(sort),Pair(sort));
		s32 n0 = n*2;
		s32 n1 = n*2+1;
		pairDmaListBuf[0][n].eal = io.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
		stateDmaListBuf[0][n0].eal = io.pclStatesAddr + sizeof(PclState)*getStateA(sort);
		stateDmaListBuf[0][n1].eal = io.pclStatesAddr + sizeof(PclState)*getStateB(sort);
	}
	numPairs[0] = n;
	//PRINTF("numPairs %d %d\n",numPairs[0],n);

	if(numPairs[0] > 0) {
		spu_dma_list_get(pairBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
		spu_dma_list_get(stateBuffer[0], 0, stateDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0]*2, tag1, 0, 0);
	}

	spu_dma_wait_tag_status_all(1<<tag1);

	//PRINTF("sortData dmaGetBuffer ls:0x%x ea:0x%x sz:%d\n",sortBuffer[1],io.sortsAddr+sortDataBatchSize,sortDataBatchSize);
	numSorts[1] = dmaGetBuffer(sortBuffer[1], startSortAddr + sortDataBatchSize, endEA, sortDataBatchSize, tag1);
	numSorts[1] /= sizeof(SortData);
	//PRINTF("numSorts %d\n",numSorts[1]);

	for(j=numBatch,n=0;j < numBatch + numBatch && j < numSorts[0];j++,n++) {
		SortData &sort = sortBuffer[0][j];
		//PRINTF("    taskId %d sort(%2d/%2d) key %7d pair %5d state %4d %4d body %3d %3d\n",taskId,n,numBatch,Key(sort),Pair(sort),StateA(sort),StateB(sort),BodyA(sort),BodyB(sort));
		s32 n0 = n*2;
		s32 n1 = n*2+1;
		pairDmaListBuf[1][n].eal = io.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
		stateDmaListBuf[1][n0].eal = io.pclStatesAddr + sizeof(PclState)*getStateA(sort);
		stateDmaListBuf[1][n1].eal = io.pclStatesAddr + sizeof(PclState)*getStateB(sort);
	}
	numPairs[1] = n;
	//PRINTF("numPairs %d\n",numPairs[1]);

	if(numPairs[1] > 0) {
		spu_dma_list_get(pairBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
		spu_dma_list_get(stateBuffer[1], 0, stateDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1]*2, tag1, 0, 0);
	}

	currSort = 0;
	currData = 0;

	// プリフェッチ
	for(u32 i=0;i < numBatchSorts;) {
		// UPDATE
		for(j=0;j < numPairs[currData];j++) {
			SortData& sort = sortBuffer[currSort][numBatch*batchFlag + j];
			//PRINTF("taskId %d key %5d pair %5d state %d(%d) %d(%d)\n",
			//	taskId,Key(sort),Pair(sort),
			//	StateA(sort),pairBuffer[currData][j].stateIndex[0],
			//	StateB(sort),pairBuffer[currData][j].stateIndex[1]);

			if(getFlag(sort) == 0) continue;

			//uint16_t pA = pairBuffer[currData][j].stateIndex[0];
			//uint16_t sA = StateA(sortBuffer[currSort][numBatch*batchFlag+j]);
			s32 j0 = j*2;
			s32 j1 = j*2+1;

			pairBuffer[currData][j].reset();

			closestContactPcl(pairBuffer[currData][j], stateBuffer[currData][j0].fX,stateBuffer[currData][j0].fRadius, stateBuffer[currData][j1].fX,stateBuffer[currData][j1].fRadius, CONTACT_EPSILON);
		}
		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
			spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		numPairs[currData] = 0;

		s32 start = (batchFlag + 2)%4*numBatch;
		s32 sortId = (batchFlag == 2 || batchFlag == 3) ? 1 - currSort : currSort;
		for(j=start,n=0;j < start + numBatch && j < numSorts[sortId];j++,n++) {
			SortData& sort = sortBuffer[sortId][j];
			//PRINTF("    get key %d A %d B %d pair %d\n",Key(sort),StateA(sort),StateB(sort),Pair(sort));
			s32 n0 = n*2;
			s32 n1 = n*2+1;
			pairDmaListBuf[currData][n].eal = io.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
			stateDmaListBuf[currData][n0].eal = io.pclStatesAddr + sizeof(PclState)*getStateA(sort);
			stateDmaListBuf[currData][n1].eal = io.pclStatesAddr + sizeof(PclState)*getStateB(sort);
		}
		numPairs[currData] = n;
		//PRINTF("numPairs %d\n",numPairs[currData]);

		if(numPairs[currData] > 0) {
			spu_dma_list_getf(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
			spu_dma_list_get(stateBuffer[currData], 0, stateDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]*2, tag1, 0, 0);
		}

		currData = 1 - currData;
		batchFlag = (batchFlag + 1)%4;

		if(batchFlag == 0) {
			//PRINTF("sortData dmaGetBuffer ls:0x%x ea:0x%x sz:%d\n",sortBuffer[currSort],startSortAddr+sizeof(SortData)*(i+4*numBatch),sortDataBatchSize);
			numSorts[currSort] = dmaGetBuffer(sortBuffer[currSort], startSortAddr + sizeof(SortData)*(i + 4*numBatch), endEA, sortDataBatchSize, tag1);
			numSorts[currSort] /= sizeof(SortData);
			//PRINTF("numSorts %d\n",numSorts[currSort]);
			currSort = 1 - currSort;
		}
	}

	if(numPairs[currData] > 0) {
		spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
	}

	spu_dma_wait_tag_status_all(1<<tag1);

	gPool.deallocate(stateDmaListBuf[1]);
	gPool.deallocate(stateDmaListBuf[0]);

	gPool.deallocate(pairDmaListBuf[2]);
	gPool.deallocate(pairDmaListBuf[1]);
	gPool.deallocate(pairDmaListBuf[0]);

	gPool.deallocate(stateBuffer[1]);
	gPool.deallocate(stateBuffer[0]);

	gPool.deallocate(pairBuffer[1]);
	gPool.deallocate(pairBuffer[0]);
	gPool.deallocate(sortBuffer[1]);
	gPool.deallocate(sortBuffer[0]);
}

void detectCollisionRig(u32 startSortAddr, u32 numBatchSorts, s32 batchUnit = 16)
{
	const s32 tag1 = 8;

	u32 numBatch = batchUnit;
	u32 numBatchx4 = numBatch*4;

	SortData *sortBuffer[2];
	PclContactPair *pairBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];

	PclState *pclStateBuffer[2];
	TrbState *rigStateBuffer[2];
	CollObject *collBuffer[2];
	spu_dma_list_element *pclStateDmaListBuf[2];
	spu_dma_list_element *rigStateDmaListBuf[2];
	spu_dma_list_element *collDmaListBuf[2];

	sortBuffer[0] = (SortData*)gPool.allocate(sizeof(SortData)*numBatchx4, HeapManager::ALIGN128);
	sortBuffer[1] = (SortData*)gPool.allocate(sizeof(SortData)*numBatchx4, HeapManager::ALIGN128);
	pairBuffer[0] = (PclContactPair*)gPool.allocate(sizeof(PclContactPair)*numBatch, HeapManager::ALIGN128);
	pairBuffer[1] = (PclContactPair*)gPool.allocate(sizeof(PclContactPair)*numBatch, HeapManager::ALIGN128);

	pclStateBuffer[0] = (PclState*)gPool.allocate(sizeof(PclState)*numBatch, HeapManager::ALIGN128);
	pclStateBuffer[1] = (PclState*)gPool.allocate(sizeof(PclState)*numBatch, HeapManager::ALIGN128);
	rigStateBuffer[0] = (TrbState*)gPool.allocate(sizeof(TrbState)*numBatch, HeapManager::ALIGN128);
	rigStateBuffer[1] = (TrbState*)gPool.allocate(sizeof(TrbState)*numBatch, HeapManager::ALIGN128);
	collBuffer[0] = (CollObject*)gPool.allocate(sizeof(CollObject)*numBatch, HeapManager::ALIGN128);
	collBuffer[1] = (CollObject*)gPool.allocate(sizeof(CollObject)*numBatch, HeapManager::ALIGN128);

	pairDmaListBuf[0] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	pairDmaListBuf[1] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	pairDmaListBuf[2] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);// PUT用

	pclStateDmaListBuf[0] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	pclStateDmaListBuf[1] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	rigStateDmaListBuf[0] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	rigStateDmaListBuf[1] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	collDmaListBuf[0] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);
	collDmaListBuf[1] = (spu_dma_list_element*)gPool.allocate(sizeof(spu_dma_list_element)*numBatch, HeapManager::ALIGN128);

	memset(pairDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[2], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pclStateDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pclStateDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(rigStateDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(rigStateDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(collDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(collDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatch);

	//PRINTF("SPU : task %d allocated %d bytes\n",taskId,gPool.getAllocated());

	//uint32_t startSortAddr = io.sortsAddr + sizeof(SortData) * io.startBatch;

	u32 endEA = startSortAddr + sizeof(SortData)*numBatchSorts;
	s32 sortDataBatchSize = sizeof(SortData)*numBatch * 4;
	s32 batchFlag = 0;
	s32 currSort = 0;
	s32 currData = 0;
	u32 numPairs[2] = {0};
	u32 numSorts[2] = {0};
	u32 j,n;

	for(j=0,n=0;j < numBatch;j++,n++) {
		pairDmaListBuf[0][n].size = sizeof(PclContactPair);
		pairDmaListBuf[1][n].size = sizeof(PclContactPair);
		pclStateDmaListBuf[0][n].size = sizeof(PclState);
		pclStateDmaListBuf[1][n].size = sizeof(PclState);
		rigStateDmaListBuf[0][n].size = sizeof(TrbState);
		rigStateDmaListBuf[1][n].size = sizeof(TrbState);
		collDmaListBuf[0][n].size = sizeof(CollObject);
		collDmaListBuf[1][n].size = sizeof(CollObject);
	}

	// 初期状態
	//PRINTF("sortData dmaGetBuffer ls:0x%x ea:0x%x sz:%d\n",sortBuffer[0],io.sortsAddr,sortDataBatchSize);
	numSorts[0] = dmaGetBuffer(sortBuffer[0], startSortAddr, endEA, sortDataBatchSize, tag1);
	numSorts[0] /= sizeof(SortData);
	//PRINTF("numSorts %d\n",numSorts[0]);

	spu_dma_wait_tag_status_all(1<<tag1);

	for(j=0,n=0;j < numBatch && j < numSorts[0];j++,n++) {
		SortData& sort = sortBuffer[0][j];
		//PRINTF("    get key %d A %d B %d pair %d\n",Key(sort),StateA(sort),StateB(sort),Pair(sort));
		pairDmaListBuf[0][n].eal = io.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
		pclStateDmaListBuf[0][n].eal = io.pclStatesAddr + sizeof(PclState)*getStateA(sort);
		rigStateDmaListBuf[0][n].eal = io.rigStatesAddr + sizeof(TrbState)*getStateB(sort);
		collDmaListBuf[0][n].eal = io.collsAddr + sizeof(CollObject)*getBodyB(sort);
	}
	numPairs[0] = n;
	//PRINTF("numPairs %d %d\n",numPairs[0],n);

	if(numPairs[0] > 0) {
		spu_dma_list_get(pairBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
		spu_dma_list_get(pclStateBuffer[0], 0, pclStateDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
		spu_dma_list_get(rigStateBuffer[0], 0, rigStateDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
		spu_dma_list_get(collBuffer[0], 0, collDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
	}

	spu_dma_wait_tag_status_all(1<<tag1);

	//PRINTF("sortData dmaGetBuffer ls:0x%x ea:0x%x sz:%d\n",sortBuffer[1],io.sortsAddr+sortDataBatchSize,sortDataBatchSize);
	numSorts[1] = dmaGetBuffer(sortBuffer[1], startSortAddr + sortDataBatchSize, endEA, sortDataBatchSize, tag1);
	numSorts[1] /= sizeof(SortData);
	//PRINTF("numSorts %d\n",numSorts[1]);

	for(j=numBatch,n=0;j < numBatch + numBatch && j < numSorts[0];j++,n++) {
		SortData& sort = sortBuffer[0][j];
		//PRINTF("    taskId %d sort(%2d/%2d) key %7d pair %5d state %4d %4d body %3d %3d\n",taskId,n,numBatch,Key(sort),Pair(sort),StateA(sort),StateB(sort),BodyA(sort),BodyB(sort));
		pairDmaListBuf[1][n].eal = io.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
		pclStateDmaListBuf[1][n].eal = io.pclStatesAddr + sizeof(PclState)*getStateA(sort);
		rigStateDmaListBuf[1][n].eal = io.rigStatesAddr + sizeof(TrbState)*getStateB(sort);
		collDmaListBuf[1][n].eal = io.collsAddr + sizeof(CollObject)*getBodyB(sort);
	}
	numPairs[1] = n;
	//PRINTF("numPairs %d\n",numPairs[1]);

	if(numPairs[1] > 0) {
		spu_dma_list_get(pairBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
		spu_dma_list_get(pclStateBuffer[1], 0, pclStateDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
		spu_dma_list_get(rigStateBuffer[1], 0, rigStateDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
		spu_dma_list_get(collBuffer[1], 0, collDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
	}

	currSort = 0;
	currData = 0;

	for(u32 i=0;i < numBatchSorts;) {
		// UPDATE
		for(j=0;j < numPairs[currData];j++) {
			SortData& sort = sortBuffer[currSort][numBatch*batchFlag + j];

			if(getFlag(sort) == 0) continue;

			//uint16_t pA = pairBuffer[currData][j].stateIndex[0];
			//uint16_t sA = StateA(sortBuffer[currSort][numBatch*batchFlag+j]);

			Transform3 tB(rigStateBuffer[currData][j].getOrientation(), rigStateBuffer[currData][j].getPosition());

			pairBuffer[currData][j].reset();

			closestContactRig(pairBuffer[currData][j], pclStateBuffer[currData][j].fX, pclStateBuffer[currData][j].fRadius, collBuffer[currData][j], tB, CONTACT_EPSILON);
		}
		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
			spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		numPairs[currData] = 0;

		s32 start = (batchFlag+2)%4*numBatch;
		int sortId = (batchFlag == 2 || batchFlag == 3) ? 1 - currSort : currSort;
		for(j=start,n=0;j < start + numBatch && j < numSorts[sortId];j++,n++) {
			SortData& sort = sortBuffer[sortId][j];
			//PRINTF("    get key %d A %d B %d pair %d\n",Key(sort),StateA(sort),StateB(sort),Pair(sort));
			pairDmaListBuf[currData][n].eal = io.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
			pclStateDmaListBuf[currData][n].eal = io.pclStatesAddr + sizeof(PclState)*getStateA(sort);
			rigStateDmaListBuf[currData][n].eal = io.rigStatesAddr + sizeof(TrbState)*getStateB(sort);
			collDmaListBuf[currData][n].eal = io.collsAddr + sizeof(CollObject)*getBodyB(sort);
		}
		numPairs[currData] = n;
		//PRINTF("numPairs %d\n",numPairs[currData]);

		if(numPairs[currData] > 0) {
			spu_dma_list_getf(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
			spu_dma_list_get(pclStateBuffer[currData], 0, pclStateDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
			spu_dma_list_get(rigStateBuffer[currData], 0, rigStateDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
			spu_dma_list_get(collBuffer[currData], 0, collDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		currData = 1 - currData;
		batchFlag = (batchFlag + 1)%4;

		if(batchFlag == 0) {
			//PRINTF("sortData dmaGetBuffer ls:0x%x ea:0x%x sz:%d\n",sortBuffer[currSort],startSortAddr+sizeof(SortData)*(i+4*numBatch),sortDataBatchSize);
			numSorts[currSort] = dmaGetBuffer(sortBuffer[currSort], startSortAddr + sizeof(SortData)*(i + 4*numBatch), endEA, sortDataBatchSize, tag1);
			numSorts[currSort] /= sizeof(SortData);
			//PRINTF("numSorts %d\n",numSorts[currSort]);
			currSort = 1 - currSort;
		}
	}

	if(numPairs[currData] > 0)
		spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

	spu_dma_wait_tag_status_all(1<<tag1);

	gPool.deallocate(collDmaListBuf[1]);
	gPool.deallocate(collDmaListBuf[0]);
	gPool.deallocate(rigStateDmaListBuf[1]);
	gPool.deallocate(rigStateDmaListBuf[0]);
	gPool.deallocate(pclStateDmaListBuf[1]);
	gPool.deallocate(pclStateDmaListBuf[0]);

	gPool.deallocate(pairDmaListBuf[2]);
	gPool.deallocate(pairDmaListBuf[1]);
	gPool.deallocate(pairDmaListBuf[0]);

	gPool.deallocate(collBuffer[1]);
	gPool.deallocate(collBuffer[0]);
	gPool.deallocate(rigStateBuffer[1]);
	gPool.deallocate(rigStateBuffer[0]);
	gPool.deallocate(pclStateBuffer[1]);
	gPool.deallocate(pclStateBuffer[0]);

	gPool.deallocate(pairBuffer[1]);
	gPool.deallocate(pairBuffer[0]);
	gPool.deallocate(sortBuffer[1]);
	gPool.deallocate(sortBuffer[0]);
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

	bool empty = false;
	u32 addrIo = taskbuff[0];
	u32 function = taskbuff[1];
	u32 commonBuffAddr = taskbuff[2];
	u32 *commonBuff = (u32*)gPool.allocate(sizeof(u32)*32, HeapManager::ALIGN128);

	spu_dma_get(&io, addrIo, sizeof(IOParamCollisionPcl), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	while(!empty) {
		u32 startBatch;
		u32 numBatch;

		lock(commonBuff,commonBuffAddr);

		startBatch = commonBuff[1];
		numBatch = commonBuff[2];

		s32 nextStartBatch = startBatch + numBatch;
		s32 rest = MAX((s32)io.numContactPairs - nextStartBatch, 0);
		s32 nextNumBatch = (rest > (s32)numBatch) ? (s32)numBatch : rest;

		commonBuff[1] = nextStartBatch;
		commonBuff[2] = nextNumBatch;

		unlock(commonBuff,commonBuffAddr);

		if(numBatch > 0) {
			if(function == PARTICLE_DETECTCOLLISIONS_PCL)
				detectCollisionPcl(io.sortsAddr + sizeof(SortData)*startBatch, numBatch);
			else
				detectCollisionRig(io.sortsAddr + sizeof(SortData)*startBatch, numBatch);
		} else
			empty = true;
	}

	gPool.deallocate(commonBuff);

	spu_dma_put(&io, addrIo, sizeof(IOParamCollisionPcl), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}


