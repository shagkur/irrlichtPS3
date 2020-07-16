/*
 * main.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/rigidbodyio.h"
#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/contact.h"
#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/joint.h"
#include "rigidbody/common/forces.h"
#include "rigidbody/common/closestcontact.h"

#include "base/heapmanager.h"
#include "base/safedma.h"

#define STATIC_MEM					0
#define DYNAMIC_MEM					(81*1024)
#define HEAP_BYTES 					(STATIC_MEM + DYNAMIC_MEM)

ATTRIBUTE_ALIGNED128(u32 lockBuffer[32]);

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool, HEAP_BYTES);

#define ALLOCATE(align, size) 		gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16)
#define DEALLOCATE(ptr) 			if(ptr) {gPool.deallocate((void*)ptr);ptr=NULL;}

IOParamCollision io;

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

void detectClosestContact(ContactPair& pair, TrbState& stateA, CollObject& colA, TrbState& stateB, CollObject& colB)
{
	Transform3 tA0(stateA.getOrientation(), stateA.getPosition());
	Transform3 tB0(stateB.getOrientation(), stateB.getPosition());

	bool ret = false;

	ContactPair *newContact = (ContactPair*)ALLOCATE(128, sizeof(ContactPair));

	newContact->reset();
	newContact->stateIndex[0] = pair.stateIndex[0];
	newContact->stateIndex[1] = pair.stateIndex[1];

	if(UNLIKELY(io.ccdEnable)) {
		Transform3 tA1 = integrateTransform(io.timeStep, tA0, stateA.getLinearVelocity(), stateA.getAngularVelocity());
		Transform3 tB1 = integrateTransform(io.timeStep, tB0, stateB.getLinearVelocity(), stateB.getAngularVelocity());
		ret = findContactCCD(*newContact, colA, tA0, tA1, stateA.getUseCcd(), colB, tB0, tB1, stateB.getUseCcd(), CONTACT_EPSILON);
	} else
		ret = closestContact(*newContact, colA, tA0, colB, tB0, CONTACT_EPSILON);

	if(ret)
		pair.merge(*newContact);

	DEALLOCATE(newContact);
}

void detectCollision(u32 startSortAddr, u32 numBatchSorts, s32 batchUnit = 16)
{
	const s32 tag1 = 8;

	u32 numBatch = batchUnit;
	u32 numBatchx2 = numBatch*2;
	u32 numBatchx4 = numBatch*4;

	SortData *sortBuffer[2];
	ContactPair *pairBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];

	TrbState *stateBuffer[2];
	CollObject *collBuffer[2];
	spu_dma_list_element *stateDmaListBuf[2];
	spu_dma_list_element *collDmaListBuf[2];

	sortBuffer[0] = (SortData*)ALLOCATE(128, sizeof(SortData)*numBatchx4);
	sortBuffer[1] = (SortData*)ALLOCATE(128, sizeof(SortData)*numBatchx4);
	pairBuffer[0] = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numBatch);
	pairBuffer[1] = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numBatch);

	stateBuffer[0] = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numBatchx2);
	stateBuffer[1] = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numBatchx2);
	collBuffer[0] = (CollObject*)ALLOCATE(128, sizeof(CollObject)*numBatchx2);
	collBuffer[1] = (CollObject*)ALLOCATE(128, sizeof(CollObject)*numBatchx2);

	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);

	stateDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatchx2);
	stateDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatchx2);
	collDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatchx2);
	collDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatchx2);

	memset(pairDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[2], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(stateDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatchx2);
	memset(stateDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatchx2);
	memset(collDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatchx2);
	memset(collDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatchx2);

	u32 endEA = startSortAddr + sizeof(SortData)*numBatchSorts;

	s32 sortDataBatchSize = sizeof(SortData)*numBatch*4;
	s32 batchFlag = 0;
	s32 currSort = 0;
	s32 currData = 0;
	u32 numPairs[2] = {0};
	u32 numSorts[2] = {0};
	u32 j,n;

	for(j=0, n=0;j < numBatch;j++, n++) {
		pairDmaListBuf[0][n].size = sizeof(ContactPair);
		pairDmaListBuf[1][n].size = sizeof(ContactPair);
		stateDmaListBuf[0][n*2 + 0].size = sizeof(TrbState);
		stateDmaListBuf[0][n*2 + 1].size = sizeof(TrbState);
		stateDmaListBuf[1][n*2 + 0].size = sizeof(TrbState);
		stateDmaListBuf[1][n*2 + 1].size = sizeof(TrbState);
		collDmaListBuf[0][n*2 + 0].size = sizeof(CollObject);
		collDmaListBuf[0][n*2 + 1].size = sizeof(CollObject);
		collDmaListBuf[1][n*2 + 0].size = sizeof(CollObject);
		collDmaListBuf[1][n*2 + 1].size = sizeof(CollObject);
	}

	numSorts[0] = dmaGetBuffer(sortBuffer[0], startSortAddr, endEA, sortDataBatchSize, tag1);
	numSorts[0] /= sizeof(SortData);

	spu_dma_wait_tag_status_all(1<<tag1);

	for(j=0, n=0;j < numBatch && j < numSorts[0];j++, n++) {
		SortData &sort = sortBuffer[0][j];

		s32 n0 = n*2;
		s32 n1 = n*2 + 1;
		pairDmaListBuf[0][n].eal = io.contactPairsAddr + sizeof(ContactPair)*getPair(sort);
		stateDmaListBuf[0][n0].eal = io.statesAddr + sizeof(TrbState)*getStateA(sort);
		stateDmaListBuf[0][n1].eal = io.statesAddr + sizeof(TrbState)*getStateB(sort);
		collDmaListBuf[0][n0].eal = io.collsAddr + sizeof(CollObject)*getBodyA(sort);
		collDmaListBuf[0][n1].eal = io.collsAddr + sizeof(CollObject)*getBodyB(sort);
	}
	numPairs[0] = n;

	spu_dma_list_get(pairBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
	spu_dma_list_get(stateBuffer[0], 0, stateDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0]*2, tag1, 0, 0);
	spu_dma_list_get(collBuffer[0], 0, collDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0]*2, tag1, 0, 0);
	spu_dma_wait_tag_status_all(1<<tag1);

	numSorts[1] = dmaGetBuffer(sortBuffer[1], startSortAddr + sortDataBatchSize, endEA, sortDataBatchSize, tag1);
	numSorts[1] /= sizeof(SortData);

	for(j=numBatch, n=0;j < numBatch+numBatch && j < numSorts[0];j++, n++) {
		SortData &sort = sortBuffer[0][j];

		s32 n0 = n*2;
		s32 n1 = n*2 + 1;
		pairDmaListBuf[1][n].eal = io.contactPairsAddr + sizeof(ContactPair)*getPair(sort);
		stateDmaListBuf[1][n0].eal = io.statesAddr + sizeof(TrbState)*getStateA(sort);
		stateDmaListBuf[1][n1].eal = io.statesAddr + sizeof(TrbState)*getStateB(sort);
		collDmaListBuf[1][n0].eal = io.collsAddr + sizeof(CollObject)*getBodyA(sort);
		collDmaListBuf[1][n1].eal = io.collsAddr + sizeof(CollObject)*getBodyB(sort);
	}
	numPairs[1] = n;

	spu_dma_list_get(pairBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
	spu_dma_list_get(stateBuffer[1], 0, stateDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1]*2, tag1, 0, 0);
	spu_dma_list_get(collBuffer[1], 0, collDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1]*2, tag1, 0, 0);

	currSort = 0;
	currData = 0;

	for(u32 i=0;i < numBatchSorts;) {
		// UPDATE
		for(j=0;j < numPairs[currData];j++) {

			if(getFlag(sortBuffer[currSort][numBatch*batchFlag + j]) == 0) continue;

			u16 pA = pairBuffer[currData][j].stateIndex[0];
			u16 sA = getStateA(sortBuffer[currSort][numBatch*batchFlag + j]);
			s32 j0 = (pA == sA) ? j*2 : j*2 + 1;
			s32 j1 = (pA == sA) ? j*2 + 1: j*2;

			detectClosestContact(pairBuffer[currData][j], stateBuffer[currData][j0], collBuffer[currData][j0], stateBuffer[currData][j1],collBuffer[currData][j1]);
		}
		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
		spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

		numPairs[currData] = 0;

		s32 start = (batchFlag + 2)%4*numBatch;
		s32 sortId = (batchFlag == 2 || batchFlag == 3) ? 1 - currSort : currSort;
		for(j=start, n=0;j < start + numBatch && j < numSorts[sortId];j++, n++) {
			SortData &sort = sortBuffer[sortId][j];

			s32 n0 = n*2;
			s32 n1 = n*2 + 1;
			pairDmaListBuf[currData][n].eal = io.contactPairsAddr + sizeof(ContactPair)*getPair(sort);
			stateDmaListBuf[currData][n0].eal = io.statesAddr + sizeof(TrbState)*getStateA(sort);
			stateDmaListBuf[currData][n1].eal = io.statesAddr + sizeof(TrbState)*getStateB(sort);
			collDmaListBuf[currData][n0].eal = io.collsAddr + sizeof(CollObject)*getBodyA(sort);
			collDmaListBuf[currData][n1].eal = io.collsAddr + sizeof(CollObject)*getBodyB(sort);
		}
		numPairs[currData] = n;

		spu_dma_list_getf(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		spu_dma_list_get(stateBuffer[currData], 0, stateDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]*2, tag1, 0, 0);
		spu_dma_list_get(collBuffer[currData], 0, collDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]*2, tag1, 0, 0);

		currData = 1-currData;

		batchFlag = (batchFlag + 1)%4;

		if(batchFlag == 0) {
			numSorts[currSort] = dmaGetBuffer(sortBuffer[currSort], startSortAddr + sizeof(SortData)*(i + 4*numBatch), endEA, sortDataBatchSize, tag1);
			numSorts[currSort] /= sizeof(SortData);
			currSort = 1 - currSort;
		}
	}

	spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
	spu_dma_wait_tag_status_all(1<<tag1);

	DEALLOCATE(collDmaListBuf[1]);
	DEALLOCATE(collDmaListBuf[0]);
	DEALLOCATE(stateDmaListBuf[1]);
	DEALLOCATE(stateDmaListBuf[0]);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);

	DEALLOCATE(collBuffer[1]);
	DEALLOCATE(collBuffer[0]);
	DEALLOCATE(stateBuffer[1]);
	DEALLOCATE(stateBuffer[0]);

	DEALLOCATE(pairBuffer[1]);
	DEALLOCATE(pairBuffer[0]);
	DEALLOCATE(sortBuffer[1]);
	DEALLOCATE(sortBuffer[0]);
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
	u32 commonBuffAddr = taskbuff[2];

	spu_dma_get(&io, addrIo, sizeof(IOParamCollision), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);

	bool empty = false;
	while(!empty) {
		u32 startBatch;
		u32 numBatch;

		lock(commonBuff, commonBuffAddr);

		startBatch = commonBuff[1];
		numBatch = commonBuff[2];

		s32 nextStartBatch = startBatch + numBatch;
		s32 rest = MAX((s32)io.numContactPairs - nextStartBatch, 0);
		s32 nextNumBatch = (rest > (s32)numBatch) ? (s32)numBatch : rest;

		commonBuff[1] = nextStartBatch;
		commonBuff[2] = nextNumBatch;

		unlock(commonBuff, commonBuffAddr);

		if(numBatch > 0)
			detectCollision(io.sortsAddr + sizeof(SortData)*startBatch, numBatch);
		else
			empty = true;
	}

	DEALLOCATE(commonBuff);

	// put io
	spu_dma_put(&io, addrIo, sizeof(IOParamCollision), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}
