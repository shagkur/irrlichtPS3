/*
 * main.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/rigidbodyio.h"
#include "rigidbody/common/forces.h"
#include "rigidbody/common/parallelgroup.h"

#include "base/heapmanager.h"
#include "base/prefetchiterator.h"

#include "rigidbody/common/contactsolver.h"
#include "rigidbody/common/jointsolver.h"
#include "rigidbody/common/solverfunctable.h"

#define STATIC_MEM				0
#define DYNAMIC_MEM				(100*1024)
#define HEAP_BYTES (STATIC_MEM+DYNAMIC_MEM)
ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool, HEAP_BYTES);

#define ALLOCATE(align, size) 	gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16)
#define DEALLOCATE(ptr) 		if(ptr) {gPool.deallocate((void*)ptr);ptr=NULL;}

#define PREFETCH_NUM			64

#define ENABLE_PREFETCH_SOLVER

ATTRIBUTE_ALIGNED16(IOParamSolver solverIO);
ATTRIBUTE_ALIGNED16(IOParamPostResponse postResponseIO);
ATTRIBUTE_ALIGNED16(IOParamSplitConstraints splitConstraintsIO);
ATTRIBUTE_ALIGNED16(IOParamCreateJointPairs createJointPairsIO);

ATTRIBUTE_ALIGNED16(SolverInfo *contactSolverInfo);
ATTRIBUTE_ALIGNED16(SolverInfo *jointSolverInfo);

u32 taskId;

u32 barrier = 0;

ATTRIBUTE_ALIGNED128(static u32 atomicBuf[32]);

void sync()
{
	while(mars_task_barrier_try_notify(barrier) != MARS_SUCCESS) {}
	while(mars_task_barrier_try_wait(barrier) != MARS_SUCCESS) {}
}

static inline void setListDma(spu_dma_list_element *dmaListState, u32 numList, u32 size)
{
	u32 j;
	vec_uint4 vec = ((vec_uint4){size, 0, size, 0});
	u64 elem64 = (((u64)size)<<32);
	for(j=0;j + 8 <= numList;j+=8) {
		*((vec_uint4*)(dmaListState + j + 0)) = vec;
		*((vec_uint4*)(dmaListState + j + 2)) = vec;
		*((vec_uint4*)(dmaListState + j + 4)) = vec;
		*((vec_uint4*)(dmaListState + j + 6)) = vec;
	}
	for(;j < numList;j++)
		*((u64*)(dmaListState) + j) = elem64;
}

static inline void setListDmaDual(spu_dma_list_element *dmaListState0, spu_dma_list_element *dmaListState1, u32 numList, u32 size)
{
	u32 j;
	vec_uint4 vec = ((vec_uint4){size, 0, size, 0});
	u64 elem64 = (((u64)size)<<32);
	for(j=0;j + 8 <= numList;j+=8) {
		*((vec_uint4*)(dmaListState0 + j + 0)) = vec;
		*((vec_uint4*)(dmaListState1 + j + 0)) = vec;
		*((vec_uint4*)(dmaListState0 + j + 2)) = vec;
		*((vec_uint4*)(dmaListState1 + j + 2)) = vec;
		*((vec_uint4*)(dmaListState0 + j + 4)) = vec;
		*((vec_uint4*)(dmaListState1 + j + 4)) = vec;
		*((vec_uint4*)(dmaListState0 + j + 6)) = vec;
		*((vec_uint4*)(dmaListState1 + j + 6)) = vec;
	}
	for(;j < numList;j++){
		*((u64*)(dmaListState0) + j) = elem64;
		*((u64*)(dmaListState1) + j) = elem64;
	}
}

static inline void startStructDmaList(u32 length, void *localBuffer, u32 tag, spu_dma_list_element *dmaListBuf, SortData *sorts, u32 dmaBytes, u32 baseAddr, u32 strideBytes)
{
	u32 j;
	for(j=0;j + 8 <= length;j+=8){
		*((vec_uint4*)(dmaListBuf + j + 0)) = ((vec_uint4){dmaBytes, baseAddr + strideBytes*getPair(sorts[j + 0]), dmaBytes, baseAddr + strideBytes*getPair(sorts[j + 1])});
		*((vec_uint4*)(dmaListBuf + j + 2)) = ((vec_uint4){dmaBytes, baseAddr + strideBytes*getPair(sorts[j + 2]), dmaBytes, baseAddr + strideBytes*getPair(sorts[j + 3])});
		*((vec_uint4*)(dmaListBuf + j + 4)) = ((vec_uint4){dmaBytes, baseAddr + strideBytes*getPair(sorts[j + 4]), dmaBytes, baseAddr + strideBytes*getPair(sorts[j + 5])});
		*((vec_uint4*)(dmaListBuf + j + 6)) = ((vec_uint4){dmaBytes, baseAddr + strideBytes*getPair(sorts[j + 6]), dmaBytes, baseAddr + strideBytes*getPair(sorts[j + 7])});
		spu_dma_list_get((void*)((uintptr_t)localBuffer + dmaBytes*j), 0, dmaListBuf + j, sizeof(spu_dma_list_element)*8, tag, 0, 0);
	}
	for(;j<length;j++){
		*((u64*)(dmaListBuf + j)) = (((u64)dmaBytes)<<32) + (baseAddr + strideBytes*getPair(sorts[j]));
		spu_dma_list_get((void*)((uintptr_t)localBuffer + dmaBytes*j), 0, dmaListBuf + j, sizeof(spu_dma_list_element), tag, 0, 0);
	}
}

static inline void startStateDmaListGet(u32 length, TrbState *states, u32 tag, spu_dma_list_element *dmaListState, SortData *sorts, u8 *stateRefTable, u32 *statesEA, u32& numStates, s32& numBatchStates)
{
	for(u32 j=0;j < length;j++) {
		u32 stateA = getStateA(sorts[j]);
		u32 stateB = getStateB(sorts[j]);

		u32 candEAA = 0;
		u32 candEAB = 0;

		u32 sizeA = 0;
		u32 sizeB = 0;

		if(stateRefTable[stateA] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateA;
			candEAA = statesEA[numStates];
			sizeA = sizeof(TrbState);
			stateRefTable[stateA] = numStates;
			numBatchStates++;
			numStates++;
		}

		if(stateRefTable[stateB] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateB;
			candEAB= statesEA[numStates];
			sizeB = sizeof(TrbState);
			stateRefTable[stateB] = numStates;
			numBatchStates++;
			numStates++;
		}

		*((vec_uint4*)(dmaListState) + j) = ((vec_uint4){sizeA, candEAA, sizeB, candEAB});
	}
	spu_dma_list_get(states, 0, dmaListState, sizeof(spu_dma_list_element)*2*length, tag, 0, 0);
}

static inline void startStateDmaListPutImmediate(u32 numStates, TrbState *states, u32 tag, u32 *statesEA)
{
	spu_dma_list_element *dmaList = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*(numStates+1)/2*2);

	if(numStates > 0) {
		u32 i;
		for(i=0;i + 8 <= numStates;i+=8){
			*((vec_uint4*)(dmaList + i + 0)) = ((vec_uint4){sizeof(TrbState), statesEA[i + 0], sizeof(TrbState), statesEA[i + 1]});
			*((vec_uint4*)(dmaList + i + 2)) = ((vec_uint4){sizeof(TrbState), statesEA[i + 2], sizeof(TrbState), statesEA[i + 3]});
			*((vec_uint4*)(dmaList + i + 4)) = ((vec_uint4){sizeof(TrbState), statesEA[i + 4], sizeof(TrbState), statesEA[i + 5]});
			*((vec_uint4*)(dmaList + i + 6)) = ((vec_uint4){sizeof(TrbState), statesEA[i + 6], sizeof(TrbState), statesEA[i + 7]});
			spu_dma_list_put(states + i, 0, dmaList + i, sizeof(spu_dma_list_element)*8, tag, 0, 0);
		}
		for(;i + 2 <= numStates;i+=2){
			*((vec_uint4*)(dmaList+i)) = ((vec_uint4){sizeof(TrbState), statesEA[i], sizeof(TrbState), statesEA[i + 1]});
			spu_dma_list_put(states + i, 0, dmaList + i, sizeof(spu_dma_list_element)*2, tag, 0, 0);
		}
		if(i < numStates){
			*((vec_uint4*)(dmaList + i)) = ((vec_uint4){sizeof(TrbState), statesEA[i], 0, 0});
			spu_dma_list_put(states + i, 0, dmaList + i, sizeof(spu_dma_list_element), tag, 0, 0);
		}
	}
	spu_dma_wait_tag_status_all(1<<tag);
	DEALLOCATE(dmaList);
}

#ifdef ENABLE_PREFETCH_SOLVER

void preContactImpulseEx(SortData *sorts, u32 numSorts, const ContactSolverConfig& config)
{
	s32 tag1 = 11;

	TrbState *states = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numSorts*2);
	u32 *statesEA = (u32*)ALLOCATE(16, sizeof(u32)*numSorts*2);
	TrbDynBody *bodies = (TrbDynBody*)ALLOCATE(128, sizeof(TrbDynBody)*numSorts*2);

	u32 numBatch = 8;

	u8 numStates = 0;
	u8 numBodies = 0;

	u8 *stateRefTable  = (u8*)ALLOCATE(16, sizeof(u8)*solverIO.numStates);
	u8 *bodyRefTable  = (u8*)ALLOCATE(16, sizeof(u8)*solverIO.numBodies);

	memset(stateRefTable, 0xff, sizeof(u8)*solverIO.numStates);
	memset(bodyRefTable, 0xff, sizeof(u8)*solverIO.numBodies);

	spu_dma_list_element *dmaListState[2];
	spu_dma_list_element *dmaListBody[2];

	s32 numBatchStates[2] = {0};
	s32 numBatchBodies[2] = {0};

	dmaListState[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	dmaListState[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	dmaListBody[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	dmaListBody[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);

	setListDmaDual(dmaListState[0], dmaListState[1], numBatch*2, sizeof(TrbState));
	setListDmaDual(dmaListBody[0], dmaListBody[1], numBatch*2, sizeof(TrbDynBody));

	u32 contactAddr = solverIO.contactsAddr;

	ContactPair *contactBuffer[2];
	spu_dma_list_element *contactDmaListBuf[3];

	contactBuffer[0] = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numBatch);
	contactBuffer[1] = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numBatch);
	contactDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	contactDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	contactDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);// PUT用

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numContacts[2] = {0};

	TrbState *nextStatesPtr = states;
	TrbDynBody *nextBodiesPtr = bodies;

	for(u32 j=0;j < numBatch;j++) {
		contactDmaListBuf[0][j].notify = 0;
		contactDmaListBuf[0][j].size = sizeof(ContactPair);
		contactDmaListBuf[1][j].notify = 0;
		contactDmaListBuf[1][j].size = sizeof(ContactPair);
	}

	for(u32 j=0;j < numBatch && j < numSorts;j++) {
		contactDmaListBuf[0][numContacts[0]].eal = contactAddr + sizeof(ContactPair)*getPair(sorts[j]);
		numContacts[0]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);
		u16 bodyA = getBodyA(sorts[j]);
		u16 bodyB = getBodyB(sorts[j]);

		// State
		if(stateRefTable[stateA] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateA;
			dmaListState[0][numBatchStates[0]].eal = statesEA[numStates];
			stateRefTable[stateA] = numStates;
			numBatchStates[0]++;
			numStates++;
		}

		if(stateRefTable[stateB] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateB;
			dmaListState[0][numBatchStates[0]].eal = statesEA[numStates];
			stateRefTable[stateB] = numStates;
			numBatchStates[0]++;
			numStates++;
		}

		// Body
		if(bodyRefTable[bodyA] == 0xff) {
			dmaListBody[0][numBatchBodies[0]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyA;
			bodyRefTable[bodyA] = numBodies;
			numBatchBodies[0]++;
			numBodies++;
		}

		if(bodyRefTable[bodyB] == 0xff) {
			dmaListBody[0][numBatchBodies[0]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyB;
			bodyRefTable[bodyB] = numBodies;
			numBatchBodies[0]++;
			numBodies++;
		}
	}

	spu_dma_list_get(contactBuffer[0], 0, contactDmaListBuf[0], sizeof(spu_dma_list_element)*numContacts[0], tag1, 0, 0);
	spu_dma_list_get(states, 0, dmaListState[0], sizeof(spu_dma_list_element)*numBatchStates[0], tag1, 0, 0);
	spu_dma_list_get(bodies, 0, dmaListBody[0], sizeof(spu_dma_list_element)*numBatchBodies[0], tag1, 0, 0);

	nextStatesPtr += numBatchStates[0];
	nextBodiesPtr += numBatchBodies[0];

	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 j=numBatch;j < numBatch + numBatch && j < numSorts;j++) {
		contactDmaListBuf[1][numContacts[1]].eal = contactAddr + sizeof(ContactPair)*getPair(sorts[j]);
		numContacts[1]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);
		u16 bodyA = getBodyA(sorts[j]);
		u16 bodyB = getBodyB(sorts[j]);

		// State
		if(stateRefTable[stateA] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateA;
			dmaListState[1][numBatchStates[1]].eal = statesEA[numStates];
			stateRefTable[stateA] = numStates;
			numBatchStates[1]++;
			numStates++;
		}

		if(stateRefTable[stateB] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateB;
			dmaListState[1][numBatchStates[1]].eal = statesEA[numStates];
			stateRefTable[stateB] = numStates;
			numBatchStates[1]++;
			numStates++;
		}

		// Body
		if(bodyRefTable[bodyA] == 0xff) {
			dmaListBody[1][numBatchBodies[1]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyA;
			bodyRefTable[bodyA] = numBodies;
			numBatchBodies[1]++;
			numBodies++;
		}

		if(bodyRefTable[bodyB] == 0xff) {
			dmaListBody[1][numBatchBodies[1]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyB;
			bodyRefTable[bodyB] = numBodies;
			numBatchBodies[1]++;
			numBodies++;
		}
	}

	spu_dma_list_get(contactBuffer[1], 0, contactDmaListBuf[1], sizeof(spu_dma_list_element)*numContacts[1], tag1, 0, 0);
	spu_dma_list_get(nextStatesPtr, 0, dmaListState[1], sizeof(spu_dma_list_element)*numBatchStates[1], tag1, 0, 0);
	spu_dma_list_get(nextBodiesPtr, 0, dmaListBody[1], sizeof(spu_dma_list_element)*numBatchBodies[1], tag1, 0, 0);

	nextStatesPtr += numBatchStates[1];
	nextBodiesPtr += numBatchBodies[1];

	currData = 0;
	batchFlag = 1;

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		for(int j=0;j < numContacts[currData];j++) {
			ContactPair& contact = contactBuffer[currData][j];
			TrbState& stateA = states[stateRefTable[contact.stateIndex[0]]];
			TrbState& stateB = states[stateRefTable[contact.stateIndex[1]]];
			TrbDynBody& bodyA = bodies[bodyRefTable[stateA.trbBodyIdx]];
			TrbDynBody& bodyB = bodies[bodyRefTable[stateB.trbBodyIdx]];

			funcTbl_preResponse[stateA.moveType + stateA.sleeping][stateB.moveType + stateB.sleeping](contact, stateA, bodyA, stateB, bodyB, config);
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		memcpy(contactDmaListBuf[2], contactDmaListBuf[currData], sizeof(spu_dma_list_element)*numContacts[currData]);
		spu_dma_list_put(contactBuffer[currData], 0, contactDmaListBuf[2], sizeof(spu_dma_list_element)*numContacts[currData], tag1, 0, 0);

		numContacts[currData] = 0;
		numBatchStates[currData] = 0;
		numBatchBodies[currData] = 0;

		batchFlag++;
		s32 start = batchFlag*numBatch;

		for(u32 j=start;j < start + numBatch && j < numSorts;j++) {
			contactDmaListBuf[currData][numContacts[currData]].eal = contactAddr + sizeof(ContactPair)*getPair(sorts[j]);
			numContacts[currData]++;

			u16 stateA = getStateA(sorts[j]);
			u16 stateB = getStateB(sorts[j]);
			u16 bodyA = getBodyA(sorts[j]);
			u16 bodyB = getBodyB(sorts[j]);

			// State
			if(stateRefTable[stateA] == 0xff) {
				statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateA;
				dmaListState[currData][numBatchStates[currData]].eal = statesEA[numStates];
				stateRefTable[stateA] = numStates;
				numBatchStates[currData]++;
				numStates++;
			}

			if(stateRefTable[stateB] == 0xff) {
				statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateB;
				dmaListState[currData][numBatchStates[currData]].eal = statesEA[numStates];
				stateRefTable[stateB] = numStates;
				numBatchStates[currData]++;
				numStates++;
			}

			// Body
			if(bodyRefTable[bodyA] == 0xff) {
				dmaListBody[currData][numBatchBodies[currData]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyA;
				bodyRefTable[bodyA] = numBodies;
				numBatchBodies[currData]++;
				numBodies++;
			}

			if(bodyRefTable[bodyB] == 0xff) {
				dmaListBody[currData][numBatchBodies[currData]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyB;
				bodyRefTable[bodyB] = numBodies;
				numBatchBodies[currData]++;
				numBodies++;
			}
		}

		spu_dma_list_getf(contactBuffer[currData], 0, contactDmaListBuf[currData], sizeof(spu_dma_list_element)*numContacts[currData], tag1, 0, 0);
		spu_dma_list_get(nextStatesPtr, 0, dmaListState[currData], sizeof(spu_dma_list_element)*numBatchStates[currData], tag1, 0, 0);
		spu_dma_list_get(nextBodiesPtr, 0, dmaListBody[currData], sizeof(spu_dma_list_element)*numBatchBodies[currData], tag1, 0, 0);

		nextStatesPtr += numBatchStates[currData];
		nextBodiesPtr += numBatchBodies[currData];

		currData = 1 - currData;
	}

	spu_dma_list_put(contactBuffer[currData], 0, contactDmaListBuf[currData], sizeof(spu_dma_list_element)*numContacts[currData], tag1, 0, 0);

	startStateDmaListPutImmediate(numStates, states, tag1, statesEA);

	DEALLOCATE(contactDmaListBuf[2]);
	DEALLOCATE(contactDmaListBuf[1]);
	DEALLOCATE(contactDmaListBuf[0]);

	DEALLOCATE(contactBuffer[1]);
	DEALLOCATE(contactBuffer[0]);

	DEALLOCATE(dmaListBody[1]);
	DEALLOCATE(dmaListBody[0]);
	DEALLOCATE(dmaListState[1]);
	DEALLOCATE(dmaListState[0]);

	DEALLOCATE(bodyRefTable);
	DEALLOCATE(stateRefTable);

	DEALLOCATE(bodies);
	DEALLOCATE(statesEA);
	DEALLOCATE(states);
}

void applyContactImpulseEx(SortData *sorts, u32 numSorts,const ContactSolverConfig& config)
{
	s32 tag0 = 10;
	s32 tag1 = 11;

	TrbState *states = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numSorts*2);
	u32 *statesEA = (u32*)ALLOCATE(16, sizeof(u32)*numSorts*2);

	u32 numBatch = 8;

	u32 numStates  = 0;

	u8 *stateRefTable  = (u8*)ALLOCATE(16, sizeof(u8)*solverIO.numStates);

	memset(stateRefTable, 0xff, sizeof(u8)*solverIO.numStates);

	spu_dma_list_element *dmaListState[2];

	s32 numBatchStates[2] = {0};

	dmaListState[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	dmaListState[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);

	u32 contactAddr = solverIO.contactsAddr;

	ContactPair *contactBuffer[2];
	spu_dma_list_element *contactDmaListBuf[3];

	contactBuffer[0] = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numBatch);
	contactBuffer[1] = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numBatch);
	contactDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	contactDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	contactDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);// PUT用

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numContacts[2] = {0};

	TrbState *nextStatesPtr = states;

	u32 jEnd = (numBatch < numSorts) ? numBatch : numSorts;
	startStateDmaListGet(jEnd, nextStatesPtr, tag0, dmaListState[0], &sorts[0], stateRefTable, statesEA, numStates, numBatchStates[0]);
	startStructDmaList(jEnd, contactBuffer[0], tag0, contactDmaListBuf[0], &sorts[0], sizeof(ContactPair), contactAddr, sizeof(ContactPair));
	numContacts[0] += jEnd;

	nextStatesPtr += numBatchStates[0];

	jEnd = (numBatch+numBatch < numSorts) ? numBatch : numSorts - numBatch;
	jEnd = (numBatch < numSorts) ? jEnd : 0;
	startStateDmaListGet(jEnd, nextStatesPtr, tag1, dmaListState[1], &sorts[numBatch], stateRefTable, statesEA, numStates, numBatchStates[1]);
	startStructDmaList(jEnd, contactBuffer[1], tag1, contactDmaListBuf[1], &sorts[numBatch], sizeof(ContactPair), contactAddr, sizeof(ContactPair));
	numContacts[1] += jEnd;

	nextStatesPtr += numBatchStates[1];

	batchFlag = 1;
	currData = 0;

	spu_dma_wait_tag_status_all(1<<tag0);

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		for(int j=0;j < numContacts[currData];j++) {
			ContactPair& contact = contactBuffer[currData][j];
			TrbState& stateA = states[stateRefTable[contact.stateIndex[0]]];
			TrbState& stateB = states[stateRefTable[contact.stateIndex[1]]];
			applyImpulseMovAndMovInline(contact, stateA, stateB, config);
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		memcpy(contactDmaListBuf[2], contactDmaListBuf[currData], sizeof(spu_dma_list_element)*numContacts[currData]);
		spu_dma_list_put(contactBuffer[currData], 0, contactDmaListBuf[2], sizeof(spu_dma_list_element)*numContacts[currData], tag1, 0, 0);

		numContacts[currData] = 0;
		numBatchStates[currData] = 0;

		batchFlag++;
		u32 start = batchFlag * numBatch;
		jEnd = (start+numBatch < numSorts) ? numBatch : numSorts - start;
		jEnd = (start < numSorts) ? jEnd : 0;
		startStateDmaListGet(jEnd, nextStatesPtr, tag1, dmaListState[currData], &sorts[start], stateRefTable, statesEA, numStates, numBatchStates[currData]);
		mfc_barrier(tag1);
		startStructDmaList(jEnd, contactBuffer[currData], tag1, contactDmaListBuf[currData], &sorts[start], sizeof(ContactPair), contactAddr, sizeof(ContactPair));
		numContacts[currData] += jEnd;

		nextStatesPtr += numBatchStates[currData];

		currData = 1-currData;
	}

	spu_dma_list_put(contactBuffer[currData], 0, contactDmaListBuf[currData], sizeof(spu_dma_list_element)*numContacts[currData], tag1, 0, 0);

	startStateDmaListPutImmediate(numStates, states, tag1, statesEA);

	DEALLOCATE(contactDmaListBuf[2]);
	DEALLOCATE(contactDmaListBuf[1]);
	DEALLOCATE(contactDmaListBuf[0]);

	DEALLOCATE(contactBuffer[1]);
	DEALLOCATE(contactBuffer[0]);

	DEALLOCATE(dmaListState[1]);
	DEALLOCATE(dmaListState[0]);

	DEALLOCATE(stateRefTable);

	DEALLOCATE(statesEA);
	DEALLOCATE(states);
}

void preJointImpulseEx(SortData *sorts, u32 numSorts, const JointSolverConfig& config)
{
	s32 tag1 = 11;

	TrbState *states = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numSorts*2);
	u32 *statesEA = (u32*)ALLOCATE(16, sizeof(u32)*numSorts*2);
	TrbDynBody *bodies = (TrbDynBody*)ALLOCATE(128, sizeof(TrbDynBody)*numSorts*2);

	u32 numBatch = 8;

	u8 numStates  = 0;
	u8 numBodies  = 0;

	u8 *stateRefTable  = (u8*)ALLOCATE(16, sizeof(u8)*solverIO.numStates);
	u8 *bodyRefTable  = (u8*)ALLOCATE(16, sizeof(u8)*solverIO.numBodies);

	memset(stateRefTable, 0xff, sizeof(u8)*solverIO.numStates);
	memset(bodyRefTable, 0xff, sizeof(u8)*solverIO.numBodies);

	spu_dma_list_element *dmaListState[2];
	spu_dma_list_element *dmaListBody[2];

	s32 numBatchStates[2] = {0};
	s32 numBatchBodies[2] = {0};

	dmaListState[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	dmaListState[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	dmaListBody[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	dmaListBody[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);

	setListDmaDual(dmaListState[0], dmaListState[1], numBatch*2, sizeof(TrbState));
	setListDmaDual(dmaListBody[0], dmaListBody[1], numBatch*2, sizeof(TrbDynBody));

	u32 jointAddr = solverIO.jointsAddr;

	Joint *jointBuffer[2];
	spu_dma_list_element *jointDmaListBuf[3];

	jointBuffer[0] = (Joint*)ALLOCATE(128, sizeof(Joint)*numBatch);
	jointBuffer[1] = (Joint*)ALLOCATE(128, sizeof(Joint)*numBatch);
	jointDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	jointDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	jointDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);// PUT用

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numJoints[2] = {0};

	TrbState *nextStatesPtr = states;
	TrbDynBody *nextBodiesPtr = bodies;

	setListDmaDual(jointDmaListBuf[0], jointDmaListBuf[1], numBatch, sizeof(Joint));

	for(u32 j=0;j < numBatch && j < numSorts;j++) {
		jointDmaListBuf[0][numJoints[0]].eal = jointAddr + sizeof(Joint)*getPair(sorts[j]);
		numJoints[0]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);
		u16 bodyA = getBodyA(sorts[j]);
		u16 bodyB = getBodyB(sorts[j]);

		// State
		if(stateRefTable[stateA] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateA;
			dmaListState[0][numBatchStates[0]].eal = statesEA[numStates];
			stateRefTable[stateA] = numStates;
			numBatchStates[0]++;
			numStates++;
		}

		if(stateRefTable[stateB] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateB;
			dmaListState[0][numBatchStates[0]].eal = statesEA[numStates];
			stateRefTable[stateB] = numStates;
			numBatchStates[0]++;
			numStates++;
		}

		// Body
		if(bodyRefTable[bodyA] == 0xff) {
			dmaListBody[0][numBatchBodies[0]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyA;
			bodyRefTable[bodyA] = numBodies;
			numBatchBodies[0]++;
			numBodies++;
		}

		if(bodyRefTable[bodyB] == 0xff) {
			dmaListBody[0][numBatchBodies[0]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyB;
			bodyRefTable[bodyB] = numBodies;
			numBatchBodies[0]++;
			numBodies++;
		}
	}

	spu_dma_list_get(jointBuffer[0], 0, jointDmaListBuf[0], sizeof(spu_dma_list_element)*numJoints[0], tag1, 0, 0);
	spu_dma_list_get(states, 0, dmaListState[0], sizeof(spu_dma_list_element)*numBatchStates[0], tag1, 0, 0);
	spu_dma_list_get(bodies, 0, dmaListBody[0], sizeof(spu_dma_list_element)*numBatchBodies[0], tag1, 0, 0);

	nextStatesPtr += numBatchStates[0];
	nextBodiesPtr += numBatchBodies[0];

	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 j=numBatch;j < numBatch + numBatch && j < numSorts;j++) {
		jointDmaListBuf[1][numJoints[1]].eal = jointAddr + sizeof(Joint)*getPair(sorts[j]);
		numJoints[1]++;

		u16 stateA = getStateA(sorts[j]);
		u16 stateB = getStateB(sorts[j]);
		u16 bodyA = getBodyA(sorts[j]);
		u16 bodyB = getBodyB(sorts[j]);

		// State
		if(stateRefTable[stateA] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateA;
			dmaListState[1][numBatchStates[1]].eal = statesEA[numStates];
			stateRefTable[stateA] = numStates;
			numBatchStates[1]++;
			numStates++;
		}

		if(stateRefTable[stateB] == 0xff) {
			statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateB;
			dmaListState[1][numBatchStates[1]].eal = statesEA[numStates];
			stateRefTable[stateB] = numStates;
			numBatchStates[1]++;
			numStates++;
		}

		// Body
		if(bodyRefTable[bodyA] == 0xff) {
			dmaListBody[1][numBatchBodies[1]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyA;
			bodyRefTable[bodyA] = numBodies;
			numBatchBodies[1]++;
			numBodies++;
		}

		if(bodyRefTable[bodyB] == 0xff) {
			dmaListBody[1][numBatchBodies[1]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyB;
			bodyRefTable[bodyB] = numBodies;
			numBatchBodies[1]++;
			numBodies++;
		}
	}

	spu_dma_list_get(jointBuffer[1], 0, jointDmaListBuf[1],sizeof(spu_dma_list_element)*numJoints[1], tag1, 0, 0);
	spu_dma_list_get(nextStatesPtr, 0, dmaListState[1], sizeof(spu_dma_list_element)*numBatchStates[1], tag1, 0, 0);
	spu_dma_list_get(nextBodiesPtr, 0, dmaListBody[1], sizeof(spu_dma_list_element)*numBatchBodies[1], tag1, 0, 0);

	nextStatesPtr += numBatchStates[1];
	nextBodiesPtr += numBatchBodies[1];

	currData = 0;
	batchFlag = 1;

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		for(int j=0;j < numJoints[currData];j++) {
			Joint& joint = jointBuffer[currData][j];
			TrbState& stateA = states[stateRefTable[joint.stateIndexA]];
			TrbState& stateB = states[stateRefTable[joint.stateIndexB]];
			TrbDynBody& bodyA = bodies[bodyRefTable[stateA.trbBodyIdx]];
			TrbDynBody& bodyB = bodies[bodyRefTable[stateB.trbBodyIdx]];

			funcTbl_preJoint[stateA.moveType + stateA.sleeping][stateB.moveType + stateB.sleeping](joint, stateA, bodyA, stateB, bodyB, config);
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		memcpy(jointDmaListBuf[2], jointDmaListBuf[currData], sizeof(spu_dma_list_element)*numJoints[currData]);
		spu_dma_list_put(jointBuffer[currData], 0, jointDmaListBuf[2], sizeof(spu_dma_list_element)*numJoints[currData], tag1, 0, 0);

		numJoints[currData] = 0;
		numBatchStates[currData] = 0;
		numBatchBodies[currData] = 0;

		batchFlag++;
		s32 start = batchFlag*numBatch;

		for(u32 j=start;j < start + numBatch && j < numSorts;j++) {
			jointDmaListBuf[currData][numJoints[currData]].eal = jointAddr + sizeof(Joint)*getPair(sorts[j]);
			numJoints[currData]++;

			u16 stateA = getStateA(sorts[j]);
			u16 stateB = getStateB(sorts[j]);
			u16 bodyA = getBodyA(sorts[j]);
			u16 bodyB = getBodyB(sorts[j]);

			// State
			if(stateRefTable[stateA] == 0xff) {
				statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateA;
				dmaListState[currData][numBatchStates[currData]].eal = statesEA[numStates];
				stateRefTable[stateA] = numStates;
				numBatchStates[currData]++;
				numStates++;
			}

			if(stateRefTable[stateB] == 0xff) {
				statesEA[numStates] = solverIO.statesAddr + sizeof(TrbState)*stateB;
				dmaListState[currData][numBatchStates[currData]].eal = statesEA[numStates];
				stateRefTable[stateB] = numStates;
				numBatchStates[currData]++;
				numStates++;
			}

			// Body
			if(bodyRefTable[bodyA] == 0xff) {
				dmaListBody[currData][numBatchBodies[currData]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyA;
				bodyRefTable[bodyA] = numBodies;
				numBatchBodies[currData]++;
				numBodies++;
			}

			if(bodyRefTable[bodyB] == 0xff) {
				dmaListBody[currData][numBatchBodies[currData]].eal = solverIO.bodiesAddr + sizeof(TrbDynBody)*bodyB;
				bodyRefTable[bodyB] = numBodies;
				numBatchBodies[currData]++;
				numBodies++;
			}
		}

		spu_dma_list_getf(jointBuffer[currData], 0, jointDmaListBuf[currData], sizeof(spu_dma_list_element)*numJoints[currData], tag1, 0, 0);
		spu_dma_list_get(nextStatesPtr, 0, dmaListState[currData], sizeof(spu_dma_list_element)*numBatchStates[currData], tag1, 0, 0);
		spu_dma_list_get(nextBodiesPtr, 0, dmaListBody[currData], sizeof(spu_dma_list_element)*numBatchBodies[currData], tag1, 0, 0);

		nextStatesPtr += numBatchStates[currData];
		nextBodiesPtr += numBatchBodies[currData];

		currData = 1-currData;
	}

	spu_dma_list_put(jointBuffer[currData], 0, jointDmaListBuf[currData], sizeof(spu_dma_list_element)*numJoints[currData], tag1, 0, 0);

	startStateDmaListPutImmediate(numStates, states, tag1, statesEA);

	DEALLOCATE(jointDmaListBuf[2]);
	DEALLOCATE(jointDmaListBuf[1]);
	DEALLOCATE(jointDmaListBuf[0]);

	DEALLOCATE(jointBuffer[1]);
	DEALLOCATE(jointBuffer[0]);

	DEALLOCATE(dmaListBody[1]);
	DEALLOCATE(dmaListBody[0]);
	DEALLOCATE(dmaListState[1]);
	DEALLOCATE(dmaListState[0]);

	DEALLOCATE(bodyRefTable);
	DEALLOCATE(stateRefTable);

	DEALLOCATE(bodies);
	DEALLOCATE(statesEA);
	DEALLOCATE(states);
}

void applyJointImpulseEx(SortData *sorts, u32 numSorts, const JointSolverConfig& config)
{
	s32 tag0 = 10;
	s32 tag1 = 11;

	const s32 jointDmaBytes = 384 + 128;

	TrbState *states = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numSorts*2);
	u32 *statesEA = (u32*)ALLOCATE(16, sizeof(u32)*numSorts*2);

	u32 numBatch = 8;

	u32 numStates  = 0;

	u8 *stateRefTable  = (u8*)ALLOCATE(16, sizeof(u8)*solverIO.numStates);

	memset(stateRefTable, 0xff, sizeof(u8)*solverIO.numStates);

	spu_dma_list_element *dmaListState[2];

	s32 numBatchStates[2] = {0};

	dmaListState[0] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);
	dmaListState[1] = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*numBatch*2);

	u32 jointAddr = solverIO.jointsAddr;

	Joint *jointBuffer[2];
	spu_dma_list_element *jointDmaListBuf[3];

	jointBuffer[0] = (Joint*)ALLOCATE(128, jointDmaBytes*numBatch);
	jointBuffer[1] = (Joint*)ALLOCATE(128, jointDmaBytes*numBatch);
	jointDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*((numBatch+1)/2*2));
	jointDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*((numBatch+1)/2*2));
	jointDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*((numBatch+1)/2*2));// PUT用

	s32 batchFlag = 0;
	s32 currData = 0;
	s32 numJoints[2] = {0};

	TrbState *nextStatesPtr = states;

	u32 jEnd = (numBatch < numSorts) ? numBatch : numSorts;
	numJoints[0] += jEnd;
	startStructDmaList(jEnd, jointBuffer[0], tag0, jointDmaListBuf[0], &sorts[0], jointDmaBytes, jointAddr, sizeof(Joint));
	startStateDmaListGet(jEnd, nextStatesPtr, tag0, dmaListState[0], &sorts[0], stateRefTable, statesEA, numStates, numBatchStates[0]);

	nextStatesPtr += numBatchStates[0];


	jEnd = (numBatch+numBatch < numSorts) ? numBatch : numSorts - numBatch;
	jEnd = (numBatch < numSorts) ? jEnd : 0;
	startStructDmaList(jEnd, jointBuffer[1], tag1, jointDmaListBuf[1], &sorts[numBatch], jointDmaBytes, jointAddr, sizeof(Joint));
	startStateDmaListGet(jEnd, nextStatesPtr, tag1, dmaListState[1], &sorts[numBatch], stateRefTable, statesEA, numStates, numBatchStates[1]);
	numJoints[1] += jEnd;

	nextStatesPtr += numBatchStates[1];

	batchFlag = 1;
	currData = 0;

	jEnd = (numBatch+numBatch < numSorts) ? numBatch : numSorts - numBatch;
	jEnd = (numBatch < numSorts) ? jEnd : 0;

	spu_dma_wait_tag_status_all(1<<tag0);

	for(u32 i=0;i < numSorts;) {
		// UPDATE
		s32 j;
		for(j=0;j + 2 <= numJoints[currData];j+=2) {
			Joint& joint0 = *((Joint*)((uintptr_t)jointBuffer[currData] + jointDmaBytes*j));
			Joint& joint1 = *((Joint*)((uintptr_t)jointBuffer[currData] + jointDmaBytes*(j + 1)));
			TrbState& stateA0 = states[stateRefTable[joint0.stateIndexA]];
			TrbState& stateB0 = states[stateRefTable[joint0.stateIndexB]];
			TrbState& stateA1 = states[stateRefTable[joint1.stateIndexA]];
			TrbState& stateB1 = states[stateRefTable[joint1.stateIndexB]];

			if((joint0.stateIndexA != joint1.stateIndexA) &&
			   (joint0.stateIndexA != joint1.stateIndexB) &&
			   (joint0.stateIndexB != joint1.stateIndexA) &&
			   (joint0.stateIndexB != joint1.stateIndexB))
			{
				applyJointMovAndMovInlineDouble(joint0, stateA0, stateB0, joint1, stateA1, stateB1, config);
			} else {
				applyJointMovAndMovInline(joint0, stateA0, stateB0, config);
				applyJointMovAndMovInline(joint1, stateA1, stateB1, config);
			}
		}
		if(j < numJoints[currData]) {
			Joint& joint = *((Joint*)((uintptr_t)jointBuffer[currData] + jointDmaBytes*j));
			TrbState& stateA = states[stateRefTable[joint.stateIndexA]];
			TrbState& stateB = states[stateRefTable[joint.stateIndexB]];
			applyJointMovAndMovInline(joint, stateA, stateB, config);
		}

		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		memcpy(jointDmaListBuf[2], jointDmaListBuf[currData], sizeof(spu_dma_list_element)*numJoints[currData]);
		spu_dma_list_put(jointBuffer[currData], 0, jointDmaListBuf[2], sizeof(spu_dma_list_element)*numJoints[currData], tag1, 0, 0);

		if(i >= numSorts){
			currData = 1 - currData;
			break;
		}

		numJoints[currData] = 0;
		numBatchStates[currData] = 0;

		batchFlag++;
		u32 start = batchFlag*numBatch;

		jEnd = (start+numBatch < numSorts) ? numBatch : numSorts - start;
		jEnd = (start < numSorts) ? jEnd : 0;

		startStateDmaListGet(jEnd, nextStatesPtr, tag1, dmaListState[currData], &sorts[start], stateRefTable, statesEA, numStates, numBatchStates[currData]);
		mfc_barrier(tag1);
		startStructDmaList(jEnd, jointBuffer[currData], tag1, jointDmaListBuf[currData], &sorts[start], jointDmaBytes, jointAddr, sizeof(Joint));
		numJoints[currData] += jEnd;

		nextStatesPtr += numBatchStates[currData];

		currData = 1-currData;
	}

	spu_dma_list_put(jointBuffer[currData], 0, jointDmaListBuf[currData], sizeof(spu_dma_list_element)*numJoints[currData], tag1, 0, 0);

	startStateDmaListPutImmediate(numStates, states, tag1, statesEA);

	DEALLOCATE(jointDmaListBuf[2]);
	DEALLOCATE(jointDmaListBuf[1]);
	DEALLOCATE(jointDmaListBuf[0]);

	DEALLOCATE(jointBuffer[1]);
	DEALLOCATE(jointBuffer[0]);

	DEALLOCATE(dmaListState[1]);
	DEALLOCATE(dmaListState[0]);

	DEALLOCATE(stateRefTable);

	DEALLOCATE(statesEA);
	DEALLOCATE(states);
}

#endif

#ifndef ENABLE_PREFETCH_SOLVER

void preContactImpulse(SortData *sorts,u32 numSorts,const ContactSolverConfig &config)
{
	int tag1 = 10;

	TrbState stateA,stateB;
	TrbDynBody bodyA,bodyB;
	ContactPair pair;

	for(u32 i=0;i<numSorts;i++) {
		SortData &sort = sorts[i];

		spu_dma_get(&stateA,solverIO.statesAddr + sizeof(TrbState)*StateA(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_get(&stateB,solverIO.statesAddr + sizeof(TrbState)*StateB(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_get(&bodyA,solverIO.bodiesAddr + sizeof(TrbDynBody)*BodyA(sort),sizeof(TrbDynBody),tag1,0,0);
		spu_dma_get(&bodyB,solverIO.bodiesAddr + sizeof(TrbDynBody)*BodyB(sort),sizeof(TrbDynBody),tag1,0,0);
		spu_dma_get(&pair,solverIO.contactsAddr + sizeof(ContactPair)*Pair(sort),sizeof(ContactPair),tag1,0,0);
		spu_dma_wait_tag_status_all(1<<tag1);

		funcTbl_preResponse[stateA.moveType+stateA.sleeping][stateB.moveType+stateB.sleeping](pair,stateA,bodyA,stateB,bodyB,config);

		spu_dma_put(&stateA,solverIO.statesAddr + sizeof(TrbState)*StateA(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_put(&stateB,solverIO.statesAddr + sizeof(TrbState)*StateB(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_put(&bodyA,solverIO.bodiesAddr + sizeof(TrbDynBody)*BodyA(sort),sizeof(TrbDynBody),tag1,0,0);
		spu_dma_put(&bodyB,solverIO.bodiesAddr + sizeof(TrbDynBody)*BodyB(sort),sizeof(TrbDynBody),tag1,0,0);
		spu_dma_put(&pair,solverIO.contactsAddr + sizeof(ContactPair)*Pair(sort),sizeof(ContactPair),tag1,0,0);
		spu_dma_wait_tag_status_all(1<<tag1);
	}
}

void applyContactImpulse(SortData *sorts,u32 numSorts,const ContactSolverConfig &config)
{
	int tag1 = 10;

	TrbState stateA,stateB;
	TrbDynBody bodyA,bodyB;
	ContactPair pair;

	for(u32 i=0;i<numSorts;i++) {
		SortData &sort = sorts[i];

		spu_dma_get(&stateA,solverIO.statesAddr + sizeof(TrbState)*StateA(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_get(&stateB,solverIO.statesAddr + sizeof(TrbState)*StateB(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_get(&pair,solverIO.contactsAddr + sizeof(ContactPair)*Pair(sort),sizeof(ContactPair),tag1,0,0);
		spu_dma_wait_tag_status_all(1<<tag1);

		applyImpulseMovAndMovInline(pair,stateA,stateB,config);

		spu_dma_put(&stateA,solverIO.statesAddr + sizeof(TrbState)*StateA(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_put(&stateB,solverIO.statesAddr + sizeof(TrbState)*StateB(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_put(&pair,solverIO.contactsAddr + sizeof(ContactPair)*Pair(sort),sizeof(ContactPair),tag1,0,0);
		spu_dma_wait_tag_status_all(1<<tag1);
	}
}

void preJointImpulse(SortData *sorts,u32 numSorts,const JointSolverConfig &config)
{
	int tag1 = 10;

	TrbState stateA,stateB;
	TrbDynBody bodyA,bodyB;
	Joint joint;

	for(u32 i=0;i<numSorts;i++) {
		SortData &sort = sorts[i];

		spu_dma_get(&stateA,solverIO.statesAddr + sizeof(TrbState)*StateA(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_get(&stateB,solverIO.statesAddr + sizeof(TrbState)*StateB(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_get(&bodyA,solverIO.bodiesAddr + sizeof(TrbDynBody)*BodyA(sort),sizeof(TrbDynBody),tag1,0,0);
		spu_dma_get(&bodyB,solverIO.bodiesAddr + sizeof(TrbDynBody)*BodyB(sort),sizeof(TrbDynBody),tag1,0,0);
		spu_dma_get(&joint,solverIO.jointsAddr + sizeof(Joint)*Pair(sort),sizeof(Joint),tag1,0,0);
		spu_dma_wait_tag_status_all(1<<tag1);

		funcTbl_preJoint[stateA.moveType+stateA.sleeping][stateB.moveType+stateB.sleeping](joint,stateA,bodyA,stateB,bodyB,config);

		spu_dma_put(&stateA,solverIO.statesAddr + sizeof(TrbState)*StateA(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_put(&stateB,solverIO.statesAddr + sizeof(TrbState)*StateB(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_put(&bodyA,solverIO.bodiesAddr + sizeof(TrbDynBody)*BodyA(sort),sizeof(TrbDynBody),tag1,0,0);
		spu_dma_put(&bodyB,solverIO.bodiesAddr + sizeof(TrbDynBody)*BodyB(sort),sizeof(TrbDynBody),tag1,0,0);
		spu_dma_put(&joint,solverIO.jointsAddr + sizeof(Joint)*Pair(sort),sizeof(Joint),tag1,0,0);
		spu_dma_wait_tag_status_all(1<<tag1);
	}
}

void applyJointImpulse(SortData *sorts,u32 numSorts,const JointSolverConfig &config)
{
	int tag1 = 10;

	TrbState stateA,stateB;
	Joint joint;

	for(u32 i=0;i<numSorts;i++) {
		SortData &sort = sorts[i];

		spu_dma_get(&stateA,solverIO.statesAddr + sizeof(TrbState)*StateA(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_get(&stateB,solverIO.statesAddr + sizeof(TrbState)*StateB(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_get(&joint,solverIO.jointsAddr + sizeof(Joint)*Pair(sort),sizeof(Joint),tag1,0,0);
		spu_dma_wait_tag_status_all(1<<tag1);

		applyJointMovAndMovInline(joint, stateA, stateB, config);

		spu_dma_put(&stateA,solverIO.statesAddr + sizeof(TrbState)*StateA(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_put(&stateB,solverIO.statesAddr + sizeof(TrbState)*StateB(sort),sizeof(TrbState),tag1,0,0);
		spu_dma_put(&joint,solverIO.jointsAddr + sizeof(Joint)*Pair(sort),sizeof(Joint),tag1,0,0);
		spu_dma_wait_tag_status_all(1<<tag1);
	}
}

#endif

void preContactImpulsePhaseEx()
{
	ContactSolverConfig config;
	config.timeStep = solverIO.timeStep;
	config.separateBias = solverIO.separateBias;
	config.deformMeshEnable = solverIO.deformMeshEnable;

	spu_dma_list_element *dmaListPairs = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*MAX_SOLVER_PAIRS);
	for(s32 i=0;i < MAX_SOLVER_PAIRS;i++) {
		dmaListPairs[i].notify = 0;
		dmaListPairs[i].size = sizeof(SortData);
	}

	for(u32 phaseId=0;phaseId < contactSolverInfo->numPhases;phaseId++) {
		while(1) {
			u32 groupId;
			groupId = spu_atomic_incr32(atomicBuf, solverIO.nextGroupIndexAddr);
			u32 numPairs = contactSolverInfo->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId >= contactSolverInfo->numGroups[phaseId]) {
				if(groupId == contactSolverInfo->numGroups[phaseId] + solverIO.numSPU - 1)
					spu_atomic_store32(atomicBuf, solverIO.nextGroupIndexAddr, 0);
				break;
			}
			if(numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIO.contactGroupsAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);

				for(u32 i=0;i < numPairs;i++)
					dmaListPairs[i].eal = solverIO.contactPairsAddr + sizeof(SortData)*group->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs, sizeof(spu_dma_list_element)*numPairs, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

#ifdef ENABLE_PREFETCH_SOLVER
				preContactImpulseEx(sortBuff, numPairs, config);
#else
				preContactImpulse(sortBuff, numPairs, config);
#endif

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	DEALLOCATE(dmaListPairs);
}

void applyContactImpulsePhaseEx()
{
	ContactSolverConfig config;
	config.timeStep = solverIO.timeStep;
	config.separateBias = solverIO.separateBias;
	config.deformMeshEnable = solverIO.deformMeshEnable;

	spu_dma_list_element *dmaListPairs = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*MAX_SOLVER_PAIRS);
	setListDma(dmaListPairs, MAX_SOLVER_PAIRS, sizeof(SortData));

	for(u32 phaseId=0;phaseId < contactSolverInfo->numPhases;phaseId++) {
		while(1) {
			u32 groupId;
			groupId = spu_atomic_incr32(atomicBuf, solverIO.nextGroupIndexAddr);
			u32 numPairs = contactSolverInfo->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId >= contactSolverInfo->numGroups[phaseId]){
				if(groupId == contactSolverInfo->numGroups[phaseId] + solverIO.numSPU - 1)
					spu_atomic_store32(atomicBuf, solverIO.nextGroupIndexAddr, 0);
				break;
			}
			if(numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIO.contactGroupsAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);

				for(u32 i=0;i < numPairs;i++) {
					dmaListPairs[i].eal = solverIO.contactPairsAddr + sizeof(SortData)*group->pairIndices[i];
					spu_dma_list_get(sortBuff + i, 0, dmaListPairs + i, sizeof(spu_dma_list_element), 0, 0, 0);
				}
				spu_dma_wait_tag_status_all(1);

#ifdef ENABLE_PREFETCH_SOLVER
				applyContactImpulseEx(sortBuff, numPairs, config);
#else
				applyContactImpulse(sortBuff, numPairs, config);
#endif

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	DEALLOCATE(dmaListPairs);
}

void preJointImpulsePhaseEx()
{
	JointSolverConfig config;
	config.timeStep = solverIO.timeStep;
	config.deformMeshEnable = solverIO.deformMeshEnable;

	spu_dma_list_element *dmaListPairs = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*MAX_SOLVER_PAIRS);
	for(s32 i=0;i < MAX_SOLVER_PAIRS;i++) {
		dmaListPairs[i].notify = 0;
		dmaListPairs[i].size = sizeof(SortData);
	}

	for(u32 phaseId=0;phaseId < jointSolverInfo->numPhases;phaseId++) {
		while(1) {
			u32 groupId;
			groupId = spu_atomic_incr32(atomicBuf, solverIO.nextGroupIndexAddr);
			u32 numPairs = jointSolverInfo->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId];
			if(groupId >= jointSolverInfo->numGroups[phaseId]){
				if(groupId == jointSolverInfo->numGroups[phaseId] + solverIO.numSPU - 1)
					spu_atomic_store32(atomicBuf, solverIO.nextGroupIndexAddr, 0);
				break;
			}
			if(numPairs > 0) {
				SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				spu_dma_get(group, solverIO.jointGroupsAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*numPairs);

				for(u32 i=0;i < numPairs;i++)
					dmaListPairs[i].eal = solverIO.jointPairsAddr + sizeof(SortData)*group->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs, sizeof(spu_dma_list_element)*numPairs, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

#ifdef ENABLE_PREFETCH_SOLVER
				preJointImpulseEx(sortBuff, numPairs, config);
#else
				preJointImpulse(sortBuff, numPairs, config);
#endif

				DEALLOCATE(sortBuff);
				DEALLOCATE(group);
			}
		}
		sync();
	}

	DEALLOCATE(dmaListPairs);
}

void applyJointImpulsePhaseEx()
{
	JointSolverConfig config;
	config.timeStep = solverIO.timeStep;
	config.deformMeshEnable = solverIO.deformMeshEnable;

	spu_dma_list_element *dmaListPairs = (spu_dma_list_element*)ALLOCATE(16, sizeof(spu_dma_list_element)*MAX_SOLVER_PAIRS*2);
	setListDma(dmaListPairs, MAX_SOLVER_PAIRS*2, sizeof(SortData));

	for(u32 phaseId=0;phaseId < jointSolverInfo->numPhases;phaseId++) {
		u32 numGroups = (jointSolverInfo->numGroups[phaseId] + 1)/2;
		while(1) {
			u32 groupId;
			groupId = spu_atomic_incr32(atomicBuf, solverIO.nextGroupIndexAddr);
			u32 numPairs1 = jointSolverInfo->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId*2];
			u32 numPairs2;
			if(groupId*2 + 1 < jointSolverInfo->numGroups[phaseId])
				numPairs2 = jointSolverInfo->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId*2 + 1];
			else
				numPairs2 = 0;

			if(groupId >= numGroups) {
				if(groupId == numGroups + solverIO.numSPU - 1)
					spu_atomic_store32(atomicBuf, solverIO.nextGroupIndexAddr, 0);
				break;
			}
			if(numPairs1 + numPairs2 > 0) {
				SolverGroup *group1 = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				SolverGroup *group2 = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));
				if(numPairs1)
					spu_dma_get(group1, solverIO.jointGroupsAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId*2), sizeof(SolverGroup), 0, 0, 0);
				if(numPairs2)
					spu_dma_get(group2, solverIO.jointGroupsAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId*2 + 1), sizeof(SolverGroup), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				SortData *sortBuff = (SortData*)ALLOCATE(128, sizeof(SortData)*(numPairs1 + numPairs2));

				u32 numLists = 0;
				u32 numMin = MIN(numPairs1, numPairs2);
				u32 numMax = MAX(numPairs1, numPairs2);
				SolverGroup *groupMax = numPairs1 > numPairs2 ? group1 : group2;

				u32 i=0;
				for(;i < numMin;i++) {
					dmaListPairs[numLists++].eal = solverIO.jointPairsAddr + sizeof(SortData)*group1->pairIndices[i];
					dmaListPairs[numLists++].eal = solverIO.jointPairsAddr + sizeof(SortData)*group2->pairIndices[i];
				}

				for(;i < numMax;i++)
					dmaListPairs[numLists++].eal = solverIO.jointPairsAddr + sizeof(SortData)*groupMax->pairIndices[i];

				spu_dma_list_get(sortBuff, 0, dmaListPairs, sizeof(spu_dma_list_element)*numLists, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

#ifdef ENABLE_PREFETCH_SOLVER
				applyJointImpulseEx(sortBuff, numLists, config);
#else
				applyJointImpulse(sortBuff, numLists, config);
#endif

				DEALLOCATE(sortBuff);
				DEALLOCATE(group2);
				DEALLOCATE(group1);
			}
		}
		sync();
	}

	DEALLOCATE(dmaListPairs);
}

void postResponse()
{
	PrefetchForwardIterator<TrbState> itrStates(
			&gPool,
			postResponseIO.statesAddr,
			postResponseIO.statesAddr + sizeof(TrbState)*postResponseIO.numStates,
			PREFETCH_NUM, 10);

	for(u32 i=0;i < postResponseIO.numStates;i++, ++itrStates) {
		TrbState& state = *itrStates;

		state.setLinearVelocity(state.getLinearVelocity() + state.getDeltaLinearVelocity());
		state.setAngularVelocity(state.getAngularVelocity() + state.getDeltaAngularVelocity());
		state.setDeltaLinearVelocity(Vector3(0.0f));
		state.setDeltaAngularVelocity(Vector3(0.0f));

		f32 maxVal = postResponseIO.maxLinearVelocity;

		if(length(state.getLinearVelocity()) > maxVal)
			state.setLinearVelocity(normalize(state.getLinearVelocity())*maxVal);

		if(length(state.getAngularVelocity()) > postResponseIO.maxAngularVelocity)
			state.setAngularVelocity(normalize(state.getAngularVelocity())*postResponseIO.maxAngularVelocity);
	}
}

void splitConstraints()
{
	u32 movMask1 = ((1<<(MoveTypeActive + 1))|(1<<(MoveTypeKeyframe + 1))|(1<<(MoveTypeOneWay + 1)));
	u32 movMask2 = ~MOVE_TYPE_CAN_SLEEP;
	u32 movMask = movMask1|movMask2;
	u32 movKey = (1<<(MoveTypeKeyframe))|(1<<(MoveTypeKeyframe + 1)|(1<<(MoveTypeFixed)));

	u32 numPairs = splitConstraintsIO.numPairs;

	SolverInfo *info = (SolverInfo*)ALLOCATE(128, sizeof(SolverInfo));

	spu_dma_get(info, splitConstraintsIO.solverInfoAddr, sizeof(SolverInfo), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	s32 bufSize = sizeof(u8)*splitConstraintsIO.numInstances;
	bufSize = ((bufSize + 127)>>7)<<7; // 128 bytes alignment
	u8 *stateTable = (u8*)ALLOCATE(128, bufSize);

	u32 *pairTable;
	pairTable = (u32*)ALLOCATE(16, sizeof(u32)*((numPairs + 31)/32));
	memset(pairTable, 0, sizeof(u32)*((numPairs + 31)/32));

	u32 targetCount = MAX(MIN_SOLVER_PAIRS, MIN(numPairs/(splitConstraintsIO.numSpu*2), MAX_SOLVER_PAIRS));
	u32 startIndex = 0;

	u32 phaseId;
	u32 groupId;
	u32 totalCount = 0;

	u32 maxGroups = MIN(splitConstraintsIO.numSpu, MAX_SOLVER_GROUPS);

	for(phaseId=0;phaseId < MAX_SOLVER_PHASES && totalCount < numPairs;phaseId++) {
		bool startIndexCheck = true;

		info->numGroups[phaseId] = 0;

		u32 i = startIndex;

		memset(stateTable, 0xff, bufSize);

		ReadOnlyPrefetchForwardIterator<SortData> itrPairs(
				&gPool,
				splitConstraintsIO.pairsAddr + sizeof(SortData)*i,
				splitConstraintsIO.pairsAddr + sizeof(SortData)*numPairs,
				PREFETCH_NUM, 10);

		for(groupId=0;i < numPairs && totalCount < numPairs && groupId < maxGroups;groupId++) {
			u32 pairCount=0;

			u32 groupAddr = splitConstraintsIO.solverGroupsAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + groupId);
			SolverGroup *group = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup));

			spu_dma_get(group, groupAddr, sizeof(SolverGroup), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			u32 pairId = 0;

			for(;i < numPairs && pairCount < targetCount;i++, ++itrPairs) {
				u32 idxP = i>>5;
				u32 maskP = 1L<<(i&31);

				if(pairTable[idxP]&maskP)
					continue;

				const SortData& pair = *itrPairs;

				u32 idxA = getStateA(pair);
				u32 idxB = getStateB(pair);
				u32 movA = 1<<getMovA(pair);
				u32 movB = 1<<getMovB(pair);

				if(((movA&movMask) && (movB&movMask)) || ((movA&movKey) && (movB&movKey)) ) {
					if(startIndexCheck) startIndex++;
					pairTable[idxP] |= maskP;
					totalCount++;
					continue;
				}

				if((stateTable[idxA] != groupId && stateTable[idxA] != 0xff) ||
				   (stateTable[idxB] != groupId && stateTable[idxB] != 0xff))
				{
					startIndexCheck = false;
					continue;
				}

				if(!(movA&movMask2)) stateTable[idxA] = groupId;
				if(!(movB&movMask2)) stateTable[idxB] = groupId;

				if(startIndexCheck) startIndex++;

				pairTable[idxP] |= maskP;

				group->pairIndices[pairId++] = i;
				pairCount++;
			}

			spu_dma_put(group, groupAddr, sizeof(SolverGroup), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			DEALLOCATE(group);

			info->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId] = (u16)pairId;
			totalCount += pairCount;
		}

		info->numGroups[phaseId] = groupId;
	}

	info->numPhases = phaseId;

	spu_dma_put(info, splitConstraintsIO.solverInfoAddr, sizeof(SolverInfo), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	DEALLOCATE(pairTable);
	DEALLOCATE(stateTable);
	DEALLOCATE(info);
}

static void splitConstraints2()
{
	u32 movMask1 = ((1<<(MoveTypeActive + 1))|(1<<(MoveTypeKeyframe + 1))|(1<<(MoveTypeOneWay + 1)));
	u32 movMask2 = ~MOVE_TYPE_CAN_SLEEP;
	u32 movMask = movMask1|movMask2;
	u32 movKey = (1<<(MoveTypeKeyframe))|(1<<(MoveTypeKeyframe + 1)|(1<<(MoveTypeFixed)));

	u32 numPairs = splitConstraintsIO.numPairs;

	SolverInfo *info = (SolverInfo*)ALLOCATE(128, sizeof(SolverInfo));

	spu_dma_get(info, splitConstraintsIO.solverInfoAddr, sizeof(SolverInfo), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	s32 bufSize = sizeof(u8)*splitConstraintsIO.numInstances;
	bufSize = ((bufSize + 127)>>7)<<7; // 128 bytes alignment
	u8 *stateTable = (u8*)ALLOCATE(128, bufSize);

	u32 *pairTable;
	pairTable = (u32*)ALLOCATE(16, sizeof(u32)*((numPairs + 31)/32));
	memset(pairTable, 0, sizeof(u32)*((numPairs + 31)/32));

	u32 targetCount = MAX(MIN_SOLVER_PAIRS, MIN(numPairs/(splitConstraintsIO.numSpu*2), MAX_SOLVER_PAIRS));
	u32 startIndex = 0;

	u32 phaseId;
	u32 groupId;
	u32 totalCount = 0;

	u32 maxGroups = MIN(splitConstraintsIO.numSpu*2, MAX_SOLVER_GROUPS);

	for(phaseId=0;phaseId < MAX_SOLVER_PHASES && totalCount < numPairs;phaseId++) {
		bool startIndexCheck = true;

		info->numGroups[phaseId] = 0;

		u32 i = startIndex;

		memset(stateTable, 0xff, bufSize);

		ReadOnlyPrefetchForwardIterator<SortData> itrPairs(
				&gPool,
				splitConstraintsIO.pairsAddr + sizeof(SortData)*i,
				splitConstraintsIO.pairsAddr + sizeof(SortData)*numPairs,
				PREFETCH_NUM, 10);

		u32 groupAddr = splitConstraintsIO.solverGroupsAddr + sizeof(SolverGroup)*(phaseId*MAX_SOLVER_GROUPS + 0);
		SolverGroup *groupList = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup)*MAX_SOLVER_GROUPS);
		spu_dma_get(groupList, groupAddr, sizeof(SolverGroup)*MAX_SOLVER_GROUPS, 0, 0, 0);
		spu_dma_wait_tag_status_all(1);
		{
			u32 numPairsInGroup[MAX_SOLVER_GROUPS];
			memset(numPairsInGroup, 0, sizeof(numPairsInGroup));
			u32 pairCount = 0;


			//u32 pairId = 0;

			for(;i < numPairs;i++, ++itrPairs) {
				const u32 idxP = i>>5;
				const u32 maskP = 1L<<(i&31);

				if(UNLIKELY(pairTable[idxP] & maskP))
					continue;

				const SortData& pair = *itrPairs;

				u32 idxA = getStateA(pair);
				u32 idxB = getStateB(pair);
				u32 movA = 1<<getMovA(pair);
				u32 movB = 1<<getMovB(pair);

				if(UNLIKELY((((movA&movMask) !=  0)&((movB&movMask) != 0)) | (((movA&movKey) != 0)&((movB&movKey) != 0)))) {
					if(startIndexCheck) startIndex++;
					pairTable[idxP] |= maskP;
					totalCount++;
					continue;
				}

				u32 groupA = stateTable[idxA];
				u32 groupB = stateTable[idxB];
				if((groupA == 0xff)&(groupB == 0xff)){
					//add to new group
					groupId = 0;
					for(u32 gid =0;gid < maxGroups;gid++)
						groupId = numPairsInGroup[gid] < numPairsInGroup[groupId] ? gid : groupId;

					if(UNLIKELY(numPairsInGroup[groupId] >= targetCount)) break;
				} else if((groupA != 0xff)&(groupB != 0xff)&(groupA != groupB)) {
					//keep to next phase
					startIndexCheck = false;
					continue;
				} else {
					groupId = groupA != 0xff ? groupA : groupB;
					//add to existing group
					if(numPairsInGroup[groupId] >= targetCount){
						//keep to next phase
						startIndexCheck = false;
						continue;
					}
				}

				SolverGroup *group = (groupList + groupId);


				if(!(movA&movMask2)) stateTable[idxA] = groupId;
				if(!(movB&movMask2)) stateTable[idxB] = groupId;

				if(startIndexCheck) startIndex++;

				pairTable[idxP] |= maskP;

				group->pairIndices[numPairsInGroup[groupId]++] = i;
				pairCount++;
			}

			for(groupId=0;groupId < maxGroups;groupId++) {
				info->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId] = numPairsInGroup[groupId];
				if(numPairsInGroup[groupId] > 0)
					info->numGroups[phaseId] = groupId + 1;
			}
			totalCount += pairCount;
		}

		spu_dma_put(groupList, groupAddr, sizeof(SolverGroup)*MAX_SOLVER_GROUPS, 0, 0, 0);
		spu_dma_wait_tag_status_all(1);
		DEALLOCATE(groupList);
	}

	info->numPhases = phaseId;

	spu_dma_put(info, splitConstraintsIO.solverInfoAddr, sizeof(SolverInfo), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	DEALLOCATE(pairTable);
	DEALLOCATE(stateTable);
	DEALLOCATE(info);
}

void createJointPairs()
{
	PrefetchForwardIterator<SortData> itrPairs(
			&gPool,
			createJointPairsIO.pairsAddr,
			createJointPairsIO.pairsAddr + sizeof(SortData)*createJointPairsIO.batchJoint,
			PREFETCH_NUM, 10);

	ReadOnlyPrefetchForwardIterator<Joint> itrJoints(
			&gPool,
			createJointPairsIO.jointsAddr + sizeof(Joint)*createJointPairsIO.startJoint,
			createJointPairsIO.jointsAddr + sizeof(Joint)*(createJointPairsIO.startJoint + createJointPairsIO.batchJoint),
			PREFETCH_NUM>>2, 11);

	u32 stateIdxA = 0xffffffff;
	u32 stateIdxB = 0xffffffff;
	TrbState stateA, stateB;

	for(u32 i=0;i < createJointPairsIO.batchJoint;i++, ++itrJoints) {
		const Joint& joint = *itrJoints;
		SortData& pair = *itrPairs;

		if(joint.isActive()) {
			if(stateIdxA != joint.stateIndexA) {
				spu_dma_get(&stateA, createJointPairsIO.statesAddr + sizeof(TrbState)*joint.stateIndexA, sizeof(TrbState), 12, 0, 0);
				stateIdxA = joint.stateIndexA;
			}
			if(stateIdxB != joint.stateIndexB) {
				spu_dma_get(&stateB, createJointPairsIO.statesAddr + sizeof(TrbState)*joint.stateIndexB, sizeof(TrbState), 12, 0, 0);
				stateIdxB = joint.stateIndexB;
			}
			spu_dma_wait_tag_status_all(1<<12);

			setStateA(pair, joint.stateIndexA);
			setStateB(pair, joint.stateIndexB);
			setBodyA(pair, stateA.trbBodyIdx);
			setBodyB(pair, stateB.trbBodyIdx);
			setMovA(pair, stateA.moveType + stateA.sleeping);
			setMovB(pair, stateB.moveType + stateB.sleeping);
			setPair(pair, createJointPairsIO.startJoint + i);
			createJointPairsIO.numActiveJoints++;
			++itrPairs;
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

	switch(function) {
		case SOLVER_SPLIT_CONSTRAINTS:
		{
			spu_dma_get(&splitConstraintsIO, addrIo, sizeof(IOParamSplitConstraints), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			splitConstraints2();
		}
		break;

		case SOLVER_CREATE_JOINT_PAIRS:
		{
			spu_dma_get(&createJointPairsIO, addrIo, sizeof(IOParamCreateJointPairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			createJointPairs();

			spu_dma_put(&createJointPairsIO, addrIo, sizeof(IOParamCreateJointPairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
		}
		break;

		case SOLVER_CONSTRAINT_EX:
		{
			contactSolverInfo = (SolverInfo*)ALLOCATE(16, sizeof(SolverInfo));
			jointSolverInfo = (SolverInfo*)ALLOCATE(16, sizeof(SolverInfo));

			spu_dma_get(&solverIO, addrIo, sizeof(IOParamSolver), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(contactSolverInfo, solverIO.contactSolverInfoAddr, sizeof(SolverInfo), 0, 0, 0);
			spu_dma_get(jointSolverInfo, solverIO.jointSolverInfoAddr, sizeof(SolverInfo), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			// Pre Contact Impulse
			preContactImpulsePhaseEx();
			preJointImpulsePhaseEx();

			// Apply Contact Impulse
			u32 iteration = MAX(solverIO.numCollIteration, solverIO.numJointIteration);
			for(u32 i=0;i < iteration;i++) {
				if(LIKELY((iteration - i) <= solverIO.numCollIteration))
					applyContactImpulsePhaseEx();

				if(LIKELY((iteration - i) <= solverIO.numJointIteration))
					applyJointImpulsePhaseEx();
			}

			spu_dma_put(&solverIO, addrIo, sizeof(IOParamSolver), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			DEALLOCATE(jointSolverInfo);
			DEALLOCATE(contactSolverInfo);
		}
		break;

		case SOLVER_POSTRESPONSE:
		{
			spu_dma_get(&postResponseIO, addrIo, sizeof(IOParamPostResponse), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			postResponse();

			spu_dma_put(&postResponseIO, addrIo, sizeof(IOParamPostResponse), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
		}
		break;
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}
