/*
 * main.cpp
 *
 *  Created on: Jan 27, 2014
 *      Author: mike
 */
#include "base/common.h"
#include "base/sortcommon.h"
#include "base/heapmanager.h"
#include "base/prefetchiterator.h"

#include "particle/common/particleconfig.h"
#include "particle/common/particleio.h"
#include "particle/common/pclcontact.h"
#include "particle/common/pclstate.h"

#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/worldvolume.h"

#include "sort/parallelsort.h"

#include "base/testaabb.h"
#include "base/safedma.h"

#define PREFETCH_NUM					64

#define STATIC_MEM						0
#define DYNAMIC_MEM						(96*1024)
#define HEAP_BYTES						(STATIC_MEM + DYNAMIC_MEM)

#define ALLOCATE(align, size) 			gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16)
#define DEALLOCATE(ptr) 				if(ptr) {gPool.deallocate((void*)ptr); ptr = NULL;}

IOParamAssignStatesPcl assignStatesIOPcl;
IOParamDetectPairsPcl detectPairsIOPcl;
IOParamSortPcl sortIOPcl;
IOParamRefreshPairsPcl refreshPairsIOPcl;
IOParamIntegratePcl integrateIOPcl;

u32 taskId;
u32 barrier = 0;

// World Volume
WorldVolume worldVolume;

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

inline bool isCollidable(u32 selfA, u32 targetA, u32 selfB, u32 targetB)
{
	return ((selfA&targetB) && (targetA&selfB));
}

void refreshContactPairs()
{
	s32 numBatch = 10;
	s32 numBatchx4 = numBatch*4;
	s32 tag1 = 8;
	SortData *sortBuffer[2];
	PclContactPair *pairBuffer[2];
	spu_dma_list_element *pairDmaListBuf[3];

	sortBuffer[0] = (SortData*)ALLOCATE(128, sizeof(SortData)*numBatchx4);
	sortBuffer[1] = (SortData*)ALLOCATE(128, sizeof(SortData)*numBatchx4);
	pairBuffer[0] = (PclContactPair*)ALLOCATE(128, sizeof(PclContactPair)*numBatch);
	pairBuffer[1] = (PclContactPair*)ALLOCATE(128, sizeof(PclContactPair)*numBatch);

	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);

	memset(pairDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[2], 0, sizeof(spu_dma_list_element)*numBatch);

	u32 startSortAddr = refreshPairsIOPcl.sortsAddr + sizeof(SortData)*refreshPairsIOPcl.startBatch;
	u32 endEA = startSortAddr + sizeof(SortData)*refreshPairsIOPcl.numBatch;

	s32 sortDataBatchSize = sizeof(SortData)*numBatch*4;
	s32 batchCount = 0;
	s32 sortBatch = 0;
	s32 currSort = 0;
	s32 currData = 0;
	s32 numPairs[2] = {0};
	s32 numSorts[2] = {0};
	s32 j,n;

	for(j=0,n=0;j < numBatch;j++,n++) {
		pairDmaListBuf[0][n].size = sizeof(PclContactPair);
		pairDmaListBuf[1][n].size = sizeof(PclContactPair);
	}

	numSorts[0] = dmaGetBuffer(sortBuffer[0], startSortAddr, endEA, sortDataBatchSize, tag1);
	numSorts[0] /= sizeof(SortData);

	spu_dma_wait_tag_status_all(1<<tag1);

	for(j=0,n=0;j < numBatch && j < numSorts[0];j++,n++) {
		SortData& sort = sortBuffer[0][j];
		pairDmaListBuf[0][n].eal = refreshPairsIOPcl.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
	}
	numPairs[0] = n;

	if(numPairs[0] > 0)
		spu_dma_list_get(pairBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);

	spu_dma_wait_tag_status_all(1<<tag1);

	numSorts[1] = dmaGetBuffer(sortBuffer[1], startSortAddr+sortDataBatchSize, endEA, sortDataBatchSize, tag1);
	numSorts[1] /= sizeof(SortData);

	for(j=numBatch,n=0;j < numBatch+numBatch && j < numSorts[0];j++,n++) {
		SortData& sort = sortBuffer[0][j];
		pairDmaListBuf[1][n].eal = refreshPairsIOPcl.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
	}
	numPairs[1] = n;

	if(numPairs[1] > 0)
		spu_dma_list_get(pairBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);

	currSort = 0;
	currData = 0;

	for(s32 i=0;i < (int)refreshPairsIOPcl.numBatch;) {
		// UPDATE
		for(j=0;j < numPairs[currData];j++) {
			setFlag(sortBuffer[currSort][numBatch*batchCount+j], 0);
			if(pairBuffer[currData][j].numContacts == 0) {
				setKey(sortBuffer[currSort][numBatch*batchCount+j], NULL_KEY);
				refreshPairsIOPcl.numRemovedPairs++;
			}
		}
		i+=numBatch;

		// PUT & GET
		spu_dma_wait_tag_status_all(1<<tag1);

		if(numPairs[currData] > 0) {
			memcpy(pairDmaListBuf[2], pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData]);
			spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);
		}

		numPairs[currData] = 0;

		s32 start = (batchCount+2)%4*numBatch;
		s32 sortId = (batchCount == 2 || batchCount == 3) ? 1 - currSort : currSort;
		for(j=start,n=0;j < start + numBatch && j < numSorts[sortId];j++,n++) {
			SortData& sort = sortBuffer[sortId][j];
			pairDmaListBuf[currData][n].eal = refreshPairsIOPcl.contactPairsAddr + sizeof(PclContactPair)*getPair(sort);
		}
		numPairs[currData] = n;

		if(numPairs[currData] > 0)
			spu_dma_list_getf(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

		currData = 1 - currData;
		batchCount = (batchCount + 1)%4;

		if(batchCount == 0) {
			if(numSorts[currSort] > 0)
				dmaPutBuffer(sortBuffer[currSort], startSortAddr + sizeof(SortData)*(sortBatch*numBatchx4), endEA, sortDataBatchSize, tag1);

			sortBatch++;
			numSorts[currSort] = dmaGetBufferf(sortBuffer[currSort], startSortAddr + sizeof(SortData)*((sortBatch + 1)*numBatchx4), endEA, sortDataBatchSize, tag1);
			numSorts[currSort] /= sizeof(SortData);
			currSort = 1-currSort;
		}
	}

	if(numSorts[currSort] > 0)
		dmaPutBuffer(sortBuffer[currSort], startSortAddr + sizeof(SortData)*(sortBatch*numBatchx4), endEA, sortDataBatchSize, tag1);

	if(numPairs[currData] > 0)
		spu_dma_list_put(pairBuffer[currData], 0, pairDmaListBuf[currData], sizeof(spu_dma_list_element)*numPairs[currData], tag1, 0, 0);

	spu_dma_wait_tag_status_all(1<<tag1);

	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);

	DEALLOCATE(pairBuffer[1]);
	DEALLOCATE(pairBuffer[0]);
	DEALLOCATE(sortBuffer[1]);
	DEALLOCATE(sortBuffer[0]);
}

template< typename T >
inline T RungeKutta(const T &deriv, f32 dt)
{
	T	k0,k1,k2,k3;
	k0 = deriv*dt;
	k1 = (deriv + k0*0.5f)*dt;
	k2 = (deriv + k1*0.5f)*dt;
	k3 = (deriv + k2)*dt;
	return (k0 + k1*2.0f + k2*2.0f + k3)/6.0f;
}

void integrate()
{
	PrefetchForwardIterator<PclState> itrStates(
			&gPool,
			integrateIOPcl.statesAddr,
			integrateIOPcl.statesAddr + sizeof(PclState)*integrateIOPcl.numStates,
			PREFETCH_NUM, 11);

	for(u32 i=0;i < integrateIOPcl.numStates;i++,++itrStates) {
		PclState& particle = *itrStates;

		if(!particle.isActive())
			continue;

		Vector3 totalForce = integrateIOPcl.gravity*particle.getMass() + integrateIOPcl.extraForce;

#ifdef PARTICLE_VELOCITY_BASE
		PclState bodyDerivs;

		// compute derivs
		bodyDerivs.fX = particle.fV;
		bodyDerivs.fV = totalForce/particle.getMass();

		PclState nextState = particle;

		nextState.fX += RungeKutta(bodyDerivs.fX, integrateIOPcl.timeStep);
		nextState.fV += RungeKutta(bodyDerivs.fV, integrateIOPcl.timeStep);

		// apply damping
		nextState.fV *= integrateIOPcl.linearDamping;

		particle = nextState;
#else
		Vector3 oldX = particle.oldX;
		particle.oldX = particle.fX;

		particle.fX += (particle.fX - oldX)*integrateIOPcl.linearDamping + totalForce/particle.getMass() * integrateIOPcl.timeStep*integrateIOPcl.timeStep;
#endif
	}
}

void checkVariancePcl()
{
	PrefetchForwardIterator<PclState> itrState(
		&gPool,
		assignStatesIOPcl.statesAddr + sizeof(PclState)*assignStatesIOPcl.batchStartState,
		assignStatesIOPcl.statesAddr + sizeof(PclState)*(assignStatesIOPcl.batchStartState + assignStatesIOPcl.numBatchStates),
		PREFETCH_NUM, 13);

	Vector3 s(0.0f),s2(0.0f);

	for(u32 i=0;i < assignStatesIOPcl.numBatchStates;i++,++itrState) {
		PclState& particle = (*itrState);

		Vector3 pclCenter(particle.fX);
		s += pclCenter;
		s2 += mulPerElem(pclCenter, pclCenter);
	}

	assignStatesIOPcl.s = s;
	assignStatesIOPcl.s2 = s2;
}

void assignStatesPcl()
{
	PrefetchForwardIterator<SortData> itrAabb(
		&gPool,
		assignStatesIOPcl.aabbAddr[0],
		assignStatesIOPcl.aabbAddr[0] + sizeof(SortData)*assignStatesIOPcl.numStates,
		PREFETCH_NUM, 10);

	PrefetchForwardIterator<PclState> itrState(
		&gPool,
		assignStatesIOPcl.statesAddr + sizeof(PclState)*assignStatesIOPcl.batchStartState,
		assignStatesIOPcl.statesAddr + sizeof(PclState)*(assignStatesIOPcl.batchStartState + assignStatesIOPcl.numBatchStates),
		PREFETCH_NUM, 11);

	for(u32 i=0;i < assignStatesIOPcl.numBatchStates;i++,++itrState) {
		PclState& particle = (*itrState);

		if(!particle.isActive()) continue;

		// AABB
		Vector3 pclCenter(particle.getPosition());
		Vector3 pclHalf(particle.getRadius());
		Vector3 aabbMin = pclCenter - pclHalf;
		Vector3 aabbMax = pclCenter + pclHalf;

		Vector3 worldHalf(worldVolume.extent);
		Vector3 checkInWorld = absPerElem(pclCenter-worldVolume.origin) - (pclHalf + worldHalf);
		if(checkInWorld[0] > 0.0f || checkInWorld[1] > 0.0f || checkInWorld[2] > 0.0f) {
			particle.setActive(false);
			continue;
		}

		VecInt3 aabbMinL,aabbMaxL;
		worldVolume.worldToLocalPosition(aabbMin, aabbMax, aabbMinL, aabbMaxL);

		SortData& aabb = *itrAabb;
		setKey(aabb, aabbMinL.get(assignStatesIOPcl.chkAxis));
		setXMin(aabb, aabbMinL.getX());
		setXMax(aabb, aabbMaxL.getX());
		setYMin(aabb, aabbMinL.getY());
		setYMax(aabb, aabbMaxL.getY());
		setZMin(aabb, aabbMinL.getZ());
		setZMax(aabb, aabbMaxL.getZ());
		setStateId(aabb, assignStatesIOPcl.batchStartState + i);

		++itrAabb;
		assignStatesIOPcl.numAabb++;
	}
}

void assignStatesRig()
{
	PrefetchForwardIterator<SortData> itrAabbX(
		&gPool,
		assignStatesIOPcl.aabbAddr[0],
		assignStatesIOPcl.aabbAddr[0] + sizeof(SortData)*assignStatesIOPcl.numStates,
		PREFETCH_NUM, 10);

	PrefetchForwardIterator<SortData> itrAabbY(
		&gPool,
		assignStatesIOPcl.aabbAddr[1],
		assignStatesIOPcl.aabbAddr[1] + sizeof(SortData)*assignStatesIOPcl.numStates,
		PREFETCH_NUM, 11);

	PrefetchForwardIterator<SortData> itrAabbZ(
		&gPool,
		assignStatesIOPcl.aabbAddr[2],
		assignStatesIOPcl.aabbAddr[2] + sizeof(SortData)*assignStatesIOPcl.numStates,
		PREFETCH_NUM, 12);

	PrefetchForwardIterator<TrbState> itrState(
		&gPool,
		assignStatesIOPcl.statesAddr + sizeof(TrbState)*assignStatesIOPcl.batchStartState,
		assignStatesIOPcl.statesAddr + sizeof(TrbState)*(assignStatesIOPcl.batchStartState + assignStatesIOPcl.numBatchStates),
		PREFETCH_NUM, 13);

	for(u32 i=0;i < assignStatesIOPcl.numBatchStates;i++,++itrState) {
		TrbState& state = (*itrState);

		if(state.isDeleted()) continue;

		// AABB
		Vector3 stateCenter = read_Vector3(state.center);
		Vector3 stateHalf = read_Vector3(state.half);
		Vector3 aabbMin = stateCenter - stateHalf;
		Vector3 aabbMax = stateCenter + stateHalf;

		Vector3 worldHalf(worldVolume.extent);
		Vector3 checkInWorld = absPerElem(stateCenter - worldVolume.origin) - (stateHalf + worldHalf);
		if(checkInWorld[0] > 0.0f || checkInWorld[1] > 0.0f || checkInWorld[2] > 0.0f)
			continue;

		VecInt3 aabbMinL,aabbMaxL;
		worldVolume.worldToLocalPosition(aabbMin, aabbMax, aabbMinL, aabbMaxL);

		SortData& aabbX = *itrAabbX;
		setKey(aabbX, aabbMinL.getX());
		setXMin(aabbX, aabbMinL.getX());
		setXMax(aabbX, aabbMaxL.getX());
		setYMin(aabbX, aabbMinL.getY());
		setYMax(aabbX, aabbMaxL.getY());
		setZMin(aabbX, aabbMinL.getZ());
		setZMax(aabbX, aabbMaxL.getZ());
		setStateId(aabbX, assignStatesIOPcl.batchStartState + i);
		setBodyId(aabbX, state.trbBodyIdx);
		setMovType(aabbX, state.getMoveType());
		setSelf(aabbX, state.getContactFilterSelf());
		setTarget(aabbX, state.getContactFilterTarget());
		++itrAabbX;

		SortData& aabbY = *itrAabbY;
		setKey(aabbY, aabbMinL.getY());
		setXMin(aabbY, aabbMinL.getX());
		setXMax(aabbY, aabbMaxL.getX());
		setYMin(aabbY, aabbMinL.getY());
		setYMax(aabbY, aabbMaxL.getY());
		setZMin(aabbY, aabbMinL.getZ());
		setZMax(aabbY, aabbMaxL.getZ());
		setStateId(aabbY, assignStatesIOPcl.batchStartState + i);
		setBodyId(aabbY, state.trbBodyIdx);
		setMovType(aabbY, state.getMoveType());
		setSelf(aabbY, state.getContactFilterSelf());
		setTarget(aabbY, state.getContactFilterTarget());
		++itrAabbY;

		SortData& aabbZ = *itrAabbZ;
		setKey(aabbZ, aabbMinL.getZ());
		setXMin(aabbZ, aabbMinL.getX());
		setXMax(aabbZ, aabbMaxL.getX());
		setYMin(aabbZ, aabbMinL.getY());
		setYMax(aabbZ, aabbMaxL.getY());
		setZMin(aabbZ, aabbMinL.getZ());
		setZMax(aabbZ, aabbMaxL.getZ());
		setStateId(aabbZ, assignStatesIOPcl.batchStartState + i);
		setBodyId(aabbZ, state.trbBodyIdx);
		setMovType(aabbZ, state.getMoveType());
		setSelf(aabbZ, state.getContactFilterSelf());
		setTarget(aabbZ, state.getContactFilterTarget());
		++itrAabbZ;

		assignStatesIOPcl.numAabb++;
	}
}

void detectPairsPclSub(u16 stateIndexA, u16 aabbMaxA, SortData aabbA, u32 startAabbAddr, u32 numAabb, PrefetchForwardIterator<SortData>& itrPair)
{
	ReadOnlyPrefetchForwardIterator<SortData> itrAabb2(
		&gPool,
		startAabbAddr,
		startAabbAddr + sizeof(SortData)*numAabb,
		PREFETCH_NUM, 12);

	for(u32 i=0;i < numAabb;i++,++itrAabb2) {
		u16 stateIndexB,aabbMinB[3];
		SortData aabbB = *itrAabb2;

		stateIndexB = getStateId(aabbB);
		aabbMinB[0] = getXMin(aabbB);
		aabbMinB[1] = getYMin(aabbB);
		aabbMinB[2] = getZMin(aabbB);

		if(aabbMaxA < aabbMinB[detectPairsIOPcl.chkAxis])
			break;

		if(testAABB16(aabbA, aabbB) && detectPairsIOPcl.numTmpSorts < detectPairsIOPcl.maxSorts) {
			setStatePair(*itrPair, stateIndexA, stateIndexB);
			setFlag(*itrPair, 1);
			++itrPair;
			detectPairsIOPcl.numTmpSorts++;
		}
	}
}

void detectPairsPcl(u32 startBatch, u32 numBatch, PrefetchForwardIterator<SortData>& itrPair)
{
	const s32 tag1 = 8;
	SortData *aabbArray1 = (SortData*)ALLOCATE(128, sizeof(SortData)*numBatch);

	spu_dma_get(aabbArray1, detectPairsIOPcl.pclAabbAddr + sizeof(SortData)*startBatch, sizeof(SortData)*numBatch, tag1, 0, 0);
	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 i=0;i < numBatch;i++) {
		u16 stateIndexA,aabbMaxA[3];
		SortData aabbA = aabbArray1[i];

		stateIndexA = getStateId(aabbA);
		aabbMaxA[0] = getXMax(aabbA);
		aabbMaxA[1] = getYMax(aabbA);
		aabbMaxA[2] = getZMax(aabbA);

		u32 startB = startBatch + i + 1;
		u32 startBAddr = detectPairsIOPcl.pclAabbAddr + sizeof(SortData)*startB;
		u32 numB = detectPairsIOPcl.numPclAabb - startB;

		if(LIKELY(numB > 0))
			detectPairsPclSub(stateIndexA, aabbMaxA[detectPairsIOPcl.chkAxis], aabbA, startBAddr, numB, itrPair);
	}

	DEALLOCATE(aabbArray1);
}

void detectPairsRigSub(u16 stateIndexA, u16 aabbMaxA, SortData aabbA,u32 startAabbAddr, u32 numAabb, PrefetchForwardIterator<SortData>& itrPair)
{
	ReadOnlyPrefetchForwardIterator<SortData> itrAabb2(
		&gPool,
		startAabbAddr,
		startAabbAddr + sizeof(SortData)*numAabb,
		PREFETCH_NUM, 12);

	for(u32 i=0;i < numAabb;i++,++itrAabb2) {
		u16 stateIndexB,aabbMinB[3];
		SortData aabbB = *itrAabb2;

		stateIndexB = getStateId(aabbB);
		aabbMinB[0] = getXMin(aabbB);
		aabbMinB[1] = getYMin(aabbB);
		aabbMinB[2] = getZMin(aabbB);

		if(aabbMaxA < aabbMinB[detectPairsIOPcl.chkAxis])
			break;

		if(isCollidable(detectPairsIOPcl.contactFilterSelf, detectPairsIOPcl.contactFilterTarget, getSelf(aabbB), getTarget(aabbB)) && testAABB16(aabbA, aabbB) && detectPairsIOPcl.numTmpSorts < detectPairsIOPcl.maxSorts) {
			setStatePair2(*itrPair, stateIndexA, stateIndexB);
			setBodyB(*itrPair, getBodyId(aabbB));
			setFlag(*itrPair, 1);
			++itrPair;
			detectPairsIOPcl.numTmpSorts++;
		}
	}
}

void detectPairsRig(u32 startBatch, u32 numBatch, PrefetchForwardIterator<SortData>& itrPair)
{
	const s32 tag1 = 8;
	SortData *aabbArray1 = (SortData*)ALLOCATE(128, sizeof(SortData)*numBatch);

	spu_dma_get(aabbArray1,detectPairsIOPcl.pclAabbAddr + sizeof(SortData)*startBatch, sizeof(SortData)*numBatch, tag1, 0, 0);
	spu_dma_wait_tag_status_all(1<<tag1);

	for(u32 i=0;i < numBatch;i++) {
		u16 stateIndexA,aabbMaxA[3];
		SortData aabbA = aabbArray1[i];

		stateIndexA = getStateId(aabbA);
		aabbMaxA[0] = getXMax(aabbA);
		aabbMaxA[1] = getYMax(aabbA);
		aabbMaxA[2] = getZMax(aabbA);

		u32 startBAddr = detectPairsIOPcl.rigAabbAddr;
		u32 numB = detectPairsIOPcl.numRigAabb;

		if(LIKELY(numB > 0))
			detectPairsRigSub(stateIndexA, aabbMaxA[detectPairsIOPcl.chkAxis], aabbA, startBAddr, numB, itrPair);
	}

	DEALLOCATE(aabbArray1);
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
		case PARTICLE_CALCVARIANCE_PCL:
			spu_dma_get(&assignStatesIOPcl, addrIo, sizeof(IOParamAssignStatesPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			checkVariancePcl();

			spu_dma_put(&assignStatesIOPcl, addrIo, sizeof(IOParamAssignStatesPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case PARTICLE_ASSIGNSTATES_PCL:
			spu_dma_get(&assignStatesIOPcl, addrIo, sizeof(IOParamAssignStatesPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(&worldVolume, assignStatesIOPcl.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			assignStatesPcl();

			spu_dma_put(&assignStatesIOPcl, addrIo, sizeof(IOParamAssignStatesPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case PARTICLE_ASSIGNSTATES_RIG:
			spu_dma_get(&assignStatesIOPcl, addrIo, sizeof(IOParamAssignStatesPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(&worldVolume, assignStatesIOPcl.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			assignStatesRig();

			spu_dma_put(&assignStatesIOPcl, addrIo, sizeof(IOParamAssignStatesPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case PARTICLE_DETECTPAIRS_PCL:
		case PARTICLE_DETECTPAIRS_RIG:
			spu_dma_get(&detectPairsIOPcl, addrIo, sizeof(IOParamDetectPairsPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			{
				PrefetchForwardIterator<SortData> itrPair(
					&gPool,
					detectPairsIOPcl.tmpSortsAddr,
					detectPairsIOPcl.tmpSortsAddr + sizeof(SortData)*detectPairsIOPcl.maxSorts,
					PREFETCH_NUM, 10);

				u32 commonBuffAddr = taskbuff[2];
				{
					u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
					bool empty = false;
					while(!empty) {
						u32 startBatch;
						u32 numBatch;

						lock(commonBuff, commonBuffAddr);

						startBatch = commonBuff[1];
						numBatch = commonBuff[2];

						s32 nextStartBatch = startBatch + numBatch;
						s32 rest = MAX((s32)detectPairsIOPcl.numPclAabb - nextStartBatch, 0);
						s32 nextNumBatch = (rest > (s32)numBatch) ? (s32)numBatch : rest;

						commonBuff[1] = nextStartBatch;
						commonBuff[2] = nextNumBatch;

						unlock(commonBuff, commonBuffAddr);

						if(numBatch > 0) {
							if(function == PARTICLE_DETECTPAIRS_PCL)
								detectPairsPcl(startBatch, numBatch, itrPair);
							else
								detectPairsRig(startBatch, numBatch, itrPair);
						} else
							empty = true;
					}

					DEALLOCATE(commonBuff);
				}

			}

			spu_dma_put(&detectPairsIOPcl, addrIo, sizeof(IOParamDetectPairsPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case PARTICLE_REFRESHCONTACTPAIRS_PCL:
		case PARTICLE_REFRESHCONTACTPAIRS_RIG:
			spu_dma_get(&refreshPairsIOPcl, addrIo, sizeof(IOParamRefreshPairsPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			refreshContactPairs();
			sync();
			parallelsort((SortData*)refreshPairsIOPcl.sortsAddr, (SortData*)refreshPairsIOPcl.buffAddr, refreshPairsIOPcl.numContactPairs, taskId, refreshPairsIOPcl.numSpu);

			spu_dma_put(&refreshPairsIOPcl, addrIo, sizeof(IOParamRefreshPairsPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case PARTICLE_INTEGRATE:
			spu_dma_get(&integrateIOPcl, addrIo, sizeof(IOParamIntegratePcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			integrate();
			break;
		case PARTICLE_SORT:
			spu_dma_get(&sortIOPcl, addrIo, sizeof(IOParamSortPcl), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			parallelsort((SortData*)sortIOPcl.sortsAddr, (SortData*)sortIOPcl.buffAddr, sortIOPcl.numSorts, taskId, sortIOPcl.numSpu);
			break;
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}
