/*
 * main.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */
#include "base/common.h"
#include "base/simdfunc.h"
#include "base/heapmanager.h"
#include "base/prefetchiterator.h"
#include "base/safedma.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/rigidbodyio.h"
#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/contact.h"
#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/forces.h"
#include "rigidbody/common/worldvolume.h"
#include "rigidbody/common/collidabletable.h"
#include "rigidbody/common/rigidbodyconfig.h"

#include "base/testaabb.h"

#include "sort/parallelsort.h"

#define STATIC_MEM						0
#define DYNAMIC_MEM						(128*1024)
#define HEAP_BYTES 						(STATIC_MEM + DYNAMIC_MEM)

#define ALLOCATE(align, size) 			gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16)
#define DEALLOCATE(ptr) 				if(ptr) {gPool.deallocate((void*)ptr); ptr = NULL;}

#define PREFETCH_NUM					64
#define DETECTPAIR_PREFETCH_NUM			32
#define DETECTPAIR_STORE_NUM			2048

IOParamAssignStates assignStatesIO;
IOParamDetectPairs detectPairsIO;
IOParamMergePairs mergePairsIO;
IOParamRefreshPairs refreshPairsIO;
IOParamSort sortIO;
IOParamIntegrate integrateIO;
IOParamFindAabbOverlap findAabbOverlapIO;
IOParamAddNewPairs addNewPairsIO;
IOParamDetectSet detectSetIO;
IOParamMergeSet mergeSetIO;

u32 taskId;
u32 barrier = 0;

// World Volume
WorldVolume worldVolume;

ATTRIBUTE_ALIGNED128(u32 lockBuffer[32]);

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool, HEAP_BYTES);

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

qword lastNonContactPairIdx128_vec;
ATTRIBUTE_ALIGNED128(u32 nonContactPairVec[64]);

ATTRIBUTE_ALIGNED128(u32 nonContactPair[32]);
u32 lastNonContactPairIdx128;

inline bool isCollidablePair(u16 i, u16 j)
{
	u32 minIdx = i > j ? j : i;
	u32 maxIdx = i > j ? i : j;
	u32 idx = maxIdx*(maxIdx - 1)/2 + minIdx;

	u32 idx128 = idx>>10;
	u32 idx32  = (idx&1023)>>5;
	u32 idx1   = 1L<<(idx&31);

	if(idx128 != lastNonContactPairIdx128) {
		spu_dma_get(nonContactPair, detectPairsIO.nonContactPairAddr + 128*idx128, 128, 0, 0, 0);
		spu_dma_wait_tag_status_all(1);
		lastNonContactPairIdx128 = idx128;
	}
	return (nonContactPair[idx32]&idx1) == 0;
}

int removeDuplicateAndMergePairs()
{
	s32 numRemoved = 0;
	s32 dmaTag[] = {10, 11, 12, 13, 14};

	ReadOnlyPrefetchForwardIterator<SortData> itrOldSort(
		&gPool,
		(u32)mergePairsIO.oldSortsAddr,
		(u32)(mergePairsIO.oldSortsAddr + sizeof(SortData)*mergePairsIO.numOldSorts),
		PREFETCH_NUM, dmaTag[0]);

	PrefetchForwardIterator<SortData> itrNewSort(
		&gPool,
		(u32)mergePairsIO.newSortsAddr,
		(u32)(mergePairsIO.newSortsAddr + sizeof(SortData)*mergePairsIO.numNewSorts),
		PREFETCH_NUM, dmaTag[2]);

	s32 numPutSorts = 0;
	const s32 maxPutSorts = 50;
	SortData *putSorts = (SortData*)ALLOCATE(128, sizeof(SortData)*maxPutSorts);

	u32 asleepMask = ((1<<(MoveTypeActive + 1))|(1<<(MoveTypeOneWay + 1)));

	u32 oldIdx = 0, newIdx = 0;
	for(;newIdx < mergePairsIO.numNewSorts;newIdx++, ++itrNewSort) {
		u16 stateIndexA = getStateA(*itrNewSort);
		u16 stateIndexB = getStateB(*itrNewSort);

		if(1<<getMovA(*itrNewSort)&asleepMask) {
			TrbState stateA;
			spu_dma_get(&stateA, mergePairsIO.statesAddr + sizeof(TrbState)*stateIndexA, sizeof(TrbState), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			stateA.wakeup();
			spu_dma_put(&stateA, mergePairsIO.statesAddr + sizeof(TrbState)*stateIndexA, sizeof(TrbState), 0, 0, 0);
		}
		if(1<<getMovB(*itrNewSort)&asleepMask) {
			TrbState stateB;
			spu_dma_get(&stateB, mergePairsIO.statesAddr + sizeof(TrbState)*stateIndexB, sizeof(TrbState), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			stateB.wakeup();
			spu_dma_put(&stateB, mergePairsIO.statesAddr + sizeof(TrbState)*stateIndexB, sizeof(TrbState), 0, 0, 0);
		}
		spu_dma_wait_tag_status_all(1);

		bool loopOut = false;
		do {
			if(oldIdx >= mergePairsIO.numOldSorts)
				loopOut = true;
			else if(getKey(*itrNewSort) > getKey(*itrOldSort)) {
				oldIdx++;
				++itrOldSort;
			} else if(getKey(*itrNewSort) == getKey(*itrOldSort)) {
				putSorts[numPutSorts] = *itrOldSort;
				setFlag(putSorts[numPutSorts],1);
				spu_dma_put(&putSorts[numPutSorts], (u64)(mergePairsIO.oldSortsAddr + sizeof(SortData)*oldIdx), sizeof(SortData), dmaTag[4], 0, 0);
				setKey(*itrNewSort,NULL_KEY);
				oldIdx++;
				++itrOldSort;
				numPutSorts++;
				numRemoved++;
				if(numPutSorts == maxPutSorts) {
					spu_dma_wait_tag_status_all(1<<dmaTag[4]);
					numPutSorts = 0;
				}
				loopOut = true;
			} else
				loopOut = true;
		} while(!loopOut);
	}

	spu_dma_wait_tag_status_all(1<<dmaTag[4]);
	DEALLOCATE(putSorts);

	return numRemoved;
}

class CachedState
{
private:
	TrbState state;
	u32 stateAddr;

public:
	CachedState() : stateAddr(0) {}
	~CachedState() {put();}

	void set(u32 stateAddr_, TrbState &state_)
	{
		if(stateAddr != stateAddr_) {
			put();
			stateAddr = stateAddr_;
			state = state_;
		}
	}

	void put()
	{
		if(stateAddr) {
			spu_dma_put(&state, stateAddr, sizeof(TrbState), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
		}
	}
};

class RCPDblBuff
{
private:
	SortData *ptrs;
	s32 numPtrs;
	u32 pairAddr;
	u32 stateAddr;
	s32 numPairs[2];
	s32 numBatch;
	s32 index;
	s32 iptr;
	s32 tag1, tag2;
	s32 curr;

	spu_dma_list_element *pairDmaListBuf[3];
	spu_dma_list_element *stateDmaListBuf[3];
	ContactPair *pairBuffer[2];
	TrbState	*stateBuffer[2];

	CachedState cachedStateA, cachedStateB;

public:
	RCPDblBuff(SortData *ptrs_, s32 numPtrs_, u32 pairAddr_, u32 stateAddr_, s32 numBatch_, s32 tag1_, s32 tag2_);
	~RCPDblBuff();

	void update()
	{
		calc();
		index++;
		iptr++;
		if(UNLIKELY(index >= numBatch)) {
			wait();
			put();
			get();
			index = 0;
			curr = 1-curr;
		}
	}

	inline void calc();
	inline void wait();
	inline void get();
	inline void put();
};

RCPDblBuff::RCPDblBuff(SortData *ptrs_, s32 numPtrs_, u32 pairAddr_, u32 stateAddr_, s32 numBatch_, s32 tag1_, s32 tag2_)
: ptrs(ptrs_), numPtrs(numPtrs_), pairAddr(pairAddr_), stateAddr(stateAddr_), numBatch(numBatch_),
  index(0), iptr(0), tag1(tag1_), tag2(tag2_), curr(0)
{
	s32 numBatchx2 = numBatch<<1;

	// Allocate
	pairBuffer[0] = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numBatch);
	pairBuffer[1] = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numBatch);
	stateBuffer[0] = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numBatchx2);
	stateBuffer[1] = (TrbState*)ALLOCATE(128, sizeof(TrbState)*numBatchx2);
	pairDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	pairDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatch);
	stateDmaListBuf[0] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatchx2);
	stateDmaListBuf[1] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatchx2);
	stateDmaListBuf[2] = (spu_dma_list_element*)ALLOCATE(128, sizeof(spu_dma_list_element)*numBatchx2);
	memset(pairDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(pairDmaListBuf[2], 0, sizeof(spu_dma_list_element)*numBatch);
	memset(stateDmaListBuf[0], 0, sizeof(spu_dma_list_element)*numBatchx2);
	memset(stateDmaListBuf[1], 0, sizeof(spu_dma_list_element)*numBatchx2);
	for(s32 i=0;i < numBatch;i++) {
		pairDmaListBuf[0][i].size = sizeof(ContactPair);
		pairDmaListBuf[1][i].size = sizeof(ContactPair);
		pairDmaListBuf[2][i].size = sizeof(ContactPair);
		stateDmaListBuf[0][i*2 + 0].size = sizeof(TrbState);
		stateDmaListBuf[0][i*2 + 1].size = sizeof(TrbState);
		stateDmaListBuf[1][i*2 + 0].size = sizeof(TrbState);
		stateDmaListBuf[1][i*2 + 1].size = sizeof(TrbState);
		stateDmaListBuf[2][i*2 + 0].size = sizeof(TrbState);
		stateDmaListBuf[2][i*2 + 1].size = sizeof(TrbState);
	}

	// Initialize
	s32 j, n;
	for(j=0, n=0;n < numBatch && j < numPtrs;j++, n++) {
		SortData& ptr = ptrs[j];
		pairDmaListBuf[0][n].eal = pairAddr + sizeof(ContactPair)*getPair(ptr);
		stateDmaListBuf[0][n*2 + 0].eal = stateAddr + sizeof(TrbState)*getStateA(ptr);
		stateDmaListBuf[0][n*2 + 1].eal = stateAddr + sizeof(TrbState)*getStateB(ptr);
	}
	numPairs[0] = n;
	if(n > 0) {
		spu_dma_list_get(pairBuffer[0], 0, pairDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0], tag1, 0, 0);
		spu_dma_list_get(stateBuffer[0], 0, stateDmaListBuf[0], sizeof(spu_dma_list_element)*numPairs[0]*2, tag1, 0, 0);
	}
	wait();

	for(n=0;n < numBatch && j < numPtrs;j++, n++) {
		SortData &ptr = ptrs[j];
		pairDmaListBuf[1][n].eal = pairAddr + sizeof(ContactPair)*getPair(ptr);
		stateDmaListBuf[1][n*2 + 0].eal = stateAddr + sizeof(TrbState)*getStateA(ptr);
		stateDmaListBuf[1][n*2 + 1].eal = stateAddr + sizeof(TrbState)*getStateB(ptr);
	}

	numPairs[1] = n;
	if(n > 0) {
		spu_dma_list_get(pairBuffer[1], 0, pairDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1], tag1, 0, 0);
		spu_dma_list_get(stateBuffer[1], 0, stateDmaListBuf[1], sizeof(spu_dma_list_element)*numPairs[1]*2, tag1, 0, 0);
	}
}

RCPDblBuff::~RCPDblBuff()
{
	if(numPairs[curr] > 0)
		spu_dma_list_put(pairBuffer[curr], 0, pairDmaListBuf[curr], sizeof(spu_dma_list_element)*numPairs[curr], tag1, 0, 0);

	wait();

	DEALLOCATE(stateDmaListBuf[2]);
	DEALLOCATE(stateDmaListBuf[1]);
	DEALLOCATE(stateDmaListBuf[0]);
	DEALLOCATE(pairDmaListBuf[2]);
	DEALLOCATE(pairDmaListBuf[1]);
	DEALLOCATE(pairDmaListBuf[0]);
	DEALLOCATE(stateBuffer[1]);
	DEALLOCATE(stateBuffer[0]);
	DEALLOCATE(pairBuffer[1]);
	DEALLOCATE(pairBuffer[0]);
}

inline void RCPDblBuff::calc()
{
	SortData& ptr = ptrs[iptr];
	ContactPair& pair = pairBuffer[curr][index];
	s32 iA, iB;
	if(getStateA(ptr) == pair.stateIndex[0]) {
		iA = index*2;
		iB = index*2 + 1;
	} else {
		iA = index*2 + 1;
		iB = index*2;
	}
	TrbState& stateA = stateBuffer[curr][iA];
	TrbState& stateB = stateBuffer[curr][iB];

	const u32 asleepMask = ((1<<(MoveTypeActive + 1))|(1<<(MoveTypeOneWay + 1)));

	// Update pair info
	setFlag(ptr, 0);
	setMovA(ptr, stateA.moveType + stateA.sleeping);
	setMovB(ptr, stateB.moveType + stateB.sleeping);

	if(pair.numContacts == 0 ||
	   !((stateA.getContactFilterSelf()&stateB.getContactFilterTarget()) &&
	    (stateA.getContactFilterTarget()&stateB.getContactFilterSelf())) )
	{

		if(stateA.getMoveTypeBits()&asleepMask) {
			stateA.wakeup();
			cachedStateA.set((u32)stateDmaListBuf[curr][iA].eal, stateA);
		}
		if(stateB.getMoveTypeBits()&asleepMask) {
			stateB.wakeup();
			cachedStateB.set((u32)stateDmaListBuf[curr][iB].eal, stateB);
		}
		setKey(ptr, NULL_KEY);
		refreshPairsIO.numRemovedPairs++;
	}

	pair.refreshContactPoints(stateA.getPosition(), stateA.getOrientation(), stateB.getPosition(), stateB.getOrientation());
}

inline void RCPDblBuff::wait()
{
	spu_dma_wait_tag_status_all((1<<tag1)|(1<<tag2));
}

inline void RCPDblBuff::get()
{
	s32 j,n;
	for(j=iptr + numBatch, n=0;n < numBatch && j < numPtrs;j++, n++) {
		SortData &ptr = ptrs[j];
		pairDmaListBuf[curr][n].eal = pairAddr + sizeof(ContactPair)*getPair(ptr);
		stateDmaListBuf[curr][n*2 + 0].eal = stateAddr + sizeof(TrbState)*getStateA(ptr);
		stateDmaListBuf[curr][n*2 + 1].eal = stateAddr + sizeof(TrbState)*getStateB(ptr);
	}
	numPairs[curr] = n;
	if(n > 0) {
		spu_dma_list_getf(pairBuffer[curr], 0, pairDmaListBuf[curr], sizeof(spu_dma_list_element)*numPairs[curr], tag1, 0, 0);
		spu_dma_list_get(stateBuffer[curr], 0, stateDmaListBuf[curr], sizeof(spu_dma_list_element)*numPairs[curr]*2, tag1, 0, 0);
	}
}

inline void RCPDblBuff::put()
{
	if(numPairs[curr] > 0) {
		memcpy(pairDmaListBuf[2], pairDmaListBuf[curr], sizeof(spu_dma_list_element)*numPairs[curr]);
		spu_dma_list_put(pairBuffer[curr], 0, pairDmaListBuf[2], sizeof(spu_dma_list_element)*numPairs[curr], tag1, 0, 0);
	}
}

void refreshContactPairs()
{
	u32 sSortAddr = refreshPairsIO.sortsAddr + sizeof(SortData)*refreshPairsIO.startBatch;
	u32 eSortAddr = sSortAddr + sizeof(SortData)*refreshPairsIO.numBatch;
	u32 sortBatch = 64;
	u32 pairBatch = 16;

	PrefetchForwardIterator<SortData> itrSort(
			&gPool,
			sSortAddr,
			eSortAddr,
			sortBatch, 10);

	s32 batchCount = refreshPairsIO.numBatch/sortBatch;
	s32 rest = refreshPairsIO.numBatch;
	for(s32 i=0;i < batchCount;i++, rest-=sortBatch) {
		RCPDblBuff rcp(itrSort.getPtr(), sortBatch, refreshPairsIO.contactPairsAddr, refreshPairsIO.statesAddr, pairBatch, 11, 12);
		for(s32 j=0;j < (s32)sortBatch;j++, ++itrSort) {
			rcp.update();
		}
	}
	{
		RCPDblBuff rcp(itrSort.getPtr(), rest, refreshPairsIO.contactPairsAddr, refreshPairsIO.statesAddr, pairBatch, 11, 12);
		for(s32 j=0;j < rest;j++,++itrSort) {
			rcp.update();
		}
	}
}

template< typename T >
inline T RungeKutta(const T &deriv, f32 dt)
{
	T	k0, k1, k2, k3;
	k0 = deriv * dt;
	k1 = (deriv + k0*0.5f)*dt;
	k2 = (deriv + k1*0.5f)*dt;
	k3 = (deriv + k2)*dt;
	return (k0 + k1*2.0f + k2*2.0f + k3)/6.0f;
}

void integrate()
{
	s32 dmaTag[] = {10, 11, 12, 13, 14, 15, 16, 17};

	ReadOnlyPrefetchForwardIterator<Forces> itrForces(
			&gPool,
			integrateIO.forcesAddr,
			integrateIO.forcesAddr + sizeof(Forces)*integrateIO.numStates,
			PREFETCH_NUM, dmaTag[0]);

	PrefetchForwardIterator<TrbState> itrStates(
			&gPool,
			integrateIO.statesAddr,
			integrateIO.statesAddr + sizeof(TrbState)*integrateIO.numStates,
			PREFETCH_NUM, dmaTag[2]);

	PrefetchForwardIterator<TrbState> itrPrev(
			&gPool,
			integrateIO.prevAddr,
			integrateIO.prevAddr + sizeof(TrbState)*integrateIO.numStates,
			PREFETCH_NUM, dmaTag[4]);

	for(u32 i=0;i < integrateIO.numStates;i++, ++itrForces, ++itrPrev, ++itrStates) {
		TrbState& state = *itrStates;

		// copy last state
		*itrPrev = state;

		// check MoveType
		if(state.isAsleep() || state.getMoveTypeBits()&~MOVE_TYPE_CAN_SLEEP)
			continue;

		TrbDynBody trbBody;
		CollObject coll;
		spu_dma_get(&trbBody, integrateIO.bodiesAddr + sizeof(TrbDynBody)*state.trbBodyIdx, sizeof(TrbDynBody), dmaTag[6], 0, 0);
		spu_dma_get(&coll, integrateIO.collsAddr + sizeof(CollObject)*state.trbBodyIdx, sizeof(CollObject), dmaTag[7], 0, 0);
		spu_dma_wait_tag_status_all(1<<dmaTag[6]);

		if(state.getMoveType() == MoveTypeKeyframe) {
			Quat derivQ = Quat(state.getAngularVelocity(),0)*state.getOrientation()*0.5f;
			state.setPosition(state.getPosition() + state.getLinearVelocity()*integrateIO.timeStep);
			state.setOrientation(normalize(state.getOrientation() + derivQ*integrateIO.timeStep));
			if(integrateIO.lastSubStep) {
				state.setLinearVelocity(Vector3(0.0f));
				state.setAngularVelocity(Vector3(0.0f));
			}
		} else {
			Matrix3 fR(state.getOrientation());
			Matrix3 fI = fR*trbBody.getBodyInertia()*transpose(fR);
			Matrix3 fIInv = fR*trbBody.getBodyInertiaInv()*transpose(fR);

			// compute derivs
			Vector3 totalForce = (*itrForces).force + integrateIO.gravity*trbBody.getMass();
			Vector3 totalTorque = (*itrForces).torque;
			Vector3 dx = state.getLinearVelocity();
			Quat dq = Quat(state.getAngularVelocity(),0)*state.getOrientation()*0.5f;
			Vector3 dv = totalForce*trbBody.getMassInv();
			Vector3 dw = fIInv*(cross(fI*state.getAngularVelocity(), state.getAngularVelocity()) + totalTorque);

			Vector3 nx = state.getPosition();
			Quat nq = state.getOrientation();
			Vector3 nv = state.getLinearVelocity();
			Vector3 nw = state.getAngularVelocity();

			nx += RungeKutta(dx, integrateIO.timeStep);
			nq += RungeKutta(dq, integrateIO.timeStep);
			nq = normalize(nq);
			nv += RungeKutta(dv, integrateIO.timeStep);
			nw += RungeKutta(dw, integrateIO.timeStep);

			// apply damping
			nv *= state.linearDamping;
			nw *= state.angularDamping;

			state.setPosition(nx);
			state.setOrientation(nq);
			state.setLinearVelocity(nv);
			state.setAngularVelocity(nw);
		}

		spu_dma_wait_tag_status_all(1<<dmaTag[7]);

		// update AABB
		if(integrateIO.ccdEnable && state.getUseCcd())
			state.setAuxilsCcd(coll.getCenter(), coll.getHalf(), integrateIO.timeStep);
		else
			state.setAuxils(coll.getCenter(), coll.getHalf());
	}
}

void sleepOrWakeup()
{
	PrefetchForwardIterator<TrbState> itrStates(
			&gPool,
			integrateIO.statesAddr,
			integrateIO.statesAddr + sizeof(TrbState)*integrateIO.numStates,
			PREFETCH_NUM, 10);

	ReadOnlyPrefetchForwardIterator<TrbState> itrPrev(
			&gPool,
			integrateIO.prevAddr,
			integrateIO.prevAddr + sizeof(TrbState)*integrateIO.numStates,
			PREFETCH_NUM, 11);

	for(u32 i=0;i < integrateIO.numStates;i++, ++itrPrev, ++itrStates) {
		TrbState& state = *itrStates;
		const TrbState& prev = *itrPrev;

		f32 slvel = integrateIO.sleepLinearVelocity*integrateIO.sleepLinearVelocity;
		f32 savel = integrateIO.sleepAngularVelocity*integrateIO.sleepAngularVelocity;
		f32 wlvel = integrateIO.wakeLinearVelocity*integrateIO.wakeLinearVelocity;
		f32 wavel = integrateIO.wakeAngularVelocity*integrateIO.wakeAngularVelocity;

		// check Sleep or Wakeup
		if(integrateIO.sleepEnable) {
			if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP) {
				f32 dX = lengthSqr(state.getPosition() - prev.getPosition());
				f32 dQ = length(state.getOrientation() - prev.getOrientation());
				f32 lV = lengthSqr(state.getLinearVelocity());
				f32 lO = lengthSqr(state.getAngularVelocity());

				if(state.isAwake()) {
					if(lV < slvel && lO < savel && dX < slvel && dQ < integrateIO.sleepAngularVelocity)
						state.incrementSleepCount();
					else
						state.resetSleepCount();

					if((integrateIO.worldSleepCount%integrateIO.sleepInterval) == 0 && state.getSleepCount() > integrateIO.sleepCount)
						state.sleep();
				} else {
					if(lV > wlvel || lO > wavel || dX > wlvel || dQ > integrateIO.wakeAngularVelocity)
						state.wakeup();
				}
			}
		}
	}
}

void checkVariance()
{
	PrefetchForwardIterator<TrbState> itrState(
		&gPool,
		assignStatesIO.statesAddr + sizeof(TrbState)*assignStatesIO.batchStartState,
		assignStatesIO.statesAddr + sizeof(TrbState)*(assignStatesIO.batchStartState + assignStatesIO.numBatchStates),
		PREFETCH_NUM, 12);

	Vector3 s(0.0f), s2(0.0f);

	for(u32 i=0;i < assignStatesIO.numBatchStates;i++, ++itrState) {
		TrbState& state = (*itrState);

		Vector3 stateCenter(state.center[0], state.center[1], state.center[2]);
		s += stateCenter;
		s2 += mulPerElem(stateCenter, stateCenter);
	}

	assignStatesIO.s = s;
	assignStatesIO.s2 = s2;
}

void assignStates()
{
	PrefetchForwardIterator<SortData> itrMovAabb(
		&gPool,
		assignStatesIO.movAabbAddr,
		assignStatesIO.movAabbAddr + sizeof(SortData)*assignStatesIO.numStates,
		PREFETCH_NUM, 10);

	PrefetchForwardIterator<SortData> itrFixAabb(
		&gPool,
		assignStatesIO.fixAabbAddr,
		assignStatesIO.fixAabbAddr + sizeof(SortData)*assignStatesIO.numStates,
		PREFETCH_NUM, 11);

	PrefetchForwardIterator<TrbState> itrState(
		&gPool,
		assignStatesIO.statesAddr + sizeof(TrbState)*assignStatesIO.batchStartState,
		assignStatesIO.statesAddr + sizeof(TrbState)*(assignStatesIO.batchStartState + assignStatesIO.numBatchStates),
		PREFETCH_NUM, 12);

	for(u32 i=0;i < assignStatesIO.numBatchStates;i++, ++itrState) {
		TrbState& state = (*itrState);

		if(state.isDeleted()) continue;

		// AABB
		Vector3 stateCenter = read_Vector3(state.center);
		Vector3 stateHalf = read_Vector3(state.half);
		Vector3 aabbMin = stateCenter - stateHalf;
		Vector3 aabbMax = stateCenter + stateHalf;

		Vector3 worldHalf(worldVolume.extent);
		Vector3 checkInWorld = absPerElem(stateCenter - worldVolume.origin) - (stateHalf + worldHalf);
		if(checkInWorld[0] > 0.0f || checkInWorld[1] > 0.0f || checkInWorld[2] > 0.0f) {
			if(state.getMoveTypeBits()&MOVE_TYPE_DYNAMIC) {
				state.setLinearVelocity(Vector3(0.0f));
				state.setAngularVelocity(Vector3(0.0f));
				state.sleep();
			}
			continue;
		}

		VecInt3 aabbMinL, aabbMaxL;
		worldVolume.worldToLocalPosition(aabbMin, aabbMax, aabbMinL, aabbMaxL);

		SortData &aabb = (state.getMoveType() == MoveTypeFixed || state.isAsleep()) ? *itrFixAabb : *itrMovAabb;
		setKey(aabb, aabbMinL.get(assignStatesIO.chkAxis));
		setXMin(aabb, aabbMinL.getX());
		setXMax(aabb, aabbMaxL.getX());
		setYMin(aabb, aabbMinL.getY());
		setYMax(aabb, aabbMaxL.getY());
		setZMin(aabb, aabbMinL.getZ());
		setZMax(aabb, aabbMaxL.getZ());
		setStateId(aabb, assignStatesIO.batchStartState + i);
		setBodyId(aabb, state.trbBodyIdx);
		setMovType(aabb, state.moveType + state.sleeping); // set index for func table
		setSelf(aabb, state.getContactFilterSelf());
		setTarget(aabb, state.getContactFilterTarget());
		setCallback(aabb, state.getUseContactCallback());

		if(state.getMoveType() == MoveTypeFixed || state.isAsleep()) {
			++itrFixAabb;
			assignStatesIO.numFixAabb++;
		} else {
			++itrMovAabb;
			assignStatesIO.numMovAabb++;
		}
	}
}

inline bool checkCollidable(SortData& aabbA, u16 stateA, u16 movA, u32 slfA, u32 tgtA, SortData& aabbB, u16 stateB, u16 movB, u32 slfB, u32 tgtB)
{
	return  ((slfA&tgtB) && (tgtA&slfB)) && testAABB16(aabbA, aabbB) && collidableTable[movA][movB] && isCollidablePair(stateA, stateB);
}

void detectPairsSub(u16 aabbMaxA, SortData aabbA, u32 startAabbAddr, u32 numAabb, PrefetchForwardIterator<SortData>& itrPair)
{
	ReadOnlyPrefetchForwardIterator<SortData> itrAabb2(
		&gPool,
		startAabbAddr,
		startAabbAddr + sizeof(SortData)*numAabb,
		DETECTPAIR_PREFETCH_NUM, 12);

	u32 slfA, tgtA;
	u16 stateIndexA, movA;

	stateIndexA = getStateId(aabbA);
	movA = getMovType(aabbA);
	slfA = getSelf(aabbA);
	tgtA = getTarget(aabbA);

	for(u32 i=0;i < numAabb;i++, ++itrAabb2) {
		u32 slfB,tgtB;
		u16 stateIndexB,movB;
		SortData aabbB = *itrAabb2;

		u16 aabbMinB = getXYZMin(aabbB, detectPairsIO.chkAxis);

		if(UNLIKELY(aabbMaxA < aabbMinB))
			break;

		stateIndexB = getStateId(aabbB);
		movB = getMovType(aabbB);
		slfB = getSelf(aabbB);
		tgtB = getTarget(aabbB);

		if(checkCollidable(aabbA, stateIndexA, movA, slfA, tgtA, aabbB, stateIndexB, movB, slfB, tgtB)) {
			setStatePair(*itrPair, stateIndexA, stateIndexB);
			setBodyA(*itrPair, getBodyId(aabbA));
			setBodyB(*itrPair, getBodyId(aabbB));
			setMovA(*itrPair, movA);
			setMovB(*itrPair, movB);
			setFlag(*itrPair, 1);
			setCallbackFlag(*itrPair, getCallback(aabbA)|getCallback(aabbB));
			++itrPair;
			detectPairsIO.numTmpSorts++;
		}
	}
}

void detectPairsSubLocal(u16 aabbMaxA, SortData aabbA, SortData *localAabbB, u32 numAabb, PrefetchForwardIterator<SortData>& itrPair)
{
	const qword cZero		= si_il(0);
	const qword cOne		= si_il(1);
	const qword c0B000B00mask = ((qword)(vec_uchar16){0x80, 0x80, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80});
	const qword c0A000a00mask = ((qword)(vec_uchar16){0x80, 0x80, 0x00, 0x01, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x10, 0x11, 0x80, 0x80, 0x80, 0x80});
	const qword cSASSSaSSmask = ((qword)(vec_uchar16){0xE0, 0xE0, 0xE0, 0x01, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0x11, 0xE0, 0xE0, 0xE0, 0xE0});
	const qword c0G0a0G0aMask = ((qword)(vec_uchar16){0x80, 0x80, 0x0C, 0x0D, 0x80, 0x80, 0x10, 0x11, 0x80, 0x80, 0x0C, 0x0D, 0x80, 0x80, 0x10, 0x11}); // for A and B

	const qword c00B0h0D0mask = ((qword)(vec_uchar16){0x80, 0x80, 0x80, 0x80, 0x02, 0x03, 0x80, 0x80, 0x1E, 0x1F, 0x80, 0x80, 0x06, 0x07, 0x80, 0x80});
	const qword c000B0h0Dmask = ((qword)(vec_uchar16){0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x02, 0x03, 0x80, 0x80, 0x1E, 0x1F, 0x80, 0x80, 0x06, 0x07});
	const qword c000F0h0Hmask = ((qword)(vec_uchar16){0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x0A, 0x0B, 0x80, 0x80, 0x1E, 0x1F, 0x80, 0x80, 0x0E, 0x0F});
	const qword cbB000000mask = ((qword)(vec_uchar16){0x12, 0x13, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80});
	const qword c0B0000abmask = ((qword)(vec_uchar16){0x80, 0x80, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x10, 0x11, 0x12, 0x13});
	const qword c0B0000efmask = ((qword)(vec_uchar16){0x80, 0x80, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x18, 0x19, 0x1A, 0x1B});
	const qword cabCDEFghmask = ((qword)(vec_uchar16){0x10, 0x11, 0x12, 0x13, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x1C, 0x1D, 0x1E, 0x1F});

	const qword cF000mask = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
	const qword cF0F0mask = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00});
	const qword cYZYZmask = ((qword)(vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B}); // for A {self tag}
	const qword cZYzymask = ((qword)(vec_uchar16){0x08, 0x09, 0x0A, 0x0B, 0x04, 0x05, 0x06, 0x07, 0x18, 0x19, 0x1A, 0x1B, 0x14, 0x15, 0x16, 0x17}); // for B {tag self}
	const qword cX0X0mask = ((qword)(vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80, 0x00, 0x01, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80});
	const qword cX0x0mask = ((qword)(vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80, 0x10, 0x11, 0x12, 0x13, 0x80, 0x80, 0x80, 0x80});
	const qword cXXXXmask = ((qword)(vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03});
	const qword cXyzwMask = ((qword)(vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F});

	const qword cSelMaskTt = ((qword)(vec_uchar16){0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});

	const qword cAABBTest_ptn_mask0 = ((qword)(vec_uchar16){0x02, 0x03, 0x12, 0x13, 0x06, 0x07, 0x16, 0x17, 0x0A, 0x0B, 0x1A, 0x1B, 0x80, 0x80, 0x80, 0x80});
	const qword cAABBTest_ptn_mask1 = ((qword)(vec_uchar16){0x10, 0x11, 0x00, 0x01, 0x14, 0x15, 0x04, 0x05, 0x18, 0x19, 0x08, 0x09, 0x80, 0x80, 0x80, 0x80});

	qword vec_numAabb = si_shufb(si_from_uint(numAabb), cZero, cXXXXmask);
	qword bigLoopCount = si_rotmi(si_ai(vec_numAabb, 1), -1);
	qword retMaskForLastLoop = si_shlqby(cF0F0mask, si_shli(si_andi(vec_numAabb, 0x1), 3)); // 8byte Left Shift

	qword v_aabbMaxA = si_shufb(si_from_ushort(aabbMaxA), si_from_ushort(aabbMaxA), c0B000B00mask);

	qword aabbA_0 = (qword)aabbA.vdata[0];
	qword aabbA_1 = (qword)aabbA.vdata[1];
	qword aabbA_sIdx_move = si_shufb(aabbA_0, aabbA_1, c0G0a0G0aMask);
	qword aabbA_slf_tag = si_shufb(aabbA_1, aabbA_1, cYZYZmask);

	qword localAabbBBase = si_and(si_from_ptr(localAabbB), cF000mask);

	// 	detectPairsIO.chkAxis*4
	qword chkAxis_m4 = si_lqd(si_from_ptr(&detectPairsIO), 0x10);
	chkAxis_m4 = si_shli(si_rotqbyi( chkAxis_m4, 12), 2);

	// collidableTable_movA
	qword collidableTable_movA = si_lqx(si_from_ptr(collidableTable), si_rotmi(aabbA_1, -13));
	collidableTable_movA = si_rotqby(collidableTable_movA, si_andi( si_rotmi(aabbA_1, -13), 0x08));

	// Cache Control
	qword nonContactPairVecBase_0 = si_from_ptr(nonContactPairVec);
	nonContactPairVecBase_0 = si_shufb(nonContactPairVecBase_0, nonContactPairVecBase_0, cXXXXmask);
	qword nonContactPairVecBase_1 = si_ai(nonContactPairVecBase_0, 128);
	qword cacheSlotDefaultAllMiss = si_shufb(nonContactPairVecBase_0, nonContactPairVecBase_1, cX0x0mask);

	// Output from A Side
	qword ret_sortData_0_A = si_shufb(aabbA_sIdx_move, aabbA_0, c00B0h0D0mask);
	qword ret_sortData_1_A = si_shufb(aabbA_1, cOne, cbB000000mask);

	qword loopEndFlag = si_ceqi(bigLoopCount, 0);
	qword retMask = cF0F0mask;

	qword collidableRet_Predict = cZero;
	qword isCollidablePair_idx = cZero;
	qword isDmaIssued = cZero;
	qword cacheHitSlot = si_il(0xFF);
	qword checkCollidable_ret_final = cZero;
	qword ret_sortData_0_0 = cZero;
	qword ret_sortData_0_1 = cZero;
	qword ret_sortData_1_0 = cZero;
	qword ret_sortData_1_1 = cZero;

	bool resultLeft = false;

	do{
		bigLoopCount = si_ai(bigLoopCount, -1);
		{
			loopEndFlag = si_ceqi(bigLoopCount, 0);
			retMask = si_selb(retMask, retMaskForLastLoop, loopEndFlag);
		}
		// Load 2 aabbB
		qword aabbB_0_0 = si_lqd(localAabbBBase, 0);
		qword aabbB_1_0 = si_lqd(localAabbBBase, 0x20);

		qword aabbMinB_0_1 = si_shufb(si_rotqby( aabbB_0_0, chkAxis_m4), si_rotqby( aabbB_1_0, chkAxis_m4), c0A000a00mask);

		qword aabbB_0_1 = si_lqd(localAabbBBase, 0x10);
		qword aabbB_1_1 = si_lqd(localAabbBBase, 0x30);

		localAabbBBase = si_ai(localAabbBBase, 0x40);


		qword min_max_cmp_ret = si_and(si_clgt(aabbMinB_0_1, v_aabbMaxA), retMask); // B>A -> 1
		if(UNLIKELY(si_to_uint(si_gb(min_max_cmp_ret))) ) {
			retMask = cF000mask;
			retMask = cF000mask;
			loopEndFlag = cF000mask;
			if(UNLIKELY(si_to_uint(min_max_cmp_ret)))
				break;
		}

		qword aabbB_sIdx_move = si_selb(si_shufb(aabbB_0_0, aabbB_0_1, c0G0a0G0aMask), si_shufb(aabbB_1_0, aabbB_1_1, c0G0a0G0aMask), cSelMaskTt);
		qword aabbB_tag_slf   = si_shufb(aabbB_0_1, aabbB_1_1, cZYzymask);


		qword checkCollidable_ret = si_clgti(si_and(aabbA_slf_tag, aabbB_tag_slf ), 0x00);
		checkCollidable_ret = si_and(checkCollidable_ret, si_rotqbyi(checkCollidable_ret, 4));

		// testAABB16
		{
			qword vecMin_0 = si_shufb(aabbA_0, aabbB_0_0, cAABBTest_ptn_mask1);
			qword vecMax_0 = si_shufb(aabbA_0, aabbB_0_0, cAABBTest_ptn_mask0);
			qword vecMin_1 = si_shufb(aabbA_0, aabbB_1_0, cAABBTest_ptn_mask1);
			qword vecMax_1 = si_shufb(aabbA_0, aabbB_1_0, cAABBTest_ptn_mask0);
			qword isGt_0 = si_and(si_ceqi(si_gbh(si_clgth(vecMin_0, vecMax_0)), 0), cF000mask);
			qword isGt_1 = si_and(si_ceqi(si_gbh(si_clgth(vecMin_1, vecMax_1)), 0), cF000mask);
			checkCollidable_ret = si_and(checkCollidable_ret, si_or(isGt_0, si_rotqbyi(isGt_1, 8)));
		}


		// collidableTable[movA][movB]
		{
			qword mov_pattern = si_shufb(aabbB_0_1, aabbB_1_1, cSASSSaSSmask);
			checkCollidable_ret = si_and(checkCollidable_ret, si_shufb(collidableTable_movA, collidableTable_movA, mov_pattern));
			checkCollidable_ret = si_ceqi(checkCollidable_ret, 0x1);
		}

		checkCollidable_ret = si_and(checkCollidable_ret, retMask);
		qword earlyContinue = si_ceqi(si_gbh(checkCollidable_ret), 0x00 );

		if(UNLIKELY((si_to_uint(earlyContinue))))
			continue;

		// isCollidablePair前半
		{
			qword idx32 = si_andi(si_rotmi(si_and(isCollidablePair_idx, si_il(1023)), -3), -4);
			qword idx1 = si_shl(si_il(1), si_andi(isCollidablePair_idx, 31));

			if(__builtin_expect((si_to_uint(isDmaIssued)), (si_to_uint(isDmaIssued))))
				spu_dma_wait_tag_status_all(1);

			// TODO:: FIXME!!
			// Better SIMD?
			cacheHitSlot = si_a(cacheHitSlot, idx32);
			qword collidableRet = si_shufb(si_from_uint(*(u32*)si_to_ptr(cacheHitSlot)), si_from_uint(*(u32*)si_to_ptr(si_rotqbyi(cacheHitSlot, 8))), cX0x0mask);
			collidableRet = si_and(collidableRet, idx1);
			collidableRet = si_ceqi(collidableRet, 0x00);
			checkCollidable_ret_final = si_and(checkCollidable_ret_final, collidableRet);
		}

		// TODO:: Take the computation out and interleave with others?
		if(__builtin_expect((si_to_uint(checkCollidable_ret_final)), (si_to_uint(collidableRet_Predict)))){
			qword pairPtr = si_from_ptr(itrPair.getPtr());
			qword pair_0 = si_lqd(pairPtr, 0);
			qword pair_1 = si_lqd(pairPtr, 16);
			pair_0 = si_shufb(pair_0, ret_sortData_0_0, cXyzwMask);
			si_stqd(pair_0, pairPtr, 0);

			pair_1 = si_shufb(pair_1, ret_sortData_0_1, cabCDEFghmask);
			si_stqd( pair_1, pairPtr, 16);
			++itrPair;
			detectPairsIO.numTmpSorts++;

		}
		if(__builtin_expect((si_to_uint(si_rotqbyi(checkCollidable_ret_final, 8))), (si_to_uint(si_rotqbyi(collidableRet_Predict, 8))))){
			qword pairPtr = si_from_ptr(itrPair.getPtr());
			qword pair_0 = si_lqd(pairPtr, 0);
			qword pair_1 = si_lqd(pairPtr, 16);
			pair_0 = si_shufb(pair_0, ret_sortData_1_0, cXyzwMask);
			si_stqd(pair_0, pairPtr, 0);

			pair_1 = si_shufb(pair_1, ret_sortData_1_1, cabCDEFghmask);
			si_stqd(pair_1, pairPtr, 16);
			++itrPair;
			detectPairsIO.numTmpSorts++;
		}

		collidableRet_Predict = cZero;
		isCollidablePair_idx = cZero;
		isDmaIssued = cZero;
		cacheHitSlot = si_il(0xFF);
		checkCollidable_ret_final = cZero;
		resultLeft = false;



		ret_sortData_0_0 = si_or(ret_sortData_0_A, si_shufb(aabbB_sIdx_move, aabbB_0_0, c000B0h0Dmask));
		ret_sortData_1_0 = si_or(ret_sortData_0_A, si_shufb(aabbB_sIdx_move, aabbB_1_0, c000F0h0Hmask));

		// isCollidablePair
		{
			qword minMaxMask = si_clgt(aabbA_sIdx_move, aabbB_sIdx_move); // a>b -> 1
			qword minIdx = si_selb(aabbA_sIdx_move, aabbB_sIdx_move, minMaxMask);
			qword maxIdx = si_selb(aabbB_sIdx_move, aabbA_sIdx_move, minMaxMask);

			// u32 idx = maxIdx * (maxIdx - 1) / 2 + minIdx;
			qword idx = si_a(si_rotmi(si_mpyu( maxIdx, si_ai(maxIdx, -1)), -1), minIdx);
			idx = si_and(idx, cF0F0mask);

			qword idx128 = si_rotmi(idx, -10);
			qword lastNonContactPairIdx128_0 = si_shufb(lastNonContactPairIdx128_vec, lastNonContactPairIdx128_vec, cXXXXmask);
			qword lastNonContactPairIdx128_1 = si_rotqbyi(lastNonContactPairIdx128_vec, 8);
			lastNonContactPairIdx128_1 = si_shufb(lastNonContactPairIdx128_1, lastNonContactPairIdx128_1, cXXXXmask);
			qword sizeMask_0 = si_ceq(idx128, lastNonContactPairIdx128_0); // idx128==lastNonContactPairIdx128_vec -> 1
			qword sizeMask_1 = si_ceq(idx128, lastNonContactPairIdx128_1); // idx128==lastNonContactPairIdx128_vec -> 1

			// Check which slot we should use
			cacheHitSlot = si_selb(cacheHitSlot, nonContactPairVecBase_1, sizeMask_1);
			cacheHitSlot = si_selb(cacheHitSlot, nonContactPairVecBase_0, sizeMask_0);

			// Check Final Cache hit
			qword cacheRet = si_gb(si_and(si_and(si_ceqi(cacheHitSlot, 0xFF), cF0F0mask), checkCollidable_ret)); // Missed -> 1
			isDmaIssued = cacheRet;

			if(UNLIKELY(si_to_uint(cacheRet))){
				// TODO:: Only Fetch what we needed?
				// 1010 (0xA) : ALL MISS
				// 1000 (0x8): 1st miss, 2ed hit
				// 0010 (0x2): 1st hit, 2ed miss
				// 0000 (0x0): ALL HIT
				qword nonCPAddr = si_lqd(si_from_ptr(&detectPairsIO.nonContactPairAddr), 0);
				nonCPAddr = si_shufb(nonCPAddr, nonCPAddr, cX0X0mask);
				nonCPAddr = si_a(nonCPAddr, si_shli(idx128, 7) );
				spu_dma_get(nonContactPairVec, si_to_uint(nonCPAddr), 128, 0, 0, 0);
				spu_dma_get(&nonContactPairVec[32], si_to_uint(si_rotqbyi(nonCPAddr, 8)), 128, 0, 0, 0);
				cacheHitSlot = cacheSlotDefaultAllMiss;
				lastNonContactPairIdx128_vec = idx128;
			}
			isCollidablePair_idx = idx;
		}

		ret_sortData_0_1 = si_or(ret_sortData_1_A, si_shufb(aabbB_0_1, isCollidablePair_idx, c0B0000abmask));
		ret_sortData_1_1 = si_or(ret_sortData_1_A, si_shufb(aabbB_1_1, isCollidablePair_idx, c0B0000efmask));

		collidableRet_Predict = checkCollidable_ret;
		checkCollidable_ret_final = checkCollidable_ret;

		resultLeft = true;

	} while(LIKELY(si_to_uint(loopEndFlag) == 0));

	// isCollidablePair
	if(resultLeft) {
		{
			qword idx32 = si_andi(si_rotmi(si_and(isCollidablePair_idx, si_il(1023)), -3), -4);
			qword idx1 = si_shl(si_il(1), si_andi(isCollidablePair_idx, 31));

			if(__builtin_expect((si_to_uint(isDmaIssued)), (si_to_uint(isDmaIssued))))
				spu_dma_wait_tag_status_all(1);

			// TODO:: FIXME!!
			// Better SIMD?
			cacheHitSlot = si_a(cacheHitSlot, idx32);
			qword cpllidableRet = si_shufb(si_from_uint(*(u32*)si_to_ptr(cacheHitSlot)), si_from_uint(*(u32*)si_to_ptr(si_rotqbyi(cacheHitSlot, 8))), cX0x0mask);
			cpllidableRet = si_and(cpllidableRet, idx1);
			cpllidableRet = si_ceqi(cpllidableRet, 0x00);
			checkCollidable_ret_final = si_and(checkCollidable_ret_final, cpllidableRet);
		}

		// TODO:: Take the computation out and interleave with others?
		if(__builtin_expect((si_to_uint(checkCollidable_ret_final)), (si_to_uint(collidableRet_Predict)))) {
			qword pairPtr = si_from_ptr(itrPair.getPtr());
			qword pair_0 = si_lqd(pairPtr, 0);
			qword pair_1 = si_lqd(pairPtr, 16);
			pair_0 = si_shufb(pair_0, ret_sortData_0_0, cXyzwMask);
			si_stqd(pair_0, pairPtr, 0);

			pair_1 = si_shufb(pair_1, ret_sortData_0_1, cabCDEFghmask);
			si_stqd(pair_1, pairPtr, 16);
			++itrPair;
			detectPairsIO.numTmpSorts++;

		}
		if(__builtin_expect((si_to_uint(si_rotqbyi(checkCollidable_ret_final, 8))), (si_to_uint(si_rotqbyi(collidableRet_Predict, 8))))) {
			qword pairPtr = si_from_ptr(itrPair.getPtr() );
			qword pair_0 = si_lqd(pairPtr, 0);
			qword pair_1 = si_lqd(pairPtr, 16);
			pair_0 = si_shufb(pair_0, ret_sortData_1_0, cXyzwMask);
			si_stqd(pair_0, pairPtr, 0);

			pair_1 = si_shufb(pair_1, ret_sortData_1_1, cabCDEFghmask);
			si_stqd(pair_1, pairPtr, 16);
			++itrPair;
			detectPairsIO.numTmpSorts++;
		}
	}
}

void detectPairsMov(u32 startBatch, u32 numBatch, PrefetchForwardIterator<SortData>& itrPair)
{
	lastNonContactPairIdx128 = 0xffffffff;
	lastNonContactPairIdx128_vec = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});

	const s32 tag1 = 8;

	ReadOnlyPrefetchForwardIterator<SortData> itrAabbA(
		&gPool,
		detectPairsIO.movAabbAddr + sizeof(SortData)*startBatch,
		detectPairsIO.movAabbAddr + sizeof(SortData)*(startBatch + numBatch),
		DETECTPAIR_PREFETCH_NUM, 11);

	SortData *aabbArrayB = (SortData*)ALLOCATE(128, sizeof(SortData)*DETECTPAIR_STORE_NUM);
	bool storeB = false;
	u32 storeStartB = 0;

	for(u32 i=0;i < numBatch;i++, ++itrAabbA) {
		SortData aabbA = *itrAabbA;
		u16 aabbMaxA = getXYZMax(aabbA, detectPairsIO.chkAxis);

		u32 startB = startBatch + i + 1;
		u32 startBAddr = detectPairsIO.movAabbAddr + sizeof(SortData)*startB;
		s32 numB = (s32)detectPairsIO.numMovAabb - (s32)startB;

		if(LIKELY(numB > 0)) {
			if(numB > DETECTPAIR_STORE_NUM)
				detectPairsSub(aabbMaxA, aabbA, startBAddr, (u32)numB, itrPair);
			else {
				if(!storeB) {
					storeB = true;
					spu_dma_large_get(aabbArrayB, startBAddr, sizeof(SortData)*numB, tag1, 0, 0);
					spu_dma_wait_tag_status_all(1<<tag1);
				}
				detectPairsSubLocal(aabbMaxA, aabbA, aabbArrayB + storeStartB, (u32)numB, itrPair);
				storeStartB++;
			}
		}
	}

	DEALLOCATE(aabbArrayB);
}

void detectPairsFix(u32 startBatch, u32 numBatch, PrefetchForwardIterator<SortData>& itrPair)
{
	lastNonContactPairIdx128 = 0xffffffff;
	lastNonContactPairIdx128_vec = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});

	const s32 tag1 = 8;

	ReadOnlyPrefetchForwardIterator<SortData> itrAabbA(
		&gPool,
		detectPairsIO.movAabbAddr + sizeof(SortData)*startBatch,
		detectPairsIO.movAabbAddr + sizeof(SortData)*(startBatch + numBatch),
		DETECTPAIR_PREFETCH_NUM, 11);

	SortData *aabbArrayB = (SortData*)ALLOCATE(128, sizeof(SortData)*DETECTPAIR_STORE_NUM);
	bool storeB = false;

	for(u32 i=0;i < numBatch;i++, ++itrAabbA) {
		SortData aabbA = *itrAabbA;
		u16 aabbMaxA = getXYZMax(aabbA, detectPairsIO.chkAxis);

		u32 startBAddr = detectPairsIO.fixAabbAddr;

		if(LIKELY(detectPairsIO.numFixAabb > 0)) {
			if(detectPairsIO.numFixAabb > DETECTPAIR_STORE_NUM)
				detectPairsSub(aabbMaxA, aabbA, startBAddr, detectPairsIO.numFixAabb, itrPair);
			else {
				if(!storeB) {
					storeB = true;
					spu_dma_large_get(aabbArrayB, startBAddr, sizeof(SortData)*detectPairsIO. numFixAabb, tag1, 0, 0);
					spu_dma_wait_tag_status_all(1<<tag1);
				}
				detectPairsSubLocal(aabbMaxA, aabbA, aabbArrayB, detectPairsIO.numFixAabb, itrPair);
			}
		}
	}

	DEALLOCATE(aabbArrayB);
}

void findAabbOvelap(u32 startBatch, u32 numBatch, PrefetchForwardIterator<vec_uint4>& itrOverlapped)
{
	SortData inAabb = findAabbOverlapIO.inAabb;

	u32 chkAxis = findAabbOverlapIO.chkAxis;
	u16 inAabbMax = getXYZMax(inAabb, chkAxis);
	u32 startAabbAddrA = findAabbOverlapIO.aabbAddr + sizeof(SortData)*startBatch;

	ReadOnlyPrefetchForwardIterator<SortData> itrAabbA(
		&gPool,
		startAabbAddrA,
		startAabbAddrA + sizeof(SortData)*numBatch,
		DETECTPAIR_PREFETCH_NUM, 11);

	for(u32 i=0;i < numBatch;i++, ++itrAabbA) {
		u32 slfA, tgtA;
		u16 stateIndexA;
		SortData aabbA = *itrAabbA;

		u16 aabbMinA = getXYZMin(aabbA, chkAxis);

		if(UNLIKELY(inAabbMax < aabbMinA))
			break;

		stateIndexA = getStateId(aabbA);
		slfA = getSelf(aabbA);
		tgtA = getTarget(aabbA);

		if(((slfA&findAabbOverlapIO.inTarget) && (tgtA&findAabbOverlapIO.inSelf)) && testAABB16(aabbA, inAabb)) {
			*(itrOverlapped) = spu_splats((u32)stateIndexA);
			++itrOverlapped;
			findAabbOverlapIO.numOverlapped++;
		}
	}
}

void addNewPairs()
{
	PrefetchForwardIterator<SortData> itrPair(
		&gPool,
		addNewPairsIO.pairsAddr + sizeof(SortData)*(addNewPairsIO.numPairs + addNewPairsIO.startPair),
		addNewPairsIO.pairsAddr + sizeof(SortData)*(addNewPairsIO.numPairs + addNewPairsIO.startPair + addNewPairsIO.batchPair),
		PREFETCH_NUM, 10);

	PrefetchForwardIterator<SortData> itrNewPair(
		&gPool,
		addNewPairsIO.newPairsAddr + sizeof(SortData)*addNewPairsIO.startPair,
		addNewPairsIO.newPairsAddr + sizeof(SortData)*(addNewPairsIO.startPair + addNewPairsIO.batchPair),
		PREFETCH_NUM, 11);

	s32 numContactBatch = 10;
	ContactPair *contacts = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*numContactBatch);

	for(u32 i=0;i < addNewPairsIO.batchPair;i++, ++itrPair, ++itrNewPair) {
		SortData& pair = *itrPair;
		SortData& newPair = *itrNewPair;
		ContactPair& newContact = contacts[i%numContactBatch];

		if(i%numContactBatch == 0) spu_dma_wait_tag_status_all(1<<12);

		u32 contactIdx = getPair(pair);

		newContact.duration = 0;
		newContact.numContacts = 0;
		newContact.stateIndex[0] = getStateA(newPair);
		newContact.stateIndex[1] = getStateB(newPair);

		spu_dma_put(&newContact, addNewPairsIO.contactsAddr + sizeof(ContactPair)*contactIdx, 128, 12, 0, 0);

		pair = newPair;
		setPair(pair, contactIdx);
		setFlag(pair, 1);
	}

	spu_dma_wait_tag_status_all(1<<12);
}

void detectPairs(u32 function, u32 commonBuffAddr)
{
	PrefetchForwardIterator<SortData> itrPair(
		&gPool,
		detectPairsIO.tmpSortsAddr + sizeof(SortData)*detectPairsIO.numTmpSorts,
		detectPairsIO.tmpSortsAddr + sizeof(SortData)*detectPairsIO.maxSorts,
		PREFETCH_NUM, 10);

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
			s32 rest = MAX((s32)detectPairsIO.numMovAabb - nextStartBatch, 0);
			s32 nextNumBatch = (rest > (s32)numBatch) ? (s32)numBatch : rest;

			commonBuff[1] = nextStartBatch;
			commonBuff[2] = nextNumBatch;

			unlock(commonBuff, commonBuffAddr);

			if(numBatch > 0) {
				if(function == BROADPHASE_DETECTPAIRS_MOV)
					detectPairsMov(startBatch, numBatch, itrPair);
				else
					detectPairsFix(startBatch, numBatch, itrPair);
			} else
				empty = true;
		}

		DEALLOCATE(commonBuff);
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
		case BROADPHASE_CALCVARIANCE:
			spu_dma_get(&assignStatesIO, addrIo, sizeof(IOParamAssignStates), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			checkVariance();

			spu_dma_put(&assignStatesIO, addrIo, sizeof(IOParamAssignStates), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case BROADPHASE_ASSIGNSTATES:
			spu_dma_get(&assignStatesIO, addrIo, sizeof(IOParamAssignStates), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(&worldVolume, assignStatesIO.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			assignStates();

			spu_dma_put(&assignStatesIO, addrIo, sizeof(IOParamAssignStates), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case BROADPHASE_DETECTPAIRS_MOV:
		case BROADPHASE_DETECTPAIRS_FIX:
			spu_dma_get(&detectPairsIO, addrIo, sizeof(IOParamDetectPairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			detectPairs(function, taskbuff[2]);

			spu_dma_put(&detectPairsIO, addrIo, sizeof(IOParamDetectPairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case BROADPHASE_MERGEPAIRS:
			spu_dma_get(&mergePairsIO, addrIo, sizeof(IOParamMergePairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			if(mergePairsIO.numNewSorts > 0) {
				s32 numRemoved = removeDuplicateAndMergePairs();
				mergePairsIO.numNewSorts -= numRemoved;
			}

			spu_dma_put(&mergePairsIO, addrIo, sizeof(IOParamMergePairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case BROADPHASE_REFRESHCONTACTPAIRS:
			spu_dma_get(&refreshPairsIO, addrIo, sizeof(IOParamRefreshPairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			refreshContactPairs();
			sync();
			parallelsort((SortData*)refreshPairsIO.sortsAddr, (SortData*)refreshPairsIO.buffAddr, refreshPairsIO.numContactPairs, taskId, refreshPairsIO.numSpu);

			spu_dma_put(&refreshPairsIO, addrIo, sizeof(IOParamRefreshPairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case BROADPHASE_ADDNEWPAIRS:
			spu_dma_get(&addNewPairsIO, addrIo, sizeof(IOParamAddNewPairs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			addNewPairs();
			break;
		case BROADPHASE_INTEGRATE:
			spu_dma_get(&integrateIO, addrIo, sizeof(IOParamIntegrate), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			integrate();
			break;
		case BROADPHASE_SLEEP:
			spu_dma_get(&integrateIO, addrIo, sizeof(IOParamIntegrate), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			sleepOrWakeup();
			break;
		case BROADPHASE_SORT:
			spu_dma_get(&sortIO, addrIo, sizeof(IOParamSort), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			parallelsort((SortData*)sortIO.sortsAddr, (SortData*)sortIO.buffAddr, sortIO.numSorts, taskId, sortIO.numSpu);
			break;
		case FIND_AABB_OVERLAP:
			spu_dma_get(&findAabbOverlapIO, addrIo, sizeof(IOParamFindAabbOverlap), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			{
				PrefetchForwardIterator<vec_uint4> itrOverlapped(
					&gPool,
					findAabbOverlapIO.overlappedAddr,
					findAabbOverlapIO.overlappedAddr + sizeof(vec_uint4)*findAabbOverlapIO.maxOverlapped,
					PREFETCH_NUM, 10);

				u32 commonBuffAddr = taskbuff[2];
				{
					u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(uint32_t)*32);
					bool empty = false;
					while(!empty) {
						u32 startBatch;
						u32 numBatch;

						lock(commonBuff, commonBuffAddr);

						startBatch = commonBuff[1];
						numBatch = commonBuff[2];

						s32 nextStartBatch = startBatch + numBatch;
						s32 rest = MAX((s32)findAabbOverlapIO.numAabb - nextStartBatch, 0);
						s32 nextNumBatch = (rest > (s32)numBatch) ? (s32)numBatch : rest;

						commonBuff[1] = nextStartBatch;
						commonBuff[2] = nextNumBatch;

						unlock(commonBuff, commonBuffAddr);

						if(numBatch > 0)
							findAabbOvelap(startBatch, numBatch, itrOverlapped);
						else
							empty = true;
					}

					DEALLOCATE(commonBuff);
				}
			}

			spu_dma_put(&findAabbOverlapIO, addrIo, sizeof(IOParamFindAabbOverlap), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case BROADPHASE_ASSIGN_SET:
			spu_dma_get(&assignStatesIO, addrIo, sizeof(IOParamAssignStates), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);


			//check variance
			checkVariance();

			spu_dma_put(&assignStatesIO, addrIo, sizeof(IOParamAssignStates), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			sync();

			{
				u32 chkAxis = assignStatesIO.chkAxis;
				u32 numSpu = assignStatesIO.numSpu;
				IOParamAssignStates *assignStatesIOFull = (IOParamAssignStates*)ALLOCATE(128, sizeof(IOParamAssignStates)*numSpu);
				spu_dma_get(assignStatesIOFull, addrIo - taskId*sizeof(IOParamAssignStates), sizeof(IOParamAssignStates)*numSpu, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				Vector3 s(0.0f), s2(0.0f);
				for(u32 i=0;i < numSpu;i++) {
					s += assignStatesIOFull[i].s;
					s2 += assignStatesIOFull[i].s2;
				}

				Vector3 v = s2 - mulPerElem(s, s)/(f32)assignStatesIO.numStates;
				if(v[1] > v[0]) chkAxis = 1;
				if(v[2] > v[chkAxis]) chkAxis = 2;

				assignStatesIO.chkAxis = chkAxis;
				DEALLOCATE(assignStatesIOFull);
			}

			spu_dma_get(&worldVolume, assignStatesIO.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			assignStates();

			spu_dma_put(&assignStatesIO, addrIo, sizeof(IOParamAssignStates), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case BROADPHASE_DETECT_SET:
			spu_dma_get(&detectSetIO, addrIo, sizeof(detectSetIO), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			//Sort
			memcpy(&sortIO, &detectSetIO.movAabbSort, sizeof(sortIO));
			parallelsort((SortData*)sortIO.sortsAddr, (SortData*)sortIO.buffAddr, sortIO.numSorts, taskId, sortIO.numSpu);
			sync();
			//Sort
			memcpy(&sortIO, &detectSetIO.fixAabbSort, sizeof(sortIO));
			parallelsort((SortData*)sortIO.sortsAddr, (SortData*)sortIO.buffAddr, sortIO.numSorts, taskId, sortIO.numSpu);
			sync();
			//Detect pairs mov-mov
			memcpy(&detectPairsIO, &detectSetIO.movDetectPairs, sizeof(detectPairsIO));
			detectPairs(BROADPHASE_DETECTPAIRS_MOV, detectPairsIO.commonBufAddr);
			sync();
			//Detect Pairs mov-fix
			detectPairsIO.commonBufAddr = detectSetIO.fixDetectPairs.commonBufAddr;
			detectPairs(BROADPHASE_DETECTPAIRS_FIX, detectPairsIO.commonBufAddr);
			memcpy(&detectSetIO.movDetectPairs, &detectPairsIO, sizeof(detectPairsIO));
			//write back
			spu_dma_put(&detectSetIO, addrIo, sizeof(detectSetIO), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
		case BROADPHASE_MERGE_SET:
			spu_dma_get(&mergeSetIO, addrIo, sizeof(mergeSetIO), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			//sort
			memcpy(&sortIO, &mergeSetIO.mergeSort, sizeof(sortIO));
			parallelsort((SortData*)sortIO.sortsAddr, (SortData*)sortIO.buffAddr, sortIO.numSorts, taskId, sortIO.numSpu);
			sync();
			//merge pairs
			memcpy(&mergePairsIO, &mergeSetIO.mergePairs, sizeof(mergePairsIO));
			if(mergePairsIO.numNewSorts > 0) {
				s32 numRemoved = removeDuplicateAndMergePairs();
				mergePairsIO.numNewSorts -= numRemoved;
			}
			{
				u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
				spu_atomic_add32(commonBuff, mergeSetIO.numNewPairsAddr, mergePairsIO.numNewSorts);
				DEALLOCATE(commonBuff);
			}

			memcpy(&mergeSetIO.mergePairs, &mergePairsIO, sizeof(mergePairsIO));
			sync();
			//sort
			memcpy(&sortIO, &mergeSetIO.mergeSort, sizeof(sortIO));
			parallelsort((SortData*)sortIO.sortsAddr, (SortData*)sortIO.buffAddr, sortIO.numSorts, taskId, sortIO.numSpu);
			sync();

			//add pairs
			u32 numNewPairs;
			numNewPairs = spu_dma_get_uint32(mergeSetIO.numNewPairsAddr, 0, 0, 0);
			{
				s32 maxTasks = mergeSetIO.mergeSort.numSpu;
				s32 numBatch = (numNewPairs + maxTasks - 1)/maxTasks;
				s32 startPairs = numBatch*taskId;
				s32 curBatch = ((s32)numNewPairs - startPairs < numBatch ) ? (s32)numNewPairs - startPairs : numBatch;
				if(startPairs < (s32)numNewPairs) {
					addNewPairsIO.startPair = startPairs;
					addNewPairsIO.batchPair = curBatch;
					addNewPairsIO.numPairs = mergeSetIO.mergePairs.numOldSorts;
					addNewPairsIO.contactsAddr = mergeSetIO.contactsAddr;
					addNewPairsIO.pairsAddr = mergeSetIO.mergePairs.oldSortsAddr;
					addNewPairsIO.newPairsAddr = mergeSetIO.mergeSort.sortsAddr;

					addNewPairs();
				}
			}

			//write back
			spu_dma_put(&mergeSetIO, addrIo, sizeof(mergeSetIO), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}

