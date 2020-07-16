/*
 * main.cpp
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/sortcommon.h"
#include "base/heapmanager.h"
#include "base/prefetchiterator.h"

#include "softbody/common/softbodyconfig.h"
#include "softbody/common/softbodyio.h"
#include "softbody/common/softcontact.h"
#include "softbody/common/softstate.h"

#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/worldvolume.h"

#include "sort/parallelsort.h"

#include "base/testaabb.h"
#include "base/safedma.h"

#define STATIC_MEM					0
#define DYNAMIC_MEM					(96*1024)
#define HEAP_BYTES 					(STATIC_MEM + DYNAMIC_MEM)

#define PREFETCH_NUM				64

#define ALLOCATE(align,size) 		gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16);
#define DEALLOCATE(ptr) 			if(ptr) {gPool.deallocate(((void*)ptr)); ptr = NULL;}

ATTRIBUTE_ALIGNED128(u32 lockBuffer[32]);

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool, HEAP_BYTES);

IOParamAssignStatesSoft assignStatesIO;
IOParamSortSoft sortIO;
IOParamSoftBodyGroups softbodyGroupIO;
IOParamFindPairsSoft findPairsIO;
IOParamIntegrateSoft integrateIO;

u32 taskId;
u32 barrier = 0;

WorldVolume worldVolume;

extern void hybridsort(SortData *data, SortData *buff, u32 n);

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

void integrate()
{
	PrefetchForwardIterator<SoftState> itrState(
		&gPool,
		softbodyGroupIO.sftStatesAddr,
		softbodyGroupIO.sftStatesAddr + sizeof(SoftState)*softbodyGroupIO.curProperty.numParticles,
		PREFETCH_NUM, 10);

	Vector3 gravity = softbodyGroupIO.curProperty.gravity;
	Vector3 extraForce = softbodyGroupIO.curProperty.extraForce;
	f32 timeStepSqr = integrateIO.timeStep*integrateIO.timeStep;
	f32 linearDamping = softbodyGroupIO.curProperty.linearDamping;

	for(u32 i=0;i < softbodyGroupIO.curProperty.numParticles;i++, ++itrState) {
		SoftState& particle = *itrState;

		if(!particle.isActive()) continue;

		Vector3 totalForce = gravity*particle.getMass() + particle.externalForce + extraForce;

		Vector3 pos = particle.getPosition();
		Vector3 oldPos = particle.getOldPosition();
		particle.setOldPosition(pos);

		pos += (pos - oldPos)*linearDamping + totalForce/particle.getMass()*timeStepSqr;

		particle.setPosition(pos);

		particle.externalForce = Vector3(0);
	}
}

void assignStatesRig()
{
	PrefetchForwardIterator<SortData> itrAabbX(
		&gPool,
		assignStatesIO.aabbAddr[0],
		assignStatesIO.aabbAddr[0] + sizeof(SortData)*assignStatesIO.numStates,
		PREFETCH_NUM, 10);

	PrefetchForwardIterator<SortData> itrAabbY(
		&gPool,
		assignStatesIO.aabbAddr[1],
		assignStatesIO.aabbAddr[1] + sizeof(SortData)*assignStatesIO.numStates,
		PREFETCH_NUM, 11);

	PrefetchForwardIterator<SortData> itrAabbZ(
		&gPool,
		assignStatesIO.aabbAddr[2],
		assignStatesIO.aabbAddr[2] + sizeof(SortData)*assignStatesIO.numStates,
		PREFETCH_NUM, 12);

	PrefetchForwardIterator<TrbState> itrState(
		&gPool,
		assignStatesIO.statesAddr + sizeof(TrbState)*assignStatesIO.batchStartState,
		assignStatesIO.statesAddr + sizeof(TrbState)*(assignStatesIO.batchStartState + assignStatesIO.numBatchStates),
		PREFETCH_NUM, 13);

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
		if(checkInWorld[0] > 0.0f || checkInWorld[1] > 0.0f || checkInWorld[2] > 0.0f)
			continue;

		VecInt3 aabbMinL, aabbMaxL;
		worldVolume.worldToLocalPosition(aabbMin, aabbMax, aabbMinL, aabbMaxL);

		SortData& aabbX = *itrAabbX;
		setKey(aabbX, aabbMinL.getX());
		setXMin(aabbX, aabbMinL.getX());
		setXMax(aabbX, aabbMaxL.getX());
		setYMin(aabbX, aabbMinL.getY());
		setYMax(aabbX, aabbMaxL.getY());
		setZMin(aabbX, aabbMinL.getZ());
		setZMax(aabbX, aabbMaxL.getZ());
		setStateId(aabbX, assignStatesIO.batchStartState + i);
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
		setStateId(aabbY, assignStatesIO.batchStartState + i);
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
		setStateId(aabbZ, assignStatesIO.batchStartState+i);
		setBodyId(aabbZ, state.trbBodyIdx);
		setMovType(aabbZ, state.getMoveType());
		setSelf(aabbZ, state.getContactFilterSelf());
		setTarget(aabbZ, state.getContactFilterTarget());
		++itrAabbZ;

		assignStatesIO.numAabb++;
	}
}

void findPairs()
{
	s32 chkAxis = 0;
	u32 numPclAabb = 0;
	u32 numParticles = softbodyGroupIO.curProperty.numParticles;
	SortData *pclAabbArray = (SortData*)ALLOCATE(16, sizeof(SortData)*numParticles);

	{
		PrefetchForwardIterator<SoftState> itrState(
			&gPool,
			softbodyGroupIO.sftStatesAddr,
			softbodyGroupIO.sftStatesAddr + sizeof(SoftState)*numParticles,
			PREFETCH_NUM, 10);

		s32 n = 0;
		Vector3 s(0.0f), s2(0.0f);

		for(u32 i=0;i < numParticles;i++, ++itrState) {
			SoftState& particle = (*itrState);

			if(!particle.isActive()) continue;

			Vector3 pclCenter(particle.getPosition());
			s += pclCenter;
			s2 += mulPerElem(pclCenter, pclCenter);
			n++;
		}

		Vector3 v = s2 - mulPerElem(s, s)/(f32)n;
		if(v[1] > v[0]) chkAxis = 1;
		if(v[2] > v[chkAxis]) chkAxis = 2;
	}

	{
		PrefetchForwardIterator<SoftState> itrState(
			&gPool,
			softbodyGroupIO.sftStatesAddr,
			softbodyGroupIO.sftStatesAddr + sizeof(SoftState)*numParticles,
			PREFETCH_NUM, 10);

		for(u32 i=0;i < numParticles;i++, ++itrState) {
			SoftState& particle = *itrState;

			if(!particle.isActive()) continue;

			// AABB
			Vector3 pclCenter(particle.getPosition());
			Vector3 pclHalf(softbodyGroupIO.curProperty.particleRadius);
			Vector3 aabbMin = pclCenter - pclHalf;
			Vector3 aabbMax = pclCenter + pclHalf;

			Vector3 worldHalf(worldVolume.extent);
			Vector3 checkInWorld = absPerElem(pclCenter - worldVolume.origin) - (pclHalf + worldHalf);
			if(checkInWorld[0] > 0.0f || checkInWorld[1] > 0.0f || checkInWorld[2] > 0.0f) {
				particle.setActive(false);
				continue;
			}

			VecInt3 aabbMinL, aabbMaxL;
			worldVolume.worldToLocalPosition(aabbMin, aabbMax, aabbMinL, aabbMaxL);

			// 配列に追加
			SortData& aabb = pclAabbArray[numPclAabb];
			setKey(aabb, aabbMinL.get(chkAxis));
			setXMin(aabb, aabbMinL.getX());
			setXMax(aabb, aabbMaxL.getX());
			setYMin(aabb, aabbMinL.getY());
			setYMax(aabb, aabbMaxL.getY());
			setZMin(aabb, aabbMinL.getZ());
			setZMax(aabb, aabbMaxL.getZ());
			setStateId(aabb, i);

			numPclAabb++;
		}

		SortData *workBuff = (SortData*)ALLOCATE(16, sizeof(SortData)*numParticles);
		hybridsort(pclAabbArray, workBuff, numPclAabb);
		DEALLOCATE(workBuff);
	}

	if(softbodyGroupIO.curProperty.selfCollisionEnable) {
		PrefetchForwardIterator<SortData> itrPair(
			&gPool,
			softbodyGroupIO.contactSortsPclPclAddr,
			softbodyGroupIO.contactSortsPclPclAddr + sizeof(SortData)*softbodyGroupIO.maxContactPairs,
			PREFETCH_NUM, 10);

		u32 numContactsPclPcl = 0;

		for(u32 i=0;i < numPclAabb;i++) {
			u16 stateIndexA;
			u16 aabbMaxA[3];
			SortData aabbA = pclAabbArray[i];

			stateIndexA = getStateId(aabbA);
			aabbMaxA[0] = getXMax(aabbA);
			aabbMaxA[1] = getYMax(aabbA);
			aabbMaxA[2] = getZMax(aabbA);

			for(u32 j=i + 1;j < numPclAabb;j++) {
				u16 stateIndexB;
				u16 aabbMinB[3];
				SortData aabbB = pclAabbArray[j];

				stateIndexB = getStateId(aabbB);
				aabbMinB[0] = getXMin(aabbB);
				aabbMinB[1] = getYMin(aabbB);
				aabbMinB[2] = getZMin(aabbB);

				if(aabbMaxA[chkAxis] < aabbMinB[chkAxis])
					break;

				if(testAABB16(aabbA, aabbB) && numContactsPclPcl < softbodyGroupIO.maxContactPairs) {
					setStatePair(*itrPair, stateIndexA, stateIndexB);
					setFlag(*itrPair, 1);
					numContactsPclPcl++;
					++itrPair;
				}
			}
		}
		softbodyGroupIO.numContactsPclPcl = numContactsPclPcl;
	}

	{
		PrefetchForwardIterator<SortData> itrPair(
			&gPool,
			softbodyGroupIO.contactSortsPclRigAddr,
			softbodyGroupIO.contactSortsPclRigAddr + sizeof(SortData)*softbodyGroupIO.maxContactPairs,
			PREFETCH_NUM, 10);

		u32 numContactsPclRig = 0;

		ReadOnlyPrefetchForwardIterator<SortData> itrRigAabb(
			&gPool,
			findPairsIO.rigAabbAddr[chkAxis],
			findPairsIO.rigAabbAddr[chkAxis] + sizeof(SortData)*findPairsIO.numRigAabb,
			PREFETCH_NUM,11);

		for(u32 j=0;j < findPairsIO.numRigAabb;j++, ++itrRigAabb) {
			u16 stateIndexB;
			u32 aabbMaxB[3];
			SortData aabbB = *itrRigAabb;

			stateIndexB = getStateId(aabbB);
			aabbMaxB[0] = getXMax(aabbB);
			aabbMaxB[1] = getYMax(aabbB);
			aabbMaxB[2] = getZMax(aabbB);

			for(u32 i=0;i < numPclAabb;i++) {
				u16 stateIndexA;
				u32 aabbMinA[3];
				SortData& aabbA = pclAabbArray[i];

				stateIndexA = getStateId(aabbA);
				aabbMinA[0] = getXMin(aabbA);
				aabbMinA[1] = getYMin(aabbA);
				aabbMinA[2] = getZMin(aabbA);

				if(aabbMaxB[chkAxis] < aabbMinA[chkAxis])
					break;

				if(isCollidable(softbodyGroupIO.curProperty.contactFilterSelf, softbodyGroupIO.curProperty.contactFilterTarget, getSelf(aabbB), getTarget(aabbB)) &&
				   testAABB16(aabbA, aabbB) && numContactsPclRig < softbodyGroupIO.maxContactPairs)
				{
					setStatePair2(*itrPair, stateIndexA, stateIndexB);
					setBodyB(*itrPair, getBodyId(aabbB));
					setFlag(*itrPair,1);
					numContactsPclRig++;
					++itrPair;
				}
			}
		}
		softbodyGroupIO.numContactsPclRig = numContactsPclRig;
	}

	DEALLOCATE(pclAabbArray);
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
		case SOFTBODY_FINDPAIRS:
		{
			s32 groupDmaTag = 8;

			spu_dma_get(&findPairsIO, addrIo, sizeof(IOParamFindPairsSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			spu_dma_get(&worldVolume, findPairsIO.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			u32 commonBuffAddr = taskbuff[2];
			u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
			bool empty = false;
			while(!empty) {
				lock(commonBuff, commonBuffAddr);

				u32 groupAddr = commonBuff[1];
				u32 nextGroup = commonBuff[2];
				u32 maxGroup  = commonBuff[3];

				if(nextGroup < maxGroup) {
					spu_dma_wait_tag_status_all(1<<groupDmaTag);
					spu_dma_get(&softbodyGroupIO, groupAddr + sizeof(IOParamSoftBodyGroups)*nextGroup, sizeof(IOParamSoftBodyGroups), groupDmaTag, 0, 0);
					commonBuff[2]++;
				} else
					empty = true;

				unlock(commonBuff, commonBuffAddr);

				if(!empty) {
					spu_dma_wait_tag_status_all(1<<groupDmaTag);
					findPairs();
					spu_dma_put(&softbodyGroupIO, groupAddr + sizeof(IOParamSoftBodyGroups)*nextGroup, sizeof(IOParamSoftBodyGroups), groupDmaTag, 0, 0);
				}
			}
			spu_dma_wait_tag_status_all(1<<groupDmaTag);

			DEALLOCATE(commonBuff);

			spu_dma_put(&findPairsIO, addrIo, sizeof(IOParamFindPairsSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
		}
		break;

		case SOFTBODY_INTEGRATE:
		{
			s32 groupDmaTag = 8;

			spu_dma_get(&integrateIO, addrIo, sizeof(IOParamIntegrateSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			u32 commonBuffAddr = taskbuff[2];
			u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
			bool empty = false;
			while(!empty) {
				lock(commonBuff, commonBuffAddr);

				u32 groupAddr = commonBuff[1];
				u32 nextGroup = commonBuff[2];
				u32 maxGroup  = commonBuff[3];

				if(nextGroup < maxGroup) {
					spu_dma_get(&softbodyGroupIO, groupAddr + sizeof(IOParamSoftBodyGroups)*nextGroup, sizeof(IOParamSoftBodyGroups), groupDmaTag, 0, 0);
					commonBuff[2]++;
				} else
					empty = true;

				unlock(commonBuff, commonBuffAddr);

				if(!empty) {
					spu_dma_wait_tag_status_all(1<<groupDmaTag);
					integrate();
				}
			}

			DEALLOCATE(commonBuff);
		}
		break;

		case SOFTBODY_ASSIGNSTATES_RIG:
		{
			spu_dma_get(&assignStatesIO, addrIo, sizeof(IOParamAssignStatesSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(&worldVolume, assignStatesIO.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			assignStatesRig();

			spu_dma_put(&assignStatesIO, addrIo, sizeof(IOParamAssignStatesSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
		}
		break;

		case SOFTBODY_SORT:
		{
			spu_dma_get(&sortIO, addrIo, sizeof(IOParamSortSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			parallelsort((SortData*)sortIO.sortsAddr, (SortData*)sortIO.buffAddr, sortIO.numSorts, taskId, sortIO.numSpu);
		}
		break;
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}


