/*
 * main.cpp
 *
 *  Created on: Jun 7, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/heapmanager.h"
#include "base/sortcommon.h"
#include "base/safedma.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/heightfield.h"
#include "rigidbody/common/intersectfunction.h"
#include "rigidbody/common/worldvolume.h"

#include "raycast/common/raycastconfig.h"
#include "raycast/common/raycastio.h"
#include "raycast/common/rayintersect.h"
#include "raycast/common/ray.h"

#include "base/prefetchiterator.h"
#include "base/simplestack.h"
#include "base/testaabb.h"

#include "sort/parallelsort.h"

#define STATIC_MEM						0
#define DYNAMIC_MEM						(96*1024)
#define HEAP_BYTES 						(STATIC_MEM + DYNAMIC_MEM)

#define REPSILON 						0.00001f

#define PREFETCH_NUM					64

#define ALLOCATE(align, size) 			gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16)
#define DEALLOCATE(ptr) 				if(ptr) {gPool.deallocate((void*)ptr); ptr = NULL;}

u32 taskId;
u32 barrier = 0;

IOParamRayCastCommon io;
IOParamBroadPhase assignStatesIO;
IOParamSortAabbs sortAabbIO;

WorldVolume worldVolume;

u32 lastRayGroup;
u32 lastNonContactFlagIdx128;

ATTRIBUTE_ALIGNED128(u32 nonContactFlag[32]);

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool, HEAP_BYTES);

void sync()
{
	while(mars_task_barrier_try_notify(barrier) != MARS_SUCCESS) {}
	while(mars_task_barrier_try_wait(barrier) != MARS_SUCCESS) {}
}

void assignStates()
{
	s32 dmaTag[] = {10, 11, 12, 13, 14, 15, 16, 17};

	PrefetchForwardIterator<SortData> itrAabb[6] = {
		PrefetchForwardIterator<SortData>(
			&gPool,
			assignStatesIO.aabbAddr[0],
			assignStatesIO.aabbAddr[0] + sizeof(SortData)*assignStatesIO.numBatchStates,
			PREFETCH_NUM, dmaTag[0]),
		PrefetchForwardIterator<SortData>(
			&gPool,
			assignStatesIO.aabbAddr[1],
			assignStatesIO.aabbAddr[1] + sizeof(SortData)*assignStatesIO.numBatchStates,
			PREFETCH_NUM, dmaTag[1]),
		PrefetchForwardIterator<SortData>(
			&gPool,
			assignStatesIO.aabbAddr[2],
			assignStatesIO.aabbAddr[2] + sizeof(SortData)*assignStatesIO.numBatchStates,
			PREFETCH_NUM, dmaTag[2]),
		PrefetchForwardIterator<SortData>(
			&gPool,
			assignStatesIO.aabbAddr[3],
			assignStatesIO.aabbAddr[3] + sizeof(SortData)*assignStatesIO.numBatchStates,
			PREFETCH_NUM, dmaTag[3]),
		PrefetchForwardIterator<SortData>(
			&gPool,
			assignStatesIO.aabbAddr[4],
			assignStatesIO.aabbAddr[4] + sizeof(SortData)*assignStatesIO.numBatchStates,
			PREFETCH_NUM, dmaTag[4]),
		PrefetchForwardIterator<SortData>(
			&gPool,
			assignStatesIO.aabbAddr[5],
			assignStatesIO.aabbAddr[5] + sizeof(SortData)*assignStatesIO.numBatchStates,
			PREFETCH_NUM, dmaTag[5])
	};

	ReadOnlyPrefetchForwardIterator<TrbState> itrState(
		&gPool,
		assignStatesIO.statesAddr + sizeof(TrbState)*assignStatesIO.batchStartState,
		assignStatesIO.statesAddr + sizeof(TrbState)*(assignStatesIO.batchStartState + assignStatesIO.numBatchStates),
		PREFETCH_NUM, dmaTag[6]);

	for(u32 i=0;i < assignStatesIO.numBatchStates;i++, ++itrState) {
		const TrbState& state = (*itrState);

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

		{
			SortData& aabbX = *itrAabb[0];
			SortData& aabbY = *itrAabb[1];
			SortData& aabbZ = *itrAabb[2];

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

			setKey(aabbZ, aabbMinL.getZ());
			setXMin(aabbZ, aabbMinL.getX());
			setXMax(aabbZ, aabbMaxL.getX());
			setYMin(aabbZ, aabbMinL.getY());
			setYMax(aabbZ, aabbMaxL.getY());
			setZMin(aabbZ, aabbMinL.getZ());
			setZMax(aabbZ, aabbMaxL.getZ());
			setStateId(aabbZ, assignStatesIO.batchStartState + i);
			setBodyId(aabbZ, state.trbBodyIdx);
			setMovType(aabbZ, state.getMoveType());
			setSelf(aabbZ, state.getContactFilterSelf());
			setTarget(aabbZ, state.getContactFilterTarget());

			SortData& aabbInvX = *itrAabb[3];
			aabbInvX = aabbX;
			setKey(aabbInvX, getXMax(aabbX));

			SortData& aabbInvY = *itrAabb[4];
			aabbInvY = aabbY;
			setKey(aabbInvY, getYMax(aabbY));

			SortData& aabbInvZ = *itrAabb[5];
			aabbInvZ = aabbZ;
			setKey(aabbInvZ, getZMax(aabbZ));
		}

		++itrAabb[0];
		++itrAabb[1];
		++itrAabb[2];
		++itrAabb[3];
		++itrAabb[4];
		++itrAabb[5];
		assignStatesIO.numAabb++;
	}
}

inline bool isCollidable(const Ray& ray, u16 rigidBodyIndex, u32 contactFilterSelf, u32 contactFilterTarget)
{
	u32 idx = ray.rayGroup*io.maxRayGroups + rigidBodyIndex;

	u32 idx128 = idx>>10;
	u32 idx32 = (idx&1023)>>5;
	u32 idx1 = 1L<<(idx&31);

	if(lastRayGroup != ray.rayGroup || idx128 != lastNonContactFlagIdx128) {
		spu_dma_get(nonContactFlag, io.nonContactFlagAddr + 128*idx128, 128, 0, 0, 0);
		spu_dma_wait_tag_status_all(1);
		lastNonContactFlagIdx128 = idx128;
		lastRayGroup = ray.rayGroup;
	}

	return (nonContactFlag[idx32]&idx1) == 0 && (ray.contactFilterSelf&contactFilterTarget) && (ray.contactFilterTarget&contactFilterSelf);
}

void traverseForward(Ray& ray, SortData rayAABB, s32 chkAxis)
{
	ReadOnlyPrefetchForwardIterator<SortData> itrAabbArray(
		&gPool,
		io.aabbAddr[chkAxis],
		io.aabbAddr[chkAxis] + sizeof(SortData)*io.numAabb,
		PREFETCH_NUM, 12);

	Vector3 boundOnRay = ray.endPos;

	for(u32 i=0;i < io.numAabb;i++, ++itrAabbArray) {
		const SortData rbAABB = *itrAabbArray;

		if(getXYZMax(rbAABB, chkAxis) < getXYZMin(rayAABB, chkAxis))
			continue;

		const vec_uchar16 mask_ptn = ((vec_uchar16){0x80, 0x80, 0x00, 0x01, 0x80, 0x80, 0x04, 0x05, 0x80, 0x80, 0x08, 0x09, 0x80, 0x80, 0x80, 0x80});
		vec_int4 aabbMin = (vec_int4)spu_shuffle(rbAABB.vdata[0], rbAABB.vdata[0], mask_ptn);
		Vector3 boundOnAABB = worldVolume.localToWorldPosition(VecInt3(aabbMin));

		if(UNLIKELY(getXYZMax(rayAABB, chkAxis) < getXYZMin(rbAABB, chkAxis) || boundOnRay[chkAxis] < boundOnAABB[chkAxis]))
			return;

		u16 stateIndex = getStateId(rbAABB);
		u32 contactFilterSelf = getSelf(rbAABB);
		u32 contactFilterTarget = getTarget(rbAABB);

		if(isCollidable(ray, stateIndex, contactFilterSelf, contactFilterTarget) && testAABB16(rayAABB, rbAABB)) {
			TrbState state;
			CollObject coll;

			spu_dma_get(&state, io.statesAddr + sizeof(TrbState)*stateIndex, sizeof(TrbState), 14, 0, 0);
			spu_dma_get(&coll, io.collsAddr + sizeof(CollObject)*getBodyId(rbAABB), sizeof(CollObject), 15, 0, 0);
			spu_dma_wait_tag_status_all(1<<14);

			Vector3 center = read_Vector3(state.center);
			Vector3 half = read_Vector3(state.half);
			f32 t;
			if(rayIntersectAABBFast(half, center, ray.startPos, ray.rayDir, t) && t < ray.t) {
				Transform3 transform(state.getOrientation(), state.getPosition());
				spu_dma_wait_tag_status_all(1<<15);
				if(rayIntersect(coll, transform, ray, ray.t)) {
					ray.contactFlag = true;
					ray.contactInstance = stateIndex;
					boundOnRay = ray.startPos + ray.t*ray.rayDir;
				}
			}
			spu_dma_wait_tag_status_all(1<<15);
		}
	}
}

void traverseBackward(Ray& ray, SortData rayAABB, s32 chkAxis)
{
	ReadOnlyPrefetchBackwardIterator<SortData> itrAabbArray(
		&gPool,
		io.aabbAddr[chkAxis + 3] + sizeof(SortData)*(io.numAabb),
		io.aabbAddr[chkAxis + 3],
		PREFETCH_NUM, 12);

	--itrAabbArray;

	Vector3 boundOnRay = ray.endPos;

	for(s32 i=(s32)io.numAabb - 1;i >= 0;i--, --itrAabbArray) {
		SortData rbAABB = *itrAabbArray;

		if(getXYZMax(rayAABB, chkAxis) < getXYZMin(rbAABB, chkAxis))
			continue;

		const vec_uchar16 mask_ptn = ((vec_uchar16){0x80, 0x80, 0x02, 0x03, 0x80, 0x80, 0x06, 0x07, 0x80, 0x80, 0x0a, 0x0b, 0x80, 0x80, 0x80, 0x80});
		vec_int4 aabbMax = (vec_int4)spu_shuffle(rbAABB.vdata[0], rbAABB.vdata[0], mask_ptn);
		Vector3 boundOnAABB = worldVolume.localToWorldPosition(VecInt3(aabbMax));

		if(UNLIKELY(getXYZMax(rbAABB, chkAxis) < getXYZMin(rayAABB, chkAxis) || boundOnAABB[chkAxis] < boundOnRay[chkAxis]))
			return;

		u16 stateIndex = getStateId(rbAABB);
		u32 contactFilterSelf = getSelf(rbAABB);
		u32 contactFilterTarget = getTarget(rbAABB);

		if(isCollidable(ray, stateIndex, contactFilterSelf, contactFilterTarget) && testAABB16(rayAABB, rbAABB)) {
			TrbState state;
			CollObject coll;

			spu_dma_get(&state, io.statesAddr + sizeof(TrbState)*stateIndex, sizeof(TrbState), 14, 0, 0);
			spu_dma_get(&coll, io.collsAddr + sizeof(CollObject)*getBodyId(rbAABB), sizeof(CollObject), 15, 0, 0);
			spu_dma_wait_tag_status_all(1<<14);

			Vector3 center = read_Vector3(state.center);
			Vector3 half = read_Vector3(state.half);
			f32 t;
			if(rayIntersectAABBFast(half, center, ray. startPos, ray.rayDir, t) && t < ray.t) {
				Transform3 transform(state.getOrientation(), state.getPosition());
				spu_dma_wait_tag_status_all(1<<15);
				if(rayIntersect(coll, transform, ray, ray.t)) {
					ray.contactFlag = true;
					ray.contactInstance = stateIndex;
					boundOnRay = ray.startPos + ray.t*ray.rayDir;
				}
			}
			spu_dma_wait_tag_status_all(1<<15);
		}
	}
}

void traverseForwardStride(Ray *rays, s32 numStride, SortData rayAABB, s32 chkAxis)
{
	ReadOnlyPrefetchForwardIterator<SortData> itrAabbArray(
		&gPool,
		io.aabbAddr[chkAxis],
		io.aabbAddr[chkAxis] + sizeof(SortData)*io.numAabb,
		PREFETCH_NUM, 12);

	for(u32 i=0;i < io.numAabb;i++, ++itrAabbArray) {
		const SortData rbAABB = *itrAabbArray;

		if(getXYZMax(rayAABB, chkAxis) < getXYZMin(rbAABB, chkAxis))
			return;

		if(getXYZMax(rbAABB, chkAxis) < getXYZMin(rayAABB, chkAxis))
			continue;

		if(!testAABB16(rayAABB, rbAABB))
			continue;

		u16 stateIndex = getStateId(rbAABB);
		u32 contactFilterSelf = getSelf(rbAABB);
		u32 contactFilterTarget = getTarget(rbAABB);

		bool transferred = false;
		TrbState state;
		CollObject coll;

		for(s32 r=0;r < numStride;r++) {
			Ray& ray = rays[r];

			if(isCollidable(ray, stateIndex, contactFilterSelf, contactFilterTarget)) {
				if(!transferred) {
					spu_dma_get(&state, io.statesAddr + sizeof(TrbState)*stateIndex, sizeof(TrbState), 14, 0, 0);
					spu_dma_get(&coll, io.collsAddr + sizeof(CollObject)*getBodyId(rbAABB), sizeof(CollObject), 15, 0, 0);
					spu_dma_wait_tag_status_all(1<<14);
					transferred = true;
				}

				Vector3 center = read_Vector3(state.center);
				Vector3 half = read_Vector3(state.half);
				f32 t;
				if(rayIntersectAABBFast(half, center, ray.startPos, ray.rayDir,t) && t < ray.t) {
					Transform3 transform(state.getOrientation(), state.getPosition());
					spu_dma_wait_tag_status_all(1<<15);
					if(rayIntersect(coll, transform, ray, ray.t)) {
						ray.contactFlag = true;
						ray.contactInstance = stateIndex;
					}
				}
				spu_dma_wait_tag_status_all(1<<15);
			}
		}
	}
}

void traverseBackwardStride(Ray *rays, s32 numStride, SortData rayAABB, s32 chkAxis)
{
	ReadOnlyPrefetchBackwardIterator<SortData> itrAabbArray(
		&gPool,
		io.aabbAddr[chkAxis + 3] + sizeof(SortData)*(io.numAabb),
		io.aabbAddr[chkAxis + 3],
		PREFETCH_NUM, 12);

	--itrAabbArray;

	for(s32 i=(s32)io.numAabb - 1;i >= 0;i--, --itrAabbArray) {
		SortData rbAABB = *itrAabbArray;

		if(getXYZMax(rbAABB, chkAxis) < getXYZMin(rayAABB, chkAxis))
			return;

		if(getXYZMax(rayAABB, chkAxis) < getXYZMin(rbAABB, chkAxis))
			continue;

		if(!testAABB16(rayAABB, rbAABB))
			continue;

		u16 stateIndex = getStateId(rbAABB);
		u32 contactFilterSelf = getSelf(rbAABB);
		u32 contactFilterTarget = getTarget(rbAABB);

		bool transferred = false;
		TrbState state;
		CollObject coll;

		for(s32 r=0;r < numStride;r++) {
			Ray& ray = rays[r];

			if(isCollidable(ray, stateIndex, contactFilterSelf, contactFilterTarget)) {
				if(!transferred) {
					spu_dma_get(&state, io.statesAddr + sizeof(TrbState)*stateIndex, sizeof(TrbState), 14, 0, 0);
					spu_dma_get(&coll, io.collsAddr + sizeof(CollObject)*getBodyId(rbAABB), sizeof(CollObject), 15, 0, 0);
					spu_dma_wait_tag_status_all(1<<14);
					transferred = true;
				}

				Vector3 center = read_Vector3(state.center);
				Vector3 half = read_Vector3(state.half);
				f32 t;
				if(rayIntersectAABBFast(half, center, ray.startPos, ray.rayDir,t) && t < ray.t) {
					Transform3 transform(state.getOrientation(), state.getPosition());
					spu_dma_wait_tag_status_all(1<<15);
					if(rayIntersect(coll, transform, ray, ray.t)) {
						ray.contactFlag = true;
						ray.contactInstance = stateIndex;
					}
				}
				spu_dma_wait_tag_status_all(1<<15);
			}
		}
	}
}

void castSingleRay(Ray& ray)
{
	lastNonContactFlagIdx128 = lastRayGroup = 0xffffffff;

	ray.contactFlag = false;
	ray.rayDir = ray.endPos - ray.startPos;

	Vector3 chkAxisVec = absPerElem(ray.rayDir);
	s32 chkAxis = 0;
	if(chkAxisVec[1] < chkAxisVec[0]) chkAxis = 1;
	if(chkAxisVec[2] < chkAxisVec[chkAxis]) chkAxis = 2;

	Vector3 rayAabbMin = minPerElem(ray.startPos, ray.endPos);
	Vector3 rayAabbMax = maxPerElem(ray.startPos, ray.endPos);
	VecInt3 rayMin, rayMax;
	worldVolume.worldToLocalPosition(rayAabbMin, rayAabbMax, rayMin, rayMax);

	SortData rayAABB(0);
	setXMin(rayAABB, rayMin.getX());
	setXMax(rayAABB, rayMax.getX());
	setYMin(rayAABB, rayMin.getY());
	setYMax(rayAABB, rayMax.getY());
	setZMin(rayAABB, rayMin.getZ());
	setZMax(rayAABB, rayMax.getZ());

	s32 sign = ray.rayDir[chkAxis] < 0.0f ? -1 : 1;

	if(sign > 0)
		traverseForward(ray, rayAABB, chkAxis);
	else
		traverseBackward(ray, rayAABB, chkAxis);
}

void castStrideRays(Ray *rays, s32 numStride)
{
	lastNonContactFlagIdx128 = lastRayGroup = 0xffffffff;

	for(s32 j=0;j < numStride;j++) {
		Ray &ray = rays[j];
		ray.t = 1.0f;
		ray.contactFlag = false;
		ray.rayDir = ray.endPos - ray.startPos;
	}

	Vector3 chkAxisVec = absPerElem(rays[0].rayDir);
	s32 chkAxis = 0;
	if(chkAxisVec[1] < chkAxisVec[0]) chkAxis = 1;
	if(chkAxisVec[2] < chkAxisVec[chkAxis]) chkAxis = 2;

	s32 sign = rays[0].rayDir[chkAxis] < 0.0f ? -1 : 1;

	SortData rayAABB(0);

	for(s32 j=0;j < numStride;j++) {
		Ray& ray = rays[j];

		Vector3 rayAabbMin = minPerElem(ray.startPos, ray.endPos);
		Vector3 rayAabbMax = maxPerElem(ray.startPos, ray.endPos);
		VecInt3 rayMin, rayMax;
		worldVolume.worldToLocalPosition(rayAabbMin, rayAabbMax, rayMin, rayMax);

		if(j == 0) {
			setXMin(rayAABB, rayMin.getX());
			setXMax(rayAABB, rayMax.getX());
			setYMin(rayAABB, rayMin.getY());
			setYMax(rayAABB, rayMax.getY());
			setZMin(rayAABB, rayMin.getZ());
			setZMax(rayAABB, rayMax.getZ());
		} else {
			setXMin(rayAABB,MIN(getXMin(rayAABB), rayMin.getX()));
			setXMax(rayAABB,MAX(getXMax(rayAABB), rayMax.getX()));
			setYMin(rayAABB,MIN(getYMin(rayAABB), rayMin.getY()));
			setYMax(rayAABB,MAX(getYMax(rayAABB), rayMax.getY()));
			setZMin(rayAABB,MIN(getZMin(rayAABB), rayMin.getZ()));
			setZMax(rayAABB,MAX(getZMax(rayAABB), rayMax.getZ()));
		}
	}

	if(sign > 0)
		traverseForwardStride(rays, numStride, rayAABB, chkAxis);
	else
		traverseBackwardStride(rays, numStride, rayAABB, chkAxis);
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
		case RAYCAST_ASSIGNSTATES:
			spu_dma_get(&assignStatesIO, addrIo, sizeof(IOParamBroadPhase), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(&worldVolume, assignStatesIO.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			assignStates();

			spu_dma_put(&assignStatesIO, addrIo, sizeof(IOParamBroadPhase), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
			break;

		case RAYCAST_SORTAABB:
			spu_dma_get(&sortAabbIO, addrIo, sizeof(IOParamSortAabbs), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			parallelsort((SortData*)sortAabbIO.aabbAddr[0], (SortData*)sortAabbIO.buffAddr, sortAabbIO.numAabb, taskId, sortAabbIO.numSpu);
			sync();
			parallelsort((SortData*)sortAabbIO.aabbAddr[1], (SortData*)sortAabbIO.buffAddr, sortAabbIO.numAabb, taskId, sortAabbIO.numSpu);
			sync();
			parallelsort((SortData*)sortAabbIO.aabbAddr[2], (SortData*)sortAabbIO.buffAddr, sortAabbIO.numAabb, taskId, sortAabbIO.numSpu);
			sync();
			parallelsort((SortData*)sortAabbIO.aabbAddr[3], (SortData*)sortAabbIO.buffAddr, sortAabbIO.numAabb, taskId, sortAabbIO.numSpu);
			sync();
			parallelsort((SortData*)sortAabbIO.aabbAddr[4], (SortData*)sortAabbIO.buffAddr, sortAabbIO.numAabb, taskId, sortAabbIO.numSpu);
			sync();
			parallelsort((SortData*)sortAabbIO.aabbAddr[5], (SortData*)sortAabbIO.buffAddr, sortAabbIO.numAabb, taskId, sortAabbIO.numSpu);
			sync();
			break;

		case RAYCAST_RAYCAST_SINGLE:
		{
			u32 eaRay = taskbuff[2];
			ATTRIBUTE_ALIGNED128(Ray ray);

			spu_dma_get(&io, addrIo, sizeof(IOParamRayCastCommon), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(&worldVolume, io.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(&ray, eaRay, sizeof(Ray), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			castSingleRay(ray);

			spu_dma_put(&ray, eaRay, sizeof(Ray), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);
		}
		break;

		case RAYCAST_RAYCAST_STRIDE:
		{
			u32 eaRay = taskbuff[2];
			u32 numStride = taskbuff[3];
			Ray *ray = (Ray*)ALLOCATE(128, sizeof(Ray)*numStride);

			spu_dma_get(&io, addrIo, sizeof(IOParamRayCastCommon), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(&worldVolume, io.worldVolumeAddr, sizeof(WorldVolume), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			spu_dma_get(ray, eaRay, sizeof(Ray)*numStride, 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			castStrideRays(ray, numStride);

			spu_dma_put(ray, eaRay, sizeof(Ray)*numStride, 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			DEALLOCATE(ray);
		}
		break;
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}

