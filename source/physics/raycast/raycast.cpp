/*
 * raycast.cpp
 *
 *  Created on: Jun 10, 2013
 *      Author: mike
 */

#include "raycast/raycast.h"

#include "rigidbody/rigidbodies.h"

#include "rayCast_task.h"

#define ALLOCATE(align, size) 	mPool->allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16);
#define DEALLOCATE(ptr) 		if(ptr) {mPool->deallocate((void*)(u64)ptr); ptr = 0;}

RayCast::RayCast(MARSTaskManager *task, s32 taskId, HeapManager *pool)
{
	mTask = task;
	mTaskId = taskId;

	mPool = pool;

	mRigidBodies = NULL;
	states = NULL;
	collObjs = NULL;
	numInstances = 0;

	nonContactFlag = NULL;
}

RayCast::~RayCast()
{
	deallocateBuffers();
}

void RayCast::reset()
{
	deallocateBuffers();
	allocateBuffers();

	clearNonContactFlag();
}

void RayCast::allocateBuffers()
{
	sizeContactTable = raycastProperty.maxInstances*raycastProperty.maxRayGroups;
	sizeContactTable = ALIGN128((sizeContactTable + 31)/32, sizeof(u32));
	nonContactFlag = (u32*)ALLOCATE(128, sizeof(u32)*sizeContactTable);
}

void RayCast::deallocateBuffers()
{
	DEALLOCATE(nonContactFlag);
}

void RayCast::setupWorldSize()
{
	if(raycastProperty.useRigidBodyWorldSize)
		worldVolume.setWorldSize(mRigidBodies->worldProperty.worldCenter, mRigidBodies->worldProperty.worldExtent);
	else
		worldVolume.setWorldSize(raycastProperty.worldCenter, raycastProperty.worldExtent);
}

void RayCast::getWorldSize(Vector3& center, Vector3& extent)
{
	center = raycastProperty.worldCenter;
	extent = raycastProperty.worldExtent;
}

void RayCast::setWorldSize(const Vector3& center, const Vector3& extent)
{
	raycastProperty.worldCenter = center;
	raycastProperty.worldExtent = extent;
	setupWorldSize();
}

void RayCast::appendNonContactPair(u8 rayGroup, u16 rigidbodyIndex)
{
	u32 idx = rayGroup*raycastProperty.maxRayGroups + rigidbodyIndex;
	nonContactFlag[idx>>5] |= 1L<<(idx&31);
}

void RayCast::removeNonContactPair(u8 rayGroup, u16 rigidbodyIndex)
{
	u32 idx = rayGroup*raycastProperty.maxRayGroups + rigidbodyIndex;
	nonContactFlag[idx>>5] = nonContactFlag[idx>>5]&~(1L<<(idx&31));
}

void RayCast::clearNonContactFlag()
{
	memset(nonContactFlag, 0, sizeof(u32)*sizeContactTable);
}

void RayCast::attachRigidBodies(RigidBodies *rb)
{
	ASSERT(rb);
	mRigidBodies = rb;
}

void RayCast::detachRigidBodies()
{
	mRigidBodies = NULL;
}

void RayCast::setupRayCast()
{
	ASSERT(mRigidBodies);

	states = mRigidBodies->statesBuffer[mRigidBodies->readBuffer];
	collObjs = mRigidBodies->collObjs;
	numInstances = mRigidBodies->numInstances;

	setupWorldSize();

	aabbArray[0] = (SortData*)ALLOCATE(128, sizeof(SortData)*numInstances);
	aabbArray[1] = (SortData*)ALLOCATE(128, sizeof(SortData)*numInstances);
	aabbArray[2] = (SortData*)ALLOCATE(128, sizeof(SortData)*numInstances);
	aabbArray[3] = (SortData*)ALLOCATE(128, sizeof(SortData)*numInstances);
	aabbArray[4] = (SortData*)ALLOCATE(128, sizeof(SortData)*numInstances);
	aabbArray[5] = (SortData*)ALLOCATE(128, sizeof(SortData)*numInstances);

	broadphaseOk = false;
}

void RayCast::finalizeRayCast()
{
	DEALLOCATE(aabbArray[5]);
	DEALLOCATE(aabbArray[4]);
	DEALLOCATE(aabbArray[3]);
	DEALLOCATE(aabbArray[2]);
	DEALLOCATE(aabbArray[1]);
	DEALLOCATE(aabbArray[0]);
}

void RayCast::broadPhaseSPU(SortData *aabbArray[6], s32& numAabb)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rayCast_task);

	numAabb = 0;

	{
		IOParamBroadPhase *io = (IOParamBroadPhase*)mTask->allocate(sizeof(IOParamBroadPhase)*maxTasks);
		s32 numBatch = (numInstances + maxTasks - 1)/maxTasks;
		s32 restStates = numInstances;
		s32 startState = 0;
		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restStates - numBatch) > 0 ? numBatch : restStates;
			io[i].statesAddr = (u32)(u64)states;
			io[i].numStates = numInstances;
			io[i].batchStartState = startState;
			io[i].numBatchStates = curBatch;
			io[i].aabbAddr[0] = (u32)(u64)ALLOCATE(16, sizeof(SortData)*curBatch);
			io[i].aabbAddr[1] = (u32)(u64)ALLOCATE(16, sizeof(SortData)*curBatch);
			io[i].aabbAddr[2] = (u32)(u64)ALLOCATE(16, sizeof(SortData)*curBatch);
			io[i].aabbAddr[3] = (u32)(u64)ALLOCATE(16, sizeof(SortData)*curBatch);
			io[i].aabbAddr[4] = (u32)(u64)ALLOCATE(16, sizeof(SortData)*curBatch);
			io[i].aabbAddr[5] = (u32)(u64)ALLOCATE(16, sizeof(SortData)*curBatch);
			io[i].numAabb = 0;
			io[i].worldVolumeAddr = (u32)((u64)&worldVolume);
			mTask->startTask(i, (u32)((u64)&io[i]), RAYCAST_ASSIGNSTATES, 0, 0);
			restStates -= curBatch;
			startState += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0],  buf[1], buf[2], buf[3]);
			memcpy(aabbArray[0] + numAabb, (void*)(u64)io[taskId].aabbAddr[0], sizeof(SortData)*io[taskId].numAabb);
			memcpy(aabbArray[1] + numAabb, (void*)(u64)io[taskId].aabbAddr[1], sizeof(SortData)*io[taskId].numAabb);
			memcpy(aabbArray[2] + numAabb, (void*)(u64)io[taskId].aabbAddr[2], sizeof(SortData)*io[taskId].numAabb);
			memcpy(aabbArray[3] + numAabb, (void*)(u64)io[taskId].aabbAddr[3], sizeof(SortData)*io[taskId].numAabb);
			memcpy(aabbArray[4] + numAabb, (void*)(u64)io[taskId].aabbAddr[4], sizeof(SortData)*io[taskId].numAabb);
			memcpy(aabbArray[5] + numAabb, (void*)(u64)io[taskId].aabbAddr[5], sizeof(SortData)*io[taskId].numAabb);
			numAabb += io[taskId].numAabb;
		}

		for(s32 i=maxTasks - 1;i >= 0;i--) {
			DEALLOCATE(io[i].aabbAddr[5]);
			DEALLOCATE(io[i].aabbAddr[4]);
			DEALLOCATE(io[i].aabbAddr[3]);
			DEALLOCATE(io[i].aabbAddr[2]);
			DEALLOCATE(io[i].aabbAddr[1]);
			DEALLOCATE(io[i].aabbAddr[0]);
		}
		mTask->deallocate(io);
	}

	{
		IOParamSortAabbs *io = (IOParamSortAabbs*)mTask->allocate(sizeof(IOParamSortAabbs));
		SortData *tmpSorts = (SortData*)ALLOCATE(16, sizeof(SortData)*numAabb);
		for(s32 i=0;i < maxTasks;i++) {
			io->numSpu = maxTasks;
			io->buffAddr = (u32)(u64)tmpSorts;
			io->aabbAddr[0] = (u32)(u64)aabbArray[0];
			io->aabbAddr[1] = (u32)(u64)aabbArray[1];
			io->aabbAddr[2] = (u32)(u64)aabbArray[2];
			io->aabbAddr[3] = (u32)(u64)aabbArray[3];
			io->aabbAddr[4] = (u32)(u64)aabbArray[4];
			io->aabbAddr[5] = (u32)(u64)aabbArray[5];
			io->numAabb = numAabb;
			mTask->startTask(i, (u32)(u64)io, RAYCAST_SORTAABB, 0, 0);
		}
		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}
		DEALLOCATE(tmpSorts);
		mTask->deallocate(io);
	}
}

void RayCast::spuRayCastBroadphase()
{
	broadPhaseSPU(aabbArray, numAabb);

	mRayCastCommonIO.statesAddr = (u32)(u64)states;
	mRayCastCommonIO.collsAddr = (u32)(u64)collObjs;
	mRayCastCommonIO.numStates = numInstances;

	mRayCastCommonIO.aabbAddr[0] = (u32)(u64)aabbArray[0];
	mRayCastCommonIO.aabbAddr[1] = (u32)(u64)aabbArray[1];
	mRayCastCommonIO.aabbAddr[2] = (u32)(u64)aabbArray[2];
	mRayCastCommonIO.aabbAddr[3] = (u32)(u64)aabbArray[3];
	mRayCastCommonIO.aabbAddr[4] = (u32)(u64)aabbArray[4];
	mRayCastCommonIO.aabbAddr[5] = (u32)(u64)aabbArray[5];
	mRayCastCommonIO.numAabb = numAabb;

	mRayCastCommonIO.nonContactFlagAddr = (u32)(u64)nonContactFlag;
	mRayCastCommonIO.numNonContactFlag = sizeContactTable;

	mRayCastCommonIO.worldVolumeAddr = (u32)((u64)&worldVolume);
	mRayCastCommonIO.maxRayGroups = raycastProperty.maxRayGroups;
}

void RayCast::spuRayCast(Ray *rays, s32 numRay, u32 flag)
{
	if(!broadphaseOk) {
		spuRayCastBroadphase();
		broadphaseOk = true;
	}

	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rayCast_task);

	s32 startRay = 0;
	s32 numBatch = (numRay + maxTasks - 1)/maxTasks;
	s32 restRays = numRay;
	for(s32 i=0;i < maxTasks;i++) {
		s32 curBatch = (restRays - numBatch) > 0 ? numBatch : restRays;
		mTask->startTask(i, (u32)((u64)&mRayCastCommonIO), RAYCAST_RAYCAST_STRIDE, (u32)((u64)&rays[startRay]), curBatch);
		startRay += numBatch;
		restRays -= numBatch;
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0],  buf[1], buf[2], buf[3]);
	}
}
