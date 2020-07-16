/*
 * heightfluid.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: mike
 */

#include "heightfluid/heightfluid.h"

#include "rigidbody/rigidbodies.h"

#include "heightFluid_task.h"

HeightFluid::HeightFluid(MARSTaskManager *task, s32 taskId, HeapManager *pool)
{
	mTask = task;
	mTaskId = taskId;

	mPool = pool;

	mHeightFluidSurfaces = NULL;
	mForces = NULL;
	mSurface = NULL;
	mNumHeightFluidSurfaces = 0;

	mRigidBodies = NULL;
}

void HeightFluid::allocateBuffers()
{
	#define ALLOCATE(align,size) mPool->allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16);

	mHeightFluidSurfaces = (HeightFluidSurface**)ALLOCATE(16, sizeof(HeightFluidSurface*)*heightfluidProperty.maxHeightFluidSurface);
	mForces = (Forces*)ALLOCATE(128, sizeof(Forces)*heightfluidProperty.maxInstances);

	#undef ALLOCATE
}

void HeightFluid::deallocateBuffers()
{
	#define DEALLOCATE(ptr) if(ptr) {mPool->deallocate(ptr); ptr = NULL;}

	DEALLOCATE(mForces);
	DEALLOCATE(mHeightFluidSurfaces);

	#undef DEALLOCATE
}

void HeightFluid::attachRigidBodies(RigidBodies *rb)
{
	ASSERT(rb);
	mRigidBodies = rb;
}

void HeightFluid::detachRigidBodies()
{
	mRigidBodies = NULL;
}

void HeightFluid::addHeightFluidSurface(HeightFluidSurface *surface)
{
	ASSERT(mNumHeightFluidSurfaces < heightfluidProperty.maxHeightFluidSurface);
	mHeightFluidSurfaces[mNumHeightFluidSurfaces++] = surface;
}

void HeightFluid::removeHeightFluidSurface(const HeightFluidSurface *surface)
{
	for(u32 i=0;i < mNumHeightFluidSurfaces;i++) {
		if(mHeightFluidSurfaces[i] == surface) {
			mHeightFluidSurfaces[i] = mHeightFluidSurfaces[--mNumHeightFluidSurfaces];
			break;
		}
	}
}

void HeightFluid::setupSimulate()
{
	memset(mForces, 0, sizeof(Forces)*heightfluidProperty.maxInstances);
}

void HeightFluid::finalizeSimulate()
{
	for(u32 s=0;s < mNumHeightFluidSurfaces;s++)
		mHeightFluidSurfaces[s]->swapBuffers();

	for(u32 b=0;b < mRigidBodies->getInstanceCount();b++)
		mRigidBodies->applyForce(b, mForces[b]);
}

void HeightFluid::reset()
{
	deallocateBuffers();
	allocateBuffers();

	mNumHeightFluidSurfaces = 0;
}

void HeightFluid::setup()
{
}

void HeightFluid::calcWaveSPU(f32 timeStep)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(heightFluid_task);

	HeightFluidIO *io = (HeightFluidIO*)mTask->allocate(sizeof(HeightFluidIO)*maxTasks);

	for(s32 i=0;i < maxTasks;i++) {
		io[i].numSPU = maxTasks;

		io[i].fieldAddr = (u32)(u64)mSurface->fields[mSurface->writeBuffer];
		io[i].fieldWidth = mSurface->fieldWidth;
		io[i].fieldDepth = mSurface->fieldDepth;
		io[i].fieldScale = mSurface->fieldScale;
		io[i].fieldPosition = mSurface->fieldPosition;
		io[i].fieldOrientation = mSurface->fieldOrientation;
		io[i].calcWave.cConst = mSurface->cConst;
		io[i].calcWave.deltaX = mSurface->deltaX;
		io[i].calcWave.kConst = mSurface->kConst;
		io[i].calcWave.heightMin = mSurface->heightMin;
		io[i].calcWave.heightMax = mSurface->heightMax;
		io[i].calcWave.timeStep = timeStep;

		mTask->startTask(i, (u32)((u64)&io[i]), HEIGHTFLUID_CALCWAVE, 0, 0);
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	}

	mTask->deallocate(io);
}

void HeightFluid::calcPositionAndNormalSPU()
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(heightFluid_task);

	HeightFluidIO *io = (HeightFluidIO*)mTask->allocate(sizeof(HeightFluidIO)*maxTasks);

	for(s32 i=0;i < maxTasks;i++) {
		io[i].numSPU = maxTasks;

		io[i].fieldAddr = (u32)(u64)mSurface->fields[mSurface->writeBuffer];
		io[i].fieldWidth = mSurface->fieldWidth;
		io[i].fieldDepth = mSurface->fieldDepth;
		io[i].fieldScale = mSurface->fieldScale;
		io[i].fieldPosition = mSurface->fieldPosition;
		io[i].fieldOrientation = mSurface->fieldOrientation;
		io[i].calcSurface.vtxAddr = (u32)(u64)mSurface->hfVertex[mSurface->writeBuffer];
		io[i].calcSurface.nmlAddr = (u32)(u64)mSurface->hfNormal[mSurface->writeBuffer];
		io[i].calcSurface.amplitude = mSurface->amplitude;

		mTask->startTask(i, (u32)((u64)&io[i]), HEIGHTFLUID_CALCMESH, 0, 0);
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	}

	mTask->deallocate(io);
}

void HeightFluid::setMotionSPU()
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(heightFluid_task);

	HeightFluidIO *io = (HeightFluidIO*)mTask->allocate(sizeof(HeightFluidIO)*maxTasks);

	f32 reducedWater = 0.0f;

	// Stage1
	for(s32 i=0;i < maxTasks;i++) {
		io[i].numSPU = maxTasks;

		io[i].fieldAddr = (u32)(u64)mSurface->fields[mSurface->writeBuffer];
		io[i].fieldWidth = mSurface->fieldWidth;
		io[i].fieldDepth = mSurface->fieldDepth;
		io[i].fieldScale = mSurface->fieldScale;
		io[i].fieldPosition = mSurface->fieldPosition;
		io[i].fieldOrientation = mSurface->fieldOrientation;
		io[i].setMotion.reducedWater = 0.0f;

		mTask->startTask(i, (u32)((u64)&io[i]), HEIGHTFLUID_SETMOTION_STAGE1, 0, 0);
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		reducedWater += io[taskId].setMotion.reducedWater;
	}

	reducedWater = mSurface->totalWater - reducedWater;
	reducedWater /= (f32)(mSurface->fieldWidth*mSurface->fieldDepth);

	// Stage2
	for(s32 i=0;i < maxTasks;i++) {
		io[i].numSPU = maxTasks;

		io[i].fieldAddr = (u32)(u64)mSurface->fields[mSurface->writeBuffer];
		io[i].fieldWidth = mSurface->fieldWidth;
		io[i].fieldDepth = mSurface->fieldDepth;
		io[i].fieldScale = mSurface->fieldScale;
		io[i].fieldPosition = mSurface->fieldPosition;
		io[i].fieldOrientation = mSurface->fieldOrientation;
		io[i].setMotion.reducedWater = reducedWater;

		mTask->startTask(i, (u32)((u64)&io[i]), HEIGHTFLUID_SETMOTION_STAGE2, 0, 0);
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	}

	mTask->deallocate(io);
}

void HeightFluid::applyForceToRigidBodiesSPU()
{
	int maxTasks = (int)mTask->getMaxCores();

	mTask->setTaskEntry(heightFluid_task);

	HeightFluidIO *io = (HeightFluidIO*)mTask->allocate(sizeof(HeightFluidIO)*maxTasks);

	u32 *commonBuff = (u32*)mPool->allocate(sizeof(u32)*32, HeapManager::ALIGN128);
	memset(commonBuff, 0, sizeof(u32)*32);

	s32 numBatch = (mRigidBodies->numInstances + maxTasks - 1)/maxTasks;
	s32 restStates = mRigidBodies->numInstances;
	int startState = 0;

	for(s32 i=0;i < maxTasks;i++) {
		s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;

		io[i].numSPU = maxTasks;

		io[i].fieldAddr          = (u32)(u64)mSurface->fields[mSurface->writeBuffer];
		io[i].fieldWidth		  = mSurface->fieldWidth;
		io[i].fieldDepth		  = mSurface->fieldDepth;
		io[i].fieldScale		  = mSurface->fieldScale;
		io[i].fieldPosition	  = mSurface->fieldPosition;
		io[i].fieldOrientation	  = mSurface->fieldOrientation;
		io[i].applyForces.commonAddr	  = (u32)(u64)commonBuff;
		io[i].applyForces.forcesAddr	  = (u32)(u64)mForces;
		io[i].applyForces.statesAddr	  = (u32)(u64)mRigidBodies->statesBuffer[mRigidBodies->readBuffer];
		io[i].applyForces.collsAddr	  = (u32)(u64)mRigidBodies->collObjs;
		io[i].applyForces.startState	  = startState;
		io[i].applyForces.numStates	  = curBatch;
		io[i].applyForces.buoyPower	  = mSurface->buoyPower;
		io[i].applyForces.limitHeight	  = mSurface->limitHeight;
		io[i].applyForces.amplitude	  = mSurface->amplitude;
		io[i].applyForces.buoyDamping	  = mSurface->buoyDamping;
		io[i].applyForces.downCurrWave	  = mSurface->downCurrWave;
		io[i].applyForces.downPrevWave	  = mSurface->downPrevWave;

		mTask->startTask(i, (u32)((u64)&io[i]), HEIGHTFLUID_APPLY_FORCES, 0, 0);

		restStates -= curBatch;
		startState += curBatch;
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	}

	mPool->deallocate(commonBuff);

	mTask->deallocate(io);
}

void HeightFluid::spuSimulate(f32 timeStep, u32 flag)
{
	ASSERT(mRigidBodies);

	for(u32 s=0;s < mNumHeightFluidSurfaces;s++) {
		mSurface = mHeightFluidSurfaces[s];
		mSurface->copyFields();

		mFields = mSurface->fields[mSurface->writeBuffer];
		mVertex = mSurface->hfVertex[mSurface->writeBuffer];
		mNormal = mSurface->hfNormal[mSurface->writeBuffer];
		mTexCoords = mSurface->hfTexCoords;
		mIndices = mSurface->hfMeshIndices;

		calcWaveSPU(timeStep);

		applyForceToRigidBodiesSPU();

		setMotionSPU();

		calcPositionAndNormalSPU();
	}
}
