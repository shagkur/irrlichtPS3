/*
 * particles.cpp
 *
 *  Created on: Jan 27, 2014
 *      Author: mike
 */

#include "particle/particles.h"

#include "pclBroadphase_task.h"
#include "pclCollision_task.h"
#include "pclSolver_task.h"
#include "pclMesh_task.h"

#define ALLOCATE(align, size) 	mPool->allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16);
#define DEALLOCATE(ptr) 		if(ptr) {mPool->deallocate((void*)(u64)ptr); ptr = 0;}

Particles::Particles(MARSTaskManager *task, s32 taskId, HeapManager *pool)
{
	mTask = task;
	mTaskId = taskId;

	mPool = pool;

	particleGroups = NULL;
	extForces = NULL;
	contactsPcl = NULL;
	contactsRig = NULL;
	contactSortsPcl = NULL;
	contactSortsRig = NULL;
	numContactPairsPcl = 0;
	numContactPairsRig = 0;
	numParticleGroups = 0;
}

void Particles::allocateBuffers()
{
	particleGroups = (ParticleGroup**)ALLOCATE(16, sizeof(ParticleGroup*)*worldProperty.maxParticleGroups);
	extForces = (Vector3*)ALLOCATE(128, sizeof(Vector3)*worldProperty.maxInstances);
	contactsPcl = (PclContactPair*)ALLOCATE(128, sizeof(PclContactPair)*worldProperty.maxContactPairs);
	contactsRig = (PclContactPair*)ALLOCATE(128, sizeof(PclContactPair)*worldProperty.maxContactPairs);
	contactSortsPcl = (SortData*)ALLOCATE(128, sizeof(SortData)*worldProperty.maxContactPairs);
	contactSortsRig = (SortData*)ALLOCATE(128, sizeof(SortData)*worldProperty.maxContactPairs);
}

void Particles::deallocateBuffers()
{
	DEALLOCATE(contactSortsRig);
	DEALLOCATE(contactSortsPcl);
	DEALLOCATE(contactsRig);
	DEALLOCATE(contactsPcl);
	DEALLOCATE(extForces);
	DEALLOCATE(particleGroups);
}

void Particles::setupWorldSize()
{
	if(worldProperty.useRigidBodyWorldSize)
		worldVolume.setWorldSize(mRigidBodies->worldProperty.worldCenter, mRigidBodies->worldProperty.worldExtent);
	else
		worldVolume.setWorldSize(worldProperty.worldCenter, worldProperty.worldExtent);
}

void Particles::getWorldSize(Vector3& center, Vector3& extent)
{
	center = worldProperty.worldCenter;
	extent = worldProperty.worldExtent;
}

void Particles::setWorldSize(const Vector3& center, const Vector3& extent)
{
	worldProperty.worldCenter = center;
	worldProperty.worldExtent = extent;
}

void Particles::attachRigidBodies(RigidBodies *rb)
{
	mRigidBodies = rb;
}

void Particles::detachRigidBodies()
{
	mRigidBodies = NULL;
}

void Particles::addParticleGroup(ParticleGroup *pclGroup)
{
	particleGroups[numParticleGroups++] = pclGroup;
}

void Particles::removeParticleGroup(const ParticleGroup *pclGroup)
{
	for(u32 i=0;i < numParticleGroups;i++) {
		if(particleGroups[i] == pclGroup) {
			particleGroups[i] = particleGroups[--numParticleGroups];
			break;
		}
	}
}

void Particles::reset()
{
	deallocateBuffers();
	allocateBuffers();

	writeBuffer = 0;
	readBuffer = 1;

	numParticleGroups = 0;
	numContactPairsPcl = 0;
	numContactPairsRig = 0;

	for(u32 i=0;i < worldProperty.maxContactPairs;i++) {
		setPair(contactSortsPcl[i], i);
		setPair(contactSortsRig[i], i);
	}
}

void Particles::setup()
{
	colls = mRigidBodies->collObjs;
	bodies = mRigidBodies->bodies;

	setupWorldSize();

	for(u32 i=0;i < numParticleGroups;i++) {
		particleGroups[i]->writeBuffer = writeBuffer;
		particleGroups[i]->readBuffer = readBuffer;
	}
}

void Particles::setupSimulate()
{
	states = mRigidBodies->statesBuffer[mRigidBodies->readBuffer];
	numInstances = mRigidBodies->numInstances;
	numBodies = mRigidBodies->numBodies;
}

void Particles::finalizeSimulate()
{
	writeBuffer = 1 - writeBuffer;
	readBuffer = 1 - readBuffer;

	for(u32 i=0;i < numParticleGroups;i++) {
		particleGroups[i]->writeBuffer = writeBuffer;
		particleGroups[i]->readBuffer = readBuffer;
	}
}

void Particles::spuSimulate(f32 timeStep)
{
	for(u32 i=0;i < numParticleGroups;i++)
		memcpy(particleGroups[i]->pclStates[writeBuffer], particleGroups[i]->pclStates[readBuffer], sizeof(PclState)*particleGroups[i]->property.numParticles);

	preBroadPhaseRigSPU(timeStep);

	for(u32 subStep=0;subStep < worldProperty.subStepCount;subStep++) {
		f32 subTimeStep = timeStep/(f32)worldProperty.subStepCount;

		for(u32 i=0;i < numParticleGroups;i++) {
			curProperty = particleGroups[i]->property;
			pclStates = particleGroups[i]->pclStates[writeBuffer];
			pclJoints = particleGroups[i]->pclJoints;

			broadPhasePclSPU(subTimeStep);

			detectCollisionsSPU(subTimeStep);

			refreshContactPairsSPU();

			solveConstraintsSPU(subTimeStep);

			integrateSPU(subTimeStep);

			if(worldProperty.contactCallback != NULL) {
				if(numContactPairsPcl > 0)
					worldProperty.contactCallback->onContactPclPcl(particleGroups[i], contactsPcl, numContactPairsPcl);

				if(numContactPairsRig > 0)
					worldProperty.contactCallback->onContactPclRig(particleGroups[i], contactsRig, numContactPairsRig);
			}
		}
	}

	for(u32 i=0;i < numParticleGroups;i++) {
		if(particleGroups[i]->property.particleType != ParticleTypeGroup) {
			pclStates = particleGroups[i]->pclStates[writeBuffer];
			pclJoints = particleGroups[i]->pclJoints;
			pclVtx = particleGroups[i]->pclVertices;
			pclNml = particleGroups[i]->pclNormals;
			pclIdx = particleGroups[i]->pclIndices;
			numVertices = particleGroups[i]->numVertices;
			numIndices = particleGroups[i]->numIndices;

			buildMeshSPU();
		}
	}

	postBroadPhaseRigSPU();
}

void Particles::preBroadPhaseRigSPU(f32 timeStep)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(pclBroadphase_task);

	numRigAabb = 0;
	rigAabbArray[0] = (SortData*)ALLOCATE(16, sizeof(SortData*)*numInstances);
	rigAabbArray[1] = (SortData*)ALLOCATE(16, sizeof(SortData*)*numInstances);
	rigAabbArray[2] = (SortData*)ALLOCATE(16, sizeof(SortData*)*numInstances);
	{
		IOParamAssignStatesPcl *io = (IOParamAssignStatesPcl*)mTask->allocate(sizeof(IOParamAssignStatesPcl)*maxTasks);
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
			io[i].numAabb = 0;
			io[i].worldVolumeAddr = (u32)((u64)&worldVolume);
			io[i].timeStep = timeStep;
			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_ASSIGNSTATES_RIG, 0, 0);
			restStates -= curBatch;
			startState += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			memcpy(rigAabbArray[0] + numRigAabb, (void*)(u64)io[taskId].aabbAddr[0], sizeof(SortData)*io[taskId].numAabb);
			memcpy(rigAabbArray[1] + numRigAabb, (void*)(u64)io[taskId].aabbAddr[1], sizeof(SortData)*io[taskId].numAabb);
			memcpy(rigAabbArray[2] + numRigAabb, (void*)(u64)io[taskId].aabbAddr[2], sizeof(SortData)*io[taskId].numAabb);
			numRigAabb += io[taskId].numAabb;
		}

		for(s32 i=maxTasks - 1;i >= 0;i--) {
			DEALLOCATE(io[i].aabbAddr[2]);
			DEALLOCATE(io[i].aabbAddr[1]);
			DEALLOCATE(io[i].aabbAddr[0]);
		}

		mTask->deallocate(io);
	}

	{
		IOParamSortPcl *io = (IOParamSortPcl*)mTask->allocate(sizeof(IOParamSortPcl));

		for(s32 j=0;j < 3;j++) {
			SortData *tmpSorts = (SortData*)ALLOCATE(16, sizeof(SortData)*numRigAabb);

			for(s32 i=0;i < maxTasks;i++) {
				io->numSpu = maxTasks;
				io->buffAddr = (u32)(u64)tmpSorts;
				io->sortsAddr = (u32)(u64)rigAabbArray[j];
				io->numSorts = numRigAabb;
				mTask->startTask(i, (u32)(u64)io, PARTICLE_SORT, 0, 0);
			}
			for(s32 i=0;i<maxTasks;i++) {
				s32 taskId;
				u32 buf[4];

				mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			}

			DEALLOCATE(tmpSorts);
		}

		mTask->deallocate(io);
	}
}

void Particles::postBroadPhaseRigSPU()
{
	DEALLOCATE(rigAabbArray[2]);
	DEALLOCATE(rigAabbArray[1]);
	DEALLOCATE(rigAabbArray[0]);
}

void Particles::broadPhasePclSPU(f32 timeStep)
{
	s32 chkAxis = 0;
	u32 numPclAabb = 0;
	SortData *pclAabbArray = (SortData*)ALLOCATE(16, sizeof(SortData)*curProperty.numParticles);

	assignStatesSPU(timeStep, pclAabbArray, numPclAabb, chkAxis);
	detectPairsSPU(pclAabbArray, numPclAabb, chkAxis);

	DEALLOCATE(pclAabbArray);
}

void Particles::assignStatesSPU(f32 timeStep, SortData *pclAabbArray, u32& numPclAabb, s32& chkAxis)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(pclBroadphase_task);
	{
		IOParamAssignStatesPcl *io = (IOParamAssignStatesPcl*)mTask->allocate(sizeof(IOParamAssignStatesPcl)*maxTasks);
		s32 numBatch = (curProperty.numParticles + maxTasks - 1)/maxTasks;
		s32 restStates = curProperty.numParticles;
		s32 startState = 0;
		Vector3 s(0.0f),s2(0.0f);

		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;
			io[i].statesAddr = (u32)(u64)pclStates;
			io[i].numStates = curProperty.numParticles;
			io[i].batchStartState = startState;
			io[i].numBatchStates = curBatch;
			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_CALCVARIANCE_PCL, 0, 0);
			restStates -= curBatch;
			startState += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			s += io[taskId].s;
			s2 += io[taskId].s2;
		}

		Vector3 v = s2 - mulPerElem(s, s)/numPclAabb;
		if(v[1] > v[0]) chkAxis = 1;
		if(v[2] > v[chkAxis]) chkAxis = 2;

		mTask->deallocate(io);
	}

	{
		IOParamAssignStatesPcl *io = (IOParamAssignStatesPcl*)mTask->allocate(sizeof(IOParamAssignStatesPcl)*maxTasks);

		for(s32 i=0;i < maxTasks;i++) {
			io[i].chkAxis = chkAxis;
			io[i].aabbAddr[0] = (u32)(u64)ALLOCATE(16,sizeof(SortData)*io[i].numBatchStates);
			io[i].numAabb = 0;
			io[i].worldVolumeAddr = (u32)((u64)&worldVolume);
			io[i].timeStep = timeStep;
			mTask->startTask(i,(u32)((u64)&io[i]), PARTICLE_ASSIGNSTATES_PCL, 0, 0);
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			memcpy(pclAabbArray + numPclAabb, (void*)(u64)io[taskId].aabbAddr[0], sizeof(SortData)*io[taskId].numAabb);
			numPclAabb += io[taskId].numAabb;
		}

		for(int i=maxTasks-1;i>=0;i--) {
			DEALLOCATE(io[i].aabbAddr[0]);
		}

		mTask->deallocate(io);
	}

	{
		IOParamSortPcl *io = (IOParamSortPcl*)mTask->allocate(sizeof(IOParamSortPcl));
		SortData *tmpSorts = (SortData*)ALLOCATE(16, sizeof(SortData)*numPclAabb);

		for(s32 i=0;i < maxTasks;i++) {
			io->numSpu = maxTasks;
			io->buffAddr = (u32)(u64)tmpSorts;
			io->sortsAddr = (u32)(u64)pclAabbArray;
			io->numSorts = numPclAabb;
			mTask->startTask(i, (u32)(u64)io, PARTICLE_SORT, 0, 0);
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

void Particles::detectPairsSPU(SortData *pclAabbArray, u32 numPclAabb, s32 chkAxis)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(pclBroadphase_task);

	u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
	u32 div = maxTasks*4;
	u32 maxSorts = worldProperty.maxContactPairs/maxTasks;

	numContactPairsPcl = 0;

	IOParamDetectPairsPcl *io = (IOParamDetectPairsPcl*)mTask->allocate(sizeof(IOParamDetectPairsPcl)*maxTasks);
	if(curProperty.selfCollisionEnable) {
		u32 numBatch = (numPclAabb + div - 1)/div;
		commonBuff[0] = 0;					// 0 : unlocked , 1 : locked
		commonBuff[1] = 0;					// start pos
		commonBuff[2] = MIN(numBatch, 32);	// batch num

		for(s32 i=0;i < maxTasks;i++) {
			io[i].chkAxis = chkAxis;
			io[i].pclStatesAddr = (u32)(u64)pclStates;
			io[i].numPclStates = curProperty.numParticles;
			io[i].pclAabbAddr = (u32)(u64)pclAabbArray;
			io[i].numPclAabb = numPclAabb;
			io[i].tmpSortsAddr = (u32)(u64)ALLOCATE(128, sizeof(SortData)*maxSorts);
			io[i].numTmpSorts = 0;
			io[i].maxSorts = maxSorts;
			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_DETECTPAIRS_PCL, (u32)(u64)commonBuff, 0);
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);

			for(u32 j=0;j < io[taskId].numTmpSorts && numContactPairsPcl < worldProperty.maxContactPairs;j++) {
				SortData& sort = contactSortsPcl[numContactPairsPcl];
				SortData& newSort = *((SortData*)((u64)io[taskId].tmpSortsAddr + j));
				PclContactPair& pair = contactsPcl[getPair(sort)];
				pair.numContacts = 0;
				pair.stateIndex[0] = getStateA(newSort);
				pair.stateIndex[1] = getStateB(newSort);
				pair.contactPoint.distance = FLT_MAX;
				setKey(sort, getKey(newSort));
				setStateA(sort, getStateA(newSort));
				setStateB(sort, getStateB(newSort));
				setFlag(sort, 1);
				numContactPairsPcl++;
			}
		}

		for(int i=maxTasks-1;i>=0;i--) {
			DEALLOCATE(io[i].tmpSortsAddr);
		}
	}

	numContactPairsRig = 0;
	{
		u32 numBatch = (numPclAabb + div - 1)/div;
		commonBuff[0] = 0;					// 0 : unlocked , 1 : locked
		commonBuff[1] = 0;					// start pos
		commonBuff[2] = MIN(numBatch, 32);	// batch num

		for(s32 i=0;i < maxTasks;i++) {
			io[i].chkAxis = chkAxis;
			io[i].contactFilterSelf = curProperty.contactFilterSelf;
			io[i].contactFilterTarget = curProperty.contactFilterTarget;

			// ステート
			io[i].pclStatesAddr = (u32)(u64)pclStates;
			io[i].numPclStates = curProperty.numParticles;
			io[i].rigStatesAddr = (u32)(u64)states;
			io[i].numRigStates = numInstances;

			// ソート済みAABB配列
			io[i].pclAabbAddr = (u32)(u64)pclAabbArray;
			io[i].numPclAabb = numPclAabb;
			io[i].rigAabbAddr = (u32)(u64)rigAabbArray[chkAxis];
			io[i].numRigAabb = numRigAabb;

			// コンタクトペア出力
			io[i].tmpSortsAddr = (u32)(u64)ALLOCATE(128, sizeof(SortData)*maxSorts);
			io[i].numTmpSorts = 0;
			io[i].maxSorts = worldProperty.maxContactPairs;
			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_DETECTPAIRS_RIG, (u32)(u64)commonBuff, 0);
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId,buf[0], buf[1], buf[2], buf[3]);

			ASSERT(numContactPairsRig + io[taskId].numTmpSorts <= worldProperty.maxContactPairs);
			for(u32 j=0;j < io[taskId].numTmpSorts;j++) {
				SortData& sort = contactSortsRig[numContactPairsRig];
				SortData& newSort = *((SortData*)((u64)io[taskId].tmpSortsAddr + j));
				PclContactPair& pair = contactsRig[getPair(sort)];
				pair.numContacts = 0;
				pair.stateIndex[0] = getStateA(newSort);
				pair.stateIndex[1] = getStateB(newSort);
				pair.contactPoint.distance = FLT_MAX;
				setKey(sort, getKey(newSort));
				setStateA(sort, getStateA(newSort));
				setStateB(sort, getStateB(newSort));
				setBodyB(sort, states[pair.stateIndex[1]].trbBodyIdx);
				setFlag(sort, 1);
				numContactPairsRig++;
			}
		}

		for(s32 i=maxTasks - 1;i >= 0;i--)
			DEALLOCATE(io[i].tmpSortsAddr);
	}

	// contact pair check
	//for(int i=0;i<numContactPairsPcl;i++) {
	//	PRINTF("added pcl %5d pair %5d state %3d %3d\n",i,Pair(contactSortsPcl[i]),StateA(contactSortsPcl[i]),StateB(contactSortsPcl[i]));
	//}
	//for(int i=0;i<numContactPairsRig;i++) {
	//	PRINTF("added  rb %5d pair %5d state %3d %3d(body %3d)\n",i,Pair(contactSortsRig[i]),StateA(contactSortsRig[i]),StateB(contactSortsRig[i]),BodyB(contactSortsRig[i]));
	//}

	mTask->deallocate(io);

	DEALLOCATE(commonBuff);
}

void Particles::refreshContactPairsSPU()
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(pclBroadphase_task);

	if(numContactPairsPcl > 0) {
		u32 numRemovedContactPairs = 0;
		IOParamRefreshPairsPcl *io = (IOParamRefreshPairsPcl*)mTask->allocate(sizeof(IOParamRefreshPairsPcl)*maxTasks);
		SortData *buffAddr = (SortData*)ALLOCATE(128, sizeof(SortData)*numContactPairsPcl);

		s32 numBatch = (numContactPairsPcl + maxTasks - 1)/maxTasks;
		s32 restPairs = numContactPairsPcl;
		s32 startPairs = 0;
		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restPairs - numBatch ) > 0 ? numBatch : restPairs;

			io[i].numSpu = maxTasks;
			io[i].startBatch = startPairs;
			io[i].numBatch = curBatch;
			io[i].contactPairsAddr = (u32)(u64)contactsPcl;
			io[i].buffAddr = (u32)(u64)buffAddr;
			io[i].sortsAddr = (u32)(u64)contactSortsPcl;
			io[i].numContactPairs = numContactPairsPcl;
			io[i].statesAddr = (u32)(u64)pclStates;
			io[i].numStates = curProperty.numParticles;
			io[i].numRemovedPairs = 0;
			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_REFRESHCONTACTPAIRS_PCL, 0, 0);

			restPairs -= curBatch;
			startPairs += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			numRemovedContactPairs += io[taskId].numRemovedPairs;
		}

		DEALLOCATE(buffAddr);

		mTask->deallocate(io);

		numContactPairsPcl -= numRemovedContactPairs;
	}

	if(numContactPairsRig > 0) {
		u32 numRemovedContactPairs = 0;
		IOParamRefreshPairsPcl *io = (IOParamRefreshPairsPcl*)mTask->allocate(sizeof(IOParamRefreshPairsPcl)*maxTasks);
		SortData *buffAddr = (SortData*)ALLOCATE(128, sizeof(SortData)*numContactPairsRig);

		s32 numBatch = (numContactPairsRig + maxTasks - 1)/maxTasks;
		s32 restPairs = numContactPairsRig;
		s32 startPairs = 0;
		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restPairs - numBatch ) > 0 ? numBatch : restPairs;

			io[i].numSpu = maxTasks;
			io[i].startBatch = startPairs;
			io[i].numBatch = curBatch;
			io[i].contactPairsAddr = (u32)(u64)contactsRig;
			io[i].buffAddr = (u32)(u64)buffAddr;
			io[i].sortsAddr = (u32)(u64)contactSortsRig;
			io[i].numContactPairs = numContactPairsRig;
			io[i].statesAddr = (u32)(u64)pclStates;
			io[i].numStates = curProperty.numParticles;
			io[i].numRemovedPairs = 0;
			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_REFRESHCONTACTPAIRS_RIG, 0, 0);

			restPairs -= curBatch;
			startPairs += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			numRemovedContactPairs += io[taskId].numRemovedPairs;
		}

		DEALLOCATE(buffAddr);

		mTask->deallocate(io);

		numContactPairsRig -= numRemovedContactPairs;
	}
}

void Particles::detectCollisionsSPU(f32 timeStep)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(pclCollision_task);

	u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
	u32 div = maxTasks*4;
	IOParamCollisionPcl *io = (IOParamCollisionPcl*)mTask->allocate(sizeof(IOParamCollisionPcl)*maxTasks);

	if(curProperty.selfCollisionEnable) {
		u32 numBatch = (numContactPairsPcl + div - 1)/div;

		commonBuff[0] = 0; // 0 : unlocked , 1 : locked
		commonBuff[1] = 0; // start pos
		commonBuff[2] = MIN(numBatch, 256); // batch num

		for(s32 i=0;i < maxTasks;i++) {
			io[i].contactPairsAddr = (u32)(u64)contactsPcl;
			io[i].numContactPairs = numContactPairsPcl;
			io[i].sortsAddr = (u32)(u64)contactSortsPcl;
			io[i].pclStatesAddr = (u32)(u64)pclStates;
			io[i].numPclStates = curProperty.numParticles;
			io[i].timeStep = timeStep;
			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_DETECTCOLLISIONS_PCL, (u32)(u64)commonBuff, 0);
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			uint32_t buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}
	}

	{
		u32 numBatch = (numContactPairsRig + div - 1)/div;

		commonBuff[0] = 0; // 0 : unlocked , 1 : locked
		commonBuff[1] = 0; // start pos
		commonBuff[2] = MIN(numBatch, 256); // batch num

		for(s32 i=0;i < maxTasks;i++) {
			io[i].contactPairsAddr = (u32)(u64)contactsRig;
			io[i].numContactPairs = numContactPairsRig;
			io[i].sortsAddr = (u32)(u64)contactSortsRig;
			io[i].pclStatesAddr = (u32)(u64)pclStates;
			io[i].numPclStates = curProperty.numParticles;
			io[i].rigStatesAddr = (u32)(u64)states;
			io[i].numRigStates = numInstances;
			io[i].collsAddr = (u32)(u64)colls;
			io[i].numBodies = numBodies;
			io[i].timeStep = timeStep;
			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_DETECTCOLLISIONS_RIG, (u32)(u64)commonBuff, 0);
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}
	}

	mTask->deallocate(io);

	DEALLOCATE(commonBuff);
}

void Particles::splitPairsExPcl(SolverInfo *info, SolverGroup *groups, SortData *pairs, u32 numPairs)
{
	s32 maxTasks = (s32)mTask->getMaxCores();
	s32 bufSize = sizeof(u8)*curProperty.numParticles;
	bufSize = ((bufSize + 127)>>7)<<7; // 128 bytes alignment
	u8 *stateTable = (u8*)ALLOCATE(128, bufSize);

	u32 *pairTable;
	pairTable = (u32*)ALLOCATE(16, sizeof(u32)*((numPairs + 31)/32));
	memset(pairTable, 0, sizeof(u32)*((numPairs + 31)/32));

	u32 targetCount = MAX(MIN_SOLVER_PAIRS, MIN(numPairs/(maxTasks*2), MAX_SOLVER_PAIRS));
	u32 startIndex = 0;

	u32 phaseId;
	u32 groupId;
	u32 totalCount=0;

	u32 maxGroups = MIN(maxTasks, MAX_SOLVER_GROUPS);

	for(phaseId=0;phaseId < MAX_SOLVER_PHASES && totalCount < numPairs;phaseId++) {
		bool startIndexCheck = true;

		info->numGroups[phaseId] = 0;

		u32 i = startIndex;

		memset(stateTable, 0xff, bufSize);

		for(groupId=0;i < numPairs && totalCount < numPairs && groupId < maxGroups;groupId++) {
			u32 pairCount = 0;

			SolverGroup& group = groups[phaseId*MAX_SOLVER_GROUPS + groupId];
			u32 pairId = 0;

			for(;i < numPairs && pairCount < targetCount;i++) {
				u32 idxP = i>>5;
				u32 maskP = 1L<<(i&31);

				if(pairTable[idxP]&maskP)
					continue;

				u32 idxA = getStateA(pairs[i]);
				u32 idxB = getStateB(pairs[i]);

				if((stateTable[idxA] != groupId && stateTable[idxA] != 0xff) ||
				   (stateTable[idxB] != groupId && stateTable[idxB] != 0xff))
				{
					startIndexCheck = false;
					continue;
				}

				stateTable[idxA] = groupId;
				stateTable[idxB] = groupId;

				if(startIndexCheck) startIndex++;

				pairTable[idxP] |= maskP;

				group.pairIndices[pairId++] = i;
				pairCount++;
			}

			info->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId] = (u16)pairId;
			totalCount += pairCount;
		}

		info->numGroups[phaseId] = groupId;
	}

	info->numPhases = phaseId;

	DEALLOCATE(pairTable);
	DEALLOCATE(stateTable);
}

void Particles::splitPairsExRig(SolverInfo *info, SolverGroup *groups, SortData *pairs, u32 numPairs)
{
	s32 maxTasks = (s32)mTask->getMaxCores();
	s32 pclBufSize = sizeof(u8)*curProperty.numParticles;
	pclBufSize = ((pclBufSize + 127)>>7)<<7; // 128 bytes alignment
	u8 *pclStateTable = (u8*)ALLOCATE(128, pclBufSize);

	s32 rigBufSize = sizeof(u8)*numInstances;
	rigBufSize = ((rigBufSize + 127)>>7)<<7; // 128 bytes alignment
	u8 *rigStateTable = (u8*)ALLOCATE(128, rigBufSize);

	u32 *pairTable;
	pairTable = (u32*)ALLOCATE(16, sizeof(u32)*((numPairs + 31)/32));
	memset(pairTable, 0, sizeof(u32)*((numPairs + 31)/32));

	u32 targetCount = MAX(MIN_SOLVER_PAIRS, MIN(numPairs/(maxTasks*2), MAX_SOLVER_PAIRS));
	u32 startIndex = 0;

	u32 phaseId;
	u32 groupId;
	u32 totalCount=0;

	u32 maxGroups = MIN(maxTasks, MAX_SOLVER_GROUPS);

	for(phaseId=0;phaseId < MAX_SOLVER_PHASES && totalCount < numPairs;phaseId++) {
		bool startIndexCheck = true;

		info->numGroups[phaseId] = 0;

		u32 i = startIndex;

		memset(pclStateTable, 0xff, pclBufSize);
		memset(rigStateTable, 0xff, rigBufSize);

		for(groupId=0;i < numPairs && totalCount < numPairs && groupId < maxGroups;groupId++) {
			u32 pairCount=0;

			SolverGroup& group = groups[phaseId*MAX_SOLVER_GROUPS + groupId];
			u32 pairId = 0;

			for(;i < numPairs && pairCount < targetCount;i++) {
				u32 idxP = i>>5;
				u32 maskP = 1L<<(i&31);

				if(pairTable[idxP]&maskP)
					continue;

				u32 idxA = getStateA(pairs[i]);
				u32 idxB = getStateB(pairs[i]);

				if((pclStateTable[idxA] != groupId && pclStateTable[idxA] != 0xff) ||
				   (rigStateTable[idxB] != groupId && rigStateTable[idxB] != 0xff))
				{
					startIndexCheck = false;
					continue;
				}

				pclStateTable[idxA] = groupId;
				rigStateTable[idxB] = groupId;

				if(startIndexCheck) startIndex++;

				pairTable[idxP] |= maskP;

				group.pairIndices[pairId++] = i;
				pairCount++;
			}

			info->numPairs[phaseId*MAX_SOLVER_GROUPS + groupId] = (u16)pairId;
			totalCount += pairCount;
		}

		info->numGroups[phaseId] = groupId;
	}

	info->numPhases = phaseId;

	DEALLOCATE(pairTable);
	DEALLOCATE(rigStateTable);
	DEALLOCATE(pclStateTable);
}

void Particles::solveConstraintsSPU(f32 timeStep)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(pclSolver_task);

	u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
	memset(commonBuff, 0, sizeof(u32)*32);

	//PRINTF("INPUT -----------------------------\n");
	//printRigs(states,numInstances);

	// Constraint Solver
	{
		SortData *jointSortsPcl = (SortData*)ALLOCATE(16, sizeof(SortData)*curProperty.numJoints);
		SortData *jointSortsRig = (SortData*)ALLOCATE(16, sizeof(SortData)*curProperty.numJoints);
		SolverGroup *contactGroupsPcl = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup)*MAX_SOLVER_PHASES*MAX_SOLVER_GROUPS);
		SolverInfo *contactSolverInfoPcl  = (SolverInfo*)ALLOCATE(128, sizeof(SolverInfo));
		SolverGroup *contactGroupsRig = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup)*MAX_SOLVER_PHASES*MAX_SOLVER_GROUPS);
		SolverInfo *contactSolverInfoRig  = (SolverInfo*)ALLOCATE(128, sizeof(SolverInfo));
		SolverGroup *jointGroupsPcl = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup)*MAX_SOLVER_PHASES*MAX_SOLVER_GROUPS);
		SolverInfo *jointSolverInfoPcl  = (SolverInfo*)ALLOCATE(128, sizeof(SolverInfo));
		SolverGroup *jointGroupsRig = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup)*MAX_SOLVER_PHASES*MAX_SOLVER_GROUPS);
		SolverInfo *jointSolverInfoRig  = (SolverInfo*)ALLOCATE(128, sizeof(SolverInfo));

		contactSolverInfoPcl->numPhases = 0;
		contactSolverInfoRig->numPhases = 0;
		jointSolverInfoPcl->numPhases = 0;
		jointSolverInfoRig->numPhases = 0;

		// Split Pairs
		if(numContactPairsPcl > 0)
			splitPairsExPcl(contactSolverInfoPcl, contactGroupsPcl, contactSortsPcl, numContactPairsPcl);

		if(numContactPairsRig > 0)
			splitPairsExRig(contactSolverInfoRig, contactGroupsRig, contactSortsRig, numContactPairsRig);

		if(curProperty.numJoints > 0) {
			u32 numActivePclJoints = 0;
			u32 numActiveRigJoints = 0;
			for(u32 i=0;i < curProperty.numJoints;i++) {
				PclJoint& joint = pclJoints[i];

				if(joint.type == PclJointTypePcl) {
					SortData& jpair = jointSortsPcl[numActivePclJoints];
					if(joint.active && pclStates[joint.stateIndexA].isActive() && pclStates[joint.stateIndexB].isActive()) {
						setStateA(jpair, joint.stateIndexA);
						setStateB(jpair, joint.stateIndexB);
						setPair(jpair, i);
						numActivePclJoints++;
					}
				} else {
					SortData& jpair = jointSortsRig[numActiveRigJoints];
					if(joint.active && pclStates[joint.stateIndexA].isActive()) {
						setStateA(jpair, joint.stateIndexA);
						setStateB(jpair, joint.stateIndexB);
						setPair(jpair, i);
						numActiveRigJoints++;
					}
				}
			}
			if(numActivePclJoints > 0)
				splitPairsExPcl(jointSolverInfoPcl, jointGroupsPcl, jointSortsPcl, numActivePclJoints);

			if(numActiveRigJoints > 0)
				splitPairsExRig(jointSolverInfoRig, jointGroupsRig, jointSortsRig, numActiveRigJoints);
		}

		//PRINTF("contactSolverInfoPcl\n");
		//printSolverInfo(contactSolverInfoPcl,contactGroupsPcl,contactSortsPcl);
		//PRINTF("contactSolverInfoRig\n");
		//printSolverInfo(contactSolverInfoRig,contactGroupsRig,contactSortsRig);
		//PRINTF("jointSolverInfoPcl\n");
		//printSolverInfo(jointSolverInfoPcl,jointGroupsPcl,jointSortsPcl);
		//PRINTF("jointSolverInfoRig\n");
		//printSolverInfo(jointSolverInfoRig,jointGroupsRig,jointSortsRig);

		IOParamSolverPcl *io = (IOParamSolverPcl*)mTask->allocate(sizeof(IOParamSolverPcl)*maxTasks);
		for(s32 i=0;i < maxTasks;i++) {
			io[i].contactsPclAddr = (u32)(u64)contactsPcl;
			io[i].contactsRigAddr = (u32)(u64)contactsRig;
			io[i].contactSortsPclAddr = (u32)(u64)contactSortsPcl;
			io[i].contactSortsRigAddr = (u32)(u64)contactSortsRig;

			io[i].jointsAddr = (u32)(u64)pclJoints;
			io[i].jointSortsPclAddr = (u32)(u64)jointSortsPcl;
			io[i].jointSortsRigAddr = (u32)(u64)jointSortsRig;

			io[i].pclStatesAddr = (u32)(u64)pclStates;
			io[i].rigStatesAddr = (u32)(u64)states;
			io[i].bodiesAddr = (u32)(u64)bodies;

			io[i].contactSolverInfoPclAddr = (u32)(u64)contactSolverInfoPcl;
			io[i].contactGroupsPclAddr = (u32)(u64)contactGroupsPcl;
			io[i].contactSolverInfoRigAddr = (u32)(u64)contactSolverInfoRig;
			io[i].contactGroupsRigAddr = (u32)(u64)contactGroupsRig;
			io[i].numCollIteration = curProperty.contactIteration;

			io[i].jointSolverInfoPclAddr = (u32)(u64)jointSolverInfoPcl;
			io[i].jointGroupsPclAddr = (u32)(u64)jointGroupsPcl;
			io[i].jointSolverInfoRigAddr = (u32)(u64)jointSolverInfoRig;
			io[i].jointGroupsRigAddr = (u32)(u64)jointGroupsRig;
			io[i].numJointIteration = curProperty.jointIteration;

			io[i].numSPU = maxTasks;
			io[i].numPclStates = curProperty.numParticles;
			io[i].numRigStates = numInstances;
			io[i].numBodies = numBodies;
			io[i].numContactPairsPcl = numContactPairsPcl;
			io[i].numContactPairsRig = numContactPairsRig;

			io[i].numJoints = curProperty.numJoints;

			io[i].timeStep = timeStep;
			io[i].separateBias = curProperty.separateBias;

			io[i].twoWayInteraction = curProperty.twoWayInteraction;

			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_SOLVE_CONSTRAINT_EX, (u32)(u64)commonBuff, 0);
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}

		mTask->deallocate(io);

		DEALLOCATE(jointSolverInfoRig);
		DEALLOCATE(jointGroupsRig);
		DEALLOCATE(jointSolverInfoPcl);
		DEALLOCATE(jointGroupsPcl);
		DEALLOCATE(contactSolverInfoRig);
		DEALLOCATE(contactGroupsRig);
		DEALLOCATE(contactSolverInfoPcl);
		DEALLOCATE(contactGroupsPcl);
		DEALLOCATE(jointSortsRig);
		DEALLOCATE(jointSortsPcl);
	}

	// Post response
	{
		IOParamPostResponsePcl *io = (IOParamPostResponsePcl*)mTask->allocate(sizeof(IOParamPostResponsePcl)*maxTasks);

		s32 numBatch = (curProperty.numParticles + maxTasks - 1)/maxTasks;
		s32 restStates = curProperty.numParticles;
		s32 startStates = 0;

		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;
			io[i].statesAddr = (u32)(u64)(pclStates + startStates);
			io[i].numStates = curBatch;
			io[i].maxLinearVelocity = curProperty.maxLinearVelocity;

			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_POSTRESPONSE, 0, 0);
			restStates -= curBatch;
			startStates += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}

		mTask->deallocate(io);
	}

	DEALLOCATE(commonBuff);

	//PRINTF("OUTPUT -----------------------------\n");
	//printRigs(states,numInstances);
}

void Particles::integrateSPU(f32 timeStep)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(pclBroadphase_task);

	IOParamIntegratePcl *io = (IOParamIntegratePcl*)mTask->allocate(sizeof(IOParamIntegratePcl)*maxTasks);

	s32 numBatch = (curProperty.numParticles + maxTasks - 1)/maxTasks;
	s32 restStates = curProperty.numParticles;
	s32 startStates = 0;

	for(s32 i=0;i < maxTasks;i++) {
		s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;

		io[i].statesAddr = (u32)(u64)(pclStates + startStates);
		io[i].numStates = curBatch;

		io[i].timeStep = timeStep;
		io[i].linearDamping = curProperty.linearDamping;
		io[i].extraForce = curProperty.extraForce;
		io[i].gravity = curProperty.gravity;

		mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_INTEGRATE, 0, 0);
		restStates -= curBatch;
		startStates += curBatch;
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];

		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	}

	mTask->deallocate(io);
}

void Particles::buildMeshSPU()
{
	if(numVertices == 0) return;

	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(pclMesh_task);

	// Gather
	for(u32 i=0;i < numVertices;i++)
		pclVtx[i] = pclStates[i].fX;

	Vector3 *tmpNormals[TM_MAX_TASKS];
	for(s32 i=0;i < maxTasks;i++)
		tmpNormals[i] = (Vector3*)ALLOCATE(128, sizeof(Vector3)*numVertices);

	// Cross
	{
		IOParamCrossPcl *io = (IOParamCrossPcl*)mTask->allocate(sizeof(IOParamCrossPcl)*maxTasks);

		s32 idxStep = ((((numIndices + 2)/3) + maxTasks -1)/maxTasks)*3;
		s32 idxRest = numIndices;
		s32 idxStart = 0;

		for(s32 i=0;i < maxTasks;i++) {
			s32 curIdxNum = (idxRest - idxStep ) > 0 ? idxStep : idxRest;

			// SPU
			u32 idxStartAlign  = idxStart&(128/sizeof(u16) - 1);

			io[i].indicesAddr   = (u32)(u64)(((u16*)pclIdx) + idxStart - idxStartAlign);
			io[i].indicesAlign  = idxStartAlign;
			io[i].numIndices    = curIdxNum;
			io[i].numVertices   = numVertices;
			io[i].verticesAddr  = (u32)(u64)pclVtx;
			io[i].normalsAddr   = (u32)(u64)tmpNormals[i];

			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_CROSS, 0, 0);

			idxRest -= curIdxNum;
			idxStart += curIdxNum;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}

		mTask->deallocate(io);
	}

	// Sum & Normalize
	{
		IOParamNormalizePcl *io = (IOParamNormalizePcl*)mTask->allocate(sizeof(IOParamNormalizePcl)*maxTasks);

		s32 numBatch = (numVertices + maxTasks - 1)/maxTasks;
		s32 restNormals = numVertices;
		s32 startNormals = 0;

		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restNormals - numBatch ) > 0 ? numBatch : restNormals;

			io[i].outNormalsAddr   = (u32)(u64)pclNml;
			io[i].startNormals     = startNormals;
			io[i].numNormals       = curBatch;
			io[i].numInNormalBuffers = maxTasks;
			for(s32 t=0;t < maxTasks;t++)
				io[i].inNormalBuffers[t] = (u32)(u64)tmpNormals[t];


			mTask->startTask(i, (u32)((u64)&io[i]), PARTICLE_NORMALIZE, 0, 0);
			restNormals -= curBatch;
			startNormals += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];

			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}

		for(s32 i=maxTasks - 1;i >= 0;i--)
			DEALLOCATE(tmpNormals[i]);

		mTask->deallocate(io);
	}
}
