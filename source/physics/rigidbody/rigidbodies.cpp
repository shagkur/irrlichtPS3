/*
 * rigidbodies.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: mike
 */

#include "rigidbody/rigidbodies.h"
#include "rigidbody/common/vec_utils.h"

#include "rbBroadphase_task.h"
#include "rbCollision_task.h"
#include "rbSolver_task.h"

#define ALLOCATE(align, size) 	mPool->allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16);
#define DEALLOCATE(ptr) 		if(ptr) {mPool->deallocate((void*)(u64)ptr); ptr = 0;}

RigidBodies::RigidBodies(MARSTaskManager *task, s32 taskId, HeapManager *pool)
{
	mTask = task;
	mTaskId = taskId;

	mPool = pool;

	forces = NULL;
	statesBuffer[0] = NULL;
	statesBuffer[1] = NULL;
	prevStates = NULL;
	bodies = NULL;
	collObjs = NULL;
	prims = NULL;
	joints = NULL;
	springs = NULL;
	contactPairs = NULL;
	sortedContactPairs = NULL;
	nonContactPair = NULL;
	bodiesPool = NULL;
	collsPool = NULL;
	primsPool = NULL;
	statesPool = NULL;
	jointsPool = NULL;
	springsPool = NULL;
	userData = NULL;
}

void RigidBodies::allocateBuffers()
{
	s32 maxPair = (worldProperty.maxInstances*(worldProperty.maxInstances - 1)/2);
	sizeContactTable = ALIGN128((maxPair + 31)/32,sizeof(u32));

	forces = (Forces*)ALLOCATE(128, sizeof(Forces)*worldProperty.maxInstances);
	statesBuffer[0] = (TrbState*)ALLOCATE(128, sizeof(TrbState)*worldProperty.maxInstances);
	statesBuffer[1] = (TrbState*)ALLOCATE(128, sizeof(TrbState)*worldProperty.maxInstances);
	prevStates = (TrbState*)ALLOCATE(128, sizeof(TrbState)*worldProperty.maxInstances);
	bodies = (TrbDynBody*)ALLOCATE(128, sizeof(TrbDynBody)*worldProperty.maxDynBodies);
	collObjs = (CollObject*)ALLOCATE(128, sizeof(CollObject)*worldProperty.maxDynBodies);
	prims = (CollPrim*)ALLOCATE(128, sizeof(CollPrim)*worldProperty.maxPrimitives);
	joints = (Joint*)ALLOCATE(128, sizeof(Joint)*worldProperty.maxJoints);
	springs = (Spring*)ALLOCATE(128, sizeof(Spring)*worldProperty.maxSprings);
	contactPairs = (ContactPair*)ALLOCATE(128, sizeof(ContactPair)*worldProperty.maxContactPairs);
	sortedContactPairs = (SortData*)ALLOCATE(128, sizeof(SortData)*worldProperty.maxContactPairs);

	nonContactPair = (u32*)ALLOCATE(128, sizeof(u32)*sizeContactTable);
	userData = (u32*)ALLOCATE(128, sizeof(u32)*worldProperty.maxInstances);

	bodiesPool  = (u16*)ALLOCATE(16, sizeof(u16)*worldProperty.maxDynBodies);
	collsPool = (u16*)ALLOCATE(16, sizeof(u16)*worldProperty.maxDynBodies);
	primsPool = (u16*)ALLOCATE(16, sizeof(u16)*worldProperty.maxPrimitives);
	statesPool  = (u16*)ALLOCATE(16, sizeof(u16)*worldProperty.maxInstances);
	jointsPool  = (u16*)ALLOCATE(16, sizeof(u16)*worldProperty.maxJoints);
	springsPool = (u16*)ALLOCATE(16, sizeof(u16)*worldProperty.maxSprings);
}

void RigidBodies::deallocateBuffers()
{
	DEALLOCATE(springsPool);
	DEALLOCATE(jointsPool);
	DEALLOCATE(statesPool);
	DEALLOCATE(primsPool);
	DEALLOCATE(collsPool);
	DEALLOCATE(bodiesPool);
	DEALLOCATE(userData);
	DEALLOCATE(nonContactPair);
	DEALLOCATE(sortedContactPairs);
	DEALLOCATE(contactPairs);
	DEALLOCATE(springs);
	DEALLOCATE(joints);
	DEALLOCATE(prims);
	DEALLOCATE(collObjs);
	DEALLOCATE(bodies);
	DEALLOCATE(prevStates);
	DEALLOCATE(statesBuffer[1]);
	DEALLOCATE(statesBuffer[0]);
	DEALLOCATE(forces);
}

void RigidBodies::setupWorldSize()
{
	worldVolume.setWorldSize(worldProperty.worldCenter, worldProperty.worldExtent);
}

void RigidBodies::getWorldSize(Vector3& center, Vector3& extent)
{
	center = worldProperty.worldCenter;
	extent = worldProperty.worldExtent;
}

void RigidBodies::setWorldSize(const Vector3& center, const Vector3& extent)
{
	worldProperty.worldCenter = center;
	worldProperty.worldExtent = extent;
	setupWorldSize();
}

s32	RigidBodies::createCollPrim()
{
	u32 newIdx;

	if(numPrimsPool > 0)
		newIdx = primsPool[--numPrimsPool];
	else if(numPrims < worldProperty.maxPrimitives)
		newIdx = numPrims++;
	else
		return -1;

	prims[newIdx].reset();

	return (s32)newIdx;
}

bool RigidBodies::deleteCollPrim(s32 primId)
{
	u32 delIdx = primId;

	if(delIdx >= worldProperty.maxPrimitives)
		return false;

	for(u32 i=0;i < numPrimsPool;i++) {
		if(primsPool[i] == delIdx)
			return false;
	}

	primsPool[numPrimsPool++] = delIdx;

	return true;
}

CollObject*	RigidBodies::createCollObject()
{
	u32 newIdx;

	if(numCollsPool > 0)
		newIdx = collsPool[--numCollsPool];
	else if(numCollObjs < worldProperty.maxDynBodies)
		newIdx = numCollObjs++;
	else
		return NULL;

	collObjs[newIdx].clear();
	collObjs[newIdx].mPrimBase = prims;
	collObjs[newIdx].mMaxPrims = 1;
	return &collObjs[newIdx];
}

CollObject*	RigidBodies::createCollObject(s32 numPrims)
{
	if(numPrims < 2) return createCollObject();

	u32 newIdx;

	if(numCollsPool > 0)
		newIdx = collsPool[--numCollsPool];
	else if(numCollObjs < worldProperty.maxDynBodies)
		newIdx = numCollObjs++;
	else
		return NULL;

	collObjs[newIdx].clear();
	collObjs[newIdx].mPrimBase = prims;
	collObjs[newIdx].mMaxPrims = numPrims;
	for(s32 i=0;i < numPrims - 1;i++) {
		s32 newPrimId = createCollPrim();
		ASSERT(newPrimId >= 0);
		collObjs[newIdx].mPrimIds[i] = (u16)newPrimId;
	}
	return &collObjs[newIdx];
}

bool RigidBodies::deleteCollObject(CollObject *coll)
{
	u32 delIdx = coll - collObjs;

	if(delIdx >= worldProperty.maxDynBodies)
		return false;

	for(u16 i=0;i < numCollsPool;i++) {
		if(collsPool[i] == delIdx)
			return false;
	}

	ASSERT(coll->mPrimBase == prims);

	for(s32 i=0;i < collObjs[delIdx].mNumPrims - 1;i++)
		deleteCollPrim(collObjs[delIdx].mPrimIds[i]);

	collsPool[numCollsPool++] = delIdx;

	return true;
}

TrbDynBody* RigidBodies::createRigidBody(const RigidBodyProperty& param)
{
	if(!param.collObject) return NULL;

	u32 newIdx;
	if(numBodiesPool > 0)
		newIdx = bodiesPool[--numBodiesPool];
	else if(numBodies < worldProperty.maxDynBodies)
		newIdx = numBodies++;
	else
		return NULL;

	bodies[newIdx].setMass(param.mass);
	bodies[newIdx].setFriction(param.friction);
	bodies[newIdx].setElasticity(param.restitution);
	bodies[newIdx].setBodyInertia(param.inertia);
	bodies[newIdx].setCollObject(param.collObject);

	return &bodies[newIdx];
}

bool RigidBodies::deleteRigidBody(TrbDynBody *rigidbody)
{
	u32 delIdx = rigidbody - bodies;

	if(delIdx >= worldProperty.maxDynBodies)
		return false;

	for(u16 i=0;i < numBodiesPool;i++) {
		if(bodiesPool[i] == delIdx)
			return false;
	}

	bodiesPool[numBodiesPool++] = delIdx;

	return true;
}

s32 RigidBodies::createInstance(const InstanceProperty& param)
{
	if(!param.rigidBody) return -1;

	u32 newIdx;
	if(numStatesPool > 0)
		newIdx = statesPool[--numStatesPool];
	else if(numInstances < worldProperty.maxInstances)
		newIdx = numInstances++;
	else
		return -1;

	if(param.rigidBody->getMass() <= 0.0f && param.moveType != MoveTypeKeyframe)
		statesBuffer[readBuffer][newIdx].setMoveType(MoveTypeFixed);
	else
		statesBuffer[readBuffer][newIdx].setMoveType(param.moveType);

	if(param.sleeping == 1) statesBuffer[readBuffer][newIdx].sleep();

	statesBuffer[readBuffer][newIdx].setContactFilterSelf(param.contactFilterSelf);
	statesBuffer[readBuffer][newIdx].setContactFilterTarget(param.contactFilterTarget);
	statesBuffer[readBuffer][newIdx].linearDamping = param.linearDamping;
	statesBuffer[readBuffer][newIdx].angularDamping = param.angularDamping;
	statesBuffer[readBuffer][newIdx].setPosition(param.position);
	statesBuffer[readBuffer][newIdx].setOrientation(param.orientation);
	statesBuffer[readBuffer][newIdx].setLinearVelocity(param.velocity);
	statesBuffer[readBuffer][newIdx].setAngularVelocity(param.angularVelocity);
	statesBuffer[readBuffer][newIdx].trbBodyIdx = ((u32)(u64)param.rigidBody - (u32)(u64)bodies)/sizeof(TrbDynBody);
	statesBuffer[readBuffer][newIdx].deleted = 0;
	statesBuffer[readBuffer][newIdx].useSleep = 1;
	statesBuffer[readBuffer][newIdx].useCcd = 0;
	statesBuffer[readBuffer][newIdx].useContactCallback = 0;
	statesBuffer[readBuffer][newIdx].useSleepCallback = 0;
	statesBuffer[readBuffer][newIdx].setDeltaLinearVelocity(Vector3(0.0f));
	statesBuffer[readBuffer][newIdx].setDeltaAngularVelocity(Vector3(0.0f));

	return (s32)newIdx;
}

bool RigidBodies::deleteInstance(s32 stateIndex)
{
	if(stateIndex < 0 || stateIndex >= (s32)worldProperty.maxInstances)
		return false;

	for(u16 i=0;i < numStatesPool;i++) {
		if(statesPool[i] == stateIndex)
			return false;
	}

	statesPool[numStatesPool++] = stateIndex;

	TrbState *delState = getState(stateIndex);
	delState->setMoveType(MoveTypeFixed);
	delState->deleted = 1;
	Vector3 half(delState->half[0], delState->half[1], delState->half[2]);
	setPosition(stateIndex, worldVolume.origin + worldVolume.extent + half);
	return true;
}

s32 RigidBodies::cloneInstance(u32 instance)
{
	if(instance >= numInstances) return -1;

	u32 newIdx;
	if(numStatesPool > 0)
		newIdx = statesPool[--numStatesPool];
	else if(numInstances < worldProperty.maxInstances)
		newIdx = numInstances++;
	else
		return -1;

	statesBuffer[readBuffer][newIdx] = statesBuffer[readBuffer][instance];
	statesBuffer[readBuffer][newIdx].trbBodyIdx = statesBuffer[readBuffer][instance].trbBodyIdx;

	return (s32)newIdx;
}

s32 RigidBodies::createSpring(f32 length, f32 ks, f32 kd, u16 stateIndexA, u16 stateIndexB, const Vector3& worldAnchor)
{
	Matrix3 rotA = transpose(Matrix3(statesBuffer[readBuffer][stateIndexA].getOrientation()));
	Matrix3 rotB = transpose(Matrix3(statesBuffer[readBuffer][stateIndexB].getOrientation()));

	return createSpring(length, ks, kd, stateIndexA, stateIndexB, rotA*(worldAnchor - statesBuffer[readBuffer][stateIndexA].getPosition()), rotB*(worldAnchor - statesBuffer[readBuffer][stateIndexB].getPosition()));
}

s32 RigidBodies::createSpring(f32 length, f32 ks, f32 kd, u16 stateIndexA, u16 stateIndexB, const Vector3& localAnchorA, const Vector3& localAnchorB)
{
	u32 newIdx;
	if(numSpringsPool > 0)
		newIdx = springsPool[--numSpringsPool];
	else if(numSprings < worldProperty.maxSprings)
		newIdx = numSprings++;
	else
		return -1;

	springs[newIdx].length = length;
	springs[newIdx].ks = ks;
	springs[newIdx].kd = kd;
	springs[newIdx].active = true;
	springs[newIdx].stateIndexA = stateIndexA;
	springs[newIdx].stateIndexB = stateIndexB;
	springs[newIdx].anchorA = localAnchorA;
	springs[newIdx].anchorB = localAnchorB;

	return (s32)newIdx;
}

bool RigidBodies::deleteSpring(s32 springIndex)
{
	if(springIndex < 0 || springIndex >= (s32)worldProperty.maxSprings)
		return false;

	for(u16 i=0;i < numSpringsPool;i++) {
		if(springsPool[i] == springIndex)
			return false;
	}

	springsPool[numSpringsPool++] = springIndex;
	springs[springIndex].active = false;

	return true;
}

s32 RigidBodies::createJoint()
{
	u32 newIdx;
	if(numJointsPool > 0)
		newIdx = jointsPool[--numJointsPool];
	else if(numJoints < worldProperty.maxJoints)
		newIdx = numJoints++;
	else
		return -1;

	return (s32)newIdx;
}

s32 RigidBodies::createJoint(const JointProperty& jointParam)
{
	ASSERT(jointParam.jointType < JointTypeCount);

	s32 jointIdx = createJoint();
	ASSERT(jointIdx >= 0);

	Joint& newJoint = joints[jointIdx];
	newJoint.reset();

	newJoint.jointType = jointParam.jointType;
	newJoint.linearDamping = jointParam.linearDamping;
	newJoint.angularDamping = jointParam.angularDamping;
	newJoint.maxLinearImpulse = jointParam.maxLinearImpulse;
	newJoint.maxAngularImpulse = jointParam.maxAngularImpulse;
	newJoint.linearImpulseWeight = jointParam.linearImpulseWeight;
	newJoint.angularImpulseWeight = jointParam.angularImpulseWeight;
	newJoint.linearBias = jointParam.linearBias;
	newJoint.angularBias = jointParam.angularBias;
	newJoint.targetFrame = Matrix3::identity();
	newJoint.stateIndexA = jointParam.parentBody;
	newJoint.stateIndexB = jointParam.childBody;
	if(jointParam.breakableLimit > 0.0f) {
		newJoint.breakableLimit = jointParam.breakableLimit;
		newJoint.enableBreakable();
	}

	for(s32 i=0;i < 6;i++) {
		if(jointParam.warmStarting[i])
			newJoint.enableWarmStarting(i);
		else
			newJoint.disableWarmStarting(i);
	}

	switch(jointParam.jointType) {
		case JointTypeBall:
			newJoint.setLock(0);
			newJoint.setLock(1);
			newJoint.setLock(2);
			newJoint.setFree(3);
			newJoint.setFree(4);
			newJoint.setFree(5);
			break;

		case JointTypeChain:
			newJoint.setLock(0);
			newJoint.setLock(1);
			newJoint.setLock(2);
			newJoint.setLimit(3);
			newJoint.setLimit(4);
			newJoint.setFree(5);
			if(jointParam.lowerLimit1 < jointParam.upperLimit1) {
				newJoint.lowerLimit[3] = jointParam.lowerLimit1;
				newJoint.upperLimit[3] = jointParam.upperLimit1;
			} else {
				newJoint.lowerLimit[3] = -0.26f;
				newJoint.upperLimit[3] =  0.26f;
			}
			if(jointParam.lowerLimit2 < jointParam.upperLimit2) {
				newJoint.lowerLimit[4] = jointParam.lowerLimit2;
				newJoint.upperLimit[4] = jointParam.upperLimit2;
			} else {
				newJoint.lowerLimit[4] =  0.0f;
				newJoint.upperLimit[4] =  0.7f;
			}
			break;

		case JointTypeSlider:
			newJoint.setLock(1);
			newJoint.setLock(2);
			newJoint.setLock(3);
			newJoint.setLock(4);
			newJoint.setLock(5);
			if(jointParam.lowerLimit1 < jointParam.upperLimit1) {
				newJoint.setLimit(0);
				newJoint.lowerLimit[0] = jointParam.lowerLimit1;
				newJoint.upperLimit[0] = jointParam.upperLimit1;
			} else
				newJoint.setFree(0);
			break;

		case JointTypeHinge:
			newJoint.setLock(0);
			newJoint.setLock(1);
			newJoint.setLock(2);
			newJoint.setLock(4);
			newJoint.setLock(5);
			if(jointParam.lowerLimit1 < jointParam.upperLimit1) {
				newJoint.setLimit(3);
				newJoint.lowerLimit[3] = jointParam.lowerLimit1;
				newJoint.upperLimit[3] = jointParam.upperLimit1;
			} else
				newJoint.setFree(3);
			break;

		case JointTypeFix:
		case JointTypeAnimation:
			newJoint.setLock(0);
			newJoint.setLock(1);
			newJoint.setLock(2);
			newJoint.setLock(3);
			newJoint.setLock(4);
			newJoint.setLock(5);
			break;

		case JointTypeUniversal:
			newJoint.setLock(0);
			newJoint.setLock(1);
			newJoint.setLock(2);
			newJoint.setLock(3);
			newJoint.setLimit(4);
			newJoint.setLimit(5);
			if(jointParam.lowerLimit1 < jointParam.upperLimit1) {
				newJoint.lowerLimit[4] = jointParam.lowerLimit1;
				newJoint.upperLimit[4] = jointParam.upperLimit1;
			} else {
				newJoint.lowerLimit[4] = -0.5f;
				newJoint.upperLimit[4] =  0.5f;
			}
			if(jointParam.lowerLimit2 < jointParam.upperLimit2) {
				newJoint.lowerLimit[5] = jointParam.lowerLimit2;
				newJoint.upperLimit[5] = jointParam.upperLimit2;
			} else {
				newJoint.lowerLimit[5] =  -0.7f;
				newJoint.upperLimit[5] =   0.7f;
			}
			break;

		case JointTypeDistance:
			newJoint.lowerLimit[0] = jointParam.distance;
			newJoint.upperLimit[0] = jointParam.distance;
			newJoint.setLimit(0);
			newJoint.setLock(1);
			newJoint.setLock(2);
			newJoint.setFree(3);
			newJoint.setFree(4);
			newJoint.setFree(5);
			break;
	}

	if(jointParam.parentBody < numInstances && jointParam.childBody < numInstances &&
	   jointParam.parentBody != jointParam.childBody)
	{
		TrbState& stateA = statesBuffer[readBuffer][jointParam.parentBody];
		TrbState& stateB = statesBuffer[readBuffer][jointParam.childBody];

		Matrix3 rotA = transpose(Matrix3(stateA.getOrientation()));
		Matrix3 rotB = transpose(Matrix3(stateB.getOrientation()));

		Vector3 axisInA = rotA*normalize(jointParam.axis);
		Vector3 axisInB = rotB*normalize(jointParam.axis);

		newJoint.anchorA = rotA*(jointParam.anchor - stateA.getPosition());
		newJoint.anchorB = rotB*(jointParam.anchor - stateB.getPosition());

		Vector3 axis1, axis2;

		getPlaneSpace(axisInA, axis1, axis2 );
		newJoint.frameA = Matrix3(axisInA, axis1, axis2);
		newJoint.frameB = rotB*Matrix3(stateA.getOrientation())*newJoint.frameA;
	} else {
		newJoint.anchorA = Vector3(1.0f, 0.0f, 0.0f);
		newJoint.anchorB = Vector3(1.0f, 0.0f, 0.0f);
		newJoint.frameA = Matrix3::identity();
		newJoint.frameB = Matrix3::identity();
		newJoint.disableActive();
	}

	return (s32)jointIdx;
}

bool RigidBodies::deleteJoint(s32 jointIndex)
{
	if(jointIndex < 0 || jointIndex >= (s32)worldProperty.maxJoints)
		return false;

	for(u16 i=0;i < numJointsPool;i++) {
		if(jointsPool[i] == jointIndex)
			return false;
	}

	jointsPool[numJointsPool++] = jointIndex;
	joints[jointIndex].disableActive();

	return true;
}

void RigidBodies::appendNonContactPair(u16 stateIndexA, u16 stateIndexB)
{
	ASSERT(stateIndexA != stateIndexB);
	ASSERT(stateIndexA < worldProperty.maxInstances);
	ASSERT(stateIndexB < worldProperty.maxInstances);

	u32 minIdx = stateIndexA > stateIndexB ? stateIndexB : stateIndexA;
	u32 maxIdx = stateIndexA > stateIndexB ? stateIndexA : stateIndexB;

	u32 idx = maxIdx*(maxIdx - 1)/2 + minIdx;
	ASSERT((idx>>5) < sizeContactTable);

	nonContactPair[idx>>5] |= 1L<<(idx&31);
}

void RigidBodies::removeNonContactPair(u16 stateIndexA, u16 stateIndexB)
{
	ASSERT(stateIndexA != stateIndexB);
	ASSERT(stateIndexA < worldProperty.maxInstances);
	ASSERT(stateIndexB < worldProperty.maxInstances);

	u32 minIdx = stateIndexA > stateIndexB ? stateIndexB : stateIndexA;
	u32 maxIdx = stateIndexA > stateIndexB ? stateIndexA : stateIndexB;

	u32 idx = maxIdx*(maxIdx - 1)/2 + minIdx;
	ASSERT((idx>>5) < sizeContactTable);

	nonContactPair[idx>>5] = nonContactPair[idx>>5]&~(1L<<(idx&31));
}

bool RigidBodies::checkNonContactPair(u16 stateIndexA, u16 stateIndexB)
{
	return isCollidablePair(stateIndexA, stateIndexB);
}

void RigidBodies::setSleepEnable(bool b)
{
	worldProperty.sleepEnable = b;
	for(u32 i=0;i < numInstances;i++) {
		TrbState& state = statesBuffer[readBuffer][i];
		if(state.getMoveTypeBits()&MOVE_TYPE_DYNAMIC && state.isAsleep())
			state.wakeup();
	}
}


void RigidBodies::throwContactCallback()
{
	// Throw contact callback
	if(worldProperty.contactCallback) {
		for(u32 i=0;i < numContactPairs;i++) {
			if(getCallbackFlag(sortedContactPairs[i])) {
				ContactPair& curPair = contactPairs[getPair(sortedContactPairs[i])];
				worldProperty.contactCallback->onContact(curPair);
			}
		}
	}
}

void RigidBodies::throwSleepCallback()
{
	// Throw sleep callback
	if(worldProperty.sleepCallback) {
		for(u32 i=0;i < numInstances;i++) {
			if(states[i].getMoveTypeBits()&MOVE_TYPE_DYNAMIC && states[i].getUseSleepCallback()) {
				if(prevStates[i].isAwake() && states[i].isAsleep())
					worldProperty.sleepCallback->onSleep(i);
				else if(prevStates[i].isAsleep() && states[i].isAwake())
					worldProperty.sleepCallback->onActive(i);
			}
		}
	}
}

void RigidBodies::updateFacetLocal(ContactPoint& cp, u32 p, LargeTriMesh *curLargeMesh, LargeTriMesh *preLargeMesh, f32 timeStep)
{
	SubData& subData = cp.subData;
	TriMesh& islandCur = curLargeMesh->islands[subData.getIslandIndex()];
	TriMesh& islandPre = preLargeMesh->islands[subData.getIslandIndex()];
	MeshFacet& facet = islandCur.facets[subData.getFacetIndex()];

	f32 fs = subData.getFacetLocalS();
	f32 ft = subData.getFacetLocalT();

	Vector3 pntsCur[3] =
	{
		islandCur.verts[facet.vertIndices[0]],
		islandCur.verts[facet.vertIndices[1]],
		islandCur.verts[facet.vertIndices[2]]
	};

	Vector3 pntsPre[3] =
	{
		islandPre.verts[facet.vertIndices[0]],
		islandPre.verts[facet.vertIndices[1]],
		islandPre.verts[facet.vertIndices[2]]
	};

	Vector3 cpCur = pntsCur[0] + fs*(pntsCur[1] - pntsCur[0]) + ft*(pntsCur[2] - pntsCur[0]);
	Vector3 cpPre = pntsPre[0] + fs*(pntsPre[1] - pntsPre[0]) + ft*(pntsPre[2] - pntsPre[0]);

	if(p == 0) {
		cp.setLocalVelocityA((cpCur - cpPre)/timeStep);
		cp.setLocalPointA(cpCur);
	} else {
		cp.setLocalVelocityB((cpCur - cpPre)/timeStep);
		cp.setLocalPointB(cpCur);
	}
}

void RigidBodies::updateFacetLocal(Joint& joint, u32 p, LargeTriMesh *curLargeMesh, LargeTriMesh *preLargeMesh, f32 timeStep)
{
	SubData& subData = joint.subData;
	TriMesh& islandCur = curLargeMesh->islands[subData.getIslandIndex()];
	TriMesh& islandPre = preLargeMesh->islands[subData.getIslandIndex()];
	MeshFacet& facet = islandCur.facets[subData.getFacetIndex()];

	f32 fs = subData.getFacetLocalS();
	f32 ft = subData.getFacetLocalT();

	Vector3 pntsCur[3] =
	{
		islandCur.verts[facet.vertIndices[0]],
		islandCur.verts[facet.vertIndices[1]],
		islandCur.verts[facet.vertIndices[2]]
	};

	Vector3 pntsPre[3] =
	{
		islandPre.verts[facet.vertIndices[0]],
		islandPre.verts[facet.vertIndices[1]],
		islandPre.verts[facet.vertIndices[2]]
	};

	Vector3 cpCur = pntsCur[0] + fs*(pntsCur[1] - pntsCur[0]) + ft*(pntsCur[2] - pntsCur[0]);
	Vector3 cpPre = pntsPre[0] + fs*(pntsPre[1] - pntsPre[0]) + ft*(pntsPre[2] - pntsPre[0]);

	if(p == 0) {
		joint.localVelocityA = (cpCur - cpPre)/timeStep;
		joint.anchorA = cpCur;
	} else {
		joint.localVelocityB = (cpCur - cpPre)/timeStep;
		joint.anchorB = cpCur;
	}
}

void RigidBodies::updateContactPoints(f32 timeStep)
{
	for(u32 i=0;i < numContactPairs;i++) {
		ContactPair& contact = contactPairs[getPair(sortedContactPairs[i])];
		CollObject *collA = getTrbDynBody(contact.stateIndex[0])->getCollObject();
		CollObject *collB = getTrbDynBody(contact.stateIndex[1])->getCollObject();

		ASSERT(collA->getNumPrims() > 0);
		ASSERT(collB->getNumPrims() > 0);

		if(collA->getDefPrim().getType() == LARGEMESH) {
			for(u32 c=0;c < contact.numContacts;c++) {
				ContactPoint& cp = contact.contactPoints[c];
				CollPrim& prim = collA->getDefPrim();
				if(prim.getPreLargeMesh() == 0) continue;
				LargeTriMesh *curLargeMesh = prim.getLargeMesh();
				LargeTriMesh *preLargeMesh = prim.getPreLargeMesh();
				updateFacetLocal(cp, 0, curLargeMesh, preLargeMesh, timeStep);
			}
		}
		else if(collB->getDefPrim().getType() == LARGEMESH) {
			for(u32 c=0;c < contact.numContacts;c++) {
				ContactPoint& cp = contact.contactPoints[c];
				CollPrim& prim = collB->getDefPrim();
				if(prim.getPreLargeMesh() == 0) continue;
				LargeTriMesh *curLargeMesh = prim.getLargeMesh();
				LargeTriMesh *preLargeMesh = prim.getPreLargeMesh();
				updateFacetLocal(cp, 1, curLargeMesh, preLargeMesh, timeStep);
			}
		}
	}
}

void RigidBodies::updateJointPoints(f32 timeStep)
{
	for(u32 i=0;i < numJoints;i++) {
		Joint& joint = joints[i];
		CollObject *collA = getTrbDynBody(joint.stateIndexA)->getCollObject();
		CollObject *collB = getTrbDynBody(joint.stateIndexB)->getCollObject();

		ASSERT(collA->getNumPrims() > 0);
		ASSERT(collB->getNumPrims() > 0);

		if(collA->getDefPrim().getType() == LARGEMESH) {
			CollPrim& prim = collA->getDefPrim();
			if(prim.getPreLargeMesh() == 0) continue;
			LargeTriMesh *curLargeMesh = prim.getLargeMesh();
			LargeTriMesh *preLargeMesh = prim.getPreLargeMesh();
			updateFacetLocal(joint, 0, curLargeMesh, preLargeMesh, timeStep);
		}
		else if(collB->getDefPrim().getType() == LARGEMESH) {
			CollPrim& prim = collB->getDefPrim();
			if(prim.getPreLargeMesh() == 0) continue;
			LargeTriMesh *curLargeMesh = prim.getLargeMesh();
			LargeTriMesh *preLargeMesh = prim.getPreLargeMesh();
			updateFacetLocal(joint, 1, curLargeMesh, preLargeMesh, timeStep);
		}
	}
}

void RigidBodies::reset()
{
	deallocateBuffers();
	allocateBuffers();

	clearNonContactPair();

	memset(userData, 0, sizeof(u32)*worldProperty.maxInstances);

	numInstances = 0;
	numBodies = 0;
	numCollObjs = 0;
	numPrims = 0;
	numJoints = 0;
	numSprings = 0;
	writeBuffer = 0;
	readBuffer = 1;

	states = statesBuffer[writeBuffer];

	setupWorldSize();

	memset(sortedContactPairs, 0, sizeof(SortData)*worldProperty.maxContactPairs);
	for(u32 i=0;i < worldProperty.maxContactPairs;i++)
		setPair(sortedContactPairs[i], i);

	numContactPairs = 0;

	numBodiesPool = 0;
	numCollsPool = 0;
	numPrimsPool = 0;
	numStatesPool = 0;
	numJointsPool = 0;
	numSpringsPool = 0;

	worldSleepCount = 0;

	userAction = NULL;
}

void RigidBodies::setup()
{
	for(u32 i=0;i < numInstances;i++)
		setupRigidBody(i);
}

void RigidBodies::setupRigidBody(u16 stateIndex)
{
	CollObject *coll = bodies[statesBuffer[readBuffer][stateIndex].trbBodyIdx].getCollObject();
	forces[stateIndex].force = forces[stateIndex].torque = Vector3(0.0f);
	statesBuffer[readBuffer][stateIndex].setAuxils(coll->getCenter(), coll->getHalf());
	statesBuffer[writeBuffer][stateIndex].setAuxils(coll->getCenter(), coll->getHalf());

	statesBuffer[readBuffer][stateIndex].sleepCount = 0;

	prevStates[stateIndex] = statesBuffer[writeBuffer][stateIndex] = statesBuffer[readBuffer][stateIndex];
}

void RigidBodies::setupSimulate()
{
	applySpring();
}

void RigidBodies::finalizeSimulate()
{
	memset(forces, 0, sizeof(Forces)*numInstances);

	writeBuffer = 1 - writeBuffer;
	readBuffer = 1 - readBuffer;
}

void RigidBodies::applySpring()
{
	for(u32 i=0;i < numSprings;i++) {
		if(!springs[i].active) continue;

		u16 bodyA = springs[i].stateIndexA;
		u16 bodyB = springs[i].stateIndexB;

		Vector3 rA = rotate(statesBuffer[readBuffer][bodyA].getOrientation(), springs[i].anchorA);
		Vector3 rB = rotate(statesBuffer[readBuffer][bodyB].getOrientation(), springs[i].anchorB);

		Vector3 pA = statesBuffer[readBuffer][bodyA].getPosition() + rA;
		Vector3 pB = statesBuffer[readBuffer][bodyB].getPosition() + rB;

		Vector3 distance = pB - pA;

		f32 distanceSqr = lengthSqr(distance);
		if(distanceSqr < 1.0e-10f) continue;

		Vector3 velocityA = statesBuffer[readBuffer][bodyA].getLinearVelocity() + cross(statesBuffer[readBuffer][bodyA].getAngularVelocity(), rA);
		Vector3 velocityB = statesBuffer[readBuffer][bodyB].getLinearVelocity() + cross(statesBuffer[readBuffer][bodyB].getAngularVelocity(), rB);

		Vector3 relativeVelocity = velocityB - velocityA;

		f32 lenDiff = sqrtf(distanceSqr) - (springs[i].length);

		if(lenDiff < 1.0e-10f) continue;

		Vector3 springVector = normalize(distance);
		Vector3 springForce = -springVector*(springs[i].ks*lenDiff + springs[i].kd*dot(relativeVelocity, springVector));
		if(statesBuffer[readBuffer][bodyA].getMoveTypeBits()&MOVE_TYPE_DYNAMIC)
			applyForceByPosition(bodyA, -springForce, pA);

		if(statesBuffer[readBuffer][bodyB].getMoveTypeBits()&MOVE_TYPE_DYNAMIC)
			applyForceByPosition(bodyB, springForce, pB);
	}
}

void RigidBodies::spuSimulate(f32 timeStep, u32 flag)
{
	memcpy(statesBuffer[writeBuffer], statesBuffer[readBuffer], sizeof(TrbState)*numInstances);
	states = statesBuffer[writeBuffer];

	f32 subTimeStep = timeStep/(f32)worldProperty.subStepCount;
	for(u32 substep=0;substep < worldProperty.subStepCount;substep++) {
		broadPhaseSPU(subTimeStep);

		detectCollisionsSPU(subTimeStep);

		if(numContactPairs > 0) {
			u32 numRemovedContactPairs = refreshContactPairsSPU();
			numContactPairs -= numRemovedContactPairs;
		}

		if(worldProperty.deformMeshEnable) {
			updateContactPoints(subTimeStep);
			updateJointPoints(subTimeStep);
		}

		solveConstraintsSPU(subTimeStep);

		if(userAction) (*userAction)();

		if(worldProperty.sleepEnable) sleepOrWakeupSPU();

		throwSleepCallback();

		integrateSPU(subTimeStep, ((s32)substep == worldProperty.subStepCount - 1));

		throwContactCallback();
	}
}

void RigidBodies::broadPhaseSPU(f32 timeStep)
{
	s32 chkAxis = 0;
	u32 numMovAabb = 0;
	u32 numFixAabb = 0;

	SortData *movAabbArray;
	SortData *fixAabbArray;
	movAabbArray = (SortData*)ALLOCATE(16, sizeof(SortData)*numInstances);
	fixAabbArray = (SortData*)ALLOCATE(16, sizeof(SortData)*numInstances);

	u32 maxNewPairs = worldProperty.maxContactPairs;
	u32 numNewPairs = 0;
	SortData *newPairs = (SortData*)ALLOCATE(128, sizeof(SortData)*maxNewPairs);

	assignSetSPU(timeStep, movAabbArray, numMovAabb, fixAabbArray, numFixAabb, chkAxis);
	detectSetSPU(newPairs, numNewPairs, maxNewPairs, movAabbArray, numMovAabb, fixAabbArray, numFixAabb, chkAxis);
	mergeSetSPU(newPairs, numNewPairs);

	DEALLOCATE(newPairs);
	DEALLOCATE(fixAabbArray);
	DEALLOCATE(movAabbArray);
}

void RigidBodies::assignStatesSPU(f32 timeStep, SortData *movAabbArray, u32& numMovAabb, SortData *fixAabbArray, u32 &numFixAabb, s32& chkAxis)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	s32 numBatch = (numInstances + maxTasks - 1)/maxTasks;
	s32 restStates = numInstances;
	s32 startState = 0;

	{
		IOParamAssignStates *io = (IOParamAssignStates*)mTask->allocate(sizeof(IOParamAssignStates)*maxTasks);
		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;
			io[i].statesAddr = (u32)(u64)states;
			io[i].numStates = numInstances;
			io[i].batchStartState = startState;
			io[i].numBatchStates = curBatch;
			mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_CALCVARIANCE, 0, 0);
			restStates -= curBatch;
			startState += curBatch;
		}

		Vector3 s(0.0f), s2(0.0f);
		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			s += io[taskId].s;
			s2 += io[taskId].s2;
		}

		Vector3 v = s2 - mulPerElem(s, s)/(f32)numInstances;
		if(v[1] > v[0]) chkAxis = 1;
		if(v[2] > v[chkAxis]) chkAxis = 2;

		for(s32 i=0;i < maxTasks;i++) {
			io[i].chkAxis = chkAxis;
			io[i].movAabbAddr = (u32)(u64)ALLOCATE(16, sizeof(SortData)*io[i].numBatchStates);
			io[i].fixAabbAddr = (u32)(u64)ALLOCATE(16, sizeof(SortData)*io[i].numBatchStates);
			io[i].numMovAabb = 0;
			io[i].numFixAabb = 0;
			io[i].worldVolumeAddr = (u32)((u64)&worldVolume);
			io[i].timeStep = timeStep;
			mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_ASSIGNSTATES, 0, 0);
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			memcpy(movAabbArray + numMovAabb, (void*)(u64)io[taskId].movAabbAddr, sizeof(SortData)*io[taskId].numMovAabb);
			memcpy(fixAabbArray + numFixAabb, (void*)(u64)io[taskId].fixAabbAddr, sizeof(SortData)*io[taskId].numFixAabb);
			numMovAabb += io[taskId].numMovAabb;
			numFixAabb += io[taskId].numFixAabb;
		}

		for(s32 i=maxTasks - 1;i >= 0;i--) {
			DEALLOCATE(io[i].fixAabbAddr);
			DEALLOCATE(io[i].movAabbAddr);
		}
		mTask->deallocate(io);
	}

	{
		IOParamSort *io = (IOParamSort*)mTask->allocate(sizeof(IOParamSort));
		SortData *tmpSorts = (SortData*)ALLOCATE(16, sizeof(SortData)*numMovAabb);
		for(s32 i=0;i < maxTasks;i++) {
			io->numSpu = maxTasks;
			io->buffAddr = (u32)(u64)tmpSorts;
			io->sortsAddr = (u32)(u64)movAabbArray;
			io->numSorts = numMovAabb;
			mTask->startTask(i, (u32)(u64)io, BROADPHASE_SORT, 0, 0);
		}
		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}
		DEALLOCATE(tmpSorts);
		mTask->deallocate(io);
	}

	{
		IOParamSort *io = (IOParamSort*)mTask->allocate(sizeof(IOParamSort));
		SortData *tmpSorts = (SortData*)ALLOCATE(16, sizeof(SortData)*numFixAabb);
		for(s32 i=0;i < maxTasks;i++) {
			io->numSpu = maxTasks;
			io->buffAddr = (u32)(u64)tmpSorts;
			io->sortsAddr = (u32)(u64)fixAabbArray;
			io->numSorts = numFixAabb;
			mTask->startTask(i, (u32)(u64)io, BROADPHASE_SORT, 0, 0);
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

void RigidBodies::detectPairsSPU(SortData *newPairs, u32& numNewPairs, u32 maxNewPairs, SortData *movAabbArray, u32 numMovAabb, SortData *fixAabbArray, u32 numFixAabb, s32 chkAxis)
{
	(void) maxNewPairs;
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	IOParamDetectPairs *io = (IOParamDetectPairs*)mTask->allocate(sizeof(IOParamDetectPairs)*maxTasks);

	u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
	u32 div = maxTasks*4;
	u32 numBatch = (numMovAabb + div - 1)/div;
	commonBuff[0] = 0;					// 0 : unlocked , 1 : locked
	commonBuff[1] = 0;					// start pos
	commonBuff[2] = MIN(numBatch, 128);	// batch num

	// Mov-Mov
	for(s32 i=0;i < maxTasks;i++) {
		io[i].chkAxis = chkAxis;
		io[i].nonContactPairAddr = (u32)(u64)nonContactPair;

		io[i].movAabbAddr = (u32)(u64)movAabbArray;
		io[i].numMovAabb = numMovAabb;

		io[i].tmpSortsAddr = (u32)(u64)ALLOCATE(128, sizeof(SortData)*worldProperty.maxContactPairs);
		io[i].numTmpSorts = 0;
		io[i].maxSorts = worldProperty.maxContactPairs;

		mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_DETECTPAIRS_MOV, (u32)(u64)commonBuff, 0);
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		ASSERT(numNewPairs + io[taskId].numTmpSorts <= worldProperty.maxContactPairs);
		memcpy(newPairs + numNewPairs, (void*)(u64)io[taskId].tmpSortsAddr, sizeof(SortData)*io[taskId].numTmpSorts);
		numNewPairs += io[taskId].numTmpSorts;
	}

	for(s32 i=maxTasks - 1;i >= 0;i--) {
		DEALLOCATE(io[i].tmpSortsAddr);
	}

	// Fix-Mov
	commonBuff[0] = 0;					// 0 : unlocked , 1 : locked
	commonBuff[1] = 0;					// start pos
	commonBuff[2] = MIN(numBatch, 64);	// batch num

	for(s32 i=0;i < maxTasks;i++) {
		io[i].movAabbAddr = (u32)(u64)movAabbArray;
		io[i].numMovAabb = numMovAabb;
		io[i].fixAabbAddr = (u32)(u64)fixAabbArray;
		io[i].numFixAabb = numFixAabb;

		io[i].tmpSortsAddr = (u32)(u64)ALLOCATE(128, sizeof(SortData)*worldProperty.maxContactPairs);
		io[i].numTmpSorts = 0;
		io[i].maxSorts = worldProperty.maxContactPairs;
		mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_DETECTPAIRS_FIX, (u32)(u64)commonBuff, 0);
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		ASSERT(numNewPairs + io[taskId].numTmpSorts <= worldProperty.maxContactPairs);
		memcpy(newPairs + numNewPairs, (void*)(u64)io[taskId].tmpSortsAddr, sizeof(SortData)*io[taskId].numTmpSorts);
		numNewPairs += io[taskId].numTmpSorts;
	}

	for(s32 i=maxTasks - 1;i >= 0;i--) {
		DEALLOCATE(io[i].tmpSortsAddr);
	}

	DEALLOCATE(commonBuff);

	mTask->deallocate(io);
}

void RigidBodies::mergePairsSPU(SortData *newPairs, u32& numNewPairs)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	SortData *tmpPairs = (SortData*)ALLOCATE(128, sizeof(SortData)*numNewPairs);
	memset(tmpPairs, 0, sizeof(SortData)*numNewPairs);

	{
		IOParamSort *io = (IOParamSort*)mTask->allocate(sizeof(IOParamSort));
		for(s32 i=0;i < maxTasks;i++) {
			io->numSpu = maxTasks;
			io->buffAddr = (u32)(u64)tmpPairs;
			io->sortsAddr = (u32)(u64)newPairs;
			io->numSorts = numNewPairs;
			mTask->startTask(i, (u32)(u64)io, BROADPHASE_SORT, 0, 1);
		}
		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}
		mTask->deallocate(io);
	}

	s32 removedPairs = 0;

	{
		IOParamMergePairs *io = (IOParamMergePairs*)mTask->allocate(sizeof(IOParamMergePairs)*maxTasks);
		s32 numBatch = (numNewPairs + maxTasks - 1)/maxTasks;
		s32 restSort = numNewPairs;
		s32 startSort = 0;
		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restSort - numBatch ) > 0 ? numBatch : restSort;
			io[i].newSortsAddr = (u32)(u64)(newPairs + startSort);
			io[i].numNewSorts = curBatch;
			io[i].oldSortsAddr = (u32)(u64)sortedContactPairs;
			io[i].numOldSorts = numContactPairs;
			io[i].statesAddr = (u32)(u64)states;
			io[i].numStates = numInstances;
			io[i].maxInstances = worldProperty.maxInstances;
			mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_MERGEPAIRS, 0, 0);

			restSort -= curBatch;
			startSort += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			removedPairs += io[taskId].numNewSorts;
		}
		removedPairs = numNewPairs - removedPairs;
		mTask->deallocate(io);
	}

	{
		IOParamSort *io = (IOParamSort*)mTask->allocate(sizeof(IOParamSort));
		for(s32 i=0;i < maxTasks;i++) {
			io->numSpu = maxTasks;
			io->buffAddr = (u32)(u64)tmpPairs;
			io->sortsAddr = (u32)(u64)newPairs;
			io->numSorts = numNewPairs;
			mTask->startTask(i, (u32)(u64)io, BROADPHASE_SORT, 0, 0);
		}
		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}
		mTask->deallocate(io);
	}

	numNewPairs -= removedPairs;

	DEALLOCATE(tmpPairs);
}

void RigidBodies::addNewPairsSPU(SortData *newPairs, u32 numNewPairs)
{
	ASSERT(numContactPairs + numNewPairs <= worldProperty.maxContactPairs);

	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	IOParamAddNewPairs *io = (IOParamAddNewPairs*)mTask->allocate(sizeof(IOParamAddNewPairs)*maxTasks);

	s32 numBatch = (numNewPairs + maxTasks - 1)/maxTasks;
	s32 restPairs = numNewPairs;
	s32 startPairs = 0;
	for(s32 i=0;i < maxTasks;i++) {
		s32 curBatch = (restPairs - numBatch ) > 0 ? numBatch : restPairs;

		io[i].startPair = startPairs;
		io[i].batchPair = curBatch;
		io[i].numPairs = numContactPairs;
		io[i].contactsAddr = (u32)(u64)contactPairs;
		io[i].pairsAddr = (u32)(u64)sortedContactPairs;
		io[i].newPairsAddr = (u32)(u64)newPairs;
		mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_ADDNEWPAIRS, 0, 0);

		restPairs -= curBatch;
		startPairs += curBatch;
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	}

	mTask->deallocate(io);

	numContactPairs += numNewPairs;
}

void RigidBodies::assignSetSPU(f32 timeStep, SortData *movAabbArray, u32& numMovAabb, SortData *fixAabbArray,u32& numFixAabb, s32& chkAxis)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	s32 numBatch = (numInstances + maxTasks - 1)/maxTasks;
	s32 restStates = numInstances;
	s32 startState = 0;

	{
		IOParamAssignStates *io = (IOParamAssignStates*)mTask->allocate(sizeof(IOParamAssignStates)*maxTasks);
		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;
			io[i].statesAddr = (u32)(u64)states;
			io[i].numStates = numInstances;
			io[i].batchStartState = startState;
			io[i].numBatchStates = curBatch;
			io[i].chkAxis = chkAxis;
			io[i].movAabbAddr = (u32)(u64)ALLOCATE(16, sizeof(SortData)*io[i].numBatchStates);
			io[i].fixAabbAddr = (u32)(u64)ALLOCATE(16, sizeof(SortData)*io[i].numBatchStates);
			io[i].numMovAabb = 0;
			io[i].numFixAabb = 0;
			io[i].worldVolumeAddr = (u32)((u64)&worldVolume);
			io[i].timeStep = timeStep;
			io[i].numSpu = maxTasks;
			mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_ASSIGN_SET, 0, 0);
			restStates -= curBatch;
			startState += curBatch;
		}
		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			memcpy(movAabbArray + numMovAabb, (void*)(u64)io[taskId].movAabbAddr, sizeof(SortData)*io[taskId].numMovAabb);
			memcpy(fixAabbArray + numFixAabb, (void*)(u64)io[taskId].fixAabbAddr, sizeof(SortData)*io[taskId].numFixAabb);
			numMovAabb += io[taskId].numMovAabb;
			numFixAabb += io[taskId].numFixAabb;
		}
		chkAxis = io[0].chkAxis;

		for(s32 i=maxTasks - 1;i >= 0;i--) {
			DEALLOCATE(io[i].fixAabbAddr);
			DEALLOCATE(io[i].movAabbAddr);
		}
		mTask->deallocate(io);
	}
}

void RigidBodies::detectSetSPU(SortData *newPairs, u32& numNewPairs, u32 maxNewPairs, SortData *movAabbArray, u32 numMovAabb, SortData *fixAabbArray, u32 numFixAabb, s32 chkAxis)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	{
		IOParamDetectSet *io = (IOParamDetectSet*)mTask->allocate(sizeof(IOParamDetectSet)*maxTasks);
		//allocate max size buffer
		u32 commonBufSize = sizeof(SortData)*numMovAabb;
		if(commonBufSize < sizeof(SortData)*numFixAabb) commonBufSize = sizeof(SortData)*numFixAabb;
		void *commonBuf = ALLOCATE(16, commonBufSize);
		SortData *tmpSorts = (SortData*)commonBuf;
		u32 *movCommonBuf = (u32*)ALLOCATE(128, sizeof(u32)*32);
		u32 *fixCommonBuf = (u32*)ALLOCATE(128, sizeof(u32)*32);
		u32 div = maxTasks*4;
		u32 numBatch = (numMovAabb + div - 1)/div;
		movCommonBuf[0] = 0;					// 0 : unlocked , 1 : locked
		movCommonBuf[1] = 0;					// start pos
		movCommonBuf[2] = MIN(numBatch, 128);	// batch num
		fixCommonBuf[0] = 0;					// 0 : unlocked , 1 : locked
		fixCommonBuf[1] = 0;					// start pos
		fixCommonBuf[2] = MIN(numBatch, 64);	// batch num
		for(s32 i=0;i < maxTasks;i++) {
			//Preparing movAabbSort
			io[i].movAabbSort.numSpu = maxTasks;
			io[i].movAabbSort.buffAddr = (u32)(u64)tmpSorts;
			io[i].movAabbSort.sortsAddr = (u32)(u64)movAabbArray;
			io[i].movAabbSort.numSorts = numMovAabb;
			//Preparing fixAabbSort
			io[i].fixAabbSort.numSpu = maxTasks;
			io[i].fixAabbSort.buffAddr = (u32)(u64)tmpSorts;
			io[i].fixAabbSort.sortsAddr = (u32)(u64)fixAabbArray;
			io[i].fixAabbSort.numSorts = numFixAabb;
			//detect collision mov
			io[i].movDetectPairs.chkAxis = chkAxis;
			io[i].movDetectPairs.nonContactPairAddr = (u32)(u64)nonContactPair;
			io[i].movDetectPairs.movAabbAddr = (u32)(u64)movAabbArray;
			io[i].movDetectPairs.numMovAabb = numMovAabb;
			io[i].movDetectPairs.fixAabbAddr = (u32)(u64)fixAabbArray;
			io[i].movDetectPairs.numFixAabb = numFixAabb;
			io[i].movDetectPairs.tmpSortsAddr = (u32)(u64)ALLOCATE(128, sizeof(SortData)*worldProperty.maxContactPairs);
			io[i].movDetectPairs.numTmpSorts = 0;
			io[i].movDetectPairs.maxSorts = worldProperty.maxContactPairs;
			io[i].movDetectPairs.commonBufAddr = (u32)(u64)movCommonBuf;
			//detect collision fix
			io[i].fixDetectPairs.chkAxis = chkAxis;
			io[i].fixDetectPairs.nonContactPairAddr = (u32)(u64)nonContactPair;
			io[i].fixDetectPairs.movAabbAddr = (u32)(u64)movAabbArray;
			io[i].fixDetectPairs.numMovAabb = numMovAabb;
			io[i].fixDetectPairs.fixAabbAddr = (u32)(u64)fixAabbArray;
			io[i].fixDetectPairs.numFixAabb = numFixAabb;
			io[i].fixDetectPairs.tmpSortsAddr = io[i].movDetectPairs.tmpSortsAddr;
			io[i].fixDetectPairs.numTmpSorts = 0;
			io[i].fixDetectPairs.maxSorts = worldProperty.maxContactPairs;
			io[i].fixDetectPairs.commonBufAddr = (u32)(u64)fixCommonBuf;

			mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_DETECT_SET, 0, 0);
		}
		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			ASSERT(numNewPairs + io[taskId].movDetectPairs.numTmpSorts <= worldProperty.maxContactPairs);
			memcpy(newPairs + numNewPairs, (void*)(u64)io[taskId].movDetectPairs.tmpSortsAddr, sizeof(SortData)*io[taskId].movDetectPairs.numTmpSorts);
			numNewPairs += io[taskId].movDetectPairs.numTmpSorts;
		}
		for(s32 i=maxTasks - 1;i >= 0;i--) {
			DEALLOCATE(io[i].movDetectPairs.tmpSortsAddr);
		}
		DEALLOCATE(commonBuf);
		DEALLOCATE(movCommonBuf);
		DEALLOCATE(fixCommonBuf);
		mTask->deallocate(io);
	}
}

void RigidBodies::mergeSetSPU(SortData *newPairs, u32& numNewPairs)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	SortData *tmpPairs = (SortData*)ALLOCATE(128, sizeof(SortData)*numNewPairs);
	memset(tmpPairs, 0, sizeof(SortData)*numNewPairs);

	s32 removedPairs = 0;
	{
		IOParamMergeSet *io = (IOParamMergeSet*)mTask->allocate(sizeof(IOParamMergeSet)*maxTasks);
		u32 *numNewPairsCommon = (u32*)mTask->allocate(sizeof(u32));
		*numNewPairsCommon = 0;
		s32 numBatch = (numNewPairs + maxTasks - 1) / maxTasks;
		s32 restSort = numNewPairs;
		s32 startSort = 0;
		for(s32 i=0;i < maxTasks;i++) {
			io[i].mergeSort.numSpu = maxTasks;
			io[i].mergeSort.buffAddr = (u32)(u64)tmpPairs;
			io[i].mergeSort.sortsAddr = (u32)(u64)newPairs;
			io[i].mergeSort.numSorts = numNewPairs;

			s32 curBatch = (restSort - numBatch ) > 0 ? numBatch : restSort;
			io[i].mergePairs.newSortsAddr = (u32)(u64)(newPairs + startSort);
			io[i].mergePairs.numNewSorts = curBatch;
			io[i].mergePairs.oldSortsAddr = (u32)(u64)sortedContactPairs;
			io[i].mergePairs.numOldSorts = numContactPairs;
			io[i].mergePairs.statesAddr = (u32)(u64)states;
			io[i].mergePairs.numStates = numInstances;
			io[i].mergePairs.maxInstances = worldProperty.maxInstances;

			io[i].numNewPairsAddr = (u32)(u64)numNewPairsCommon;
			io[i].contactsAddr = (u32)(u64)contactPairs;

			mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_MERGE_SET, 0, 0);

			restSort -= curBatch;
			startSort += curBatch;
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			removedPairs += io[taskId].mergePairs.numNewSorts;
		}
		removedPairs = numNewPairs - removedPairs;
		mTask->deallocate(io);
		mTask->deallocate(numNewPairsCommon);
	}

	numNewPairs -= removedPairs;
	numContactPairs += numNewPairs;


	DEALLOCATE(tmpPairs);
}

u32 RigidBodies::refreshContactPairsSPU()
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	IOParamRefreshPairs *io = (IOParamRefreshPairs*)mTask->allocate(sizeof(IOParamRefreshPairs)*maxTasks);

	u32 numRemovedContactPairs = 0;

	if(numContactPairs == 0) return 0;

	SortData *buffAddr = (SortData*)ALLOCATE(128, sizeof(SortData)*numContactPairs);

	s32 numBatch = (numContactPairs + maxTasks - 1)/maxTasks;
	s32 restPairs = numContactPairs;
	s32 startPairs = 0;
	for(s32 i=0;i < maxTasks;i++) {
		s32 curBatch = (restPairs - numBatch ) > 0 ? numBatch : restPairs;

		io[i].numSpu = maxTasks;
		io[i].startBatch = startPairs;
		io[i].numBatch = curBatch;
		io[i].contactPairsAddr = (u32)(u64)contactPairs;
		io[i].buffAddr = (u32)(u64)buffAddr;
		io[i].sortsAddr = (u32)(u64)sortedContactPairs;
		io[i].numContactPairs = (u32)(u64)numContactPairs;
		io[i].statesAddr = (u32)(u64)states;
		io[i].numStates = numInstances;
		io[i].numRemovedPairs = 0;
		mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_REFRESHCONTACTPAIRS, 0, 0);

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

	return numRemovedContactPairs;
}

void RigidBodies::detectCollisionsSPU(f32 timeStep)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbCollision_task);

	IOParamCollision *io = (IOParamCollision*)mTask->allocate(sizeof(IOParamCollision)*maxTasks);

	u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);

	u32 div = maxTasks*4;
	u32 numBatch = (numContactPairs + div - 1)/div;

	commonBuff[0] = 0; // 0 : unlocked , 1 : locked
	commonBuff[1] = 0; // start pos
	commonBuff[2] = MIN(numBatch, CONTACT_BATCH); // batch num

	for(s32 i=0;i < maxTasks;i++) {
		io[i].contactPairsAddr = (u32)(u64)contactPairs;
		io[i].numContactPairs = numContactPairs;
		io[i].sortsAddr = (u32)(u64)sortedContactPairs;
		io[i].statesAddr = (u32)(u64)states;
		io[i].bodiesAddr = (u32)(u64)bodies;
		io[i].collsAddr = (u32)(u64)collObjs;
		io[i].numStates = numInstances;
		io[i].numBodies = numBodies;
		io[i].ccdEnable = worldProperty.ccdEnable;
		io[i].timeStep = timeStep;
		mTask->startTask(i, (u32)((u64)&io[i]), 0, (u32)(u64)commonBuff, 0);
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	}

	DEALLOCATE(commonBuff);

	mTask->deallocate(io);
}

void RigidBodies::splitConstraints(SolverInfo *info, SolverGroup *groups, SortData *pairs, u32 numPairs)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	u32 movMask1 = ((1<<(MoveTypeActive + 1))|(1<<(MoveTypeKeyframe + 1))|(1<<(MoveTypeOneWay + 1)));
	u32 movMask2 = ~MOVE_TYPE_CAN_SLEEP;
	u32 movMask = movMask1|movMask2;
	u32 movKey = (1<<(MoveTypeKeyframe))|(1<<(MoveTypeKeyframe + 1));

	memset(info, 0, sizeof(SolverInfo));

	s32 bufSize = sizeof(u8)*numInstances;
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
			u32 pairCount=0;

			SolverGroup &group = groups[phaseId*MAX_SOLVER_GROUPS + groupId];
			u32 pairId = 0;

			for(;i < numPairs && pairCount < targetCount;i++) {
				u32 idxP = i>>5;
				u32 maskP = 1L<<(i&31);

				if(pairTable[idxP]&maskP)
					continue;

				u32 idxA = getStateA(pairs[i]);
				u32 idxB = getStateB(pairs[i]);
				u32 movA = 1<<getMovA(pairs[i]);
				u32 movB = 1<<getMovB(pairs[i]);

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

void RigidBodies::splitConstraintsSPU(SolverInfo *info, SolverGroup *groups, SortData *pairs, u32 numPairs)
{
	mTask->setTaskEntry(rbSolver_task);

	u32 buf[4];
	IOParamSplitConstraints *io = (IOParamSplitConstraints*)mTask->allocate(sizeof(IOParamSplitConstraints));
	io->numSpu = mTask->getMaxCores();
	io->numInstances = numInstances;
	io->numPairs = numPairs;
	io->pairsAddr = (u32)(u64)pairs;
	io->solverInfoAddr = (u32)(u64)info;
	io->solverGroupsAddr = (u32)(u64)groups;

	s32 taskId = 0;

	mTask->startTask(taskId,(u32)(u64)io, SOLVER_SPLIT_CONSTRAINTS, 0, 0);
	mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	mTask->deallocate(io);
}

u32 RigidBodies::createJointPairsSPU(SortData *pairs)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbSolver_task);

	u32 numActiveJoints = 0;

	IOParamCreateJointPairs *io = (IOParamCreateJointPairs*)mTask->allocate(sizeof(IOParamCreateJointPairs)*maxTasks);

	s32 numBatch = (numJoints + maxTasks - 1)/maxTasks;
	s32 rest = numJoints;
	s32 start = 0;
	for(s32 i=0;i < maxTasks;i++) {
		s32 curBatch = (rest - numBatch ) > 0 ? numBatch : rest;

		io[i].startJoint = start;
		io[i].batchJoint = curBatch;
		io[i].jointsAddr = (u32)(u64)joints;
		io[i].pairsAddr = (u32)(u64)ALLOCATE(128, sizeof(SortData)*curBatch);
		io[i].statesAddr = (u32)(u64)states;
		io[i].numActiveJoints = 0;
		mTask->startTask(i, (u32)((u64)&io[i]), SOLVER_CREATE_JOINT_PAIRS, 0, 0);

		rest -= curBatch;
		start += curBatch;
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		memcpy(pairs + numActiveJoints, (void*)(u64)io[taskId].pairsAddr, sizeof(SortData)*io[taskId].numActiveJoints);
		numActiveJoints += io[taskId].numActiveJoints;
	}

	for(s32 i=maxTasks - 1;i >= 0;i--) {
		DEALLOCATE(io[i].pairsAddr);
	}

	mTask->deallocate(io);

	return numActiveJoints;
}

void RigidBodies::solveConstraintsSPU(f32 timeStep)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbSolver_task);

	SortData *jointPairs = (SortData*)ALLOCATE(16, sizeof(SortData)*numJoints);

	SolverGroup *contactGroups = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup)*MAX_SOLVER_PHASES*MAX_SOLVER_GROUPS);
	SolverInfo *contactSolverInfo  = (SolverInfo*)ALLOCATE(128, sizeof(SolverInfo));

	SolverGroup *jointGroups = (SolverGroup*)ALLOCATE(128, sizeof(SolverGroup)*MAX_SOLVER_PHASES*MAX_SOLVER_GROUPS);
	SolverInfo *jointSolverInfo  = (SolverInfo*)ALLOCATE(128, sizeof(SolverInfo));

	contactSolverInfo->numPhases = 0;
	jointSolverInfo->numPhases = 0;

	{
		u32 numActiveJoints = 0;
		if(numJoints > 0)
			numActiveJoints = createJointPairsSPU(jointPairs);

		IOParamSplitConstraints *io_contact = (IOParamSplitConstraints*)mTask->allocate(sizeof(IOParamSplitConstraints));
		IOParamSplitConstraints *io_joint = (IOParamSplitConstraints*)mTask->allocate(sizeof(IOParamSplitConstraints));
		if(numContactPairs > 0) {
			mTask->setTaskEntry(rbSolver_task);
			io_contact->numSpu = mTask->getMaxCores();
			io_contact->numInstances = numInstances;
			io_contact->numPairs = numContactPairs;
			io_contact->pairsAddr = (u32)(u64)sortedContactPairs;
			io_contact->solverInfoAddr = (u32)(u64)contactSolverInfo;
			io_contact->solverGroupsAddr = (u32)(u64)contactGroups;
			mTask->startTask(0, (u32)(u64)io_contact, SOLVER_SPLIT_CONSTRAINTS, 0, 0);
		}

		if(numJoints > 0) {
			if(numActiveJoints > 0) {
				mTask->setTaskEntry(rbSolver_task);
				io_joint->numSpu = mTask->getMaxCores();
				io_joint->numInstances = numInstances;
				io_joint->numPairs = numActiveJoints;
				io_joint->pairsAddr = (u32)(u64)jointPairs;
				io_joint->solverInfoAddr = (u32)(u64)jointSolverInfo;
				io_joint->solverGroupsAddr = (u32)(u64)jointGroups;
				mTask->startTask(0, (u32)(u64)io_joint, SOLVER_SPLIT_CONSTRAINTS, 0, 0);
			}
		}
		if(numContactPairs > 0) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}
		if(numJoints > 0) {
			if(numActiveJoints > 0) {
				s32 taskId;
				u32 buf[4];
				mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
			}
		}
		mTask->deallocate(io_contact);
		mTask->deallocate(io_joint);
	}

	// Constraint Solver
	{
		IOParamSolver *io = (IOParamSolver*)mTask->allocate(sizeof(IOParamSolver)*maxTasks);

		nextGroupIndex = 0;

		for(s32 i=0;i < maxTasks;i++) {
			io[i].statesAddr = (u32)(u64)states;
			io[i].bodiesAddr = (u32)(u64)bodies;
			io[i].collsAddr = (u32)(u64)collObjs;

			io[i].contactSolverInfoAddr = (u32)(u64)contactSolverInfo;
			io[i].contactGroupsAddr = (u32)(u64)contactGroups;
			io[i].contactPairsAddr = (u32)(u64)sortedContactPairs;
			io[i].contactsAddr = (u32)(u64)contactPairs;
			io[i].numCollIteration = worldProperty.contactIteration;

			io[i].jointSolverInfoAddr = (u32)(u64)jointSolverInfo;
			io[i].jointGroupsAddr = (u32)(u64)jointGroups;
			io[i].jointPairsAddr = (u32)(u64)jointPairs;
			io[i].jointsAddr = (u32)(u64)joints;
			io[i].numJointIteration = worldProperty.jointIteration;

			io[i].numSPU = maxTasks;
			io[i].numStates = numInstances;
			io[i].numBodies = numBodies;
			io[i].numContactPairs = numContactPairs;
			io[i].numJoints = numJoints;
			io[i].timeStep = timeStep;
			io[i].separateBias = worldProperty.separateBias;
			io[i].deformMeshEnable = worldProperty.deformMeshEnable;

			io[i].nextGroupIndexAddr = (u32)((u64)&nextGroupIndex);

			mTask->startTask(i, (u32)((u64)&io[i]), SOLVER_CONSTRAINT_EX, 0, 0);
		}

		for(s32 i=0;i < maxTasks;i++) {
			s32 taskId;
			u32 buf[4];
			mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		}

		mTask->deallocate(io);
	}

	DEALLOCATE(jointSolverInfo);
	DEALLOCATE(jointGroups);
	DEALLOCATE(contactSolverInfo);
	DEALLOCATE(contactGroups);
	DEALLOCATE(jointPairs);

	// Post response
	{
		IOParamPostResponse *io = (IOParamPostResponse*)mTask->allocate(sizeof(IOParamPostResponse)*maxTasks);

		s32 numBatch = (numInstances + maxTasks - 1)/maxTasks;
		s32 restStates = numInstances;
		s32 startStates = 0;

		for(s32 i=0;i < maxTasks;i++) {
			s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;
			io[i].statesAddr = (u32)(u64)(states + startStates);
			io[i].numStates = curBatch;
			io[i].maxLinearVelocity = worldProperty.maxLinearVelocity;
			io[i].maxAngularVelocity = worldProperty.maxAngularVelocity;
			mTask->startTask(i, (u32)((u64)&io[i]), SOLVER_POSTRESPONSE, 0, 0);
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
}

void RigidBodies::sleepOrWakeupSPU()
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	s32 numBatch = (numInstances + maxTasks - 1) / maxTasks;
	s32 restStates = numInstances;
	s32 startStates = 0;

	IOParamIntegrate *io = (IOParamIntegrate*)mTask->allocate(sizeof(IOParamIntegrate)*maxTasks);

	for(s32 i=0;i < maxTasks;i++) {
		s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;

		io[i].startIdx = startStates;
		io[i].statesAddr = (u32)(u64)(states + startStates);
		io[i].numStates = curBatch;
		io[i].prevAddr = (u32)(u64)(prevStates + startStates);

		io[i].sleepEnable = worldProperty.sleepEnable;
		io[i].sleepCount = worldProperty.sleepCount;
		io[i].sleepInterval = worldProperty.sleepInterval;
		io[i].sleepLinearVelocity = worldProperty.sleepLinearVelocity;
		io[i].sleepAngularVelocity = worldProperty.sleepAngularVelocity;
		io[i].wakeLinearVelocity = worldProperty.wakeLinearVelocity;
		io[i].wakeAngularVelocity = worldProperty.wakeAngularVelocity;
		io[i].worldSleepCount = worldSleepCount;

		mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_SLEEP, 0, 0);
		restStates -= curBatch;
		startStates += curBatch;
	}

	for(s32 i=0;i < maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
	}

	mTask->deallocate(io);

	worldSleepCount++;
}

void RigidBodies::integrateSPU(f32 timeStep, bool lastSubStep)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	s32 numBatch = (numInstances + maxTasks - 1)/maxTasks;
	s32 restStates = numInstances;
	s32 startStates = 0;

	IOParamIntegrate *io = (IOParamIntegrate*)mTask->allocate(sizeof(IOParamIntegrate)*maxTasks);

	for(s32 i=0;i < maxTasks;i++) {
		s32 curBatch = (restStates - numBatch ) > 0 ? numBatch : restStates;

		io[i].startIdx = startStates;
		io[i].statesAddr = (u32)(u64)(states + startStates);
		io[i].numStates = curBatch;
		io[i].prevAddr = (u32)(u64)(prevStates + startStates);
		io[i].forcesAddr = (u32)(u64)(forces + startStates);
		io[i].bodiesAddr = (u32)(u64)bodies;
		io[i].collsAddr = (u32)(u64)collObjs;
		io[i].timeStep = timeStep;
		io[i].ccdEnable = worldProperty.ccdEnable;
		io[i].gravity = worldProperty.gravity;
		io[i].lastSubStep = lastSubStep;

		mTask->startTask(i, (u32)((u64)&io[i]), BROADPHASE_INTEGRATE, 0, 0);
		restStates -= curBatch;
		startStates += curBatch;
	}

	for(s32 i=0;i<maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId,buf[0], buf[1], buf[2], buf[3]);
	}

	mTask->deallocate(io);
}

void RigidBodies::spuFindAabbOverlap(Vector3 aabbMin, Vector3 aabbMax, u32 self, u32 target, AabbOverlapCallback *findAabbOverlap)
{
	s32 maxTasks = (s32)mTask->getMaxCores();

	mTask->setTaskEntry(rbBroadphase_task);

	VecInt3 inAabbMin, inAabbMax;
	worldVolume.worldToLocalPosition(aabbMin, aabbMax, inAabbMin, inAabbMax);

	SortData inAabb(0);
	setXMin(inAabb, inAabbMin.getX());
	setXMax(inAabb, inAabbMax.getX());
	setYMin(inAabb, inAabbMin.getY());
	setYMax(inAabb, inAabbMax.getY());
	setZMin(inAabb, inAabbMin.getZ());
	setZMax(inAabb, inAabbMax.getZ());

	u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
	u32 div = maxTasks * 4;
	u32 numBatch = (mNumAabbArray + div - 1)/div;
	commonBuff[0] = 0;					// 0 : unlocked , 1 : locked
	commonBuff[1] = 0;					// start pos
	commonBuff[2] = MIN(numBatch, 128);	// batch num

	IOParamFindAabbOverlap *io = (IOParamFindAabbOverlap*)mTask->allocate(sizeof(IOParamFindAabbOverlap)*maxTasks);

	for(s32 i=0;i < maxTasks;i++) {
		io[i].chkAxis = mCheckAxis;
		io[i].inAabb = inAabb;
		io[i].inTarget = target;
		io[i].inSelf = self;

		io[i].aabbAddr = (u32)(u64)mAabbArray;
		io[i].numAabb = mNumAabbArray;

		io[i].overlappedAddr = (u32)(u64)ALLOCATE(128, sizeof(vec_uint4)*numInstances);
		io[i].numOverlapped = 0;
		io[i].maxOverlapped = numInstances;

		mTask->startTask(i, (u32)((u64)&io[i]), FIND_AABB_OVERLAP, (u32)(u64)commonBuff, 0);
	}

	for(s32 i=0;i<maxTasks;i++) {
		s32 taskId;
		u32 buf[4];
		mTask->waitTask(taskId, buf[0], buf[1], buf[2], buf[3]);
		for(u32 s=0;s < io[taskId].numOverlapped;s++) {
			vec_uint4 stateIndex = *((vec_uint4*)(io[taskId].overlappedAddr + sizeof(vec_uint4)*s));
			findAabbOverlap->onOverlap(vec_extract(stateIndex, 0));
		}
	}

	for(s32 i=maxTasks - 1;i >= 0;i--) {
		DEALLOCATE(io[i].overlappedAddr);
	}

	mTask->deallocate(io);

	DEALLOCATE(commonBuff);
}

