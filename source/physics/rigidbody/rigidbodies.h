/*
 * rigidbodies.h
 *
 *  Created on: Jun 5, 2013
 *      Author: mike
 */

#ifndef RIGIDBODIES_H_
#define RIGIDBODIES_H_

#include "base/common.h"
#include "base/heapmanager.h"

#include "rigidbody/common/rigidbodyio.h"
#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/worldvolume.h"
#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/contact.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/heightfield.h"
#include "rigidbody/common/forces.h"
#include "rigidbody/common/joint.h"
#include "rigidbody/common/spring.h"
#include "rigidbody/common/parallelgroup.h"

#include "taskutil/marstaskmanager.h"

struct RigidBodyProperty
{
	f32	mass;
	Matrix3	inertia;
	f32	friction;
	f32	restitution;
	CollObject *collObject;

	RigidBodyProperty()
	{
		mass = 0.0f;
		friction = 0.6f;
		restitution = 0.2f;
		inertia = Matrix3::identity();
		collObject = NULL;
	}
};

struct InstanceProperty
{
	u8 moveType;
	u8 sleeping;
	u32 contactFilterSelf;
	u32 contactFilterTarget;
	Vector3 position;
	Quat orientation;
	Vector3 velocity;
	Vector3 angularVelocity;
	TrbDynBody *rigidBody;
	f32 linearDamping;
	f32 angularDamping;

	InstanceProperty()
	{
		moveType = MoveTypeFixed;
		sleeping = 0;
		contactFilterSelf = 0xffffffff;
		contactFilterTarget = 0xffffffff;
		position = velocity = angularVelocity = Vector3(0.0f);
		orientation = Quat::identity();
		rigidBody = NULL;
		linearDamping = 1.0f;
		angularDamping = 0.99f;
	}
};

struct JointProperty
{
	u8	jointType;
	u16	parentBody;
	u16	childBody;
	Vector3	anchor;
	Vector3	axis;

	f32	lowerLimit1;
	f32	upperLimit1;
	f32	lowerLimit2;
	f32	upperLimit2;
	f32	distance;

	f32	linearDamping;
	f32	angularDamping;
	f32	linearImpulseWeight;
	f32	angularImpulseWeight;
	f32	linearBias;
	f32	angularBias;
	f32	maxLinearImpulse;
	f32	maxAngularImpulse;
	f32	breakableLimit;
	bool warmStarting[6];

	JointProperty()
	{
		jointType = JointTypeBall;
		parentBody = childBody = 0;
		anchor = Vector3(0.0f);
		axis = Vector3(1.0f, 0.0f, 0.0f);
		lowerLimit1 = upperLimit1 = 0.0f;
		lowerLimit2 = upperLimit2 = 0.0f;
		linearDamping = angularDamping = 0.0f;
		linearImpulseWeight = angularImpulseWeight = 1.0f;
		linearBias = 0.2f;
		angularBias = 0.2f;
		maxLinearImpulse = 10000.0f;
		maxAngularImpulse = 10000.0f;
		breakableLimit = 0.0f;
		distance = 0.0f;
		warmStarting[0] = false;
		warmStarting[1] = false;
		warmStarting[2] = false;
		warmStarting[3] = false;
		warmStarting[4] = false;
		warmStarting[5] = false;
	}
};

class ContactCallback
{
public:
	ContactCallback() {}
	virtual ~ContactCallback() {}

	virtual void onContact(ContactPair& pair) = 0;
};

class SleepCallback
{
public:
	SleepCallback() {}
	virtual ~SleepCallback() {}

	virtual void onSleep(u16 stateIndex) = 0;
	virtual void onActive(u16 stateIndex) = 0;
};

struct WorldProperty
{
	u32 maxInstances;
	u32 maxDynBodies;
	u32 maxPrimitives;
	u32 maxJoints;
	u32 maxSprings;
	u32 maxContactPairs;

	Vector3 worldCenter;
	Vector3 worldExtent;
	Vector3 gravity;

	u8 subStepCount;
	u8 contactIteration;
	u8 jointIteration;
	f32 maxLinearVelocity;
	f32 maxAngularVelocity;
	f32 separateBias;

	bool sleepEnable;
	u16 sleepCount;
	u16 sleepInterval;
	f32 sleepLinearVelocity;
	f32 sleepAngularVelocity;
	f32 wakeLinearVelocity;
	f32 wakeAngularVelocity;

	bool ccdEnable;
	bool deformMeshEnable;

	ContactCallback *contactCallback;
	SleepCallback   *sleepCallback;

	WorldProperty()
	{
		maxInstances = 550;
		maxDynBodies = 40;
		maxPrimitives = 200;
		maxJoints = 600;
		maxSprings = 200;
		maxContactPairs = 5000;
		worldExtent = Vector3(200.0f);
		worldCenter = Vector3(0.0f, 90.0f, 0.0f);
		gravity = Vector3(0.0f, -9.8f, 0.0f);
		subStepCount = 1;
		contactIteration = 5;
		jointIteration = 8;
		maxLinearVelocity = 500.0f;
		maxAngularVelocity = 100.0f;
		separateBias = 0.1f;
		sleepEnable = true;
		sleepLinearVelocity = 0.1f;
		sleepAngularVelocity = 0.1f;
		sleepCount = 100;
		sleepInterval = 300;
		wakeLinearVelocity = 0.2f;
		wakeAngularVelocity = 0.2f;
		ccdEnable = false;
		deformMeshEnable = false;
		contactCallback = NULL;
		sleepCallback = NULL;
	}
};

class AabbOverlapCallback
{
public:
	AabbOverlapCallback() {}
	virtual ~AabbOverlapCallback() {}

	virtual void onOverlap(u16 stateIndex) = 0;
};

class RigidBodies
{
	friend class RayCast;
	friend class Particles;
	friend class HeightFluid;

private:
	HeapManager *mPool;

	s32 mTaskId;
	MARSTaskManager *mTask;

	u8 writeBuffer;
	u8 readBuffer;
	TrbState *states;

	u32 nextGroupIndex;

	WorldProperty worldProperty;

	u32	numInstances;
	u32	numBodies;
	u32	numCollObjs;
	u32	numPrims;
	u32	numJoints;
	u32	numSprings;
	u32	numContactPairs;
	u32	sizeContactTable;

	ATTRIBUTE_ALIGNED16(Forces *forces);
	ATTRIBUTE_ALIGNED16(TrbState *statesBuffer[2]);
	ATTRIBUTE_ALIGNED16(TrbState *prevStates);
	ATTRIBUTE_ALIGNED16(TrbDynBody *bodies);
	ATTRIBUTE_ALIGNED16(CollObject *collObjs);
	ATTRIBUTE_ALIGNED16(CollPrim *prims);
	ATTRIBUTE_ALIGNED16(Joint *joints);
	ATTRIBUTE_ALIGNED16(Spring 	*springs);
	ATTRIBUTE_ALIGNED16(ContactPair *contactPairs);
	ATTRIBUTE_ALIGNED16(SortData *sortedContactPairs);
	ATTRIBUTE_ALIGNED16(u32	*nonContactPair);
	ATTRIBUTE_ALIGNED16(u32	*userData);

	u16	numBodiesPool;
	u16	numCollsPool;
	u16	numPrimsPool;
	u16	numStatesPool;
	u16	numJointsPool;
	u16	numSpringsPool;

	ATTRIBUTE_ALIGNED16(u16	*bodiesPool);
	ATTRIBUTE_ALIGNED16(u16	*collsPool);
	ATTRIBUTE_ALIGNED16(u16	*primsPool);
	ATTRIBUTE_ALIGNED16(u16	*statesPool);
	ATTRIBUTE_ALIGNED16(u16	*jointsPool);
	ATTRIBUTE_ALIGNED16(u16	*springsPool);

	inline void clearNonContactPair();
	inline bool isCollidablePair(u16 i, u16 j);

	WorldVolume worldVolume;

	void assignStatesSPU(f32 timeStep, SortData *movAabbArray, u32& numMovAabb, SortData *fixAabbArray, u32& numFixAabb, s32& chkAxis);
	void detectPairsSPU(SortData *newPairs, u32& numNewPairs, u32 maxNewPairs, SortData *movAabbArray, u32 numMovAabb, SortData *fixAabbArray, u32 numFixAabb, s32 chkAxis);
	void mergePairsSPU(SortData *newPairs, u32 &numNewPairs);
	void addNewPairsSPU(SortData *newPairs, u32 numNewPairs);
	void broadPhaseSPU(f32 timeStep);
	void assignSetSPU(f32 timeStep, SortData *movAabbArray, u32& numMovAabb, SortData *fixAabbArray, u32& numFixAabb, s32& chkAxis);
	void detectSetSPU(SortData *newPairs, u32& numNewPairs, u32 maxNewPairs, SortData *movAabbArray, u32 numMovAabb, SortData *fixAabbArray, u32 numFixAabb, s32 chkAxis);
	void mergeSetSPU(SortData *newPairs, u32& numNewPairs);

	void integrateSPU(f32 timeStep, bool lastSubStep);

	u32 refreshContactPairsSPU();

	void applySpring();

	void splitConstraints(SolverInfo *info, SolverGroup *groups, SortData *pairs, u32 numPairs);
	void splitConstraintsSPU(SolverInfo *info, SolverGroup *groups, SortData *pairs, u32 numPairs);
	u32 createJointPairsSPU(SortData *pairs);
	void detectCollisionsSPU(f32 timeStep);
	void solveConstraintsSPU(f32 timeStep);

	u32 worldSleepCount;

	void sleepOrWakeupSPU();

	void updateContactPoints(f32 timeStep);
	void updateJointPoints(f32 timeStep);
	void updateFacetLocal(ContactPoint& cp, u32 p, LargeTriMesh *curLargeMesh, LargeTriMesh *preLargeMesh, f32 timeStep);
	void updateFacetLocal(Joint& joint, u32 p, LargeTriMesh *curLargeMesh, LargeTriMesh *preLargeMesh, f32 timeStep);

	void throwSleepCallback();
	void throwContactCallback();

	s32 createJoint();

	void allocateBuffers();
	void deallocateBuffers();

	void setupWorldSize();

	RigidBodies() {}

public:
	RigidBodies(MARSTaskManager *task, s32 taskId, HeapManager *pool);
	virtual ~RigidBodies()
	{
		deallocateBuffers();
	}

	void reset();
	void setup();

	void setupSimulate();
	void finalizeSimulate();

	void spuSimulate(f32 timeStep, u32 flag=0);

private:
	s32 mCheckAxis;
	s32 mNumAabbArray;
	SortData *mAabbArray;

public:
	void setupAabbOverlapUtil();
	void finalizeAabbOverlapUtil();

	void spuFindAabbOverlap(Vector3 aabbMin, Vector3 aabbMax, u32 self, u32 target, AabbOverlapCallback *findAabbOverlap);

public:
	void (*userAction)();

	void setUserAction(void (*us)()) {userAction = us;}

	s32 createCollPrim();
	bool deleteCollPrim(s32 primId);

	CollObject*	createCollObject();
	CollObject*	createCollObject(s32 numPrims);
	bool deleteCollObject(CollObject *coll);

	TrbDynBody*	createRigidBody(const RigidBodyProperty& param);
	bool deleteRigidBody(TrbDynBody *rigidbody);

	s32	createInstance(const InstanceProperty &param);
	bool deleteInstance(s32 stateIndex);

	s32	cloneInstance(u32 instance);

	bool instanceExists(u32 stateIndex) {return !statesBuffer[readBuffer][stateIndex].isDeleted();}

	s32 createJoint(const JointProperty& jointParam);
	bool deleteJoint(s32 jointIndex);

	s32 createSpring(f32 length, f32 ks, f32 kd, u16 stateIndexA, u16 stateIndexB, const Vector3& worldAnchor);
	s32 createSpring(f32 length, f32 ks, f32 kd, u16 stateIndexA, u16 stateIndexB, const Vector3& localAnchorA, const Vector3& localAnchorB);
	bool deleteSpring(s32 springIndex);

	void getWorldProperty(WorldProperty& worldProp) {worldProp = worldProperty;}
	void setWorldProperty(WorldProperty& worldProp) {worldProperty = worldProp;}

	Vector3	getGravity() {return worldProperty.gravity;}
	void setGravity(const Vector3 &gravity) {worldProperty.gravity = gravity;}

	u32	getSubStepCount() {return worldProperty.subStepCount;}
	void setSubStepCount(u32 i) {worldProperty.subStepCount = (u8)i;}

	u8 getContactIteration() {return worldProperty.contactIteration;}
	void setContactIteration(u8 i) {worldProperty.contactIteration = i;}

	u8 getJointIteration() {return worldProperty.jointIteration;}
	void setJointIteration(u8 i) {worldProperty.jointIteration = i;}

	f32 getMaxLinearVelocity() {return worldProperty.maxLinearVelocity;}
	void setMaxLinearVelocity(f32 value) {worldProperty.maxLinearVelocity = value;}

	f32	getMaxAngularVelocity() {return worldProperty.maxAngularVelocity;}
	void setMaxAngularVelocity(f32 value) {worldProperty.maxAngularVelocity = value;}

	f32	getSeparateBias() {return worldProperty.separateBias;}
	void setSeparateBias(f32 value) {worldProperty.separateBias = value;}

	bool getSleepEnable() {return worldProperty.sleepEnable;}
	void setSleepEnable(bool b);

	bool getCCDEnable() {return worldProperty.ccdEnable;}
	void setCCDEnable(bool b)  {worldProperty.ccdEnable = b;}

	bool getDeformMeshEnable() {return worldProperty.deformMeshEnable;}
	void setDeformMeshEnable(bool b)  {worldProperty.deformMeshEnable = b;}

	void getWorldSize(Vector3& center, Vector3& extent);
	void setWorldSize(const Vector3& center, const Vector3& extent);

	void appendNonContactPair(u16 stateIndexA, u16 stateIndexB);
	void removeNonContactPair(u16 stateIndexA, u16 stateIndexB);
	bool checkNonContactPair(u16 stateIndexA, u16 stateIndexB);

	inline u32 getNonContactPair(s32 index) {return nonContactPair[index];}
	inline void	setNonContactPair(s32 index, u32 v) {nonContactPair[index] = v;}

	inline u32 getTrbDynBodyCount() {return numBodies;}
	inline u32 getInstanceCount() {return numInstances;}
	inline u32 getJointCount() {return numJoints;}
	inline u32 getSpringCount() {return numSprings;}
	inline u32 getContactCount() {return numContactPairs;}

	inline TrbDynBody *getTrbDynBody(s32 stateIndex) {return &bodies[statesBuffer[readBuffer][stateIndex].trbBodyIdx];}
	inline CollObject *getCollObject(s32 stateIndex) {return getTrbDynBody(stateIndex)->getCollObject();}
	inline TrbDynBody *getTrbDynBodyByIndex(s32 index) {return &bodies[index];}
	inline Forces *getForce(s32 stateIndex) {return &forces[stateIndex];}
	inline Joint *getJoint(s32 jointIndex) {return &joints[jointIndex];}
	inline Spring *getSpring(s32 springIndex) {return &springs[springIndex];}
	inline ContactPair *getContactPair(s32 contactIndex) {return &contactPairs[getPair(sortedContactPairs[contactIndex])];}
	ContactPair	*getContactPairByStates(u16 stateIndexA, u16 stateIndexB);

	inline TrbState	*getState(s32 stateIndex) {return &statesBuffer[readBuffer][stateIndex];}
	inline void	setState(s32 stateIndex,const TrbState &state) {statesBuffer[readBuffer][stateIndex] = state;}

	inline TrbState	*getOldState(s32 stateIndex) {return &prevStates[stateIndex];}

	bool isAsleep(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].isAsleep();}
	bool isAwake(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].isAwake();}

	void wakeup(s32 stateIndex) {statesBuffer[readBuffer][stateIndex].wakeup();}
	void sleep(s32 stateIndex) {statesBuffer[readBuffer][stateIndex].sleep();}

	f32 getLinearDamping(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].linearDamping;}
	void setLinearDamping(s32 stateIndex, f32 value) {statesBuffer[readBuffer][stateIndex].linearDamping = value;}

	f32	getAngularDamping(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].angularDamping;}
	void setAngularDamping(s32 stateIndex, f32 value) {statesBuffer[readBuffer][stateIndex].angularDamping = value;}

	inline u8 getMoveType(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getMoveType();}
	inline void	setMoveType(s32 stateIndex, u8 moveType) {statesBuffer[readBuffer][stateIndex].setMoveType(moveType);}

	inline u32 getContactFilterSelf(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getContactFilterSelf();}
	inline void	setContactFilterSelf(s32 stateIndex, u32 filter) {statesBuffer[readBuffer][stateIndex].setContactFilterSelf(filter);}

	inline u32	getContactFilterTarget(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getContactFilterTarget();}
	inline void	setContactFilterTarget(s32 stateIndex, u32 filter) {statesBuffer[readBuffer][stateIndex].setContactFilterTarget(filter);}

	inline void	setContactFilter(s32 stateIndex, u32 filter) {setContactFilterSelf(stateIndex,filter);setContactFilterTarget(stateIndex,filter);}

	inline bool	getUseContactCallback(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getUseContactCallback()!=0;}
	inline void	setUseContactCallback(s32 stateIndex, bool b) {statesBuffer[readBuffer][stateIndex].setUseContactCallback(b?1:0);}

	inline bool	getUseSleepCallback(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getUseSleepCallback()!=0;}
	inline void	setUseSleepCallback(s32 stateIndex, bool b) {statesBuffer[readBuffer][stateIndex].setUseSleepCallback((u8)b);}

	inline bool	getUseCcd(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getUseCcd()!=0;}
	inline void	setUseCcd(s32 stateIndex, bool b) {statesBuffer[readBuffer][stateIndex].setUseCcd((u8)b);}

	inline bool	getUseSleep(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getUseSleep()!=0;}
	inline void	setUseSleep(s32 stateIndex, bool b) {statesBuffer[readBuffer][stateIndex].setUseSleep((u8)b);}

	inline u32 getUserData(s32 stateIndex) {return userData[stateIndex];}
	inline void setUserData(s32 stateIndex, u32 data) {userData[stateIndex] = data;}

	inline Vector3 getWorldPosition(s32 stateIndex, const Vector3& localPos);
	inline Vector3 getLocalPosition(s32 stateIndex,const Vector3& worldPos);
	inline Vector3 getPosition(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getPosition();}
	inline void	setPosition(s32 stateIndex, const Vector3& position);

	inline Quat	getOrientation(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getOrientation();}
	inline void	setOrientation(s32 stateIndex,const Quat &orientation);

	inline Vector3 getVelocity(s32 stateIndex)  {return statesBuffer[readBuffer][stateIndex].getLinearVelocity();}
	inline void	setVelocity(s32 stateIndex, const Vector3& velocity);

	inline Vector3 getAngularVelocity(s32 stateIndex) {return statesBuffer[readBuffer][stateIndex].getAngularVelocity();}
	inline void	setAngularVelocity(s32 stateIndex, const Vector3& angularVelocity);

	inline void	applyForce(s32 stateIndex, const Forces& force);
	inline void	applyForceByPosition(s32 stateIndex, const Vector3& force, const Vector3& worldPosition);

	inline void applyLinearImpulse(s32 stateIndex, const Vector3& impulse);
	inline void	applyAngularImpulse(s32 stateIndex, const Vector3& impulse);
	inline void	applyImpulseByPosition(s32 stateIndex, const Vector3& impulse, const Vector3& worldPosition);

	inline void	updateAABB(s32 stateIndex);
	inline void	updateAABBCcd(s32 stateIndex, f32 timeStep);

	inline void	movePosition(s32 stateIndex, const Vector3& position, f32 timeStep);
	inline void	moveOrientation(s32 stateIndex, const Quat& orientation, f32 timeStep);

	inline void	setPoseLocal(s32 jointIndex, const Quat& poseLocal);
	inline void	setPoseWorld(s32 jointIndex, const Quat& poseWorld);

	inline f32 getJointLinearImpulseWeight(s32 jointIndex) {return joints[jointIndex].linearImpulseWeight;}
	inline void	setJointLinearImpulseWeight(s32 jointIndex, f32 weight) {joints[jointIndex].linearImpulseWeight = weight;}

	inline f32 getJointAngularImpulseWeight(s32 jointIndex) {return joints[jointIndex].angularImpulseWeight;}
	inline void	setJointAngularImpulseWeight(s32 jointIndex, f32 weight) {joints[jointIndex].angularImpulseWeight = weight;}

	inline f32 getJointLinearDamping(s32 jointIndex) {return joints[jointIndex].linearDamping;}
	inline void	setJointLinearDamping(s32 jointIndex, f32 damping) {joints[jointIndex].linearDamping = damping;}

	inline f32 getJointAngularDamping(s32 jointIndex) {return joints[jointIndex].angularDamping;}
	inline void	setJointAngularDamping(s32 jointIndex, f32 damping) {joints[jointIndex].angularDamping = damping;}

	inline f32 getJointLinearBias(s32 jointIndex) {return joints[jointIndex].linearBias;}
	inline void	setJointLinearBias(s32 jointIndex, f32 value) {joints[jointIndex].linearBias = value;}

	inline f32 getJointAngularBias(s32 jointIndex) {return joints[jointIndex].angularBias;}
	inline void	setJointAngularBias(s32 jointIndex, f32 value) {joints[jointIndex].angularBias = value;}

	void setupRigidBody(u16 stateIndex);

	inline Vector3 getWorldPositionInternal(s32 stateIndex, const Vector3& localPos);
	inline Vector3 getLocalPositionInternal(s32 stateIndex, const Vector3& worldPos);
	inline Vector3 getPositionInternal(s32 stateIndex) {return statesBuffer[writeBuffer][stateIndex].getPosition();}
	inline void	setPositionInternal(s32 stateIndex, const Vector3& position);

	inline Quat	getOrientationInternal(s32 stateIndex) {return statesBuffer[writeBuffer][stateIndex].getOrientation();}
	inline void	setOrientationInternal(s32 stateIndex, const Quat& orientation);

	inline Vector3 getVelocityInternal(s32 stateIndex)  {return statesBuffer[writeBuffer][stateIndex].getLinearVelocity();}
	inline void	setVelocityInternal(s32 stateIndex, const Vector3& velocity);

	inline Vector3 getAngularVelocityInternal(s32 stateIndex) {return statesBuffer[writeBuffer][stateIndex].getAngularVelocity();}
	inline void	setAngularVelocityInternal(s32 stateIndex, const Vector3& angularVelocity);

	inline void	applyLinearImpulseInternal(s32 stateIndex, const Vector3& impulse);
	inline void	applyAngularImpulseInternal(s32 stateIndex, const Vector3& impulse);
	inline void	applyImpulseByPositionInternal(s32 stateIndex, const Vector3& impulse, const Vector3& worldPosition);

	inline void	updateAABBInternal(s32 stateIndex);
	inline void	updateAABBCcdInternal(s32 stateIndex, f32 timeStep);
};

inline void RigidBodies::clearNonContactPair()
{
	memset(nonContactPair, 0, sizeof(u32)*sizeContactTable);
}

inline bool RigidBodies::isCollidablePair(u16 i, u16 j)
{
	ASSERT(i != j);
	ASSERT(i < worldProperty.maxInstances);
	ASSERT(j < worldProperty.maxInstances);

	u32 minIdx = i > j ? j : i;
	u32 maxIdx = i > j ? i : j;
	u32 idx = maxIdx*(maxIdx - 1)/2 + minIdx;
	u32 mask = 1L<<(idx&31);
	return (nonContactPair[idx>>5]&mask) == 0;
}

inline void RigidBodies::updateAABB(s32 stateIndex)
{
	CollObject& coll = collObjs[statesBuffer[readBuffer][stateIndex].trbBodyIdx];
	statesBuffer[readBuffer][stateIndex].setAuxils(coll.getCenter(), coll.getHalf());
}

inline void RigidBodies::updateAABBCcd(s32 stateIndex, f32 timeStep)
{
	CollObject& coll = collObjs[statesBuffer[readBuffer][stateIndex].trbBodyIdx];
	statesBuffer[readBuffer][stateIndex].setAuxilsCcd(coll.getCenter(), coll.getHalf(), timeStep);
}

inline void RigidBodies::setPosition(s32 stateIndex, const Vector3& position)
{
	statesBuffer[readBuffer][stateIndex].setPosition(position);
	updateAABB(stateIndex);
}

inline void RigidBodies::setOrientation(s32 stateIndex, const Quat& orientation)
{
	statesBuffer[readBuffer][stateIndex].setOrientation(orientation);
	updateAABB(stateIndex);
}

inline void RigidBodies::movePosition(s32 stateIndex, const Vector3& position, f32 timeStep)
{
	TrbState& state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveType() != MoveTypeKeyframe) return;
	if(state.isAsleep()) state.wakeup();

	Vector3 prePos = getPosition(stateIndex);
	Vector3 vel = (position - prePos )/timeStep;

	setVelocity(stateIndex, vel);

	if(worldProperty.ccdEnable && getUseCcd(stateIndex))
		updateAABBCcd(stateIndex,timeStep);
	else
		updateAABB(stateIndex);
}

inline void RigidBodies::moveOrientation(s32 stateIndex, const Quat& orientation, f32 timeStep)
{
	TrbState& state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveType() != MoveTypeKeyframe) return;
	if(state.isAsleep()) state.wakeup();

	Quat preOri = getOrientation(stateIndex);
	Quat oldOri = orientation;

	if(dot(orientation, preOri) < 0.0f)
		oldOri = -orientation;

	Quat dq = (oldOri - preOri)/timeStep;
	dq = dq*2.0f*conj(preOri);
	Vector3 omega = dq.getXYZ();

	setAngularVelocity(stateIndex, omega);

	if(worldProperty.ccdEnable && getUseCcd(stateIndex))
		updateAABBCcd(stateIndex,timeStep);
	else
		updateAABB(stateIndex);
}

inline void RigidBodies::setVelocity(s32 stateIndex, const Vector3& velocity)
{
	statesBuffer[readBuffer][stateIndex].setLinearVelocity(velocity);
}

inline void RigidBodies::setAngularVelocity(s32 stateIndex, const Vector3& angularVelocity)
{
	statesBuffer[readBuffer][stateIndex].setAngularVelocity(angularVelocity);
}

inline void RigidBodies::applyForce(s32 stateIndex, const Forces& force)
{
	TrbState& state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP && state.isAsleep()) state.wakeup();

	forces[stateIndex].force += force.force;
	forces[stateIndex].torque += force.torque;
}

inline void RigidBodies::applyForceByPosition(s32 stateIndex, const Vector3& force, const Vector3& worldPosition)
{
	TrbState& state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP && state.isAsleep()) state.wakeup();

	forces[stateIndex].force += force;
	forces[stateIndex].torque += cross(worldPosition - state.getPosition(), force);
}

inline void RigidBodies::applyLinearImpulse(s32 stateIndex, const Vector3& impulse)
{
	TrbState& state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP && state.isAsleep()) state.wakeup();

	state.setLinearVelocity(state.getLinearVelocity() + bodies[state.trbBodyIdx].getMassInv()*impulse);
}

inline void RigidBodies::applyAngularImpulse(s32 stateIndex, const Vector3& impulse)
{
	TrbState& state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP && state.isAsleep()) state.wakeup();

	state.setAngularVelocity(state.getAngularVelocity() + bodies[state.trbBodyIdx].getBodyInertiaInv()*impulse);
}

inline void RigidBodies::applyImpulseByPosition(s32 stateIndex, const Vector3& impulse, const Vector3& worldPosition)
{
	TrbState &state = statesBuffer[readBuffer][stateIndex];
	if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP && state.isAsleep()) state.wakeup();

	state.setLinearVelocity(state.getLinearVelocity() + bodies[state.trbBodyIdx].getMassInv()*impulse);
	state.setAngularVelocity(state.getAngularVelocity() + bodies[state.trbBodyIdx].getBodyInertiaInv()*cross(worldPosition - state.getPosition(), impulse));
}

inline Vector3 RigidBodies::getWorldPosition(s32 stateIndex, const Vector3& localPos)
{
	return statesBuffer[readBuffer][stateIndex].getPosition() + rotate(statesBuffer[readBuffer][stateIndex].getOrientation(), localPos);
}

inline Vector3 RigidBodies::getLocalPosition(s32 stateIndex, const Vector3& worldPos)
{
	return rotate(conj(statesBuffer[readBuffer][stateIndex].getOrientation()), (worldPos - statesBuffer[readBuffer][stateIndex].getPosition()));
}

inline void RigidBodies::setPoseLocal(s32 jointIndex, const Quat& poseLocal)
{
	joints[jointIndex].targetFrame = Matrix3(poseLocal);
}

inline void RigidBodies::setPoseWorld(s32 jointIndex, const Quat& poseWorld)
{
	Matrix3 ma(getOrientation(joints[jointIndex].stateIndexA));
	Matrix3 mf(joints[jointIndex].frameA);
	joints[jointIndex].targetFrame = transpose(ma*mf)*Matrix3(poseWorld);
}

inline void RigidBodies::updateAABBInternal(s32 stateIndex)
{
	CollObject& coll = collObjs[statesBuffer[writeBuffer][stateIndex].trbBodyIdx];
	statesBuffer[writeBuffer][stateIndex].setAuxils(coll.getCenter(), coll.getHalf());
}

inline void RigidBodies::updateAABBCcdInternal(s32 stateIndex, f32 timeStep)
{
	CollObject& coll = collObjs[statesBuffer[writeBuffer][stateIndex].trbBodyIdx];
	statesBuffer[writeBuffer][stateIndex].setAuxilsCcd(coll.getCenter(), coll.getHalf(), timeStep);
}

inline Vector3 RigidBodies::getWorldPositionInternal(s32 stateIndex, const Vector3& localPos)
{
	return statesBuffer[writeBuffer][stateIndex].getPosition() + rotate(statesBuffer[writeBuffer][stateIndex].getOrientation(), localPos);
}

inline Vector3 RigidBodies::getLocalPositionInternal(s32 stateIndex, const Vector3& worldPos)
{
	return rotate(conj(statesBuffer[writeBuffer][stateIndex].getOrientation()), (worldPos - statesBuffer[writeBuffer][stateIndex].getPosition()));
}

inline void RigidBodies::setPositionInternal(s32 stateIndex, const Vector3& position)
{
	statesBuffer[writeBuffer][stateIndex].setPosition(position);
	updateAABB(stateIndex);
}

inline void RigidBodies::setOrientationInternal(s32 stateIndex, const Quat& orientation)
{
	statesBuffer[writeBuffer][stateIndex].setOrientation(orientation);
	updateAABB(stateIndex);
}

inline void RigidBodies::setVelocityInternal(s32 stateIndex, const Vector3& velocity)
{
	statesBuffer[writeBuffer][stateIndex].setLinearVelocity(velocity);
}

inline void RigidBodies::setAngularVelocityInternal(s32 stateIndex, const Vector3& angularVelocity)
{
	statesBuffer[writeBuffer][stateIndex].setAngularVelocity(angularVelocity);
}

inline void RigidBodies::applyLinearImpulseInternal(s32 stateIndex, const Vector3& impulse)
{
	TrbState& state = statesBuffer[writeBuffer][stateIndex];
	if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP && state.isAsleep()) state.wakeup();

	state.setLinearVelocity(state.getLinearVelocity() + bodies[state.trbBodyIdx].getMassInv()*impulse);
}

inline void RigidBodies::applyAngularImpulseInternal(s32 stateIndex, const Vector3& impulse)
{
	TrbState& state = statesBuffer[writeBuffer][stateIndex];
	if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP && state.isAsleep()) state.wakeup();

	state.setAngularVelocity(state.getAngularVelocity() + bodies[state.trbBodyIdx].getBodyInertiaInv()*impulse);
}

inline void RigidBodies::applyImpulseByPositionInternal(s32 stateIndex, const Vector3& impulse, const Vector3& worldPosition)
{
	TrbState& state = statesBuffer[writeBuffer][stateIndex];
	if(state.getMoveTypeBits()&MOVE_TYPE_CAN_SLEEP && state.isAsleep()) state.wakeup();

	state.setLinearVelocity(state.getLinearVelocity() + bodies[state.trbBodyIdx].getMassInv()*impulse);
	state.setAngularVelocity(state.getAngularVelocity() + bodies[state.trbBodyIdx].getBodyInertiaInv()*cross(worldPosition - state.getPosition(), impulse));
}

#endif /* RIGIDBODIES_H_ */
