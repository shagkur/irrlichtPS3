/*
 * particles.h
 *
 *  Created on: Jan 27, 2014
 *      Author: mike
 */

#ifndef PARTICLES_H_
#define PARTICLES_H_

#include "base/common.h"
#include "base/heapmanager.h"

#include "particle/common/pclstate.h"
#include "particle/common/pclcontact.h"
#include "particle/common/pcljoint.h"
#include "particle/common/particleio.h"
#include "particle/common/particleconfig.h"

#include "particle/particlegroup.h"

#include "rigidbody/common/worldvolume.h"
#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/contact.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/heightfield.h"
#include "rigidbody/common/parallelgroup.h"
#include "rigidbody/rigidbodies.h"

#include "taskutil/marstaskmanager.h"

class ParticleContactCallback
{
public:
	ParticleContactCallback() {}
	virtual ~ParticleContactCallback() {}

	virtual void onContactPclPcl(ParticleGroup *group, PclContactPair *contacts, u32 numContacts) = 0;
	virtual void onContactPclRig(ParticleGroup *group, PclContactPair *contacts, u32 numContacts) = 0;
};

struct ParticleWorldProperty
{
	u32	maxParticleGroups;
	u32	maxContactPairs;
	u32	maxInstances;

	u32	subStepCount;
	Vector3 worldCenter;
	Vector3	worldExtent;
	bool useRigidBodyWorldSize;

	ParticleContactCallback *contactCallback;

	ParticleWorldProperty()
	{
		subStepCount = 1;
		maxInstances = 550;
		maxParticleGroups = 1;
		maxContactPairs = 5000;
		worldExtent = Vector3(200.0f);
		worldCenter = Vector3(0.0f, 90.0f, 0.0f);
		useRigidBodyWorldSize = true;
		contactCallback = NULL;
	}
};

class Particles
{
private:
	HeapManager *mPool;

	s32 mTaskId;
	MARSTaskManager *mTask;

	u8 writeBuffer;
	u8 readBuffer;

	ParticleGroupProperty curProperty;

	PclState *pclStates;
	PclJoint *pclJoints;
	u32 numVertices;
	u32 numIndices;
	Vector3 *pclVtx;
	Vector3 *pclNml;
	u16 *pclIdx;

	RigidBodies *mRigidBodies;
	TrbState *states;
	CollObject *colls;
	TrbDynBody *bodies;
	u32 numInstances;
	u32 numBodies;
	SortData *rigAabbArray[3];
	u32 numRigAabb;

	ParticleWorldProperty worldProperty;

	u32 numParticleGroups;
	u32 numContactPairsPcl;
	u32 numContactPairsRig;

	ATTRIBUTE_ALIGNED16(ParticleGroup **particleGroups);
	ATTRIBUTE_ALIGNED16(Vector3 *extForces);
	ATTRIBUTE_ALIGNED16(PclContactPair *contactsPcl);
	ATTRIBUTE_ALIGNED16(PclContactPair *contactsRig);
	ATTRIBUTE_ALIGNED16(SortData *contactSortsPcl);
	ATTRIBUTE_ALIGNED16(SortData *contactSortsRig);

	WorldVolume worldVolume;

	void preBroadPhaseRigSPU(f32 timeStep);
	void postBroadPhaseRigSPU();
	void broadPhasePclSPU(f32 timeStep);
	void assignStatesSPU(f32 timeStep, SortData *pclAabbArray, u32& numPclAabb, s32& chkAxis);
	void detectPairsSPU(SortData *pclAabbArray, u32 numPclAabb, s32 chkAxis);
	void refreshContactPairsSPU();
	void detectCollisionsSPU(f32 timeStep);
	void solveConstraintsSPU(f32 timeStep);
	void integrateSPU(f32 timeStep);
	void buildMeshSPU();
	void splitPairsExPcl(SolverInfo *info, SolverGroup *groups, SortData *pairs, u32 numPairs);
	void splitPairsExRig(SolverInfo *info, SolverGroup *groups, SortData *pairs, u32 numPairs);

	void allocateBuffers();
	void deallocateBuffers();

	void setupWorldSize();

	Particles() {}

public:
	Particles(MARSTaskManager *task, s32 taskId, HeapManager *pool);
	virtual ~Particles()
	{
		deallocateBuffers();
	}

	void reset();
	void setup();

	void attachRigidBodies(RigidBodies *rb);
	void detachRigidBodies();

	void addParticleGroup(ParticleGroup *pclGroup);
	void removeParticleGroup(const ParticleGroup *pclGroup);
	u32 getParticleGroupCount() const { return numParticleGroups; }

	void setupSimulate();
	void finalizeSimulate();

	void spuSimulate(f32 timeStep);

	ParticleWorldProperty& getWorldProperty() { return worldProperty; }
	void setWorldProperty(const ParticleWorldProperty& property) { worldProperty = property; }

	u32 getSubStepCount() const { return worldProperty.subStepCount; }
	void setSubStepCount(u32 i) { worldProperty.subStepCount = i; }

	void getWorldSize(Vector3& center, Vector3& extent);
	void setWorldSize(const Vector3& center, const Vector3& extent);
};

#endif /* PARTICLES_H_ */
