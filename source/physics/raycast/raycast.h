/*
 * raycast.h
 *
 *  Created on: Jun 10, 2013
 *      Author: mike
 */

#ifndef RAYCAST_H_
#define RAYCAST_H_

#include "base/common.h"
#include "base/heapmanager.h"
#include "base/sortcommon.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/worldvolume.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/trbstatevec.h"

#include "raycast/common/raycastio.h"
#include "raycast/common/raycastconfig.h"
#include "raycast/common/ray.h"

#include "taskutil/marstaskmanager.h"

#define REPSILON 				0.00001f

class RigidBodies;

struct RaycastProperty
{
	u32	maxInstances;
	u32	maxRayGroups;
	Vector3	worldCenter;
	Vector3	worldExtent;
	bool useRigidBodyWorldSize;

	RaycastProperty()
	{
		maxInstances = 550;
		maxRayGroups = 1;
		worldExtent = Vector3(200.0f);
		worldCenter = Vector3(0.0f, 90.0f, 0.0f);
		useRigidBodyWorldSize = true;
	}
};

class RayCast
{
private:
	HeapManager *mPool;

	s32 mTaskId;
	MARSTaskManager *mTask;

	TrbState *states;
	CollObject *collObjs;
	u32 numInstances;

	WorldVolume worldVolume;

	RigidBodies *mRigidBodies;

	RaycastProperty raycastProperty;

	bool broadphaseOk;

	IOParamRayCastCommon mRayCastCommonIO;

	u32 sizeContactTable;
	ATTRIBUTE_ALIGNED16(u32 *nonContactFlag);

	s32 numAabb;
	ATTRIBUTE_ALIGNED16(SortData *aabbArray[6]);

	void broadPhaseSPU(SortData *aabbArray[6], s32& numAabb);

	void allocateBuffers();
	void deallocateBuffers();

	void setupWorldSize();

	RayCast() {}

public:
	RayCast(MARSTaskManager *task, s32 taskId, HeapManager *pool);
	~RayCast();

	void reset();
	void setup() {}

	void attachRigidBodies(RigidBodies *rb);
	void detachRigidBodies();

	void setupRayCast();
	void finalizeRayCast();

	void setRaycastProperty(RaycastProperty& raycastProp) { raycastProperty = raycastProp; }

	void getWorldSize(Vector3& center, Vector3& extent);
	void setWorldSize(const Vector3& center, const Vector3& extent);

	void appendNonContactPair(u8 rayGroup, u16 rigidBodyIndex);
	void removeNonContactPair(u8 rayGroup, u16 rigidBodyIndex);
	void clearNonContactFlag();

	void spuRayCast(Ray *rays, s32 numRay, u32 flag);

	void spuRayCastBroadphase();
};
#endif /* RAYCAST_H_ */
