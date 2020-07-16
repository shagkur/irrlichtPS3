/*
 * heightfluid.h
 *
 *  Created on: Jun 11, 2013
 *      Author: mike
 */

#ifndef HEIGHTFLUID_H_
#define HEIGHTFLUID_H_

#include "base/common.h"
#include "base/heapmanager.h"

#include "heightfluid/common/heightfluidconfig.h"
#include "heightfluid/common/heightfluidio.h"

#include "heightfluid/heightfluidsurface.h"

#include "rigidbody/common/forces.h"

#include "taskutil/marstaskmanager.h"

class RigidBodies;

struct HeightFluidProperty
{
	u32	maxHeightFluidSurface;
	u32	maxInstances;

	HeightFluidProperty()
	{
		maxInstances = 550;
		maxHeightFluidSurface = 1;
	}
};

class HeightFluid
{
private:
	HeapManager *mPool;

	s32 mTaskId;
	MARSTaskManager *mTask;

	HeightFluidProperty heightfluidProperty;

	HeightFluidSurface **mHeightFluidSurfaces;
	u32 mNumHeightFluidSurfaces;

	ATTRIBUTE_ALIGNED16(HeightFluidSurface *mSurface);

	FieldPoint *mFields;
	Vector3 *mVertex;
	Vector3 *mNormal;
	f32 *mTexCoords;
	u16 *mIndices;

	ATTRIBUTE_ALIGNED16(Forces *mForces);

	RigidBodies *mRigidBodies;

	void calcWaveSPU(f32 timeStep);
	void applyForceToRigidBodiesSPU();
	void setMotionSPU();
	void calcPositionAndNormalSPU();

	void setupHeightFluidIO();

	void allocateBuffers();
	void deallocateBuffers();

	HeightFluid() {}

public:
	HeightFluid(MARSTaskManager *task, s32 taskId, HeapManager *pool);
	virtual ~HeightFluid()
	{
		deallocateBuffers();
	}

	void reset();
	void setup();

	void setHeightFluidProperty(HeightFluidProperty& hfluidProp) { heightfluidProperty = hfluidProp; }

	void attachRigidBodies(RigidBodies *rb);
	void detachRigidBodies();

	void addHeightFluidSurface(HeightFluidSurface *surface);
	void removeHeightFluidSurface(const HeightFluidSurface *surface);
	u32 getHeightFluidSurfaceCount() const { return mNumHeightFluidSurfaces; }

	void setupSimulate();
	void finalizeSimulate();

	void spuSimulate(f32 timeStep, u32 flag = 0);
};

#endif /* HEIGHTFLUID_H_ */
