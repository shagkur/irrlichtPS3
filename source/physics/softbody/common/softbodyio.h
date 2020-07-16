/*
 * softbodyio.h
 *
 *  Created on: Jun 12, 2013
 *      Author: mike
 */

#ifndef SOFTBODYIO_H_
#define SOFTBODYIO_H_

#include "base/common.h"

#include "softbody/common/softstate.h"
#include "softbody/common/softcontact.h"
#include "softbody/common/softjoint.h"
#include "softbody/common/softbodyconfig.h"
#include "softbody/softbodygroup.h"

enum
{
	SOFTBODY_ASSIGNSTATES_RIG,
	SOFTBODY_FINDPAIRS,
	SOFTBODY_DETECTCONTACTS,
	SOFTBODY_SOLVE_CONSTRAINTS,
	SOFTBODY_PUSH_BODIES,
	SOFTBODY_INTEGRATE,
	SOFTBODY_BUILDMESH,
	SOFTBODY_SORT,
	SOFTBODY_SFTSFTCOLLISION,
	SOFTBODY_ACCUMPRESSURE,
};

ATTRIBUTE_ALIGNED16(struct) SoftPushImpulse
{
	Vector3 linearImpulse;
	Vector3 angularImpulse;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSoftBodyGroups
{
	SoftBodyGroupProperty curProperty;
	u32 sftStatesAddr;
	u32 sftJointsAddr;
	u32 maxContactPairs;
	u32 numContactsPclPcl;
	u32 numContactsPclRig;
	u32 contactPairsPclPclAddr;
	u32 contactPairsPclRigAddr;
	u32 contactSortsPclPclAddr;
	u32 contactSortsPclRigAddr;
	u32 pushImpulseAddr;

	u32 numVertices;
	u32 numIndices;
	u32 verticesAddr;
	u32 normalsAddr;
	u32 indicesAddr;

	u32 pclPressureLocalAddr;
	u32 pclPressureGlobalAddr;
};

ATTRIBUTE_ALIGNED16(struct) IOParamAssignStatesSoft
{
	u32 statesAddr;
	u32 numStates;
	u32 batchStartState;
	u32 numBatchStates;
	u32 aabbAddr[3];
	u32 numAabb;
	u32 worldVolumeAddr;
	f32 timeStep;
	u32 chkAxis;
	Vector3 s;
	Vector3 s2;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSortSoft
{
	u32 numSpu;
	u32 buffAddr;
	u32 sortsAddr;
	u32 numSorts;
};

ATTRIBUTE_ALIGNED16(struct) IOParamFindPairsSoft
{
	u32 worldVolumeAddr;
	u32 rigAabbAddr[3];
	u32 numRigAabb;
};

ATTRIBUTE_ALIGNED16(struct) IOParamDetectContactsSoft
{
	u32 rigStatesAddr;
	u32 rigCollsAddr;
	u32 numRigStates;
	u32 numRigBodies;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSolveConstraintsSoft
{
	u32 rigStatesAddr;
	u32 rigBodiesAddr;
	u32 numRigStates;
	u32 numRigBodies;
	f32 timeStep;
};

ATTRIBUTE_ALIGNED16(struct) IOParamPushBodiesSoft
{
	u32 groupAddr;
	u32 numGroups;
	u32 rigStartState;
	u32 rigStatesAddr;
	u32 numRigStates;
};

ATTRIBUTE_ALIGNED16(struct) IOParamIntegrateSoft
{
	f32 timeStep;
};

ATTRIBUTE_ALIGNED16(struct) IOParamBuildMeshSoft
{
	u32 ioParamAddr;
	u32 commonbuf;
};

#endif /* SOFTBODYIO_H_ */
