/*
 * particleio.h
 *
 *  Created on: Jan 24, 2014
 *      Author: mike
 */

#ifndef PARTICLEIO_H_
#define PARTICLEIO_H_

#include "base/common.h"

enum {
	PARTICLE_CALCVARIANCE_PCL = 0,
	PARTICLE_ASSIGNSTATES_PCL,
	PARTICLE_ASSIGNSTATES_RIG,
	PARTICLE_DETECTPAIRS_PCL,
	PARTICLE_DETECTPAIRS_RIG,
	PARTICLE_DETECTCOLLISIONS_PCL,
	PARTICLE_DETECTCOLLISIONS_RIG,
	PARTICLE_REFRESHCONTACTPAIRS_PCL,
	PARTICLE_REFRESHCONTACTPAIRS_RIG,
	PARTICLE_INTEGRATE,
	PARTICLE_SORT,
	PARTICLE_SOLVE_CONSTRAINT_EX,
	PARTICLE_POSTRESPONSE,
	PARTICLE_NORMALIZE,
	PARTICLE_CROSS,
};

ATTRIBUTE_ALIGNED16(struct) IOParamAssignStatesPcl
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

ATTRIBUTE_ALIGNED16(struct) IOParamDetectPairsPcl
{
	u32 maxSorts;
	u32 chkAxis;
	u32 contactFilterSelf;
	u32 contactFilterTarget;
	u32 pclStatesAddr;
	u32 numPclStates;
	u32 rigStatesAddr;
	u32 numRigStates;
	u32 pclAabbAddr;
	u32 numPclAabb;
	u32 rigAabbAddr;
	u32 numRigAabb;
	u32 tmpSortsAddr;
	u32 numTmpSorts;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSortPcl
{
	u32 numSpu;
	u32 buffAddr;
	u32 sortsAddr;
	u32 numSorts;
};

ATTRIBUTE_ALIGNED16(struct) IOParamCollisionPcl
{
	u32 pclStatesAddr;
	u32 rigStatesAddr;
	u32 collsAddr;
	u32 numPclStates;
	u32 numRigStates;
	u32 numBodies;

	u32 sortsAddr;
	u32 contactPairsAddr;
	u32 numContactPairs;

	u32 startBatch;
	u32 numBatch;

	f32 timeStep;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSolverPcl
{
	u32 contactsPclAddr;
	u32 contactsRigAddr;
	u32 contactSortsPclAddr;
	u32 contactSortsRigAddr;

	u32 jointsAddr;
	u32 jointSortsPclAddr;
	u32 jointSortsRigAddr;

	u32 pclStatesAddr;
	u32 rigStatesAddr;
	u32 bodiesAddr;

	u32 contactSolverInfoPclAddr;
	u32 contactGroupsPclAddr;
	u32 contactSolverInfoRigAddr;
	u32 contactGroupsRigAddr;
	u32 numCollIteration;

	u32 jointSolverInfoPclAddr;
	u32 jointGroupsPclAddr;
	u32 jointSolverInfoRigAddr;
	u32 jointGroupsRigAddr;
	u32 numJointIteration;

	u32 numSPU;
	u32 numPclStates;
	u32 numRigStates;
	u32 numBodies;
	u32 numContactPairsPcl;
	u32 numContactPairsRig;
	u32 numJoints;

	f32 timeStep;
	f32 separateBias;

	bool twoWayInteraction;
};

ATTRIBUTE_ALIGNED16(struct) IOParamPostResponsePcl
{
	u32 statesAddr;
	u32 numStates;
	f32 maxLinearVelocity;
};

ATTRIBUTE_ALIGNED16(struct) IOParamRefreshPairsPcl
{
	u32 numSpu;

	u32 statesAddr;
	u32 numStates;

	u32 buffAddr;
	u32 sortsAddr;
	u32 contactPairsAddr;
	u32 numContactPairs;

	u32 startBatch;
	u32 numBatch;

	u32 numRemovedPairs;
};

ATTRIBUTE_ALIGNED16(struct) IOParamIntegratePcl
{
	u32 statesAddr;
	u32 numStates;
	f32 timeStep;
	f32 linearDamping;
	Vector3 gravity;
	Vector3 extraForce;
};

ATTRIBUTE_ALIGNED16(struct) IOParamNormalizePcl
{
	u32 outNormalsAddr;
	u32 startNormals;
	u32 numNormals;
	u32 inNormalBuffers[5];
	u32 numInNormalBuffers;
};

ATTRIBUTE_ALIGNED16(struct) IOParamCrossPcl
{
	u32 indicesAddr;
	u32 indicesAlign;
	u32 numIndices;
	u32 verticesAddr;
	u32 numVertices;
	u32 normalsAddr;
};

#endif /* PARTICLEIO_H_ */
