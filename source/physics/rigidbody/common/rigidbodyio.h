/*
 * rigidbodyio.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef RIGIDBODYIO_H_
#define RIGIDBODYIO_H_

#include "base/sortcommon.h"

///////////////////////////////////////////////////////////////////////////////
// Event

enum {
	// Broadphase Event
	BROADPHASE_CALCVARIANCE = 0,
	BROADPHASE_ASSIGNSTATES,
	BROADPHASE_DETECTPAIRS_MOV,
	BROADPHASE_DETECTPAIRS_FIX,
	BROADPHASE_MERGEPAIRS,
	BROADPHASE_REFRESHCONTACTPAIRS,
	BROADPHASE_ADDNEWPAIRS,
	BROADPHASE_INTEGRATE,
	BROADPHASE_SLEEP,
	BROADPHASE_SORT,
	FIND_AABB_OVERLAP,

	BROADPHASE_ASSIGN_SET,
	BROADPHASE_DETECT_SET,
	BROADPHASE_MERGE_SET,

	// Solver Event
	SOLVER_SPLIT_CONSTRAINTS = 0,
	SOLVER_CREATE_JOINT_PAIRS,
	SOLVER_CONSTRAINT,
	SOLVER_CONSTRAINT_EX,
	SOLVER_POSTRESPONSE,
};

///////////////////////////////////////////////////////////////////////////////
// IO Parameter

ATTRIBUTE_ALIGNED16(struct) IOParamAssignStates
{
	u32 statesAddr;
	u32 numStates;
	u32 batchStartState;
	u32 numBatchStates;
	u32 movAabbAddr;
	u32 fixAabbAddr;
	u32 numMovAabb;
	u32 numFixAabb;
	u32 worldVolumeAddr;
	u32 chkAxis;
	u32 numSpu;
	f32	 timeStep;
	Vector3 s;
	Vector3 s2;
};

ATTRIBUTE_ALIGNED16(struct) IOParamDetectPairs
{
	u32 movAabbAddr;
	u32 fixAabbAddr;
	u32 numMovAabb;
	u32 numFixAabb;
	u32 tmpSortsAddr;
	u32 numTmpSorts;
	u32 maxSorts;
	u32 chkAxis;
	u32 nonContactPairAddr;
	u32 commonBufAddr;
};

ATTRIBUTE_ALIGNED16(struct) IOParamMergePairs
{
	u32 newSortsAddr;
	u32 numNewSorts;
	u32 oldSortsAddr;
	u32 numOldSorts;
	u32 statesAddr;
	u32 numStates;
	u32 maxInstances;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSort
{
	u32 numSpu;
	u32 buffAddr;
	u32 sortsAddr;
	u32 numSorts;
};

ATTRIBUTE_ALIGNED16(struct) IOParamIntegrate
{
	u32 startIdx;
	u32 statesAddr;
	u32 numStates;
	u32 prevAddr;
	u32 forcesAddr;
	u32 bodiesAddr;
	u32 collsAddr;
	f32 timeStep;
	bool ccdEnable;
	Vector3 gravity;

	bool sleepEnable;
	u16 sleepCount;
	u16 sleepInterval;
	f32 sleepLinearVelocity;
	f32 sleepAngularVelocity;
	f32 wakeLinearVelocity;
	f32 wakeAngularVelocity;
	u32 worldSleepCount;

	bool lastSubStep;
};

ATTRIBUTE_ALIGNED16(struct) IOParamRefreshPairs
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

ATTRIBUTE_ALIGNED16(struct) IOParamAddNewPairs
{
	u32 startPair;
	u32 batchPair;
	u32 numPairs;
	u32 contactsAddr;
	u32 pairsAddr;
	u32 newPairsAddr;
};

ATTRIBUTE_ALIGNED16(struct) IOParamFindAabbOverlap
{
	u32 chkAxis;
	SortData inAabb;
	u32 inTarget;
	u32 inSelf;
	u32 aabbAddr;
	u32 numAabb;
	u32 overlappedAddr;
	u32 numOverlapped;
	u32 maxOverlapped;
};

ATTRIBUTE_ALIGNED16(struct) IOParamCollision
{
	u32 statesAddr;
	u32 bodiesAddr;
	u32 collsAddr;
	u32 numStates;
	u32 numBodies;

	u32 sortsAddr;
	u32 contactPairsAddr;
	u32 numContactPairs;

	u32 startBatch;
	u32 numBatch;

	bool ccdEnable;

	f32	timeStep;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSplitConstraints
{
	u32 numSpu;
	u32 numInstances;
	u32 numPairs;
	u32 pairsAddr;
	u32 solverInfoAddr;
	u32 solverGroupsAddr;
};

ATTRIBUTE_ALIGNED16(struct) IOParamCreateJointPairs
{
	u32 startJoint;
	u32 batchJoint;
	u32 jointsAddr;
	u32 pairsAddr;
	u32 statesAddr;
	u32 numActiveJoints;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSolver
{
	u32 statesAddr;
	u32 bodiesAddr;
	u32 collsAddr;

	u32 contactSolverInfoAddr;
	u32 contactGroupsAddr;
	u32 contactPairsAddr;
	u32 contactsAddr;
	u32 numCollIteration;

	u32 jointSolverInfoAddr;
	u32 jointGroupsAddr;
	u32 jointPairsAddr;
	u32 jointsAddr;
	u32 numJointIteration;

	u32 numSPU;
	u32 numStates;
	u32 numBodies;
	u32 numContactPairs;
	u32 numJoints;

	f32	timeStep;
	f32	separateBias;
	bool deformMeshEnable;

	u32 nextGroupIndexAddr;
};

ATTRIBUTE_ALIGNED16(struct) IOParamPostResponse
{
	u32 statesAddr;
	u32 numStates;
	f32	maxLinearVelocity;
	f32	maxAngularVelocity;
};

ATTRIBUTE_ALIGNED16(struct) IOParamDetectSet
{
	IOParamSort movAabbSort;
	IOParamSort fixAabbSort;
	IOParamDetectPairs movDetectPairs;
	IOParamDetectPairs fixDetectPairs;
	IOParamSort preMergeSort;
	IOParamMergePairs mergePairs;
	IOParamSort postMergeSort;
};

ATTRIBUTE_ALIGNED16(struct) IOParamMergeSet
{
	IOParamSort mergeSort;
	IOParamMergePairs mergePairs;
	u32 numNewPairsAddr;
	u32 contactsAddr;
};

#endif /* RIGIDBODYIO_H_ */
