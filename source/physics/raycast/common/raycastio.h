/*
 * raycastio.h
 *
 *  Created on: Jun 7, 2013
 *      Author: mike
 */

#ifndef RAYCASTIO_H_
#define RAYCASTIO_H_

#include "base/common.h"

enum
{
	RAYCAST_ASSIGNSTATES = 0,
	RAYCAST_SORTAABB,
	RAYCAST_RAYCAST_SINGLE,
	RAYCAST_RAYCAST_STRIDE
};

ATTRIBUTE_ALIGNED16(struct) IOParamBroadPhase
{
	u32 statesAddr;
	u32 numStates;
	u32 batchStartState;
	u32 numBatchStates;
	u32 aabbAddr[6];
	u32 numAabb;
	u32 worldVolumeAddr;
};

ATTRIBUTE_ALIGNED16(struct) IOParamSortAabbs
{
	u32 numSpu;
	u32 buffAddr;
	u32 aabbAddr[6];
	u32 numAabb;
};

ATTRIBUTE_ALIGNED16(struct) IOParamRayCastCommon
{
	u32 statesAddr;
	u32 collsAddr;
	u32 numStates;
	u32 aabbAddr[6];
	u32 numAabb;
	u32 nonContactFlagAddr;
	u32 numNonContactFlag;
	u32 worldVolumeAddr;
	u32 maxRayGroups;
};

#endif /* RAYCASTIO_H_ */
