/*
 * testaabb.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef TESTAABB_H_
#define TESTAABB_H_

#include "base/common.h"

///////////////////////////////////////////////////////////////////////////////
// AABB

inline bool testAABB(const TrbState &stateA, const TrbState &stateB)
{
	if(fabs(stateA.center[0] - stateB.center[0]) > (stateA.half[0] + stateB.half[0])) return false;
	if(fabs(stateA.center[1] - stateB.center[1]) > (stateA.half[1] + stateB.half[1])) return false;
	if(fabs(stateA.center[2] - stateB.center[2]) > (stateA.half[2] + stateB.half[2])) return false;
	return true;
}

inline bool testAABB16(const SortData &aabbA, const SortData &aabbB)
{
#ifdef __SPU__
	vec_ushort8 vecAabbA = (vec_ushort8)aabbA.vdata[0];
	vec_ushort8 vecAabbB = (vec_ushort8)aabbB.vdata[0];
	vec_uint4 ptn_mask0 = ((vec_uint4){0x02031213, 0x06071617, 0x0A0B1A1B, 0x80808080});
	vec_uint4 ptn_mask1 = ((vec_uint4){0x10110001, 0x14150405, 0x18190809, 0x80808080});
	vec_ushort8 vecMin = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask1);
	vec_ushort8 vecMax = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask0);
	vec_ushort8 isGt = spu_cmpgt(vecMin, vecMax);
	return spu_extract(spu_gather(isGt), 0) == 0;
#else
	if(getXMax(aabbA) < getXMin(aabbB) || getXMin(aabbA) > getXMax(aabbB)) return false;
	if(getYMax(aabbA) < getYMin(aabbB) || getYMin(aabbA) > getYMax(aabbB)) return false;
	if(getZMax(aabbA) < getZMin(aabbB) || getZMin(aabbA) > getZMax(aabbB)) return false;
	return true;
#endif
}

#endif /* TESTAABB_H_ */
