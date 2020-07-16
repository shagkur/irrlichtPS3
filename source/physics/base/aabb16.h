/*
 * aabb16.h
 *
 *  Created on: Jun 1, 2013
 *      Author: mike
 */

#ifndef AABB16_H_
#define AABB16_H_

#include "common.h"

#ifdef __SPU__

typedef vec_uint4 AABB16;

inline static void setXMin(AABB16& s, u16 i) {s = (vec_uint4)spu_insert(i, (vec_ushort8)s, 0);}
inline static void setXMax(AABB16& s, u16 i) {s = (vec_uint4)spu_insert(i, (vec_ushort8)s, 1);}
inline static void setYMin(AABB16& s, u16 i) {s = (vec_uint4)spu_insert(i, (vec_ushort8)s, 2);}
inline static void setYMax(AABB16& s, u16 i) {s = (vec_uint4)spu_insert(i, (vec_ushort8)s, 3);}
inline static void setZMin(AABB16& s, u16 i) {s = (vec_uint4)spu_insert(i, (vec_ushort8)s, 4);}
inline static void setZMax(AABB16& s, u16 i) {s = (vec_uint4)spu_insert(i, (vec_ushort8)s, 5);}
inline static void setData16_0(AABB16& s, u16 i) {s = (vec_uint4)spu_insert(i, (vec_ushort8)s, 6);}
inline static void setData16_1(AABB16& s, u16 i) {s = (vec_uint4)spu_insert(i, (vec_ushort8)s, 7);}
inline static void setData32(AABB16& s, u32 i) {s = (vec_uint4)spu_insert(i, (vec_uint4)s, 3);}

inline static u16 getXMin(const AABB16& s) {return spu_extract((vec_ushort8)s, 0);}
inline static u16 getXMax(const AABB16& s) {return spu_extract((vec_ushort8)s, 1);}
inline static u16 getYMin(const AABB16& s) {return spu_extract((vec_ushort8)s, 2);}
inline static u16 getYMax(const AABB16& s) {return spu_extract((vec_ushort8)s, 3);}
inline static u16 getZMin(const AABB16& s) {return spu_extract((vec_ushort8)s, 4);}
inline static u16 getZMax(const AABB16& s) {return spu_extract((vec_ushort8)s, 5);}
inline static u16 getData16_0(const AABB16& s) {return spu_extract((vec_ushort8)s, 6);}
inline static u16 getData16_1(const AABB16& s) {return spu_extract((vec_ushort8)s, 7);}
inline static u32 getData32(const AABB16& s) {return spu_extract((vec_uint4)s, 3);}

inline bool testAABB(const AABB16 &aabbA, const AABB16 &aabbB)
{
	vec_ushort8 vecAabbA = (vec_ushort8)aabbA;
	vec_ushort8 vecAabbB = (vec_ushort8)aabbB;
	vec_uint4 ptn_mask0 = ((vec_uint4){0x02031213, 0x06071617, 0x0A0B1A1B, 0x80808080});
	vec_uint4 ptn_mask1 = ((vec_uint4){0x10110001, 0x14150405, 0x18190809, 0x80808080});
	vec_ushort8 vecMin = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask1);
	vec_ushort8 vecMax = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask0);
	vec_ushort8 isGt = spu_cmpgt(vecMin, vecMax);
	return spu_extract(spu_gather(isGt), 0) == 0;
}

#else

ATTRIBUTE_ALIGNED16(struct) AABB16
{
	u32 data[4];
};

inline static void setXMin(AABB16& s, u16 i) {s.data[0] &= 0x0000ffff; s.data[0] |= i << 16;}
inline static void setXMax(AABB16& s, u16 i) {s.data[0] &= 0xffff0000; s.data[0] |= i;}
inline static void setYMin(AABB16& s, u16 i) {s.data[1] &= 0x0000ffff; s.data[1] |= i << 16;}
inline static void setYMax(AABB16& s, u16 i) {s.data[1] &= 0xffff0000; s.data[1] |= i;}
inline static void setZMin(AABB16& s, u16 i) {s.data[2] &= 0x0000ffff; s.data[2] |= i << 16;}
inline static void setZMax(AABB16& s, u16 i) {s.data[2] &= 0xffff0000; s.data[2] |= i;}
inline static void setData16_0(AABB16& s, u16 i) {s.data[3]&=0x0000ffff; s.data[3] |= i << 16;}
inline static void setData16_1(AABB16& s, u16 i) {s.data[3]&=0xffff0000; s.data[3] |= i;}
inline static void setData32(AABB16& s, u32 i) {s.data[3] = i;}

inline static u16 getXMin(const AABB16& s) {return ((u16)(s.data[0] >> 16));}
inline static u16 getXMax(const AABB16& s) {return ((u16)(s.data[0] & 0x0000ffff));}
inline static u16 getYMin(const AABB16& s) {return ((u16)(s.data[1] >> 16));}
inline static u16 getYMax(const AABB16& s) {return ((u16)(s.data[1] & 0x0000ffff));}
inline static u16 getZMin(const AABB16& s) {return ((u16)(s.data[2] >> 16));}
inline static u16 getZMax(const AABB16& s) {return ((u16)(s.data[2] & 0x0000ffff));}
inline static u16 getData16_0(const AABB16& s) {return ((u16)(s.data[3] >> 16));}
inline static u16 getData16_1(const AABB16& s) {return ((u16)(s.data[3] & 0x0000ffff));}
inline static u32 getData32(const AABB16& s) {return s.data[3];}

inline bool testAABB(const AABB16 &aabbA, const AABB16 &aabbB)
{
	if(getXMax(aabbA) < getXMin(aabbB) || getXMin(aabbA) > getXMax(aabbB)) return false;
	if(getYMax(aabbA) < getYMin(aabbB) || getYMin(aabbA) > getYMax(aabbB)) return false;
	if(getZMax(aabbA) < getZMin(aabbB) || getZMin(aabbA) > getZMax(aabbB)) return false;
	return true;
}

inline AABB16 mergeAABB(const AABB16 &aabbA,const AABB16 &aabbB)
{
	AABB16 aabb;
	setXMin(aabb, MIN(getXMin(aabbA), getXMin(aabbB)));
	setXMax(aabb, MAX(getXMax(aabbA), getXMax(aabbB)));
	setYMin(aabb, MIN(getYMin(aabbA), getYMin(aabbB)));
	setYMax(aabb, MAX(getYMax(aabbA), getYMax(aabbB)));
	setZMin(aabb, MIN(getZMin(aabbA), getZMin(aabbB)));
	setZMax(aabb, MAX(getZMax(aabbA), getZMax(aabbB)));
	return aabb;
}

#endif

#endif /* AABB16_H_ */
