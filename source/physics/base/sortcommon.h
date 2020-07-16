/*
 * sortcommon.h
 *
 *  Created on: Jun 1, 2013
 *      Author: mike
 */

#ifndef SORTCOMMON_H_
#define SORTCOMMON_H_

#include "common.h"

#define NULL_KEY			0xffffffff

#ifndef __SPU__
	ATTRIBUTE_ALIGNED16(struct) SortData {
		union {
			u8   i8data[2][16];
			u16  i16data[2][8];
			u32  i32data[2][4];
		};

		SortData() {}
		SortData(u32 v)
		{
			i32data[0][0] = i32data[0][1] = i32data[0][2] = i32data[0][3]=v;
			i32data[1][0] = i32data[1][1] = i32data[1][2] = i32data[1][3]=v;
		}
	};

	#define SET8(vecId, elem, data)  s.i8data[vecId][elem] = (u8)data;
	#define SET16(vecId, elem, data) s.i16data[vecId][elem] = (u16)data;
	#define SET32(vecId, elem, data) s.i32data[vecId][elem] = (u32)data;
	#define GET8(vecId, elem)  s.i8data[vecId][elem];
	#define GET16(vecId, elem) s.i16data[vecId][elem];
	#define GET32(vecId, elem) s.i32data[vecId][elem];
#else
	ATTRIBUTE_ALIGNED16(struct) SortData {
		vec_uint4 vdata[2];

		SortData() {}
		SortData(u32 v)
		{
			vdata[0] = spu_splats(v);
			vdata[1] = spu_splats(v);
		}
	};

	#define SET8(vecId, elem, data)  s.vdata[vecId] = (vec_uint4)spu_insert((u8)data, (vec_uchar16)s.vdata[vecId], elem);
	#define SET16(vecId, elem, data) s.vdata[vecId] = (vec_uint4)spu_insert((u16)data, (vec_ushort8)s.vdata[vecId], elem);
	#define SET32(vecId, elem, data) s.vdata[vecId] = (vec_uint4)spu_insert((u32)data, (vec_uint4)s.vdata[vecId], elem);
	#define GET8(vecId, elem)  spu_extract((vec_uchar16)s.vdata[vecId], elem);
	#define GET16(vecId, elem) spu_extract((vec_ushort8)s.vdata[vecId], elem);
	#define GET32(vecId, elem) spu_extract((vec_uint4)s.vdata[vecId], elem);
#endif

inline static void setKey(SortData& s, u32 key) {SET32(1, 3, key);}
inline static u32 getKey(const SortData& s) {return GET32(1, 3);}

inline static void setPair(SortData& s, u32 pair)	{SET32(0, 0, pair);}
inline static void setStateA(SortData& s, u16 i)	{SET16(0, 2, i);}
inline static void setStateB(SortData& s, u16 i)	{SET16(0, 3, i);}
inline static void setBodyA(SortData& s, u16 i)		{SET16(0, 4, i);}
inline static void setBodyB(SortData& s, u16 i)		{SET16(0, 5, i);}
inline static void setMovA(SortData& s, u16 i)		{SET16(0, 6, i);}
inline static void setMovB(SortData& s, u16 i)		{SET16(0, 7, i);}
inline static void setFlag(SortData& s, u16 flag)	{SET16(1, 0, flag);}
inline static void setCallbackFlag(SortData& s, u16 flag)	{SET16(1, 1, flag);}

inline static u32 getPair(const SortData& s)   {return GET32(0, 0);}
inline static u16 getStateA(const SortData& s) {return GET16(0, 2);}
inline static u16 getStateB(const SortData& s) {return GET16(0, 3);}
inline static u16 getBodyA(const SortData& s)  {return GET16(0, 4);}
inline static u16 getBodyB(const SortData& s)  {return GET16(0, 5);}
inline static u16 getMovA(const SortData& s)   {return GET16(0, 6);}
inline static u16 getMovB(const SortData& s)   {return GET16(0, 7);}
inline static u16 getFlag(const SortData& s)   {return GET16(1, 0);}
inline static u16 getCallbackFlag(const SortData& s)	{return GET16(1, 1);}

// AABB
inline static void setXMin(SortData& s, u16 i) {SET16(0, 0, i);}
inline static void setXMax(SortData& s, u16 i) {SET16(0, 1, i);}
inline static void setYMin(SortData& s, u16 i) {SET16(0, 2, i);}
inline static void setYMax(SortData& s, u16 i) {SET16(0, 3, i);}
inline static void setZMin(SortData& s, u16 i) {SET16(0, 4, i);}
inline static void setZMax(SortData& s, u16 i) {SET16(0, 5, i);}
inline static void setXYZMin(SortData& s, u16 i, s32 axis) {SET16(0, axis<<1, i);}
inline static void setXYZMax(SortData& s, u16 i, s32 axis) {SET16(0, 1 + (axis<<1), i);}

inline static u16 getXMin(const SortData& s) {return GET16(0, 0);}
inline static u16 getXMax(const SortData& s) {return GET16(0, 1);}
inline static u16 getYMin(const SortData& s) {return GET16(0, 2);}
inline static u16 getYMax(const SortData& s) {return GET16(0, 3);}
inline static u16 getZMin(const SortData& s) {return GET16(0, 4);}
inline static u16 getZMax(const SortData& s) {return GET16(0, 5);}
inline static u16 getXYZMin(const SortData& s, s32 axis) {return GET16(0, axis<<1);}
inline static u16 getXYZMax(const SortData& s, s32 axis) {return GET16(0, 1 + (axis<<1));}

inline static void setStateId(SortData& s, u16 i) {SET16(0, 6, i);}
inline static void setBodyId(SortData& s, u16 i)  {SET16(0, 7, i);}
inline static void setMovType(SortData& s, u16 i) {SET16(1, 0, i);}
inline static void setCallback(SortData& s, u16 i){SET16(1, 1, i);}
inline static void setSelf(SortData& s, u32 i)    {SET32(1, 1, i);}
inline static void setTarget(SortData& s, u32 i)  {SET32(1, 2, i);}

inline static u16 getStateId(const SortData& s) {return GET16(0, 6);}
inline static u16 getBodyId(const SortData& s)  {return GET16(0, 7);}
inline static u16 getMovType(const SortData& s) {return GET16(1, 0);}
inline static u16 getCallback(const SortData& s){return GET16(1, 1);}
inline static u32 getSelf(const SortData& s)    {return GET32(1, 1);}
inline static u32 getTarget(const SortData& s)  {return GET32(1, 2);}

inline static void setStatePair(SortData& s, u16 i, u16 j)
{
	u32 minIdx = i < j ? i : j;
	u32 maxIdx = i > j ? i : j;
	setKey(s, maxIdx*(maxIdx - 1)/2 + minIdx);
	setStateA(s, i);
	setStateB(s, j);
}

inline static void setStatePair2(SortData& s, u16 i, u16 j)
{
	u32 minIdx = i < j ? i : j;
	u32 maxIdx = i > j ? i : j;
	setKey(s, maxIdx*(maxIdx + 1)/2 + minIdx);
	setStateA(s, i);
	setStateB(s, j);
}

#endif /* SORTCOMMON_H_ */
