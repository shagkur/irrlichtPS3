/*
 * simdfunc.h
 *
 *  Created on: Jun 1, 2013
 *      Author: mike
 */

#ifndef SIMDFUNC_H_
#define SIMDFUNC_H_

#include "base/common.h"

#ifdef __SPU__

static inline vec_float4 vec_min(vec_float4 a, vec_float4 b)
{
	return spu_sel(a, b, spu_cmpgt(a, b));
}

static inline vec_float4 vec_max(vec_float4 a, vec_float4 b)
{
	return spu_sel(b, a, spu_cmpgt(a, b));
}

static inline u32 cmpgtAny(const Vector3 &a, const Vector3 &b)
{
	vec_float4 va = a.get128();
	vec_float4 vb = b.get128();
	vec_uint4 cmp = spu_insert(0, spu_cmpgt(va, vb), 3);
	return spu_extract(spu_gather(cmp), 0);
}

static inline vec_float4 vec_dot3( vec_float4 vec0, vec_float4 vec1 )
{
    vec_float4 result;
    result = spu_mul( vec0, vec1 );
    result = spu_madd( spu_rlqwbyte( vec0, 4 ), spu_rlqwbyte( vec1, 4 ), result );
    return spu_madd( spu_rlqwbyte( vec0, 8 ), spu_rlqwbyte( vec1, 8 ), result );
}

#endif

static inline Vector3 read_Vector3(const f32* p)
{
	Vector3 v;
	loadXYZ(v, p);
	return v;
}

static inline Quat read_Quat(const f32* p)
{
	Quat vq;
	loadXYZW(vq, p);
	return vq;
}

static inline void store_Vector3(const Vector3 &src, f32* p)
{
	Vector3 v = src;
	storeXYZ(v, p);
}

static inline void store_Quat(const Quat &src, f32* p)
{
	Quat vq = src;
	storeXYZW(vq, p);
}

#endif /* SIMDFUNC_H_ */
