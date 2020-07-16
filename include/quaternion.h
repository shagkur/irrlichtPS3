/*
 * quaternion.h
 *
 *  Created on: May 16, 2013
 *      Author: mike
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

#include "irrtypes.h"
#include "vector3d.h"

#define _VECTORMATH_PERM_YXXW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_X, _VECTORMATH_PERM_X, _VECTORMATH_PERM_W })
#define _VECTORMATH_PERM_ZZYW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Z, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_W })
#define _VECTORMATH_PERM_XBZW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_B, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_W })

namespace irr
{
	namespace core
	{
		typedef Quat quaternion;

		inline void toEulerRadians(const quaternion& q, vector3df& euler)
		{
			/*
			f32 W = q.getW();
			f32 X = q.getX();
			f32 Y = q.getY();
			f32 Z = q.getZ();
			const f32 sqrw = W*W;
			const f32 sqrx = X*X;
			const f32 sqry = Y*Y;
			const f32 sqrz = Z*Z;
			const f32 test = -2.0f*(X*Z - Y*W);

			if(core::equals(test, 1.0f, 0.000001f)) {
				euler.setZ(-2.0f*atan2f(X, W));
				euler.setX(0.0f);
				euler.setY(core::PI/2.0f);
			} else if(core::equals(test, -1.0f, 0.000001f)) {
				euler.setZ(2.0f*atan2f(X, W));
				euler.setX(0.0f);
				euler.setY(core::PI/-2.0f);
			} else {
				euler.setZ(atan2f(2.0f*(X*Y + Z*W), (sqrx - sqry - sqrz + sqrw)));
				euler.setX(atan2f(2.0f*(Y*Z + X*W), (-sqrx - sqry + sqrz + sqrw)));
				euler.setY(asinf(core::clamp(test, -1.0f, 1.0f)));
			}
			*/
			vec_float4 vsqr, qvec = q.get128();
			vec_float4 wwww, xyzw2, yxxw, zzyw, tmp0, tmp1, vsqr0, vsqr1;
			vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});
			vec_float4 one = ((vec_float4){1.0f,1.0f,1.0f,1.0f});
			vec_float4 minusOne = ((vec_float4){-1.0f,-1.0f,-1.0f,-1.0f});

			wwww = vec_splat(qvec, 3);
			vsqr = vec_madd(qvec, qvec, zero);
			xyzw2 = vec_madd(qvec, wwww, zero);
			yxxw = vec_perm(qvec, qvec, _VECTORMATH_PERM_YXXW);
			zzyw = vec_perm(qvec, qvec, _VECTORMATH_PERM_ZZYW);
			tmp0 = vec_madd(vec_sld(qvec, qvec, 4), vec_sld(qvec, qvec, 4), vsqr);
			vsqr0 = vec_nmsub(vec_sld(qvec, qvec, 4), vec_sld(qvec, qvec, 4), (vec_float4)vec_xor((vec_uint4)vsqr, ((vec_uint4){0x80000000,0,0,0})));
			vsqr0 = vec_add(vec_sld(tmp0, tmp0, 8), vsqr0);
			vsqr1 = vec_nmsub(vec_sld(qvec, qvec, 4), vec_sld(qvec, qvec, 4), vsqr);
			vsqr1 = vec_nmsub(vec_sld(qvec, qvec, 8), vec_sld(qvec, qvec, 8), vsqr1);
			vsqr1 = vec_add(vec_sld(vsqr, vsqr, 12), vsqr1);
			tmp0 = vec_madd(yxxw, zzyw, (vec_float4)vec_xor((vec_uint4)xyzw2, ((vec_uint4){0,0x80000000,0,0})));
			tmp0 = vec_madd(tmp0, ((vec_float4){2.0f,-2.0f,2.0f,0.0f}), zero);
			tmp1 = vec_min(vec_max(tmp0, minusOne), one);
			tmp1 = asinf4(tmp1);
			tmp0 = atan2f4(tmp0, vec_perm(vsqr0, vsqr1, _VECTORMATH_PERM_XYAB));

			euler = vector3df(vec_sel(vec_perm(tmp0, tmp1, _VECTORMATH_PERM_XBZW), zero, _VECTORMATH_MASK_0x000F));
		}
	}
}


#endif /* QUATERNION_H_ */
