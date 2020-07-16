/*
 * vector3d.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef VECTOR3D_H_
#define VECTOR3D_H_

#include "irrmath.h"

namespace irr
{
	namespace core
	{
		typedef Vector3 vector3df;

		inline f32 distanceFrom(const vector3df& a, const vector3df& b)
		{
			return length(a - b);
		}

		inline f32 distanceFromSQ(const vector3df& a, const vector3df& b)
		{
			return lengthSqr(a - b);
		}

		inline void horizontalAngle(const vector3df& in, vector3df& out)
		{
			vec_float4 xxxx,yyyy,zzzz;
			vec_float4 tmpX, tmpY, tmp, zl;
			vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});
			vec_float4 nninty = ((vec_float4){-90.0f,-90.0f,-90.0f,-90.0f});
			vec_float4 threesixty = ((vec_float4){360.0f,360.0f,360.0f,360.0f});
			vec_float4 rad2deg = ((vec_float4){57.2957795130785f, 57.2957795130785f, 57.2957795130785f, 57.2957795130785f});

			xxxx = vec_splat(in.get128(), 0);
			yyyy = vec_splat(in.get128(), 1);
			zzzz = vec_splat(in.get128(), 2);
			tmpY = vec_madd(atan2f4(xxxx, zzzz), rad2deg, zero);

			zl = vec_madd(xxxx, xxxx, zero);
			zl = sqrtf4(vec_madd(zzzz, zzzz, zl));
			tmpX = vec_madd(atan2f4(zl, yyyy), rad2deg, nninty);

			tmp = vec_mergeh(tmpX, tmpY);

			vec_uint4 lmask = (vec_uint4)vec_cmplt(tmp, zero);
			vec_uint4 hmask = (vec_uint4)vec_cmpge(tmp, threesixty);

			tmp = vec_add(tmp, vec_sel(zero, threesixty, lmask));
			tmp = vec_add(tmp, vec_sel(zero, negatef4(threesixty), hmask));

			out = vector3df(vec_perm(tmp, zero, _VECTORMATH_PERM_XYAB));
		}

		inline void rotateXZBy(f32 degrees, const vector3df& in, vector3df& out, const vector3df& center = vector3df(0, 0, 0))
		{
			vec_float4 deg, res;
			vec_float4 sn, cs;
			vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});

			degrees *= DEGTORAD;

			deg = vec_ld(0, &degrees);

			sincosf4(vec_splat(vec_perm(deg, deg, vec_lvsl(0, &degrees)), 0), &sn, &cs);
			res = vec_sub(in.get128(), vec_sel(center.get128(), zero, ((vec_uint4){ 0, 0xffffffff, 0, 0xffffffff })));

			out = vector3df(vec_add(res, vec_sel(center.get128(), zero, ((vec_uint4){ 0, 0xffffffff, 0, 0xffffffff }))));
		}

		inline void interpolate(const vector3df& in, const vector3df& other, vector3df& out, f32 d)
		{
			vec_float4 D;
			vec_float4 res;
			vec_float4 one = ((vec_float4){1.0f,1.0f,1.0f,1.0f});
			vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});

			D = vec_ld(0, &d);
			D = vec_splat(vec_perm(D, D, vec_lvsl(0, &d)), 0);
			res = vec_madd(in.get128(), D, zero);
			res = vec_madd(other.get128(), vec_sub(one, D), res);

			out = vector3df(res);
		}
	}
}

#endif /* VECTOR3D_H_ */
