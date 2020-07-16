/*
 * matrix4.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef MATRIX4_H_
#define MATRIX4_H_

#include "irrtypes.h"
#include "vector4d.h"
#include "vector3d.h"
#include "vector2d.h"
#include "plane3d.h"
#include "aabbox3d.h"
#include "rect.h"

#define _VECTORMATH_PERM_AYZW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_A, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_W })

namespace irr
{
	namespace core
	{
		typedef Matrix4 matrix4;

		extern const matrix4 identityMatrix;

		inline matrix4 buildLookAtMatrix(const vector3df& pos, const vector3df& target, const vector3df& upVector)
		{
			return matrix4::lookAt(Point3(pos), Point3(target), upVector);
		}

		inline matrix4 buildPerspectiveProjectionMatrix(f32 fovy,f32 aspect,f32 znear,f32 zfar)
		{
			return matrix4::perspective(fovy, aspect, znear, zfar);
		}

		inline void transformVect(const matrix4& mat, vector3df& vect)
		{
			vec_float4 res;
			vec_float4 xxxx,yyyy,zzzz;
			vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});

			xxxx = vec_splat(vect.get128(),0);
			yyyy = vec_splat(vect.get128(),1);
			zzzz = vec_splat(vect.get128(),2);
			res = vec_madd(mat.getCol(0).get128(),xxxx,zero);
			res = vec_madd(mat.getCol(1).get128(),yyyy,res);
			res = vec_madd(mat.getCol(2).get128(),zzzz,res);
			vect = vector3df(vec_add(res,vec_sel(mat.getCol(3).get128(),zero,_VECTORMATH_MASK_0x000F)));
		}

		inline void transformVect(const matrix4& mat, const vector3df& in, vector3df& out)
		{
			vec_float4 res;
			vec_float4 xxxx,yyyy,zzzz;
			vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});

			xxxx = vec_splat(in.get128(),0);
			yyyy = vec_splat(in.get128(),1);
			zzzz = vec_splat(in.get128(),2);
			res = vec_madd(mat.getCol(0).get128(),xxxx,zero);
			res = vec_madd(mat.getCol(1).get128(),yyyy,res);
			res = vec_madd(mat.getCol(2).get128(),zzzz,res);
			out = vector3df(vec_add(res,vec_sel(mat.getCol(3).get128(),zero,_VECTORMATH_MASK_0x000F)));
		}

		inline void rotateVect(const matrix4& mat, vector3df& vect)
		{
			vec_float4 res;
			vec_float4 xxxx,yyyy,zzzz;
			vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});

			xxxx = vec_splat(vect.get128(),0);
			yyyy = vec_splat(vect.get128(),1);
			zzzz = vec_splat(vect.get128(),2);
			res = vec_madd(mat.getCol(0).get128(),xxxx,zero);
			res = vec_madd(mat.getCol(1).get128(),yyyy,res);
			vect = vector3df(vec_madd(mat.getCol(2).get128(),zzzz,res));
		}

		inline void rotateVect(const matrix4& mat, const vector3df& in, vector3df& out)
		{
			vec_float4 res;
			vec_float4 xxxx,yyyy,zzzz;
			vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});

			xxxx = vec_splat(in.get128(),0);
			yyyy = vec_splat(in.get128(),1);
			zzzz = vec_splat(in.get128(),2);
			res = vec_madd(mat.getCol(0).get128(),xxxx,zero);
			res = vec_madd(mat.getCol(1).get128(),yyyy,res);
			out = vector3df(vec_madd(mat.getCol(2).get128(),zzzz,res));
		}

		inline void transformPlane(const matrix4& mat, plane3df& plane)
		{
			vector3df member;

			transformVect(mat, plane.getMemberPoint(), member);

			matrix4 ti = inverse(transpose(mat));
			vector3df normal = plane.normal;

			transformVect(ti, normal);

			plane.setPlane(member, normal);
		}

		inline void transformBox(const matrix4& mat, aabbox3df& box)
		{
			transformVect(mat, box.minEdge);
			transformVect(mat, box.maxEdge);
			box.repair();
		}

		inline void transformBoxEx(const matrix4& mat, aabbox3df& box)
		{
			vec_float4 sel, tmp;
			vec_float4 BMin = mat.getCol(3).get128();
			vec_float4 BMax = mat.getCol(3).get128();
			const vec_float4 AMin = box.minEdge.get128();
			const vec_float4 AMax = box.maxEdge.get128();
			const vec_float4 zero = ((vec_float4){0.0f,0.0f,0.0f,0.0f});
			const vec_uchar16 permmask[3] = {
				_VECTORMATH_PERM_AYZW,
				_VECTORMATH_PERM_XAZW,
				_VECTORMATH_PERM_XYAW
			};

			for(u32 i=0;i < 3;i++) {
				vec_float4 a = vec_madd(mat.getCol(i).get128(), AMin, zero);
				vec_float4 b = vec_madd(mat.getCol(i).get128(), AMax, zero);
				vec_uint4 selmask = (vec_uint4)vec_cmplt(a, b);

				sel = vec_sel(b, a, selmask);
				tmp = vec_add(sel, vec_sld(sel, zero, 4));
				tmp = vec_add(tmp, vec_sld(sel, zero, 8));
				BMin = vec_add(BMin, vec_perm(zero, tmp, permmask[i]));

				sel = vec_sel(a, b, selmask);
				tmp = vec_add(sel, vec_sld(sel, zero, 4));
				tmp = vec_add(tmp, vec_sld(sel, zero, 8));
				BMax = vec_add(BMax, vec_perm(zero, tmp, permmask[i]));
			}

			box.minEdge = vector3df(BMin);
			box.maxEdge = vector3df(BMax);
		}
	}
}

#endif /* MATRIX4_H_ */
