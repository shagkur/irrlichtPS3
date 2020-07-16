/*
 * irrmath.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef IRRMATH_H_
#define IRRMATH_H_

#include "irrtypes.h"
#include <stddef.h>
#include <altivec.h>
#include <vec_types.h>
#include <simdmath/simdmath.h>

namespace irr
{
	namespace core
	{
		typedef union { u32 u; s32 s; f32 f; } ieee32;
		typedef union { vec_float4 v; f32 f[4]; } veee32;

		const f32 ROUNDING_ERROR_f32 = 0.000001f;

		const f32 PI = 3.14159265359f;
		const f32 RECIPROCAL_PI	= 1.0f/PI;

		const f32 DEGTORAD = PI/180.0f;
		const f32 RADTODEG = 180.0f/PI;

		inline f32 radToDeg(f32 radians)
		{
			return RADTODEG*radians;
		}

		inline f32 degToRad(f32 degrees)
		{
			return DEGTORAD*degrees;
		}

		template< class T >
		inline const T& min_(const T& a,const T& b)
		{
			return a<b?a:b;
		}

		template< class T >
		inline const T& max_(const T& a,const T& b)
		{
			return a<b?b:a;
		}

		template< class T >
		inline T lerp(const T& a, const T& b, const f32 t)
		{
			return (T)(a*(1.0f - t)) + (b*t);
		}

		template< class T >
		inline const T clamp(const T& val,const T& low,const T& high)
		{
			return min_(max_(val,low),high);
		}

		inline bool equals(const f32 a, const f32 b, const f32 tolerance = ROUNDING_ERROR_f32)
		{
			return (a + tolerance >= b) && (a - tolerance <= b);
		}

		inline s32 s32_min(s32 a, s32 b)
		{
			const s32 mask = (a - b)>>31;
			return (a&mask) | (b&~mask);
		}

		inline s32 s32_max(s32 a, s32 b)
		{
			const s32 mask = (a - b)>>31;
			return (b&mask) | (a&~mask);
		}

		inline s32 s32_clamp(s32 value, s32 low, s32 high)
		{
			return s32_min(s32_max(value, low), high);
		}

		inline f32 reciprocal_sqrtf32(const f32 val)
		{
#ifndef __PPU__
			return 1.0f/sqrtf(val);
#else
			register f32 half = 0.5f;
			register f32 three = 3.0f;
			register f32 tmp0,tmp1,rsqrt;
			__asm__ __volatile__ (
				"frsqrte	%0,%3\n"
				//Newton-Raphson refinement 1 step: (E/2)*(3 - x*E*E)
				"fmuls		%1,%0,%0\n"			// E*E
				"fmuls		%2,%0,%4\n"			// E*0.5 = E/2
				"fnmsubs	%1,%1,%3,%5\n"		// -(E*E*x - 3) = (3 - x*E*E)
				"fmuls		%0,%1,%2\n"			// (E/2)*(3 - x*E*E)
				: "=&f"(rsqrt),"=&f"(tmp0),"=&f"(tmp1)
				: "f"(val),"f"(half),"f"(three)
			);
			return rsqrt;
#endif
		}

		inline f32 reciprocalf32(const f32 val)
		{
#ifndef __PPU__
			return 1.0f/val;
#else
			register f32 d = 0.0f;
			__asm__ __volatile__ (
				"fres		%0,%1"
				: "=f"(d)
				: "f"(val)
			);
			return d;
#endif
		}

		inline f32 sqrtf32(f32 x)
		{
#ifndef __PPU__
			return sqrtf(x);
#else
			register f32 half = 0.5f;
			register f32 three = 3.0f;
			register f32 rmag,tmp0,tmp1,mag = 0.0f;
			__asm__ __volatile__ (
				"frsqrte	%0,%4\n"			// 1.0f/sqrf : Estimate [E]
				//Newton-Raphson refinement 1 step: (E/2)*(3 - x*E*E)
				"fmuls		%1,%0,%0\n"			// E*E
				"fmuls		%2,%0,%5\n"			// E*0.5 = E/2
				"fnmsubs	%1,%1,%4,%6\n"		// -(E*E*x - 3) = (3 - x*E*E)
				"fmuls		%0,%1,%2\n"			// (E/2)*(3 - x*E*E)
				"fsel		%0,%0,%0,%4\n"		// NaN check (if x==0.0f)
				"fmuls		%3,%4,%0"			// 1.0f/sqrt(x)*x = sqrt(x)
				: "=&f"(rmag),"=&f"(tmp0),"=&f"(tmp1),"=f"(mag)
				: "f"(x),"f"(half),"f"(three)
			);
			return mag;
#endif
		}

		inline f32 fract(f32 x)
		{
			return x - floorf(x);
		}

		inline u32 IR(f32 x)
		{
			ieee32 tmp;
			tmp.f = x;
			return tmp.u;
		}

		inline f32 FR(u32 x)
		{
			ieee32 tmp;
			tmp.u = x;
			return tmp.f;
		}

		inline f32 FR(s32 x)
		{
			ieee32 tmp;
			tmp.s = x;
			return tmp.f;
		}

		inline bool ispoweroftwo(u32 x)
		{
			return (!(x&(x - 1)) && x);
		}

	}
}

#endif /* IRRMATH_H_ */
