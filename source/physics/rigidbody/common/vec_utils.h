/*
 * vec_utils.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef VEC_UTILS_H_
#define VEC_UTILS_H_

#include "base/common.h"

static const f32 lenSqrTol = 1.0e-30f;

inline Vector3 safeNormalize(Vector3 vec)
{
	f32 lenSqr = lengthSqr(vec);

	if(lenSqr > lenSqrTol) {
		f32 lenInv = 1.0f/sqrtf(lenSqr);
		return vec*lenInv;
	} else
		return Vector3(1.0f, 0.0f, 0.0f);
}

inline f32 square( f32 a )
{
	return a*a;
}

inline f32 randfloat()
{
	return 0.5f;
}

inline Vector3 rand_perp(Vector3 vec)
{
	Vector3 result;
	Vector3 abs_vec = Vector3(absPerElem(vec));

	Vector3 cross_vec;

	if((abs_vec[0] > abs_vec[1]) && (abs_vec[0] > abs_vec[2]))
		cross_vec = Vector3(0.9f*randfloat(), 1.0f, 0.9f*randfloat());
	else
		cross_vec = Vector3(1.0f, 0.9f*randfloat(), 0.9f*randfloat());

	return cross(vec, cross_vec);
}

inline Vector3 perp(Vector3 vec)
{
	Vector3 result;
	Vector3 core_vec = Vector3(vec);
	Vector3 abs_vec = Vector3(absPerElem(vec));

	if((abs_vec[0] > abs_vec[1]) && (abs_vec[0] > abs_vec[2])) {
		result[0] = -core_vec[2];
		result[1] = 0.0f;
		result[2] = core_vec[0];
	} else {
		result[0] = 0.0f;
		result[1] = core_vec[2];
		result[2] = -core_vec[1];
	}

	return Vector3(result);
}

inline Vector3 unit_cross(Vector3 veca, Vector3 vecb)
{
	Vector3 result;

	result = cross(veca, vecb);

	f32 lenSqr = lengthSqr(result);

	if(lenSqr > lenSqrTol) {
		f32 lenInv = 1.0f/sqrtf(lenSqr);
		result *= lenInv;
	} else
		result = normalize(perp(veca));

	return result;
}

inline void angleAxis(const Quat& unitQuat, f32& angle, Vector3& axis)
{
	const f32 epsilon = 0.00001f;

	if(fabsf(unitQuat.getW()) < 1.0f - epsilon && lengthSqr(unitQuat.getXYZ()) > epsilon) {
		f32 angleHalf = acosf(unitQuat.getW());
		f32 sinAngleHalf = sinf(angleHalf);

		if(fabsf(sinAngleHalf) > 1.0e-10f)
			axis = unitQuat.getXYZ()/sinAngleHalf;
		else
			axis = unitQuat.getXYZ();

		angle = 2.0f*angleHalf;
	} else {
		angle = 0.0f;
		axis = Vector3(1.0f, 0.0f, 0.0f);
	}
}

inline void getPlaneSpace(const Vector3& n, Vector3& p, Vector3& q)
{
	if(fabsf(n[2]) > 0.707f) {
		// choose p in y-z plane
		f32 a = n[1]*n[1] + n[2]*n[2];
		f32 k = 1.0f/sqrtf(a);
		p[0] = 0;
		p[1] = -n[2]*k;
		p[2] = n[1]*k;
		// set q = n x p
		q[0] = a*k;
		q[1] = -n[0]*p[2];
		q[2] = n[0]*p[1];
	} else {
		// choose p in x-y plane
		f32 a = n[0]*n[0] + n[1]*n[1];
		f32 k = 1.0f/sqrtf(a);
		p[0] = -n[1]*k;
		p[1] = n[0]*k;
		p[2] = 0;
		// set q = n x p
		q[0] = -n[2]*p[1];
		q[1] = n[2]*p[0];
		q[2] = a*k;
	}
}

#endif /* VEC_UTILS_H_ */
