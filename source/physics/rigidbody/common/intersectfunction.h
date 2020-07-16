/*
 * intersectfunction.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef INTERSECTFUNCTION_H_
#define INTERSECTFUNCTION_H_

#include "base/common.h"

#define RAY_TRIANGLE_EPSILON 		0.00001f

inline bool rayIntersectAABBFast(const Vector3& AABBhalf, const Vector3& AABBpos, const Vector3& rayStart, const Vector3& rayDir, f32& t)
{
	Vector3 AABBmin = AABBpos - AABBhalf;
	Vector3 AABBmax = AABBpos + AABBhalf;

	Vector3 dir = rayDir;
	Vector3 absDir = absPerElem(dir);
	Vector3 sign = copySignPerElem(Vector3(1), dir);

#ifdef __SPU__
	vec_float4 eps = spu_splats(RAY_TRIANGLE_EPSILON);
	vec_uint4 chk1 = spu_cmpgt(eps, absDir.get128());
	vec_uint4 chk2 = spu_cmpgt(AABBmin.get128(), rayStart.get128());
	vec_uint4 chk3 = spu_cmpgt(rayStart.get128(), AABBmax.get128());
	vec_uint4 chk4 = spu_and(chk1, spu_or(chk2, chk3));

	if(spu_extract(spu_gather(spu_rlmaskqwbyte(chk4, -4)), 0) > 0)
		return false;

	dir = Vector3(spu_sel(dir.get128(), spu_mul(sign.get128(), eps), chk1));
#else
	if(absDir[0] < RAY_TRIANGLE_EPSILON) {
		if(rayStart[0] < AABBmin[0] || rayStart[0] > AABBmax[0])
			return false;

		dir[0] = sign[0]*RAY_TRIANGLE_EPSILON;
	}

	if(absDir[1] < RAY_TRIANGLE_EPSILON) {
		if(rayStart[1] < AABBmin[1] || rayStart[1] > AABBmax[1])
			return false;

		dir[1] = sign[1]*RAY_TRIANGLE_EPSILON;
	}

	if(absDir[2] < RAY_TRIANGLE_EPSILON) {
		if(rayStart[2] < AABBmin[2] || rayStart[2] > AABBmax[2])
			return false;

		dir[2] = sign[2]*RAY_TRIANGLE_EPSILON;
	}
#endif

	Vector3 t1 = divPerElem(AABBmin - rayStart, dir);
	Vector3 t2 = divPerElem(AABBmax - rayStart, dir);

	Vector3 tmin = minPerElem(t1, t2);
	Vector3 tmax = maxPerElem(t1, t2);

#ifdef __SPU__
	const vec_uint4 mask001 = ((vec_uint4){0x80808080, 0x00010203, 0x00010203, 0x04050607});
	const vec_uint4 mask122 = ((vec_uint4){0x80808080, 0x04050607, 0x08090a0b, 0x08090a0b});
	const vec_ushort8 mintable = ((vec_ushort8){0, 0, 2, 2, 1, 2, 1, 2});
	const vec_ushort8 maxtable = ((vec_ushort8){2, 1, 2, 1, 2, 2, 0, 0});
	const vec_float4 vtmin = tmin.get128();
	const vec_float4 vtmax = tmax.get128();

	vec_uint4 cmpmin = spu_cmpgt(
								 spu_shuffle(vtmin, vtmin, (vec_uchar16)mask001),
								 spu_shuffle(vtmin, vtmin, (vec_uchar16)mask122)
								);

	vec_uint4 cmpmax = spu_cmpgt(
								 spu_shuffle(vtmax, vtmax, (vec_uchar16)mask001),
								 spu_shuffle(vtmax, vtmax, (vec_uchar16)mask122)
								);

	u32 minId = spu_extract(maxtable, spu_extract(spu_gather(cmpmin), 0));
	u32 maxId = spu_extract(mintable, spu_extract(spu_gather(cmpmax), 0));

	if(tmin[minId] > tmax[maxId]) return false;

	t = tmin[minId];
#else
	if(maxElem(tmin) > minElem(tmax)) return false;

	if(tmin[0] > tmin[1]) {
		if(tmin[0] > tmin[2])
			t = tmin[0];
		else
			t = tmin[2];
	} else {
		if(tmin[1] > tmin[2])
			t = tmin[1];
		else
			t = tmin[2];
	}
#endif
	return true;
}

inline bool rayIntersectAABB(const Vector3& AABBhalf, const Vector3& AABBpos, const Vector3& rayStart, const Vector3& rayDir, f32& t, Vector3& nml)
{
	Vector3 AABBmin = AABBpos - AABBhalf;
	Vector3 AABBmax = AABBpos + AABBhalf;

	Vector3 dir = rayDir;
	Vector3 absDir = absPerElem(dir);
	Vector3 sign = copySignPerElem(Vector3(1), dir);

	// 始点がBoxの内側にあるか判定
	if(AABBmin[0] < rayStart[0] && rayStart[0] < AABBmax[0] &&
	   AABBmin[1] < rayStart[1] && rayStart[1] < AABBmax[1] &&
	   AABBmin[2] < rayStart[2] && rayStart[2] < AABBmax[2])
	{
		return false;
	}

#ifdef __SPU__
	vec_float4 eps = spu_splats(RAY_TRIANGLE_EPSILON);
	vec_uint4 chk1 = spu_cmpgt(eps, absDir.get128());
	vec_uint4 chk2 = spu_cmpgt(AABBmin.get128(), rayStart.get128());
	vec_uint4 chk3 = spu_cmpgt(rayStart.get128(), AABBmax.get128());
	vec_uint4 chk4 = spu_and(chk1, spu_or(chk2, chk3));

	if(spu_extract(spu_gather(spu_rlmaskqwbyte(chk4, -4)), 0) > 0)
		return false;

	dir = Vector3(spu_sel(dir.get128(), spu_mul(sign.get128(), eps), chk1));
#else
	if(absDir[0] < RAY_TRIANGLE_EPSILON) {
		if(rayStart[0] < AABBmin[0] || rayStart[0] > AABBmax[0])
			return false;

		dir[0] = sign[0]*RAY_TRIANGLE_EPSILON;
	}

	if(absDir[1] < RAY_TRIANGLE_EPSILON) {
		if(rayStart[1] < AABBmin[1] || rayStart[1] > AABBmax[1])
			return false;

		dir[1] = sign[1]*RAY_TRIANGLE_EPSILON;
	}

	if(absDir[2] < RAY_TRIANGLE_EPSILON) {
		if(rayStart[2] < AABBmin[2] || rayStart[2] > AABBmax[2])
			return false;

		dir[2] = sign[2]*RAY_TRIANGLE_EPSILON;
	}
#endif

	Vector3 t1 = divPerElem(AABBmin - rayStart, dir);
	Vector3 t2 = divPerElem(AABBmax - rayStart, dir);

	Vector3 tmin = minPerElem(t1, t2);
	Vector3 tmax = maxPerElem(t1, t2);

#ifdef __SPU__
	const vec_uint4 mask001 = ((vec_uint4){0x80808080, 0x00010203, 0x00010203, 0x04050607});
	const vec_uint4 mask122 = ((vec_uint4){0x80808080, 0x04050607, 0x08090a0b, 0x08090a0b});
	const vec_ushort8 mintable = ((vec_ushort8){0, 0, 2, 2, 1, 2, 1, 2});
	const vec_ushort8 maxtable = ((vec_ushort8){2, 1, 2, 1, 2, 2, 0, 0});
	const vec_float4 vtmin = tmin.get128();
	const vec_float4 vtmax = tmax.get128();

	vec_uint4 cmpmin = spu_cmpgt(
								 spu_shuffle(vtmin, vtmin, (vec_uchar16)mask001),
								 spu_shuffle(vtmin, vtmin, (vec_uchar16)mask122)
								);

	vec_uint4 cmpmax = spu_cmpgt(
								 spu_shuffle(vtmax, vtmax, (vec_uchar16)mask001),
								 spu_shuffle(vtmax, vtmax, (vec_uchar16)mask122)
								);

	u32 minId = spu_extract(maxtable, spu_extract(spu_gather(cmpmin), 0));
	u32 maxId = spu_extract(mintable, spu_extract(spu_gather(cmpmax), 0));

	if(tmin[minId] > tmax[maxId]) return false;

	t = tmin[minId];
	nml = Vector3(0);
	nml[minId] = -sign[minId];
#else
	if(maxElem(tmin) > minElem(tmax)) return false;

	nml = Vector3(0);

	if(tmin[0] > tmin[1]) {
		if(tmin[0] > tmin[2]) {
			t = tmin[0];
			nml[0] = -sign[0];
		} else {
			t = tmin[2];
			nml[2] = -sign[2];
		}
	} else {
		if(tmin[1] > tmin[2]) {
			t = tmin[1];
			nml[1] = -sign[1];
		} else {
			t = tmin[2];
			nml[2] = -sign[2];
		}
	}
#endif

	return true;
}

inline bool rayIntersectTriangle(const Vector3 *pnts, const Vector3& rayStart, const Vector3& rayDir, f32& t)
{
	f32 v, w;
	Vector3 ab = pnts[1] - pnts[0];
	Vector3 ac = pnts[2] - pnts[0];

	Vector3 n = cross(ab, ac);

	f32 d = dot(-rayDir, n);

	if(fabsf(d) < 0.00001f) return false;

	Vector3 ap = rayStart - pnts[0];
	t = dot(ap, n)/d;

	if(t <= 0.0f || t >= 1.0f) return false;

	Vector3 e = cross(-rayDir, ap);
	v = dot(ac, e)/d;
	if(v < -RAY_TRIANGLE_EPSILON || v > 1.0f + RAY_TRIANGLE_EPSILON) return false;

	w = -dot(ab, e)/d;
	if(w < -RAY_TRIANGLE_EPSILON || v + w > 1.0f + RAY_TRIANGLE_EPSILON) return false;

	return true;
}

inline bool rayIntersectTriangleWithoutFrontFace(const Vector3 *pnts, const Vector3& rayStart, const Vector3& rayDir, f32& t)
{
	f32 v, w;
	Vector3 ab = pnts[1] - pnts[0];
	Vector3 ac = pnts[2] - pnts[0];

	Vector3 n = cross(ab, ac);

	f32 d = dot(-rayDir, n);

	if(d >= 0.0f) return false;

	Vector3 ap = rayStart - pnts[0];
	t = dot(ap, n)/d;

	if(t <= 0.0f || t >= 1.0f) return false;

	Vector3 e = cross(-rayDir, ap);
	v = dot(ac, e)/d;
	if(v < -RAY_TRIANGLE_EPSILON || v > 1.0f + RAY_TRIANGLE_EPSILON) return false;

	w = -dot(ab, e)/d;
	if(w < -RAY_TRIANGLE_EPSILON || v + w > 1.0 + RAY_TRIANGLE_EPSILON) return false;

	return true;
}

inline bool rayIntersectTriangleWithoutBackFace(const Vector3 *pnts, const Vector3& rayStart, const Vector3& rayDir, f32& t)
{
	f32 v, w;
	Vector3 ab = pnts[1] - pnts[0];
	Vector3 ac = pnts[2] - pnts[0];

	Vector3 n = cross(ab, ac);

	f32 d = dot(-rayDir, n);

	if(d <= 0.0f) return false;

	Vector3 ap = rayStart - pnts[0];
	t = dot(ap, n)/d;

	if(t <= 0.0f || t >= 1.0f) return false;

	Vector3 e = cross(-rayDir, ap);
	v = dot(ac, e)/d;
	if(v < -RAY_TRIANGLE_EPSILON || v > 1.0f + RAY_TRIANGLE_EPSILON) return false;

	w = -dot(ab, e)/d;
	if(w < -RAY_TRIANGLE_EPSILON || v + w > 1.0f + RAY_TRIANGLE_EPSILON) return false;

	return true;
}

inline bool fastCheckIntersectTwoTriangles(const Vector3 *pA, const Vector3& normalA, const Vector3 *pB, const Vector3& normalB)
{
	Vector3 axis;
	f32 areaMinA, areaMaxA, areaMinB, areaMaxB;

	// A -> B
	{
		axis = normalA;
		areaMinA = areaMaxA = dot(axis, pA[0]);

		areaMinB = FLT_MAX;
		areaMaxB = FLT_MIN;
		f32 prj;
		prj = dot(axis, pB[0]);
		areaMinB = areaMinB > prj ? prj : areaMinB;
		areaMaxB = areaMaxB < prj ? prj : areaMaxB;
		prj = dot(axis, pB[1]);
		areaMinB = areaMinB > prj ? prj : areaMinB;
		areaMaxB = areaMaxB < prj ? prj : areaMaxB;
		prj = dot(axis, pB[2]);
		areaMinB = areaMinB > prj ? prj : areaMinB;
		areaMaxB = areaMaxB < prj ? prj : areaMaxB;

		if(areaMaxB <= areaMinA || areaMinB >= areaMaxA)
			return false;
	}

	// B -> A
	{
		axis = normalB;
		areaMinB = areaMaxB = dot(axis, pB[0]);

		areaMinA = FLT_MAX;
		areaMaxA = FLT_MIN;
		f32 prj;
		prj = dot(axis, pA[0]);
		areaMinA = areaMinA > prj ? prj : areaMinA;
		areaMaxA = areaMaxA < prj ? prj : areaMaxA;
		prj = dot(axis, pA[1]);
		areaMinA = areaMinA > prj ? prj : areaMinA;
		areaMaxA = areaMaxA < prj ? prj : areaMaxA;
		prj = dot(axis, pA[2]);
		areaMinA = areaMinA > prj ? prj : areaMinA;
		areaMaxA = areaMaxA < prj ? prj : areaMaxA;

		if(areaMaxB <= areaMinA || areaMinB >= areaMaxA)
			return false;
	}

	for(s32 ia=0;ia < 3;ia++) {
		for(s32 ib=0;ib < 3;ib++) {
			Vector3 eA = normalize(pA[(ia + 1)%3] - pA[ia]);
			Vector3 eB = normalize(pB[(ib + 1)%3] - pB[ib]);

			if(1.0f - fabsf(dot(eA, eB)) < 0.00001f) continue;

			axis = normalize(cross(eA, eB));

			f32 prj;

			areaMinA = FLT_MAX;
			areaMaxA = FLT_MIN;

			prj = dot(axis, pA[0]);
			areaMinA = areaMinA > prj ? prj : areaMinA;
			areaMaxA = areaMaxA < prj ? prj : areaMaxA;
			prj = dot(axis, pA[1]);
			areaMinA = areaMinA > prj ? prj : areaMinA;
			areaMaxA = areaMaxA < prj ? prj : areaMaxA;
			prj = dot(axis, pA[2]);
			areaMinA = areaMinA > prj ? prj : areaMinA;
			areaMaxA = areaMaxA < prj ? prj : areaMaxA;

			areaMinB = FLT_MAX;
			areaMaxB = FLT_MIN;

			prj = dot(axis, pB[0]);
			areaMinB = areaMinB > prj ? prj : areaMinB;
			areaMaxB = areaMaxB < prj ? prj : areaMaxB;
			prj = dot(axis, pB[1]);
			areaMinB = areaMinB > prj ? prj : areaMinB;
			areaMaxB = areaMaxB < prj ? prj : areaMaxB;
			prj = dot(axis, pB[2]);
			areaMinB = areaMinB > prj ? prj : areaMinB;
			areaMaxB = areaMaxB < prj ? prj : areaMaxB;

			if(areaMaxB <= areaMinA || areaMinB >= areaMaxA)
				return false;
		}
	}

	return true;
}

inline void distanceTwoLines(const Vector3& p1, const Vector3& q1, const Vector3& p2, const Vector3& q2, Vector3& s1, Vector3& s2)
{
	Vector3 v1 = q1 - p1;
	Vector3 v2 = q2 - p2;
	Vector3 r = p1 - p2;

	f32 a = dot(v1, v1);
	f32 e = dot(v2, v2);
	f32 f = dot(v2, r);
	f32 b = dot(v1, v2);
	f32 c = dot(v1, r);
	f32 den = a*e - b*b;

	f32 s, t;

	if(den != 0.0f)
		s = CLAMP((b*f - c*e)/den, 0.0f, 1.0f);
	else
		s = 0.0f;

	t = (b*s + f)/e;

	if(t < 0.0f) {
		t = 0.0f;
		s = CLAMP(-c/a, 0.0f, 1.0f);
	} else if(t > 1.0f) {
		t = 1.0f;
		s = CLAMP((b - c)/a, 0.0f, 1.0f);
	}

	s1 = p1 + s*v1;
	s2 = p2 + t*v2;
}

inline s32 distancePointAndTriangle(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& p, Vector3& s)
{
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = p - a;
    f32 d1 = dot(ab, ap);
    f32 d2 = dot(ac, ap);
	if(d1 <= 0.0f && d2 <= 0.0f) {
		s = a;
		return 0; // point A
	}

    Vector3 bp = p - b;
    f32 d3 = dot(ab, bp);
    f32 d4 = dot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3) {
		s = b;
		return 1; // point B
	}

    f32 vc = d1*d4 - d3*d2;
    if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        f32 v = d1/(d1 - d3);
        s = a + v*ab;
		return 2; // edge AB
    }

    Vector3 cp = p - c;
    f32 d5 = dot(ab, cp);
    f32 d6 = dot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6) {
		s = c;
		return 3; // point C
	}

    f32 vb = d5*d2 - d1*d6;
    if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        f32 w = d2/(d2 - d6);
        s = a + w*ac;
		return 4; // edge AC
    }

    f32 va = d3*d6 - d5*d4;
    if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        f32 w = (d4 - d3)/((d4 - d3) + (d5 - d6));
        s = b + w*(c - b);
		return 5; // edge BC
    }

    f32 den = 1.0f/(va + vb + vc);
    f32 v = vb*den;
    f32 w = vc*den;
    s = a + ab*v + ac*w;
	return 6; // triangle ABC
}

inline void closestPointAndAABB(const Vector3& p, const Vector3& aabb, Vector3& s)
{
	s = p;
	s = maxPerElem(s, -aabb);
	s = minPerElem(s, aabb);
}

inline void get_ST(f32& s, f32& t, const Vector3& v0, const Vector3& v1, const Vector3& dir)
{
	Vector3 v = cross(v0, v1);
	Vector3 crS = cross(v, v0);
	Vector3 crT = cross(v, v1);
	s = dot(crT, dir)/dot(crT, v0);
	t = dot(crS, dir)/dot(crS, v1);
}

inline bool pointOnLine(const Vector3& p, const Vector3& a, const Vector3& b)
{
	Vector3 ab = normalize(b - a);
	Vector3 q = a + ab*dot(p - a, ab);
	return lengthSqr(p - q) < 0.00001f;
}

inline bool pointOnSegment(const Vector3& p, const Vector3& a, const Vector3& b)
{
	Vector3 ab = b - a;
	Vector3 ap = p - a;
	f32 denom = dot(ab, ab);
	f32 num = dot(ap, ab);
	f32 t= num/denom;
	if(t < 0.0f || t > 1.0f) return false;
	return (dot(ap, ap) - num*t) < 0.00001f;
}

#endif /* INTERSECTFUNCTION_H_ */
