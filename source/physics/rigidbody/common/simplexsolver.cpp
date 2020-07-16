/*
 * simplexsolver.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/simplexsolver.h"
#include "rigidbody/common/gjksolver.h"

inline static bool operator==(const Vector3& a, const Vector3& b)
{
	return lengthSqr(a - b) < (GJK_EPSILON*GJK_EPSILON);
}

bool SimplexSolver::closest(Vector3& v)
{
	bool ret = false;

	bc.reset();

	switch(numVertices) {
		case 0:
		{
			ret = false;
		}
		break;

		case 1:
		{
			Vector3 tmpP = P[0];
			Vector3 tmpQ = Q[0];
			v = tmpP - tmpQ;
			bc.reset();
			bc.setBarycentricCoordinates(1.0f, 0.0f, 0.0f, 0.0f);
			ret = bc.isValid();
		}
		break;

		case 2:
		{
			Vector3 dir = W[1] - W[0];
			f32 t = dot(-W[0], dir)/dot(dir, dir);

			if(t < 0.0f) t = 0.0f;
			if(t > 1.0f) t = 1.0f;

			bc.setBarycentricCoordinates(1 - t, t, 0.0f, 0.0f);

			Vector3 tmpP = P[0] + t*(P[1] - P[0]);
			Vector3 tmpQ = Q[0] + t*(Q[1] - Q[0]);
			v = tmpP - tmpQ;

			reduceVertices();

			ret = bc.isValid();
			break;
		}

		case 3:
		{
			const Vector3& a = W[0];
			const Vector3& b = W[1];
			const Vector3& c = W[2];

			closestPointTriangleFromOrigin(a, b, c, bc);

#ifdef __SPU__
			Vector3 tmpP(spu_add(
								spu_add(
									spu_mul(P[0].get128(), spu_splats(spu_extract(bc.barycentricCoords, 0))),
									spu_mul(P[1].get128(), spu_splats(spu_extract(bc.barycentricCoords, 1)))
								),
								spu_mul(P[2].get128(), spu_splats(spu_extract(bc.barycentricCoords, 2)))
							));

			Vector3 tmpQ(spu_add(
								spu_add(
									spu_mul(Q[0].get128(), spu_splats(spu_extract(bc.barycentricCoords, 0))),
									spu_mul(Q[1].get128(), spu_splats(spu_extract(bc.barycentricCoords, 1)))
								),
								spu_mul(Q[2].get128(), spu_splats(spu_extract(bc.barycentricCoords, 2)))
							));
#else
			Vector3 tmpP = P[0]*bc.barycentricCoords[0] +
						   P[1]*bc.barycentricCoords[1] +
						   P[2]*bc.barycentricCoords[2];

			Vector3 tmpQ = Q[0]*bc.barycentricCoords[0] +
						   Q[1]*bc.barycentricCoords[1] +
						   Q[2]*bc.barycentricCoords[2];
#endif

			v = tmpP - tmpQ;

			reduceVertices();
			ret = bc.isValid();
			break;
		}

		case 4:
		{
			const Vector3& a = W[0];
			const Vector3& b = W[1];
			const Vector3& c = W[2];
			const Vector3& d = W[3];

			if(closestPointTetrahedronFromOrigin(a, b, c, d, bc)) {
#ifdef __SPU__
				Vector3 tmpP(spu_add(
									spu_add(
										spu_add(
											spu_mul(P[0].get128(), spu_splats(spu_extract(bc.barycentricCoords, 0))),
											spu_mul(P[1].get128(), spu_splats(spu_extract(bc.barycentricCoords, 1)))
										),
										spu_mul(P[2].get128(), spu_splats(spu_extract(bc.barycentricCoords, 2)))
									),
									spu_mul(P[3].get128(), spu_splats(spu_extract(bc.barycentricCoords, 3)))
								));

				Vector3 tmpQ(spu_add(
									spu_add(
										spu_add(
											spu_mul(Q[0].get128(), spu_splats(spu_extract(bc.barycentricCoords, 0))),
											spu_mul(Q[1].get128(), spu_splats(spu_extract(bc.barycentricCoords, 1)))
										),
										spu_mul(Q[2].get128(), spu_splats(spu_extract(bc.barycentricCoords, 2)))
									),
									spu_mul(Q[3].get128(), spu_splats(spu_extract(bc.barycentricCoords, 3)))
								));
#else
				Vector3 tmpP = P[0]*bc.barycentricCoords[0] +
							   P[1]*bc.barycentricCoords[1] +
							   P[2]*bc.barycentricCoords[2] +
							   P[3]*bc.barycentricCoords[3];

				Vector3 tmpQ = Q[0]*bc.barycentricCoords[0] +
							   Q[1]*bc.barycentricCoords[1] +
							   Q[2]*bc.barycentricCoords[2] +
							   Q[3]*bc.barycentricCoords[3];
#endif
				v = tmpP - tmpQ;

				reduceVertices();
				ret = bc.isValid();
			} else {
				ret = true;
				v = Vector3(0.0f);
			}
			break;
		}
	};

	return ret;
}

bool SimplexSolver::inSimplex(const Vector3& w)
{
	for(s32 i=0;i < numVertices;i++) {
		if(W[i] == w)
			return true;
	}
	return false;
}

bool SimplexSolver::closestPointTriangleFromOrigin(Vector3 a, Vector3 b, Vector3 c, BarycentricCoords& result)
{
	result.usedVertices = 0;
	Vector3 p(0.0f);

    Vector3 ab = b - a;
    Vector3 ac = c - a;
    Vector3 ap = p - a;
    f32 d1 = dot(ab, ap);
    f32 d2 = dot(ac, ap);
    if(d1 <= 0.0f && d2 <= 0.0f) {
		result.closest = a;
		result.setBarycentricCoordinates(1.0f, 0.0f, 0.0f, 0.0f);
		return true;
	}

    Vector3 bp = p - b;
    f32 d3 = dot(ab, bp);
    f32 d4 = dot(ac, bp);
    if(d3 >= 0.0f && d4 <= d3) {
		result.closest = b;
		result.setBarycentricCoordinates(0.0f, 1.0f, 0.0f, 0.0f);
		return true;
	}

    f32 vc = d1*d4 - d3*d2;
    if(vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
        f32 v = d1/(d1 - d3);
		result.closest = a + v*ab;
		result.setBarycentricCoordinates(1.0f - v, v, 0.0f, 0.0f);
		return true;
    }

    Vector3 cp = p - c;
    f32 d5 = dot(ab, cp);
    f32 d6 = dot(ac, cp);
    if(d6 >= 0.0f && d5 <= d6) {
		result.closest = c;
		result.setBarycentricCoordinates(0.0f, 0.0f, 1.0f, 0.0f);
		return true;
	}

    f32 vb = d5*d2 - d1*d6;
    if(vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
        f32 w = d2/(d2 - d6);
		result.closest = a + w*ac;
		result.setBarycentricCoordinates(1.0f - w, 0.0f, w, 0.0f);
		return true;
    }

    f32 va = d3*d6 - d5*d4;
    if(va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
        f32 w = (d4 - d3)/((d4 - d3) + (d5 - d6));
		result.closest = b + w*(c - b);
		result.setBarycentricCoordinates(0.0f, 1.0f - w, w, 0.0f);
		return true;
    }

    f32 denom = f32(1.0)/(va + vb + vc);
    f32 v = vb*denom;
    f32 w = vc*denom;

	result.closest = a + ab*v + ac*w;
	result.setBarycentricCoordinates(1.0f - v - w, v, w, 0.0f);

	return true;
}

bool SimplexSolver::closestPointTetrahedronFromOrigin(Vector3 a, Vector3 b, Vector3 c, Vector3 d, BarycentricCoords& finalResult)
{
	BarycentricCoords tempResult;
	Vector3 p(0.0f);

	finalResult.closest = p;
	finalResult.usedVertices = 0;

	bool pointOutsideABC = originOutsideOfPlane(a, b, c, d);
	bool pointOutsideACD = originOutsideOfPlane(a, c, d, b);
	bool pointOutsideADB = originOutsideOfPlane(a, d, b, c);
	bool pointOutsideBDC = originOutsideOfPlane(b, d, c, a);

	if(!pointOutsideABC && !pointOutsideACD && !pointOutsideADB && !pointOutsideBDC)
		return false;

	f32 bestSqDist = FLT_MAX;

	if(pointOutsideABC) {
		closestPointTriangleFromOrigin(a, b, c, tempResult);
		Vector3 q = tempResult.closest;
		f32 sqDist = dot((q - p), (q - p));
		if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
#ifdef __SPU__
			finalResult.setBarycentricCoordinates(spu_extract(tempResult.barycentricCoords, 0),
												  spu_extract(tempResult.barycentricCoords, 1),
												  spu_extract(tempResult.barycentricCoords, 2),
												  0);
#else
			finalResult.setBarycentricCoordinates(tempResult.barycentricCoords[0], tempResult.barycentricCoords[1], tempResult.barycentricCoords[2], 0);
#endif
		}
    }

	if(pointOutsideACD) {
		closestPointTriangleFromOrigin(a, c, d, tempResult);
		Vector3 q = tempResult.closest;
        f32 sqDist = dot((q - p), (q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
#ifdef __SPU__
			finalResult.setBarycentricCoordinates(spu_extract(tempResult.barycentricCoords, 0),
												  0,
												  spu_extract(tempResult.barycentricCoords, 1),
												  spu_extract(tempResult.barycentricCoords ,2));
#else
			finalResult.setBarycentricCoordinates(tempResult.barycentricCoords[0], 0, tempResult.barycentricCoords[1], tempResult.barycentricCoords[2]);
#endif
		}
    }

	if(pointOutsideADB) {
		closestPointTriangleFromOrigin(a, d, b, tempResult);
		Vector3 q = tempResult.closest;
        f32 sqDist = dot((q - p), (q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
#ifdef __SPU__
			finalResult.setBarycentricCoordinates(spu_extract(tempResult.barycentricCoords, 0),
												  spu_extract(tempResult.barycentricCoords, 2),
												  0,
												  spu_extract(tempResult.barycentricCoords, 1));
#else
			finalResult.setBarycentricCoordinates(tempResult.barycentricCoords[0], tempResult.barycentricCoords[2], 0, tempResult.barycentricCoords[1]);
#endif
		}
    }

	if(pointOutsideBDC) {
		closestPointTriangleFromOrigin(b, d, c, tempResult);
		Vector3 q = tempResult.closest;
        f32 sqDist = dot((q - p), (q - p));
        if(sqDist < bestSqDist) {
			bestSqDist = sqDist;
			finalResult.closest = q;
#ifdef __SPU__
			finalResult.setBarycentricCoordinates(0,
												  spu_extract(tempResult.barycentricCoords, 0),
												  spu_extract(tempResult.barycentricCoords, 2),
												  spu_extract(tempResult.barycentricCoords, 1));
#else
			finalResult.setBarycentricCoordinates(0, tempResult.barycentricCoords[0], tempResult.barycentricCoords[2], tempResult.barycentricCoords[1]);
#endif
		}
    }

    return true;
}

