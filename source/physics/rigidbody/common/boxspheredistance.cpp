/*
 * boxspheredistance.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/box.h"
#include "rigidbody/common/sphere.h"

//---------------------------------------------------------------------------
// lenSqrTol: minimum square of length for safe normalize.
//---------------------------------------------------------------------------

static const f32 lenSqrTol = 1.0e-30f;

//---------------------------------------------------------------------------
// separating axis tests: gaps along each axis are computed, and the axis with the maximum
// gap is stored.  cross product axes must be normalized.
//---------------------------------------------------------------------------

#define AaxisTest(dim, letter, first)                                                         \
{                                                                                               \
   f32 gap = gapsA.get##letter();                                                             \
                                                                                                \
   if(gap > distanceThreshold)                                                               \
      return gap;                                                                               \
                                                                                                \
   if(first) {                                                                                            \
      maxGap = gap;                                                                             \
      faceDimA = dim;                                                                           \
      axisA = mulPerElem(identity.getCol##dim(), signsA);                                     \
   } else {                                                                                            \
      if(gap > maxGap) {                                                                                         \
         maxGap = gap;                                                                          \
         faceDimA = dim;                                                                        \
         axisA = mulPerElem(identity.getCol##dim(), signsA);                                  \
      }                                                                                         \
   }                                                                                            \
}

inline f32 VertexBFaceATest(Vector3& ptsVec, f32& t0, f32& t1, const Vector3& hA, Vector3 offsetAB)
{
	// compute center of sphere in box's coordinate system

	Vector3 cptsVec = Vector3(offsetAB);

	// compute the parameters of the point on the face

	t0 = cptsVec[0];
	t1 = cptsVec[1];

	if(t0 > hA[0])
		t0 = hA[0];
	else if(t0 < -hA[0])
		t0 = -hA[0];
	if(t1 > hA[1])
		t1 = hA[1];
	else if(t1 < -hA[1])
		t1 = -hA[1];

	cptsVec[0] -= t0;
	cptsVec[1] -= t1;

	ptsVec = Vector3(cptsVec);

	return dot(ptsVec, ptsVec);
}

f32 boxSphereDistance(Vector3& normal, BoxPoint& boxPointA, SpherePoint& spherePointB, Box boxA, const Transform3& transformA, const Sphere& sphereB, const Transform3& transformB, f32 distanceThreshold)
{
	Matrix3 identity = Matrix3::identity();
	Vector3 ident[3];
	ident[0] = identity.getCol0();
	ident[1] = identity.getCol1();
	ident[2] = identity.getCol2();

	// offsetAB is vector from A's center to B's center, in A's coordinate system

	Vector3 translationB = transformB.getTranslation();
	Vector3 offsetAB = transpose(transformA.getUpper3x3())*( translationB - transformA.getTranslation());

	// find separating axis with largest gap between objects

	Vector3 axisA;
	s32 faceDimA;
	f32 maxGap;

	Vector3 gapsA = absPerElem(offsetAB) - boxA.half - Vector3(sphereB.radius);
	Vector3 signsA = copySignPerElem(Vector3(1.0f), offsetAB);

	AaxisTest(0, X, true);
	AaxisTest(1, Y, false);
	AaxisTest(2, Z, false);

	// choose face in this direction, and make a new coordinate system which the z axis = face
	// normal, x and y axes tangent to the face.  to transform vectors into this coordinate
	// system, will use a permutation matrix.

	s32 dimA[3];

	dimA[2] = faceDimA;
	dimA[0] = (faceDimA + 1)%3;
	dimA[1] = (faceDimA + 2)%3;

	Matrix3 apermCol;

	apermCol.setCol0(ident[dimA[0]]);
	apermCol.setCol1(ident[dimA[1]]);
	apermCol.setCol2(ident[dimA[2]]);

	Matrix3 apermRow = transpose(apermCol);

	// permute vectors

	Vector3 halfA_perm = apermRow*boxA.half;
	Vector3 offsetAB_perm = apermRow*offsetAB;
	Vector3 signsA_perm = apermRow*signsA;

	// compute the vector between the center of the box face and the sphere center

	f32 signA2 = signsA_perm.getZ();
	f32 scaleA2 = halfA_perm.getZ()*signA2;
	offsetAB_perm.setZ(offsetAB_perm.getZ() - scaleA2);

	// find point on face closest to sphere center

	f32 t0, t1;
	f32 minDistSqr;
	Vector3 closestPtsVec_perm;
	Point3 localPointA_perm;

	minDistSqr = VertexBFaceATest(closestPtsVec_perm, t0, t1, Vector3( halfA_perm ), offsetAB_perm);

	localPointA_perm = Point3(t0, t1, scaleA2);

	// compute normal

	bool centerInside = (signA2*closestPtsVec_perm.getZ() < 0.0f );

	if(centerInside || (minDistSqr < lenSqrTol))
		normal = transformA*axisA;
	else {
		Vector3 closestPtsVec = apermCol*closestPtsVec_perm;
		normal = transformA*(closestPtsVec*(1.0f/sqrtf(minDistSqr)));
	}

	// compute box point

	boxPointA.localPoint = Point3(apermCol*Vector3(localPointA_perm));
	boxPointA.setFaceFeature(faceDimA, localPointA_perm.getZ() > 0.0f);

	// compute sphere point

	spherePointB.localPoint = Point3(transpose(transformB.getUpper3x3())*(-normal*sphereB.radius));
	spherePointB.setVertexFeature();

	// return distance

	if(centerInside)
		return -sqrtf(minDistSqr) - sphereB.radius;
	else
		return sqrtf(minDistSqr) - sphereB.radius;
}

