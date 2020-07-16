/*
 * capsulespheredistance.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/capsule.h"
#include "rigidbody/common/sphere.h"
#include "rigidbody/common/vec_utils.h"

inline void segmentPointClosestPoints(Vector3& ptsVector, Vector3& offsetA, f32& tA, Vector3 translation, Vector3 dirA, f32 hLenA)
{
	// compute the parameters of the closest points on each line segment

	tA = dot(dirA, translation);

	if(tA < -hLenA)
		tA = -hLenA;
	else if(tA > hLenA)
		tA = hLenA;

	// compute the closest point on segment relative to its center.

	offsetA = dirA*tA;
	ptsVector = translation - offsetA;
}

inline void segmentPointNormal(Vector3& normal, Vector3 ptsVector)
{
	// compute the unit direction vector between the closest points.
	// with convex objects, you want the unit direction providing the largest gap between the
	// objects when they're projected onto it.  So, if you have a few candidates covering different
	// configurations of the objects, you can compute them all, test the gaps and pick best axis
	// based on this.  Some directions might be degenerate, and the normalized() function tests for
	// degeneracy and returns an arbitrary unit vector in that case.

	// closest points vector

	normal = safeNormalize(ptsVector);
}

f32 capsuleSphereDistance(Vector3& normal, CapsulePoint& capsulePointA, SpherePoint& spherePointB, const Capsule& capsuleA, const Transform3& transformA, Sphere sphereB, const Transform3& transformB, f32 distanceThreshold)
{
	Vector3 directionA = transformA.getUpper3x3().getCol0();
	Vector3 translationA = transformA.getTranslation();
	Vector3 translationB = transformB.getTranslation();

	// translation between centers of capsule and sphere

	Vector3 translation = translationB - translationA;

	// compute the closest point on the capsule line segment to the sphere center

	Vector3 ptsVector;
	Vector3 offsetA;
	f32 tA;

	segmentPointClosestPoints(ptsVector, offsetA, tA, translation, directionA, capsuleA.hLength);

	f32 distance = length(ptsVector) - capsuleA.radius - sphereB.radius;

	if(distance > distanceThreshold)
		return distance;

	// compute the contact normal

	segmentPointNormal(normal, ptsVector);

	// compute points on capsule and sphere

	capsulePointA.lineParam = tA;
	capsulePointA.localPoint = Point3(transpose(transformA.getUpper3x3())*(offsetA + normal*capsuleA.radius));
	capsulePointA.setEdgeFeature();

	spherePointB.localPoint = Point3(transpose(transformB.getUpper3x3())*(-normal*sphereB.radius));
	spherePointB.setVertexFeature();

	// return distance

	return distance;
}

