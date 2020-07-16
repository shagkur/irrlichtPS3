/*
 * spherespheredistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef SPHERESPHEREDISTANCE_H_
#define SPHERESPHEREDISTANCE_H_

#include "rigidbody/common/sphere.h"

//---------------------------------------------------------------------------
// sphereSphereDistance:
//
// returns:
//    positive or negative distance between two spheres.
//
// args:
//    Vector3& normal: set to a unit contact normal pointing from sphere A to sphere B.
//
//    SpherePoint& spherePointA, SpherePoint& spherePointB:
//       set to a closest point or point of penetration on each object
//
//    const Sphere& sphereA, const Sphere& sphereB: sphere sizes
//
//    const Transform3 & transformA,B: the translation gives the center position
//
//    float distanceThreshold:
//       the algorithm will exit early if it finds that the objects are more distant than this
//       threshold, and not compute a contact normal or points.  if this distance returned
//       exceeds the threshold, all the other output data may not have been computed.  by
//       default, this is set to MAX_FLOAT so it will have no effect.
//
//---------------------------------------------------------------------------

#define lenSqrTol 			1.0e-30f

inline f32 sphereSphereDistance(Vector3& direction, SpherePoint& spherePointA, SpherePoint& spherePointB, Sphere sphereA, const Transform3& transformA, Sphere sphereB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX)
{
	Vector3 translationA = transformA.getTranslation();
	Vector3 translationB = transformB.getTranslation();

	// get the offset vector between sphere centers

	Vector3 offsetAB;

	offsetAB = translationB - translationA;

	// normalize the offset to compute the direction vector

	f32 distSqr = dot(offsetAB, offsetAB);
	f32 dist = sqrtf(distSqr);
	f32 sphereDist = dist - sphereA.radius - sphereB.radius;

	if(sphereDist > distanceThreshold)
		return sphereDist;

	if(distSqr > lenSqrTol) {
		f32 distInv = 1.0f/dist;
		direction = offsetAB*distInv;
	} else
		direction = Vector3(0.0f, 0.0f, 1.0f);

	// compute the points on the spheres, in world space

	spherePointA.localPoint = Point3(transpose(transformA.getUpper3x3())*(direction*sphereA.radius));
	spherePointB.localPoint = Point3(transpose(transformB.getUpper3x3())*(-direction*sphereB.radius));

	spherePointA.setVertexFeature();
	spherePointB.setVertexFeature();

	// return the distance between the spheres

	return sphereDist;
}

#endif /* SPHERESPHEREDISTANCE_H_ */
