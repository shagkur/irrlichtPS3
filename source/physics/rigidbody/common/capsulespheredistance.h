/*
 * capsulespheredistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CAPSULESPHEREDISTANCE_H_
#define CAPSULESPHEREDISTANCE_H_

#include "rigidbody/common/capsule.h"
#include "rigidbody/common/sphere.h"

f32 capsuleSphereDistance(Vector3& normal, CapsulePoint& capsulePointA, SpherePoint& spherePointB, const Capsule& capsuleA, const Transform3& transformA, Sphere sphereB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* CAPSULESPHEREDISTANCE_H_ */
