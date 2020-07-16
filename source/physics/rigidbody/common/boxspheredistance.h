/*
 * boxspheredistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef BOXSPHEREDISTANCE_H_
#define BOXSPHEREDISTANCE_H_

#include "rigidbody/common/box.h"
#include "rigidbody/common/sphere.h"

f32 boxSphereDistance(Vector3& normal, BoxPoint& boxPointA, SpherePoint& spherePointB, Box boxA, const Transform3& transformA, const Sphere& sphereB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* BOXSPHEREDISTANCE_H_ */
