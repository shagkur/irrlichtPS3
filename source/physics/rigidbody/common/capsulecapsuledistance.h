/*
 * capsulecapsuledistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CAPSULECAPSULEDISTANCE_H_
#define CAPSULECAPSULEDISTANCE_H_

#include "rigidbody/common/capsule.h"

f32 capsuleCapsuleDistance(Vector3& normal, CapsulePoint& capsulePointA, CapsulePoint& capsulePointB, const Capsule& capsuleA, const Transform3& transformA, const Capsule& capsuleB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* CAPSULECAPSULEDISTANCE_H_ */
