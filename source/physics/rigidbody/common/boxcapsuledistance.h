/*
 * boxcapsuledistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef BOXCAPSULEDISTANCE_H_
#define BOXCAPSULEDISTANCE_H_

#include "rigidbody/common/box.h"
#include "rigidbody/common/capsule.h"

f32 boxCapsuleDistance(Vector3& normal,	BoxPoint& boxPointA, CapsulePoint& capsulePointB, Box boxA, const Transform3& transformA, const Capsule& capsuleB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* BOXCAPSULEDISTANCE_H_ */
