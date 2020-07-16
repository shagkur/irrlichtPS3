/*
 * boxboxdistance.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef BOXBOXDISTANCE_H_
#define BOXBOXDISTANCE_H_

#include "rigidbody/common/box.h"

f32 boxBoxDistance(Vector3& normal, BoxPoint& boxPointA, BoxPoint& boxPointB, Box boxA, const Transform3& transformA, Box boxB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* BOXBOXDISTANCE_H_ */
