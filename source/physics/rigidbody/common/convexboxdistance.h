/*
 * convexboxdistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CONVEXBOXDISTANCE_H_
#define CONVEXBOXDISTANCE_H_

#include "rigidbody/common/trimesh.h"

f32 closestConvexBox(Vector3& normal, Point3 &pointA, Point3 &pointB, const ConvexMesh *meshA, const Transform3& transformA, Box boxB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* CONVEXBOXDISTANCE_H_ */
