/*
 * convexconvexdistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CONVEXCONVEXDISTANCE_H_
#define CONVEXCONVEXDISTANCE_H_

#include "rigidbody/common/trimesh.h"

f32 closestConvexConvex(Vector3& normal, Point3& pointA, Point3& pointB, const ConvexMesh *meshA, const Transform3& transformA, const ConvexMesh *meshB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* CONVEXCONVEXDISTANCE_H_ */
