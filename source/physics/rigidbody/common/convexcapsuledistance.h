/*
 * convexcapsuledistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CONVEXCAPSULEDISTANCE_H_
#define CONVEXCAPSULEDISTANCE_H_

#include "rigidbody/common/trimesh.h"

f32 closestConvexCapsule(Vector3& normal, Point3& pointA, Point3& pointB, const ConvexMesh *meshA, const Transform3& transformA, Capsule capB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* CONVEXCAPSULEDISTANCE_H_ */
