/*
 * convexspheredistance.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CONVEXSPHEREDISTANCE_H_
#define CONVEXSPHEREDISTANCE_H_

#include "rigidbody/common/sphere.h"
#include "rigidbody/common/trimesh.h"

f32 closestConvexSphere(Vector3& normal, Point3& pointA, Point3& pointB, const ConvexMesh *meshA, const Transform3& transformA, Sphere sphereB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* CONVEXSPHEREDISTANCE_H_ */
