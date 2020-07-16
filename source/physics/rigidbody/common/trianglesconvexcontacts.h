/*
 * trianglesconvexcontacts.h
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#ifndef TRIANGLESCONVEXCONTACTS_H_
#define TRIANGLESCONVEXCONTACTS_H_

#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/subdata.h"

s32 trianglesConvexContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, f32 *distance, const TriMesh *meshA, const Transform3& transformA, const ConvexMesh *meshB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* TRIANGLESCONVEXCONTACTS_H_ */
