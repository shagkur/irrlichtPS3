/*
 * trianglescapsulecontacts.h
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#ifndef TRIANGLESCAPSULECONTACTS_H_
#define TRIANGLESCAPSULECONTACTS_H_

#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/capsule.h"
#include "rigidbody/common/subdata.h"

s32 trianglesCapsuleContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, f32 *distance, const TriMesh *meshA, const Transform3& transformA, Capsule capB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* TRIANGLESCAPSULECONTACTS_H_ */
