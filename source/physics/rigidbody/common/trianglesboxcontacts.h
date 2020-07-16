/*
 * trianglesboxcontacts.h
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#ifndef TRIANGLESBOXCONTACTS_H_
#define TRIANGLESBOXCONTACTS_H_

#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/box.h"
#include "rigidbody/common/subdata.h"

s32 trianglesBoxContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, f32 *distance, const TriMesh *meshA, const Transform3& transformA, Box boxB, const Transform3& transformB, f32 distanceThreshold = FLT_MAX);

#endif /* TRIANGLESBOXCONTACTS_H_ */
