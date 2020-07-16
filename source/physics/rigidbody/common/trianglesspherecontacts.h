/*
 * trianglesspherecontacts.h
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#ifndef TRIANGLESSPHERECONTACTS_H_
#define TRIANGLESSPHERECONTACTS_H_

#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/sphere.h"
#include "rigidbody/common/subdata.h"

s32 trianglesSphereContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, f32 *distance, const TriMesh *meshA, const Transform3& transformA, Sphere sphereB, const Transform3& transformB,float distanceThreshold = FLT_MAX);

#endif /* TRIANGLESSPHERECONTACTS_H_ */
