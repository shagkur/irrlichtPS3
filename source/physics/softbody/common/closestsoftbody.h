/*
 * closestsoftbody.h
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#ifndef CLOSESTSOFTBODY_H_
#define CLOSESTSOFTBODY_H_

#include "softbody/common/softcontact.h"

#include "rigidbody/common/collobject.h"

// Particle x Particle
bool closestContactPcl(SoftContactPair& contactPair, const Vector3& posA, f32 radA, const Vector3& posB, f32 radB, f32 objsInContactDist = FLT_MAX);

// Particle x Rigid Body
bool closestContactRig(SoftContactPair& contactPair, const Vector3& posA, f32 radA, const CollObject&  objB, const Transform3& transformB, f32 objsInContactDist = FLT_MAX);

#endif /* CLOSESTSOFTBODY_H_ */
