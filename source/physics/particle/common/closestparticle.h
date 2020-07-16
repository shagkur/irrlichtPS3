/*
 * closestparticle.h
 *
 *  Created on: Jan 27, 2014
 *      Author: mike
 */

#ifndef CLOSESTPARTICLE_H_
#define CLOSESTPARTICLE_H_

#include "pclcontact.h"

#include "rigidbody/common/collobject.h"

// Particle x Particle
bool closestContactPcl(PclContactPair& contactPair, const Vector3& posA, f32 radA, const Vector3& posB, f32 radB,  f32 objsInContactDist = FLT_MAX);

// Particle x Rigid Body
bool closestContactRig(PclContactPair& contactPair, const Vector3& posA, f32 radA, const CollObject& objB, const Transform3& transformB, f32 objsInContactDist = FLT_MAX);

#endif /* CLOSESTPARTICLE_H_ */
