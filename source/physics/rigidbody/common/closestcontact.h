/*
 * closestcontact.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CLOSESTCONTACT_H_
#define CLOSESTCONTACT_H_

#include "rigidbody/common/contact.h"
#include "rigidbody/common/collobject.h"

struct CCDDebugSphere {
	float r;
	bool collide;
	Vector3 pos;
};

inline Transform3 interpTransform(f32 t, const Transform3& tr0, const Transform3& tr1)
{
	return Transform3(slerp(t, Quat(tr0.getUpper3x3()), Quat(tr1.getUpper3x3())), lerp(t, tr0.getTranslation(), tr1.getTranslation()));
}

inline Transform3 integrateTransform(f32 timeStep, const Transform3& tr, const Vector3& linVel, const Vector3& angVel)
{
	Quat orientation(tr.getUpper3x3());
	Quat dq =  Quat(angVel, 0)*orientation*0.5f;
	return Transform3(normalize(orientation + dq*timeStep), tr.getTranslation() + linVel*timeStep);
}

bool closestContact(ContactPair& contactPair, const CollObject& objA, const Transform3& transformA, const CollObject& objB, const Transform3& transformB, f32 objsInContactDist = FLT_MAX);
u32 findContactCCD(ContactPair& contactPair, const CollObject& objA, const Transform3& tA0, const Transform3& tA1, bool useCcdA, const CollObject& objB, const Transform3& tB0,const Transform3& tB1, bool useCcdB, f32 objsInContactDist = FLT_MAX);

#endif /* CLOSESTCONTACT_H_ */
