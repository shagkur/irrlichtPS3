/*
 * pclcontact.h
 *
 *  Created on: Jan 25, 2014
 *      Author: mike
 */

#ifndef PCLCONTACT_H_
#define PCLCONTACT_H_

#include "base/common.h"

#include "particleconfig.h"

struct PclContactPoint
{
	f32 distance;
	f32 maxImpulse;
#ifdef PARTICLE_VELOCITY_BASE
	f32 impulseDen;
	f32 frictionDen;
#endif
	Vector3 localPoint[2];
	Vector3 normal;

	void reset()
	{
		distance = FLT_MAX;
	}

	const Vector3& getNormal() const
	{
		return normal;
	}

	f32 getMaxImpulse() const
	{
		return maxImpulse;
	}
};

ATTRIBUTE_ALIGNED16(struct) PclContactPair
{
	PclContactPoint contactPoint;

	u16 stateIndex[2];
	u16 numContacts;
#ifdef PARTICLE_VELOCITY_BASE
	f32 compositeElasticity;
	f32 compositeFriction;
#endif

	void reset()
	{
		numContacts = 0;
		contactPoint.reset();
	}
};

#endif /* PCLCONTACT_H_ */
