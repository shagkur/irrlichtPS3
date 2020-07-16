/*
 * softcontact.h
 *
 *  Created on: Jun 12, 2013
 *      Author: mike
 */

#ifndef SOFTCONTACT_H_
#define SOFTCONTACT_H_

#include "base/common.h"

#include "softbody/common/softbodyconfig.h"

struct SoftContactPoint
{
	f32 distance;
	f32 maxImpulse;
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

ATTRIBUTE_ALIGNED16(struct) SoftContactPair
{
	SoftContactPoint contactPoint;

	u16 stateIndex[2];
	u16 numContacts;

	void reset()
	{
		numContacts = 0;
		contactPoint.reset();
	}
};

#endif /* SOFTCONTACT_H_ */
