/*
 * pcljoint.h
 *
 *  Created on: Jan 25, 2014
 *      Author: mike
 */

#ifndef PCLJOINT_H_
#define PCLJOINT_H_

#include "base/common.h"

#include "particleconfig.h"

enum
{
	PclJointTypePcl,
	PclJointTypeRig
};

ATTRIBUTE_ALIGNED16(struct) PclJoint
{
	bool active : 1;
	u16 type;
	u16 stateIndexA;
	u16 stateIndexB;

	f32 length;
	f32 bias;

#ifdef PARTICLE_VELOCITY_BASE
	f32 maxImpulse;
	f32 impulseDen;
	f32 damping;
#endif

	Vector3 localPosB;

	PclJoint() : localPosB(0.0f)
	{
		active = false;
		bias = 0.2f;
#ifdef PARTICLE_VELOCITY_BASE
		maxImpulse = 0.0f;
		impulseDen = 0.0f;
		damping = 0.2f
#endif
	}
};

#endif /* PCLJOINT_H_ */
