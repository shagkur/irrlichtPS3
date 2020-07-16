/*
 * softjoint.h
 *
 *  Created on: Jun 12, 2013
 *      Author: mike
 */

#ifndef SOFTJOINT_H_
#define SOFTJOINT_H_

#include "base/common.h"

#include "softbody/common/softbodyconfig.h"

enum
{
	SoftJointTypePcl,
	SoftJointTypeRig
};

ATTRIBUTE_ALIGNED16(struct) SoftJoint
{
	bool active : 1;
	u16 type;
	u16 stateIndexA;
	u16 stateIndexB;
	f32 length;
	f32 bias;
	Vector3 localPosB;

	SoftJoint() : localPosB(0.0f)
	{
		active = false;
		bias = 0.2f;
	}
};

#endif /* SOFTJOINT_H_ */
