/*
 * spring.h
 *
 *  Created on: Jun 5, 2013
 *      Author: mike
 */

#ifndef SPRING_H_
#define SPRING_H_

#include "base/common.h"

ATTRIBUTE_ALIGNED16(struct) Spring
{
	f32 length;
	f32 ks;
	f32 kd;

	u8 active;

	u16 stateIndexA;
	u16 stateIndexB;

	Vector3 anchorA;
	Vector3 anchorB;
};

#endif /* SPRING_H_ */
