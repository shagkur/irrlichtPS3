/*
 * heightfluidconfig.h
 *
 *  Created on: Jun 11, 2013
 *      Author: mike
 */

#ifndef HEIGHTFLUIDCONFIG_H_
#define HEIGHTFLUIDCONFIG_H_

#include "base/common.h"

ATTRIBUTE_ALIGNED16(struct) FieldPoint
{
	f32 height[3];
	u8 flag;
	u8 offset;
	u16 userData;
};

#endif /* HEIGHTFLUIDCONFIG_H_ */
