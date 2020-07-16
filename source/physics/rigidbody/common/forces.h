/*
 * forces.h
 *
 *  Created on: Apr 26, 2013
 *      Author: mike
 */

#ifndef FORCES_H_
#define FORCES_H_

#include "base/common.h"

struct Forces
{
	Vector3 force;
	Vector3 torque;

	Forces() : force(0.0f), torque(0.0f) {}
};

#endif /* FORCES_H_ */
