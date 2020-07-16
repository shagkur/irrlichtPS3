/*
 * constraintcache.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef CONSTRAINTCACHE_H_
#define CONSTRAINTCACHE_H_

#include "base/common.h"

// Don't change following order of parameters
ATTRIBUTE_ALIGNED16(struct) ConstraintCache {
	f32 normal[3];
	f32 rhs;
	f32 jacDiagInv;
	f32 lowerLimit;
	f32 upperLimit;
	f32 accumImpulse;
};

#endif /* CONSTRAINTCACHE_H_ */
