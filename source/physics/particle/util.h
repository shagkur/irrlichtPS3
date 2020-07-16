/*
 * util.h
 *
 *  Created on: Jan 29, 2014
 *      Author: mike
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "particles.h"

namespace ParticleUtil
{
	void createParticleGroup(ParticleGroup& particleGroup, ParticleGroupProperty& property);
	void releaseParticleGroup(ParticleGroup& particleGroup);
}

#endif /* UTIL_H_ */
