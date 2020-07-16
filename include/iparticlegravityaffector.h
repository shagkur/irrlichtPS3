/*
 * iparticlegravityaffector.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef IPARTICLEGRAVITYAFFECTOR_H_
#define IPARTICLEGRAVITYAFFECTOR_H_

#include "iparticleaffector.h"

namespace irr
{
	namespace scene
	{
		class IParticleGravityAffector : public IParticleAffector
		{
		public:
			virtual void setTimeForceLost(f32 timeForceLost) = 0;

			virtual void setGravity(const core::vector3df& gravity) = 0;

			virtual f32 getTimeForceLost() const = 0;

			virtual const core::vector3df& getGravity() const = 0;

			virtual E_PARTICLE_AFFECTOR_TYPE getType() const { return EPAT_GRAVITY; }
		};
	}
}

#endif /* IPARTICLEGRAVITYAFFECTOR_H_ */
