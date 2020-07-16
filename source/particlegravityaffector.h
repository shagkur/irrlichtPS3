/*
 * particlegravityaffector.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef PARTICLEGRAVITYAFFECTOR_H_
#define PARTICLEGRAVITYAFFECTOR_H_

#include "iparticlegravityaffector.h"
#include "scolor.h"

namespace irr
{
	namespace scene
	{
		class CParticleGravityAffector : public IParticleGravityAffector
		{
		public:
			CParticleGravityAffector(const core::vector3df& gravity = core::vector3df(0.0f, -0.03f, 0.0f), u32 timeForceLost = 1000);

			virtual void affect(u32 now, SParticle *particleArray, u32 count);

			virtual void setTimeForceLost(f32 timeForceLost) { _timeForceLost = timeForceLost; }

			virtual void setGravity(const core::vector3df& gravity) { _gravity = gravity; }

			virtual f32 getTimeForceLost() const { return _timeForceLost; }

			virtual const core::vector3df& getGravity() const { return _gravity; }

		private:
			f32 _timeForceLost;
			core::vector3df _gravity;
		};
	}
}

#endif /* PARTICLEGRAVITYAFFECTOR_H_ */
