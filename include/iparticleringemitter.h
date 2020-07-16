/*
 * iparticleringemitter.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef IPARTICLERINGEMITTER_H_
#define IPARTICLERINGEMITTER_H_

#include "iparticleemitter.h"

namespace irr
{
	namespace scene
	{
		class IParticleRingEmitter : public IParticleEmitter
		{
		public:
			virtual void setCenter(const core::vector3df& center) = 0;

			virtual void setRadius(f32 radius) = 0;

			virtual void setRingThickness(f32 ringThickness) = 0;

			virtual const core::vector3df& getCenter() const = 0;

			virtual f32 getRadius() const = 0;

			virtual f32 getRingThickness() const = 0;

			virtual E_PARTICLE_EMITTER_TYPE getType() const { return EPET_RING; }
		};
	}
}

#endif /* IPARTICLERINGEMITTER_H_ */
