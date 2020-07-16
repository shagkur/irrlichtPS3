/*
 * iparticlesphereemitter.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef IPARTICLESPHEREEMITTER_H_
#define IPARTICLESPHEREEMITTER_H_

#include "iparticleemitter.h"

namespace irr
{
	namespace scene
	{
		class IParticleSphereEmitter : public IParticleEmitter
		{
		public:
			virtual void setCenter(const core::vector3df& center) = 0;

			virtual void setRadius(f32 radius) = 0;

			virtual const core::vector3df& getCenter() const = 0;

			virtual f32 getRadius() const = 0;

			virtual E_PARTICLE_EMITTER_TYPE getType() const { return EPET_SPHERE; }
		};
	}
}

#endif /* IPARTICLESPHEREEMITTER_H_ */
