/*
 * iparticlecylinderemitter.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef IPARTICLECYLINDEREMITTER_H_
#define IPARTICLECYLINDEREMITTER_H_

#include "iparticleemitter.h"

namespace irr
{
	namespace scene
	{
		class IParticleCylinderEmitter : public IParticleEmitter
		{
		public:
			virtual void setCenter(const core::vector3df& center) = 0;

			virtual void setNormal(const core::vector3df& normal) = 0;

			virtual void setRadius(f32 radius) = 0;

			virtual void setLength(f32 length) = 0;

			virtual void setOutlineOnly(bool outlineOnly = true) = 0;

			virtual const core::vector3df& getCenter() const = 0;

			virtual const core::vector3df& getNormal() const = 0;

			virtual f32 getRadius() const = 0;

			virtual f32 getLength() const = 0;

			virtual bool getOutlineOnly() const = 0;

			virtual E_PARTICLE_EMITTER_TYPE getType() const { return EPET_CYLINDER; }
		};
	}
}

#endif /* IPARTICLECYLINDEREMITTER_H_ */
