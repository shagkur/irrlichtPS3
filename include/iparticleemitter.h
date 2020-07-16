/*
 * iparticleemitter.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mike
 */

#ifndef IPARTICLEEMITTER_H_
#define IPARTICLEEMITTER_H_

#include "irefcounter.h"
#include "sparticle.h"

namespace irr
{
	namespace scene
	{
		enum E_PARTICLE_EMITTER_TYPE
		{
			EPET_POINT = 0,
			EPET_ANIMATED_MESH,
			EPET_BOX,
			EPET_CYLINDER,
			EPET_MESH,
			EPET_RING,
			EPET_SPHERE,
			EPET_COUNT
		};

		class IParticleEmitter : public virtual IRefCounter
		{
		public:
			virtual s32 emitt(u32 now, u32 timeSinceLastCall, SParticle *outArray) = 0;

			virtual void setDirection(const core::vector3df& newDirection) = 0;

			virtual void setMinParticlesPerSecond(u32 minPPS) = 0;

			virtual void setMaxParticlesPerSecond(u32 maxPPS) = 0;

			virtual void setMinStartColor(const video::SColor& color) = 0;

			virtual void setMaxStartColor(const video::SColor& color) = 0;

			virtual void setMinStartSize(const core::dimension2df& size) = 0;

			virtual void setMaxStartSize(const core::dimension2df& size) = 0;

			virtual void setMinLifeTime(u32 lifeTime) = 0;

			virtual void setMaxLifeTime(u32 lifeTime) = 0;

			virtual void setMaxAngleDegrees(s32 maxAngleDegrees) = 0;

			virtual const core::vector3df& getDirection() const = 0;

			virtual u32 getMinParticlesPerSecond() const = 0;

			virtual u32 getMaxParticlesPerSecond() const = 0;

			virtual const video::SColor& getMinStartColor() const = 0;

			virtual const video::SColor& getMaxStartColor() const = 0;

			virtual const core::dimension2df& getMinStartSize() const = 0;

			virtual const core::dimension2df& getMaxStartSize() const = 0;

			virtual u32 getMinLifeTime() const = 0;

			virtual u32 getMaxLifeTime() const = 0;

			virtual s32 getMaxAngleDegrees() const = 0;

			virtual E_PARTICLE_EMITTER_TYPE getType() const { return EPET_POINT; }
		};

		typedef IParticleEmitter IParticlePointEmitter;
	}
}

#endif /* IPARTICLEEMITTER_H_ */
