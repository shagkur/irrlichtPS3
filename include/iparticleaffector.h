/*
 * iparticleaffector.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mike
 */

#ifndef IPARTICLEAFFECTOR_H_
#define IPARTICLEAFFECTOR_H_

#include "irefcounter.h"
#include "sparticle.h"

namespace irr
{
	namespace scene
	{
		enum E_PARTICLE_AFFECTOR_TYPE
		{
			EPAT_NONE = 0,
			EPAT_ATTRACT,
			EPAT_FADE_OUT,
			EPAT_GRAVITY,
			EPAT_ROTATE,
			EPAT_SCALE,
			EPAT_COUNT
		};

		class IParticleAffector : public virtual IRefCounter
		{
		public:
			IParticleAffector() : _enabled(true) {}

			virtual void affect(u32 now, SParticle *particleArray, u32 count) = 0;

			virtual void setEnabled(bool enabled) { _enabled = enabled; }

			virtual bool getEnabled() const { return _enabled; }

			virtual E_PARTICLE_AFFECTOR_TYPE getType() const = 0;
		protected:
			bool _enabled;
		};
	}
}

#endif /* IPARTICLEAFFECTOR_H_ */
