/*
 * iparticlefadeoutaffector.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef IPARTICLEFADEOUTAFFECTOR_H_
#define IPARTICLEFADEOUTAFFECTOR_H_

#include "iparticleaffector.h"

namespace irr
{
	namespace scene
	{
		class IParticleFadeOutAffector : public IParticleAffector
		{
		public:
			virtual void setTargetColor(const video::SColor& targetColor) = 0;

			virtual void setFadeOutTime(u32 fadeOutTime) = 0;

			virtual const video::SColor& getTargetColor() const = 0;

			virtual u32 getFadeOutTime() const = 0;

			virtual E_PARTICLE_AFFECTOR_TYPE getType() const { return EPAT_FADE_OUT; }
		};
	}
}

#endif /* IPARTICLEFADOUTAFFECTOR_H_ */
