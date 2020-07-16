/*
 * particlefadeoutaffector.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef PARTICLEFADEOUTAFFECTOR_H_
#define PARTICLEFADEOUTAFFECTOR_H_

#include "iparticlefadeoutaffector.h"
#include "scolor.h"

namespace irr
{
	namespace scene
	{
		class CParticleFadeOutAffector : public IParticleFadeOutAffector
		{
		public:
			CParticleFadeOutAffector(const video::SColor& targetColor, u32 fadeOutTime);

			virtual void affect(u32 now, SParticle *particleArray, u32 count);

			virtual void setTargetColor(const video::SColor& targetColor) { _targetColor = targetColor; }

			virtual void setFadeOutTime(f32 fadeOutTime) { _fadeOutTime = fadeOutTime; }

			virtual const video::SColor& getTargetColor() const { return _targetColor; }

			virtual u32 getFadeOutTime() const { return static_cast<u32>(_fadeOutTime); }

		private:
			video::SColor _targetColor;
			f32 _fadeOutTime;
		};
	}
}

#endif /* PARTICLEFADEOUTAFFECTOR_H_ */
