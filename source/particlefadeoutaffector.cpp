/*
 * particlefadeoutaffector.cpp
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#include "particlefadeoutaffector.h"
#include "os.h"

namespace irr
{
	namespace scene
	{
		CParticleFadeOutAffector::CParticleFadeOutAffector(const video::SColor& targetColor, u32 fadeOutTime)
		: IParticleFadeOutAffector(), _targetColor(targetColor)
		{
			_fadeOutTime = fadeOutTime ? static_cast<f32>(fadeOutTime) : 1.0f;
		}

		void CParticleFadeOutAffector::affect(u32 now, SParticle *particleArray, u32 count)
		{
			if(!_enabled) return;

			f32 d;

			for(u32 i=0;i < count;i++) {
				if(particleArray[i].endTime - now < _fadeOutTime) {
					d = (particleArray[i].endTime - now)/_fadeOutTime;
					particleArray[i].color = particleArray[i].startColor.getInterpolated(_targetColor, d);
				}
			}
		}
	}
}


