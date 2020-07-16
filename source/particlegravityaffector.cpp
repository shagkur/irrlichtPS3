/*
 * particlegravityaffector.cpp
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#include "particlegravityaffector.h"
#include "os.h"

namespace irr
{
	namespace scene
	{
		CParticleGravityAffector::CParticleGravityAffector(const core::vector3df& gravity, u32 timeForceLost)
		: IParticleGravityAffector(), _timeForceLost(static_cast<f32>(timeForceLost)), _gravity(gravity)
		{

		}

		void CParticleGravityAffector::affect(u32 now, SParticle *particleArray, u32 count)
		{
			if(!_enabled) return;

			f32 d;

			for(u32 i=0;i < count;i++) {
				d = (now - particleArray[i].startTime)/_timeForceLost;
				if(d > 1.0f)
					d = 1.0f;
				if(d < 0.0f)
					d = 0.0f;

				d = 1.0f - d;

				core::interpolate(particleArray[i].startVector, _gravity, particleArray[i].vector, d);
			}
		}
	}
}
