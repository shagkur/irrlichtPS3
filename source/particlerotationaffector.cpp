/*
 * particlerotationaffector.cpp
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#include "particlerotationaffector.h"

namespace irr
{
	namespace scene
	{
		CParticleRotationAffector::CParticleRotationAffector(const core::vector3df& speed, const core::vector3df& point)
		: _pivotPoint(point), _speed(speed), _lastTime(0)
		{

		}

		void CParticleRotationAffector::affect(u32 now, SParticle *particleArray, u32 count)
		{
			if(_lastTime == 0) {
				_lastTime = now;
				return;
			}

			f32 timeDelta = (now - _lastTime)/1000.0f;

			_lastTime = now;
			if(!_enabled) return;

			for(u32 i=0;i < count;i++) {
			}
		}
	}
}
