/*
 * particlerotationaffector.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef PARTICLEROTATIONAFFECTOR_H_
#define PARTICLEROTATIONAFFECTOR_H_

#include "iparticlerotationaffector.h"

namespace irr
{
	namespace scene
	{
		class CParticleRotationAffector : public IParticleRotationAffector
		{
		public:
			CParticleRotationAffector(const core::vector3df& speed = core::vector3df(5.0f, 5.0f, 5.0f), const core::vector3df& point = core::vector3df(0, 0, 0));

			virtual void affect(u32 now, SParticle *particleArray, u32 count);

			virtual void setPivotPoint(const core::vector3df& point) { _pivotPoint = point; }

			virtual void setSpeed(const core::vector3df& speed) { _speed = speed; }

			virtual const core::vector3df& getPivotPoint() const { return _pivotPoint; }

			virtual const core::vector3df& getSpeed() const { return _speed; }

		private:
			core::vector3df _pivotPoint;
			core::vector3df _speed;
			u32 _lastTime;
		};
	}
}

#endif /* PARTICLEROTATIONAFFECTOR_H_ */
