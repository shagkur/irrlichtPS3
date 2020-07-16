/*
 * iparticlerotationaffector.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mike
 */

#ifndef IPARTICLEROTATIONAFFECTOR_H_
#define IPARTICLEROTATIONAFFECTOR_H_

#include "iparticleaffector.h"

namespace irr
{
	namespace scene
	{
		class IParticleRotationAffector : public IParticleAffector
		{
		public:
			virtual void setPivotPoint(const core::vector3df& point) = 0;

			virtual void setSpeed(const core::vector3df& speed) = 0;

			virtual const core::vector3df& getPivotPoint() const = 0;

			virtual const core::vector3df& getSpeed() const = 0;

			virtual E_PARTICLE_AFFECTOR_TYPE getType() const { return EPAT_ROTATE; }
		};
	}
}

#endif /* IPARTICLEROTATIONAFFECTOR_H_ */
