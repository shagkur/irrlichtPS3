/*
 * iparticleattractionaffector.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef IPARTICLEATTRACTIONAFFECTOR_H_
#define IPARTICLEATTRACTIONAFFECTOR_H_

#include "iparticleaffector.h"

namespace irr
{
	namespace scene
	{
		class IParticleAttractionAffector : public IParticleAffector
		{
		public:
			virtual void setPoint(const core::vector3df& point) = 0;

			virtual void setAttract(bool attract) = 0;

			virtual void setAffectX(bool affect) = 0;

			virtual void setAffectY(bool affect) = 0;

			virtual void setAffectZ(bool affect) = 0;

			virtual const core::vector3df& getPoint() const = 0;

			virtual bool getAttract() const = 0;

			virtual bool getAffectX() const = 0;

			virtual bool getAffectY() const = 0;

			virtual bool getAffectZ() const = 0;

			virtual E_PARTICLE_AFFECTOR_TYPE getType() const { return EPAT_ATTRACT; }
		};
	}
}

#endif /* IPARTICLEATTRACTIONAFFECTOR_H_ */
