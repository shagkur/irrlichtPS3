/*
 * iparticleboxemitter.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mike
 */

#ifndef IPARTICLEBOXEMITTER_H_
#define IPARTICLEBOXEMITTER_H_

#include "iparticleemitter.h"
#include "aabbox3d.h"

namespace irr
{
	namespace scene
	{
		class IParticleBoxEmitter : public IParticleEmitter
		{
		public:
			virtual void setBox(const core::aabbox3df& box) = 0;

			virtual const core::aabbox3df& getBox() const = 0;

			virtual E_PARTICLE_EMITTER_TYPE getType() const { return EPET_BOX; }
		};
	}
}

#endif /* IPARTICLEBOXEMITTER_H_ */
