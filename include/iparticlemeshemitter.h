/*
 * iparticlemeshemitter.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef IPARTICLEMESHEMITTER_H_
#define IPARTICLEMESHEMITTER_H_

#include "iparticleemitter.h"
#include "imesh.h"

namespace irr
{
	namespace scene
	{
		class IParticleMeshEmitter : public IParticleEmitter
		{
		public:
			virtual void setMesh(IMesh *mesh) = 0;

			virtual void setUseNormalDirection(bool useNormalDirection = true) = 0;

			virtual void setNormalDirectionModifier(f32 normalDirectionModifier) = 0;

			virtual void setEveryMeshVertex(bool everyMeshVertex = true) = 0;

			virtual const IMesh* getMesh() const = 0;

			virtual bool isUsingNormalDirection() const = 0;

			virtual f32 getNormalDirectionModifier() const = 0;

			virtual bool getEveryMeshVertex() const = 0;

			virtual E_PARTICLE_EMITTER_TYPE getType() const { return EPET_MESH; }
		};
	}
}

#endif /* IPARTICLEMESHEMITTER_H_ */
