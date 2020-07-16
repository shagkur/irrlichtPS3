/*
 * iparticleanimatedmeshscenenodeemitter.h
 *
 *  Created on: Feb 27, 2013
 *      Author: mike
 */

#ifndef IPARTICLEANIMATEDMESHSCENENODEEMITTER_H_
#define IPARTICLEANIMATEDMESHSCENENODEEMITTER_H_

#include "iparticleemitter.h"
#include "ianimatedmeshscenenode.h"

namespace irr
{
	namespace scene
	{
		class IParticleAnimatedMeshSceneNodeEmitter : public IParticleEmitter
		{
		public:
			virtual void setAnimatedMeshSceneNode(IAnimatedMeshSceneNode *node) = 0;

			virtual void setUseNormalDirection(bool useNormalDirection = true) = 0;

			virtual void setNormalDirectionModifier(f32 normalDirectionModifier) = 0;

			virtual void setEveryMeshVertex(bool everyMeshVertex = true) = 0;

			virtual const IAnimatedMeshSceneNode* getAnimatedMeshSceneNode() const = 0;

			virtual bool isUsingNormalDirection() const = 0;

			virtual f32 getNormalDirectionModifier() const = 0;

			virtual bool getEveryMeshVertex() const = 0;

			virtual E_PARTICLE_EMITTER_TYPE getType() const { return EPET_ANIMATED_MESH; }
		};
	}
}


#endif /* IPARTICLEANIMATEDMESHSCENENODEEMITTER_H_ */
