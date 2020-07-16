/*
 * iparticlesystemscenenode.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mike
 */

#ifndef IPARTICLESYSTEMSCENENODE_H_
#define IPARTICLESYSTEMSCENENODE_H_

#include "iscenenode.h"
#include "iparticleboxemitter.h"
#include "iparticleringemitter.h"
#include "iparticlemeshemitter.h"
#include "iparticlesphereemitter.h"
#include "iparticlecylinderemitter.h"
#include "iparticleanimatedmeshscenenodeemitter.h"
#include "iparticlefadeoutaffector.h"
#include "iparticlegravityaffector.h"
#include "iparticlerotationaffector.h"
#include "iparticleattractionaffector.h"
#include "dimension2d.h"

namespace irr
{
	namespace scene
	{
		class IParticleSystemSceneNode : public ISceneNode
		{
		public:
			IParticleSystemSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos = core::vector3df(0, 0, 0), const core::vector3df& rot = core::vector3df(0, 0, 0), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f))
			: ISceneNode(parent, mgr, pos, rot, scale)
			{}

			virtual void setParticleSize(const core::dimension2df& size = core::dimension2df(5.0f, 5.0f)) = 0;

			virtual void setParticlesAreGlobal(bool global) = 0;

			virtual void clearParticles() = 0;

			virtual void doParticleSystem(u32 time) = 0;

			virtual IParticleEmitter* getEmitter() = 0;

			virtual void setEmitter(IParticleEmitter *emitter) = 0;

			virtual void addAffector(IParticleAffector *affector) = 0;

			virtual const core::list< IParticleAffector* >& getAffectors() const = 0;

			virtual void removeAllAffectors() = 0;

			virtual IParticleBoxEmitter* createBoxEmitter(const core::aabbox3df& box = core::aabbox3df(-10, 28, -10, 10, 30, 10),
														  const core::vector3df& direction = core::vector3df(0.0f, 0.3f, 0.0f),
														  u32 minParticlesPerSecond = 5, u32 maxParticlesPerSecond = 10,
														  const video::SColor& minStartColor = video::SColor(0, 0, 0, 255),
														  const video::SColor& maxStartColor = video::SColor(255, 255, 255, 255),
														  u32 lifeTimeMin = 2000, u32 lifeTimeMax = 4000, s32 maxAngleDegrees = 0,
														  const core::dimension2df& minStartSize = core::dimension2df(5.0f, 5.0f),
														  const core::dimension2df& maxStartSize = core::dimension2df(5.0f, 5.0f)) = 0;
		};
	}
}

#endif /* IPARTICLESYSTEMSCENENODE_H_ */
