/*
 * particlesystemscenenode.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mike
 */

#ifndef PARTICLESYSTEMSCENENODE_H_
#define PARTICLESYSTEMSCENENODE_H_

#include "iparticlesystemscenenode.h"
#include "irrarray.h"
#include "irrlist.h"
#include "smeshbuffer.h"

#include "particle/particles.h"

#define HEAP_SIZE_PCL			(15*1024*1024)

namespace irr
{
	namespace scene
	{
		class CParticleSystemSceneNode : public IParticleSystemSceneNode
		{
		public:
			CParticleSystemSceneNode(bool createDefaultEmitter, ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, const core::vector3df& rot, const core::vector3df& scale);
			virtual ~CParticleSystemSceneNode();

			virtual IParticleEmitter* getEmitter();

			virtual void setEmitter(IParticleEmitter *emitter);

			virtual void addAffector(IParticleAffector *affector);

			virtual const core::list< IParticleAffector* >& getAffectors() const;

			virtual void removeAllAffectors();

			virtual video::SMaterial& getMaterial(u32 i);

			virtual u32 getMaterialCount() const;

			virtual void onRegisterSceneNode();

			virtual void render();

			virtual const core::aabbox3df& getBoundingBox() const;

			virtual void setParticlesAreGlobal(bool global = true);

			virtual void doParticleSystem(u32 time);

			virtual IParticleBoxEmitter* createBoxEmitter(const core::aabbox3df& box = core::aabbox3df(-10, 28, -10, 10, 30, 10),
														  const core::vector3df& direction = core::vector3df(0.0f, 0.3f, 0.0f),
														  u32 minParticlesPerSecond = 5, u32 maxParticlesPerSecond = 10,
														  const video::SColor& minStartColor = video::SColor(0, 0, 0, 255),
														  const video::SColor& maxStartColor = video::SColor(255, 255, 255, 255),
														  u32 lifeTimeMin = 2000, u32 lifeTimeMax = 4000, s32 maxAngleDegrees = 0,
														  const core::dimension2df& minStartSize = core::dimension2df(5.0f, 5.0f),
														  const core::dimension2df& maxStartSize = core::dimension2df(5.0f, 5.0f));

			virtual ESCENE_NODE_TYPE getType() const { return ESNT_PARTICLE_SYSTEM; }

		private:
			void reallocateBuffers();

			core::list< IParticleAffector* > _affectorList;
			IParticleEmitter *_emitter;
			core::array< SParticle > _particles;
			core::dimension2df _particleSize;
			u32 _lastEmitTime;
			s32 _maxParticles;

			HeapManager _poolPCL;
			Particles *_particle;

			MARSTaskManager *_taskManager;

			SMeshBuffer *_buffer;

			enum E_PARTICLES_PRIMITIVE
			{
				EPP_POINT = 0,
				EPP_BILLBOARD,
				EPP_POINTSPRITE
			};

			E_PARTICLES_PRIMITIVE _particlePrimitive;

			bool _particlesAreGlobal;

			mars_context *_marsContext;
		};
	}
}

#endif /* PARTICLESYSTEMSCENENODE_H_ */
