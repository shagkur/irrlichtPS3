/*
 * particlesystemscenenode.cpp
 *
 *  Created on: Feb 26, 2013
 *      Author: mike
 */

#include "particlesystemscenenode.h"
#include "os.h"
#include "iscenemanager.h"
#include "icamerascenenode.h"
#include "ivideodriver.h"

#include "sviewfrustum.h"

#define NUM_MAX_SPU			5

static u8 memPoolPCL[HEAP_SIZE_PCL] __attribute__((aligned(128)));

namespace irr
{
	namespace scene
	{
		CParticleSystemSceneNode::CParticleSystemSceneNode(bool createDefaultEmitter, ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, const core::vector3df& rot, const core::vector3df& scale)
		: IParticleSystemSceneNode(parent, mgr, pos, rot, scale), _emitter(NULL), _particleSize(core::dimension2df(5.0f, 5.0f)), _lastEmitTime(0),
		  _maxParticles(0xffff), _poolPCL(memPoolPCL, HEAP_SIZE_PCL), _buffer(NULL), _particlesAreGlobal(true)
		{
			_buffer = new SMeshBuffer();
			_marsContext = _sceneManager->getVideoDriver()->getMARSContext();

			_taskManager = new MARSTaskManager(_marsContext, NUM_MAX_SPU);
			_taskManager->initialize();

			_particle = new Particles(_taskManager, 0, &_poolPCL);

			if(createDefaultEmitter) {

			}
		}

		CParticleSystemSceneNode::~CParticleSystemSceneNode()
		{
			if(_emitter != NULL) _emitter->drop();
			if(_buffer != NULL) _buffer->drop();

			_taskManager->finalize();

			removeAllAffectors();

			delete _taskManager;
		}

		IParticleEmitter* CParticleSystemSceneNode::getEmitter()
		{
			return _emitter;
		}

		void CParticleSystemSceneNode::setEmitter(IParticleEmitter *emitter)
		{
			if(emitter == _emitter) return;

			if(_emitter != NULL) _emitter->drop();

			_emitter = emitter;

			if(_emitter) _emitter->grab();
		}

		void CParticleSystemSceneNode::addAffector(IParticleAffector *affector)
		{
			affector->grab();
			_affectorList.push_back(affector);
		}

		const core::list< IParticleAffector* >& CParticleSystemSceneNode::getAffectors() const
		{
			return _affectorList;
		}

		void CParticleSystemSceneNode::removeAllAffectors()
		{
			core::list< IParticleAffector* >::Iterator it = _affectorList.begin();
			while(it != _affectorList.end()) {
				(*it)->drop();
				it = _affectorList.erase(it);
			}
		}

		video::SMaterial& CParticleSystemSceneNode::getMaterial(u32 i)
		{
			return _buffer->material;
		}

		u32 CParticleSystemSceneNode::getMaterialCount() const
		{
			return 1;
		}

		const core::aabbox3df& CParticleSystemSceneNode::getBoundingBox() const
		{
			return _buffer->bbox;
		}

		void CParticleSystemSceneNode::onRegisterSceneNode()
		{
			doParticleSystem(os::Timer::getTime());

			if(_isVisible && _particles.size() > 0) {
				_sceneManager->registerNodeForRendering(this);
				ISceneNode::onRegisterSceneNode();
			}
		}

		void CParticleSystemSceneNode::render()
		{
			video::IVideoDriver *driver = _sceneManager->getVideoDriver();
			ICameraSceneNode *camera = _sceneManager->getActiveCamera();

			if(camera == NULL || driver == NULL) return;

			const core::matrix4 mv = transpose(camera->getViewFrustum()->getTransform(video::ETS_VIEW));
			const core::vector3df view((vec_float4)vec_xor((vec_uint4)mv.getCol(2).get128(), ((vec_uint4){0x80000000,0x80000000,0,0})));

			reallocateBuffers();

			s32 idx = 0;
			for(u32 i=0;i < _particles.size();i++) {
				const SParticle& particle = _particles[i];
				floatInVec h(particle.size.width*0.5f);
				floatInVec v(particle.size.height*0.5f);

				_buffer->vertices[idx + 0].pos = core::vector3df(vec_add(particle.pos.get128(), (h*mv.getCol(0) + v*mv.getCol(1)).get128()));
				_buffer->vertices[idx + 0].col = particle.color;
				_buffer->vertices[idx + 0].nrm = view;

				_buffer->vertices[idx + 1].pos = core::vector3df(vec_add(particle.pos.get128(), (h*mv.getCol(0) - v*mv.getCol(1)).get128()));
				_buffer->vertices[idx + 1].col = particle.color;
				_buffer->vertices[idx + 1].nrm = view;

				_buffer->vertices[idx + 2].pos = core::vector3df(vec_sub(particle.pos.get128(), (h*mv.getCol(0) - v*mv.getCol(1)).get128()));
				_buffer->vertices[idx + 2].col = particle.color;
				_buffer->vertices[idx + 2].nrm = view;

				_buffer->vertices[idx + 3].pos = core::vector3df(vec_sub(particle.pos.get128(), (h*mv.getCol(0) + v*mv.getCol(1)).get128()));
				_buffer->vertices[idx + 3].col = particle.color;
				_buffer->vertices[idx + 3].nrm = view;

				idx += 4;
			}

			core::matrix4 mat;
			if(!_particlesAreGlobal) mat.setTranslation(_absTransformation.getTranslation());

			driver->setTransform(video::ETS_WORLD, mat);
			driver->setMaterial(_buffer->material);
			driver->drawVertexPrimitiveList(_buffer->getVertices(), _particles.size()*4, _buffer->getIndices(), _particles.size()*2, video::EVT_STANDARD, EPT_TRIANGLES);
		}

		void CParticleSystemSceneNode::doParticleSystem(u32 time)
		{
			if(_lastEmitTime == 0) {
				_lastEmitTime = time;
				return;
			}

			u32 now = time;
			u32 timeDiff = time - _lastEmitTime;

			_lastEmitTime = time;

			if(_emitter != NULL && _isVisible) {
				SParticle *array = NULL;
				s32 newParticles = _emitter->emitt(now, timeDiff, array);

				if(newParticles > 0 && array != NULL) {
					s32 j = _particles.size();

					if(newParticles > 16250 - j) newParticles = 16250 - j;

					_particles.set_used(j + newParticles);
					for(s32 i=j;i < j + newParticles;i++) {
						_particles[i] = array[i - j];

						core::rotateVect(_absTransformation, _particles[i].startVector);
						if(_particlesAreGlobal) core::transformVect(_absTransformation, _particles[i].pos);
					}
				}
			}

			core::list< IParticleAffector* >::Iterator it = _affectorList.begin();
			for(;it != _affectorList.end();it++) (*it)->affect(now, _particles.pointer(), _particles.size());

			if(_particlesAreGlobal)
				_buffer->bbox.reset(_absTransformation.getTranslation());
			else
				_buffer->bbox.reset(core::vector3df(0, 0, 0));

			f32 scale = (f32)timeDiff;
			for(u32 i=0;i < _particles.size();) {
				if(now > _particles[i].endTime) {
					_particles[i] = _particles[_particles.size() - 1];
					_particles.erase(_particles.size() - 1);
				} else {
					_particles[i].pos += (_particles[i].vector*scale);
					_buffer->bbox.addInternalPoint(_particles[i].pos);
					i++;
				}
			}

			const f32 m = (_particleSize.width > _particleSize.height ? _particleSize.width : _particleSize.height);
			_buffer->bbox.maxEdge.setX(_buffer->bbox.maxEdge.getX() + m);
			_buffer->bbox.maxEdge.setY(_buffer->bbox.maxEdge.getY() + m);
			_buffer->bbox.maxEdge.setZ(_buffer->bbox.maxEdge.getZ() + m);

			_buffer->bbox.minEdge.setX(_buffer->bbox.minEdge.getX() - m);
			_buffer->bbox.minEdge.setY(_buffer->bbox.minEdge.getY() - m);
			_buffer->bbox.minEdge.setZ(_buffer->bbox.minEdge.getZ() - m);

			if(_particlesAreGlobal) {
				core::matrix4 absinv = inverse(_absTransformation);
				core::transformBox(absinv, _buffer->bbox);
			}
		}

		void CParticleSystemSceneNode::reallocateBuffers()
		{
			if(_particles.size()*4 > _buffer->getVertexCount() ||
			   _particles.size()*6 > _buffer->getIndexCount())
			{
				u32 i, oldSize = _buffer->getVertexCount();

				_buffer->vertices.set_used(_particles.size()*4);

				for(i=0;i < _buffer->vertices.size();i++) {
					_buffer->vertices[i + 0].tcoords = core::vector2df(0.0f, 0.0f);
					_buffer->vertices[i + 1].tcoords = core::vector2df(0.0f, 1.0f);
					_buffer->vertices[i + 2].tcoords = core::vector2df(1.0f, 1.0f);
					_buffer->vertices[i + 3].tcoords = core::vector2df(1.0f, 0.0f);
				}

				u32 oldIdxSize = _buffer->getIndexCount();
				u32 oldvertices = oldSize;

				_buffer->indices.set_used(_particles.size()*6);

				for(i=oldIdxSize;i < _buffer->indices.size();i+=6) {
					_buffer->indices[i + 0] = (u16)(oldvertices + 0);
					_buffer->indices[i + 1] = (u16)(oldvertices + 2);
					_buffer->indices[i + 2] = (u16)(oldvertices + 1);
					_buffer->indices[i + 3] = (u16)(oldvertices + 0);
					_buffer->indices[i + 4] = (u16)(oldvertices + 3);
					_buffer->indices[i + 5] = (u16)(oldvertices + 2);
					oldvertices += 4;
				}
			}
		}
	}
}
