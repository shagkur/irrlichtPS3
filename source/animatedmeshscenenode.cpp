/*
 * animatedmeshscenenode.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: mike
 */

#include "animatedmeshscenenode.h"
#include "ivideodriver.h"
#include "iscenemanager.h"
#include "s3dvertex.h"
#include "os.h"
#include "imaterialrenderer.h"
#include "imesh.h"
#include "imeshcache.h"
#include "ianimatedmesh.h"

namespace irr
{
	namespace scene
	{
		CAnimatedMeshSceneNode::CAnimatedMeshSceneNode(IAnimatedMesh *mesh, ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, const core::vector3df& rot, const core::vector3df& scale)
		: IAnimatedMeshSceneNode(parent, mgr, pos, rot, scale), _mesh(NULL), _startFrame(0), _endFrame(0), _framesPerSecond(0.025f),
		  _currentFrameNr(0.0f), _lastTimeMs(0), _looping(true), _readOnlyMaterials(false), _loopCallback(NULL)
		{
			setMesh(mesh);
		}

		CAnimatedMeshSceneNode::~CAnimatedMeshSceneNode()
		{
			if(_mesh != NULL) _mesh->drop();
			if(_loopCallback != NULL) _loopCallback->drop();
		}

		void CAnimatedMeshSceneNode::setCurrentFrame(f32 frame)
		{
			_currentFrameNr = core::clamp(frame, (f32)_startFrame, (f32)_endFrame);
		}

		f32 CAnimatedMeshSceneNode::getFrameNr() const
		{
			return _currentFrameNr;
		}

		void CAnimatedMeshSceneNode::buildFrameNr(u32 timeMs)
		{
			if(_startFrame == _endFrame)
				_currentFrameNr = (f32)_startFrame;
			else if(_looping) {
				_currentFrameNr += timeMs*_framesPerSecond;

				if(_framesPerSecond > 0.0f) {
					if(_currentFrameNr > _endFrame)
						_currentFrameNr = _startFrame + fmodf(_currentFrameNr - _startFrame, (f32)(_endFrame - _startFrame));
				} else {
					if(_currentFrameNr < _startFrame)
						_currentFrameNr = _endFrame - fmodf(_endFrame - _currentFrameNr, (f32)(_endFrame - _startFrame));
				}
			} else {
				_currentFrameNr += timeMs*_framesPerSecond;

				if(_framesPerSecond > 0.0f) {
					if(_currentFrameNr > (f32)_endFrame) {
						_currentFrameNr = (f32)_endFrame;
						if(_loopCallback) _loopCallback->onAnimationEnd(this);
					}
				} else {
					if(_currentFrameNr < (f32)_startFrame) {
						_currentFrameNr = (f32)_startFrame;
						if(_loopCallback) _loopCallback->onAnimationEnd(this);
					}
				}
			}
		}

		void CAnimatedMeshSceneNode::onRegisterSceneNode()
		{
			if(_isVisible) {
				s32 solidCount = 0;
				s32 transparentCount = 0;
				video::IVideoDriver *driver = _sceneManager->getVideoDriver();

				for(u32 i=0;i < _materials.size();i++) {
					video::IMaterialRenderer *rnd = driver->getMaterialRenderer(_materials[i].materialType);

					if(rnd != NULL && rnd->isTransparent())
						transparentCount++;
					else
						solidCount++;

					if(solidCount && transparentCount)
						break;
				}

				if(solidCount)
					_sceneManager->registerNodeForRendering(this, scene::ESNRP_SOLID);
				if(transparentCount)
					_sceneManager->registerNodeForRendering(this, scene::ESNRP_TRANSPARENT);

				ISceneNode::onRegisterSceneNode();
			}
		}

		IMesh* CAnimatedMeshSceneNode::getMeshForCurrentFrame()
		{
			if(_mesh->getMeshType() != EAMT_SKINNED) {
				s32 frameNr = (s32)getFrameNr();
				s32 frameBlend = (s32)(core::fract(getFrameNr())*1000.0f);
				return _mesh->getMesh(frameNr, frameBlend, _startFrame, _endFrame);
			}
			return NULL;
		}

		void CAnimatedMeshSceneNode::onAnimate(u32 timeMs)
		{
			if(_lastTimeMs == 0) _lastTimeMs = timeMs;

			buildFrameNr(timeMs - _lastTimeMs);
			if(_mesh != NULL) {
				scene::IMesh *mesh = getMeshForCurrentFrame();
				if(mesh != NULL) _bbox = mesh->getBoundingBox();
			}
			_lastTimeMs = timeMs;

			IAnimatedMeshSceneNode::onAnimate(timeMs);
		}

		void CAnimatedMeshSceneNode::render()
		{
			video::IVideoDriver *driver = _sceneManager->getVideoDriver();

			if(_mesh == NULL || driver == NULL) return;

			scene::IMesh *mesh = getMeshForCurrentFrame();
			bool isTransparentPass = _sceneManager->getSceneNodeRenderPass() == scene::ESNRP_TRANSPARENT;

			if(mesh != NULL)
				_bbox = mesh->getBoundingBox();

			driver->setTransform(video::ETS_WORLD, _absTransformation);
			for(u32 i=0;i < mesh->getMeshBufferCount();i++) {
				video::IMaterialRenderer *rnd = driver->getMaterialRenderer(_materials[i].materialType);
				bool transparent = (rnd != NULL && rnd->isTransparent());

				if(transparent == isTransparentPass) {
					scene::IMeshBuffer *mb = mesh->getMeshBuffer(i);
					const video::SMaterial& material = _readOnlyMaterials ? mb->getMaterial() : _materials[i];

					driver->setMaterial(material);
					driver->drawMeshBuffer(mb);
				}
			}
		}

		s32 CAnimatedMeshSceneNode::getStartFrame() const
		{
			return _startFrame;
		}

		s32 CAnimatedMeshSceneNode::getEndFrame() const
		{
			return _endFrame;
		}

		bool CAnimatedMeshSceneNode::setFrameLoop(s32 begin, s32 end)
		{
			const s32 maxFrameCount = _mesh->getFrameCount() - 1;

			if(end < begin) {
				_startFrame = core::s32_clamp(end, 0, maxFrameCount);
				_endFrame = core::s32_clamp(begin, _startFrame, maxFrameCount);
			} else {
				_startFrame = core::s32_clamp(begin, 0, maxFrameCount);
				_endFrame = core::s32_clamp(end, _startFrame, maxFrameCount);
			}

			if(_framesPerSecond < 0.0f)
				setCurrentFrame((f32)_endFrame);
			else
				setCurrentFrame((f32)_startFrame);

			return true;
		}

		void CAnimatedMeshSceneNode::setAnimationSpeed(f32 fps)
		{
			_framesPerSecond = fps*0.001f;
		}

		f32 CAnimatedMeshSceneNode::getAnimationSpeed() const
		{
			return _framesPerSecond*1000.0f;
		}

		const core::aabbox3df& CAnimatedMeshSceneNode::getBoundingBox() const
		{
			return _bbox;
		}

		video::SMaterial& CAnimatedMeshSceneNode::getMaterial(u32 i)
		{
			if(i >= _materials.size())
				return ISceneNode::getMaterial(i);

			return _materials[i];
		}

		u32 CAnimatedMeshSceneNode::getMaterialCount() const
		{
			return _materials.size();
		}

		void CAnimatedMeshSceneNode::setLoopMode(bool playAnimationLooped)
		{
			_looping = playAnimationLooped;
		}

		bool CAnimatedMeshSceneNode::getLoopMode() const
		{
			return _looping;
		}

		void CAnimatedMeshSceneNode::setAnimationEndCallback(IAnimationEndCallback *callback)
		{
			if(callback == _loopCallback) return;

			if(_loopCallback != NULL) _loopCallback->drop();

			_loopCallback = callback;

			if(_loopCallback != NULL) _loopCallback->grab();
		}

		void CAnimatedMeshSceneNode::setReadOnlyMaterials(bool readonly)
		{
			_readOnlyMaterials = readonly;
		}

		bool CAnimatedMeshSceneNode::isReadOnlyMaterials() const
		{
			return _readOnlyMaterials;
		}

		void CAnimatedMeshSceneNode::setMesh(IAnimatedMesh *mesh)
		{
			if(mesh == NULL) return;

			if(_mesh != mesh) {
				if(_mesh != NULL) _mesh->drop();

				_mesh = mesh;

				if(_mesh != NULL) _mesh->grab();
			}

			_bbox = _mesh->getBoundingBox();

			IMesh *m = _mesh->getMesh(0, 0);
			if(m != NULL) {
				_materials.clear();
				_materials.reallocate(m->getMeshBufferCount());

				for(u32 i=0;i < m->getMeshBufferCount();i++) {
					IMeshBuffer *mb = m->getMeshBuffer(i);

					if(mb != NULL)
						_materials.push_back(mb->getMaterial());
					else
						_materials.push_back(video::SMaterial());
				}
			}

			setFrameLoop(0, _mesh->getFrameCount());
		}
	}
}


