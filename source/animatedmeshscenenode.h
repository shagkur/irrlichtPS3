/*
 * animatedmeshscenenode.h
 *
 *  Created on: Feb 25, 2013
 *      Author: mike
 */

#ifndef ANIMATEDMESHSCENENODE_H_
#define ANIMATEDMESHSCENENODE_H_

#include "ianimatedmeshscenenode.h"
#include "ianimatedmesh.h"
#include "matrix4.h"

namespace irr
{
	namespace scene
	{
		class CAnimatedMeshSceneNode : public IAnimatedMeshSceneNode
		{
		public:
			CAnimatedMeshSceneNode(IAnimatedMesh *mesh, ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos = core::vector3df(0, 0, 0), const core::vector3df& rot = core::vector3df(0, 0, 0), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f));
			virtual ~CAnimatedMeshSceneNode();

			virtual void setCurrentFrame(f32 frame);

			virtual void onRegisterSceneNode();

			virtual void onAnimate(u32 timeMs);

			virtual void render();

			virtual const core::aabbox3df& getBoundingBox() const;

			virtual bool setFrameLoop(s32 begin, s32 end);

			virtual f32 getAnimationSpeed() const;

			virtual void setAnimationSpeed(f32 fps);

			virtual f32 getFrameNr() const;

			virtual s32 getStartFrame() const;

			virtual s32 getEndFrame() const;

			virtual void setLoopMode(bool playAnimationLooped);

			virtual bool getLoopMode() const;

			virtual void setAnimationEndCallback(IAnimationEndCallback *callback = NULL);

			virtual void setReadOnlyMaterials(bool readonly);

			virtual bool isReadOnlyMaterials() const;

			virtual video::SMaterial& getMaterial(u32 i);

			virtual u32 getMaterialCount() const;

			virtual void setMesh(IAnimatedMesh *mesh);

			virtual IAnimatedMesh* getMesh() { return _mesh; }

			virtual ESCENE_NODE_TYPE getType() const { return ESNT_ANIMATED_MESH; }

		private:
			IMesh* getMeshForCurrentFrame();
			void buildFrameNr(u32 timeMs);

			core::array< video::SMaterial > _materials;
			core::aabbox3df _bbox;
			IAnimatedMesh *_mesh;

			s32 _startFrame;
			s32 _endFrame;
			f32 _framesPerSecond;
			f32 _currentFrameNr;

			u32 _lastTimeMs;

			bool _looping;
			bool _readOnlyMaterials;

			IAnimationEndCallback *_loopCallback;
		};
	}
}
#endif /* ANIMATEDMESHSCENENODE_H_ */
