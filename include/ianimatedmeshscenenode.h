/*
 * ianimatedmeshscenenode.h
 *
 *  Created on: Feb 25, 2013
 *      Author: mike
 */

#ifndef IANIMATEDMESHSCENENODE_H_
#define IANIMATEDMESHSCENENODE_H_

#include "iscenenode.h"
#include "ianimatedmesh.h"

namespace irr
{
	namespace scene
	{
		class IAnimatedMeshSceneNode;

		class IAnimationEndCallback : public IRefCounter
		{
		public:
			virtual void onAnimationEnd(IAnimatedMeshSceneNode *node) = 0;
		};

		class IAnimatedMeshSceneNode : public ISceneNode
		{
		public:
			IAnimatedMeshSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos = core::vector3df(0, 0, 0), const core::vector3df& rot = core::vector3df(0, 0, 0), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f))
			: ISceneNode(parent, mgr, pos, rot, scale)
			{}

			virtual ~IAnimatedMeshSceneNode() {}

			virtual void setCurrentFrame(f32 frame) = 0;

			virtual bool setFrameLoop(s32 begin, s32 end) = 0;

			virtual f32 getAnimationSpeed() const = 0;

			virtual void setAnimationSpeed(f32 fps) = 0;

			virtual f32 getFrameNr() const = 0;

			virtual s32 getStartFrame() const = 0;

			virtual s32 getEndFrame() const = 0;

			virtual void setLoopMode(bool playAnimationLooped) = 0;

			virtual bool getLoopMode() const = 0;

			virtual void setAnimationEndCallback(IAnimationEndCallback *callback = NULL) = 0;

			virtual void setReadOnlyMaterials(bool readonly) = 0;

			virtual bool isReadOnlyMaterials() const = 0;

			virtual void setMesh(IAnimatedMesh *mesh) = 0;

			virtual IAnimatedMesh* getMesh() = 0;
		};
	}
}

#endif /* IANIMATEDMESHSCENENODE_H_ */
