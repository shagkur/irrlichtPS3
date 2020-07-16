/*
 * ibillboardscenenode.h
 *
 *  Created on: Feb 2, 2014
 *      Author: mike
 */

#ifndef IBILLBOARDSCENENODE_H_
#define IBILLBOARDSCENENODE_H_

#include "iscenenode.h"

namespace irr
{
	namespace scene
	{
		class IBillboardSceneNode : public ISceneNode
		{
		public:
			IBillboardSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f))
			: ISceneNode(parent, mgr, pos) {}

			virtual void setSize(const core::dimension2df& size) = 0;
			virtual void setSize(const f32 height, f32 bottomEdgeWidth, f32 topEdgeWidth) = 0;

			virtual const core::dimension2df& getSize() const = 0;
			virtual void getSize(f32& height, f32& bottomEdgeWidth, f32& topEdgeWidth) const = 0;

			virtual void setColor(const video::SColor& overallColor) = 0;
			virtual void setColor(const video::SColor& topColor, const video::SColor& bottomColor) = 0;

			virtual void getColor(video::SColor& topColor, video::SColor& bottomColor) const = 0;
		};
	}
}

#endif /* IBILLBOARDSCENENODE_H_ */
