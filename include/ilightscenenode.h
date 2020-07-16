/*
 * ilightscenenode.h
 *
 *  Created on: Feb 10, 2013
 *      Author: mike
 */

#ifndef ILIGHTSCENENODE_H_
#define ILIGHTSCENENODE_H_

#include "iscenenode.h"
#include "slight.h"

namespace irr
{
	namespace scene
	{
		class ILightSceneNode : public ISceneNode
		{
		public:
			ILightSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f))
			: ISceneNode(parent, mgr, pos) {}

			virtual void setLightData(const video::SLight& light) = 0;

			virtual const video::SLight& getLightData() const = 0;

			virtual video::SLight& getLightData() = 0;

			virtual void setVisible(bool isVisible) = 0;

			virtual void setRadius(f32 radius) = 0;

			virtual void setLightType(video::E_LIGHT_TYPE type) = 0;

			virtual video::E_LIGHT_TYPE getLightType() const = 0;

			virtual void enableCastShadow(bool shadow = true) = 0;

			virtual bool getCastShadow() const = 0;
		};
	}
}

#endif /* ILIGHTSCENENODE_H_ */
