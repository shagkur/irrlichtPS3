/*
 * lightscenenode.h
 *
 *  Created on: Feb 10, 2013
 *      Author: mike
 */

#ifndef LIGHTSCENENODE_H_
#define LIGHTSCENENODE_H_

#include "ilightscenenode.h"

namespace irr
{
	namespace scene
	{
		class CLightSceneNode : public ILightSceneNode
		{
		public:
			CLightSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, video::SColorf color, f32 range);
			virtual ~CLightSceneNode() {}

			virtual void onRegisterSceneNode();

			virtual void render();

			virtual void setLightData(const video::SLight& light);

			virtual const video::SLight& getLightData() const;

			virtual video::SLight& getLightData();

			virtual void setVisible(bool isVisible);

			virtual const core::aabbox3df& getBoundingBox() const;

			virtual ESCENE_NODE_TYPE getType() const { return ESNT_LIGHT; }

			virtual void setRadius(f32 radius);

			virtual f32 getRadius() const;

			virtual void setLightType(video::E_LIGHT_TYPE type);

			virtual video::E_LIGHT_TYPE getLightType() const;

			virtual void enableCastShadow(bool shadow = true);

			virtual bool getCastShadow() const;

		private:
			void doLightRecalc();

			video::SLight _lightData;
			core::aabbox3df _bbox;
			s32 _driverLightIndex;
			bool _lightIsOn;
		};
	}  // namespace scene
}

#endif /* LIGHTSCENENODE_H_ */
