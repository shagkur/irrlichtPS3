/*
 * scenenodeanimatorrotation.cpp
 *
 *  Created on: Feb 15, 2013
 *      Author: mike
 */

#include "scenenodeanimatorrotation.h"

namespace irr
{
	namespace scene
	{
		CSceneNodeAnimatorRotation::CSceneNodeAnimatorRotation(u32 time, const core::vector3df& rotation)
		: _rotation(rotation), _startTime(time)
		{

		}

		void CSceneNodeAnimatorRotation::animateNode(ISceneNode *node, u32 timeMs)
		{
			if(node != NULL) {
				const u32 diffTime = timeMs - _startTime;

				if(diffTime != 0) {
					core::vector3df rot = node->getRotation() + _rotation*(diffTime*0.1f);
					if(rot.getX() > 360.0f)
						rot.setX(fmodf(rot.getX(), 360.0f));
					if(rot.getY() > 360.0f)
						rot.setY(fmodf(rot.getY(), 360.0f));
					if(rot.getZ() > 360.0f)
						rot.setZ(fmodf(rot.getZ(), 360.0f));

					node->setRotation(rot);
					_startTime = timeMs;
				}
			}
		}
	}
}

