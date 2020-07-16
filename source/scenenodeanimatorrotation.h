/*
 * scenenodeanimatorrotation.h
 *
 *  Created on: Feb 15, 2013
 *      Author: mike
 */

#ifndef SCENENODEANIMATORROTATION_H_
#define SCENENODEANIMATORROTATION_H_

#include "iscenenode.h"

namespace irr
{
	namespace scene
	{
		class CSceneNodeAnimatorRotation : public ISceneNodeAnimator
		{
		public:
			CSceneNodeAnimatorRotation(u32 time, const core::vector3df& rotation);

			virtual void animateNode(ISceneNode *node, u32 timeMs);

			virtual ESCENE_NODE_ANIMATOR_TYPE getType() const { return ESNAT_ROTATION; }

		private:
			core::vector3df _rotation;
			u32 _startTime;
		};
	}
}
#endif /* SCENENODEANIMATORROTATION_H_ */
