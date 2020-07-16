/*
 * scenenodeanimatorflycircle.h
 *
 *  Created on: Aug 14, 2013
 *      Author: mike
 */

#ifndef SCENENODEANIMATORFLYCIRCLE_H_
#define SCENENODEANIMATORFLYCIRCLE_H_

#include "iscenenode.h"

namespace irr
{
	namespace scene
	{
		class CSceneNodeAnimatorFlyCircle : public ISceneNodeAnimator
		{
		public:
			CSceneNodeAnimatorFlyCircle(u32 time, const core::vector3df& center, f32 radius, f32 speed, const core::vector3df& direction, f32 radiusEllipsoid);

			virtual void animateNode(ISceneNode *node, u32 timeMs);

			virtual ESCENE_NODE_ANIMATOR_TYPE getType() const { return ESNAT_FLY_CIRCLE; }

		private:
			void init();

			core::vector3df _center;
			core::vector3df _direction;

			core::vector3df _vecU;
			core::vector3df _vecV;

			f32 _radius;
			f32 _radiusEllipsoid;
			f32 _speed;
			u32 _startTime;
		};
	}
}

#endif /* SCENENODEANIMATORFLYCIRCLE_H_ */
