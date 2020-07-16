/*
 * iscenenodeanimatorcamerafps.h
 *
 *  Created on: Feb 25, 2013
 *      Author: mike
 */

#ifndef ISCENENODEANIMATORCAMERAFPS_H_
#define ISCENENODEANIMATORCAMERAFPS_H_

#include "iscenenodeanimator.h"
#include "ieventreceiver.h"
#include "irrarray.h"

namespace irr
{
	namespace scene
	{
		class ISceneNodeAnimatorCameraFPS : public ISceneNodeAnimator
		{
		public:
			virtual f32 getMoveSpeed() const = 0;

			virtual void setMoveSpeed(f32 moveSpeed) = 0;

			virtual f32 getRotationSpeed() const = 0;

			virtual void setRotationSpeed(f32 rotateSpeed) = 0;
		};
	}
}

#endif /* ISCENENODEANIMATORCAMERAFPS_H_ */
