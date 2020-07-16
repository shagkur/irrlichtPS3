/*
 * scenenodeanimatorcamerafps.h
 *
 *  Created on: Feb 25, 2013
 *      Author: mike
 */

#ifndef SCENENODEANIMATORCAMERAFPS_H_
#define SCENENODEANIMATORCAMERAFPS_H_

#include "iscenenodeanimatorcamerafps.h"
#include "vector2d.h"
#include "position2d.h"
#include "irrarray.h"


namespace irr
{
	namespace scene
	{
		class CSceneNodeAnimatorCameraFPS : public ISceneNodeAnimatorCameraFPS
		{
		public:
			CSceneNodeAnimatorCameraFPS(f32 rotateSpeed = 25.0f, f32 moveSpeed = 0.5f, f32 jumpSpeed = 0.0f, bool noVerticalMovement = false, bool invertY = false);
			virtual ~CSceneNodeAnimatorCameraFPS();

			virtual void animateNode(ISceneNode *node, u32 timeMs);

			virtual bool onEvent(const SEvent& event);

			virtual f32 getMoveSpeed() const;
			virtual void setMoveSpeed(f32 moveSpeed);

			virtual f32 getRotationSpeed() const;
			virtual void setRotationSpeed(f32 rotateSpeed);

			virtual bool isEventReceiverEnabled() const { return true; }
			virtual ESCENE_NODE_ANIMATOR_TYPE getType() const { return ESNAT_CAMERA_FPS; }

		private:
			f32 _maxVerticalAngle;

			f32 _moveSpeed;
			f32 _rotateSpeed;
			f32 _jumpSpeed;

			f32 _ydirection;
			s32 _lastAnimationTime;

			bool _noVerticalMovement;
			bool _firstUpdate;

			core::vector2df _analogStickL;
			core::vector2df _analogStickR;
		};
	}
}
#endif /* SCENENODEANIMATORCAMERAFPS_H_ */
