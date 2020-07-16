/*
 * scenenodeanimatorcamerafps.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: mike
 */


#include "scenenodeanimatorcamerafps.h"
#include "ivideodriver.h"
#include "iscenemanager.h"
#include "icamerascenenode.h"

namespace irr
{
	namespace scene
	{
		CSceneNodeAnimatorCameraFPS::CSceneNodeAnimatorCameraFPS(f32 rotateSpeed, f32 moveSpeed, f32 jumpSpeed, bool nonVerticalMovement, bool invertY)
		: _maxVerticalAngle(88.0f), _moveSpeed(moveSpeed), _rotateSpeed(rotateSpeed), _jumpSpeed(jumpSpeed),
		  _ydirection(invertY ? -1.0f : 1.0f), _lastAnimationTime(0), _noVerticalMovement(nonVerticalMovement),
		  _firstUpdate(true)
		{

		}

		CSceneNodeAnimatorCameraFPS::~CSceneNodeAnimatorCameraFPS()
		{

		}

		bool CSceneNodeAnimatorCameraFPS::onEvent(const SEvent& event)
		{
			if(event.eventType == EET_JOYPAD_INPUT_EVENT) {
				_analogStickL.X = event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_LSTICK_X];
				_analogStickL.Y = event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_LSTICK_Y];
				_analogStickL = _analogStickL.normalize();

				_analogStickR.X = event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_RSTICK_X];
				_analogStickR.Y = event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_RSTICK_Y];
				//_analogStickR = _analogStickR.normalize();

				//printf("CSceneNodeAnimatorCameraFPS::onEvent(L: %f/%f)\n", _analogStickL.X, _analogStickL.Y);
				//printf("CSceneNodeAnimatorCameraFPS::onEvent(R: %f/%f)\n", _analogStickR.X, _analogStickR.Y);
				return true;
			}
			return false;
		}

		void CSceneNodeAnimatorCameraFPS::animateNode(ISceneNode *node, u32 timeMs)
		{
			if(node == NULL || node->getType() != ESNT_CAMERA) return;

			ICameraSceneNode *camera = dynamic_cast<ICameraSceneNode*>(node);
			ISceneManager *smgr = camera->getSceneManager();

			if(_firstUpdate) {
				camera->updateAbsolutePosition();

				_lastAnimationTime = timeMs;
				_firstUpdate = false;
			}

			if(smgr != NULL && smgr->getActiveCamera() != camera) return;

			f32 timeDiff = (f32)(timeMs - _lastAnimationTime);
			_lastAnimationTime = timeMs;

			core::vector3df relativeRotation;
			core::vector3df pos = camera->getPosition();
			core::vector3df target = (camera->getTarget() - camera->getAbsolutePosition());

			//printf("target: %f/%f/%f\n", (f32)target.getX(), (f32)target.getY(), (f32)target.getZ());
			core::horizontalAngle(target, relativeRotation);
			//printf("relarot: %f/%f/%f\n", (f32)relativeRotation.getX(), (f32)relativeRotation.getY(), (f32)relativeRotation.getZ());

			if(_analogStickR != core::vector2df(0, 0)) {
				relativeRotation.setY(relativeRotation.getY() - (_analogStickR.X*_rotateSpeed));
				relativeRotation.setX(relativeRotation.getX() - (_analogStickR.Y*_rotateSpeed*_ydirection));

				if(relativeRotation.getX() > _maxVerticalAngle*2 &&
				   relativeRotation.getX() < 360.0f - _maxVerticalAngle)
				{
					relativeRotation.setX(360.0f - _maxVerticalAngle);
				}
				else if(relativeRotation.getX() > _maxVerticalAngle &&
						relativeRotation.getX() < 360.0f - _maxVerticalAngle)
				{
					relativeRotation.setX(_maxVerticalAngle);
				}
			}
			target = core::vector3df(0.0f, 0.0f, core::max_(1.0f, (f32)length(pos)));
			//printf("target: %f/%f/%f\n", (f32)target.getX(), (f32)target.getY(), (f32)target.getZ());

			core::vector3df moveDir = target;
			core::matrix4 mat = core::matrix4::rotationZYX(core::vector3df(relativeRotation.getX(), relativeRotation.getY(), 0.0f)*core::DEGTORAD);

			core::transformVect(mat, target);

			if(_noVerticalMovement) {
				mat = core::matrix4::rotationZYX(core::vector3df(0.0f, relativeRotation.getY(), 0.0f)*core::DEGTORAD);
				core::transformVect(mat, moveDir);
			} else
				moveDir = target;

			moveDir = normalize(moveDir);

			pos -= moveDir*timeDiff*_analogStickL.Y;

			core::vector3df strafeVect = target;

			strafeVect = cross(strafeVect, camera->getUpVector());
			if(_noVerticalMovement) strafeVect.setY(0.0f);

			strafeVect = normalize(strafeVect);

			pos += strafeVect*timeDiff*_analogStickL.X;

			//printf("campos: %f/%f/%f\n", (f32)pos.getX(), (f32)pos.getY(), (f32)pos.getZ());
			camera->setPosition(pos);

			target += pos;
			//printf("camtarget: %f/%f/%f\n", (f32)target.getX(), (f32)target.getY(), (f32)target.getZ());
			camera->setTarget(target);
		}

		void CSceneNodeAnimatorCameraFPS::setMoveSpeed(f32 moveSpeed)
		{
			_moveSpeed = moveSpeed;
		}

		f32 CSceneNodeAnimatorCameraFPS::getMoveSpeed() const
		{
			return _moveSpeed;
		}

		void CSceneNodeAnimatorCameraFPS::setRotationSpeed(f32 rotateSpeed)
		{
			_rotateSpeed = rotateSpeed;
		}

		f32 CSceneNodeAnimatorCameraFPS::getRotationSpeed() const
		{
			return _rotateSpeed;
		}
	}
}

