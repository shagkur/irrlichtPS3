/*
 * scenenodeanimatorflycircle.cpp
 *
 *  Created on: Aug 14, 2013
 *      Author: mike
 */

#include "scenenodeanimatorflycircle.h"

namespace irr
{
	namespace scene
	{
		CSceneNodeAnimatorFlyCircle::CSceneNodeAnimatorFlyCircle(u32 time, const core::vector3df& center, f32 radius, f32 speed, const core::vector3df& direction, f32 radiusEllipsoid)
		: _center(center), _direction(direction), _radius(radius), _radiusEllipsoid(radiusEllipsoid), _speed(speed), _startTime(time)
		{
			init();
		}

		void CSceneNodeAnimatorFlyCircle::init()
		{
			_direction = normalize(_direction);

			if(_direction.getY() != 0.0f)
				_vecV = normalize(cross(core::vector3df(50.0f, 0.0f, 0.0f), _direction));
			else
				_vecV = normalize(cross(core::vector3df(0.0f, 50.0f, 0.0f), _direction));

			_vecU = normalize(cross(_vecV, _direction));
		}

		void CSceneNodeAnimatorFlyCircle::animateNode(ISceneNode *node, u32 timeMs)
		{
			if(!node)
				return;

			f32 time;

			if(_startTime > timeMs)
				time = ((s32)timeMs - _startTime)*_speed;
			else
				time = (timeMs - _startTime)*_speed;

			f32 r2 = _radiusEllipsoid == 0.0f ? _radius : _radiusEllipsoid;
			node->setPosition(_center + (_radius*cosf(time)*_vecU) + (r2*sinf(time)*_vecV));
		}
	}
}
