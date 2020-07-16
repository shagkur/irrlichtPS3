/*
 * irigidbody.h
 *
 *  Created on: May 15, 2013
 *      Author: mike
 */

#ifndef IRIGIDBODY_H_
#define IRIGIDBODY_H_

#include "irefcounter.h"
#include "vector3d.h"

namespace irr
{
	namespace scene
	{
		enum E_MOVE_TYPE
		{
			EMT_FIXED 		= 0,
			EMT_ACTIVE		= 1,
			EMT_KEYFRAME	= 3,
			EMT_ONEWAY		= 5,
			EMT_TRIGGER		= 7,

			EMT_COUNT 		= 0xffffffff
		};

		enum E_RIGIDBODY_TYPE
		{
			ERBT_BOX,
			ERBT_SPHERE,
			ERBT_CONE,
			ERBT_CONVEX_HULL,

			ERBT_UNKNOWN
		};

		class IRigidBody : public virtual IRefCounter
		{
		public:
			virtual void setMoveType(E_MOVE_TYPE moveType) = 0;

			virtual void setLinearVelocity(const core::vector3df& velocity) = 0;
			virtual void setAngularVelocity(const core::vector3df& velocity) = 0;

			virtual void applyLinearImpulse(const core::vector3df& impulse) = 0;
			virtual void applyAngularImpulse(const core::vector3df& impulse) = 0;

			virtual void setElasticity(f32 elasticity) = 0;

			virtual void setFriction(f32 friction) = 0;

			virtual void sleep() = 0;
			virtual void wakeup() = 0;
			virtual bool isASleep() const = 0;
			virtual bool isAWake() const = 0;

			virtual void updateScene() = 0;

			virtual int getBodyIndex() const = 0;

			virtual E_RIGIDBODY_TYPE getType() const { return ERBT_UNKNOWN; }
		};
	}
}

#endif /* IRIGIDBODY_H_ */
