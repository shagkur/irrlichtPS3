/*
 * rigidbody.h
 *
 *  Created on: May 15, 2013
 *      Author: mike
 */

#ifndef RIGIDBODY_H_
#define RIGIDBODY_H_

#include "irigidbody.h"

namespace irr
{
	namespace scene
	{
		class ISceneNode;
		class IPhysicsManager;

		class CRigidBody : public IRigidBody
		{
		public:
			CRigidBody(E_RIGIDBODY_TYPE type, s32 instance, ISceneNode *node, IPhysicsManager *physicsManager);
			virtual ~CRigidBody();

			virtual void setMoveType(E_MOVE_TYPE moveType);

			virtual void setElasticity(f32 elasticity);
			virtual void setFriction(f32 friction);

			virtual void setLinearVelocity(const core::vector3df& velocity);
			virtual void setAngularVelocity(const core::vector3df& velocity);

			virtual void applyLinearImpulse(const core::vector3df& impulse);
			virtual void applyAngularImpulse(const core::vector3df& impulse);

			virtual void sleep();
			virtual void wakeup();
			virtual bool isASleep() const;
			virtual bool isAWake() const;

			virtual void updateScene();

			virtual int getBodyIndex() const;

			virtual E_RIGIDBODY_TYPE getType() const { return _bodyType; }

		private:
			void init();

			s32 _instance;
			ISceneNode *_sceneNode;
			IPhysicsManager *_physicsManager;

			E_RIGIDBODY_TYPE _bodyType;
		};
	}
}

#endif /* RIGIDBODY_H_ */
