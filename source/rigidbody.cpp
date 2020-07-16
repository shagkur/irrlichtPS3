/*
 * rigidbody.cpp
 *
 *  Created on: May 15, 2013
 *      Author: mike
 */

#include "rigidbody.h"
#include "iphysicsmanager.h"
#include "quaternion.h"

#include "rigidbody/rigidbodies.h"
#include "rigidbody/mass.h"

namespace irr
{
	namespace scene
	{
		CRigidBody::CRigidBody(E_RIGIDBODY_TYPE type, s32 instance, ISceneNode *node, IPhysicsManager *physicsManager)
		: _instance(instance), _sceneNode(node), _physicsManager(physicsManager), _bodyType(type)
		{
			if(_sceneNode) _sceneNode->grab();
			if(_physicsManager) _physicsManager->grab();

			init();
		}

		CRigidBody::~CRigidBody()
		{
			if(_physicsManager) _physicsManager->drop();
			if(_sceneNode) _sceneNode->drop();
		}

		void CRigidBody::setMoveType(E_MOVE_TYPE moveType)
		{
			_physicsManager->getRigidBodies()->setMoveType(_instance, moveType);
		}

		void CRigidBody::setElasticity(f32 elasticity)
		{
			_physicsManager->getRigidBodies()->getTrbDynBody(_instance)->setElasticity(elasticity);
		}

		void CRigidBody::setFriction(f32 friction)
		{
			_physicsManager->getRigidBodies()->getTrbDynBody(_instance)->setFriction(friction);
		}

		void CRigidBody::setLinearVelocity(const core::vector3df& velocity)
		{
			_physicsManager->getRigidBodies()->setVelocity(_instance, velocity);
		}

		void CRigidBody::setAngularVelocity(const core::vector3df& velocity)
		{
			_physicsManager->getRigidBodies()->setAngularVelocity(_instance, velocity);
		}

		void CRigidBody::applyLinearImpulse(const core::vector3df& impulse)
		{
			_physicsManager->getRigidBodies()->applyLinearImpulse(_instance, impulse);
		}

		void CRigidBody::applyAngularImpulse(const core::vector3df& impulse)
		{
			_physicsManager->getRigidBodies()->applyAngularImpulse(_instance, impulse);
		}

		void CRigidBody::sleep()
		{
			_physicsManager->getRigidBodies()->sleep(_instance);
		}

		bool CRigidBody::isASleep() const
		{
			return _physicsManager->getRigidBodies()->isAsleep(_instance);
		}

		bool CRigidBody::isAWake() const
		{
			return _physicsManager->getRigidBodies()->isAwake(_instance);
		}

		void CRigidBody::wakeup()
		{
			_physicsManager->getRigidBodies()->wakeup(_instance);
		}

		void CRigidBody::init()
		{
			_physicsManager->getRigidBodies()->setupRigidBody(_instance);
		}

		void CRigidBody::updateScene()
		{
			RigidBodies *bodies = _physicsManager->getRigidBodies();

			if(bodies->isAwake(_instance)) {
				core::vector3df rot;
				const core::quaternion& q = bodies->getOrientation(_instance);

				core::toEulerRadians(q, rot);

				_sceneNode->setRotation(rot*core::RADTODEG);
				_sceneNode->setPosition(bodies->getPosition(_instance));
			}
		}

		int CRigidBody::getBodyIndex() const
		{
			return _instance;
		}
	}
}
