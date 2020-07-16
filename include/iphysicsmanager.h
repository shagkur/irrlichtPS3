/*
 * iphysicsmanager.h
 *
 *  Created on: May 15, 2013
 *      Author: mike
 */

#ifndef IPHYSICSMANAGER_H_
#define IPHYSICSMANAGER_H_

#include "irefcounter.h"
#include "vector3d.h"
#include "iscenenode.h"

class RigidBodies;

namespace irr
{
	namespace scene
	{
		enum E_BODY_ORIENT
		{
			EBO_AXIS_X = 0,
			EBO_AXIS_Y,
			EBO_AXIS_Z,
			EBO_AXIS_AUTO,
		};

		class IMeshSceneNode;
		class IAnimatedMeshSceneNode;
		class IRigidBody;
		class ITypedJoint;

		class IPhysicsManager : public virtual IRefCounter
		{
		public:
			virtual void setWorldSize(const core::vector3df& center, const core::vector3df& extent) = 0;

			virtual void stepSimulation(f32 timeStep, s32 maxSubSteps = 1, f32 fixedTimeStep = 1.0f/60.0f) = 0;

			virtual IRigidBody* createRigidBodyBox(ISceneNode *node, f32 mass) = 0;

			virtual IRigidBody* createRigidBodySphere(ISceneNode *node, f32 mass) = 0;

			virtual IRigidBody* createRigidBodyCone(ISceneNode *node, f32 mass, E_BODY_ORIENT orientation = EBO_AXIS_AUTO) = 0;

			virtual IRigidBody* createRigidBodyConvexHull(IMeshSceneNode *node, f32 mass) = 0;

			virtual IRigidBody* createRigidBodyConvexHull(IAnimatedMeshSceneNode *node, f32 mass) = 0;

			//virtual ITypedJoint* createHingeJoint(IRigidBody *bodyA, IRigidBody *bodyB, const core::vector3df& pivotAnchor, const core::vector3df& rotAxis) = 0;

			virtual RigidBodies* getRigidBodies() const = 0;
		};
	}
}

#endif /* IPHYSICSMANAGER_H_ */
