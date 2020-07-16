/*
 * hingejoint.cpp
 *
 *  Created on: May 30, 2013
 *      Author: mike
 */

#include "hingejoint.h"
#include "iphysicsmanager.h"

namespace irr
{
	namespace scene
	{
		CHingeJoint::CHingeJoint(s32 jointIdx, IPhysicsManager *mgr)
		: _jointIdx(jointIdx), _physicsManager(mgr)
		{
			if(_physicsManager) _physicsManager->grab();
		}

		CHingeJoint::~CHingeJoint()
		{
			if(_physicsManager) _physicsManager->drop();
		}
	}
}

