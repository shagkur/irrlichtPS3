/*
 * hingejoint.h
 *
 *  Created on: May 30, 2013
 *      Author: mike
 */

#ifndef HINGEJOINT_H_
#define HINGEJOINT_H_

#include "itypedjoint.h"

namespace irr
{
	namespace scene
	{
		class IPhysicsManager;

		class CHingeJoint : public ITypedJoint
		{
		public:
			CHingeJoint(s32 jointIdx, IPhysicsManager *mgr);
			virtual ~CHingeJoint();

			virtual E_JOINT_TYPE getType() const { return EJT_HINGE; }

		private:
			s32 _jointIdx;
			IPhysicsManager *_physicsManager;
		};
	}
}

#endif /* HINGEJOINT_H_ */
