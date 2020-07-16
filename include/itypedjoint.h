/*
 * itypedjoint.h
 *
 *  Created on: May 30, 2013
 *      Author: mike
 */

#ifndef ITYPEDJOINT_H_
#define ITYPEDJOINT_H_

#include "irefcounter.h"

namespace irr
{
	namespace scene
	{
		enum E_JOINT_TYPE
		{
			EJT_BALL,
			EJT_CHAIN,
			EJT_HINGE,
			EJT_SLIDER,
			EJT_FIXED,
			EJT_UNIVERSAL,

			EJT_UNKNOWN
		};

		class ITypedJoint : public virtual IRefCounter
		{
		public:
			virtual E_JOINT_TYPE getType() const { return EJT_UNKNOWN; }
		};
	}
}

#endif /* ITYPEDJOINT_H_ */
