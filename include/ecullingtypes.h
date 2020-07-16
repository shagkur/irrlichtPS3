/*
 * ecullingtypes.h
 *
 *  Created on: Feb 11, 2013
 *      Author: mike
 */

#ifndef ECULLINGTYPES_H_
#define ECULLINGTYPES_H_

#include "irrtypes.h"

namespace irr
{
	namespace scene
	{
		enum E_CULLING_TYPE
		{
			EAC_OFF = 0,
			EAC_BOX = 1,
			EAC_FRUSTUM_BOX = 2,
			EAC_FRUSTUM_SPHERE = 4,
			EAC_OCC_QUERY = 8
		};
	}
}

#endif /* ECULLINGTYPES_H_ */
