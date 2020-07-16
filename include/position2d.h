/*
 * position2d.h
 *
 *  Created on: Feb 21, 2013
 *      Author: mike
 */

#ifndef POSITION2D_H_
#define POSITION2D_H_

#include "vector2d.h"

namespace irr
{
	namespace core
	{
		typedef vector2d<f32> position2df;
		typedef vector2d<s32> position2di;
	}
}

#define position2d vector2d

#endif /* POSITION2D_H_ */
