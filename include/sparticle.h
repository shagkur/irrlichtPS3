/*
 * sparticle.h
 *
 *  Created on: Feb 26, 2013
 *      Author: mike
 */

#ifndef SPARTICLE_H_
#define SPARTICLE_H_

#include "vector3d.h"
#include "dimension2d.h"
#include "scolor.h"

namespace irr
{
	namespace scene
	{
		struct SParticle
		{
			core::vector3df pos;
			core::vector3df vector;

			u32 startTime;
			u32 endTime;

			video::SColor color;
			video::SColor startColor;

			core::vector3df startVector;

			core::dimension2df size;
			core::dimension2df startSize;
		};
	}
}

#endif /* SPARTICLE_H_ */
