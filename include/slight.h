/*
 * slight.h
 *
 *  Created on: Feb 10, 2013
 *      Author: mike
 */

#ifndef SLIGHT_H_
#define SLIGHT_H_

#include "scolor.h"

namespace irr
{
	namespace  video
	{
		enum E_LIGHT_TYPE
		{
			ELT_POINT,

			ELT_SPOT,

			ELT_DIRECTIONAL,

			ELT_COUNT
		};

		struct SLight
		{
			SLight()
			: ambientColor(0.0f, 0.0f, 0.0f, 0.0f), diffuseColor(1.0f, 1.0f, 1.0f, 1.0f), specularColor(1.0f, 1.0f, 1.0f, 1.0f),
			  attenuation(1.0f, 1.0f, 1.0f), outerCone(45.0f), innerCone(0.0f), fallOff(2.0f),
			  position(0.0f, 0.0f, 0.0f), direction(0.0f, 0.0f, 1.0f), radius(100.0f), type(ELT_POINT), castShadows(true)
			{}

			SColorf ambientColor;
			SColorf diffuseColor;
			SColorf specularColor;

			core::vector3df attenuation;

			f32 outerCone;
			f32 innerCone;
			f32 fallOff;

			core::vector3df position;
			core::vector3df direction;

			f32 radius;

			E_LIGHT_TYPE type;

			bool castShadows : 1;
		};
	}  // namespace  video
}

#endif /* SLIGHT_H_ */
