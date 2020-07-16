/*
 * igeometrycreator.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef IGEOMETRYCREATOR_H_
#define IGEOMETRYCREATOR_H_

#include "irefcounter.h"
#include "imesh.h"

namespace irr
{
	namespace scene
	{
		class IGeometryCreator : public virtual IRefCounter
		{
		public:
			virtual IMesh* createCubeMesh(const core::vector3df& size = core::vector3df(5.0f, 5.0f, 5.0f)) const = 0;

			virtual IMesh* createSphereMesh(f32 radius = 5.0f, u32 polyCountX = 32, u32 polyCountY = 32) const = 0;

			virtual IMesh* createTorusMesh(f32 outerRadius = 5.0f, f32 innerRadius = 2.5f, u32 polyCountX = 32, u32 polyCountY = 32) const = 0;
		};
	}
}

#endif /* IGEOMETRYCREATOR_H_ */
