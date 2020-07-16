/*
 * geometrycreator.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef GEOMETRYCREATOR_H_
#define GEOMETRYCREATOR_H_

#include "igeometrycreator.h"
#include "smeshbuffer.h"

namespace irr
{
	namespace scene
	{
		class CGeometryCreator : public IGeometryCreator
		{
			void addToBuffer(const video::S3DVertexStandard& v, SMeshBuffer *buffer);

		public:
			virtual IMesh* createCubeMesh(const core::vector3df& size = core::vector3df(5.0f, 5.0f, 5.0f)) const;

			virtual IMesh* createSphereMesh(f32 radius = 5.0f, u32 polyCountX = 32, u32 polyCountY = 32) const;

			virtual IMesh* createTorusMesh(f32 outerRadius = 5.0f, f32 innerRadius = 2.5f, u32 polyCountX = 32, u32 polyCountY = 32) const;
		};
	}
}
#endif /* GEOMETRYCREATOR_H_ */
