/*
 * imeshmanipulator.h
 *
 *  Created on: Mar 1, 2013
 *      Author: mike
 */

#ifndef IMESHMANIPULATOR_H_
#define IMESHMANIPULATOR_H_

#include "irefcounter.h"
#include "vector3d.h"
#include "aabbox3d.h"
#include "matrix4.h"
#include "ianimatedmesh.h"
#include "imeshbuffer.h"

namespace irr
{
	namespace scene
	{
		struct SMesh;

		class IMeshManipulator : public virtual IRefCounter
		{
		public:
			virtual void flipSurfaces(IMesh *mesh) const = 0;

			virtual void recalculateNormals(IMesh *mesh, bool smooth = false, bool angleWeighted = false) const = 0;
			virtual void recalculateNormals(IMeshBuffer *buffer, bool smooth = false, bool angleWeighted = false) const = 0;

			virtual void recalculateTangents(IMesh *mesh, bool recalculateNormals = false, bool smooth = false, bool angleWeighted = false) const = 0;
			virtual void recalculateTangents(IMeshBuffer *buffer, bool recalculateNormals = false, bool smooth = false, bool angleWeighted = false) const = 0;

			virtual void makePlanarTextureMapping(IMesh *mesh, f32 resolution = 0.001f) const = 0;
			virtual void makePlanarTextureMapping(IMeshBuffer *buffer, f32 resolution = 0.001f) const = 0;

			virtual IMesh* createMeshWithTangents(IMesh *mesh, bool recalculateNormals = false, bool smooth = false, bool angleWeighted = false, bool calculateTangents = true) const = 0;

			virtual IMesh* createForsythOptimizedMesh(const IMesh *mesh) const = 0;
		};
	}
}

#endif /* IMESHMANIPULATOR_H_ */
