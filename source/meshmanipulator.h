/*
 * meshmanipulator.h
 *
 *  Created on: Mar 1, 2013
 *      Author: mike
 */

#ifndef MESHMANIPULATOR_H_
#define MESHMANIPULATOR_H_

#include "imeshmanipulator.h"

namespace irr
{
	namespace scene
	{
		class CMeshManipulator : public IMeshManipulator
		{
		public:
			virtual void flipSurfaces(scene::IMesh *mesh) const;

			virtual void recalculateNormals(IMesh *mesh, bool smooth = false, bool angleWeighted = false) const;
			virtual void recalculateNormals(IMeshBuffer *buffer, bool smooth = false, bool angleWeighted = false) const;

			virtual void recalculateTangents(IMesh *mesh, bool recalculateNormals = false, bool smooth = false, bool angleWeighted = false) const;
			virtual void recalculateTangents(IMeshBuffer *buffer, bool recalculateNormals = false, bool smooth = false, bool angleWeighted = false) const;

			virtual void makePlanarTextureMapping(IMesh *mesh, f32 resolution = 0.001f) const;
			virtual void makePlanarTextureMapping(IMeshBuffer *buffer, f32 resolution = 0.001f) const;

			virtual IMesh* createMeshWithTangents(IMesh *mesh, bool recalculateNormals = false, bool smooth = false, bool angleWeighted = false, bool calculateTangents = true) const;

			virtual IMesh* createForsythOptimizedMesh(const IMesh *mesh) const;
		};
	}
}

#endif /* MESHMANIPULATOR_H_ */
