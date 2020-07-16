/*
 * ianimatedmesh.h
 *
 *  Created on: Feb 15, 2013
 *      Author: mike
 */

#ifndef IANIMATEDMESH_H_
#define IANIMATEDMESH_H_

#include "aabbox3d.h"
#include "imesh.h"

namespace irr
{
	namespace scene
	{
		enum E_ANIMATED_MESH_TYPE
		{
			EAMT_UNKNOWN = 0,

			EAMT_MD2,

			EAMT_MD3,

			EAMT_3DS,

			EAMT_SKINNED
		};

		class IAnimatedMesh : public IMesh
		{
		public:
			virtual u32 getFrameCount() const = 0;

			virtual f32 getAnimationSpeed() const = 0;

			virtual void setAnimationSpeed(f32 fps) = 0;

			virtual IMesh* getMesh(s32 frame, s32 detailLevel = 255, s32 startFrameLoop = -1, s32 endFrameLoop = -1) = 0;

			virtual E_ANIMATED_MESH_TYPE getMeshType() const
			{
				return EAMT_UNKNOWN;
			}
		};
	}
}

#endif /* IANIMATEDMESH_H_ */
