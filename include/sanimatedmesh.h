/*
 * sanimatedmesh.h
 *
 *  Created on: Feb 15, 2013
 *      Author: mike
 */

#ifndef SANIMATEDMESH_H_
#define SANIMATEDMESH_H_

#include "ianimatedmesh.h"
#include "imesh.h"
#include "aabbox3d.h"
#include "irrarray.h"

namespace irr
{
	namespace scene
	{
		struct SAnimatedMesh : public IAnimatedMesh
		{
			SAnimatedMesh(scene::IMesh *mesh = NULL, scene::E_ANIMATED_MESH_TYPE _type = scene::EAMT_UNKNOWN)
			: IAnimatedMesh(), framesPerSecond(25.0f), type(_type)
			{
				addMesh(mesh);
				recalculateBoundingBox();
			}

			virtual ~SAnimatedMesh()
			{
				for(u32 i=0;i < meshes.size();i++)
					meshes[i]->drop();
			}

			virtual u32 getFrameCount() const
			{
				return meshes.size();
			}

			virtual f32 getAnimationSpeed() const
			{
				return framesPerSecond;
			}

			virtual void setAnimationSpeed(f32 fps)
			{
				framesPerSecond = fps;
			}

			virtual IMesh* getMesh(s32 frame, s32 detailLevel = 255, s32 startFrameLoop = -1, s32 endFrameLoop = -1)
			{
				if(meshes.empty())
					return NULL;

				return meshes[frame];
			}

			virtual E_ANIMATED_MESH_TYPE getMeshType() const
			{
				return type;
			}

			virtual u32 getMeshBufferCount() const
			{
				if(meshes.empty())
					return 0;

				return meshes[0]->getMeshBufferCount();
			}

			virtual IMeshBuffer* getMeshBuffer(u32 nr) const
			{
				if(meshes.empty())
					return NULL;

				return meshes[0]->getMeshBuffer(nr);
			}

			virtual IMeshBuffer* getMeshBuffer(const video::SMaterial& material) const
			{
				if(meshes.empty())
					return NULL;

				return meshes[0]->getMeshBuffer(material);
			}

			virtual void setMaterialFlag(video::E_MATERIAL_FLAG flag, u32 newValue)
			{
				for(u32 i=0;i < meshes.size();i++)
					meshes[i]->setMaterialFlag(flag, newValue);
			}

			virtual void setHardwareMappingHint(E_HARDWARE_MAPPING newMappingHint, E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX)
			{
				for(u32 i=0;i < meshes.size();i++)
					meshes[i]->setHardwareMappingHint(newMappingHint, buffer);
			}

			virtual void setDirty(E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX)
			{
				for(u32 i=0;i < meshes.size();i++)
					meshes[i]->setDirty(buffer);
			}

			virtual const core::aabbox3df& getBoundingBox() const
			{
				return bbox;
			}

			virtual void setBoundingBox(const core::aabbox3df& box)
			{
				bbox = box;
			}

			void addMesh(IMesh *mesh)
			{
				if(mesh != NULL) {
					mesh->grab();
					meshes.push_back(mesh);
				}
			}

			void recalculateBoundingBox()
			{
				bbox.reset(0, 0, 0);

				if(meshes.empty())
					return;

				bbox = meshes[0]->getBoundingBox();
				for(u32 i=1;i < meshes.size();i++)
					bbox.addInternalBox(meshes[i]->getBoundingBox());
			}

			core::array< IMesh* > meshes;

			core::aabbox3df bbox;

			f32 framesPerSecond;

			E_ANIMATED_MESH_TYPE type;
		};
	}
}

#endif /* SANIMATEDMESH_H_ */
