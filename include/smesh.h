/*
 * smesh.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef SMESH_H_
#define SMESH_H_

#include "imesh.h"
#include "imeshbuffer.h"
#include "irrarray.h"
#include "aabbox3d.h"

namespace irr
{
	namespace scene
	{
		struct SMesh : public IMesh
		{
		public:
			SMesh() {}
			virtual ~SMesh()
			{
				for(u32 i=0;i < meshBuffers.size();i++) meshBuffers[i]->drop();
			}

			virtual void clear()
			{
				for(u32 i=0;i < meshBuffers.size();i++) meshBuffers[i]->drop();

				meshBuffers.clear();
				bbox.reset(0.0f, 0.0f, 0.0f);
			}

			virtual u32 getMeshBufferCount() const
			{
				return meshBuffers.size();
			}

			virtual IMeshBuffer* getMeshBuffer(u32 nr) const
			{
				return meshBuffers[nr];
			}

			virtual IMeshBuffer* getMeshBuffer(const video::SMaterial& material) const
			{
				for(s32 i=(s32)meshBuffers.size() - 1;i >= 0;--i) {
					if(material == meshBuffers[i]->getMaterial())
						return meshBuffers[i];
				}
				return NULL;
			}

			virtual const core::aabbox3df& getBoundingBox() const
			{
				return bbox;
			}

			virtual void setBoundingBox(const core::aabbox3df& box)
			{
				bbox = box;
			}

			void recalculateBoundingBox()
			{
				if(meshBuffers.size()) {
					bbox = meshBuffers[0]->getBoundingBox();
					for(u32 i=1;i < meshBuffers.size();i++) bbox.addInternalBox(meshBuffers[i]->getBoundingBox());
				} else
					bbox.reset(0.0f, 0.0f, 0.0f);
			}

			void addMeshBuffer(IMeshBuffer *buffer)
			{
				if(buffer != NULL) {
					buffer->grab();
					meshBuffers.push_back(buffer);
				}
			}

			virtual void setMaterialFlag(video::E_MATERIAL_FLAG flag, u32 newValue)
			{
				for(u32 i=0;i < meshBuffers.size();i++)
					meshBuffers[i]->getMaterial().setFlag(flag, newValue);
			}

			virtual void setHardwareMappingHint(E_HARDWARE_MAPPING newMappingHint, E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX)
			{
				for(u32 i=0;i < meshBuffers.size();i++)
					meshBuffers[i]->setHardwareMappingHint(newMappingHint, buffer);
			}

			virtual void setDirty(E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX)
			{
				for(u32 i=0;i < meshBuffers.size();i++)
					meshBuffers[i]->setDirty(buffer);
			}

			core::array< IMeshBuffer* > meshBuffers;
			core::aabbox3df bbox;
		};
	}
}
#endif /* SMESH_H_ */
