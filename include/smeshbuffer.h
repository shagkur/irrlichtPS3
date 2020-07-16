/*
 * smeshbuffer.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef SMESHBUFFER_H_
#define SMESHBUFFER_H_

#include "imeshbuffer.h"
#include "irrarray.h"
#include "irrheapallocator.h"

namespace irr
{
	namespace scene
	{
		template< class T >
		class CMeshBuffer : public IMeshBuffer
		{
		public:
			CMeshBuffer() : changedID_Vertex(1), changedID_Index(1), mappingHint_Vertex(EHM_NEVER), mappingHint_Index(EHM_NEVER) {}

			virtual video::SMaterial& getMaterial()
			{
				return material;
			}

			virtual const video::SMaterial& getMaterial() const
			{
				return material;
			}

			virtual video::E_VERTEX_TYPE getVertexType() const
			{
				return T().getType();
			}

			virtual const void* getVertices() const
			{
				return vertices.const_pointer();
			}

			virtual void* getVertices()
			{
				return vertices.pointer();
			}

			virtual u32 getVertexCount() const
			{
				return vertices.size();
			}

			virtual const u16* getIndices() const
			{
				return indices.const_pointer();
			}

			virtual u16* getIndices()
			{
				return indices.pointer();
			}

			virtual u32 getIndexCount() const
			{
				return indices.size();
			}

			virtual video::E_INDEX_TYPE getIndexType() const
			{
				return video::EIT_16BIT;
			}

			virtual const core::aabbox3df& getBoundingBox() const
			{
				return bbox;
			}

			virtual void setBoundingBox(const core::aabbox3df& _bbox)
			{
				bbox = _bbox;
			}

			virtual void recalculateBoundingBox()
			{
				if(vertices.empty())
					bbox.reset(0.0f, 0.0f, 0.0f);
				else {
					bbox.reset(vertices[0].pos);
					for(u32 i=1;i < vertices.size();i++) bbox.addInternalPoint(vertices[i].pos);
				}
			}

			virtual const core::vector3df& getPosition(u32 i) const
			{
				return vertices[i].pos;
			}

			virtual core::vector3df& getPosition(u32 i)
			{
				return vertices[i].pos;
			}

			virtual const core::vector3df& getNormal(u32 i) const
			{
				return vertices[i].nrm;
			}

			virtual core::vector3df& getNormal(u32 i)
			{
				return vertices[i].nrm;
			}

			virtual const core::vector2df& getTCoords(u32 i) const
			{
				return vertices[i].tcoords;
			}

			virtual core::vector2df& getTCoords(u32 i)
			{
				return vertices[i].tcoords;
			}

			virtual E_HARDWARE_MAPPING getHardwareMappingHint_Vertex() const
			{
				return mappingHint_Vertex;
			}

			virtual E_HARDWARE_MAPPING getHardwareMappingHint_Index() const
			{
				return mappingHint_Index;
			}

			virtual void setHardwareMappingHint(E_HARDWARE_MAPPING newMappingHint, E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX)
			{
				if(buffer == EBT_VERTEX_AND_INDEX || buffer == EBT_VERTEX)
					mappingHint_Vertex = newMappingHint;
				if(buffer == EBT_VERTEX_AND_INDEX || buffer == EBT_INDEX)
					mappingHint_Index = newMappingHint;
			}

			virtual void setDirty(E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX)
			{
				if(buffer == EBT_VERTEX_AND_INDEX || buffer == EBT_VERTEX)
					changedID_Vertex++;
				if(buffer == EBT_VERTEX_AND_INDEX || buffer == EBT_INDEX)
					changedID_Index++;
			}

			virtual u32 getChangedID_Vertex() const
			{
				return changedID_Vertex;
			}

			virtual u32 getChangedID_Index() const
			{
				return changedID_Index;
			}

			u32 changedID_Vertex;
			u32 changedID_Index;

			E_HARDWARE_MAPPING mappingHint_Vertex;
			E_HARDWARE_MAPPING mappingHint_Index;

			video::SMaterial material;

			core::array< T, core::irrheapallocator< T > > vertices;
			core::array< u16, core::irrheapallocator< u16 > > indices;

			core::aabbox3df bbox;
		};

		typedef CMeshBuffer< video::S3DVertexStandard > SMeshBuffer;
		typedef CMeshBuffer< video::S3DVertexTangents> SMeshBufferTangents;
	}
}

#endif /* SMESHBUFFER_H_ */
