/*
 * imeshbuffer.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef IMESHBUFFER_H_
#define IMESHBUFFER_H_

#include "irefcounter.h"
#include "smaterial.h"
#include "aabbox3d.h"
#include "s3dvertex.h"
#include "ehardwarebufferflags.h"
#include "eprimitivetypes.h"
#include "eindextypes.h"

namespace irr
{
	namespace scene
	{
		class IMeshBuffer : public virtual IRefCounter
		{
		public:
			virtual video::SMaterial& getMaterial() = 0;
			virtual const video::SMaterial& getMaterial() const = 0;

			virtual video::E_VERTEX_TYPE getVertexType() const = 0;
			virtual const void* getVertices() const = 0;
			virtual void* getVertices() = 0;
			virtual u32 getVertexCount() const = 0;

			virtual const u16* getIndices() const = 0;
			virtual u16* getIndices() = 0;
			virtual u32 getIndexCount() const = 0;
			virtual video::E_INDEX_TYPE getIndexType() const = 0;

			virtual const core::aabbox3df& getBoundingBox() const = 0;
			virtual void setBoundingBox(const core::aabbox3df& bbox) = 0;
			virtual void recalculateBoundingBox() = 0;

			virtual const core::vector3df& getPosition(u32 i) const = 0;
			virtual core::vector3df& getPosition(u32 i) = 0;

			virtual const core::vector3df& getNormal(u32 i) const = 0;
			virtual core::vector3df& getNormal(u32 i) = 0;

			virtual const core::vector2df& getTCoords(u32 i) const = 0;
			virtual core::vector2df& getTCoords(u32 i) = 0;

			virtual E_HARDWARE_MAPPING getHardwareMappingHint_Vertex() const = 0;
			virtual E_HARDWARE_MAPPING getHardwareMappingHint_Index() const = 0;

			virtual void setHardwareMappingHint(E_HARDWARE_MAPPING newMappingHint, E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX) = 0;

			virtual void setDirty(E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX) = 0;

			virtual u32 getChangedID_Vertex() const = 0;
			virtual u32 getChangedID_Index() const = 0;
		};
	}
}

#endif /* IMESHBUFFER_H_ */
