/*
 * imesh.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef IMESH_H_
#define IMESH_H_

#include "irefcounter.h"
#include "smaterial.h"
#include "ehardwarebufferflags.h"

namespace irr
{
	namespace scene
	{
		class IMeshBuffer;

		class IMesh : public virtual IRefCounter
		{
		public:
			virtual u32 getMeshBufferCount() const = 0;

			virtual IMeshBuffer* getMeshBuffer(u32 nr) const = 0;
			virtual IMeshBuffer* getMeshBuffer(const video::SMaterial& material) const = 0;

			virtual const core::aabbox3df& getBoundingBox() const = 0;
			virtual void setBoundingBox(const core::aabbox3df& box) = 0;

			virtual void setMaterialFlag(video::E_MATERIAL_FLAG flag, u32 newValue) = 0;

			virtual void setHardwareMappingHint(E_HARDWARE_MAPPING newMappingHint, E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX) = 0;

			virtual void setDirty(E_BUFFER_TYPE buffer = EBT_VERTEX_AND_INDEX) = 0;
		};
	}
}

#endif /* IMESH_H_ */
