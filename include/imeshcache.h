/*
 * imeshcache.h
 *
 *  Created on: Feb 17, 2013
 *      Author: mike
 */

#ifndef IMESHCACHE_H_
#define IMESHCACHE_H_

#include "irefcounter.h"
#include "path.h"

namespace irr
{
	namespace scene
	{
		class IMesh;
		class IAnimatedMesh;

		class IMeshCache : public virtual IRefCounter
		{
		public:
			virtual ~IMeshCache() {}

			virtual void addMesh(const io::path& name, IAnimatedMesh *mesh) = 0;

			virtual void removeMesh(const IMesh* const mesh) = 0;

			virtual u32 getMeshCount() const = 0;

			virtual s32 getMeshIndex(const IMesh* const mesh) const = 0;

			virtual IAnimatedMesh* getMeshByIndex(u32 index) = 0;

			virtual IAnimatedMesh* getMeshByName(const io::path& name) = 0;

			virtual bool isMeshLoaded(const io::path& name) = 0;

			virtual void clear() = 0;

			virtual void clearUnusedMeshes() = 0;
		};
	}
}

#endif /* IMESHCACHE_H_ */
