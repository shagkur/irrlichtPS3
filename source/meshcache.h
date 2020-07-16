/*
 * meshcache.h
 *
 *  Created on: Feb 24, 2013
 *      Author: mike
 */

#ifndef MESHCACHE_H_
#define MESHCACHE_H_

#include "imeshcache.h"
#include "irrarray.h"

namespace irr
{
	namespace scene
	{
		class CMeshCache : public IMeshCache
		{
		public:
			virtual ~CMeshCache();

			virtual void addMesh(const io::path& filename, IAnimatedMesh *mesh);

			virtual void removeMesh(const IMesh* const mesh);

			virtual u32 getMeshCount() const;

			virtual s32 getMeshIndex(const IMesh* const mesh) const;

			virtual IAnimatedMesh* getMeshByIndex(u32 index);

			virtual IAnimatedMesh* getMeshByName(const io::path& name);

			virtual bool isMeshLoaded(const io::path& name);

			virtual void clear();

			virtual void clearUnusedMeshes();

		protected:
			struct MeshEntry
			{
				MeshEntry(const io::path& name) : namedPath(name) {}

				io::SNamedPath namedPath;
				IAnimatedMesh *mesh;

				bool operator < (const MeshEntry& other) const
				{
					return (namedPath < other.namedPath);
				}
			};

			core::array< MeshEntry > _meshes;
		};
	}
}


#endif /* MESHCACHE_H_ */
