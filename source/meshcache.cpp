/*
 * meshcache.cpp
 *
 *  Created on: Feb 24, 2013
 *      Author: mike
 */

#include "meshcache.h"
#include "ianimatedmesh.h"
#include "imesh.h"

namespace irr
{
	namespace scene
	{
		//static const io::SNamedPath emptyNamedPath;

		CMeshCache::~CMeshCache()
		{
			clear();
		}

		void CMeshCache::addMesh(const io::path& filename, IAnimatedMesh *mesh)
		{
			mesh->grab();

			MeshEntry e(filename);
			e.mesh = mesh;

			_meshes.push_back(e);
		}

		void CMeshCache::removeMesh(const IMesh* const mesh)
		{
			if(mesh == NULL) return;

			for(u32 i=0;i < _meshes.size();i++) {
				if(_meshes[i].mesh == mesh ||(_meshes[i].mesh && _meshes[i].mesh->getMesh(0) == mesh)) {
					_meshes[i].mesh->drop();
					_meshes.erase(i);
					return;
				}
			}
		}

		u32 CMeshCache::getMeshCount() const
		{
			return _meshes.size();
		}

		s32 CMeshCache::getMeshIndex(const IMesh* const mesh) const
		{
			for(u32 i=0;i < _meshes.size();i++) {
				if(_meshes[i].mesh == mesh || (_meshes[i].mesh && _meshes[i].mesh->getMesh(0) == mesh))
					return (s32)i;
			}
			return -1;
		}

		IAnimatedMesh* CMeshCache::getMeshByIndex(u32 index)
		{
			if(index >= _meshes.size()) return NULL;
			return _meshes[index].mesh;
		}

		IAnimatedMesh* CMeshCache::getMeshByName(const io::path& name)
		{
			MeshEntry e(name);
			s32 idx = _meshes.binary_search(e);
			return (idx > -1 ? _meshes[idx].mesh : NULL);
		}

		bool CMeshCache::isMeshLoaded(const io::path& name)
		{
			return getMeshByName(name) != NULL;
		}

		void CMeshCache::clear()
		{
			for(u32 i=0;i < _meshes.size();i++)
				_meshes[i].mesh->drop();

			_meshes.clear();
		}

		void CMeshCache::clearUnusedMeshes()
		{
			for(u32 i=0;i < _meshes.size();i++) {
				if(_meshes[i].mesh->getRefCount() == 1) {
					_meshes[i].mesh->drop();
					_meshes.erase(i);
					--i;
				}
			}
		}
	}
}


