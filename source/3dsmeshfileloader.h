/*
 * 3dsmeshfileloader.h
 *
 *  Created on: Feb 18, 2013
 *      Author: mike
 */

#ifndef _3DSMESHFILELOADER_H_
#define _3DSMESHFILELOADER_H_

#include "imeshloader.h"
#include "ifilesystem.h"
#include "iscenemanager.h"
#include "irrstring.h"
#include "smesh.h"
#include "matrix4.h"

namespace irr
{
	namespace scene
	{
		class C3DSMeshFileLoader : public IMeshLoader
		{
		public:
			C3DSMeshFileLoader(ISceneManager *mgr, io::IFileSystem *fs);
			virtual ~C3DSMeshFileLoader();

			virtual bool isLoadableFileExtension(const io::path& filename) const;

			virtual IAnimatedMesh* createMesh(io::IReadFile *file);

		private:
			struct ChunkHeader
			{
				u16 id;
				s32 length;
			} __attribute__((packed));

			struct ChunkData
			{
				ChunkData() : read(0) {}

				ChunkHeader header;
				s32 read;
			};

			struct SCurrentMaterial
			{
				void clear()
				{
					material = video::SMaterial();
					name = "";
					filename[0] = "";
					filename[1] = "";
					filename[2] = "";
					filename[3] = "";
					filename[4] = "";
					strength[0] = 0.0f;
					strength[1] = 0.0f;
					strength[2] = 0.0f;
					strength[3] = 0.0f;
					strength[4] = 0.0f;
				}

				video::SMaterial material;
				core::stringc name;
				core::stringc filename[5];
				f32 strength[5];
			};

			struct SMaterialGroup
			{
				SMaterialGroup() : faceCount(0), faces(NULL) {}
				SMaterialGroup(const SMaterialGroup& o)
				{
					*this = o;
				}
				~SMaterialGroup()
				{
					clear();
				}

				void clear()
				{
					delete [] faces;
					faces = NULL;
					faceCount = 0;
				}

				void operator = (const SMaterialGroup& o)
				{
					materialName = o.materialName;
					faceCount = o.faceCount;
					faces = new u16[faceCount];
					for(u32 i=0;i < faceCount;i++)
						faces[i] = o.faces[i];
				}

				core::stringc materialName;
				u16 faceCount;
				u16 *faces;
			};

			void cleanUp();
			void loadMaterials(io::IReadFile *file);
			void composeObject(io::IReadFile *file, const core::stringc& name);

			void readChunkData(io::IReadFile *file, ChunkData& data);
			void readVertices(io::IReadFile *file, ChunkData& data);
			void readIndices(io::IReadFile *file, ChunkData& data);
			void readTextureCoords(io::IReadFile *file, ChunkData& data);
			void readMaterialGroup(io::IReadFile *file, ChunkData& data);
			void readString(io::IReadFile *file, ChunkData& data, core::stringc& out);
			bool readChunk(io::IReadFile *file, ChunkData *parent);
			bool readObjectChunk(io::IReadFile *file, ChunkData *parent);
			bool readMaterialChunk(io::IReadFile *file, ChunkData *parent);
			bool readFrameChunk(io::IReadFile *file, ChunkData *parent);
			bool readColorChunk(io::IReadFile *file, ChunkData *chunk, video::SColor& out);
			bool readPercentageChunk(io::IReadFile *file, ChunkData *chunk, f32& percentage);

			scene::ISceneManager *_sceneManager;
			io::IFileSystem *_fileSystem;

			f32 *_vertices;
			u16 *_indices;
			u32 *_smoothingGroups;
			core::array< u16 > _tempIndices;
			f32 *_tcoords;
			u16 _countVertices;
			u16 _countFaces;
			u16 _countTCoords;
			core::array< SMaterialGroup > _materialGroups;

			SCurrentMaterial _currentMaterial;
			core::array< SCurrentMaterial > _materials;
			core::array< core::stringc > _meshBufferNames;
			core::matrix4 _transformationMatrix;

			SMesh *_mesh;
		};
	}
}

#endif /* 3DSMESHFILELOADER_H_ */
