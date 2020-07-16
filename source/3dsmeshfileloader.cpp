/*
 * 3dsmeshfileloader.cpp
 *
 *  Created on: Feb 18, 2013
 *      Author: mike
 */

#include "3dsmeshfileloader.h"
#include "os.h"
#include "smeshbuffer.h"
#include "sanimatedmesh.h"
#include "ireadfile.h"
#include "ivideodriver.h"

namespace irr
{
	namespace scene
	{
		enum e3DSChunk
		{
			// Primary chunk
			C3DS_MAIN3DS = 0x4D4D,

			// Main Chunks
			C3DS_EDIT3DS = 0x3D3D,
			C3DS_KEYF3DS = 0xB000,
			C3DS_VERSION = 0x0002,
			C3DS_MESHVERSION = 0x3D3E,

			// sub chunks of C3DS_EDIT3DS
			C3DS_EDIT_MATERIAL = 0xAFFF,
			C3DS_EDIT_OBJECT   = 0x4000,

			// sub chunks of C3DS_EDIT_MATERIAL
			C3DS_MATNAME       = 0xA000,
			C3DS_MATAMBIENT    = 0xA010,
			C3DS_MATDIFFUSE    = 0xA020,
			C3DS_MATSPECULAR   = 0xA030,
			C3DS_MATSHININESS  = 0xA040,
			C3DS_MATSHIN2PCT   = 0xA041,
			C3DS_TRANSPARENCY  = 0xA050,
			C3DS_TRANSPARENCY_FALLOFF  = 0xA052,
			C3DS_REFL_BLUR     = 0xA053,
			C3DS_TWO_SIDE      = 0xA081,
			C3DS_WIRE          = 0xA085,
			C3DS_SHADING       = 0xA100,
			C3DS_MATTEXMAP     = 0xA200,
			C3DS_MATSPECMAP    = 0xA204,
			C3DS_MATOPACMAP    = 0xA210,
			C3DS_MATREFLMAP    = 0xA220,
			C3DS_MATBUMPMAP    = 0xA230,
			C3DS_MATMAPFILE    = 0xA300,
			C3DS_MAT_TEXTILING = 0xA351,
			C3DS_MAT_USCALE    = 0xA354,
			C3DS_MAT_VSCALE    = 0xA356,
			C3DS_MAT_UOFFSET   = 0xA358,
			C3DS_MAT_VOFFSET   = 0xA35A,

			// subs of C3DS_EDIT_OBJECT
			C3DS_OBJTRIMESH    = 0x4100,

			// subs of C3DS_OBJTRIMESH
			C3DS_TRIVERT       = 0x4110,
			C3DS_POINTFLAGARRAY= 0x4111,
			C3DS_TRIFACE       = 0x4120,
			C3DS_TRIFACEMAT    = 0x4130,
			C3DS_TRIUV         = 0x4140,
			C3DS_TRISMOOTH     = 0x4150,
			C3DS_TRIMATRIX     = 0x4160,
			C3DS_MESHCOLOR     = 0x4165,
			C3DS_DIRECT_LIGHT  = 0x4600,
			C3DS_DL_INNER_RANGE= 0x4659,
			C3DS_DL_OUTER_RANGE= 0x465A,
			C3DS_DL_MULTIPLIER = 0x465B,
			C3DS_CAMERA        = 0x4700,
			C3DS_CAM_SEE_CONE  = 0x4710,
			C3DS_CAM_RANGES    = 0x4720,

			// subs of C3DS_KEYF3DS
			C3DS_KF_HDR        = 0xB00A,
			C3DS_AMBIENT_TAG   = 0xB001,
			C3DS_OBJECT_TAG    = 0xB002,
			C3DS_CAMERA_TAG    = 0xB003,
			C3DS_TARGET_TAG    = 0xB004,
			C3DS_LIGHTNODE_TAG = 0xB005,
			C3DS_KF_SEG        = 0xB008,
			C3DS_KF_CURTIME    = 0xB009,
			C3DS_KF_NODE_HDR   = 0xB010,
			C3DS_PIVOTPOINT    = 0xB013,
			C3DS_BOUNDBOX      = 0xB014,
			C3DS_MORPH_SMOOTH  = 0xB015,
			C3DS_POS_TRACK_TAG = 0xB020,
			C3DS_ROT_TRACK_TAG = 0xB021,
			C3DS_SCL_TRACK_TAG = 0xB022,
			C3DS_NODE_ID       = 0xB030,

			// Viewport definitions
			C3DS_VIEWPORT_LAYOUT = 0x7001,
			C3DS_VIEWPORT_DATA   = 0x7011,
			C3DS_VIEWPORT_DATA_3 = 0x7012,
			C3DS_VIEWPORT_SIZE   = 0x7020,

			// different color chunk types
			C3DS_COL_RGB    = 0x0010,
			C3DS_COL_TRU    = 0x0011,
			C3DS_COL_LIN_24 = 0x0012,
			C3DS_COL_LIN_F  = 0x0013,

			// percentage chunk types
			C3DS_PERCENTAGE_I = 0x0030,
			C3DS_PERCENTAGE_F = 0x0031,

			C3DS_CHUNK_MAX		= 0xFFFF
		};

		C3DSMeshFileLoader::C3DSMeshFileLoader(ISceneManager *mgr, io::IFileSystem *fs)
		: _sceneManager(mgr), _fileSystem(fs), _vertices(NULL), _indices(NULL), _smoothingGroups(NULL),
		  _tcoords(NULL), _countVertices(0), _countFaces(0), _countTCoords(0), _mesh(NULL)
		{
			if(_fileSystem != NULL) _fileSystem->grab();
		}

		C3DSMeshFileLoader::~C3DSMeshFileLoader()
		{
			cleanUp();

			if(_fileSystem != NULL) _fileSystem->drop();
			if(_mesh != NULL) _mesh->drop();
		}

		bool C3DSMeshFileLoader::isLoadableFileExtension(const io::path& filename) const
		{
			return core::hasFileExtension(filename, "3ds");
		}

		IAnimatedMesh* C3DSMeshFileLoader::createMesh(io::IReadFile *file)
		{
			ChunkData data;

			readChunkData(file, data);

			if(data.header.id != C3DS_MAIN3DS) return NULL;

			_currentMaterial.clear();
			_materials.clear();
			_meshBufferNames.clear();
			cleanUp();

			if(_mesh) _mesh->drop();

			_mesh = new SMesh();
			if(readChunk(file, &data)) {
				printf("createMesh(success, composing)\n");

				for(u32 i=0;i < _mesh->getMeshBufferCount();i++) {
					SMeshBuffer *mb = (SMeshBuffer*)_mesh->getMeshBuffer(i);

					if(mb->getIndexCount() == 0 || mb->getVertexCount() == 0) {
						_mesh->meshBuffers.erase(i--);
						mb->drop();
					} else {
						// check parallax material type
						_mesh->meshBuffers[i]->recalculateBoundingBox();
					}
				}
				_mesh->recalculateBoundingBox();

				SAnimatedMesh *am = new SAnimatedMesh();
				am->type = EAMT_3DS;
				am->addMesh(_mesh);
				am->recalculateBoundingBox();

				_mesh->drop();
				_mesh = NULL;

				return am;
			}

			_mesh->drop();
			_mesh = NULL;

			return NULL;
		}

		void C3DSMeshFileLoader::cleanUp()
		{
			delete [] _vertices;
			_countVertices = 0;
			_vertices = NULL;

			delete [] _indices;
			_indices = NULL;
			_countFaces = 0;

			delete [] _smoothingGroups;
			_smoothingGroups = NULL;

			delete [] _tcoords;
			_tcoords = NULL;
			_countTCoords = 0;

			_materialGroups.clear();
		}

		bool C3DSMeshFileLoader::readChunk(io::IReadFile *file, ChunkData *parent)
		{
			while(parent->read < parent->header.length) {
				ChunkData data;

				readChunkData(file, data);

				switch(data.header.id) {
					case C3DS_VERSION:
					{
						u16 version;

						file->read(&version, sizeof(u16));

						version = os::ByteSwap::byteswap(version);

						file->seek(data.header.length - data.read - 2, true);
						data.read += data.header.length - data.read;
					}
					break;

					case C3DS_EDIT_MATERIAL:
						readMaterialChunk(file, &data);
						break;

					case C3DS_KEYF3DS:
						readFrameChunk(file, &data);
						break;

					case C3DS_EDIT3DS:
						break;

					case C3DS_MESHVERSION:
					case 0x01:
					{
						u32 version;

						file->read(&version, sizeof(u32));
						version = os::ByteSwap::byteswap(version);

						data.read += sizeof(u32);
					}
					break;

					case C3DS_EDIT_OBJECT:
					{
						core::stringc name;

						readString(file, data, name);
						readObjectChunk(file, &data);
						composeObject(file, name);
					}
					break;

					default:
						file->seek(data.header.length - data.read, true);
						data.read += data.header.length - data.read;
				}

				parent->read += data.read;
			}

			return true;
		}

		bool C3DSMeshFileLoader::readFrameChunk(io::IReadFile *file, ChunkData *parent)
		{
			ChunkData data;

			readChunkData(file, data);

			if(data.header.id != C3DS_KF_HDR)
				return false;
			else {
				u32 flags;
				u16 version;
				core::stringc name;

				file->read(&version, sizeof(u16));
				version = os::ByteSwap::byteswap(version);

				readString(file, data, name);

				file->read(&flags, sizeof(u32));
				flags = os::ByteSwap::byteswap(flags);

				data.read += 4; //6
				parent->read += data.read;
			}
			data.read = 0;

			IMeshBuffer *mb = NULL;
			core::vector3df pivot, bboxCenter;
			while(parent->read < parent->header.length) {
				readChunkData(file, data);

				switch(data.header.id) {
					case C3DS_OBJECT_TAG:
					{
						mb = NULL;
						pivot = core::vector3df(0.0f, 0.0f, 0.0f);
					}
					break;

					case C3DS_KF_SEG:
					{
						u32 flags;

						file->read(&flags, sizeof(u32));
						flags = os::ByteSwap::byteswap(flags);

						file->read(&flags, sizeof(u32));
						flags = os::ByteSwap::byteswap(flags);

						data.read += sizeof(u32)*2;
					}
					break;

					case C3DS_KF_NODE_HDR:
					{
						s16 flags;
						char *c = new char[data.header.length - data.read - 6];

						file->read(c, data.header.length - data.read - 6);

						for(u32 i=0;i < _meshBufferNames.size();i++) {
							if(_meshBufferNames[i] == c) {
								mb = _mesh->getMeshBuffer(i);
								break;
							}
						}

						file->read(&flags, sizeof(s16));
						flags = os::ByteSwap::byteswap(flags);

						file->read(&flags, sizeof(s16));
						flags = os::ByteSwap::byteswap(flags);

						file->read(&flags, sizeof(s16));
						flags = os::ByteSwap::byteswap(flags);

						data.read += data.header.length - data.read;
						delete [] c;
					}
					break;

					case C3DS_KF_CURTIME:
					{
						u32 flags;

						file->read(&flags, sizeof(u32));
						flags = os::ByteSwap::byteswap(flags);

						data.read += sizeof(u32);
					}
					break;

					case C3DS_NODE_ID:
					{
						u16 flags;

						file->read(&flags, sizeof(u16));
						flags = os::ByteSwap::byteswap(flags);

						data.read += sizeof(u16);
					}
					break;

					case C3DS_PIVOTPOINT:
					{
						f32 x, y, z;

						file->read(&x, sizeof(f32));
						x = os::ByteSwap::byteswap(x);

						file->read(&y, sizeof(f32));
						y = os::ByteSwap::byteswap(y);

						file->read(&z, sizeof(f32));
						z = os::ByteSwap::byteswap(z);

						pivot = core::vector3df(x, y, z);

						data.read += sizeof(f32)*3;
					}
					break;

					case C3DS_BOUNDBOX:
					{
						f32 x, y, z;
						core::aabbox3df bbox;

					}
					break;

					default:
						file->seek(data.header.length - data.read, true);
						data.read += data.header.length - data.read;
				}

				parent->read += data.read;
				data.read = 0;
			}
			return true;
		}

		bool C3DSMeshFileLoader::readMaterialChunk(io::IReadFile *file, ChunkData *parent)
		{
			u16 matSection = 0;

			while(parent->read < parent->header.length) {
				ChunkData data;

				readChunkData(file, data);

				switch(data.header.id) {
					case C3DS_MATNAME:
					{
						char *c = new char[data.header.length - data.read];

						file->read(c, data.header.length - data.read);

						if(strlen(c)) _currentMaterial.name = c;

						data.read += data.header.length - data.read;
						delete [] c;
					}
					break;

					case C3DS_MATAMBIENT:
						readColorChunk(file, &data, _currentMaterial.material.ambientColor);
						break;
					case C3DS_MATDIFFUSE:
						readColorChunk(file, &data, _currentMaterial.material.diffuseColor);
						break;
					case C3DS_MATSPECULAR:
						readColorChunk(file, &data, _currentMaterial.material.specularColor);
						break;
					case C3DS_MATSHININESS:
						readPercentageChunk(file, &data, _currentMaterial.material.shininess);
						_currentMaterial.material.shininess = (1.0f - _currentMaterial.material.shininess)*128.0f;
						break;
					case C3DS_TRANSPARENCY:
					{
						f32 percentage;

						readPercentageChunk(file, &data, percentage);

						if(percentage > 0.0f) {
						} else {
							_currentMaterial.material.materialType = video::EMT_SOLID;
						}
					}
					break;

					case C3DS_WIRE:
						_currentMaterial.material.wireFrame = true;
						break;
					case C3DS_TWO_SIDE:
						_currentMaterial.material.backfaceCulling = false;
						break;
					case C3DS_SHADING:
					{
						s16 flags;

						file->read(&flags, sizeof(s16));

						flags = os::ByteSwap::byteswap(flags);
						switch(flags) {
							case 0:
								_currentMaterial.material.wireFrame = true;
								break;
							case 1:
								_currentMaterial.material.wireFrame = false;
								_currentMaterial.material.gouraudShading = false;
								break;
							case 2:
								_currentMaterial.material.wireFrame = false;
								_currentMaterial.material.gouraudShading = true;
								break;
							default:
								break;
						}
						data.read += data.header.length - data.read;
					}
					break;

					case C3DS_MATTEXMAP:
					case C3DS_MATSPECMAP:
					case C3DS_MATOPACMAP:
					case C3DS_MATREFLMAP:
					case C3DS_MATBUMPMAP:
					{
						matSection = data.header.id;

						s16 testval;
						const long pos = file->getPos();

						file->read(&testval, sizeof(s16));
						testval = os::ByteSwap::byteswap(testval);

						file->seek(pos, false);

						if(testval == C3DS_PERCENTAGE_I || testval == C3DS_PERCENTAGE_F) {
							switch(matSection) {
								case C3DS_MATTEXMAP:
									readPercentageChunk(file, &data, _currentMaterial.strength[0]);
									break;
								case C3DS_MATSPECMAP:
									readPercentageChunk(file, &data, _currentMaterial.strength[1]);
									break;
								case C3DS_MATOPACMAP:
									readPercentageChunk(file, &data, _currentMaterial.strength[2]);
									break;
								case C3DS_MATREFLMAP:
									readPercentageChunk(file, &data, _currentMaterial.strength[3]);
									break;
								case C3DS_MATBUMPMAP:
									readPercentageChunk(file, &data, _currentMaterial.strength[4]);
									break;
							}
						}
					}
					break;

					case C3DS_MATMAPFILE:
					{
						char *c = new char[data.header.length - data.read];

						file->read(c, data.header.length - data.read);
						switch(matSection) {
							case C3DS_MATTEXMAP:
								_currentMaterial.filename[0] = c;
								break;
							case C3DS_MATSPECMAP:
								_currentMaterial.filename[1] = c;
								break;
							case C3DS_MATOPACMAP:
								_currentMaterial.filename[2] = c;
								break;
							case C3DS_MATREFLMAP:
								_currentMaterial.filename[3] = c;
								break;
							case C3DS_MATBUMPMAP:
								_currentMaterial.filename[4] = c;
								break;
						}
						data.read += data.header.length - data.read;
						delete [] c;
					}
					break;

					case C3DS_MAT_TEXTILING:
					{
						s16 flags;

						file->read(&flags, sizeof(s16));
						flags = os::ByteSwap::byteswap(flags);

						data.read += sizeof(s16);
					}
					break;

					case C3DS_MAT_USCALE:
					case C3DS_MAT_VSCALE:
					case C3DS_MAT_UOFFSET:
					case C3DS_MAT_VOFFSET:
					{
						f32 value;

						file->read(&value, sizeof(f32));
						value = os::ByteSwap::byteswap(value);

						u32 i = 0;
						if(matSection != C3DS_MATTEXMAP) i = 1;

						u32 j = 0, k = 0;
						if(data.header.id == C3DS_MAT_VSCALE) {
							j = 1;
							k = 1;
						}
						else if(data.header.id == C3DS_MAT_UOFFSET) {
							j = 2;
							k = 0;
						}
						else if(data.header.id == C3DS_MAT_VOFFSET) {
							j = 2;
							k = 1;
						}
						_currentMaterial.material.getTextureMatrix(i).setElem(j, k, value);

						data.read += sizeof(f32);
					}
					break;

					default:
						file->seek(data.header.length - data.read, true);
						data.read += data.header.length - data.read;
				}
				parent->read += data.read;
			}

			_materials.push_back(_currentMaterial);
			_currentMaterial.clear();

			return true;
		}

		bool C3DSMeshFileLoader::readColorChunk(io::IReadFile *file, ChunkData *chunk, video::SColor& out)
		{
			ChunkData data;

			readChunkData(file, data);

			u8 c[3];
			f32 cf[3];

			switch(data.header.id) {
				case C3DS_COL_TRU:
				case C3DS_COL_LIN_24:
				{
					file->read(c, sizeof(c));
					out.set(c[0], c[1], c[2], 255);
					data.read += sizeof(c);
				}
				break;

				case C3DS_COL_RGB:
				case C3DS_COL_LIN_F:
				{
					file->read(cf, sizeof(cf));
					cf[0] = os::ByteSwap::byteswap(cf[0]);
					cf[1] = os::ByteSwap::byteswap(cf[1]);
					cf[2] = os::ByteSwap::byteswap(cf[2]);
					out.set((s32)(cf[0]*255.0f), (s32)(cf[1]*255.0f), (s32)(cf[2]*255.0f), 255);
					data.read += sizeof(cf);
				}
				break;

				default:
					file->seek(data.header.length - data.read, true);
					data.read += data.header.length - data.read;
			}
			chunk->read += data.read;

			return true;
		}

		bool C3DSMeshFileLoader::readPercentageChunk(io::IReadFile *file, ChunkData *chunk, f32& percentage)
		{
			ChunkData data;

			readChunkData(file, data);

			s16 ipercentage;
			f32 fpercentage;

			switch(data.header.id) {
				case C3DS_PERCENTAGE_I:
				{
					file->read(&ipercentage, sizeof(s16));
					ipercentage = os::ByteSwap::byteswap(ipercentage);
					percentage = ipercentage/100.0f;
					data.read += sizeof(s16);
				}
				break;

				case C3DS_PERCENTAGE_F:
				{
					file->read(&fpercentage, sizeof(f32));
					percentage = os::ByteSwap::byteswap(fpercentage);
					data.read += sizeof(f32);
				}
				break;

				default:
					file->seek(data.header.length - data.read, true);
					data.read += data.header.length - data.read;
			}
			chunk->read += data.read;

			return true;
		}

		bool C3DSMeshFileLoader::readObjectChunk(io::IReadFile *file, ChunkData *parent)
		{
			while(parent->read < parent->header.length) {
				ChunkData data;

				readChunkData(file, data);

				switch(data.header.id) {
					case C3DS_OBJTRIMESH:
						readObjectChunk(file, &data);
						break;
					case C3DS_TRIVERT:
						readVertices(file, data);
						break;
					case C3DS_POINTFLAGARRAY:
					{
						u16 numVertex, flags;

						file->read(&numVertex, sizeof(u16));
						numVertex = os::ByteSwap::byteswap(numVertex);

						for(u32 i=0;i < numVertex;i++) {
							file->read(&flags, sizeof(u16));
							flags = os::ByteSwap::byteswap(flags);
						}
						data.read += (numVertex + 1)*sizeof(u16);
					}
					break;

					case C3DS_TRIFACE:
						readIndices(file, data);
						readObjectChunk(file, &data);
						break;
					case C3DS_TRIFACEMAT:
						readMaterialGroup(file, data);
						break;
					case C3DS_TRIUV:
						readTextureCoords(file, data);
						break;
					case C3DS_TRIMATRIX:
					{
						f32 mat[4][3];

						file->read(mat, 12*sizeof(f32));

						_transformationMatrix = core::matrix4::identity();
						for(u32 i=0;i < 4;i++) {
							for(u32 j=0;j < 3;j++) {
								_transformationMatrix.setElem(i, j, os::ByteSwap::byteswap(mat[i][j]));
							}
						}
						data.read += 12*sizeof(f32);
					}
					break;

					case C3DS_MESHCOLOR:
					{
						u8 flag;

						file->read(&flag, sizeof(u8));
						++data.read;
					}
					break;

					case C3DS_TRISMOOTH:
					{
						_smoothingGroups = new u32[_countFaces];

						file->read(_smoothingGroups, _countFaces*sizeof(u32));
						for(u32 i=0;i < _countFaces;i++)
							_smoothingGroups[i] = os::ByteSwap::byteswap(_smoothingGroups[i]);

						data.read += _countFaces*sizeof(u32);
					}
					break;

					default:
						file->seek(data.header.length - data.read, true);
						data.read += data.header.length - data.read;
				}
				parent->read += data.read;
			}

			return true;
		}

		void C3DSMeshFileLoader::composeObject(io::IReadFile *file, const core::stringc& name)
		{
			if(_mesh->getMeshBufferCount() != _materials.size())
				loadMaterials(file);

			if(_materialGroups.empty()) {
				SMaterialGroup group;

				group.faceCount = _countFaces;
				group.faces = new u16[group.faceCount];
				for(u32 i=0;i < group.faceCount;i++)
					group.faces[i] = i;

				_materialGroups.push_back(group);
				if(_materials.empty()) {
					SCurrentMaterial m;
					SMeshBuffer *mb = new scene::SMeshBuffer();

					_materials.push_back(m);
					_mesh->addMeshBuffer(mb);

					mb->getMaterial() = _materials[0].material;
					mb->drop();

					_meshBufferNames.push_back("");
				}
			}

			for(u32 i=0;i < _materialGroups.size();i++) {
				u32 mbPos = 0;
				SMeshBuffer *mb = NULL;
				video::SMaterial *mat = NULL;
				u32 maxPrimitives = core::min_(_sceneManager->getVideoDriver()->getMaximalPrimitiveCount(), (u32)((1<<16) - 1)) - 3;

				for(mbPos=0;mbPos < _materials.size();mbPos++) {
					if(_materialGroups[i].materialName == _materials[mbPos].name) {
						mb = (SMeshBuffer*)_mesh->getMeshBuffer(mbPos);
						mat = &_materials[mbPos].material;
						_meshBufferNames[mbPos] = name;
						break;
					}
				}

				if(mb != NULL) {
					video::S3DVertexStandard vtx;
					core::vector3df vec;

					vtx.col = mat->diffuseColor;
					vtx.nrm = core::vector3df(0.0f, 0.0f, 0.0f);

					for(s32 f=0;f < _materialGroups[i].faceCount;f++) {
						u32 vtxCount = mb->vertices.size();

						if(vtxCount > maxPrimitives) {
							IMeshBuffer *tmp = mb;

							mb = new SMeshBuffer();
							_mesh->addMeshBuffer(mb);

							mb->drop();

							_mesh->meshBuffers[mbPos] = _mesh->meshBuffers.getLast();
							_mesh->meshBuffers[_mesh->meshBuffers.size() - 1] = tmp;
							mb->getMaterial() = tmp->getMaterial();
							vtxCount = 0;
						}

						for(s32 v=0;v < 3;v++) {
							s32 idx = _indices[_materialGroups[i].faces[f]*4 + v];

							if(_countVertices > idx) {
								vtx.pos.setX(_vertices[idx*3 + 0]);
								vtx.pos.setZ(_vertices[idx*3 + 1]);
								vtx.pos.setY(_vertices[idx*3 + 2]);
							}

							if(_countTCoords > idx) {
								vtx.tcoords.X = _tcoords[idx*2 + 0];
								vtx.tcoords.Y = 1.0f - _tcoords[idx*2 + 1];
							}

							mb->vertices.push_back(vtx);
						}

						core::plane3df pl(mb->vertices[vtxCount + 0].pos, mb->vertices[vtxCount + 2].pos, mb->vertices[vtxCount + 1].pos);
						mb->vertices[vtxCount + 0].nrm = pl.normal;
						mb->vertices[vtxCount + 1].nrm = pl.normal;
						mb->vertices[vtxCount + 2].nrm = pl.normal;

						mb->indices.push_back(vtxCount + 0);
						mb->indices.push_back(vtxCount + 1);
						mb->indices.push_back(vtxCount + 2);
					}
				}
			}
			cleanUp();
		}

		void C3DSMeshFileLoader::loadMaterials(io::IReadFile *file)
		{
			core::stringc modelFilename = file->getFilename();

			_meshBufferNames.reallocate(_materials.size());
			for(u32 i=0;i < _materials.size();i++) {
				SMeshBuffer *m = new scene::SMeshBuffer();

				_meshBufferNames.push_back("");
				_mesh->addMeshBuffer(m);

				m->getMaterial() = _materials[i].material;
				if(_materials[i].filename[0].size()) {
					video::ITexture *texture = NULL;

					printf("loading texture: %s\n", _materials[i].filename[0].c_str());
					if(_fileSystem->existFile(_materials[i].filename[0]))
						texture = _sceneManager->getVideoDriver()->getTexture(_materials[i].filename[0]);

					if(texture == NULL) {
						const core::stringc fname = _fileSystem->getFileDir(modelFilename) + "/" + _fileSystem->getFileBasename(_materials[i].filename[0]);

						if(_fileSystem->existFile(fname))
							texture = _sceneManager->getVideoDriver()->getTexture(fname);
					}

					if(texture != NULL)
						m->getMaterial().setTexture(0, texture);
				}

				if(_materials[i].filename[2].size()) {
					video::ITexture *texture = NULL;

					if(_fileSystem->existFile(_materials[i].filename[2]))
						texture = _sceneManager->getVideoDriver()->getTexture(_materials[i].filename[2]);

					if(texture == NULL) {
						const core::stringc fname = _fileSystem->getFileDir(modelFilename) + "/" + _fileSystem->getFileBasename(_materials[i].filename[2]);

						if(_fileSystem->existFile(fname))
							texture = _sceneManager->getVideoDriver()->getTexture(fname);
					}

					if(texture != NULL) {
						m->getMaterial().setTexture(0, texture);
						m->getMaterial().materialType = video::EMT_TRANSPARENT_ADD_COLOR;
					}
				}
			}
		}

		void C3DSMeshFileLoader::readChunkData(io::IReadFile *file, ChunkData& data)
		{
			file->read(&data.header, sizeof(ChunkHeader));
			data.header.id = os::ByteSwap::byteswap(data.header.id);
			data.header.length = os::ByteSwap::byteswap(data.header.length);
			data.read += sizeof(ChunkHeader);
		}

		void C3DSMeshFileLoader::readMaterialGroup(io::IReadFile *file, ChunkData& data)
		{
			SMaterialGroup group;

			readString(file, data, group.materialName);

			file->read(&group.faceCount, sizeof(u16));
			group.faceCount = os::ByteSwap::byteswap(group.faceCount);
			data.read += sizeof(u16);

			group.faces = new u16[group.faceCount];
			file->read(group.faces, group.faceCount*sizeof(u16));
			for(u32 i=0;i < group.faceCount;i++)
				group.faces[i] = os::ByteSwap::byteswap(group.faces[i]);

			data.read += group.faceCount*sizeof(u16);
			_materialGroups.push_back(group);
		}

		void C3DSMeshFileLoader::readVertices(io::IReadFile *file, ChunkData& data)
		{
			file->read(&_countVertices, sizeof(u16));
			_countVertices = os::ByteSwap::byteswap(_countVertices);
			data.read += sizeof(u16);

			const s32 vertexBufferByteSize = _countVertices*sizeof(f32)*3;
			if(data.header.length - data.read != vertexBufferByteSize) return;

			_vertices = new f32[_countVertices*3];
			file->read(_vertices, vertexBufferByteSize);
			for(u32 i=0;i < _countVertices*3;i++)
				_vertices[i] = os::ByteSwap::byteswap(_vertices[i]);

			data.read += vertexBufferByteSize;
		}

		void C3DSMeshFileLoader::readIndices(io::IReadFile *file, ChunkData& data)
		{
			file->read(&_countFaces, sizeof(u16));
			_countFaces = os::ByteSwap::byteswap(_countFaces);
			data.read += sizeof(u16);

			s32 indexBufferByteSize = _countFaces*sizeof(u16)*4;

			_indices = new u16[_countFaces*4];
			file->read(_indices, indexBufferByteSize);
			for(u32 i=0;i < _countFaces*4;i++)
				_indices[i] = os::ByteSwap::byteswap(_indices[i]);

			data.read += indexBufferByteSize;
		}

		void C3DSMeshFileLoader::readTextureCoords(io::IReadFile *file, ChunkData& data)
		{
			file->read(&_countTCoords, sizeof(u16));
			_countTCoords = os::ByteSwap::byteswap(_countTCoords);
			data.read += sizeof(u16);

			s32 tcoordsBufferByteSize = _countTCoords*sizeof(f32)*2;
			if(data.header.length - data.read != tcoordsBufferByteSize) return;

			_tcoords = new f32[_countTCoords*2];
			file->read(_tcoords, tcoordsBufferByteSize);
			for(u32 i=0;i < _countTCoords*2;i++)
				_tcoords[i] = os::ByteSwap::byteswap(_tcoords[i]);

			data.read += tcoordsBufferByteSize;
		}

		void C3DSMeshFileLoader::readString(io::IReadFile *file, ChunkData& data, core::stringc& out)
		{
			char c = 1;

			out = "";
			while(c) {
				file->read(&c, sizeof(char));
				if(c) out.append(c);
			}
			data.read += out.size() + 1;
		}
	}
}


