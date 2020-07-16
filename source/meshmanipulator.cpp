/*
 * meshmanipulator.cpp
 *
 *  Created on: Mar 1, 2013
 *      Author: mike
 */

#include "meshmanipulator.h"
#include "smesh.h"
#include "smeshbuffer.h"
#include "sanimatedmesh.h"
#include "os.h"
#include "irrmap.h"

namespace irr
{
	namespace scene
	{
		static inline core::vector3df getAngleWeight(const core::vector3df& v1, const core::vector3df& v2, const core::vector3df& v3)
		{
			const f32 a = core::distanceFromSQ(v2, v3);
			const f32 asqrt = core::sqrtf32(a);
			const f32 b = core::distanceFromSQ(v1, v3);
			const f32 bsqrt = core::sqrtf32(b);
			const f32 c = core::distanceFromSQ(v1, v2);
			const f32 csqrt = core::sqrtf32(c);

			return core::vector3df(acosf((b + c - a)/(2.0f*bsqrt*csqrt)),
								   acosf((-b + c + a)/(2.0f*asqrt*csqrt)),
								   acosf((b - c + a)/(2.0f*bsqrt*asqrt)));
		}

		void calculateTangents(core::vector3df& normal, core::vector3df& tangent, core::vector3df& binormal, const core::vector3df& vt1, const core::vector3df& vt2, const core::vector3df& vt3, const core::vector2df& tc1, const core::vector2df& tc2, const core::vector2df& tc3)
		{
			core::vector3df v1 = vt1 - vt2;
			core::vector3df v2 = vt3 - vt1;
			normal = normalize(cross(v2, v1));

			f32 deltaX1 = tc1.X - tc2.X;
			f32 deltaX2 = tc3.X - tc1.X;
			binormal = normalize((v1*deltaX2) - (v2*deltaX1));

			f32 deltaY1 = tc1.Y - tc2.Y;
			f32 deltaY2 = tc3.Y - tc1.Y;
			tangent = normalize((v1*deltaY2) - (v2*deltaY1));

			core::vector3df txb = cross(tangent, binormal);
			if(dot(txb, normal) < 0.0f) {
				tangent *= -1.0f;
				binormal *= -1.0f;
			}
		}

		template< typename T >
		void recalculateNormalsT(IMeshBuffer *buffer, bool smooth, bool angleWeighted)
		{
			const u32 vtxcnt = buffer->getVertexCount();
			const u32 idxcnt = buffer->getIndexCount();
			const T *idx = reinterpret_cast<T*>(buffer->getIndices());

			if(!smooth) {
				for(u32 i=0;i < idxcnt;i+=3) {
					const core::vector3df& v1 = buffer->getPosition(idx[i + 0]);
					const core::vector3df& v2 = buffer->getPosition(idx[i + 1]);
					const core::vector3df& v3 = buffer->getPosition(idx[i + 2]);
					const core::vector3df normal = core::plane3df(v1, v2, v3).normal;

					buffer->getNormal(idx[i + 0]) = normal;
					buffer->getNormal(idx[i + 1]) = normal;
					buffer->getNormal(idx[i + 2]) = normal;
				}
			} else {
				u32 i;

				for(i=0;i < vtxcnt;i++)
					buffer->getNormal(i) = core::vector3df(0.0f, 0.0f, 0.0f);

				for(i=0;i < idxcnt;i+=3) {
					const core::vector3df& v1 = buffer->getPosition(idx[i + 0]);
					const core::vector3df& v2 = buffer->getPosition(idx[i + 1]);
					const core::vector3df& v3 = buffer->getPosition(idx[i + 2]);
					const core::vector3df normal = core::plane3df(v1, v2, v3).normal;

					core::vector3df weight(1.0f, 1.0f, 1.0f);
					if(angleWeighted)
						weight = getAngleWeight(v1, v2, v3);

					buffer->getNormal(idx[i + 0]) = weight.getX()*normal;
					buffer->getNormal(idx[i + 1]) = weight.getY()*normal;
					buffer->getNormal(idx[i + 2]) = weight.getZ()*normal;
				}

				for(i=0;i < vtxcnt;i++)
					buffer->getNormal(i) = normalize(buffer->getNormal(i));
			}
		}

		template< typename T >
		void recalculateTangentsT(IMeshBuffer *buffer, bool recalculateNormals, bool smooth, bool angleWeighted)
		{
			if(buffer == NULL || buffer->getVertexType() != video::EVT_TANGENTS) return;

			const u32 vtxcnt = buffer->getVertexCount();
			const u32 idxcnt = buffer->getIndexCount();
			T *idx = reinterpret_cast<T*>(buffer->getIndices());
			video::S3DVertexTangents *v = (video::S3DVertexTangents*)buffer->getVertices();

			if(smooth) {
				u32 i;

				for(i=0;i < vtxcnt;i++) {
					if(recalculateNormals)
						v[i].nrm = core::vector3df(0.0f, 0.0f, 0.0f);

					v[i].tangent = core::vector3df(0.0f, 0.0f, 0.0f);
					v[i].binormal = core::vector3df(0.0f, 0.0f, 0.0f);
				}

				for(i=0;i < idxcnt;i+=3) {
					if(v[idx[i + 0]].pos == v[idx[i + 1]].pos ||
					   v[idx[i + 0]].pos == v[idx[i + 2]].pos ||
					   v[idx[i + 1]].pos == v[idx[i + 2]].pos)
					{
						continue;
					}

					core::vector3df weight(1.0f, 1.0f, 1.0f);
					if(angleWeighted) weight = getAngleWeight(v[i + 0].pos, v[i + 1].pos, v[i + 2].pos);

					core::vector3df localNormal;
					core::vector3df localTangent;
					core::vector3df localBinormal;

					calculateTangents(localNormal, localTangent, localBinormal, v[idx[i + 0]].pos, v[idx[i + 1]].pos, v[idx[i + 2]].pos, v[idx[i+ 0]].tcoords, v[idx[i+ 1]].tcoords, v[idx[i+ 2]].tcoords);
					if(recalculateNormals) v[idx[i + 0]].nrm += localNormal*weight.getX();
					v[idx[i + 0]].tangent += localTangent*weight.getX();
					v[idx[i + 0]].binormal += localBinormal*weight.getX();

					calculateTangents(localNormal, localTangent, localBinormal, v[idx[i + 1]].pos, v[idx[i + 2]].pos, v[idx[i + 0]].pos, v[idx[i+ 1]].tcoords, v[idx[i+ 2]].tcoords, v[idx[i+ 0]].tcoords);
					if(recalculateNormals) v[idx[i + 1]].nrm += localNormal*weight.getY();
					v[idx[i + 1]].tangent += localTangent*weight.getY();
					v[idx[i + 1]].binormal += localBinormal*weight.getY();

					calculateTangents(localNormal, localTangent, localBinormal, v[idx[i + 2]].pos, v[idx[i + 0]].pos, v[idx[i + 1]].pos, v[idx[i+ 2]].tcoords, v[idx[i+ 0]].tcoords, v[idx[i+ 1]].tcoords);
					if(recalculateNormals) v[idx[i + 2]].nrm += localNormal*weight.getZ();
					v[idx[i + 2]].tangent += localTangent*weight.getZ();
					v[idx[i + 2]].binormal += localBinormal*weight.getZ();
				}

				if(recalculateNormals) {
					for(i=0;i < vtxcnt;i++)
						v[i].nrm = normalize(v[i].nrm);
				}

				for(i=0;i < vtxcnt;i++) {
					v[i].tangent = normalize(v[i].tangent);
					v[i].binormal = normalize(v[i].binormal);
				}
			} else {
				core::vector3df localNormal;

				for(u32 i=0;i < idxcnt;i+=3) {
					calculateTangents(localNormal, v[idx[i + 0]].tangent, v[idx[i + 0]].binormal, v[idx[i + 0]].pos, v[idx[i + 1]].pos, v[idx[i + 2]].pos, v[idx[i + 0]].tcoords, v[idx[i + 1]].tcoords, v[idx[i + 2]].tcoords);
					if(recalculateNormals) v[idx[i + 0]].nrm = localNormal;

					calculateTangents(localNormal, v[idx[i + 1]].tangent, v[idx[i + 1]].binormal, v[idx[i + 1]].pos, v[idx[i + 2]].pos, v[idx[i + 0]].pos, v[idx[i + 1]].tcoords, v[idx[i + 2]].tcoords, v[idx[i + 0]].tcoords);
					if(recalculateNormals) v[idx[i + 1]].nrm = localNormal;

					calculateTangents(localNormal, v[idx[i + 2]].tangent, v[idx[i + 2]].binormal, v[idx[i + 2]].pos, v[idx[i + 0]].pos, v[idx[i + 1]].pos, v[idx[i + 2]].tcoords, v[idx[i + 0]].tcoords, v[idx[i + 1]].tcoords);
					if(recalculateNormals) v[idx[i + 2]].nrm = localNormal;
				}
			}
		}

		template< typename T >
		void makePlanarTextureMappingT(scene::IMeshBuffer *buffer, f32 resolution)
		{
			const u32 idxcnt = buffer->getIndexCount();
			T *idx = reinterpret_cast<T*>(buffer->getIndices());

			for(u32 i=0;i < idxcnt;i+=3) {
				core::plane3df p(buffer->getPosition(idx[i + 0]), buffer->getPosition(idx[i + 1]), buffer->getPosition(idx[i + 2]));
				p.normal.setX(fabsf(p.normal.getX()));
				p.normal.setY(fabsf(p.normal.getY()));
				p.normal.setZ(fabsf(p.normal.getZ()));

				if(p.normal.getX() > p.normal.getY() && p.normal.getX() > p.normal.getZ()) {
					for(u32 o=0;o < 3;o++) {
						buffer->getTCoords(idx[i + o]).X = buffer->getPosition(idx[i+o]).getY()*resolution;
						buffer->getTCoords(idx[i + o]).Y = buffer->getPosition(idx[i+o]).getZ()*resolution;
					}
				}
				else if(p.normal.getY() > p.normal.getX() && p.normal.getY() > p.normal.getZ()) {
					for(u32 o=0;o < 3;o++) {
						buffer->getTCoords(idx[i + o]).X = buffer->getPosition(idx[i+o]).getX()*resolution;
						buffer->getTCoords(idx[i + o]).Y = buffer->getPosition(idx[i+o]).getZ()*resolution;
					}
				}
				else {
					for(u32 o=0;o < 3;o++) {
						buffer->getTCoords(idx[i + o]).X = buffer->getPosition(idx[i+o]).getX()*resolution;
						buffer->getTCoords(idx[i + o]).Y = buffer->getPosition(idx[i+o]).getY()*resolution;
					}
				}
			}
		}

		void CMeshManipulator::makePlanarTextureMapping(IMesh *mesh, f32 resolution) const
		{
			if(!mesh)
				return;

			const u32 bcount = mesh->getMeshBufferCount();
			for(u32 b=0;b < bcount;b++)
				makePlanarTextureMapping(mesh->getMeshBuffer(b), resolution);
		}

		void CMeshManipulator::makePlanarTextureMapping(IMeshBuffer *buffer, f32 resolution) const
		{
			if(!buffer)
				return;

			if(buffer->getIndexType() == video::EIT_16BIT)
				makePlanarTextureMappingT<u16>(buffer, resolution);
			else
				makePlanarTextureMappingT<u32>(buffer, resolution);
		}

		void CMeshManipulator::flipSurfaces(IMesh *mesh) const
		{
			if(mesh == NULL) return;

			const u32 bcount = mesh->getMeshBufferCount();
			for(u32 b=0;b < bcount;b++) {
				IMeshBuffer *buffer = mesh->getMeshBuffer(b);
				const u32 idxcnt = buffer->getIndexCount();

				if(buffer->getIndexType() == video::EIT_16BIT) {
					u16 *idx = buffer->getIndices();

					for(u32 i=0;i < idxcnt;i+=3) {
						const u16 tmp = idx[i + 1];
						idx[i + 1] = idx[i + 2];
						idx[i + 2] = tmp;
					}
				} else {
					u32 *idx = reinterpret_cast<u32*>(buffer->getIndices());

					for(u32 i=0;i < idxcnt;i+=3) {
						const u32 tmp = idx[i + 1];
						idx[i + 1] = idx[i + 2];
						idx[i + 2] = tmp;
					}
				}
			}
		}

		void CMeshManipulator::recalculateNormals(scene::IMesh *mesh, bool smooth, bool angleWeighted) const
		{
			if(mesh == NULL) return;

			const u32 bcount = mesh->getMeshBufferCount();
			for(u32 b=0;b < bcount;b++)
				recalculateNormals(mesh->getMeshBuffer(b), smooth, angleWeighted);
		}

		void CMeshManipulator::recalculateNormals(IMeshBuffer *buffer, bool smooth, bool angleWeighted) const
		{
			if(buffer == NULL) return;

			if(buffer->getIndexType() == video::EIT_16BIT)
				recalculateNormalsT<u16>(buffer, smooth, angleWeighted);
			else
				recalculateNormalsT<u32>(buffer, smooth, angleWeighted);
		}

		void CMeshManipulator::recalculateTangents(IMesh *mesh, bool recalculateNormals, bool smooth, bool angleWeighted) const
		{
			if(mesh == NULL) return;

			const u32 bcount = mesh->getMeshBufferCount();
			for(u32 b=0;b < bcount;b++)
				recalculateTangents(mesh->getMeshBuffer(b), recalculateNormals, smooth, angleWeighted);
		}

		void CMeshManipulator::recalculateTangents(IMeshBuffer *buffer, bool recalculateNormals, bool smooth, bool angleWeighted) const
		{
			if(buffer != NULL && buffer->getVertexType() == video::EVT_TANGENTS) {
				if(buffer->getIndexType() == video::EIT_16BIT)
					recalculateTangentsT<u16>(buffer, recalculateNormals, smooth, angleWeighted);
				else
					recalculateTangentsT<u32>(buffer, recalculateNormals, smooth, angleWeighted);
			}
		}

		IMesh* CMeshManipulator::createMeshWithTangents(IMesh *mesh, bool recalculateNormals, bool smooth, bool angleWeighted, bool calculateTangents) const
		{
			if(mesh == NULL) return NULL;

			SMesh *clone = new SMesh();
			const u32 bcount = mesh->getMeshBufferCount();

			for(u32 b=0;b < bcount;b++) {
				const IMeshBuffer* const original = mesh->getMeshBuffer(b);
				const u32 idxcnt = original->getIndexCount();
				const u16 *idx = original->getIndices();

				SMeshBufferTangents *buffer = new SMeshBufferTangents();

				buffer->material = original->getMaterial();
				buffer->vertices.reallocate(idxcnt);
				buffer->indices.reallocate(idxcnt);

				s32 vertLocation;
				video::S3DVertexTangents vNew;
				core::map< video::S3DVertexTangents, int > vertMap;
				const video::E_VERTEX_TYPE vType = original->getVertexType();
				for(u32 i=0;i < idxcnt;i++) {
					switch(vType) {
						case video::EVT_STANDARD:
						{
							const video::S3DVertexStandard *v = (const video::S3DVertexStandard*)original->getVertices();
							vNew = video::S3DVertexTangents(v[idx[i]].pos, v[idx[i]].nrm, v[idx[i]].col, v[idx[i]].tcoords);
						}
						break;

						case video::EVT_TANGENTS:
						{
							const video::S3DVertexTangents *v = (const video::S3DVertexTangents*)original->getVertices();
							vNew = v[idx[i]];
						}
						default:
							break;

					}

					core::map< video::S3DVertexTangents, int>::Node *n = vertMap.find(vNew);
					if(n != NULL) {
						vertLocation = n->getValue();
					} else {
						vertLocation = buffer->vertices.size();
						buffer->vertices.push_back(vNew);
						vertMap.insert(vNew, vertLocation);
					}

					buffer->indices.push_back(vertLocation);
				}
				buffer->recalculateBoundingBox();

				clone->addMeshBuffer(buffer);
				buffer->drop();
			}

			clone->recalculateBoundingBox();
			if(calculateTangents)
				recalculateTangents(clone, recalculateNormals, smooth, angleWeighted);

			return clone;
		}

		namespace
		{
			struct vcache
			{
				core::array< u32 > tris;
				f32 score;
				s16 cachePos;
				u16 numActiveTris;
			};

			struct tcache
			{
				u16 ind[3];
				f32 score;
				bool drawn;
			};

			const u16 cacheSize = 32;

			f32 findVertexScore(vcache *v)
			{
				const f32 cacheDecayPower = 1.5f;
				const f32 lastTriScore = 0.75f;
				const f32 valenceBoostScale = 2.0f;
				const f32 valenceBoostPower = 0.5f;
				const f32 maxSizeVertexCache = 32.0f;

				if(v->numActiveTris == 0)
					return -1.0f;

				f32 score = 0.0f;
				s32 cachePos = v->cachePos;
				if(cachePos < 0) {}
				else {
					if(cachePos < 3)
						score = lastTriScore;
					else {
						const f32 scaler = 1.0f/(maxSizeVertexCache - 3);
						score = 1.0f - (cachePos - 3)*scaler;
						score = powf(score, cacheDecayPower);
					}
				}

				f32 valenceBoost = powf(v->numActiveTris, -valenceBoostPower);
				score += valenceBoostScale*valenceBoost;

				return score;
			}

			class f_lru
			{
			public:
				f_lru(vcache *v, tcache *t) : vc(v), tc(t)
				{
					for(u16 i=0;i < cacheSize;i++)
						cache[i] = -1;
				}

				u32 add(u16 vert, bool updateTris = false)
				{
					bool found = false;

					for(u16 i=0;i < cacheSize;i++) {
						if(cache[i] == vert) {
							for(u16 j=i;j;j--)
								cache[j] = cache[j - 1];

							found = true;
							break;
						}
					}

					if(!found) {
						if(cache[cacheSize - 1] != -1)
							vc[cache[cacheSize - 1]].cachePos = -1;

						for(u16 i=cacheSize - 1;i;i--)
							cache[i] = cache[i - 1];
					}

					cache[0] = vert;

					u32 highest = 0;
					f32 hiscore = 0.0f;
					if(updateTris) {
						for(u16 i=0;i < cacheSize;i++) {
							if(cache[i] == -1) break;

							const u16 trisize = vc[cache[i]].tris.size();
							for(u16 t=0;t < trisize;t++) {
								tcache *tri = &tc[vc[cache[i]].tris[t]];

								tri->score = vc[tri->ind[0]].score + vc[tri->ind[1]].score + vc[tri->ind[2]].score;
								if(tri->score > hiscore) {
									hiscore = tri->score;
									highest = vc[cache[i]].tris[t];
								}
							}
						}
					}

					return highest;
				}

			private:
				s32 cache[cacheSize];
				vcache *vc;
				tcache *tc;
			};
		}

		IMesh* CMeshManipulator::createForsythOptimizedMesh(const IMesh *mesh) const
		{
			if(!mesh)
				return NULL;

			SMesh *newMesh = new SMesh();

			newMesh->bbox = mesh->getBoundingBox();

			const u32 mbcount = mesh->getMeshBufferCount();
			for(u32 b=0;b < mbcount;b++) {
				const IMeshBuffer *mb = mesh->getMeshBuffer(b);

				if(mb->getIndexType() != video::EIT_16BIT) {
					newMesh->drop();
					return NULL;
				}

				const u32 icount = mb->getIndexCount();
				const u32 tcount = icount/3;
				const u32 vcount = mb->getVertexCount();
				const u16 *ind = mb->getIndices();

				vcache *vc = new vcache[vcount];
				tcache *tc = new tcache[tcount];

				f_lru lru(vc, tc);

				for(u16 i=0;i < vcount;i++) {
					vc[i].score = 0;
					vc[i].cachePos = -1;
					vc[i].numActiveTris = 0;
				}

				for(u32 i=0;i < icount;i+=3) {
					vc[ind[i + 0]].numActiveTris++;
					vc[ind[i + 1]].numActiveTris++;
					vc[ind[i + 2]].numActiveTris++;

					const u32 tri_ind = i/3;
					tc[tri_ind].ind[0] = ind[i + 0];
					tc[tri_ind].ind[1] = ind[i + 1];
					tc[tri_ind].ind[2] = ind[i + 2];
				}

				for(u32 i=0;i < tcount;i++) {
					vc[tc[i].ind[0]].tris.push_back(i);
					vc[tc[i].ind[1]].tris.push_back(i);
					vc[tc[i].ind[2]].tris.push_back(i);

					tc[i].drawn = false;
				}

				for(u16 i=0;i < vcount;i++)
					vc[i].score = findVertexScore(&vc[i]);

				for(u32 i=0;i < tcount;i++)
					tc[i].score = vc[tc[i].ind[0]].score + vc[tc[i].ind[1]].score + vc[tc[i].ind[2]].score;

				switch(mb->getVertexType()) {
					case video::EVT_STANDARD:
					{
						video::S3DVertexStandard *v = (video::S3DVertexStandard*)mb->getVertices();

						SMeshBuffer *buf = new SMeshBuffer();
						buf->material = mb->getMaterial();

						buf->vertices.reallocate(vcount);
						buf->indices.reallocate(icount);

						core::map< const video::S3DVertexStandard, const u16 > sind;
						typedef core::map< const video::S3DVertexStandard, const u16 >::Node snode;

						u32 highest = 0;
						u32 drawcalls = 0;
						for(;;) {
							if(tc[highest].drawn) {
								bool found = false;
								f32 hiscore = 0.0f;

								for(u32 t=0;t < tcount;t++) {
									if(!tc[t].drawn) {
										if(tc[t].score > hiscore) {
											highest = t;
											hiscore = tc[t].score;
											found = true;
										}
									}
								}
								if(!found) break;
							}
							u16 newind = buf->vertices.size();
							snode *s = sind.find(v[tc[highest].ind[0]]);

							if(!s) {
								buf->vertices.push_back(v[tc[highest].ind[0]]);
								buf->indices.push_back(newind);
								sind.insert(v[tc[highest].ind[0]], newind);
								newind++;
							} else
								buf->indices.push_back(s->getValue());

							s = sind.find(v[tc[highest].ind[1]]);
							if(!s) {
								buf->vertices.push_back(v[tc[highest].ind[1]]);
								buf->indices.push_back(newind);
								sind.insert(v[tc[highest].ind[1]], newind);
								newind++;
							} else
								buf->indices.push_back(s->getValue());

							s = sind.find(v[tc[highest].ind[2]]);
							if(!s) {
								buf->vertices.push_back(v[tc[highest].ind[2]]);
								buf->indices.push_back(newind);
								sind.insert(v[tc[highest].ind[2]], newind);
								newind++;
							} else
								buf->indices.push_back(s->getValue());

							vc[tc[highest].ind[0]].numActiveTris++;
							vc[tc[highest].ind[1]].numActiveTris++;
							vc[tc[highest].ind[2]].numActiveTris++;

							tc[highest].drawn = true;

							for(u16 j=0;j < 3;j++) {
								vcache *vert = &vc[tc[highest].ind[j]];
								for(u16 t=0;t < vert->tris.size();t++) {
									if(highest == vert->tris[t]) {
										vert->tris.erase(t);
										break;
									}
								}
							}

							lru.add(tc[highest].ind[0]);
							lru.add(tc[highest].ind[1]);
							highest = lru.add(tc[highest].ind[2], true);

							drawcalls++;
						}

						buf->setBoundingBox(mb->getBoundingBox());
						newMesh->addMeshBuffer(buf);
						buf->drop();
					}
					break;

					default:
						break;
				}

				delete [] vc;
				delete [] tc;
			}

			return newMesh;
		}
	}
}
