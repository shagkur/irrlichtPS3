/*
 * geometrycreator.cpp
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#include "geometrycreator.h"
#include "smeshbuffer.h"
#include "smesh.h"
#include "imesh.h"
#include "os.h"

namespace irr
{
	namespace scene
	{
		IMesh* CGeometryCreator::createCubeMesh(const core::vector3df& size) const
		{
			video::SColor clr(255,255,255,255);
			SMeshBuffer *buffer = new SMeshBuffer();
			const u16 u[36] = { 0,1,2,   2,3,0,   1,4,5,   5,2,1,   4,7,6,	 6,5,4,
								7,0,3,   3,6,7,   9,2,5,   5,8,9,   11,0,7,  7,10,11 };

			buffer->indices.set_used(36);
			for(u32 i=0;i<36;i++) buffer->indices[i] = u[i];

			buffer->vertices.reallocate(12);
			buffer->vertices.push_back(video::S3DVertexStandard(0,0,0, -1,-1,-1, clr, 1, 0));
			buffer->vertices.push_back(video::S3DVertexStandard(1,0,0,  1,-1,-1, clr, 1, 1));
			buffer->vertices.push_back(video::S3DVertexStandard(1,1,0,  1, 1,-1, clr, 0, 1));
			buffer->vertices.push_back(video::S3DVertexStandard(0,1,0, -1, 1,-1, clr, 0, 0));
			buffer->vertices.push_back(video::S3DVertexStandard(1,0,1,  1,-1, 1, clr, 1, 0));
			buffer->vertices.push_back(video::S3DVertexStandard(1,1,1,  1, 1, 1, clr, 0, 0));
			buffer->vertices.push_back(video::S3DVertexStandard(0,1,1, -1, 1, 1, clr, 0, 1));
			buffer->vertices.push_back(video::S3DVertexStandard(0,0,1, -1,-1, 1, clr, 1, 1));
			buffer->vertices.push_back(video::S3DVertexStandard(0,1,1, -1, 1, 1, clr, 1, 0));
			buffer->vertices.push_back(video::S3DVertexStandard(0,1,0, -1, 1,-1, clr, 1, 1));
			buffer->vertices.push_back(video::S3DVertexStandard(1,0,1,  1,-1, 1, clr, 0, 1));
			buffer->vertices.push_back(video::S3DVertexStandard(1,0,0,  1,-1,-1, clr, 0, 0));

			buffer->bbox.reset(0.0f, 0.0f, 0.0f);

			for(u32 i=0;i < 12;i++) {
				buffer->vertices[i].pos -= core::vector3df(0.5f, 0.5f, 0.5f);
				buffer->vertices[i].pos = mulPerElem(buffer->vertices[i].pos, size);
				buffer->bbox.addInternalPoint(buffer->vertices[i].pos);
			}

			SMesh *mesh = new SMesh();
			mesh->addMeshBuffer(buffer);
			buffer->drop();

			mesh->recalculateBoundingBox();
			return mesh;
		}

		IMesh* CGeometryCreator::createSphereMesh(f32 radius, u32 polyCountX, u32 polyCountY) const
		{
			if(polyCountX<2) polyCountX = 2;
			if(polyCountY<2) polyCountY = 2;
			while(polyCountX*polyCountY > 32767) {
				polyCountX /= 2;
				polyCountY /= 2;
			}

			const u32 polyCountXpitch = polyCountX + 1;
			SMeshBuffer *buffer = new SMeshBuffer();

			buffer->indices.reallocate((polyCountX*polyCountY)*6);

			u32 level = 0;
			const video::SColor clr(255, 255,255,255);
			for(u32 p1=0;p1 < polyCountY - 1;p1++) {
				for(u32 p2=0;p2 < polyCountX - 1;p2++) {
					const u32 curr = level + p2;
					buffer->indices.push_back(curr);
					buffer->indices.push_back(curr + polyCountXpitch);
					buffer->indices.push_back(curr + 1 + polyCountXpitch);

					buffer->indices.push_back(curr);
					buffer->indices.push_back(curr + 1 + polyCountXpitch);
					buffer->indices.push_back(curr + 1);
				}

				buffer->indices.push_back(level + polyCountX);
				buffer->indices.push_back(level + polyCountX - 1);
				buffer->indices.push_back(level + polyCountX - 1 + polyCountXpitch);

				buffer->indices.push_back(level + polyCountX);
				buffer->indices.push_back(level + polyCountX - 1 + polyCountXpitch);
				buffer->indices.push_back(level + polyCountX + polyCountXpitch);

				level += polyCountXpitch;
			}

			const u32 polyCntSq = polyCountXpitch*polyCountY;
			const u32 polyCntSq1 = polyCntSq + 1;
			const u32 polyCntSqM1 = (polyCountY - 1)*polyCountXpitch;
			for(u32 p2=0;p2 < polyCountX - 1;p2++) {
				buffer->indices.push_back(polyCntSq);
				buffer->indices.push_back(p2);
				buffer->indices.push_back(p2 + 1);

				buffer->indices.push_back(polyCntSq1);
				buffer->indices.push_back(polyCntSqM1 + p2);
				buffer->indices.push_back(polyCntSqM1 + p2 + 1);
			}

			buffer->indices.push_back(polyCntSq);
			buffer->indices.push_back(polyCountX - 1);
			buffer->indices.push_back(polyCountX);

			buffer->indices.push_back(polyCntSq1);
			buffer->indices.push_back(polyCntSqM1 + polyCountX - 1);
			buffer->indices.push_back(polyCntSqM1);

			f32 axz;
			u32 i = 0;
			f32 ay = 0;
			const f32 angelX = 2*core::PI/polyCountX;
			const f32 angelY = core::PI/polyCountY;

			buffer->vertices.set_used((polyCountXpitch*polyCountY) + 2);
			for(u32 y=0;y < polyCountY;y++) {
				axz = 0;
				ay += angelY;
				const f32 sinay = sinf(ay);
				for(u32 x=0;x < polyCountX;x++) {
					const core::vector3df pos(radius*cosf(axz)*sinay,
											  radius*cosf(ay),
											  radius*sinf(axz)*sinay);
					const core::vector3df nrm = normalize(pos);

					f32 tu = 0.5f;
					if(y == 0) {
						if(nrm.getY() != -1.0f && nrm.getY() != 1.0f)
							tu = acosf(core::clamp(nrm.getX()/sinay, -1.0f, 1.0f))*0.5f*core::RECIPROCAL_PI;
						if(nrm.getZ() < 0.0f)
							tu = 1 - tu;
					} else
						tu = buffer->vertices[i - polyCountXpitch].tcoords.X;

					buffer->vertices[i] = video::S3DVertexStandard(pos, nrm, clr, core::vector2df(tu, ay*core::RECIPROCAL_PI));
					axz += angelX;
					i++;
				}

				buffer->vertices[i] = video::S3DVertexStandard(buffer->vertices[i - polyCountX]);
				buffer->vertices[i].tcoords.X = 1.0f;
				i++;
			}

			buffer->vertices[i] = video::S3DVertexStandard(0.0f, radius, 0.0f, 0.0f, 1.0f, 0.0f, clr, 0.5f, 0.0f);
			buffer->vertices[i + 1] = video::S3DVertexStandard(0.0f, -radius, 0.0f, 0.0f, -1.0f, 0.0f, clr, 0.5f, 1.0f);

			buffer->bbox.reset(buffer->vertices[i + 1].pos);
			buffer->bbox.addInternalPoint(buffer->vertices[i].pos);
			buffer->bbox.addInternalPoint(radius, 0.0f, 0.0f);
			buffer->bbox.addInternalPoint(-radius, 0.0f, 0.0f);
			buffer->bbox.addInternalPoint(0.0f, 0.0f, radius);
			buffer->bbox.addInternalPoint(0.0f, 0.0f, -radius);

			SMesh *mesh = new SMesh();
			mesh->addMeshBuffer(buffer);
			buffer->drop();

			mesh->setHardwareMappingHint(EHM_STATIC);
			mesh->recalculateBoundingBox();
			return mesh;
		}

		IMesh* CGeometryCreator::createTorusMesh(f32 outerRadius, f32 innerRadius, u32 polyCountX, u32 polyCountY) const
		{
			if(polyCountX<2) polyCountX = 2;
			if(polyCountY<2) polyCountY = 2;
			while(polyCountX*polyCountY > 32767) {
				polyCountX /= 2;
				polyCountY /= 2;
			}

			const u32 polyCountXpitch = polyCountX + 1;
			const u32 polyCountYpitch = polyCountY + 1;
			SMeshBuffer *buffer = new SMeshBuffer();

			buffer->indices.reallocate(polyCountYpitch*polyCountXpitch*6);

			u32 level = 0;
			video::SColor clr(255,255,255,255);
			for(u32 p1=0;p1 < polyCountY;p1++) {
				for(u32 p2=0;p2 < polyCountX;p2++) {
					const u32 curr = level + p2;
					buffer->indices.push_back(curr);
					buffer->indices.push_back(curr + polyCountXpitch);
					buffer->indices.push_back(curr + polyCountXpitch + 1);

					buffer->indices.push_back(curr + polyCountXpitch + 1);
					buffer->indices.push_back(curr + 1);
					buffer->indices.push_back(curr);
				}

				buffer->indices.push_back(level + polyCountX);
				buffer->indices.push_back(level + polyCountX + polyCountXpitch);
				buffer->indices.push_back(level + polyCountXpitch);

				buffer->indices.push_back(level + polyCountXpitch);
				buffer->indices.push_back(level);
				buffer->indices.push_back(level + polyCountX);

				level += polyCountXpitch;
			}

			const u32 polyCntSq = polyCountXpitch*polyCountY;
			for(u32 p2=0;p2 < polyCountX;p2++) {
				buffer->indices.push_back(polyCntSq + p2);
				buffer->indices.push_back(p2);
				buffer->indices.push_back(p2 + 1);

				buffer->indices.push_back(p2 + 1);
				buffer->indices.push_back(polyCntSq + p2 + 1);
				buffer->indices.push_back(polyCntSq + p2);
			}

			buffer->indices.push_back(polyCntSq + polyCountX);
			buffer->indices.push_back(polyCountX);
			buffer->indices.push_back(0);

			buffer->indices.push_back(0);
			buffer->indices.push_back(polyCntSq);
			buffer->indices.push_back(polyCntSq + polyCountX);

			u32 i = 0;
			f32 ay = 0.0f;
			const f32 angleX = (2.0f*core::PI)/(f32)polyCountX;
			const f32 angleY = (2.0f*core::PI)/(f32)polyCountY;

			buffer->vertices.set_used(polyCountYpitch*polyCountXpitch);
			for(u32 y=0;y <= polyCountY;y++) {
				f32 axz = 0.0f;
				const f32 sinay = sinf(ay);
				const f32 cosay = cosf(ay);
				const f32 tu = (f32)y/(f32)polyCountY;
				for(u32 x=0;x <= polyCountX;x++) {
					const core::vector3df pos(sinay*(outerRadius + innerRadius*cosf(axz)),
											  cosay*(outerRadius + innerRadius*cosf(axz)),
											  innerRadius*sinf(axz));
					const core::vector3df nrm(sinay*cosf(axz),
											  cosay*cosf(axz),
											  sinf(axz));

					buffer->vertices[i] = video::S3DVertexStandard(pos, nrm, clr, core::vector2df(tu, (f32)x/(f32)polyCountX));
					axz += angleX;
					i++;
				}
				ay += angleY;
			}
			buffer->setBoundingBox(core::aabbox3df(-outerRadius - innerRadius, -outerRadius - innerRadius, -innerRadius,
												    outerRadius + innerRadius,  outerRadius + innerRadius,  innerRadius));

			SMesh *mesh = new SMesh();
			mesh->addMeshBuffer(buffer);
			buffer->drop();

			mesh->setHardwareMappingHint(EHM_STATIC);
			mesh->recalculateBoundingBox();
			return mesh;
		}
	}
}
