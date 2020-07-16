/*
 * shadowvolumescenenode.cpp
 *
 *  Created on: Jul 16, 2013
 *      Author: mike
 */

#include "shadowvolumescenenode.h"
#include "iscenemanager.h"
#include "imesh.h"
#include "ivideodriver.h"
#include "icamerascenenode.h"
#include "sviewfrustum.h"
#include "slight.h"
#include "os.h"

namespace irr
{
	namespace scene
	{
		CShadowVolumeSceneNode::CShadowVolumeSceneNode(const IMesh *shadowMesh, ISceneNode *parent, ISceneManager *mgr, bool zFailMethod, f32 infinity)
		: IShadowVolumeSceneNode(parent, mgr), _shadowMesh(NULL), _infinity(infinity), _useZFailMethod(zFailMethod)
		{
			setShadowMesh(shadowMesh);
			setAutomaticCulling(scene::EAC_OFF);
		}

		CShadowVolumeSceneNode::~CShadowVolumeSceneNode()
		{
			if(_shadowMesh)
				_shadowMesh->drop();
		}

		void CShadowVolumeSceneNode::render()
		{
			video::IVideoDriver *driver = _sceneManager->getVideoDriver();

			if(!driver)
				return;

			const IMesh* const mesh = _shadowMesh;
			if(!mesh)
				return;

			const u32 lightCount = driver->getDynamicLightCount();
			if(!lightCount)
				return;

			driver->setTransform(video::ETS_WORLD, _parent->getAbsoluteTransformation());

			/*
			const core::vector3df& parentpos = _parent->getAbsolutePosition();
			const core::matrix4& modelIT = inverse(driver->getTransform(video::ETS_WORLD));
			for(s32 l=lightCount - 1;l >= 0;--l) {
				const video::SLight& light = driver->getDynamicLight(l);
				core::vector3df lpos = light.position;
				core::vector3df ldir = light.direction;

				if(light.castShadows) {
					core::vector4df pos;

					if(light.type == video::ELT_DIRECTIONAL) {
						core::rotateVect(modelIT, ldir);
						pos = core::vector4df(ldir, 0.0f);
					}
					else if((light.type == video::ELT_POINT || light.type == video::ELT_SPOT) &&
						fabsf(lengthSqr(lpos - parentpos)) <= (light.radius*light.radius*4.0f))
					{
						core::transformVect(modelIT, lpos);
						pos = core::vector4df(lpos, 1.0f);
					}

					driver->drawStencilShadowVolume(_shadowVolume.const_pointer(), _shadowVolume.size(), pos, _infinity, _useZFailMethod);
				}
			}
			*/

			const video::SLight& light = driver->getActiveLight();
			const core::vector3df& parentpos = _parent->getAbsolutePosition();
			const core::matrix4& modelIT = inverse(driver->getTransform(video::ETS_WORLD));
			if(light.castShadows) {
				core::vector4df pos;
				core::vector3df lpos = light.position;
				core::vector3df ldir = light.direction;

				if(light.type == video::ELT_DIRECTIONAL) {
					core::rotateVect(modelIT, ldir);
					pos = core::vector4df(ldir, 0.0f);
				}
				else if((light.type == video::ELT_POINT || light.type == video::ELT_SPOT) &&
					fabsf(lengthSqr(lpos - parentpos)) <= (light.radius*light.radius*4.0f))
				{
					core::transformVect(modelIT, lpos);
					pos = core::vector4df(lpos, 1.0f);
				}

				driver->drawStencilShadowVolume(_shadowVolume.const_pointer(), _shadowVolume.size(), pos, _infinity, _useZFailMethod);
			}
		}

		void CShadowVolumeSceneNode::onRegisterSceneNode()
		{
			if(_isVisible) {
				_sceneManager->registerNodeForRendering(this, scene::ESNRP_SHADOW);
				ISceneNode::onRegisterSceneNode();
			}
		}

		void CShadowVolumeSceneNode::setShadowMesh(const IMesh *mesh)
		{
			if(_shadowMesh == mesh)
				return;
			if(_shadowMesh)
				_shadowMesh->drop();

			_shadowMesh = mesh;
			if(_shadowMesh) {
				_shadowMesh->grab();
				_bbox = _shadowMesh->getBoundingBox();

				createShadowVolumeMesh();
			}
		}

		const core::aabbox3df& CShadowVolumeSceneNode::getBoundingBox() const
		{
			return _bbox;
		}

		void CShadowVolumeSceneNode::createShadowVolumeMesh()
		{
			const IMesh* const mesh = _shadowMesh;
			if(!mesh)
				return;

			u32 totalVertices = 0;
			u32 totalIndices = 0;
			core::array< u16 > indices;
			core::array< core::vector3df > vertices;
			const u32 bufcnt = mesh->getMeshBufferCount();
			for(u32 i=0;i < bufcnt;i++) {
				const IMeshBuffer *mb = mesh->getMeshBuffer(i);
				totalIndices += mb->getIndexCount();
				totalVertices += mb->getVertexCount();
			}

			vertices.set_used(totalVertices);
			indices.set_used(totalIndices);

			u32 indexCount = 0;
			u32 vertexCount = 0;
			for(u32 i=0;i < bufcnt;i++) {
				const IMeshBuffer *mb = mesh->getMeshBuffer(i);
				const u16 *idxp = mb->getIndices();
				const u16 *idxpend = idxp + mb->getIndexCount();

				for(;idxp != idxpend;idxp++)
					indices[indexCount++] = *idxp + vertexCount;

				const u32 vtxcnt = mb->getVertexCount();
				for(u32 j=0;j < vtxcnt;j++)
					vertices[vertexCount++] = mb->getPosition(j);
			}

			_shadowVolume.set_used(indexCount);

			core::array< Edge > pairs;
			const u32 faceCount = indexCount/3;
			for(u32 i=0;i < faceCount;i++) {
				const core::vector3df& v0 = vertices[indices[i*3 + 0]];
				const core::vector3df& v1 = vertices[indices[i*3 + 1]];
				const core::vector3df& v2 = vertices[indices[i*3 + 2]];
				const core::vector3df& normal = core::plane3df(v0, v1, v2).normal;

				_shadowVolume[i*3 + 0] = video::S3DVertexBase(v0, normal);
				_shadowVolume[i*3 + 1] = video::S3DVertexBase(v1, normal);
				_shadowVolume[i*3 + 2] = video::S3DVertexBase(v2, normal);

				processEdge(pairs, Edge(v0, v1, normal));
				processEdge(pairs, Edge(v1, v2, normal));
				processEdge(pairs, Edge(v2, v0, normal));
			}

			if(pairs.size() > 0)
				printf("WARNING: not 2-manifold! %d edges left\n", (int)pairs.size());

			printf("SV size: %dKB\n", ((_shadowVolume.size()*video::getVertexPitchFromType(video::EVT_BASE))/1024));
		}

		void CShadowVolumeSceneNode::processEdge(core::array< Edge >& pairs, const Edge& e)
		{
			for(u32 i=0;i < pairs.size();i++) {
				if(pairs[i] == e) {
					_shadowVolume.push_back(video::S3DVertexBase(e.v1, e.normal));
					_shadowVolume.push_back(video::S3DVertexBase(e.v1, pairs[i].normal));
					_shadowVolume.push_back(video::S3DVertexBase(e.v2, e.normal));

					_shadowVolume.push_back(video::S3DVertexBase(e.v2, pairs[i].normal));
					_shadowVolume.push_back(video::S3DVertexBase(e.v2, e.normal));
					_shadowVolume.push_back(video::S3DVertexBase(e.v1, pairs[i].normal));

					pairs.erase(i);
					return;
				}
			}

			pairs.push_back(e);
		}
	}
}
