/*
 * meshscenenode.cpp
 *
 *  Created on: Feb 25, 2013
 *      Author: mike
 */

#include "meshscenenode.h"
#include "ivideodriver.h"
#include "iscenemanager.h"
#include "s3dvertex.h"
#include "icamerascenenode.h"
#include "imeshcache.h"
#include "ianimatedmesh.h"
#include "imaterialrenderer.h"
#include "shadowvolumescenenode.h"

namespace irr
{
	namespace scene
	{
		CMeshSceneNode::CMeshSceneNode(IMesh *mesh, ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, const core::vector3df& rot, const core::vector3df& scale)
		: IMeshSceneNode(parent, mgr, pos, rot, scale), _mesh(NULL), _shadow(NULL),
		  _passCount(0), _readOnlyMaterials(false)
		{
			setMesh(mesh);
		}

		CMeshSceneNode::~CMeshSceneNode()
		{
			if(_shadow != NULL) _shadow->drop();
			if(_mesh != NULL) _mesh->drop();
		}

		void CMeshSceneNode::onRegisterSceneNode()
		{
			if(_isVisible) {
				s32 solidCount = 0;
				s32 transparentCount = 0;
				video::IVideoDriver *driver = _sceneManager->getVideoDriver();

				_passCount = 0;

				if(_readOnlyMaterials && _mesh != NULL) {
					for(u32 i=0;i < _mesh->getMeshBufferCount();i++) {
						scene::IMeshBuffer *mb = _mesh->getMeshBuffer(i);
						video::IMaterialRenderer *rnd = mb != NULL ? driver->getMaterialRenderer(mb->getMaterial().materialType) : NULL;

						if(rnd != NULL && rnd->isTransparent())
							transparentCount++;
						else
							solidCount++;

						if(solidCount && transparentCount)
							break;
					}
				} else {
					for(u32 i=0;i < _materials.size();i++) {
						video::IMaterialRenderer *rnd = driver->getMaterialRenderer(_materials[i].materialType);

						if(rnd != NULL && rnd->isTransparent())
							transparentCount++;
						else
							solidCount++;

						if(solidCount && transparentCount)
							break;
					}
				}

				if(solidCount) _sceneManager->registerNodeForRendering(this, scene::ESNRP_SOLID);
				if(transparentCount) _sceneManager->registerNodeForRendering(this, ESNRP_TRANSPARENT);

				ISceneNode::onRegisterSceneNode();
			}
		}

		void CMeshSceneNode::render()
		{
			video::IVideoDriver *driver = _sceneManager->getVideoDriver();

			if(driver == NULL || _mesh == NULL) return;

			bool isTransparentPass = _sceneManager->getSceneNodeRenderPass() == scene::ESNRP_TRANSPARENT;

			_passCount++;

			driver->setTransform(video::ETS_WORLD, _absTransformation);
			_bbox = _mesh->getBoundingBox();

			for(u32 i=0;i < _mesh->getMeshBufferCount();i++) {
				scene::IMeshBuffer *mb = _mesh->getMeshBuffer(i);

				if(mb != NULL) {
					const video::SMaterial& material = _readOnlyMaterials ? mb->getMaterial() : _materials[i];
					video::IMaterialRenderer *rnd = driver->getMaterialRenderer(material.materialType);
					bool transparent = (rnd != NULL && rnd->isTransparent());

					if(transparent == isTransparentPass) {
						driver->setMaterial(material);
						driver->drawMeshBuffer(mb);
					}
				}
			}
		}

		bool CMeshSceneNode::removeChild(ISceneNode *child)
		{
			return ISceneNode::removeChild(child);
		}

		const core::aabbox3df& CMeshSceneNode::getBoundingBox() const
		{
			return _mesh != NULL ? _mesh->getBoundingBox() : _bbox;
		}

		video::SMaterial& CMeshSceneNode::getMaterial(u32 i)
		{
			if(_mesh != NULL && _readOnlyMaterials && i < _mesh->getMeshBufferCount()) {
				_readOnlyMaterial = _mesh->getMeshBuffer(i)->getMaterial();
				return _readOnlyMaterial;
			}

			if(i >= _materials.size())
				return ISceneNode::getMaterial(i);

			return _materials[i];
		}

		u32 CMeshSceneNode::getMaterialCount() const
		{
			if(_mesh != NULL && _readOnlyMaterials)
				return _mesh->getMeshBufferCount();

			return _materials.size();
		}

		void CMeshSceneNode::setMesh(IMesh *mesh)
		{
			if(mesh != NULL) {
				mesh->grab();

				if(_mesh) _mesh->drop();
				_mesh = mesh;

				copyMaterials();
			}
		}

		IShadowVolumeSceneNode* CMeshSceneNode::addShadowVolumeSceneNode(const IMesh *shadowMesh, bool zFailMethod, f32 infinity)
		{
			if(shadowMesh == NULL)
				shadowMesh = _mesh;

			if(_shadow != NULL)
				_shadow->drop();

			_shadow = new CShadowVolumeSceneNode(shadowMesh, this, _sceneManager, zFailMethod, infinity);
			return _shadow;
		}

		void CMeshSceneNode::copyMaterials()
		{
			_materials.clear();

			if(_mesh != NULL) {
				video::SMaterial material;

				for(u32 i=0;i < _mesh->getMeshBufferCount();i++) {
					scene::IMeshBuffer *mb = _mesh->getMeshBuffer(i);

					if(mb != NULL) material = mb->getMaterial();

					_materials.push_back(material);
				}
			}
		}


		void CMeshSceneNode::setReadOnlyMaterials(bool readonly)
		{
			_readOnlyMaterials = readonly;
		}

		bool CMeshSceneNode::isReadOnlyMaterials() const
		{
			return _readOnlyMaterials;
		}


	}

}

