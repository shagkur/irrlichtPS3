/*
 * scenemanager.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef SCENEMANAGER_H_
#define SCENEMANAGER_H_

#include "iscenemanager.h"
#include "iscenenode.h"
#include "irrarray.h"
#include "imeshloader.h"

#include "rigidbody/rigidbodies.h"

namespace irr
{
	namespace video
	{
		class IVideoDriver;
	}

	namespace io
	{
		class IFileSystem;
	}

	namespace scene
	{
		class IMeshCache;
		class IGeometryCreator;
		class IPhysicsManager;

		class CSceneManager : public ISceneManager, public ISceneNode
		{
		public:
			CSceneManager(video::IVideoDriver *driver, io::IFileSystem *fs, IMeshCache *cache = NULL);
			virtual ~CSceneManager();

			virtual video::IVideoDriver* getVideoDriver();

			virtual IAnimatedMesh* getMesh(const io::path& filename);
			virtual IAnimatedMesh* getMesh(io::IReadFile *file);

			virtual IMeshCache* getMeshCache();

			virtual ICameraSceneNode* addCameraSceneNode(ISceneNode *parent = NULL, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), const core::vector3df& lookat = core::vector3df(0.0f, 0.0f, 100.0f), bool makeActive = true);
			virtual ILightSceneNode* addLightSceneNode(ISceneNode *parent = NULL, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), video::SColorf color = video::SColorf(1.0f, 1.0f, 1.0f, 1.0f), f32 radius = 100.0f);
			virtual IMeshSceneNode* addMeshSceneNode(IMesh *mesh, ISceneNode *parent = NULL, const core::vector3df& pos = core::vector3df(0, 0, 0), const core::vector3df& rot = core::vector3df(0, 0, 0), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f), bool alsoAddIfMeshPointerZero = false);
			virtual ICameraSceneNode* addCameraSceneNodeFPS(ISceneNode *parent = NULL, f32 rotateSpeed = 25.0f, f32 moveSpeed = 0.5f, bool noVerticalMovement = false, f32 jumpSpeed = 0.0f, bool invertY = false, bool makeActive = true);
			virtual IAnimatedMeshSceneNode* addAnimatedMeshSceneNode(IAnimatedMesh *mesh, ISceneNode *parent = NULL, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), const core::vector3df& rot = core::vector3df(0.0f, 0.0f, 0.0f), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f), bool alsoAddIfMeshPointerZero = false);
			virtual IBillboardSceneNode* addBillboardSceneNode(ISceneNode *parent = NULL, const core::dimension2df& size = core::dimension2df(10.0f, 10.0f), const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), video::SColor topColor = 0xffffffff, video::SColor bottomColor = 0xffffffff);

			virtual ISceneNodeAnimator* createRotationAnimator(const core::vector3df& rotationSpeed);
			virtual ISceneNodeAnimator* createFlyCircleAnimator(const core::vector3df& center = core::vector3df(0.0f, 0.0f, 0.0f), f32 radius = 100.0f, f32 speed = 0.001f, const core::vector3df& direction = core::vector3df(0.0f, 1.0f, 0.0f), f32 startPosition = 0.0f, f32 radiusEllipsoid = 0.0f);

			virtual void setActiveCamera(ICameraSceneNode *camera);

			virtual void render();

			virtual void drawAll();

			virtual ICameraSceneNode* getActiveCamera() const;

			virtual u32 registerNodeForRendering(ISceneNode *node,E_SCENE_NODE_RENDER_PASS pass = ESNRP_AUTOMATIC);

			virtual ISceneNode* getRootSceneNode();

			virtual IMeshManipulator* getMeshManipulator();

			virtual IPhysicsManager* getPhysicsManager() const { return _physicsManager; }

			virtual const IGeometryCreator* getGeometryCreator() const { return _geometryCreator; }

			virtual const core::aabbox3df& getBoundingBox() const;

			virtual bool isCulled(const ISceneNode *node) const;

			virtual void setAmbientLight(const video::SColorf& ambientColor);

			virtual const video::SColorf& getAmbientLight() const;

			virtual bool postEventFromUser(const SEvent& event);

			virtual E_SCENE_NODE_RENDER_PASS getSceneNodeRenderPass() const;

		private:
			struct DefaultNodeEntry
			{
				DefaultNodeEntry(ISceneNode *n) : node(n), textureValue(NULL)
				{
					if(n->getMaterialCount())
						textureValue = n->getMaterial(0).getTexture(0);
				}

				bool operator < (const DefaultNodeEntry& other) const
				{
					return (textureValue < other.textureValue);
				}

				ISceneNode *node;
			private:
				void *textureValue;
			};

			struct TransparentNodeEntry
			{
				TransparentNodeEntry(ISceneNode *n, const core::vector3df& cameraPos) : node(n)
				{
					distance = lengthSqr(node->getAbsoluteTransformation().getTranslation() - cameraPos);
				}

				bool operator < (const TransparentNodeEntry& other) const
				{
					return distance > other.distance;
				}

				ISceneNode *node;

			private:
				f32 distance;
			};

			struct DistanceNodeEntry
			{
				DistanceNodeEntry(ISceneNode *n, const core::vector3df& cameraPos) : node(n)
				{
					setNodeAndDistanceFromPosition(n, cameraPos);
				}

				bool operator < (const DistanceNodeEntry& other) const
				{
					return distance < other.distance;
				}

				void setNodeAndDistanceFromPosition(ISceneNode *n, const core::vector3df& fromPosition)
				{
					node = n;
					distance = lengthSqr(node->getAbsoluteTransformation().getTranslation() - fromPosition);
					distance -= lengthSqr(node->getBoundingBox().getExtent()) * 0.5f;
				}

				ISceneNode *node;

			private:
				f32 distance;
			};

			core::array< ISceneNode* > _cameraList;
			core::array< ISceneNode* > _lightList;
			core::array< ISceneNode* > _shadowNodeList;
			core::array< DefaultNodeEntry > _solidNodeList;
			core::array< TransparentNodeEntry > _transparentNodeList;

			core::array< IMeshLoader* > _meshLoaderList;

			video::IVideoDriver *_driver;
			io::IFileSystem *_fileSystem;
			ICameraSceneNode *_activeCamera;

			video::SColor _shadowColor;

			core::vector3df _camWorldPos;

			video::SColorf _ambientLight;

			IMeshCache *_meshCache;

			E_SCENE_NODE_RENDER_PASS _currentRenderTime;

			IGeometryCreator *_geometryCreator;

			IPhysicsManager *_physicsManager;
		};
	}
}


#endif /* SCENEMANAGER_H_ */
