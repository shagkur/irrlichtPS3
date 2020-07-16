/*
 * iscenemanager.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef ISCENEMANAGER_H_
#define ISCENEMANAGER_H_

#include "irefcounter.h"
#include "irrarray.h"
#include "vector3d.h"
#include "scolor.h"
#include "path.h"
#include "dimension2d.h"
#include "escenenodetypes.h"
#include "igeometrycreator.h"

namespace irr
{
	struct SEvent;

	namespace video
	{
		class IVideoDriver;
		class SMaterial;
		class IImage;
		class ITexture;
	}

	namespace io
	{
		class IReadFile;
		class IFileSystem;
	}

	namespace scene
	{
		class IAnimatedMesh;
		class ISceneNode;
		class IMesh;
		class IMeshBuffer;
		class IMeshCache;
		class IMeshSceneNode;
		class ILightSceneNode;
		class ICameraSceneNode;
		class ISceneNodeAnimator;
		class IMeshLoader;
		class IMeshManipulator;
		class IAnimatedMeshSceneNode;
		class IBillboardSceneNode;
		class IPhysicsManager;

		enum E_SCENE_NODE_RENDER_PASS
		{
			ESNRP_NONE = 0,

			//! Camera pass. The active view is set up here. The very first pass.
			ESNRP_CAMERA = 1,

			//! In this pass, lights are transformed into camera space and added to the driver
			ESNRP_LIGHT = 2,

			//! This is used for sky boxes.
			ESNRP_SKY_BOX = 4,

			//! All normal objects can use this for registering themselves.
			/** This value will never be returned by
			ISceneManager::getSceneNodeRenderPass(). The scene manager
			will determine by itself if an object is transparent or solid
			and register the object as SNRT_TRANSPARENT or SNRT_SOLD
			automatically if you call registerNodeForRendering with this
			value (which is default). Note that it will register the node
			only as ONE type. If your scene node has both solid and
			transparent material types register it twice (one time as
			SNRT_SOLID, the other time as SNRT_TRANSPARENT) and in the
			render() method call getSceneNodeRenderPass() to find out the
			current render pass and render only the corresponding parts of
			the node. */
			ESNRP_AUTOMATIC = 24,

			//! Solid scene nodes or special scene nodes without materials.
			ESNRP_SOLID = 8,


			//! Transparent scene nodes, drawn after shadow nodes. They are sorted from back to front and drawn in that order.
			ESNRP_TRANSPARENT = 16,

			//! Transparent scene nodes, drawn after shadow nodes. They are sorted from back to front and drawn in that order.
			ESNRP_TRANSPARENT_EFFECT = 32,

			//! Never used, value specifing how much parameters there are.
			//! Drawn after the transparent nodes, the time for drawing shadow volumes
			ESNRP_SHADOW = 64
		};

		class ISceneManager : public virtual IRefCounter
		{
		public:
			virtual IAnimatedMesh* getMesh(const io::path& filename) = 0;
			virtual IAnimatedMesh* getMesh(io::IReadFile *file) = 0;

			virtual IMeshCache* getMeshCache() = 0;

			virtual ICameraSceneNode* addCameraSceneNode(ISceneNode *parent = NULL, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), const core::vector3df& lookat = core::vector3df(0.0f, 0.0f, 100.0f), bool makeActive = true) = 0;
			virtual ILightSceneNode* addLightSceneNode(ISceneNode *parent = NULL, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), video::SColorf color = video::SColorf(1.0f, 1.0f, 1.0f, 1.0f), f32 radius = 100.0f) = 0;
			virtual IMeshSceneNode* addMeshSceneNode(IMesh *mesh, ISceneNode *parent = NULL, const core::vector3df& pos = core::vector3df(0, 0, 0), const core::vector3df& rot = core::vector3df(0, 0, 0), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f), bool alsoAddIfMeshPointerZero = false) = 0;
			virtual ICameraSceneNode* addCameraSceneNodeFPS(ISceneNode *parent = NULL, f32 rotateSpeed = 25.0f, f32 moveSpeed = 0.5f, bool noVerticalMovement = false, f32 jumpSpeed = 0.0f, bool invertY = false, bool makeActive = true) = 0;
			virtual IAnimatedMeshSceneNode* addAnimatedMeshSceneNode(IAnimatedMesh *mesh, ISceneNode *parent = NULL, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), const core::vector3df& rot = core::vector3df(0.0f, 0.0f, 0.0f), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f), bool alsoAddIfMeshPointerZero = false) = 0;
			virtual IBillboardSceneNode* addBillboardSceneNode(ISceneNode *parent = NULL, const core::dimension2df& size = core::dimension2df(10.0f, 10.0f), const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), video::SColor topColor = 0xffffffff, video::SColor bottomColor = 0xffffffff) = 0;

			virtual ISceneNodeAnimator* createRotationAnimator(const core::vector3df& rotationSpeed) = 0;
			virtual ISceneNodeAnimator* createFlyCircleAnimator(const core::vector3df& center = core::vector3df(0.0f, 0.0f, 0.0f), f32 radius = 100.0f, f32 speed = 0.001f, const core::vector3df& direction = core::vector3df(0.0f, 1.0f, 0.0f), f32 startPosition = 0.0f, f32 radiusEllipsoid = 0.0f) = 0;

			virtual void drawAll() = 0;

			virtual void setActiveCamera(ICameraSceneNode *camera) = 0;

			virtual u32 registerNodeForRendering(ISceneNode *node,E_SCENE_NODE_RENDER_PASS pass = ESNRP_AUTOMATIC) = 0;

			virtual ICameraSceneNode* getActiveCamera() const = 0;

			virtual ISceneNode* getRootSceneNode() = 0;

			virtual video::IVideoDriver* getVideoDriver() = 0;

			virtual const IGeometryCreator* getGeometryCreator() const = 0;

			virtual IMeshManipulator* getMeshManipulator() = 0;

			virtual IPhysicsManager* getPhysicsManager() const = 0;

			virtual bool isCulled(const ISceneNode *node) const = 0;

			virtual void setAmbientLight(const video::SColorf& ambientColor) = 0;

			virtual const video::SColorf& getAmbientLight() const = 0;

			virtual bool postEventFromUser(const SEvent& event) = 0;

			virtual E_SCENE_NODE_RENDER_PASS getSceneNodeRenderPass() const = 0;
		};
	}
}

#endif /* ISCENEMANAGER_H_ */
