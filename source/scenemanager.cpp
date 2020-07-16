/*
 * scenemanager.cpp
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#include "irrtypes.h"
#include "scenemanager.h"
#include "ivideodriver.h"
#include "ifilesystem.h"
#include "imaterialrenderer.h"
#include "sanimatedmesh.h"
#include "meshcache.h"
#include "lightscenenode.h"
#include "camerascenenode.h"
#include "animatedmeshscenenode.h"
#include "meshscenenode.h"
#include "billboardscenenode.h"
#include "geometrycreator.h"
#include "physicsmanager.h"
#include "scenenodeanimatorrotation.h"
#include "scenenodeanimatorflycircle.h"
#include "scenenodeanimatorcamerafps.h"

#include "3dsmeshfileloader.h"

#include "os.h"

namespace irr
{
	namespace scene
	{
		CSceneManager::CSceneManager(video::IVideoDriver *driver, io::IFileSystem *fs, IMeshCache *cache)
		: ISceneNode(NULL, NULL), _driver(driver), _fileSystem(fs), _activeCamera(NULL), _shadowColor(0, 0, 0, 150),
		  _ambientLight(0.0f, 0.0f, 0.0f, 0.0f), _meshCache(cache), _currentRenderTime(ESNRP_NONE), _physicsManager(NULL)
		{
			_sceneManager = this;

			if(_driver) _driver->grab();
			if(_fileSystem) _fileSystem->grab();

			if(_meshCache == NULL)
				_meshCache = new CMeshCache();
			else
				_meshCache->grab();

			_geometryCreator = new CGeometryCreator();

			_physicsManager = new CPhysicsManager(this);

			_meshLoaderList.push_back(new C3DSMeshFileLoader(this, _fileSystem));
		}

		CSceneManager::~CSceneManager()
		{
			if(_fileSystem != NULL) _fileSystem->drop();
			if(_driver != NULL) _driver->drop();
			if(_geometryCreator != NULL) _geometryCreator->drop();

			u32 i;
			for(i=0;i < _meshLoaderList.size();i++)
				_meshLoaderList[i]->drop();
		}

		video::IVideoDriver* CSceneManager::getVideoDriver()
		{
			return _driver;
		}

		IAnimatedMesh* CSceneManager::getMesh(const io::path& filename)
		{
			printf("CSceneManager::getMesh(%s)\n", filename.c_str());
			IAnimatedMesh *mesh = _meshCache->getMeshByName(filename);
			if(mesh != NULL) return mesh;

			printf("CSceneManager::getMesh(opening file)\n");
			io::IReadFile *file = _fileSystem->createAndOpenFile(filename);
			if(file == NULL) return NULL;

			s32 count = _meshLoaderList.size();
			for(s32 i=count - 1;i >= 0;--i) {
				if(_meshLoaderList[i]->isLoadableFileExtension(filename)) {
					file->seek(0);

					printf("CSceneManager::getMesh(creating mesh)\n");
					mesh = _meshLoaderList[i]->createMesh(file);
					if(mesh != NULL) {
						_meshCache->addMesh(filename, mesh);
						mesh->drop();
						break;
					}
				}
			}
			file->drop();

			return mesh;
		}

		IAnimatedMesh* CSceneManager::getMesh(io::IReadFile *file)
		{
			if(file == NULL) return NULL;

			io::path name = file->getFilename();
			IAnimatedMesh *mesh = _meshCache->getMeshByName(name);
			if(mesh != NULL) return mesh;

			s32 count = _meshLoaderList.size();
			for(s32 i=count - 1;i >= 0;--i) {
				if(_meshLoaderList[i]->isLoadableFileExtension(name)) {
					file->seek(0);

					mesh = _meshLoaderList[i]->createMesh(file);
					if(mesh != NULL) {
						_meshCache->addMesh(file->getFilename(), mesh);
						mesh->drop();
						break;
					}
				}
			}
			return mesh;
		}

		ICameraSceneNode* CSceneManager::addCameraSceneNode(ISceneNode *parent, const core::vector3df& pos, const core::vector3df& lookat, bool makeActive)
		{
			if(parent == NULL) parent = this;

			ICameraSceneNode *node = new CCameraSceneNode(parent,this,pos,lookat);
			if(makeActive) setActiveCamera(node);

			node->drop();
			return node;
		}

		ILightSceneNode* CSceneManager::addLightSceneNode(ISceneNode *parent, const core::vector3df& pos, video::SColorf color, f32 radius)
		{
			if(parent == NULL) parent = this;

			ILightSceneNode *node = new CLightSceneNode(parent, this, pos, color, radius);
			node->drop();

			return node;
		}

		IMeshSceneNode* CSceneManager::addMeshSceneNode(IMesh *mesh, ISceneNode *parent, const core::vector3df& pos, const core::vector3df& rot, const core::vector3df& scale, bool alsoAddIfMeshPointerZero)
		{
			if(!alsoAddIfMeshPointerZero && mesh == NULL) return NULL;

			if(parent == NULL) parent = this;

			IMeshSceneNode *node = new CMeshSceneNode(mesh, parent, this, pos, rot, scale);
			node->drop();

			return node;
		}

		ICameraSceneNode* CSceneManager::addCameraSceneNodeFPS(ISceneNode *parent, f32 rotateSpeed, f32 moveSpeed, bool noVerticalMovement, f32 jumpSpeed, bool invertY, bool makeActive)
		{
			ICameraSceneNode *node = addCameraSceneNode(parent, core::vector3df(0.0f, 0.0f, 0.0f), core::vector3df(0.0f, 0.0f, 100.0f), makeActive);

			if(node != NULL) {
				ISceneNodeAnimator *anim = new CSceneNodeAnimatorCameraFPS(rotateSpeed, moveSpeed, jumpSpeed, noVerticalMovement, invertY);

				node->bindTargetAndRotation(true);
				node->addAnimator(anim);
				anim->drop();
			}

			return node;
		}

		IAnimatedMeshSceneNode* CSceneManager::addAnimatedMeshSceneNode(IAnimatedMesh *mesh, ISceneNode *parent, const core::vector3df& pos, const core::vector3df& rot, const core::vector3df& scale, bool alsoAddIfMeshPointerZero)
		{
			if(!alsoAddIfMeshPointerZero && mesh == NULL) return NULL;

			if(parent == NULL) parent = this;

			IAnimatedMeshSceneNode *node = new CAnimatedMeshSceneNode(mesh, parent, this, pos, rot, scale);
			node->drop();

			return node;
		}

		IBillboardSceneNode* CSceneManager::addBillboardSceneNode(ISceneNode *parent, const core::dimension2df& size, const core::vector3df& pos, video::SColor topColor, video::SColor bottomColor)
		{
			if(parent == NULL) parent = this;

			IBillboardSceneNode *node = new CBillboardSceneNode(parent, this, pos, size, topColor, bottomColor);
			node->drop();

			return node;
		}

		ISceneNodeAnimator* CSceneManager::createRotationAnimator(const core::vector3df& rotationSpeed)
		{
			ISceneNodeAnimator *anim = new CSceneNodeAnimatorRotation(os::Timer::getTime(), rotationSpeed);
			return anim;
		}

		ISceneNodeAnimator* CSceneManager::createFlyCircleAnimator(const core::vector3df& center, f32 radius, f32 speed, const core::vector3df& direction, f32 startPosition, f32 radiusEllipsoid)
		{
			const f32 orbitDurationMs = (core::DEGTORAD*360.0f)/speed;
			const u32 effectiveTime = os::Timer::getTime() + (u32)(orbitDurationMs*startPosition);

			ISceneNodeAnimator *anim = new CSceneNodeAnimatorFlyCircle(effectiveTime, center, radius, speed, direction, radiusEllipsoid);
			return anim;
		}

		void CSceneManager::setActiveCamera(ICameraSceneNode *camera)
		{
			if(camera) camera->grab();
			if(_activeCamera) _activeCamera->drop();

			_activeCamera = camera;
		}

		void CSceneManager::render()
		{
		}

		void CSceneManager::drawAll()
		{
			u32 i;
			video::IVideoDriver *driver = getVideoDriver();
			if(driver == NULL) return;

			driver->setMaterial(video::SMaterial());
			driver->setTransform(video::ETS_PROJECTION, core::identityMatrix);
			driver->setTransform(video::ETS_VIEW, core::identityMatrix);
			driver->setTransform(video::ETS_WORLD, core::identityMatrix);
			driver->setStencilShadowEnabled(_shadowNodeList.size() > 0);

			onAnimate(os::Timer::getTime());

			_camWorldPos = core::vector3df(0.0f, 0.0f, 0.0f);
			if(_activeCamera != NULL) {
				_activeCamera->render();
				_camWorldPos = _activeCamera->getAbsolutePosition();
			}

			onRegisterSceneNode();

			{
				_currentRenderTime = ESNRP_CAMERA;

				for(i=0;i < _cameraList.size();i++)
					_cameraList[i]->render();

				core::vector3df camWorldPos(0.0f, 0.0f, 0.0f);
				if(_activeCamera != NULL) camWorldPos = _activeCamera->getAbsolutePosition();

				_driver->setCamWorldPos(camWorldPos);

				_cameraList.set_used(0);
			}

			{
				_currentRenderTime = ESNRP_LIGHT;

				core::vector3df camWorldPos(0.0f, 0.0f, 0.0f);
				if(_activeCamera != NULL) camWorldPos = _activeCamera->getAbsolutePosition();

				core::array< DistanceNodeEntry > sortedLights;
				sortedLights.set_used(_lightList.size());
				for(s32 light=(s32)_lightList.size() - 1;light >= 0;--light)
					sortedLights[light].setNodeAndDistanceFromPosition(_lightList[light], camWorldPos);

				sortedLights.set_sorted(false);
				sortedLights.sort();

				for(s32 light=(s32)_lightList.size() - 1;light >= 0;--light)
					_lightList[light] = sortedLights[light].node;

				driver->deleteAllDynamicLights();
				driver->setAmbientLight(_ambientLight);

				for(i=0;i < _lightList.size();i++)
					_lightList[i]->render();
			}

			{
				_currentRenderTime = ESNRP_SOLID;

				for(i=0;i < _solidNodeList.size();i++)
					_solidNodeList[i].node->render();

				_solidNodeList.set_used(0);
			}

			{
				_currentRenderTime = ESNRP_TRANSPARENT;

				for(i=0;i < _transparentNodeList.size();i++)
					_transparentNodeList[i].node->render();

				_transparentNodeList.set_used(0);
			}

			_lightList.set_used(0);

			_currentRenderTime = ESNRP_NONE;
		}

		IMeshManipulator* CSceneManager::getMeshManipulator()
		{
			return _driver->getMeshManipulator();
		}

		ICameraSceneNode* CSceneManager::getActiveCamera() const
		{
			return _activeCamera;
		}

		u32 CSceneManager::registerNodeForRendering(ISceneNode *node, E_SCENE_NODE_RENDER_PASS pass)
		{
			u32 taken = 0;

			switch(pass) {
				case ESNRP_CAMERA:
				{
					taken = 1;
					for(u32 i=0;i < _cameraList.size();i++) {
						if(_cameraList[i] == node) {
							taken = 0;
							break;
						}
					}
					if(taken) _cameraList.push_back(node);
				}
				break;

				case ESNRP_LIGHT:
				{
					_lightList.push_back(node);
					taken = 1;
				}
				break;

				case ESNRP_SOLID:
				{
					if(!isCulled(node)) {
						_solidNodeList.push_back(node);
						taken = 1;
					}
				}
				break;

				case ESNRP_TRANSPARENT:
				{
					if(!isCulled(node)) {
						_transparentNodeList.push_back(TransparentNodeEntry(node, _camWorldPos));
						taken = 1;
					}
				}
				break;

				case ESNRP_AUTOMATIC:
				{
					if(!isCulled(node)) {
						const u32 count = node->getMaterialCount();

						taken = 0;
						for(u32 i=0;i < count;i++) {
							video::IMaterialRenderer *rnd = _driver->getMaterialRenderer(node->getMaterial(i).materialType);

							if(rnd != NULL && rnd->isTransparent()) {
								_transparentNodeList.push_back(TransparentNodeEntry(node, _camWorldPos));
								taken = 1;
								break;
							}
						}

						if(!taken) {
							_solidNodeList.push_back(node);
							taken = 1;
						}
					}
				}
				break;

				case ESNRP_SHADOW:
				{
					if(!isCulled(node)) {
						_shadowNodeList.push_back(node);
						taken = 1;
					}
				}
				break;

				default:
					break;
			}

			return taken;
		}

		ISceneNode* CSceneManager::getRootSceneNode()
		{
			return this;
		}

		const core::aabbox3df& CSceneManager::getBoundingBox() const
		{
			return *((core::aabbox3df*)0);
		}

		bool CSceneManager::isCulled(const ISceneNode *node) const
		{
			const ICameraSceneNode *cam = getActiveCamera();
			if(cam == NULL) return false;

			bool result = false;
			if(node->getAutomaticCulling()&scene::EAC_BOX) {
				core::aabbox3df tbox = node->getBoundingBox();
				core::transformBoxEx(node->getAbsoluteTransformation(), tbox);
				result = !(tbox.intersectsWithBox(cam->getViewFrustum()->getBoundingBox()));
			}

			if(!result && node->getAutomaticCulling()&EAC_FRUSTUM_SPHERE) {

			}

			if(!result && node->getAutomaticCulling()&EAC_FRUSTUM_BOX) {
				core::vector3df edges[8];
				SViewFrustum frust = *cam->getViewFrustum();
				core::matrix4 invTrans = inverse(node->getAbsoluteTransformation());

				frust.transform(invTrans);

				node->getBoundingBox().getEdges(edges);
				for(s32 i=0;i < scene::SViewFrustum::VF_PLANE_COUNT;i++) {
					bool boxInFrustum = false;

					for(u32 j=0;j < 8;j++) {
						if(frust.planes[i].classifyPointRelation(edges[j]) != core::ISREL3D_FRONT) {
							boxInFrustum = true;
							break;
						}
					}

					if(!boxInFrustum) {
						result = true;
						break;
					}
				}
			}
			return result;
		}

		void CSceneManager::setAmbientLight(const video::SColorf& ambientColor)
		{
			_ambientLight = ambientColor;
		}

		const video::SColorf& CSceneManager::getAmbientLight() const
		{
			return _ambientLight;
		}

		bool CSceneManager::postEventFromUser(const SEvent& event)
		{
			bool ret = false;
			ICameraSceneNode *camera = getActiveCamera();

			if(camera != NULL)
				ret = camera->onEvent(event);

			return ret;
		}

		E_SCENE_NODE_RENDER_PASS CSceneManager::getSceneNodeRenderPass() const
		{
			return _currentRenderTime;
		}

		IMeshCache* CSceneManager::getMeshCache()
		{
			return _meshCache;
		}

		ISceneManager* createSceneManager(video::IVideoDriver *driver, io::IFileSystem *fs)
		{
			return new CSceneManager(driver, fs);
		}
	}
}


