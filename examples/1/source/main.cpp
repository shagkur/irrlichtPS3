/*
 * main.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: mike
 */

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>
#include <ppu-types.h>

#include <sys/process.h>

#include <irrlicht/irrlicht.h>

#define HOSTBUFFER_SIZE				(128*1024*1024)

using namespace irr;

SYS_PROCESS_PARAM(1001, 0x100000);

static 	scene::IRigidBody *rigidBody = NULL;
static 	scene::IRigidBody *rigidBody2 = NULL;
static 	scene::IRigidBody *rigidBodySphere = NULL;

class EventReceiverClass : public IEventReceiver
{
public:
	virtual bool onEvent(const SEvent& event)
	{
		if(event.eventType == EET_JOYPAD_INPUT_EVENT) {
			/*
			if(event.joypadEvent.isButtonPressed(0)) {
				if(rigidBody) rigidBody->applyAngularImpulse(core::vector3df(0.0f, -1500.0f, 0.0f));
			} else if(event.joypadEvent.isButtonPressed(2)){
				if(rigidBody) rigidBody->applyAngularImpulse(core::vector3df(0.0f, 1500.0f, 0.0f));
			}
			*/
			f32 cofL, cofR;
			f32 vX, vY;

			vX = event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_RSTICK_X];
			vY = event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_RSTICK_Y];

			cofL = +1.0f*vX;
			cofR = +1.0f*vY;

			if(rigidBody2) {
				rigidBody2->applyAngularImpulse(core::vector3df(0.0f, 1500.0f, 0.0f)*cofL);
				rigidBody2->applyLinearImpulse(core::vector3df(0.0f, 150.0f, 0.0f)*cofR);
			}
		}
		return false;
	}
};

int main(int argc, char *argv[])
{
	EventReceiverClass eventReceiver;
	IrrlichtDevice *device = NULL;
	video::IVideoDriver *driver = NULL;
	scene::ISceneManager *smgr = NULL;
	scene::IPhysicsManager *pmgr = NULL;
	scene::IMeshManipulator *mpt = NULL;
	ITimer *timer;

	device = createDevice(core::dimension2d<u32>(1280, 768), true, false, &eventReceiver, HOSTBUFFER_SIZE);
	driver = device->getVideoDriver();
	timer = device->getTimer();
	smgr = device->getSceneManager();
	pmgr = smgr->getPhysicsManager();
	mpt = smgr->getMeshManipulator();

	//smgr->setAmbientLight(video::SColorf(0.125f, 0.125f, 0.125f));

	scene::IMesh *m = NULL;
	scene::ICameraSceneNode *camera = smgr->addCameraSceneNode(NULL, core::vector3df(0.0f, 0.0f, 200.0f), core::vector3df(0.0f, 0.0f, 0.0f));
	//scene::ICameraSceneNode *camera = smgr->addCameraSceneNodeFPS();
	//camera->setPosition(core::vector3df(0.0f, 0.0f, 100.0f));
	//camera->setTarget(core::vector3df(0.0f, 0.0f, 0.0f));
	//scene::IMeshSceneNode *smesh = smgr->addSphereSceneNode(20.0f, 64);
	//mpt->recalculateNormals(m, true, true);

	scene::IMeshSceneNode *smesh = NULL;
	scene::IMeshSceneNode *dmesh = NULL;
	scene::IMeshSceneNode *mesh = NULL;

	//scene::IMesh *lopolymesh = smgr->getGeometryCreator()->createSphereMesh(20.0f, 32, 32);
	//scene::IMesh *lopolymesh2 = smgr->getGeometryCreator()->createTorusMesh(20.0f, 10.0f, 32, 32);

	m = smgr->getGeometryCreator()->createCubeMesh(core::vector3df(200.0f, 8.0f, 200.0f));
	smesh = smgr->addMeshSceneNode(m);

	m = smgr->getGeometryCreator()->createSphereMesh(20.0f, 64, 64);
	mesh = smgr->addMeshSceneNode(m);
	//mesh->addShadowVolumeSceneNode(lopolymesh);

	m = smgr->getGeometryCreator()->createTorusMesh(20.0f, 10.0f, 64, 64);
	dmesh = smgr->addMeshSceneNode(m);
	//dmesh->addShadowVolumeSceneNode(lopolymesh2);

	//scene::IMeshSceneNode *mesh = smgr->addDonutSceneNode(20.0f, 10.0f, 64);
	scene::IAnimatedMesh *amesh = smgr->getMesh("/dev_usb000/models/awing/AWING.3DS");
	scene::IAnimatedMeshSceneNode *mmesh = smgr->addAnimatedMeshSceneNode(amesh);
	mmesh->setScale(core::vector3df(3.0f, 3.0f, 3.0f));
	if(smesh != NULL) {
		video::ITexture *tex = driver->getTexture("/dev_usb000/textures/tribal.png");
		video::SMaterial& mat = smesh->getMaterial(0);
		printf("loaded: %p %p\n", tex, &mat);
		mat.setTexture(0, tex);
		//mat.shininess = 4.0f;
		//mat.lighting = false;

		smesh->setPosition(core::vector3df(0.0f, -60.0f, 0.0f));
		rigidBodySphere = pmgr->createRigidBodyBox(smesh, 2.0f);
		rigidBodySphere->setMoveType(scene::EMT_FIXED);
	}
	if(mesh != NULL) {
		video::ITexture *tex = driver->getTexture("/dev_usb000/textures/AWT4.jpg");
		video::SMaterial& mat = mesh->getMaterial(0);
		printf("loaded: %p %p\n", tex, &mat);
		mat.setTexture(0, tex);
		mat.shininess = 16.0f;

		mesh->setRotation(core::vector3df(0.0f, 30.0f, 0.0f));
		mesh->setPosition(core::vector3df(-30.0f, 40.0f, 0.0f));
		rigidBody = pmgr->createRigidBodySphere(mesh, 20.0f);
		rigidBody->setElasticity(0.4f);
		//rigidBody->setAngularVelocity(core::vector3df(20.0f, 30.0f, 60.0f));
	}
	if(dmesh != NULL) {
		//video::ITexture *tex = driver->getTexture("/dev_usb000/textures/tribal.png");
		video::ITexture *tex = driver->getTexture("/dev_usb000/textures/AWT4.jpg");
		video::SMaterial& mat = dmesh->getMaterial(0);
		printf("loaded: %p %p\n", tex, &mat);
		mat.setTexture(0, tex);
		mat.shininess = 12.0f;
		//mat.wireFrame = true;
		//mat.frontfaceCulling = false;
		//mat.backfaceCulling = false;

		dmesh->setRotation(core::vector3df(0.0f, -30.0f, 0.0f));
		dmesh->setPosition(core::vector3df(30.0f, 40.0f, 0.0f));
		rigidBody2 = pmgr->createRigidBodySphere(dmesh, 20.0f);
		rigidBody2->setElasticity(0.4f);
		//rigidBody->setAngularVelocity(core::vector3df(20.0f, 30.0f, 60.0f));
	}
	/*
	if(smesh != NULL) {
		video::ITexture *tex = driver->getTexture("/dev_usb000/textures/tribal.png");
		video::SMaterial& mat = smesh->getMaterial(0);
		printf("loaded: %p %p\n", tex, &mat);
		mat.setTexture(0, tex);

		smesh->setPosition(core::vector3df(0.0f, -60.0f, -100.0f));
		rigidBodySphere = pmgr->createRigidBodyBox(smesh, 2.0f);
		rigidBodySphere->setMoveType(scene::EMT_FIXED);
	}
	*/
	scene::ILightSceneNode *light = smgr->addLightSceneNode(NULL, core::vector3df(0.0f, 0.0f, 100.0f), video::SColorf(0.5f, 0.5f, 0.5f), 110.0f);
	//scene::ILightSceneNode *light1 = smgr->addLightSceneNode(NULL, core::vector3df(0.0f, 00.0f, 0.0f), video::SColorf(1.0f, 1.0f, 1.0f), 220.0f);
	scene::ISceneNodeAnimator *anim = smgr->createRotationAnimator(core::vector3df(1.0f, 1.0f, 0.0f));
	//scene::ISceneNodeAnimator *anim2 = smgr->createFlyCircleAnimator(core::vector3df(0.0f, 200.0f, 0.0f), 160.0f);

	//donutMesh->setPosition(core::vector3df(10.0f, 20.0f, -10.0f));

	//light->getLightData().type = video::ELT_POINT;
	//light->getLightData().outerCone = 4.0f;
	//light->getLightData().innerCone = 2.0f;
	//light->getLightData().ambientColor = video::SColorf(0.1f, 0.1f, 0.1f);
	//sphereMesh->addAnimator(anim);
	//light1->getLightData().type = video::ELT_SPOT;
	//light1->getLightData().outerCone = 31.0f;
	//light1->getLightData().innerCone = 21.0f;
	//light1->addAnimator(anim2);
	mmesh->addAnimator(anim);

	printf("running 1st example of irrlichtPS3\n");

	f32 rot = 0.0f;
	u32 deltaTime = 0;
	u32 timeStamp = timer->getTime();
	while(device->run()) {
		deltaTime = timer->getTime() - timeStamp;
		timeStamp = timer->getTime();

		pmgr->stepSimulation(deltaTime*0.001f, 60, 1.0f/60.0f);

		driver->beginScene(true, true);
		smgr->drawAll();

		DebugFont::setPosition(0, 10);
		DebugFont::setColor(1.0f, 0.0f, 0.0f, 1.0f);

		DebugFont::print("Irrlicht physics sample\n\n");
		DebugFont::printf("FPS: %.2f\n", (f32)driver->getFPS());

		driver->endScene();

		//printf("fps: %.2f\n", driver->getFPS());
		//rot += 0.25f;
		//if(rot >= 360.0f) rot = 0.0f;
		//light->setRotation(core::vector3df(-60.0f, rot, 0.0f));
	}
	printf("done\n");
	return 0;
}
