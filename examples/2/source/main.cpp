/*
 * main.cpp
 *
 *  Created on: Feb 1, 2014
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

#include "room_3ds.h"
#include "rockwall_jpg.h"
#include "rockwall_height_bmp.h"
#include "particlegreen_jpg.h"

#define HOSTBUFFER_SIZE				(128*1024*1024)

using namespace irr;

SYS_PROCESS_PARAM(1001, 0x100000);

int main(int argc, char *argv[])
{
	IrrlichtDevice *device = NULL;
	scene::ISceneManager *smgr = NULL;
	video::IVideoDriver *driver = NULL;
	io::IFileSystem *fs = NULL;

	device = createDevice(core::dimension2d<u32>(1280, 768), true, false, NULL, HOSTBUFFER_SIZE);

	driver = device->getVideoDriver();
	smgr = device->getSceneManager();
	fs = device->getFileSystem();

	scene::ICameraSceneNode *camera = smgr->addCameraSceneNodeFPS();
	camera->setPosition(core::vector3df(-200.0f, 200.0f, -200.0f));

	scene::ISceneNode *room = NULL;
	scene::IAnimatedMesh *roomMesh = smgr->getMesh(fs->createMemoryReadFile(const_cast<u8*>(room_3ds), room_3ds_size, "room.3ds"));

	printf("glbamb: %08x\n", smgr->getAmbientLight().toSColor().color);

	if(roomMesh != NULL) {
		smgr->getMeshManipulator()->makePlanarTextureMapping(roomMesh->getMesh(0), 0.003f);

		scene::IMesh *tangentMesh = smgr->getMeshManipulator()->createMeshWithTangents(roomMesh->getMesh(0), false, true, false);

		room = smgr->addMeshSceneNode(tangentMesh);
		room->getMaterial(0).specularColor.set(0, 0, 0, 0);
		room->getMaterial(0).shininess = 0.0f;

		driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, false);
		video::ITexture *normalMap = driver->getTexture(fs->createMemoryReadFile(const_cast<u8*>(rockwall_height_bmp), rockwall_height_bmp_size, "rockwall_height.bmp"));
		if(normalMap != NULL) driver->makeNormalMapTexture(normalMap, 0.6f);

		driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, true);

		//room->setMaterialTexture(0, normalMap);

		room->setMaterialTexture(0, driver->getTexture(fs->createMemoryReadFile(const_cast<u8*>(rockwall_jpg), rockwall_jpg_size, "rockwall.jpg")));
		room->setMaterialTexture(1, normalMap);
		room->setMaterialType(video::EMT_PARALLAX_MAP_SOLID);
		room->getMaterial(0).materialTypeParams[0] = 0.04f;
		room->getMaterial(0).materialTypeParams[1] = 0.02f;
		//room->setMaterialFlag(video::EMF_LIGHTING, false);
		tangentMesh->drop();
	}

	scene::ILightSceneNode* light1 = smgr->addLightSceneNode(0, core::vector3df(0,0,0), video::SColorf(0.5f, 0.5f, 0.5f, 0.0f), 800.0f);
	scene::ISceneNodeAnimator *anim = smgr->createFlyCircleAnimator(core::vector3df(50, 300, 0), 190.0f, -0.003f);

	light1->addAnimator(anim);
	anim->drop();

	scene::IBillboardSceneNode *bill = smgr->addBillboardSceneNode(light1, core::dimension2df(60, 60));
	bill->setMaterialFlag(video::EMF_LIGHTING, false);
	bill->setMaterialFlag(video::EMF_ZWRITE_ENABLE, false);
	bill->setMaterialType(video::EMT_TRANSPARENT_ADD_COLOR);
	bill->setMaterialTexture(0, driver->getTexture(fs->createMemoryReadFile(const_cast<u8*>(particlegreen_jpg), particlegreen_jpg_size, "particlegreen.jpg")));

	while(device->run()) {

		driver->beginScene(true, true);
		smgr->drawAll();

		DebugFont::setPosition(10, 10);
		DebugFont::setColor(1.0f, 1.0f, 1.0f, 0.0f);

		DebugFont::print("Irrlicht sample 2\n\n");
		DebugFont::printf("FPS: %.2f\n", (f32)driver->getFPS());

		driver->endScene();
	}

	device->terminate();
	device->drop();

	printf("done\n");
	return 0;
}


