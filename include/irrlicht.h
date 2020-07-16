/*
 * irrlicht.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef IRRLICHT_H_
#define IRRLICHT_H_

#include "irrtypes.h"
#include "irrlist.h"
#include "irrmap.h"
#include "irrmath.h"
#include "irrstring.h"
#include "heapsort.h"
#include "matrix4.h"
#include "aabbox3d.h"
#include "vector3d.h"
#include "quaternion.h"
#include "dimension2d.h"
#include "irrlichtdevice.h"
#include "ifilesystem.h"
#include "irrlichtdevice.h"
#include "ecullingtypes.h"
#include "ehardwarebufferflags.h"
#include "ematerialtypes.h"
#include "ematerialflags.h"
#include "escenenodetypes.h"
#include "escenenodeanimatortypes.h"
#include "irigidbody.h"
#include "imesh.h"
#include "imeshbuffer.h"
#include "ianimatedmesh.h"
#include "iscenenode.h"
#include "iscenemanager.h"
#include "iphysicsmanager.h"
#include "icamerascenenode.h"
#include "imeshscenenode.h"
#include "ibillboardscenenode.h"
#include "ilightscenenode.h"
#include "imeshmanipulator.h"
#include "ianimatedmeshscenenode.h"
#include "ishadowvolumescenenode.h"
#include "iscenenodeanimator.h"
#include "imeshcache.h"
#include "imeshloader.h"
#include "iimage.h"
#include "iimageloader.h"
#include "imaterialrenderer.h"
#include "imaterialrendererservices.h"
#include "ivideodriver.h"
#include "itexture.h"
#include "itimer.h"
#include "ieventreceiver.h"
#include "sviewfrustum.h"
#include "sirrcreationparameters.h"

#include "debug/debugfont.h"

#define DEFAULT_HOSTBUFFER_SIZE				(32*1024*1024)

namespace irr
{
	extern "C" IrrlichtDevice* createDevice(const core::dimension2d<u32>& windowSize = core::dimension2d<u32>(1280, 768), bool fullScreen = false, bool stencilBuffer = false, IEventReceiver *receiver = NULL, u32 hostBufferSize = DEFAULT_HOSTBUFFER_SIZE);
	extern "C" IrrlichtDevice* createDeviceEx(const SIrrlichtCreationParameters& param);
}

#endif /* IRRLICHT_H_ */
