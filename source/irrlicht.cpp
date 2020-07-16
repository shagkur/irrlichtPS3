/*
 * irrlicht.cpp
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#include "irrlicht.h"
#include "iheapmanager.h"

namespace irr
{
	IHeapManager* IHeapManager::instance = NULL;

	namespace core
	{
		const matrix4 identityMatrix(matrix4::identity());
	}

	namespace video
	{
		SMaterial identityMaterial;
	}

	IrrlichtDevice* createDevice(const core::dimension2d<u32>& windowSize,bool fullScreen,bool stencilBuffer, IEventReceiver *receiver, u32 hostBufferSize)
	{
		SIrrlichtCreationParameters p;
		p.windowSize = windowSize;
		p.fullScreen = fullScreen;
		p.stencilBuffer = stencilBuffer;
		p.hostBufferSize = hostBufferSize;
		p.receiver = receiver;
		return createDeviceEx(p);
	}
}
