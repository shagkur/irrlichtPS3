#ifndef __SIRRCREATIONPARAMETERS_H__
#define __SIRRCREATIONPARAMETERS_H__

#include "dimension2d.h"

namespace irr
{
	struct SIrrlichtCreationParameters
	{
		SIrrlichtCreationParameters() {};
		SIrrlichtCreationParameters(const SIrrlichtCreationParameters& other)
		{ *this = other; }

		SIrrlichtCreationParameters& operator=(const SIrrlichtCreationParameters& other)
		{
			windowSize = other.windowSize;
			fullScreen = other.fullScreen;
			stencilBuffer = other.stencilBuffer;
			hostBufferSize = other.hostBufferSize;
			receiver = other.receiver;
			return *this;
		}

		core::dimension2d<u32> windowSize;
		bool fullScreen;
		bool stencilBuffer;
		u32 hostBufferSize;
		IEventReceiver *receiver;
	};
}

#endif
