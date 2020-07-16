/*
 * cgmaterialrenderer.cpp
 *
 *  Created on: Feb 2, 2013
 *      Author: mike
 */

#include "cgmaterialrenderer.h"

namespace irr
{
	namespace video
	{
		CCgMaterialRenderer::CCgMaterialRenderer(IShaderConstantSetCallback *callback, s32 userData)
		: _callback(callback), _userData(userData)
		{
			if(_callback) _callback->grab();
		}

		CCgMaterialRenderer::~CCgMaterialRenderer()
		{
			if(_callback) _callback->drop();
		}
	}
}


