/*
 * cgmaterialrenderer.h
 *
 *  Created on: Feb 2, 2013
 *      Author: mike
 */

#ifndef CGMATERIALRENDERER_H_
#define CGMATERIALRENDERER_H_

#include "imaterialrenderer.h"
#include "imaterialrendererservices.h"
#include "ishaderconstantsetcallback.h"
#include "ivideodriver.h"
#include "os.h"

namespace irr
{
	namespace video
	{
		class CCgMaterialRenderer : public IMaterialRenderer, public IMaterialRendererServices
		{
		public:
			CCgMaterialRenderer(IShaderConstantSetCallback *callback = NULL, s32 userData = 0);
			virtual ~CCgMaterialRenderer();

			virtual bool isTransparent() const = 0;

		protected:
			IShaderConstantSetCallback *_callback;
			s32 _userData;

			SMaterial _material;
		};
	}
}

#endif /* CGMATERIALRENDERER_H_ */
