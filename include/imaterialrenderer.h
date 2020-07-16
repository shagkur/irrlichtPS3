/*
 * imaterialrenderer.h
 *
 *  Created on: Feb 2, 2013
 *      Author: mike
 */

#ifndef IMATERIALRENDERER_H_
#define IMATERIALRENDERER_H_

#include "irefcounter.h"
#include "smaterial.h"
#include "s3dvertex.h"

namespace irr
{
	namespace video
	{
		class IVideoDriver;
		class IMaterialRendererServices;

		class IMaterialRenderer : public virtual IRefCounter
		{
		public:
			virtual void onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *services) {}

			virtual bool onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype) { return true; }

			virtual void onUnsetMaterial() {}

			virtual bool isTransparent() const { return false; }
		};
	}
}


#endif /* IMATERIALRENDERER_H_ */
