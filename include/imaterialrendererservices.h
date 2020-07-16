/*
 * imaterialrendererservices.h
 *
 *  Created on: Feb 2, 2013
 *      Author: mike
 */

#ifndef IMATERIALRENDERERSERVICES_H_
#define IMATERIALRENDERERSERVICES_H_

#include "smaterial.h"
#include "s3dvertex.h"

namespace irr
{
	namespace video
	{
		class IVideoDriver;

		class IMaterialRendererServices
		{
		public:
			virtual ~IMaterialRendererServices() {}

			virtual void setBasicRenderStates(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates) = 0;

			virtual s32 getVertexShaderConstantID(const char *name) = 0;

			virtual s32 getPixelShaderConstantID(const char *name) = 0;

			virtual bool setVertexShaderConstant(s32 index, const f32 *floats, s32 count) = 0;

			virtual bool setPixelShaderConstant(s32 index, const f32 *floats, s32 count) = 0;

			bool setVertexShaderConstant(const char *name, const f32 *floats, s32 count)
			{
				return setVertexShaderConstant(getVertexShaderConstantID(name), floats, count);
			}

			bool setPixelShaderConstant(const char *name, const f32 *floats, s32 count)
			{
				return setPixelShaderConstant(getPixelShaderConstantID(name), floats, count);
			}

			virtual IVideoDriver* getVideoDriver() = 0;
		};
	}
}
#endif /* IMATERIALRENDERERSERVICES_H_ */
