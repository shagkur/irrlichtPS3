/*
 * rsxshaderpipeline.h
 *
 *  Created on: Feb 11, 2014
 *      Author: mike
 */

#ifndef RSXSHADERPIPELINE_H_
#define RSXSHADERPIPELINE_H_

#include "irrarray.h"
#include "imaterialrendererservices.h"
#include "rsxshaderconstant.h"
#include "rsxvertexstream.h"
#include "rsxindexstream.h"
#include "rsxmaterial.h"
#include "rsxstate.h"

namespace irr
{
	namespace video
	{
		class CRSXDriver;

		class CRSXShaderPipeline : public IMaterialRendererServices
		{
		public:
			CRSXShaderPipeline(CRSXDriver *driver);
			virtual ~CRSXShaderPipeline();

			virtual void setBasicRenderStates(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates);
			virtual s32 getVertexShaderConstantID(const char *name);
			virtual s32 getPixelShaderConstantID(const char *name);
			virtual bool setVertexShaderConstant(s32 index, const f32 *floats, s32 count);
			virtual bool setPixelShaderConstant(s32 index, const f32 *floats, s32 count);
			virtual IVideoDriver* getVideoDriver();

			void reset();

			void setMaterial(CRSXMaterial *material);
			void addVertexStream(const CRSXVertexStream& stream);
			void setIndexStream(const CRSXIndexStream& stream);
			void setShaderConstant(const char *name, const CRSXShaderConstant& constant);
			void setTextureUnit(u32 unit, CRSXTexture *texture);

			void addToCmdBuffer();

		private:
			void unsetVertexStream(u32 index);
			void updateConstants();

			CRSXDriver *_driver;
			CRSXState *_gfxState;

			CRSXMaterial *_material;

			CRSXIndexStream _indexStream;
			core::array< CRSXVertexStream > _vertexStreams;
			core::map< core::stringc, CRSXShaderConstant > _parameters;

			core::array< CRSXVertexStream* > _activeVertexStreams;
			core::array< CRSXShaderConstant > _activeParameters;

			CRSXTexture *_textures[MAX_TEXTURE_UNITS];
		};
	}
}

#endif /* RSXSHADERPIPELINE_H_ */
