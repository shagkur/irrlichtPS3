/*
 * rsxcgmaterialrenderer.h
 *
 *  Created on: Jul 7, 2013
 *      Author: mike
 */

#ifndef RSXCGMATERIALRENDERER_H_
#define RSXCGMATERIALRENDERER_H_

#include "cgmaterialrenderer.h"

namespace irr
{
	namespace video
	{
		class CRSXDriver;
		class CRSXState;
		class CRSXMaterialRenderer;
		class IShaderConstantSetCallback;

		class CRSXCgMaterialRenderer : public CCgMaterialRenderer
		{
		public:
			CRSXCgMaterialRenderer(CRSXDriver *driver, s32& materialType, const void *vertexProgram, const void *fragmentProgram, IShaderConstantSetCallback *callback = NULL, E_MATERIAL_TYPE baseMaterial = video::EMT_SOLID, s32 userData = 0);
			virtual ~CRSXCgMaterialRenderer();

			virtual bool isTransparent() const;

			virtual void onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *services);
			virtual bool onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype);
			virtual void onUnsetMaterial();

			virtual void setBasicRenderStates(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates);

			virtual s32 getVertexShaderConstantID(const char *name);
			virtual s32 getPixelShaderConstantID(const char *name);
			virtual bool setVertexShaderConstant(s32 index, const f32 *floats, s32 count);
			virtual bool setPixelShaderConstant(s32 index, const f32 *floats, s32 count);

			virtual IVideoDriver* getVideoDriver();

		protected:
			void init(s32& materialType, const void *vertexProgram, const void *fragmentProgram);

			CRSXMaterialRenderer *_baseMaterial;
			CRSXDriver *_driver;

			CRSXState *_rsxState;

			rsxVertexProgram *_vertexProgram;
			void *_vertexUcode;

			rsxFragmentProgram *_fragmentProgram;
			void *_fragmentUcode;
			u32 _fragmentUcodeOffset;
			bool _fpUpdate;
		};
	}
}

#endif /* RSXCGMATERIALRENDERER_H_ */
