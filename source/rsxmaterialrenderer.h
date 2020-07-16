/*
 * rsxmaterialrenderer.h
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#ifndef RSXMATERIALRENDERER_H_
#define RSXMATERIALRENDERER_H_

#include "imaterialrenderer.h"

namespace irr
{
	namespace video
	{
		class CRSXDriver;
		class CRSXState;

		class CRSXMaterialRenderer : public IMaterialRenderer
		{
		public:
			CRSXMaterialRenderer(CRSXDriver *driver);

			virtual void onSetBaseMaterial(const SMaterial& material) {}
			virtual void onUnsetBaseMaterial() {}

		protected:
			struct lightingParams
			{
				core::vector4df pos;
				core::vector4df dir;
				core::vector3df spot;
			};

			void setupLight(const core::matrix4& worldIT, const SLight& light, lightingParams *parms);
			void renderLights(const SMaterial& material, CRSXMaterial *renderMaterial, CRSXShaderPipeline *pipeline);
			void renderAmbientEmissive(const SMaterial& material, CRSXMaterial *renderMaterial, CRSXShaderPipeline *pipeline);

			CRSXDriver *_driver;
			CRSXState *_rsxState;
		};

		class CRSXMaterialRenderer_SOLID : public CRSXMaterialRenderer
		{
		public:
			CRSXMaterialRenderer_SOLID(CRSXDriver *driver);

			virtual void onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *servcies);

			virtual bool onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype);

			virtual void onUnsetMaterial();
		};

		class CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR : public CRSXMaterialRenderer
		{
		public:
			CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR(CRSXDriver *driver);

			virtual void onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *servcies);
			virtual void onSetBaseMaterial(const SMaterial& material);

			virtual bool onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype);

			virtual void onUnsetMaterial();
			virtual void onUnsetBaseMaterial();

			virtual bool isTransparent() const
			{
				return true;
			}
		};

		class CRSXParallaxMaterialRenderer : public CRSXMaterialRenderer
		{
		public:
			CRSXParallaxMaterialRenderer(CRSXDriver *driver);

			virtual void onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *servcies);
			virtual void onSetBaseMaterial(const SMaterial& material);

			virtual bool onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype);

			virtual void onUnsetMaterial();
			virtual void onUnsetBaseMaterial();
		};
	}
}

#endif /* RSXMATERIALRENDERER_H_ */
