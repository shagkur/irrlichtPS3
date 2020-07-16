/*
 * rsxmaterialrenderer.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#include "vector4d.h"

#include "rsxdriver.h"
#include "rsxtexture.h"
#include "rsxstate.h"
#include "rsxshaderpipeline.h"
#include "rsxshaderconstant.h"

#include "rsxmaterialrenderer.h"

namespace irr
{
	namespace video
	{
		CRSXMaterialRenderer::CRSXMaterialRenderer(CRSXDriver *driver)
		: _driver(driver), _rsxState(driver->getRSXState())
		{
		}

		void CRSXMaterialRenderer::renderAmbientEmissive(const SMaterial& material, CRSXMaterial *renderMaterial, CRSXShaderPipeline *pipeline)
		{
			SColorf materialAmbient(material.ambientColor);
			SColorf materialEmissive(material.emissiveColor);

			pipeline->setShaderConstant("globalAmbient", CRSXShaderConstant(_driver->getAmbientLight().color, 4));
			pipeline->setShaderConstant("Kambient", CRSXShaderConstant(materialAmbient.color, 4));
			pipeline->setShaderConstant("Kemissive", CRSXShaderConstant(materialEmissive.color, 4));

			pipeline->setMaterial(renderMaterial);
			pipeline->setBasicRenderStates(material, identityMaterial, false);

			pipeline->addToCmdBuffer();
		}

		void CRSXMaterialRenderer::renderLights(const SMaterial& material, CRSXMaterial *renderMaterial, CRSXShaderPipeline *pipeline)
		{
			SColorf materialDiffuse(material.diffuseColor);
			SColorf materialSpecular(material.specularColor);
			core::vector3df camWorldPos = _driver->getCamWorldPos();
			const core::matrix4& worldIT = inverse(_driver->getTransform(ETS_WORLD));

			core::transformVect(worldIT, camWorldPos);
			pipeline->setShaderConstant("eyePos", CRSXShaderConstant((float*)&camWorldPos, 3));
			pipeline->setShaderConstant("Kdiffuse", CRSXShaderConstant(materialDiffuse.color, 3));
			pipeline->setShaderConstant("Kspecular", CRSXShaderConstant(materialSpecular.color, 3));
			pipeline->setShaderConstant("shininess", CRSXShaderConstant(&material.shininess, 1));

			// Render the lighting term for all lights (will extend to light culling based on sphere of influence)
			pipeline->setMaterial(renderMaterial);
			pipeline->setBasicRenderStates(material, identityMaterial, false);

			//turn off zwrite for light pass
			renderMaterial->setDepthMask(GCM_FALSE);
			for(u32 i=0;i < _driver->getDynamicLightCount();i++) {
				lightingParams params;
				const SLight& light = _driver->getDynamicLight(i);

				setupLight(worldIT, light, &params);

				pipeline->setShaderConstant("Lposition", CRSXShaderConstant((float*)&params.pos, 4));
				pipeline->setShaderConstant("Ldirection", CRSXShaderConstant((float*)&params.dir, 4));
				pipeline->setShaderConstant("Lspot", CRSXShaderConstant((float*)&params.spot, 3));
				pipeline->setShaderConstant("Ldiffuse", CRSXShaderConstant(light.diffuseColor.color, 3));
				pipeline->setShaderConstant("Lspecular", CRSXShaderConstant(light.specularColor.color, 3));
				pipeline->setShaderConstant("Lattenuation", CRSXShaderConstant((float*)&light.attenuation, 3));

				pipeline->addToCmdBuffer();
			}
		}

		void CRSXMaterialRenderer::setupLight(const core::matrix4& worldIT, const SLight& light, lightingParams *params)
		{
			core::vector3df lpos = light.position;
			core::vector3df ldir = light.direction;

			switch(light.type) {
				case ELT_SPOT:
				{
					core::transformVect(worldIT, lpos);
					core::rotateVect(worldIT, ldir);

					params->pos = core::vector4df(lpos, 1.0f);
					params->dir = core::vector4df(ldir, 0.0f);
					params->spot = core::vector3df(cosf(light.outerCone), cosf(light.innerCone), light.fallOff);
				}
				break;

				case ELT_POINT:
				{
					core::transformVect(worldIT, lpos);
					core::rotateVect(worldIT, ldir);

					params->pos = core::vector4df(lpos, 1.0f);
					params->dir = core::vector4df(ldir, 1.0f);
				}
				break;

				case ELT_DIRECTIONAL:
				{
					core::rotateVect(worldIT, ldir);

					params->pos = core::vector4df(-ldir, 0.0f);
					params->dir = core::vector4df(ldir, 1.0f);
				}
				break;

				default:
					return;
			}

		}

		//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		CRSXMaterialRenderer_SOLID::CRSXMaterialRenderer_SOLID(CRSXDriver *driver)
		: CRSXMaterialRenderer(driver)
		{
		}

		void CRSXMaterialRenderer_SOLID::onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *services)
		{
		}

		bool CRSXMaterialRenderer_SOLID::onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype)
		{
			const SMaterial& material = _driver->getCurrentMaterial();
			CRSXShaderPipeline *pipeline = dynamic_cast<CRSXShaderPipeline*>(services);
			const core::matrix4& mvp = _driver->getTransform(ETS_MVP);

			_driver->disableTextures(1);
			_driver->setTextureRenderStates(material, pipeline);

			pipeline->setShaderConstant("mvp", CRSXShaderConstant(&mvp));

			if(material.lighting) {
				CRSXMaterial *ambientEmissiveMaterial = _driver->popRSXMaterial("material_ambient_emissive");
				CRSXMaterial *lightingMaterial = _driver->popRSXMaterial("material_lighting");

				ambientEmissiveMaterial->setBlendEnable(GCM_FALSE);
				ambientEmissiveMaterial->setBlendEquation(GCM_FUNC_ADD, GCM_FUNC_ADD);
				ambientEmissiveMaterial->setBlendFunc(GCM_ONE, GCM_ZERO, GCM_ONE, GCM_ZERO);

				lightingMaterial->setBlendEnable(GCM_TRUE);
				lightingMaterial->setBlendEquation(GCM_FUNC_ADD, GCM_FUNC_ADD);
				lightingMaterial->setBlendFunc(GCM_ONE, GCM_ONE, GCM_ONE, GCM_ONE);

				renderAmbientEmissive(material, ambientEmissiveMaterial, pipeline);
				renderLights(material, lightingMaterial, pipeline);

				_driver->pushRSXMaterial(lightingMaterial);
				_driver->pushRSXMaterial(ambientEmissiveMaterial);
			} else {
				CRSXMaterial *tex2dMaterial = _driver->popRSXMaterial("material_tex2d");

				pipeline->setMaterial(tex2dMaterial);
				pipeline->setBasicRenderStates(material, identityMaterial, false);

				pipeline->addToCmdBuffer();

				_driver->pushRSXMaterial(tex2dMaterial);
			}

			return true;
		}

		void CRSXMaterialRenderer_SOLID::onUnsetMaterial()
		{

		}

		//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR::CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR(CRSXDriver *driver)
		: CRSXMaterialRenderer(driver)
		{

		}

		void CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR::onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *servcies)
		{

		}

		void CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR::onSetBaseMaterial(const SMaterial& material)
		{

		}


		bool CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR::onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype)
		{
			const SMaterial& material = _driver->getCurrentMaterial();
			CRSXShaderPipeline *pipeline = dynamic_cast<CRSXShaderPipeline*>(services);
			const core::matrix4& mvp = _driver->getTransform(ETS_MVP);

			_driver->disableTextures(1);
			_driver->setTextureRenderStates(material, pipeline);

			pipeline->setShaderConstant("mvp", CRSXShaderConstant(&mvp));

			if(material.lighting) {
				CRSXMaterial *ambientEmissiveMaterial = _driver->popRSXMaterial("material_ambient_emissive");
				CRSXMaterial *lightingMaterial = _driver->popRSXMaterial("material_lighting");

				ambientEmissiveMaterial->setBlendEnable(GCM_TRUE);
				ambientEmissiveMaterial->setBlendEquation(GCM_FUNC_ADD, GCM_FUNC_ADD);
				ambientEmissiveMaterial->setBlendFunc(GCM_SRC_COLOR, GCM_ONE_MINUS_SRC_COLOR, GCM_SRC_ALPHA, GCM_ONE_MINUS_SRC_ALPHA);

				lightingMaterial->setBlendEnable(GCM_TRUE);
				lightingMaterial->setBlendEquation(GCM_FUNC_ADD, GCM_FUNC_ADD);
				lightingMaterial->setBlendFunc(GCM_SRC_COLOR, GCM_ONE, GCM_SRC_ALPHA, GCM_ONE);

				renderAmbientEmissive(material, ambientEmissiveMaterial, pipeline);
				renderLights(material, lightingMaterial, pipeline);

				_driver->pushRSXMaterial(lightingMaterial);
				_driver->pushRSXMaterial(ambientEmissiveMaterial);
			} else {
				CRSXMaterial *tex2dMaterial = _driver->popRSXMaterial("material_tex2d");

				tex2dMaterial->setBlendEnable(GCM_TRUE);
				tex2dMaterial->setBlendEquation(GCM_FUNC_ADD, GCM_FUNC_ADD);
				tex2dMaterial->setBlendFunc(GCM_ONE, GCM_ONE_MINUS_SRC_COLOR, GCM_ONE, GCM_ONE_MINUS_SRC_ALPHA);

				pipeline->setMaterial(tex2dMaterial);
				pipeline->setBasicRenderStates(material, identityMaterial, false);

				pipeline->addToCmdBuffer();

				_driver->pushRSXMaterial(tex2dMaterial);
			}

			return true;
		}

		void CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR::onUnsetMaterial()
		{
		}

		void CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR::onUnsetBaseMaterial()
		{

		}

		//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
		CRSXParallaxMaterialRenderer::CRSXParallaxMaterialRenderer(CRSXDriver *driver)
		: CRSXMaterialRenderer(driver)
		{

		}

		void CRSXParallaxMaterialRenderer::onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *servcies)
		{

		}

		void CRSXParallaxMaterialRenderer::onSetBaseMaterial(const SMaterial& material)
		{

		}

		bool CRSXParallaxMaterialRenderer::onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype)
		{
			const SMaterial& material = _driver->getCurrentMaterial();
			CRSXShaderPipeline *pipeline = dynamic_cast<CRSXShaderPipeline*>(services);
			const core::matrix4& mvp = _driver->getTransform(ETS_MVP);

			_driver->disableTextures(2);
			_driver->setTextureRenderStates(material, pipeline);

			pipeline->setShaderConstant("mvp", CRSXShaderConstant(&mvp));
			pipeline->setShaderConstant("scale", CRSXShaderConstant(&material.materialTypeParams[0], 1));
			pipeline->setShaderConstant("bias", CRSXShaderConstant(&material.materialTypeParams[1], 1));

			if(material.lighting) {
				CRSXMaterial *ambientEmissiveMaterial = _driver->popRSXMaterial("material_ambient_emissive");
				CRSXMaterial *parallaxMaterial = _driver->popRSXMaterial("material_parallax_lighting");

				ambientEmissiveMaterial->setBlendEnable(GCM_FALSE);
				ambientEmissiveMaterial->setBlendEquation(GCM_FUNC_ADD, GCM_FUNC_ADD);
				ambientEmissiveMaterial->setBlendFunc(GCM_ONE, GCM_ZERO, GCM_ONE, GCM_ZERO);

				parallaxMaterial->setBlendEnable(GCM_TRUE);
				parallaxMaterial->setBlendEquation(GCM_FUNC_ADD, GCM_FUNC_ADD);
				parallaxMaterial->setBlendFunc(GCM_ONE, GCM_ONE, GCM_ONE, GCM_ONE);

				renderAmbientEmissive(material, ambientEmissiveMaterial, pipeline);
				renderLights(material, parallaxMaterial, pipeline);

				_driver->pushRSXMaterial(parallaxMaterial);
				_driver->pushRSXMaterial(ambientEmissiveMaterial);
			} else {
				core::vector3df camWorldPos = _driver->getCamWorldPos();
				const core::matrix4& worldIT = inverse(_driver->getTransform(ETS_WORLD));
				CRSXMaterial *parallaxMaterial = _driver->popRSXMaterial("material_parallax");

				core::transformVect(worldIT, camWorldPos);
				pipeline->setShaderConstant("eyePos", CRSXShaderConstant((float*)&camWorldPos, 3));

				pipeline->setMaterial(parallaxMaterial);
				pipeline->setBasicRenderStates(material, identityMaterial, false);

				pipeline->addToCmdBuffer();

				_driver->pushRSXMaterial(parallaxMaterial);
			}

			return true;
		}

		void CRSXParallaxMaterialRenderer::onUnsetMaterial()
		{
		}

		void CRSXParallaxMaterialRenderer::onUnsetBaseMaterial()
		{

		}
	}
}


