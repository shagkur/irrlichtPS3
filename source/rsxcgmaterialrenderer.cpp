/*
 * rsxcgmaterialrenderer.cpp
 *
 *  Created on: Jul 7, 2013
 *      Author: mike
 */

#include "rsxcgmaterialrenderer.h"
#include "rsxdriver.h"
#include "rsxtexture.h"
#include "rsxmaterialrenderer.h"
#include "rsxstate.h"

namespace irr
{
	namespace video
	{
		CRSXCgMaterialRenderer::CRSXCgMaterialRenderer(CRSXDriver *driver, s32& materialType, const void *vertexProgram, const void *fragmentProgram, IShaderConstantSetCallback *callback, E_MATERIAL_TYPE baseMaterial, s32 userData)
		: CCgMaterialRenderer(callback, userData), _baseMaterial(NULL), _driver(driver), _rsxState(driver->getRSXState()),
		  _vertexProgram(NULL), _vertexUcode(NULL), _fragmentProgram(NULL), _fragmentUcode(NULL), _fpUpdate(false)
		{
			if(baseMaterial == video::EMT_TRANSPARENT_ADD_COLOR || baseMaterial == video::EMT_TRANSPARENT_ALPHA_CHANNEL ||
			   baseMaterial == video::EMT_TRANSPARENT_VERTEX_ALPHA || baseMaterial == video::EMT_TRANSPARENT_ALPHA_CHANNEL_REF)
			{
				_baseMaterial = static_cast<CRSXMaterialRenderer*>(driver->getMaterialRenderer(baseMaterial));
			}

			if(_baseMaterial) _baseMaterial->grab();

			init(materialType, vertexProgram, fragmentProgram);
		}

		CRSXCgMaterialRenderer::~CRSXCgMaterialRenderer()
		{
			if(_baseMaterial) _baseMaterial->drop();

			if(_fragmentUcode) rsxFree(_fragmentUcode);
			if(_vertexProgram) delete [] (u8*)_vertexProgram;
			if(_fragmentProgram) delete [] (u8*)_fragmentProgram;
		}

		void CRSXCgMaterialRenderer::onSetMaterial(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates, IMaterialRendererServices *services)
		{
			_material = material;

			if(material.materialType != lastMaterial.materialType || resetAllRenderStates) {
				if(_baseMaterial) _baseMaterial->onSetBaseMaterial(material);
				if(_callback) _callback->onSetMaterial(material);
			}

			//_driver->setBasicRenderStates(material, lastMaterial, resetAllRenderStates);
		}

		bool CRSXCgMaterialRenderer::onRender(IMaterialRendererServices *services, E_VERTEX_TYPE vtxtype)
		{
			//_driver->setTextureRenderStates(_driver->getCurrentMaterial(), false);

			if(_callback && (_vertexProgram || _fragmentProgram))
				_callback->onSetConstants(this, _userData);

			if(_rsxState->state.vertex_shader_program != _vertexProgram &&
			   _rsxState->state.vertex_shader_program_ucode != _vertexUcode)
			{
				rsxLoadVertexProgram(_driver->getGcmContext(), _vertexProgram, _vertexUcode);

				_rsxState->state.vertex_shader_program = _vertexProgram;
				_rsxState->state.vertex_shader_program_ucode = _vertexUcode;
			}

			if(_rsxState->state.fragment_shader_program != _fragmentProgram &&
			   _rsxState->state.fragment_shader_offset != _fragmentUcodeOffset)
			{
				rsxLoadFragmentProgramLocation(_driver->getGcmContext(), _fragmentProgram, _fragmentUcodeOffset, GCM_LOCATION_RSX);

				_rsxState->state.fragment_shader_program = _fragmentProgram;
				_rsxState->state.fragment_shader_offset = _fragmentUcodeOffset;

				_fpUpdate = false;
			} else  if(_fpUpdate) {
				rsxUpdateFragmentProgramLocation(_driver->getGcmContext(), _fragmentUcodeOffset, GCM_LOCATION_RSX);
				_fpUpdate = false;
			}

			return true;
		}

		void CRSXCgMaterialRenderer::onUnsetMaterial()
		{
			if(_baseMaterial) _baseMaterial->onUnsetBaseMaterial();

			_material = identityMaterial;
		}

		void CRSXCgMaterialRenderer::setBasicRenderStates(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates)
		{
			_driver->setBasicRenderStates(material, lastMaterial, resetAllRenderStates);
		}

		bool CRSXCgMaterialRenderer::isTransparent() const
		{
			return _baseMaterial != NULL ? _baseMaterial->isTransparent() : false;
		}

		IVideoDriver* CRSXCgMaterialRenderer::getVideoDriver()
		{
			return _driver;
		}

		s32 CRSXCgMaterialRenderer::getVertexShaderConstantID(const char *name)
		{
			//return rsxVertexProgramGetConst(_vertexProgram, name);
			return 0;
		}

		s32 CRSXCgMaterialRenderer::getPixelShaderConstantID(const char *name)
		{
			//return rsxFragmentProgramGetConst(_fragmentProgram, name);
			return 0;
		}

		bool CRSXCgMaterialRenderer::setVertexShaderConstant(s32 index, const f32 *floats, s32 count)
		{
			//rsxSetVertexProgramParameter(_driver->getGcmContext(), _vertexProgram, index, floats);
			return true;
		}

		bool CRSXCgMaterialRenderer::setPixelShaderConstant(s32 index, const f32 *floats, s32 count)
		{
			//rsxSetFragmentProgramParameter(_driver->getGcmContext(), _fragmentProgram, index, floats, _fragmentUcodeOffset, GCM_LOCATION_RSX);
			_fpUpdate = true;

			return _fpUpdate;
		}

		void CRSXCgMaterialRenderer::init(s32& materialType, const void *vertexProgram, const void *fragmentProgram)
		{
			_vertexProgram = (rsxVertexProgram*)vertexProgram;
			_fragmentProgram = (rsxFragmentProgram*)fragmentProgram;

			if(_vertexProgram) {
				u32 size;

				rsxVertexProgramGetUCode(_vertexProgram, &_vertexUcode, &size);
			}

			if(_fragmentProgram) {
				u32 size;
				void *ucode;

				rsxFragmentProgramGetUCode(_fragmentProgram, &ucode, &size);

				_fragmentUcode = rsxMemalign(64, size);
				memcpy(_fragmentUcode, ucode, size);

				rsxAddressToOffset(_fragmentUcode, &_fragmentUcodeOffset);
			}

			materialType = _driver->addMaterialRenderer(this);
		}
	}
}


