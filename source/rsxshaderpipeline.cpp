/*
 * rsxshaderpipeline.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: mike
 */

#include "ecgprofiles.h"
#include "rsxdriver.h"
#include "rsxtexture.h"
#include "rsxshaderparam.h"
#include "rsxshaderpipeline.h"

namespace irr
{
	namespace video
	{
		CRSXShaderPipeline::CRSXShaderPipeline(CRSXDriver *driver)
		: _driver(driver), _gfxState(driver->getRSXState())
		{
			_vertexStreams.reallocate(16);
			_activeVertexStreams.reallocate(16);
			_activeParameters.reallocate(256);
		}

		CRSXShaderPipeline::~CRSXShaderPipeline()
		{

		}

		void CRSXShaderPipeline::reset()
		{
			_vertexStreams.set_used(0);
			_activeVertexStreams.set_used(0);
			_activeParameters.set_used(0);
		}

		void CRSXShaderPipeline::setBasicRenderStates(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates)
		{
			if(material.gouraudShading)
				_material->setShadeModel(GCM_SHADE_MODEL_SMOOTH);
			else
				_material->setShadeModel(GCM_SHADE_MODEL_FLAT);

			{
				u32 polygon_mode = material.wireFrame ? GCM_POLYGON_MODE_LINE : (material.pointCloud ? GCM_POLYGON_MODE_POINT : GCM_POLYGON_MODE_FILL);

				_material->setBackPolygonMode(polygon_mode);
				_material->setFrontPolygonMode(polygon_mode);
			}

			{
				u32 depth_test = GCM_FALSE;
				u32 depth_func = GCM_LESS;
				switch(material.zBuffer) {
					case ECFN_LESSEQUAL:
						depth_test = GCM_TRUE;
						depth_func = GCM_LEQUAL;
						break;
					case ECFN_EQUAL:
						depth_test = GCM_TRUE;
						depth_func = GCM_EQUAL;
						break;
					case ECFN_LESS:
						depth_test = GCM_TRUE;
						depth_func = GCM_LESS;
						break;
					case ECFN_NOTEQUAL:
						depth_test = GCM_TRUE;
						depth_func = GCM_NOTEQUAL;
						break;
					case ECFN_GREATEREQUAL:
						depth_test = GCM_TRUE;
						depth_func = GCM_GEQUAL;
						break;
					case ECFN_GREATER:
						depth_test = GCM_TRUE;
						depth_func = GCM_GREATER;
						break;
					case ECFN_ALWAYS:
						depth_test = GCM_TRUE;
						depth_func = GCM_ALWAYS;
						break;
					case ECFN_NEVER:
						depth_test = GCM_TRUE;
						depth_func = GCM_NEVER;
						break;
					default:
						break;
				}

				_material->setDepthTestEnable(depth_test);
				_material->setDepthFunc(depth_func);
			}

			{
				u32 depth_mask;

				if(material.zWriteEnable && !material.isTransparent())
					depth_mask = GCM_TRUE;
				else
					depth_mask = GCM_FALSE;

				_material->setDepthMask(depth_mask);
			}

			{
				u32 cull_face_mode = GCM_CULL_BACK;
				u32 cull_face_enable = (material.frontfaceCulling || material.backfaceCulling);

				if(material.frontfaceCulling && material.backfaceCulling)
					cull_face_mode = GCM_CULL_ALL;
				else if(material.backfaceCulling)
					cull_face_mode = GCM_CULL_BACK;
				else if(material.frontfaceCulling)
					cull_face_mode = GCM_CULL_FRONT;

				_material->setCullFace(cull_face_mode);
				_material->setCullFaceEnable(cull_face_enable);
			}

			{
				u32 color_mask = (((material.colorMask&ECP_ALPHA) ? GCM_COLOR_MASK_A : 0) |
						   	     ((material.colorMask&ECP_RED) ? GCM_COLOR_MASK_R : 0) |
						   	     ((material.colorMask&ECP_GREEN) ? GCM_COLOR_MASK_G : 0) |
						   	     ((material.colorMask&ECP_BLUE) ? GCM_COLOR_MASK_B : 0));


				_material->setColorMask(color_mask);
			}
		}

		s32 CRSXShaderPipeline::getVertexShaderConstantID(const char *name)
		{
			return -1;
		}

		s32 CRSXShaderPipeline::getPixelShaderConstantID(const char *name)
		{
			return -1;
		}

		bool CRSXShaderPipeline::setVertexShaderConstant(s32 index, const f32 *floats, s32 count)
		{
			return false;
		}

		bool CRSXShaderPipeline::setPixelShaderConstant(s32 index, const f32 *floats, s32 count)
		{
			return false;
		}

		IVideoDriver* CRSXShaderPipeline::getVideoDriver()
		{
			return _driver;
		}

		void CRSXShaderPipeline::setTextureUnit(u32 unit, CRSXTexture *texture)
		{
			_textures[unit] = texture;
		}

		void CRSXShaderPipeline::addVertexStream(const CRSXVertexStream& stream)
		{
			_vertexStreams.push_back(stream);
		}

		void CRSXShaderPipeline::setIndexStream(const CRSXIndexStream& stream)
		{
			_indexStream = stream;
		}

		void CRSXShaderPipeline::setShaderConstant(const char *name, const CRSXShaderConstant& constant)
		{
			_parameters.set(name, constant);
		}

		void CRSXShaderPipeline::setMaterial(CRSXMaterial *material)
		{
			_material = material;

			_activeVertexStreams.set_used(0);

			u32 numActiveStreams = 0;
			CRSXCgShader *shader = material->getVertexShader();
			core::map<core::stringc, CRSXShaderParam*>::Iterator it = shader->params.getIterator();

			while(!it.atEnd()) {
				CRSXShaderParam *param = it.getNode()->getValue();

				if(param->getProfile() == ECP_VERTEX && param->getVariability() == EPV_VARYING) {
					rsxProgramAttrib *attrib = (rsxProgramAttrib*)param->getParam();

					for(u32 j=0;j < _vertexStreams.size();j++) {
						CRSXVertexStream *stream = &_vertexStreams[j];

						if(attrib->index == stream->getAttributeIndex()) {
							_activeVertexStreams[numActiveStreams++] = stream;
							break;
						}
					}
				}
				it++;
			}

			_activeVertexStreams.set_used(numActiveStreams);
		}

		void CRSXShaderPipeline::addToCmdBuffer()
		{
			updateConstants();

			for(u32 i=0;i < _activeParameters.size();i++)
				_activeParameters[i].addToCmdBuffer();

			_material->addToCmdBuffer();

			for(u32 i=0; i < MAX_TEXTURE_UNITS;i++) {
				if(_textures[i] != NULL)
					_textures[i]->addToCmdBuffer(i);
			}

			bool attr_set[16];
			for(u32 i=0;i < 16;i++)
				attr_set[i] = false;

			for(u32 i=0;i < _activeVertexStreams.size();i++) {
				_activeVertexStreams[i]->addToCmdBuffer();
				attr_set[_activeVertexStreams[i]->getAttributeIndex()] = true;
			}

			for(u32 i=0;i < 16;i++) {
				if(attr_set[i] == false)
					unsetVertexStream(i);
			}

			_indexStream.addToCmdBuffer();
		}

		void CRSXShaderPipeline::unsetVertexStream(u32 index)
		{
			vertexDataArray_t *vtxArray = &_gfxState->state.vertexDataArray[index];

			if(vtxArray->size == 0) return;

			rsxBindVertexArrayAttrib(_driver->getGcmContext(), index, 0, 0, 0, 0, GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_RSX);

			vtxArray->frequency = 0;
			vtxArray->stride = 0;
			vtxArray->size = 0;
			vtxArray->type = GCM_VERTEX_DATA_TYPE_F32;
			vtxArray->location = GCM_LOCATION_RSX;
			vtxArray->offset = 0;
		}

		void CRSXShaderPipeline::updateConstants()
		{
			CRSXCgShader *shaders[2] = {
				_material->getVertexShader(),
				_material->getFragmentShader()
			};

			_activeParameters.set_used(0);

			for(s32 i=0;i < 2;i++) {
				CRSXCgShader *shader = shaders[i];
				core::map<core::stringc, CRSXShaderParam*>::Iterator it = shader->params.getIterator();

				while(!it.atEnd()) {
					core::map<core::stringc, CRSXShaderParam*>::Node *n = it.getNode();

					if(n != NULL && n->getValue()->getVariability() == EPV_UNIFORM) {
						core::map<core::stringc, CRSXShaderConstant>::Node *pn = _parameters.find(n->getKey());

						if(pn != NULL) {
							CRSXShaderParam *param = n->getValue();
							CRSXShaderConstant& constant = pn->getValue();

							constant.setParam(param, shader);
							_activeParameters.push_back(constant);
						}
					}
					it++;
				}
			}
		}
	}
}



