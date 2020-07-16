/*
 * rsxstate.cpp
 *
 *  Created on: Jun 19, 2013
 *      Author: mike
 */

#include "rsxstate.h"

namespace irr
{
	namespace video
	{
		CRSXState::CRSXState(gcmContextData *context) : _gcmContext(context)
		{
			reset();
		}

		CRSXState::~CRSXState()
		{

		}

		void CRSXState::reset()
		{
			state.blend_eq_color = GCM_FUNC_ADD;
			state.blend_eq_alpha = GCM_FUNC_ADD;

			state.blend_func_sfColor = GCM_ONE;
			state.blend_func_dfColor = GCM_ZERO;
			state.blend_func_sfAlpha = GCM_ONE;
			state.blend_func_dfAlpha = GCM_ZERO;

			state.blend_enable = GCM_FALSE;
			state.blend_enable_mrt1 = GCM_FALSE;
			state.blend_enable_mrt2 = GCM_FALSE;
			state.blend_enable_mrt3 = GCM_FALSE;

			state.depth_mask = GCM_TRUE;
			state.depth_test = GCM_FALSE;
			state.depth_func = GCM_LESS;

			state.color_mask = (GCM_COLOR_MASK_A | GCM_COLOR_MASK_R | GCM_COLOR_MASK_G | GCM_COLOR_MASK_B);
			state.color_mask_mrt = 0;

			state.clear_color = 0;
			state.clear_mask = (GCM_CLEAR_A | GCM_CLEAR_R | GCM_CLEAR_G | GCM_CLEAR_B | GCM_CLEAR_Z | GCM_CLEAR_S);
			state.clear_depth_stencil = 0xffffff00;

			state.stencil_func_func = GCM_ALWAYS;
			state.stencil_func_ref = 0;
			state.stencil_func_mask = 0xffffffff;

			state.stencil_back_func_func = GCM_ALWAYS;
			state.stencil_back_func_ref = 0;
			state.stencil_back_func_mask = 0xffffffff;

			state.stencil_mask_mask = 0xffffffff;
			state.stencil_back_mask_mask = 0xffffffff;

			state.stencil_op_fail = GCM_KEEP;
			state.stencil_op_depth_fail = GCM_KEEP;
			state.stencil_op_depth_pass = GCM_KEEP;

			state.stencil_back_op_fail = GCM_KEEP;
			state.stencil_back_op_depth_fail = GCM_KEEP;
			state.stencil_back_op_depth_pass = GCM_KEEP;

			state.stencil_test_enable = GCM_FALSE;
			state.stencil_test_enable_two_sided = GCM_FALSE;

			state.cull_face_mode = GCM_CULL_BACK;
			state.cull_face_enable = GCM_FALSE;

			state.anti_aliasing_enabled = GCM_FALSE;
			state.anti_alias_alpha_to_coverage = GCM_FALSE;
			state.anti_alias_alpha_to_one = GCM_FALSE;
			state.anti_alias_sample_mask = 0xffff;

			state.cull_near_far_enable = GCM_TRUE;
			state.zclamp_enable = GCM_FALSE;
			state.cull_ignoreW = GCM_FALSE;

			state.shade_model = GCM_SHADE_MODEL_SMOOTH;
			state.front_face = GCM_FRONTFACE_CW;

			state.back_polygon_mode = GCM_POLYGON_MODE_FILL;
			state.front_polygon_mode = GCM_POLYGON_MODE_FILL;

			state.z_cull_moveforward_limit = 0x100;
			state.z_cull_pushback_limit = 0x100;

			state.polygon_offset_fill_enable = GCM_FALSE;
			state.polygon_offset_factor = 0.0f;
			state.polygon_offset_units = 0.0f;

			state.vertex_shader_program = NULL;
			state.vertex_shader_program_ucode = NULL;

			state.fragment_shader_program = NULL;
			state.fragment_shader_offset = 0;

			for(u32 i=0;i < 16;i++) {
				state.textureControl[i].enable = GCM_FALSE;
				state.textureControl[i].minLod = 0<<8;
				state.textureControl[i].maxLod = 12<<8;
				state.textureControl[i].maxAniso = GCM_TEXTURE_MAX_ANISO_1;

				state.textureAddress[i].wraps = GCM_TEXTURE_REPEAT;
				state.textureAddress[i].wrapt = GCM_TEXTURE_REPEAT;
				state.textureAddress[i].wrapr = GCM_TEXTURE_CLAMP_TO_EDGE;
				state.textureAddress[i].unsignedRemap = GCM_TEXTURE_UNSIGNED_REMAP_NORMAL;
				state.textureAddress[i].zfunc = GCM_TEXTURE_ZFUNC_NEVER;
				state.textureAddress[i].gamma = 0;

				state.textureFilter[i].bias = 0;
				state.textureFilter[i].min = GCM_TEXTURE_NEAREST_MIPMAP_LINEAR;
				state.textureFilter[i].mag = GCM_TEXTURE_LINEAR;
				state.textureFilter[i].conv = GCM_TEXTURE_CONVOLUTION_QUINCUNX;

				state.currentTexture[i] = NULL;

				state.vertexDataArray[i].frequency = 0;
				state.vertexDataArray[i].stride = 0;
				state.vertexDataArray[i].size = 0;
				state.vertexDataArray[i].type = GCM_VERTEX_DATA_TYPE_F32;
				state.vertexDataArray[i].location = GCM_LOCATION_RSX;
				state.vertexDataArray[i].offset = 0;
			}
		}

		void CRSXState::setColorMask(u32 mask)
		{
			if(state.color_mask != mask) {
				rsxSetColorMask(_gcmContext, mask);

				state.color_mask = mask;
			}
		}

		void CRSXState::setClearColor(u32 color)
		{
			if(state.clear_color != color) {
				rsxSetClearColor(_gcmContext, color);

				state.clear_color = color;
			}
		}

		void CRSXState::setClearSurface(u32 mask)
		{
			if(state.clear_mask != mask) {
				rsxClearSurface(_gcmContext, mask);

				state.clear_mask = mask;
			}
		}

		void CRSXState::setShadeModel(u32 model)
		{
			if(state.shade_model != model) {
				rsxSetShadeModel(_gcmContext, model);

				state.shade_model = model;
			}
		}

		void CRSXState::setBackPolygonMode(u32 mode)
		{
			if(state.back_polygon_mode != mode) {
				rsxSetBackPolygonMode(_gcmContext, mode);

				state.back_polygon_mode = mode;
			}
		}

		void CRSXState::setFrontPolygonMode(u32 mode)
		{
			if(state.front_polygon_mode != mode) {
				rsxSetFrontPolygonMode(_gcmContext, mode);

				state.front_polygon_mode = mode;
			}
		}

		void CRSXState::setCullFace(u32 face_mode)
		{
			if(state.cull_face_mode != face_mode) {
				rsxSetCullFace(_gcmContext, face_mode);

				state.cull_face_mode = face_mode;
			}
		}

		void CRSXState::setCullFaceEnable(u32 enable)
		{
			if(state.cull_face_enable != enable) {
				rsxSetCullFaceEnable(_gcmContext, enable);

				state.cull_face_enable = enable;
			}
		}

		void CRSXState::setBlendEnable(u32 enable)
		{
			if(state.blend_enable != enable) {
				rsxSetBlendEnable(_gcmContext, enable);

				state.blend_enable = enable;
			}
		}

		void CRSXState::setBlendFunc(u32 sfColor, u32 dfColor, u32 sfAlpha, u32 dfAlpha)
		{
			if(state.blend_func_sfColor != sfColor || state.blend_func_dfColor != dfColor ||
			   state.blend_func_sfAlpha != sfAlpha || state.blend_func_dfAlpha != dfAlpha)
			{
				rsxSetBlendFunc(_gcmContext, sfColor, dfColor, sfAlpha, dfAlpha);

				state.blend_func_sfColor = sfColor;
				state.blend_func_dfColor = dfColor;
				state.blend_func_sfAlpha = sfAlpha;
				state.blend_func_dfAlpha = dfAlpha;
			}
		}

		void CRSXState::setBlendEquation(u32 color, u32 alpha)
		{
			if(state.blend_eq_color != color || state.blend_eq_alpha != alpha) {
				rsxSetBlendEquation(_gcmContext, color, alpha);

				state.blend_eq_color = color;
				state.blend_eq_alpha = alpha;
			}
		}

		void CRSXState::setDepthTestEnable(u32 enable)
		{
			if(state.depth_test != enable) {
				rsxSetDepthTestEnable(_gcmContext, enable);

				state.depth_test = enable;
			}
		}

		void CRSXState::setDepthFunc(u32 func)
		{
			if(state.depth_func != func) {
				rsxSetDepthFunc(_gcmContext, func);

				state.depth_func = func;
			}
		}

		void CRSXState::setDepthWriteEnable(u32 enable)
		{
			if(state.depth_mask != enable) {
				rsxSetDepthWriteEnable(_gcmContext, enable);

				state.depth_mask = enable;
			}
		}

		void CRSXState::setBackStencilEnable(u32 enable)
		{
			if(state.stencil_test_enable_two_sided != enable) {
				rsxSetTwoSidedStencilTestEnable(_gcmContext, enable);

				state.stencil_test_enable_two_sided = enable;
			}
		}

		void CRSXState::setBackStencilMask(u32 mask)
		{
			if(state.stencil_back_mask_mask != mask) {
				rsxSetBackStencilMask(_gcmContext, mask);

				state.stencil_back_mask_mask = mask;
			}
		}

		void CRSXState::setBackStencilFunc(u32 func, u32 ref, u32 mask)
		{
			if(state.stencil_back_func_func != func || state.stencil_back_func_ref != ref || state.stencil_back_func_mask != mask) {
				rsxSetBackStencilFunc(_gcmContext, func, ref, mask);

				state.stencil_back_func_func = func;
				state.stencil_back_func_ref = ref;
				state.stencil_back_func_mask = mask;
			}
		}

		void CRSXState::setBackStencilOp(u32 fail, u32 depthFail, u32 depthPass)
		{
			if(state.stencil_back_op_fail != fail || state.stencil_back_op_depth_fail != depthFail || state.stencil_back_op_depth_pass != depthPass) {
				rsxSetBackStencilOp(_gcmContext, fail, depthFail, depthPass);

				state.stencil_back_op_fail = fail;
				state.stencil_back_op_depth_fail = depthFail;
				state.stencil_back_op_depth_pass = depthPass;
			}
		}

		void CRSXState::setFrontStencilEnable(u32 enable)
		{
			if(state.stencil_test_enable != enable) {
				rsxSetStencilTestEnable(_gcmContext, enable);

				state.stencil_test_enable = enable;
			}
		}

		void CRSXState::setFrontStencilMask(u32 mask)
		{
			if(state.stencil_mask_mask != mask) {
				rsxSetStencilMask(_gcmContext, mask);

				state.stencil_mask_mask = mask;
			}
		}

		void CRSXState::setFrontStencilFunc(u32 func, u32 ref, u32 mask)
		{
			if(state.stencil_func_func != func || state.stencil_func_ref != ref || state.stencil_func_mask != mask) {
				rsxSetStencilFunc(_gcmContext, func, ref, mask);

				state.stencil_func_func = func;
				state.stencil_func_ref = ref;
				state.stencil_func_mask = mask;
			}
		}

		void CRSXState::setFrontStencilOp(u32 fail, u32 depthFail, u32 depthPass)
		{
			if(state.stencil_op_fail != fail || state.stencil_op_depth_fail != depthFail || state.stencil_op_depth_pass != depthPass) {
				rsxSetStencilOp(_gcmContext, fail, depthFail, depthPass);

				state.stencil_op_fail = fail;
				state.stencil_op_depth_fail = depthFail;
				state.stencil_op_depth_pass = depthPass;
			}
		}
	}
}
