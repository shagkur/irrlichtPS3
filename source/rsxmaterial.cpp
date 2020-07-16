/*
 * rsxmaterial.cpp
 *
 *  Created on: Feb 6, 2014
 *      Author: mike
 */

#include "rsxdriver.h"
#include "rsxmaterial.h"

namespace irr
{
	namespace video
	{
		CRSXMaterial::CRSXMaterial(CRSXDriver *driver)
		: _driver(driver), _gfxState(driver->getRSXState()), _vpShader(NULL),
		  _fpShader(NULL), _fpDepthShader(NULL)
		{
			_state.blend_eq_color = GCM_FUNC_ADD;
			_state.blend_eq_alpha = GCM_FUNC_ADD;

			_state.blend_func_sfColor = GCM_ONE;
			_state.blend_func_dfColor = GCM_ZERO;
			_state.blend_func_sfAlpha = GCM_ONE;
			_state.blend_func_dfAlpha = GCM_ZERO;

			_state.blend_enable = GCM_FALSE;
			_state.blend_enable_mrt1 = GCM_FALSE;
			_state.blend_enable_mrt2 = GCM_FALSE;
			_state.blend_enable_mrt3 = GCM_FALSE;

			_state.depth_mask = GCM_TRUE;
			_state.depth_test = GCM_FALSE;
			_state.depth_func = GCM_LESS;

			_state.color_mask = _gfxState->state.color_mask;
			_state.color_mask_mrt = _gfxState->state.color_mask_mrt;

			_state.stencil_func_func = GCM_ALWAYS;
			_state.stencil_func_ref = 0;
			_state.stencil_func_mask = 0xffffffff;

			_state.stencil_back_func_func = GCM_ALWAYS;
			_state.stencil_back_func_ref = 0;
			_state.stencil_back_func_mask = 0xffffffff;

			_state.stencil_mask_mask = 0xffffffff;
			_state.stencil_back_mask_mask = 0xffffffff;

			_state.stencil_op_fail = GCM_KEEP;
			_state.stencil_op_depth_fail = GCM_KEEP;
			_state.stencil_op_depth_pass = GCM_KEEP;

			_state.stencil_back_op_fail = GCM_KEEP;
			_state.stencil_back_op_depth_fail = GCM_KEEP;
			_state.stencil_back_op_depth_pass = GCM_KEEP;

			_state.stencil_test_enable = GCM_FALSE;
			_state.stencil_test_enable_two_sided = GCM_FALSE;

			_state.cull_face_mode = GCM_CULL_BACK;
			_state.cull_face_enable = GCM_TRUE;

			_state.z_cull_moveforward_limit = 0x100;
			_state.z_cull_pushback_limit = 0x100;

			_state.anti_aliasing_enabled = GCM_FALSE;
			_state.anti_alias_alpha_to_coverage = GCM_FALSE;
			_state.anti_alias_alpha_to_one = GCM_FALSE;
			_state.anti_alias_sample_mask = 0xffff;
		}

		CRSXMaterial::CRSXMaterial(const CRSXMaterial& other)
		{
			*this = other;
		}

		CRSXMaterial& CRSXMaterial::operator =(const CRSXMaterial& other)
		{
			_state = other._state;
			_driver = other._driver;
			_gfxState = other._gfxState;
			_vpShader = other._vpShader;
			_fpShader = other._fpShader;
			_fpDepthShader = other._fpDepthShader;

			return  *this;
		}

		CRSXMaterial::~CRSXMaterial()
		{

		}

		void CRSXMaterial::addToCmdBuffer()
		{
			addPrim();

			_fpShader->addToCmdBuffer();
		}

		void CRSXMaterial::setCullFace(const u32 face_mode)
		{
			_state.cull_face_mode = face_mode;
		}

		void CRSXMaterial::setCullFaceEnable(const u32 enable)
		{
			_state.cull_face_enable = enable;
		}

		void CRSXMaterial::setDepthTestEnable(const u32 enable)
		{
			_state.depth_test = enable;
		}

		void CRSXMaterial::setDepthMask(const u32 enable)
		{
			_state.depth_mask = enable;
		}

		void CRSXMaterial::setDepthFunc(const u32 func)
		{
			_state.depth_func = func;
		}

		void CRSXMaterial::setZCullControl(const u8 zCullDir, const u8 zCullFormat)
		{

		}

		void CRSXMaterial::setSCullControl(const u8 sFunc, const u8 sRef, const u8 sMask)
		{

		}

		void CRSXMaterial::setZMinMaxControl(const u32 cullNearFarEnable, const u32 zClampEnable, const u32 cullIgnoreW)
		{
			_state.cull_near_far_enable = cullNearFarEnable;
			_state.zclamp_enable = zClampEnable;
			_state.cull_ignoreW = cullIgnoreW;
		}

		void CRSXMaterial::setZCullLimit(const u16 moveForwardLimit, const u16 pushBackLimit)
		{
			_state.z_cull_moveforward_limit = moveForwardLimit;
			_state.z_cull_pushback_limit = pushBackLimit;
		}

		void CRSXMaterial::setBlendEnable(const u32 enable)
		{
			_state.blend_enable = enable;
		}

		void CRSXMaterial::setBlendEnableMRT(const u32 enable1, const u32 enable2, const u32 enable3)
		{
			_state.blend_enable_mrt1 = enable1;
			_state.blend_enable_mrt2 = enable2;
			_state.blend_enable_mrt3 = enable3;
		}

		void CRSXMaterial::setBlendEquation(const u16 color, const u16 alpha)
		{
			_state.blend_eq_color = color;
			_state.blend_eq_alpha = alpha;
		}

		void CRSXMaterial::setBlendFunc(const u16 sfColor, const u16 dfColor, const u16 sfAlpha, const u16 dfAlpha)
		{
			_state.blend_func_sfColor = sfColor;
			_state.blend_func_dfColor = dfColor;
			_state.blend_func_sfAlpha = sfAlpha;
			_state.blend_func_dfAlpha = dfAlpha;
		}

		void CRSXMaterial::setStencilTestEnable(const u32 enable)
		{
			_state.stencil_test_enable = enable;
		}

		void CRSXMaterial::setTwoSidedStencilTestEnable(const u32 enable)
		{
			_state.stencil_test_enable_two_sided = enable;
		}

		void CRSXMaterial::setStencilMask(const u32 mask)
		{
			_state.stencil_mask_mask = mask;
		}

		void CRSXMaterial::setBackStencilMask(const u32 mask)
		{
			_state.stencil_back_mask_mask = mask;
		}

		void CRSXMaterial::setStencilFunc(const u32 func, const u32 ref, const u32 mask)
		{
			_state.stencil_func_func = func;
			_state.stencil_func_ref = ref;
			_state.stencil_func_mask = mask;
		}

		void CRSXMaterial::setBackStencilFunc(const u32 func, const u32 ref, const u32 mask)
		{
			_state.stencil_back_func_func = func;
			_state.stencil_back_func_ref = ref;
			_state.stencil_back_func_mask = mask;
		}

		void CRSXMaterial::setShadeModel(const u32 model)
		{
			_state.shade_model = model;
		}

		void CRSXMaterial::setBackPolygonMode(const u32 mode)
		{
			_state.back_polygon_mode = mode;
		}

		void CRSXMaterial::setFrontPolygonMode(const u32 mode)
		{
			_state.front_polygon_mode = mode;
		}

		void CRSXMaterial::setPolygonOffsetFillEnable(const u32 enable)
		{
			_state.polygon_offset_fill_enable = enable;
		}

		void CRSXMaterial::setPolygonOffset(const f32 factor, const f32 units)
		{
			_state.polygon_offset_factor = factor;
			_state.polygon_offset_units = units;
		}

		void CRSXMaterial::setColorMask(const u32 mask)
		{
			_state.color_mask = mask;
		}

		void CRSXMaterial::setColorMaskMRT(const u32 mask)
		{
			_state.color_mask_mrt = mask;
		}

		void CRSXMaterial::pushState()
		{
			_stateStack.push_back(_state);
		}

		void CRSXMaterial::popState()
		{
			if(_stateStack.size() > 0) {
				_state = _stateStack[0];
				_stateStack.erase(0);
			}
		}

		void CRSXMaterial::addPrim()
		{
			gcmContextData *context = _driver->getGcmContext();

			_vpShader->addToCmdBuffer();

			if(_state.depth_func != _gfxState->state.depth_func) {
				rsxSetDepthFunc(context, _state.depth_func);
				_gfxState->state.depth_func = _state.depth_func;
			}

			if(_state.depth_mask != _gfxState->state.depth_mask) {
				rsxSetDepthWriteEnable(context, _state.depth_mask);
				_gfxState->state.depth_mask = _state.depth_mask;
			}

			if(_state.depth_test != _gfxState->state.depth_test) {
				rsxSetDepthTestEnable(context, _state.depth_test);
				_gfxState->state.depth_test = _state.depth_test;
			}

			if(_state.z_cull_moveforward_limit != _gfxState->state.z_cull_moveforward_limit ||
			   _state.z_cull_pushback_limit != _gfxState->state.z_cull_pushback_limit)
			{
				rsxSetZCullLimit(context, _state.z_cull_moveforward_limit, _state.z_cull_pushback_limit);
				_gfxState->state.z_cull_moveforward_limit = _state.z_cull_moveforward_limit;
				_gfxState->state.z_cull_pushback_limit = _state.z_cull_pushback_limit;
			}

			if(_state.cull_near_far_enable != _gfxState->state.cull_near_far_enable ||
			   _state.zclamp_enable != _gfxState->state.zclamp_enable ||
			   _state.cull_ignoreW != _gfxState->state.cull_ignoreW)
			{
				rsxSetZControl(context, _state.cull_near_far_enable, _state.zclamp_enable, _state.cull_ignoreW);

				_gfxState->state.cull_near_far_enable = _state.cull_near_far_enable;
				_gfxState->state.zclamp_enable = _state.zclamp_enable;
				_gfxState->state.cull_ignoreW = _state.cull_ignoreW;
			}

			if(_state.cull_face_enable != _gfxState->state.cull_face_enable) {
				rsxSetCullFaceEnable(context, _state.cull_face_enable);
				_gfxState->state.cull_face_enable = _state.cull_face_enable;
			}

			if(_state.stencil_func_func != _gfxState->state.stencil_func_func ||
			   _state.stencil_func_ref != _gfxState->state.stencil_func_ref ||
			   _state.stencil_func_mask != _gfxState->state.stencil_func_mask)
			{
				rsxSetStencilFunc(context, _state.stencil_func_func, _state.stencil_func_ref, _state.stencil_func_mask);
				_gfxState->state.stencil_func_func = _state.stencil_func_func;
				_gfxState->state.stencil_func_ref = _state.stencil_func_ref;
				_gfxState->state.stencil_func_mask = _state.stencil_func_mask;
			}

			if(_state.stencil_mask_mask != _gfxState->state.stencil_mask_mask) {
				rsxSetStencilMask(context, _state.stencil_mask_mask);
				_gfxState->state.stencil_mask_mask = _state.stencil_mask_mask;
			}

			if(_state.stencil_op_fail != _gfxState->state.stencil_op_fail ||
			   _state.stencil_op_depth_fail != _gfxState->state.stencil_op_depth_fail ||
			   _state.stencil_op_depth_pass != _gfxState->state.stencil_op_depth_pass)
			{
				rsxSetStencilOp(context, _state.stencil_op_fail, _state.stencil_op_depth_fail, _state.stencil_op_depth_pass);
				_gfxState->state.stencil_op_fail = _state.stencil_op_fail;
				_gfxState->state.stencil_op_depth_fail = _state.stencil_op_depth_fail;
				_gfxState->state.stencil_op_depth_pass = _state.stencil_op_depth_pass;
			}

			if(_state.stencil_test_enable != _gfxState->state.stencil_test_enable) {
				rsxSetStencilTestEnable(context, _state.stencil_test_enable);
				_gfxState->state.stencil_test_enable = _state.stencil_test_enable;
			}

			if(_state.stencil_back_func_func != _gfxState->state.stencil_back_func_func ||
			   _state.stencil_back_func_ref != _gfxState->state.stencil_back_func_ref ||
			   _state.stencil_back_func_mask != _gfxState->state.stencil_back_func_mask)
			{
				rsxSetBackStencilFunc(context, _state.stencil_back_func_func, _state.stencil_back_func_ref, _state.stencil_back_func_mask);
				_gfxState->state.stencil_back_func_func = _state.stencil_back_func_func;
				_gfxState->state.stencil_back_func_ref = _state.stencil_back_func_ref;
				_gfxState->state.stencil_back_func_mask = _state.stencil_back_func_mask;
			}

			if(_state.stencil_back_mask_mask != _gfxState->state.stencil_back_mask_mask) {
				rsxSetBackStencilMask(context, _state.stencil_back_mask_mask);
				_gfxState->state.stencil_back_mask_mask = _state.stencil_back_mask_mask;
			}

			if(_state.stencil_back_op_fail != _gfxState->state.stencil_back_op_fail ||
			   _state.stencil_back_op_depth_fail != _gfxState->state.stencil_back_op_depth_fail ||
			   _state.stencil_back_op_depth_pass != _gfxState->state.stencil_back_op_depth_pass)
			{
				rsxSetBackStencilOp(context, _state.stencil_back_op_fail, _state.stencil_back_op_depth_fail, _state.stencil_back_op_depth_pass);
				_gfxState->state.stencil_back_op_fail = _state.stencil_back_op_fail;
				_gfxState->state.stencil_back_op_depth_fail = _state.stencil_back_op_depth_fail;
				_gfxState->state.stencil_back_op_depth_pass = _state.stencil_back_op_depth_pass;
			}

			if(_state.stencil_test_enable_two_sided != _gfxState->state.stencil_test_enable_two_sided) {
				rsxSetTwoSidedStencilTestEnable(context, _state.stencil_test_enable_two_sided);
				_gfxState->state.stencil_test_enable_two_sided = _state.stencil_test_enable_two_sided;
			}

			if(_state.cull_face_mode != _gfxState->state.cull_face_mode) {
				rsxSetCullFace(context, _state.cull_face_mode);
				_gfxState->state.cull_face_mode = _state.cull_face_mode;
			}

			if(_state.blend_eq_color != _gfxState->state.blend_eq_color ||
			   _state.blend_eq_alpha != _gfxState->state.blend_eq_alpha)
			{
				rsxSetBlendEquation(context, _state.blend_eq_color, _state.blend_eq_alpha);
				_gfxState->state.blend_eq_color = _state.blend_eq_color;
				_gfxState->state.blend_eq_alpha = _state.blend_eq_alpha;
			}

			if(_state.blend_func_sfColor != _gfxState->state.blend_func_sfColor ||
			   _state.blend_func_dfColor != _gfxState->state.blend_func_dfColor ||
			   _state.blend_func_sfAlpha != _gfxState->state.blend_func_sfAlpha ||
			   _state.blend_func_dfAlpha != _gfxState->state.blend_func_dfAlpha)
			{
				rsxSetBlendFunc(context, _state.blend_func_sfColor, _state.blend_func_dfColor, _state.blend_func_sfAlpha, _state.blend_func_dfAlpha);
				_gfxState->state.blend_func_sfColor = _state.blend_func_sfColor;
				_gfxState->state.blend_func_dfColor = _state.blend_func_dfColor;
				_gfxState->state.blend_func_sfAlpha = _state.blend_func_sfAlpha;
				_gfxState->state.blend_func_dfAlpha = _state.blend_func_dfAlpha;
			}

			if(_state.blend_enable != _gfxState->state.blend_enable) {
				rsxSetBlendEnable(context, _state.blend_enable);
				_gfxState->state.blend_enable = _state.blend_enable;
			}

			if(_state.blend_enable_mrt1 != _gfxState->state.blend_enable_mrt1 ||
			   _state.blend_enable_mrt2 != _gfxState->state.blend_enable_mrt2 ||
			   _state.blend_enable_mrt3 != _gfxState->state.blend_enable_mrt3)
			{
				rsxSetBlendEnableMrt(context, _state.blend_enable_mrt1, _state.blend_enable_mrt2, _state.blend_enable_mrt3);
				_gfxState->state.blend_enable_mrt1 = _state.blend_enable_mrt1;
				_gfxState->state.blend_enable_mrt2 = _state.blend_enable_mrt2;
				_gfxState->state.blend_enable_mrt3 = _state.blend_enable_mrt3;
			}

			if(_state.stencil_test_enable != _gfxState->state.stencil_test_enable) {
				rsxSetStencilTestEnable(context, _state.stencil_test_enable);

				_gfxState->state.stencil_test_enable = _state.stencil_test_enable;
			}

			if(_state.stencil_test_enable_two_sided != _gfxState->state.stencil_test_enable_two_sided) {
				rsxSetTwoSidedStencilTestEnable(context, _state.stencil_test_enable_two_sided);

				_gfxState->state.stencil_test_enable_two_sided = _state.stencil_test_enable_two_sided;
			}

			if(_state.stencil_mask_mask != _gfxState->state.stencil_mask_mask) {
				rsxSetStencilMask(context, _state.stencil_mask_mask);

				_gfxState->state.stencil_mask_mask = _state.stencil_mask_mask;
			}

			if(_state.stencil_back_mask_mask != _gfxState->state.stencil_back_mask_mask) {
				rsxSetBackStencilMask(context, _state.stencil_back_mask_mask);

				_gfxState->state.stencil_back_mask_mask = _state.stencil_back_mask_mask;
			}

			if(_state.stencil_func_func != _gfxState->state.stencil_func_func ||
			   _state.stencil_func_ref != _gfxState->state.stencil_func_ref ||
			   _state.stencil_func_mask != _gfxState->state.stencil_func_mask)
			{
				rsxSetStencilFunc(context, _state.stencil_func_func, _state.stencil_func_ref, _state.stencil_func_mask);

				_gfxState->state.stencil_func_func = _state.stencil_func_func;
				_gfxState->state.stencil_func_ref = _state.stencil_func_ref;
				_gfxState->state.stencil_func_mask = _state.stencil_func_mask;
			}

			if(_state.stencil_back_func_func != _gfxState->state.stencil_back_func_func ||
			   _state.stencil_back_func_ref != _gfxState->state.stencil_back_func_ref ||
			   _state.stencil_back_func_mask != _gfxState->state.stencil_back_func_mask)
			{
				rsxSetBackStencilFunc(context, _state.stencil_back_func_func, _state.stencil_back_func_ref, _state.stencil_back_func_mask);

				_gfxState->state.stencil_back_func_func = _state.stencil_back_func_func;
				_gfxState->state.stencil_back_func_ref = _state.stencil_back_func_ref;
				_gfxState->state.stencil_back_func_mask = _state.stencil_back_func_mask;
			}

			if(_state.shade_model != _gfxState->state.shade_model) {
				rsxSetShadeModel(context, _state.shade_model);
				_gfxState->state.shade_model = _state.shade_model;
			}

			if(_state.back_polygon_mode != _gfxState->state.back_polygon_mode) {
				rsxSetBackPolygonMode(context, _state.back_polygon_mode);
				_gfxState->state.back_polygon_mode = _state.back_polygon_mode;
			}

			if(_state.front_polygon_mode != _gfxState->state.front_polygon_mode) {
				rsxSetFrontPolygonMode(context, _state.front_polygon_mode);
				_gfxState->state.front_polygon_mode = _state.front_polygon_mode;
			}

			if(_state.polygon_offset_fill_enable != _gfxState->state.polygon_offset_fill_enable) {
				rsxSetPolygonOffsetFillEnable(context, _state.polygon_offset_fill_enable);

				_gfxState->state.polygon_offset_fill_enable = _state.polygon_offset_fill_enable;
			}

			if(_state.polygon_offset_factor != _gfxState->state.polygon_offset_factor ||
			   _state.polygon_offset_units != _gfxState->state.polygon_offset_units)
			{
				rsxSetPolygonOffset(context, _state.polygon_offset_factor, _state.polygon_offset_units);

				_gfxState->state.polygon_offset_factor = _state.polygon_offset_factor;
				_gfxState->state.polygon_offset_units = _state.polygon_offset_units;
			}

			if(_state.color_mask != _gfxState->state.color_mask) {
				rsxSetColorMask(context, _state.color_mask);
				_gfxState->state.color_mask = _state.color_mask;
			}

			if(_state.color_mask_mrt != _gfxState->state.color_mask_mrt) {
				rsxSetColorMaskMRT(context, _state.color_mask_mrt);
				_gfxState->state.color_mask_mrt = _state.color_mask_mrt;
			}

			if(_state.anti_aliasing_enabled != _gfxState->state.anti_aliasing_enabled ||
			   _state.anti_alias_alpha_to_coverage != _gfxState->state.anti_alias_alpha_to_coverage ||
			   _state.anti_alias_alpha_to_one != _gfxState->state.anti_alias_alpha_to_one ||
			   _state.anti_alias_sample_mask != _gfxState->state.anti_alias_sample_mask)
			{
				rsxSetAntialiasingControl(context, _state.anti_aliasing_enabled, _state.anti_alias_alpha_to_coverage, _state.anti_alias_alpha_to_one, _state.anti_alias_sample_mask);
				_gfxState->state.anti_aliasing_enabled = _state.anti_aliasing_enabled;
				_gfxState->state.anti_alias_alpha_to_coverage = _state.anti_alias_alpha_to_coverage;
				_gfxState->state.anti_alias_alpha_to_one = _state.anti_alias_alpha_to_one;
				_gfxState->state.anti_alias_sample_mask = _state.anti_alias_sample_mask;
			}
		}
	}
}


