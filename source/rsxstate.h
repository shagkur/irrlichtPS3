/*

 * rsxstate.h
 *
 *  Created on: Jun 19, 2013
 *      Author: mike
 */

#ifndef RSXSTATE_H_
#define RSXSTATE_H_

#include "vector3d.h"
#include "scolor.h"
#include "slight.h"

namespace irr
{
	namespace video
	{
		const u8 MAX_ANISO = 7;
		const u32 MAX_LIGHTS = 0xffffffff;
		const u32 MAX_TEXTURE_UNITS = 16;

		typedef struct
		{
			u32 enable;
			u16 minLod;
			u16 maxLod;
			u8 maxAniso;
		} textureControl_t;

		typedef struct
		{
			u8 wraps;
			u8 wrapt;
			u8 wrapr;
			u8 unsignedRemap;
			u8 zfunc;
			u8 gamma;
		} textureAddress_t;

		typedef struct
		{
			u16 bias;
			u8 min;
			u8 mag;
			u8 conv;
		} textureFilter_t;

		typedef struct
		{
			u16 frequency;
			u8 stride;
			u8 size;
			u8 type;
			u8 location;
			u32 offset;
		} vertexDataArray_t;

		typedef struct
		{
			u16 blend_eq_color;
			u16 blend_eq_alpha;

			u16 blend_func_sfColor;
			u16 blend_func_dfColor;
			u16 blend_func_sfAlpha;
			u16 blend_func_dfAlpha;

			u32 blend_enable;
			u32 blend_enable_mrt1;
			u32 blend_enable_mrt2;
			u32 blend_enable_mrt3;

			u32 depth_mask;
			u32 depth_test;
			u32 depth_func;

			u32 color_mask;
			u32 color_mask_mrt;

			u32 clear_color;
			u32 clear_mask;
			u32 clear_depth_stencil;

			u32 stencil_func_func;
			u32 stencil_func_ref;
			u32 stencil_func_mask;

			u32 stencil_back_func_func;
			u32 stencil_back_func_ref;
			u32 stencil_back_func_mask;

			u32 stencil_mask_mask;
			u32 stencil_back_mask_mask;

			u32 stencil_op_fail;
			u32 stencil_op_depth_fail;
			u32 stencil_op_depth_pass;

			u32 stencil_back_op_fail;
			u32 stencil_back_op_depth_fail;
			u32 stencil_back_op_depth_pass;

			u32 stencil_test_enable;
			u32 stencil_test_enable_two_sided;

			u32 cull_face_mode;
			u32 cull_face_enable;

			u32 anti_aliasing_enabled;
			u32 anti_alias_alpha_to_coverage;
			u32 anti_alias_alpha_to_one;
			u32 anti_alias_sample_mask;

			u32 cull_near_far_enable;
			u32 zclamp_enable;
			u32 cull_ignoreW;

			u32 shade_model;
			u32 front_face;

			u32 back_polygon_mode;
			u32 front_polygon_mode;

			u16 z_cull_moveforward_limit;
			u16 z_cull_pushback_limit;

			u32 polygon_offset_fill_enable;
			f32 polygon_offset_factor;
			f32 polygon_offset_units;

			rsxVertexProgram *vertex_shader_program;
			void *vertex_shader_program_ucode;

			rsxFragmentProgram *fragment_shader_program;
			u32 fragment_shader_offset;

			textureControl_t textureControl[16];
			textureAddress_t textureAddress[16];
			textureFilter_t textureFilter[16];
			void *currentTexture[16];

			vertexDataArray_t vertexDataArray[16];
		} rsx_state;

		class CRSXState
		{
		public:
			CRSXState(gcmContextData *context);
			virtual ~CRSXState();

			void reset();

			void setColorMask(u32 mask);
			void setClearColor(u32 color);
			void setClearSurface(u32 mask);

			void setShadeModel(u32 model);

			void setBackPolygonMode(u32 mode);
			void setFrontPolygonMode(u32 mode);

			void setCullFace(u32 face_mode);
			void setCullFaceEnable(u32 enable);

			void setBlendEnable(u32 enable);
			void setBlendFunc(u32 sfColor, u32 dfColor, u32 sfAlpha, u32 dfAlpha);
			void setBlendEquation(u32 color, u32 alpha);

			void setDepthTestEnable(u32 enable);
			void setDepthFunc(u32 func);
			void setDepthWriteEnable(u32 enable);

			void setBackStencilEnable(u32 enable);
			void setBackStencilMask(u32 mask);
			void setBackStencilFunc(u32 func, u32 ref, u32 mask);
			void setBackStencilOp(u32 fail, u32 depthFail, u32 depthPass);

			void setFrontStencilEnable(u32 enable);
			void setFrontStencilMask(u32 mask);
			void setFrontStencilFunc(u32 func, u32 ref, u32 mask);
			void setFrontStencilOp(u32 fail, u32 depthFail, u32 depthPass);

			rsx_state state;

		private:
			gcmContextData *_gcmContext;
		};
	}
}

#endif /* RSXSTATE_H_ */
