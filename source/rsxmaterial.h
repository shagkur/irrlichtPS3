/*
 * rsxmaterial.h
 *
 *  Created on: Feb 6, 2014
 *      Author: mike
 */

#ifndef RSXMATERIAL_H_
#define RSXMATERIAL_H_

#include "irrarray.h"
#include "rsxfragmentshader.h"
#include "rsxvertexshader.h"
#include "rsxstate.h"

namespace irr
{
	namespace video
	{
		class CRSXDriver;
		class CRSXTexture;

		class CRSXMaterial
		{
		public:
			CRSXMaterial() {}
			CRSXMaterial(CRSXDriver *driver);
			CRSXMaterial(const CRSXMaterial& other);
			virtual ~CRSXMaterial();

			CRSXMaterial& operator =(const CRSXMaterial& other);

			CRSXVertexShader* getVertexShader() const { return _vpShader; }
			CRSXFragmentShader* getFragmentShader() const { return _fpShader; }
			CRSXFragmentShader* getFragmentDepthShader() const { return _fpDepthShader; }

			void setVertexShader(CRSXVertexShader *shader) { _vpShader = shader; }
			void setFragmentShader(CRSXFragmentShader *shader) { _fpShader = shader; }
			void setFragmentDepthShader(CRSXFragmentShader *shader) { _fpDepthShader = shader; }

			void addToCmdBuffer();

			void setCullFace(const u32 face_mode);
			void setCullFaceEnable(const u32 enable);

			void setDepthTestEnable(const u32 enable);
			void setDepthMask(const u32 enable);
			void setDepthFunc(const u32 func);

			void setZCullControl(const u8 zCullDir, const u8 zCullFormat);
			void setSCullControl(const u8 sFunc, const u8 sRef, const u8 sMask);
			void setZCullLimit(const u16 moveForwardLimit, const u16 pushBackLimit);
			void setZMinMaxControl(const u32 cullNearFarEnable, const u32 zClampEnable, const u32 cullIgnoreW);

			void setColorMask(const u32 mask);
			void setColorMaskMRT(const u32 mask);

			void setBlendEnable(const u32 enable);
			void setBlendEnableMRT(const u32 enable1, const u32 enable2, const u32 enable3);
			void setBlendEquation(const u16 color, const u16 alpha);
			void setBlendFunc(const u16 sfColor, const u16 dfColor, const u16 sfAlpha, const u16 dfAlpha);

			void setStencilTestEnable(const u32 enable);
			void setTwoSidedStencilTestEnable(const u32 enable);
			void setStencilMask(const u32 mask);
			void setBackStencilMask(const u32 mask);
			void setStencilFunc(const u32 func, const u32 ref, const u32 mask);
			void setBackStencilFunc(const u32 func, const u32 ref, const u32 mask);

			void setShadeModel(const u32 model);
			void setBackPolygonMode(const u32 mode);
			void setFrontPolygonMode(const u32 mode);

			void setPolygonOffsetFillEnable(const u32 enable);
			void setPolygonOffset(const f32 factor, const f32 units);

			void pushState();
			void popState();

		private:
			void addPrim();

			CRSXDriver *_driver;
			CRSXState *_gfxState;

			CRSXVertexShader *_vpShader;
			CRSXFragmentShader *_fpShader;
			CRSXFragmentShader *_fpDepthShader;

			rsx_state _state;

			core::array<rsx_state> _stateStack;
		};
	}
}

#endif /* RSXMATERIAL_H_ */
