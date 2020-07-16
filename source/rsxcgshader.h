/*
 * rsxcgshader.h
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#ifndef RSXCGSHADER_H_
#define RSXCGSHADER_H_

#include "ecgprofiles.h"
#include "irrstring.h"
#include "irrmap.h"
#include "matrix4.h"
#include "rsxstate.h"
#include "rsxshaderparambuffer.h"

namespace irr
{
	namespace video
	{
		class CRSXDriver;
		class CRSXShaderParam;

		class CRSXCgShader
		{
		public:
			CRSXCgShader(CRSXDriver *driver);
			virtual ~CRSXCgShader();

			virtual void setParameter(void *param, const float *data, u32 size) = 0;
			virtual void setParameter(void *param, const core::matrix4 *matrix) = 0;

			virtual void init(const void *shaderProgram, const core::stringc& name);

			const core::stringc& getName() const { return _shaderName; }

			E_CG_PROFILE getProfile() const { return _profile; }

			core::map<core::stringc, CRSXShaderParam*> params;

		private:
			void init(const rsxVertexProgram *vertexProgram);
			void init(const rsxFragmentProgram *fragmentProgram);

		protected:
			s32 decodeParamName(const core::stringc& fullName, core::stringc& returnName);

			E_CG_PROFILE _profile;

			CRSXState *_state;
			CRSXDriver *_driver;

			void *_shaderUCode;
			void *_ucode;
			u32 _ucodeSize;

			core::stringc _shaderName;
			void *_shaderProgram;

			core::map<void*, CRSXShaderParamBuffer*> _paramMap;
		};
	}
}

#endif /* RSXCGSHADER_H_ */
