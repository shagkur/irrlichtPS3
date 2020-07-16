/*
 * rsxshaderparam.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#include "rsxcgshader.h"
#include "rsxshaderparam.h"

namespace irr
{
	namespace video
	{
		CRSXShaderParam::CRSXShaderParam(void *param, u32 elements, E_PARAM_DIR dir, E_PARAM_VARY vary, E_CG_PROFILE profile, CRSXCgShader *shader)
		: _shader(shader)
		{
			setParam(param, elements, dir, vary, profile);
		}

		CRSXShaderParam::~CRSXShaderParam()
		{

		}

		void CRSXShaderParam::setParam(void *param, u32 elements, E_PARAM_DIR dir, E_PARAM_VARY vary, E_CG_PROFILE profile)
		{
			_param = param;
			_size = elements;
			_dir = dir;
			_vary = vary;
			_profile = profile;
		}

		void CRSXShaderParam::setSize(u32 elements)
		{
			_size = elements;
		}
	}
}
