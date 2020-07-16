/*
 * rsxfragmentshader.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#include "rsxfragmentshader.h"
#include "rsxdriver.h"

namespace irr
{
	namespace video
	{
		CRSXFragmentShader::CRSXFragmentShader(CRSXDriver *driver)
		: CRSXCgShader(driver), _shaderOffset(0xDEADBEEF), _updateFlag(false)
		{
			_profile = ECP_FRAGMENT;
		}

		CRSXFragmentShader::~CRSXFragmentShader()
		{

		}

		void CRSXFragmentShader::setParameter(void *param, const float *data, u32 size)
		{
			if(!checkParamUpdated(param, data, size))
				return;

			rsxSetFragmentProgramParameter(_driver->getGcmContext(), (rsxFragmentProgram*) _shaderProgram, (rsxProgramConst*) param, data, _shaderOffset, GCM_LOCATION_RSX);
			_updateFlag = true;
		}

		void CRSXFragmentShader::setParameter(void* param, const core::matrix4 *matrix)
		{
			core::matrix4 m = transpose(*matrix);

			if(!checkParamUpdated(param, (const float*)&m, 16))
				return;

			rsxSetFragmentProgramParameter(_driver->getGcmContext(), (rsxFragmentProgram*) _shaderProgram, (rsxProgramConst*) param, (const float*)&m, _shaderOffset, GCM_LOCATION_RSX);
			_updateFlag = true;
		}

		void CRSXFragmentShader::init(const void *shaderProgram, const core::stringc& name)
		{
			CRSXCgShader::init(shaderProgram, name);

			_shaderUCode = rsxMemalign(64, _ucodeSize);
			if(_shaderUCode == NULL)
				return;

			memcpy(_shaderUCode, _ucode, _ucodeSize);
			rsxAddressToOffset(_shaderUCode, &_shaderOffset);
		}

		void CRSXFragmentShader::addToCmdBuffer()
		{
			if(_state->state.fragment_shader_program != _shaderProgram ||
			   _state->state.fragment_shader_offset != _shaderOffset)
			{
				rsxLoadFragmentProgramLocation(_driver->getGcmContext(), (rsxFragmentProgram*) _shaderProgram, _shaderOffset, GCM_LOCATION_RSX);

				_state->state.fragment_shader_program = (rsxFragmentProgram*) _shaderProgram;
				_state->state.fragment_shader_offset = _shaderOffset;

				_updateFlag = false;
			} else if(_updateFlag) {
				rsxUpdateFragmentProgramLocation(_driver->getGcmContext(), _shaderOffset, GCM_LOCATION_RSX);
				_updateFlag = false;
			}
		}

		bool CRSXFragmentShader::checkParamUpdated(void *param, const float *buffer, u32 size)
		{
			core::map<void*, CRSXShaderParamBuffer*>::Node *n = _paramMap.find(param);

			if(n == NULL) {
				_paramMap.insert(param, new CRSXShaderParamBuffer(buffer, size));
				return true;
			}

			return n->getValue()->checkChangedAndUpdate(buffer, size);
		}
	}
}

