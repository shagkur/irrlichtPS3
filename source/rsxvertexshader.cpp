/*
 * rsxvertexshader.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#include "rsxvertexshader.h"
#include "rsxdriver.h"

namespace irr
{
	namespace video
	{
		CRSXVertexShader::CRSXVertexShader(CRSXDriver *driver)
		: CRSXCgShader(driver)
		{
			_profile = ECP_VERTEX;
		}

		CRSXVertexShader::~CRSXVertexShader()
		{

		}

		void CRSXVertexShader::setParameter(void *param, const float *data, u32 size)
		{
			rsxSetVertexProgramParameter(_driver->getGcmContext(), (rsxVertexProgram*) _shaderProgram, (rsxProgramConst*) param, data);
		}

		void CRSXVertexShader::setParameter(void *param, const core::matrix4 *matrix)
		{
			core::matrix4 m = transpose(*matrix);
			rsxSetVertexProgramParameter(_driver->getGcmContext(), (rsxVertexProgram*) _shaderProgram, (rsxProgramConst*) param, (const float*)&m);
		}

		void CRSXVertexShader::init(const void *shaderProgram, const core::stringc& name)
		{
			CRSXCgShader::init(shaderProgram, name);

			_shaderUCode = _ucode;
		}

		void CRSXVertexShader::addToCmdBuffer()
		{
			if(_state->state.vertex_shader_program != _shaderProgram ||
			   _state->state.vertex_shader_program_ucode != _shaderUCode)
			{
				rsxLoadVertexProgram(_driver->getGcmContext(), (rsxVertexProgram*) _shaderProgram, _shaderUCode);

				_state->state.vertex_shader_program = (rsxVertexProgram*) _shaderProgram;
				_state->state.vertex_shader_program_ucode = _shaderUCode;
			}
		}
	}
}


