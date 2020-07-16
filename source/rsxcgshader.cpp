/*
 * rsxcgshader.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#include "rsxcgshader.h"
#include "rsxstate.h"
#include "rsxdriver.h"
#include "rsxshaderparam.h"

namespace irr
{
	namespace video
	{
		CRSXCgShader::CRSXCgShader(CRSXDriver *driver)
		: _driver(driver)
		{
			_state = driver->getRSXState();
			_ucode = NULL;
			_shaderProgram = NULL;
			_shaderUCode = NULL;
			_ucodeSize = 0;
		}

		CRSXCgShader::~CRSXCgShader()
		{

		}

		void CRSXCgShader::init(const void *shaderProgram, const core::stringc& name)
		{
			_shaderName = name;
			_shaderProgram = (void*)shaderProgram;

			if(_profile == ECP_VERTEX)
				init((rsxVertexProgram*) shaderProgram);
			else if(_profile == ECP_FRAGMENT)
				init((rsxFragmentProgram*) shaderProgram);
			else
				printf("Error: unknown profile type.\n");
		}

		void CRSXCgShader::init(const rsxVertexProgram *vertexProgram)
		{
			s32 param_index;
			core::stringc param_name;
			rsxProgramConst *constants;
			rsxProgramAttrib *attributes;
			CRSXShaderParam *param = NULL;
			u32 numConsts = rsxVertexProgramGetNumConst(vertexProgram);
			u32 numAttribs = rsxVertexProgramGetNumAttrib(vertexProgram);

			printf("vp init %s - numConst: %d, numAttrib: %d\n", _shaderName.c_str(), numConsts, numAttribs);

			param_name = "";
			attributes = rsxVertexProgramGetAttribs(vertexProgram);
			for(u32 i=0;i < numAttribs;i++) {
				core::stringc new_param_name;
				core::stringc param_full_name;

				if(!attributes[i].name_off) continue;

				param_full_name = ((char*)_shaderProgram) + attributes[i].name_off;
				param_index = decodeParamName(param_full_name, new_param_name);
				printf("attrib - full_name: %s, new_name: %s, index %d\n", param_full_name.c_str(), new_param_name.c_str(), param_index);
				if(!new_param_name.equals(param_name)) {
					param_name = new_param_name;
					param = new CRSXShaderParam(&attributes[i], 1, EPD_IN, EPV_VARYING, _profile, this);

					params.set(param_name, param);
				} else {
					param->setSize(param_index + 1);
				}
			}

			param_name = "";
			constants = rsxVertexProgramGetConsts(vertexProgram);
			for(u32 i=0;i < numConsts;i++) {
				core::stringc new_param_name;
				core::stringc param_full_name;

				if(!constants[i].name_off || constants[i].is_internal) continue;

				param_full_name = ((char*)_shaderProgram) + constants[i].name_off;
				param_index = decodeParamName(param_full_name, new_param_name);
				printf("const - full_name: %s, new_name: %s, index %d\n", param_full_name.c_str(), new_param_name.c_str(), param_index);
				if(!new_param_name.equals(param_name)) {
					param_name = new_param_name;
					param = new CRSXShaderParam(&constants[i], 1, EPD_IN, EPV_UNIFORM, _profile, this);

					params.set(param_name, param);
				} else {
					param->setSize(param_index + 1);
				}
			}

			rsxVertexProgramGetUCode(vertexProgram, &_ucode, &_ucodeSize);
		}

		void CRSXCgShader::init(const rsxFragmentProgram *fragmentProgram)
		{
			s32 param_index;
			core::stringc param_name;
			rsxProgramConst *constants;
			rsxProgramAttrib *attributes;
			CRSXShaderParam *param = NULL;
			u32 numConsts = rsxFragmentProgramGetNumConst(fragmentProgram);
			u32 numAttribs = rsxFragmentProgramGetNumAttrib(fragmentProgram);

			printf("fp init %s - numConst: %d, numAttrib: %d\n", _shaderName.c_str(), numConsts, numAttribs);

			param_name = "";
			attributes = rsxFragmentProgramGetAttribs(fragmentProgram);
			for(u32 i=0;i < numAttribs;i++) {
				core::stringc new_param_name;
				core::stringc param_full_name;

				if(!attributes[i].name_off) continue;

				param_full_name = ((char*)_shaderProgram) + attributes[i].name_off;
				param_index = decodeParamName(param_full_name, new_param_name);
				printf("attrib - full_name: %s, new_name: %s, index %d\n", param_full_name.c_str(), new_param_name.c_str(), param_index);
				if(!new_param_name.equals(param_name)) {
					param_name = new_param_name;
					param = new CRSXShaderParam(&attributes[i], 1, EPD_IN, EPV_VARYING, _profile, this);

					params.set(param_name, param);
				} else {
					param->setSize(param_index + 1);
				}
			}

			param_name = "";
			constants = rsxFragmentProgramGetConsts(fragmentProgram);
			for(u32 i=0;i < numConsts;i++) {
				core::stringc new_param_name;
				core::stringc param_full_name;

				if(!constants[i].name_off) continue;

				param_full_name = ((char*)_shaderProgram) + constants[i].name_off;
				param_index = decodeParamName(param_full_name, new_param_name);
				printf("const - full_name: %s, new_name: %s, index %d\n", param_full_name.c_str(), new_param_name.c_str(), param_index);
				if(!new_param_name.equals(param_name)) {
					param_name = new_param_name;
					param = new CRSXShaderParam(&constants[i], 1, EPD_IN, EPV_UNIFORM, _profile, this);

					params.set(param_name, param);
				} else {
					param->setSize(param_index + 1);
				}
			}

			rsxFragmentProgramGetUCode(fragmentProgram, &_ucode, &_ucodeSize);
		}

		s32 CRSXCgShader::decodeParamName(const core::stringc& fullName, core::stringc& returnName)
		{
			s32 paramIndex = 0;
			s32 bracketIndex = fullName.findFirst('[');

			if(bracketIndex >= 0) {
				core::stringc indexStr = fullName.subString(bracketIndex + 1, fullName.size());

				indexStr = indexStr.subString(0, indexStr.findFirst(']'));
				paramIndex = atoi(indexStr.c_str());
				returnName = fullName.subString(0, bracketIndex);

			} else
				returnName = fullName;

			return paramIndex;
		}
	}
}
