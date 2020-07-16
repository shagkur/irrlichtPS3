/*
 * rsxshaderconstant.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#include "rsxshaderconstant.h"
#include "rsxshaderparam.h"

namespace irr
{
	namespace video
	{
		CRSXShaderConstant::CRSXShaderConstant(const CRSXShaderConstant& other)
		{
			*this = other;
		}

		CRSXShaderConstant::CRSXShaderConstant(const float *buffer, u32 elemCount)
		{
			init(NULL, buffer, elemCount, false, false, NULL);
		}

		CRSXShaderConstant::CRSXShaderConstant(const core::matrix4 *m)
		{
			init(NULL, (float*)m, 1, false, true, NULL);
		}

		CRSXShaderConstant::~CRSXShaderConstant()
		{

		}

		CRSXShaderConstant& CRSXShaderConstant::operator =(const CRSXShaderConstant& other)
		{
			_param = other._param;
			_buffer = other._buffer;
			_elemCount = other._elemCount;
			_isUsed = other._isUsed;
			_isMatrix = other._isMatrix;
			_isPartial = other._isPartial;
			_shader = other._shader;
			_profile = other._profile;

			return *this;
		}

		void CRSXShaderConstant::init(void *param, const float *buffer, u32 elemCount, bool isUsed, bool isMatrix, CRSXCgShader *shader)
		{
			_param = param;
			_buffer = (float*)buffer;

			if(isMatrix)
				_elemCount = elemCount*16;
			else
				_elemCount = elemCount;

			_isUsed = isUsed;
			_isMatrix = isMatrix;
			_isPartial = true;

			_shader = shader;
		}

		void CRSXShaderConstant::setParam(CRSXShaderParam *param, CRSXCgShader *shader)
		{
			_param = param->getParam();
			_profile = param->getProfile();
			_shader = shader;
			_isUsed = true;
		}

		void CRSXShaderConstant::addToCmdBuffer()
		{
			if(!_isUsed) return;

			if(_isMatrix)
				_shader->setParameter(_param, (core::matrix4*)_buffer);
			else
				_shader->setParameter(_param, _buffer, _elemCount);
		}
	}
}
