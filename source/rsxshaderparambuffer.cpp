/*
 * rsxshaderparambuffer.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#include "rsxshaderparambuffer.h"

namespace irr
{
	namespace video
	{
		CRSXShaderParamBuffer::CRSXShaderParamBuffer(const float *buffer, u32 elemCount)
		{
			set(buffer, elemCount);
		}

		CRSXShaderParamBuffer::~CRSXShaderParamBuffer()
		{

		}

		bool CRSXShaderParamBuffer::set(const float *buffer, u32 elemCount)
		{
			_elemCount = elemCount;
			_paramArray = new float[elemCount];

			if(_paramArray == NULL)
				return false;

			memmove(_paramArray, buffer, elemCount*sizeof(float));
			return true;
		}

		bool CRSXShaderParamBuffer::reset(const float *buffer, u32 elemCount)
		{
			delete [] _paramArray;
			return set(buffer, elemCount);
		}

		bool CRSXShaderParamBuffer::checkChangedAndUpdate(const float *buffer, u32 elemCount)
		{
			if(_elemCount != elemCount) {
				reset(buffer, elemCount);
				return true;
			}

			u32 i = 0;
			bool updated = false;
			for(i=0;i < elemCount;i++) {
				if(buffer[i] != _paramArray[i]) {
					updated = true;
					break;
				}
			}

			if(updated == false)
				return false;

			while(i < elemCount) {
				_paramArray[i] = buffer[i];
				i++;
			}

			return true;
		}
	}
}

