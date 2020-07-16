/*
 * rsxshaderparambuffer.h
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#ifndef RSXSHADERPARAMBUFFER_H_
#define RSXSHADERPARAMBUFFER_H_

#include "irrtypes.h"

namespace irr
{
	namespace video
	{
		class CRSXShaderParamBuffer
		{
		public:
			CRSXShaderParamBuffer(const float *buffer, u32 elemCount);
			virtual ~CRSXShaderParamBuffer();

			bool set(const float *buffer, u32 elemCount);
			bool reset(const float *buffer, u32 elemCount);

			bool checkChangedAndUpdate(const float *buffer, u32 elemCount);

		private:
			float *_paramArray;
			u32 _elemCount;
		};
	}
}

#endif /* RSXSHADERPARAMBUFFER_H_ */
