/*
 * rsxfragmentshader.h
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#ifndef RSXFRAGMENTSHADER_H_
#define RSXFRAGMENTSHADER_H_

#include "rsxcgshader.h"

namespace irr
{
	namespace video
	{
		class CRSXFragmentShader : public CRSXCgShader
		{
		public:
			CRSXFragmentShader(CRSXDriver *driver);
			virtual ~CRSXFragmentShader();

			virtual void setParameter(void *param, const float *data, u32 size);
			virtual void setParameter(void *param, const core::matrix4 *matrix);

			virtual void init(const void *shaderProgram, const core::stringc& name);

			void addToCmdBuffer();

		private:
			bool checkParamUpdated(void *param, const float *buffer, u32 size);

			u32 _shaderOffset;

			bool _updateFlag;
		};
	}
}

#endif /* RSXFRAGMENTSHADER_H_ */
