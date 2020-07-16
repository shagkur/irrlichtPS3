/*
 * rsxvertexshader.h
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#ifndef RSXVERTEXSHADER_H_
#define RSXVERTEXSHADER_H_

#include "rsxcgshader.h"

namespace irr
{
	namespace video
	{
		class CRSXVertexShader : public CRSXCgShader
		{
		public:
			CRSXVertexShader(CRSXDriver *driver);
			virtual ~CRSXVertexShader();

			virtual void setParameter(void *param, const float *data, u32 size);
			virtual void setParameter(void *param, const core::matrix4 *matrix);

			virtual void init(const void *shaderProgram, const core::stringc& name);

			void addToCmdBuffer();
		};
	}
}

#endif /* RSXVERTEXSHADER_H_ */
