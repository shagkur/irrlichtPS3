/*
 * rsxshaderconstant.h
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#ifndef RSXSHADERCONSTANT_H_
#define RSXSHADERCONSTANT_H_

#include "rsxcgshader.h"

namespace irr
{
	namespace video
	{
		class CRSXShaderConstant
		{
		public:
			CRSXShaderConstant() {}
			CRSXShaderConstant(const CRSXShaderConstant& other);
			CRSXShaderConstant(const float *buffer, u32 elemCount);
			CRSXShaderConstant(const core::matrix4 *m);
			virtual ~CRSXShaderConstant();

			CRSXShaderConstant& operator =(const CRSXShaderConstant& other);

			virtual void addToCmdBuffer();

			void init(void *param, const float *buffer, u32 elemCount, bool isUsed, bool isMatrix, CRSXCgShader *shader);

			void setParam(CRSXShaderParam *param, CRSXCgShader *shader);

			bool isPartial() const { return _isPartial; }

			E_CG_PROFILE getProfile() const { return _profile; }

		protected:
			float *_buffer;
			u32 _elemCount;
			void *_param;
			E_CG_PROFILE _profile;
			CRSXCgShader *_shader;

			bool _isUsed;
			bool _isMatrix;
			bool _isPartial;
		};
	}
}

#endif /* RSXSHADERCONSTANT_H_ */
