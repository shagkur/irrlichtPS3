/*
 * rsxshaderparam.h
 *
 *  Created on: Feb 5, 2014
 *      Author: mike
 */

#ifndef RSXSHADERPARAM_H_
#define RSXSHADERPARAM_H_

#include "ecgprofiles.h"

namespace irr
{
	namespace video
	{
		enum E_PARAM_DIR
		{
			EPD_IN,
			EPD_INOUT,
			EPD_OUT
		};

		enum E_PARAM_VARY
		{
			EPV_VARYING,
			EPV_UNIFORM,
			EPV_CONSTANT
		};

		class CRSXCgShader;

		class CRSXShaderParam
		{
		public:
			CRSXShaderParam(void *param, u32 elements, E_PARAM_DIR dir, E_PARAM_VARY vary, E_CG_PROFILE profile, CRSXCgShader *shader);
			virtual ~CRSXShaderParam();

			void setSize(u32 elements);
			void setParam(void *param, u32 elements, E_PARAM_DIR dir, E_PARAM_VARY vary, E_CG_PROFILE profile);

			void* getParam() const { return _param; }

			u32 getSize() const { return _size; }

			E_PARAM_DIR getDirection() const { return _dir; }

			E_PARAM_VARY getVariability() const { return _vary; }

			E_CG_PROFILE getProfile() const { return _profile; }

		private:
			E_PARAM_DIR _dir;
			E_PARAM_VARY _vary;
			E_CG_PROFILE _profile;
			void *_param;
			u32 _size;

			CRSXCgShader *_shader;
		};
	}
}
#endif /* RSXSHADERPARAM_H_ */
