/*
 * ishaderconstantsetcallback.h
 *
 *  Created on: Feb 2, 2013
 *      Author: mike
 */

#ifndef ISHADERCONSTANTSETCALLBACK_H_
#define ISHADERCONSTANTSETCALLBACK_H_

#include "irefcounter.h"

namespace irr
{
	namespace video
	{
		class SMaterial;

		class IShaderConstantSetCallback : public virtual IRefCounter
		{
		public:
			virtual void onSetMaterial(const SMaterial& mat) {}
			virtual void onSetConstants(IMaterialRendererServices *services, s32 userData) = 0;
		};
	}
}

#endif /* ISHADERCONSTANTSETCALLBACK_H_ */
