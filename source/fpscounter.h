/*
 * fpscounter.h
 *
 *  Created on: Feb 23, 2013
 *      Author: mike
 */

#ifndef FPSCOUNTER_H_
#define FPSCOUNTER_H_

#include "irrtypes.h"

namespace irr
{
	namespace video
	{
		class CFPSCounter
		{
		public:
			CFPSCounter();

			f32 getFPS() const;

			u32 getPrimitive() const;

			f32 getPrimitiveAverage() const;

			u32 getPrimitiveTotal() const;

			void registerFrame(u32 now, u32 primitivesDrawn);

		private:
			f32 _fps;
			u32 _primitive;
			u32 _startTime;

			u32 _framesCounted;
			u32 _primitivesCounted;
			f32 _primitiveAverage;
			u32 _primitiveTotal;
		};
	}
}

#endif /* FPSCOUNTER_H_ */
