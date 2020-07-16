/*
 * fpscounter.cpp
 *
 *  Created on: Feb 23, 2013
 *      Author: mike
 */

#include "fpscounter.h"
#include "irrmath.h"

namespace irr
{
	namespace video
	{
		CFPSCounter::CFPSCounter()
		: _fps(60.0f), _primitive(0), _startTime(0), _framesCounted(0),
		  _primitivesCounted(0), _primitiveAverage(0), _primitiveTotal(0)
		{

		}

		f32 CFPSCounter::getFPS() const
		{
			return _fps;
		}

		u32 CFPSCounter::getPrimitive() const
		{
			return _primitive;
		}

		f32 CFPSCounter::getPrimitiveAverage() const
		{
			return _primitiveAverage;
		}

		u32 CFPSCounter::getPrimitiveTotal() const
		{
			return _primitiveTotal;
		}

		void CFPSCounter::registerFrame(u32 now, u32 primitivesDrawn)
		{
			++_framesCounted;

			_primitiveTotal += primitivesDrawn;
			_primitivesCounted += primitivesDrawn;
			_primitive = primitivesDrawn;

			const u32 milliseconds = now - _startTime;
			if(milliseconds >= 1000) {
				const f32 invMilli = core::reciprocalf32((f32)milliseconds);

				_fps = (f32)(1000*_framesCounted)*invMilli;
				_primitiveAverage = (f32)(1000*_primitivesCounted)*invMilli;

				_framesCounted = 0;
				_primitivesCounted = 0;
				_startTime = now;
			}
		}
	}
}
