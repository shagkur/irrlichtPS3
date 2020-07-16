#include "os.h"

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <sys/systime.h>

namespace irr
{
	namespace os
	{
		f32 Timer::virtualTimerSpeed = 1.0f;
		s32 Timer::virtualTimerStopCounter = 0;
		u32 Timer::lastVirtualTime = 0;
		u32 Timer::startRealTime = 0;
		u32 Timer::staticTime = 0;
		f64 Timer::timeFreq = 0.0;

		void Timer::initTimer(bool usePerformanceTimer)
		{
			initVirtualTimer();
		}

		u32 Timer::getTime()
		{
			if(isStopped()) return lastVirtualTime;
			return lastVirtualTime + (u32)((staticTime - startRealTime)*virtualTimerSpeed);
		}

		void Timer::tick()
		{
			staticTime = getRealTime();
		}

		void Timer::startTimer()
		{
			++virtualTimerStopCounter;
			if(isStopped()) {
				setTime(lastVirtualTime);
			}
		}

		void Timer::stopTimer()
		{
			if(!isStopped()) lastVirtualTime = getTime();
			--virtualTimerStopCounter;
		}

		void Timer::setTime(u32 time)
		{
			staticTime = getRealTime();
			lastVirtualTime = time;
			startRealTime = staticTime;
		}

		bool Timer::isStopped()
		{
			return (virtualTimerStopCounter!=0);
		}

		u32 Timer::getRealTime()
		{
			u64 timebase = __gettime();
			return (u32)(((f64)timebase/timeFreq)*1000.0);
		}

		void Timer::initVirtualTimer()
		{
			timeFreq = (f64)sysGetTimebaseFrequency();
			staticTime = getRealTime();
			startRealTime = getRealTime();
		}

		u8 ByteSwap::byteswap(u8 num) { return num; }
		s8 ByteSwap::byteswap(s8 num) { return num; }
		u16 ByteSwap::byteswap(u16 num) { return bswap16(num); }
		s16 ByteSwap::byteswap(s16 num) { return bswap16(num); }
		u32 ByteSwap::byteswap(u32 num) { return bswap32(num); }
		s32 ByteSwap::byteswap(s32 num) { return bswap32(num); }
		f32 ByteSwap::byteswap(f32 num)
		{
			union ieee32 {
				f32 f;
				u32 i;
			};
			ieee32 v;
			char* const tmp = (char*)&num;
			v.i = __lwbrx(tmp);
			return v.f;
		}

		s32 Randomizer::seed = 0x0f0f0f0f;

		void Randomizer::reset(s32 value)
		{
			seed = value;
		}

		s32 Randomizer::rand()
		{
			seed = a*(seed%q) - r*(seed/q);
			if (seed < 0)
				seed += m;

			return seed;
		}

		f32 Randomizer::frand()
		{
			return rand()*(1.0f/rMax);
		}

		s32 Randomizer::randMax()
		{
			return rMax;
		}
	}
}
