#ifndef __IRR_OS_H__
#define __IRR_OS_H__

#include "irrtypes.h"

namespace irr
{
	namespace os
	{
		class Timer
		{
		public:
			static u32 getTime();

			static void initTimer(bool usePerformanceTimer = true);

			static void setTime(u32 time);

			static void stopTimer();

			static void startTimer();

			static void tick();

			static u32 getRealTime();

			static bool isStopped();

		private:
			static void initVirtualTimer();

			static f32 virtualTimerSpeed;
			static s32 virtualTimerStopCounter;
			static u32 startRealTime;
			static u32 lastVirtualTime;
			static u32 staticTime;
			static f64 timeFreq;
		};

		class ByteSwap
		{
		public:
			static u8 byteswap(u8 num);
			static s8 byteswap(s8 num);
			static u16 byteswap(u16 num);
			static s16 byteswap(s16 num);
			static u32 byteswap(u32 num);
			static s32 byteswap(s32 num);
			static f32 byteswap(f32 num);
		};

		class Randomizer
		{
		public:
			static void reset(s32 value = 0x0f0f0f0f);

			static s32 rand();

			static f32 frand();

			static s32 randMax();

		private:
			static s32 seed;
			static const s32 m = 2147483399;	// a non-Mersenne prime
			static const s32 a = 40692;		// another spectral success story
			static const s32 q = m/a;
			static const s32 r = m%a;		// again less than q
			static const s32 rMax = m - 1;
		};
	}
}

#endif
