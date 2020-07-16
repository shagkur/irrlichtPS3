#ifndef TIMER_H__
#define TIMER_H__

#include "itimer.h"
#include "os.h"

namespace irr
{
	class CTimer : public ITimer
	{
	public:
		CTimer()
		{
			os::Timer::initTimer();
		}

		virtual u32 getRealTime() const
		{
			return os::Timer::getRealTime();
		}

		virtual u32 getTime() const
		{
			return os::Timer::getTime();
		}

		virtual void setTime(u32 time)
		{
			os::Timer::setTime(time);
		}

		virtual void stop()
		{
			os::Timer::stopTimer();
		}

		virtual void start()
		{
			os::Timer::startTimer();
		}

		virtual bool isStopped() const
		{
			bool ret = os::Timer::isStopped();
			return ret;
		}

		virtual void tick()
		{
			os::Timer::tick();
		}
	};
}

#endif
