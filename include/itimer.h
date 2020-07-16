#ifndef ITIMER_H__
#define ITIMER_H__

#include "irefcounter.h"

namespace irr
{
	class ITimer : public virtual IRefCounter
	{
	public:
		virtual ~ITimer() {};

		virtual u32 getRealTime() const = 0;
		virtual u32 getTime() const = 0;
		virtual void setTime(u32 time) = 0;
		virtual void stop() = 0;
		virtual void start() = 0;
		virtual bool isStopped() const = 0;
		virtual void tick() = 0;
	};
}

#endif
