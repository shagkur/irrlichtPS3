#ifndef IREFCOUNTER_H__
#define IREFCOUNTER_H__

#include "irrtypes.h"

namespace irr
{
	class IRefCounter
	{
	public:
		IRefCounter() : _refCount(1) {};
		virtual ~IRefCounter() {};

		void grab() const { _refCount++; }
		void drop() const
		{
			--_refCount;
			if(_refCount==0) delete this;
		}

		u32 getRefCount() const { return _refCount; }

	private:
		mutable u32 _refCount;
	};
}

#endif
