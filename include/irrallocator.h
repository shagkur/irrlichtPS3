#ifndef __IRRALLOCATOR_H__
#define __IRRALLOCATOR_H__

#include "irrtypes.h"
#include "eallocstrategy.h"
#include <new>

namespace irr
{
	namespace core
	{
		#ifdef DEBUG_CLIENTBLOCK
		#undef DEBUG_CLIENTBLOCK
		#define DEBUG_CLIENTBLOCK new
		#endif

		template< typename T >
		class irrallocator
		{
		public:
			T* allocate(u32 cnt)
			{
				return (T*)operator new(cnt*sizeof(T));
			}

			void deallocate(T *ptr)
			{
				operator delete(ptr);
			}

			void construct(T *ptr,const T& elem)
			{
				new ((void*)ptr) T(elem);
			}

			void destruct(T *ptr)
			{
				ptr->~T();
			}
		};

		#ifdef DEBUG_CLIENTBLOCK
		#undef DEBUG_CLIENTBLOCK
		#define DEBUG_CLIENTBLOCK new( _CLIENT_BLOCK, __FILE__, __LINE__)
		#endif
	}
}

#endif
