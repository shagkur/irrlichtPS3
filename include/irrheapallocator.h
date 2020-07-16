/*
 * irrheapallocator.h
 *
 *  Created on: Mar 18, 2013
 *      Author: mike
 */

#ifndef IRRHEAPALLOCATOR_H_
#define IRRHEAPALLOCATOR_H_

#include "irrtypes.h"
#include "eallocstrategy.h"
#include "iheapmanager.h"

#include <new>

namespace irr
{
	namespace core
	{
		#ifdef DEBUG_CLIENTBLOCK
		#undef DEBUG_CLIENTBLOCK
		#define DEBUG_CLIENTBLOCK new
		#endif

		template< typename T, u32 NAlignment = 64 >
		class irrheapallocator
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

		protected:
			void* operator new(size_t size)
			{
				return IHeapManager::allocate(NAlignment, size);
			}

			void operator delete(void *ptr)
			{
				IHeapManager::deallocate(ptr);
			}
		};

		#ifdef DEBUG_CLIENTBLOCK
		#undef DEBUG_CLIENTBLOCK
		#define DEBUG_CLIENTBLOCK new( _CLIENT_BLOCK, __FILE__, __LINE__)
		#endif
	}
}

#endif /* IRRHEAPALLOCATOR_H_ */
