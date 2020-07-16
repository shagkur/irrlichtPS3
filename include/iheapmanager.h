/*
 * iheapmanager.h
 *
 *  Created on: Mar 18, 2013
 *      Author: mike
 */

#ifndef IHEAPMANAGER_H_
#define IHEAPMANAGER_H_

#include "irrtypes.h"

#include <malloc.h>

namespace irr
{
	class IHeapManager
	{
	public:
		static void* allocate(u32 size)
		{
			if(instance != NULL) {
				return instance->allocate_(0, size);
			}
			return malloc(size);
		}

		static void* allocate(u32 alignment, u32 size)
		{
			if(instance != NULL) {
				return instance->allocate_(alignment, size);
			}
			return memalign(alignment, size);
		}

		static void deallocate(void *ptr)
		{
			if(instance != NULL) {
				instance->deallocate_(ptr);
				return;
			}
			free(ptr);
		}

	protected:
		virtual void* allocate_(u32 alignement, u32 size) = 0;
		virtual void deallocate_(void *ptr) = 0;

		static IHeapManager *instance;
	};
}

#endif /* IHEAPMANAGER_H_ */
