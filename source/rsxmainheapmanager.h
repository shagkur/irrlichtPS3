/*
 * rsxmainheapmanager.h
 *
 *  Created on: Mar 18, 2013
 *      Author: mike
 */

#ifndef RSXMAINHEAPMANAGER_H_
#define RSXMAINHEAPMANAGER_H_

#include <sys/heap.h>

#include "iheapmanager.h"

namespace irr
{
	class CRSXMainHeapManager : public IHeapManager
	{
		CRSXMainHeapManager(void *addr, const u32 size);
		virtual ~CRSXMainHeapManager();

	public:
		static void initialize(void *add, const u32 size);

	protected:
		virtual void* allocate_(u32 alignment, u32 size);
		virtual void deallocate_(void *ptr);

	private:
		void *_hostAddr;
		u32 _hostSize;

		heap_cntrl _heapCtrl;
	};
}

#endif /* RSXMAINHEAPMANAGER_H_ */
