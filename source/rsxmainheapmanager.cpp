/*
 * rsxmainheapmanager.cpp
 *
 *  Created on: Mar 18, 2013
 *      Author: mike
 */

#include "rsxmainheapmanager.h"

namespace irr
{
	CRSXMainHeapManager::CRSXMainHeapManager(void *addr, const u32 size)
	: _hostAddr(addr), _hostSize(size)
	{
		heapInit(&_heapCtrl, _hostAddr, _hostSize);
	}

	CRSXMainHeapManager::~CRSXMainHeapManager()
	{

	}

	void* CRSXMainHeapManager::allocate_(u32 alignment, u32 size)
	{
		if(alignment)
			return heapAllocateAligned(&_heapCtrl, size, alignment);
		else
			return heapAllocate(&_heapCtrl, size);
	}

	void CRSXMainHeapManager::deallocate_(void *ptr)
	{
		heapFree(&_heapCtrl, ptr);
	}

	void CRSXMainHeapManager::initialize(void *addr, const u32 size)
	{
		if(instance == NULL) {
			instance = new CRSXMainHeapManager(addr, size);
		}
	}
}

