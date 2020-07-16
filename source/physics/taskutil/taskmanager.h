/*
 * taskmanager.h
 *
 *  Created on: Jun 5, 2013
 *      Author: mike
 */

#ifndef TASKMANAGER_H_
#define TASKMANAGER_H_

#include "base/common.h"
#include "base/heapmanager.h"

#define TM_MAX_TASKS			6
#define TM_IOBUFF_BYTES			1048576

class TaskManager
{
protected:
	s32 mMaxCores;
	HeapManager mPool;
	ATTRIBUTE_ALIGNED128(u8 mIoBuff[TM_IOBUFF_BYTES]);

public:
	TaskManager() : mPool(mIoBuff, TM_IOBUFF_BYTES) {}
	virtual ~TaskManager() {}

	void* allocate(size_t bytes) { return mPool.allocate(bytes); }
	void deallocate(void *ptr) { mPool.deallocate(ptr); }

	virtual s32 getMaxCores() const { return mMaxCores; }
	virtual void setMaxCores(s32 maxCores) { mMaxCores = maxCores; }

	virtual void initialize() = 0;
	virtual void finalize() = 0;

	virtual void setTaskEntry(const void *entry) = 0;

	virtual void startTask(s32 taskId, u32 data1, u32 data2, u32 data3, u32 data4) = 0;
	virtual void waitTask(s32& taskId, u32& data1, u32& data2, u32& data3, u32& data4) = 0;
};

#endif /* TASKMANAGER_H_ */
