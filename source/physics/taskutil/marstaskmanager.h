/*
 * marstaskmanager.h
 *
 *  Created on: Jun 5, 2013
 *      Author: mike
 */

#ifndef MARSTASKMANAGER_H_
#define MARSTASKMANAGER_H_

#include "taskutil/taskmanager.h"

#include "irrarray.h"

using namespace irr;

class TaskQueue;

class MARSTaskManager : public TaskManager
{
public:
	MARSTaskManager(struct mars_context *mars, s32 numSPUs);

	virtual void initialize();
	virtual void finalize();

	virtual void setTaskEntry(const void *entry) {mTaskBinary = entry;}

	virtual void startTask(s32 taskId, u32 data1, u32 data2, u32 data3, u32 data4);
	virtual void waitTask(s32& taskId, u32& data1, u32& data2, u32& data3, u32& data4);

	virtual void setMaxCores(s32 maxCores);

	void setMaxSPUs(s32 numSPUs) { setMaxCores(numSPUs); }

private:
	static const u32 QUEUE_SIZE = 32;
	static const u32 QUEUE_DEPTH = 4;

	const void *mTaskBinary;

	struct mars_context *mMars;

	TaskQueue *mTasks;

	ATTRIBUTE_ALIGNED128(u64 mBarrier);

	ATTRIBUTE_ALIGNED128(u64 mQueuePpu2Spu);
	ATTRIBUTE_ALIGNED128(u64 mQueueSpu2Ppu);
};

#endif /* MARSTASKMANAGER_H_ */
