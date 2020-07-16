/*
 * marstaskmanager.cpp
 *
 *  Created on: Jun 5, 2013
 *      Author: mike
 */

#include "taskutil/marstaskmanager.h"

class TaskQueue
{
	struct item
	{
		struct item *next;
		struct mars_task_id id;
	};

public:
	TaskQueue() : mHead(NULL), mTail(NULL)
	{
		mNodes = new struct item[NUM_TASK_ITEM];

		for(u32 i=0;i < NUM_TASK_ITEM;i++) {
			mNodes[i].next = NULL;
			if(mHead == NULL)
				mHead = mTail = &mNodes[i];
			else {
				mTail->next = &mNodes[i];
				mTail = &mNodes[i];
			}
		}
	}

	~TaskQueue()
	{

	}

	mars_task_id* pop()
	{
		if(mHead == NULL)
			return NULL;

		struct item *e = mHead;

		mHead = e->next;
		e->next = NULL;

		return (struct mars_task_id*)((u64*)e + 1);
	}

	void push(mars_task_id *id)
	{
		struct item *e = (struct item*)((u64*)id - 1);
		if(mHead == NULL)
			mHead = mTail = e;
		else {
			mTail->next = e;
			mTail = e;
		}
	}

private:
	static const u32 NUM_TASK_ITEM = 128;

	struct item *mHead;
	struct item *mTail;

	struct item *mNodes;
};

MARSTaskManager::MARSTaskManager(struct mars_context *mars, s32 numSPUs) : TaskManager()
{
	mMars = mars;
	mMaxCores = numSPUs;
	mTasks = new TaskQueue();
}

void MARSTaskManager::setMaxCores(s32 maxCores)
{
	mMaxCores = maxCores;
	mars_task_barrier_initialize(mBarrier, mMaxCores);
}

void MARSTaskManager::initialize()
{
	s32 ret;

	ret = mars_task_queue_create(mMars, &mQueuePpu2Spu, QUEUE_SIZE, QUEUE_DEPTH, MARS_TASK_QUEUE_HOST_TO_MPU);
	ASSERT(ret == MARS_SUCCESS);

	ret = mars_task_queue_create(mMars, &mQueueSpu2Ppu, QUEUE_SIZE, QUEUE_DEPTH, MARS_TASK_QUEUE_MPU_TO_HOST);
	ASSERT(ret == MARS_SUCCESS);

	ret = mars_task_barrier_create(mMars, &mBarrier, mMaxCores);
	ASSERT(ret == MARS_SUCCESS);
}

void MARSTaskManager::finalize()
{
	mars_task_barrier_destroy(mBarrier);
	mars_task_queue_destroy(mQueueSpu2Ppu);
	mars_task_queue_destroy(mQueuePpu2Spu);
}

void MARSTaskManager::startTask(s32 taskId, u32 data1, u32 data2, u32 data3, u32 data4)
{
	s32 ret;
	mars_task_args args;
	mars_task_id *task = mTasks->pop();
	ATTRIBUTE_ALIGNED16(u32 buf[8]) =
	{
		data1, data2, data3, data4, (u32)taskId, (u32)(u64)task, 0, 0
	};

	args.type.u32[0] = (u32)taskId;
	args.type.u32[1] = (u32)mBarrier;
	args.type.u32[2] = (u32)mQueuePpu2Spu;
	args.type.u32[3] = (u32)mQueueSpu2Ppu;

	ret = mars_task_create(mMars, task, "", mTaskBinary, 0);
	ASSERT(ret == MARS_SUCCESS);

	ret = mars_task_schedule(task, &args, 128);
	ASSERT(ret == MARS_SUCCESS);

	ret = mars_task_queue_push(mQueuePpu2Spu, buf);
	ASSERT(ret == MARS_SUCCESS);
}

void MARSTaskManager::waitTask(s32& taskId, u32& data1, u32& data2, u32& data3, u32& data4)
{
	s32 ret;
	mars_task_id *task;
	ATTRIBUTE_ALIGNED16(u32 buf[8]);

	ret = mars_task_queue_pop(mQueueSpu2Ppu, buf);
	ASSERT(ret == MARS_SUCCESS);

	data1 = buf[0];
	data2 = buf[1];
	data3 = buf[2];
	data4 = buf[3];
	taskId = buf[4];
	task = (mars_task_id*)(u64)buf[5];

	mars_task_wait(task, NULL);
	mars_task_destroy(task);

	mTasks->push(task);
}
