/*
 * threadutil.cpp
 *
 *  Created on: Jan 31, 2014
 *      Author: mike
 */

#include <sys/event_queue.h>

#include "threadutil.h"

#define PFXTHREAD_QUEUE_SIZE				32
#define PFXTHREAD_STACK_SIZE				0x10000

void PfxThread::create(s32 prio, const char *name, pfxThreadEntry entry)
{
	s32 ret;
	sys_event_queue_attr_t event_queue_attr = { SYS_EVENT_QUEUE_PRIO, SYS_EVENT_QUEUE_PPU, "" };

	mEntryFunc = entry;

	strncpy(mThreadName, name, PFXTHREAD_NAME_LEN);

	ret = sysEventPortCreate(&mSendPort, SYS_EVENT_PORT_LOCAL, SYS_EVENT_PORT_NO_NAME);
	if(ret) {
		printf("sysEventPortCreate failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventPortCreate(&mRecvPort, SYS_EVENT_PORT_LOCAL, SYS_EVENT_PORT_NO_NAME);
	if(ret) {
		printf("sysEventPortCreate failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventQueueCreate(&mSendQueue, &event_queue_attr, SYS_EVENT_QUEUE_KEY_LOCAL, PFXTHREAD_QUEUE_SIZE);
	if(ret) {
		printf("sysEventQueueCreate failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventQueueCreate(&mRecvQueue, &event_queue_attr, SYS_EVENT_QUEUE_KEY_LOCAL, PFXTHREAD_QUEUE_SIZE);
	if(ret) {
		printf("sysEventQueueCreate failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventPortConnectLocal(mSendPort, mSendQueue);
	if(ret) {
		printf("sysEventPortConnectLocal failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventPortConnectLocal(mRecvPort, mRecvQueue);
	if(ret) {
		printf("sysEventPortConnectLocal failed : ret %d\n", ret);
		abort();
	}

	ret = sysThreadCreate(&mHandlerThread, mEntryFunc, (void*)this, prio, PFXTHREAD_STACK_SIZE, THREAD_JOINABLE, mThreadName);
	if(ret) {
		printf("sysThreadCreate failed : ret %d\n", ret);
		abort();
	}
}

void PfxThread::release()
{
	s32 ret;
	u64 status;

	ret = sysEventPortSend(mSendPort, PFXTHREAD_SIMULATION_END, 0, 0);
	if(ret) {
		printf("sysEventPortSend failed : ret %d\n", ret);
		abort();
	}

	ret = sysThreadJoin(mHandlerThread, &status);
	if(ret) {
		printf("sysThreadJoin failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventPortDisconnect(mSendPort);
	if(ret) {
		printf("sysEventPortDisconnect failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventPortDisconnect(mRecvPort);
	if(ret) {
		printf("sysEventPortDisconnect failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventPortDestroy(mSendPort);
	if(ret) {
		printf("sysEventPortDestroy failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventPortDestroy(mRecvPort);
	if(ret) {
		printf("sysEventPortDestroy failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventQueueDestroy(mSendQueue, SYS_EVENT_QUEUE_FORCE_DESTROY);
	if(ret) {
		printf("sysEventQueueDestroy failed : ret %d\n", ret);
		abort();
	}

	ret = sysEventQueueDestroy(mRecvQueue, SYS_EVENT_QUEUE_FORCE_DESTROY);
	if(ret) {
		printf("sysEventQueueDestroy failed : ret %d\n", ret);
		abort();
	}
}

void PfxThread::startSimulation(void *arg)
{
	s32 ret = sysEventPortSend(mSendPort, PFXTHREAD_SIMULATION_START, (u64)arg, 0);
	if(ret) {
		printf("sysEventPortSend failed : ret %d\n", ret);
		abort();
	}
}

void PfxThread::waitSimulation()
{
	sys_event_t event;
	s32 ret = sysEventQueueReceive(mRecvQueue, &event, 0);
	if(ret) {
		printf("sysEventQueueReceive failed : ret %d\n", ret);
		abort();
	}
}
