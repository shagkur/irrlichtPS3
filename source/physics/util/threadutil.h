/*
 * threadutil.h
 *
 *  Created on: Jan 30, 2014
 *      Author: mike
 */

#ifndef THREADUTIL_H_
#define THREADUTIL_H_

#include "base/common.h"

#include <sys/thread.h>

#define PFXTHREAD_NAME_LEN				64

#define PFXTHREAD_SIMULATION_START		0
#define PFXTHREAD_SIMULATION_END		1

typedef void (*pfxThreadEntry)(void *arg);

class PfxThread
{
public:
	void create(s32 prio, const char *name, pfxThreadEntry entry);
	void release();
	void startSimulation(void *arg);
	void waitSimulation();

	sys_event_queue_t sendQueue() { return mSendQueue; }
	sys_event_queue_t recvQueue() { return mRecvQueue; }

	sys_event_port_t sendPort() { return mSendPort; }
	sys_event_port_t recvPort() { return mRecvPort; }

private:
	char mThreadName[PFXTHREAD_NAME_LEN];
	sys_ppu_thread_t mHandlerThread;
	sys_event_queue_t mSendQueue, mRecvQueue;
	sys_event_port_t mSendPort, mRecvPort;
	pfxThreadEntry mEntryFunc;
};

#endif /* THREADUTIL_H_ */
