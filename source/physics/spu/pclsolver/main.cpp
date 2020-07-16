/*
 * main.cpp
 *
 *  Created on: Jan 27, 2014
 *      Author: mike
 */

#include "base/common.h"
#include "base/heapmanager.h"
#include "base/prefetchiterator.h"
#include "base/cache.h"

#include "particle/common/particleconfig.h"
#include "particle/common/particleio.h"
#include "particle/common/pclcontact.h"
#include "particle/common/pclstate.h"
#include "particle/common/particlesolver.h"

#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/contact.h"
#include "rigidbody/common/parallelgroup.h"

#define PREFETCH_NUM					64
#define STATIC_MEM						0
#define DYNAMIC_MEM						(64*1024)
#define HEAP_BYTES						(STATIC_MEM + DYNAMIC_MEM)

#define ALLOCATE(align, size) 			gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16)
#define DEALLOCATE(ptr) 				if(ptr) {gPool.deallocate((void*)ptr); ptr = NULL;}

IOParamSolverPcl solverIOPcl;
IOParamPostResponsePcl postResponseIOPcl;

SolverInfo *contactSolverInfoPcl;
SolverInfo *contactSolverInfoRig;
SolverInfo *jointSolverInfoPcl;
SolverInfo *jointSolverInfoRig;

u32 taskId;
u32 barrier = 0;

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool,HEAP_BYTES);

void sync()
{
	while(mars_task_barrier_try_notify(barrier) != MARS_SUCCESS) {}
	while(mars_task_barrier_try_wait(barrier) != MARS_SUCCESS) {}
}

#include "contactphase.h"
#include "jointphase.h"

///////////////////////////////////////////////////////////////////////////////
// postResponse

#ifdef PARTICLE_VELOCITY_BASE

void postResponse()
{
	PrefetchForwardIterator<PclState> itrStates(
		&gPool,
		postResponseIOPcl.statesAddr,
		postResponseIOPcl.statesAddr + sizeof(PclState)*postResponseIOPcl.numStates,
		PREFETCH_NUM, 10);

	for(u32 i=0;i < postResponseIOPcl.numStates;i++,++itrStates) {
		PclState& particle = *itrStates;
		if(length(particle.fV) > postResponseIOPcl.maxLinearVelocity)
			particle.fV = normalize(particle.fV)*postResponseIOPcl.maxLinearVelocity;
	}
}

#endif

int mars_task_main(const struct mars_task_args *task_args)
{
	u32 inQueueEa = task_args->type.u32[2];
	u32 outQueueEa = task_args->type.u32[3];

	barrier = task_args->type.u32[1];

	ATTRIBUTE_ALIGNED16(u32 taskbuff[8]);
	while(mars_task_queue_try_pop_begin(inQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_pop_end(inQueueEa, 0);

	taskId = taskbuff[4];

	u32 addrIo = taskbuff[0];
	u32 function = taskbuff[1];
	u32 commonBuffAddr = taskbuff[2];
	{
		switch(function) {
			case PARTICLE_SOLVE_CONSTRAINT_EX:
			{
				contactSolverInfoPcl = (SolverInfo*)ALLOCATE(16, sizeof(SolverInfo));
				contactSolverInfoRig = (SolverInfo*)ALLOCATE(16, sizeof(SolverInfo));
				jointSolverInfoPcl = (SolverInfo*)ALLOCATE(16, sizeof(SolverInfo));
				jointSolverInfoRig = (SolverInfo*)ALLOCATE(16, sizeof(SolverInfo));

				spu_dma_get(&solverIOPcl, addrIo, sizeof(IOParamSolverPcl), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				spu_dma_get(contactSolverInfoPcl, solverIOPcl.contactSolverInfoPclAddr, sizeof(SolverInfo), 0, 0, 0);
				spu_dma_get(contactSolverInfoRig, solverIOPcl.contactSolverInfoRigAddr, sizeof(SolverInfo), 0, 0, 0);
				spu_dma_get(jointSolverInfoPcl, solverIOPcl.jointSolverInfoPclAddr, sizeof(SolverInfo), 0, 0, 0);
				spu_dma_get(jointSolverInfoRig, solverIOPcl.jointSolverInfoRigAddr, sizeof(SolverInfo), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);
#ifdef PARTICLE_VELOCITY_BASE
				preContactImpulsePhaseEx();
				preJointImpulsePhaseEx();
#endif
				// solver
				u32 iteration = MAX(solverIOPcl.numCollIteration, solverIOPcl.numJointIteration);
				for(u32 i=0;i < iteration;i++) {
					if(LIKELY((iteration-i) <= solverIOPcl.numCollIteration))
						applyContactImpulsePhaseEx(commonBuffAddr);

					if(LIKELY((iteration-i) <= solverIOPcl.numJointIteration))
						applyJointImpulsePhaseEx();
				}

				spu_dma_put(&solverIOPcl, addrIo, sizeof(IOParamSolverPcl), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				DEALLOCATE(jointSolverInfoRig);
				DEALLOCATE(jointSolverInfoPcl);
				DEALLOCATE(contactSolverInfoRig);
				DEALLOCATE(contactSolverInfoPcl);
			}
			break;

			// ---------------------------------------------------------------------------------------------------------------------------
			case PARTICLE_POSTRESPONSE:
				spu_dma_get(&postResponseIOPcl, addrIo, sizeof(IOParamPostResponsePcl), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);
#ifdef PARTICLE_VELOCITY_BASE
				postResponse();
#endif
				spu_dma_put(&postResponseIOPcl, addrIo, sizeof(IOParamPostResponsePcl), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);
				break;
		}
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}
