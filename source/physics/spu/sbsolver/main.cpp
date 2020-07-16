/*
 * main.cpp
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/prefetchiterator.h"
#include "base/heapmanager.h"

#include "softbody/common/softbodyconfig.h"
#include "softbody/common/softbodyio.h"
#include "softbody/common/softcontact.h"
#include "softbody/common/softstate.h"
#include "softbody/common/softbodysolver.h"

#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/contact.h"
#include "rigidbody/common/parallelgroup.h"

#define SPE_CACHE_NSETS				16
#define SPE_CACHELINE_SIZE			16
#define SPE_CACHE_SET_TAGID(set)	30

#include "swcache/spe_cache.h"

#define STATIC_MEM					0
#define DYNAMIC_MEM					(64*1024)
#define HEAP_BYTES 					(STATIC_MEM + DYNAMIC_MEM)

#define PREFETCH_NUM				64

#define ALLOCATE(align,size) 		gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16);
#define DEALLOCATE(ptr) 			if(ptr) {gPool.deallocate(((void*)ptr)); ptr = NULL;}

ATTRIBUTE_ALIGNED128(u32 lockBuffer[32]);

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool, HEAP_BYTES);

IOParamSoftBodyGroups softbodyGroupIO;
IOParamSolveConstraintsSoft solverConstraintIO;
IOParamPushBodiesSoft pushBodiesIO;

u32 taskId;
u32 barrier = 0;

SIMD_FORCE_INLINE void* __spe_cache_read_line(u64 ea)
{
	s32 set, idx, line, byte;

	_spe_cache_nway_lookup_(ea, set, idx);
	if(UNLIKELY(idx < 0)) {
		idx = _spe_cache_miss_(ea, set, -1);
		spu_writech(22, SPE_CACHE_SET_TAGMASK(set));
		spu_mfcstat(MFC_TAG_UPDATE_ALL);
	}

	line = _spe_cacheline_num_(set, idx);
	byte = _spe_cacheline_byte_offset_(ea);
	return (void*)&spe_cache_mem[line + byte];
}

static void spu_cache_read(void *dest, u64 ea, u32 size)
{
	u8 *ptr = (u8*)dest;

	while(size > 0) {
		u32 copy = size > SPE_CACHELINE_SIZE ? SPE_CACHELINE_SIZE : size;
		void *data = __spe_cache_read_line(ea);

		memcpy(ptr, data, copy);
		size -= copy;
		ptr += copy;
		ea += copy;
	}
}

void sync()
{
	while(mars_task_barrier_try_notify(barrier) != MARS_SUCCESS) {}
	while(mars_task_barrier_try_wait(barrier) != MARS_SUCCESS) {}
}

void lock(u32 *lsBuff, u32 eaBuff)
{
	s32 ch = 1;
	do {
		spu_dma_getllar(lsBuff, (u64)eaBuff, 0, 0);
		spu_dma_wait_atomic_status();
		spu_dsync();

		if(UNLIKELY(lsBuff[0] == 1))
			continue;

		lsBuff[0] = 1; // LOCKED

		spu_dsync();
		spu_dma_putllc(lsBuff, (u64)eaBuff, 0, 0);
		ch = spu_dma_wait_atomic_status();
	} while(UNLIKELY(ch != 0));
}

void unlock(u32 *lsBuff, u32 eaBuff)
{
	s32 ch = 1;
	do {
		spu_dma_getllar(lockBuffer, (u64)eaBuff, 0, 0);
		spu_dma_wait_atomic_status();
		spu_dsync();

		lsBuff[0] = 0; // UNLOCKED

		spu_dsync();
		spu_dma_putllc(lsBuff, (u64)eaBuff, 0, 0);
		ch = spu_dma_wait_atomic_status();
	} while(UNLIKELY(ch != 0));
}

void applyImpulse(SoftState *sftStates)
{
	SoftContactSolverConfig config;
	config.timeStep = solverConstraintIO.timeStep;
	config.particleRadius = softbodyGroupIO.curProperty.particleRadius;
	config.separateBias = softbodyGroupIO.curProperty.separateBias;

	{
		PrefetchForwardIterator<SoftContactPair> itrPair(
			&gPool,
			softbodyGroupIO.contactPairsPclPclAddr,
			softbodyGroupIO.contactPairsPclPclAddr + sizeof(SoftContactPair)*softbodyGroupIO.numContactsPclPcl,
			PREFETCH_NUM>>1, 10);

		for(u32 i=0;i < softbodyGroupIO.numContactsPclPcl;i++, ++itrPair) {
			SoftContactPair& contactPair = *itrPair;
			ASSERT(contactPair.stateIndex[0] < softbodyGroupIO.curProperty.numParticles);
			ASSERT(contactPair.stateIndex[1] < softbodyGroupIO.curProperty.numParticles);

			SoftState& particleA = sftStates[contactPair.stateIndex[0]];
			SoftState& particleB = sftStates[contactPair.stateIndex[1]];
			applyImpulsePcl(contactPair, particleA, particleB, config);
		}
	}

	{
		PrefetchForwardIterator<SoftContactPair> itrPair(
			&gPool,
			softbodyGroupIO.contactPairsPclRigAddr,
			softbodyGroupIO.contactPairsPclRigAddr + sizeof(SoftContactPair)*softbodyGroupIO.numContactsPclRig,
			PREFETCH_NUM>>1, 10);

		if(softbodyGroupIO.curProperty.twoWayInteraction) {
			for(u32 i=0;i < softbodyGroupIO.numContactsPclRig;i++, ++itrPair) {
				SoftContactPair& contactPair = *itrPair;
				ASSERT(contactPair.stateIndex[0] < softbodyGroupIO.curProperty.numParticles);
				ASSERT(contactPair.stateIndex[1] < solverConstraintIO.numRigStates);

				SoftState& particleA = sftStates[contactPair.stateIndex[0]];

				SoftPushImpulse totalPushImp;
				spu_dma_get(&totalPushImp, softbodyGroupIO.pushImpulseAddr + sizeof(SoftPushImpulse)*contactPair.stateIndex[1], sizeof(SoftPushImpulse), 0, 0, 0);

				TrbState stateB;
				TrbDynBody bodyB;

				spu_cache_read((void*)&stateB, solverConstraintIO.rigStatesAddr + sizeof(TrbState)*contactPair.stateIndex[1], sizeof(TrbState));
				spu_cache_read((void*)&bodyB, solverConstraintIO.rigBodiesAddr + sizeof(TrbDynBody)*stateB.trbBodyIdx, sizeof(TrbDynBody));

				spu_dma_wait_tag_status_all(1);

				Vector3 forceImp(0.0f);
				Vector3 rotImp(0.0f);

				applyImpulseRigTwoWay(contactPair, particleA, stateB, config, bodyB, forceImp, rotImp);

				totalPushImp.linearImpulse += forceImp;
				totalPushImp.angularImpulse += rotImp;
				spu_dma_put(&totalPushImp, softbodyGroupIO.pushImpulseAddr + sizeof(SoftPushImpulse)*contactPair.stateIndex[1], sizeof(SoftPushImpulse), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);
			}
		} else {
			for(u32 i=0;i < softbodyGroupIO.numContactsPclRig;i++, ++itrPair) {
				SoftContactPair& contactPair = *itrPair;
				ASSERT(contactPair.stateIndex[0] < softbodyGroupIO.curProperty.numParticles);
				ASSERT(contactPair.stateIndex[1] < solverConstraintIO.numRigStates);

				SoftState& particleA = sftStates[contactPair.stateIndex[0]];

				TrbState stateB;
				TrbDynBody bodyB;

				spu_cache_read((void*)&stateB, solverConstraintIO.rigStatesAddr + sizeof(TrbState)*contactPair.stateIndex[1], sizeof(TrbState));
				spu_cache_read((void*)&bodyB, solverConstraintIO.rigBodiesAddr + sizeof(TrbDynBody)*stateB.trbBodyIdx, sizeof(TrbDynBody));

				applyImpulseRig(contactPair, particleA, stateB, config, bodyB);
			}
		}
	}
}

void applyJoint(SoftState *sftStates)
{
	SoftJointSolverConfig config;
	config.timeStep = solverConstraintIO.timeStep;

	PrefetchForwardIterator<SoftJoint> itrJoint(
		&gPool,
		softbodyGroupIO.sftJointsAddr,
		softbodyGroupIO.sftJointsAddr + sizeof(SoftJoint)*softbodyGroupIO.curProperty.numJoints,
		PREFETCH_NUM>>1, 10);

	for(u32 i=0;i < softbodyGroupIO.curProperty.numJoints;i++, ++itrJoint) {
		SoftJoint& joint = *itrJoint;

		if(joint.type == SoftJointTypePcl) {
			ASSERT(joint.stateIndexA < softbodyGroupIO.curProperty.numParticles);
			ASSERT(joint.stateIndexB < softbodyGroupIO.curProperty.numParticles);

			SoftState& particleA = sftStates[joint.stateIndexA];
			SoftState& particleB = sftStates[joint.stateIndexB];
			if(joint.active && particleA.isActive() && particleB.isActive())
				applyJointPcl(joint, particleA, particleB, config);
		} else {
			ASSERT(joint.stateIndexA < softbodyGroupIO.curProperty.numParticles);
			ASSERT(joint.stateIndexB < solverConstraintIO.numRigStates);

			TrbState stateB;
			SoftState& particleA = sftStates[joint.stateIndexA];

			spu_cache_read((void*)&stateB, solverConstraintIO.rigStatesAddr + sizeof(TrbState)*joint.stateIndexB, sizeof(TrbState));

			if(joint.active && particleA.isActive())
				applyJointRig(joint, particleA, stateB, config);
		}
	}
}

void solveConstraints()
{
	SoftState *sftStates = (SoftState*)ALLOCATE(16, sizeof(SoftState)*softbodyGroupIO.curProperty.numParticles);
	spu_dma_large_get(sftStates, softbodyGroupIO.sftStatesAddr, sizeof(SoftState)*softbodyGroupIO.curProperty.numParticles, 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	s32 contactIteration = softbodyGroupIO.curProperty.contactIteration;
	s32 jointIteration = softbodyGroupIO.curProperty.jointIteration;
	s32 iteration = MAX(contactIteration, jointIteration);

	for(s32 i=0;i < iteration;i++) {
		if((iteration - i) <= contactIteration)
			applyImpulse(sftStates);
		if((iteration - i) <= jointIteration)
			applyJoint(sftStates);
	}

	// Sum up accumulation
	for(u32 i=0;i < softbodyGroupIO.curProperty.numParticles;i++) {
		sftStates[i].fX += sftStates[i].accumulation;
		sftStates[i].accumulation = Vector3(0.0f);
	}

	spu_dma_large_put(sftStates, softbodyGroupIO.sftStatesAddr, sizeof(SoftState)*softbodyGroupIO.curProperty.numParticles, 0, 0, 0);
	spu_dma_wait_tag_status_all(1);
	DEALLOCATE(sftStates);
}

void pushBodies()
{
	u32 total_pushImpulseAddr = 0;

	// Gather total impulse
	bool first = true;
	for(u32 i=0;i < pushBodiesIO.numGroups;i++) {
		spu_dma_get(&softbodyGroupIO, pushBodiesIO.groupAddr + sizeof(IOParamSoftBodyGroups)*i, sizeof(IOParamSoftBodyGroups), 0, 0, 0);
		spu_dma_wait_tag_status_all(1);

		if(!softbodyGroupIO.curProperty.twoWayInteraction) continue;
		if(first) {
			total_pushImpulseAddr = softbodyGroupIO.pushImpulseAddr;
			first = false;
			continue;
		}

		{
			PrefetchForwardIterator<SoftPushImpulse> itrTotalPushImp(
				&gPool,
				total_pushImpulseAddr + sizeof(SoftPushImpulse)*pushBodiesIO.rigStartState,
				total_pushImpulseAddr + sizeof(SoftPushImpulse)*(pushBodiesIO.rigStartState + pushBodiesIO.numRigStates),
				32, 11);

			PrefetchForwardIterator<SoftPushImpulse> itrPushImp(
				&gPool,
				softbodyGroupIO.pushImpulseAddr + sizeof(SoftPushImpulse)*pushBodiesIO.rigStartState,
				softbodyGroupIO.pushImpulseAddr + sizeof(SoftPushImpulse)*(pushBodiesIO.rigStartState + pushBodiesIO.numRigStates),
				48, 13);

			for(u32 j=0;j < pushBodiesIO.numRigStates;j++, ++itrTotalPushImp, ++itrPushImp) {
				SoftPushImpulse& push = *itrTotalPushImp;
				push.linearImpulse += (*itrPushImp).linearImpulse;
				push.angularImpulse += (*itrPushImp).angularImpulse;
				(*itrPushImp).linearImpulse = Vector3(0.0f);
				(*itrPushImp).angularImpulse = Vector3(0.0f);
			}
		}
	}

	// Apply impulse
	if(total_pushImpulseAddr) {
		PrefetchForwardIterator<TrbState> itrState(
			&gPool,
			pushBodiesIO.rigStatesAddr,
			pushBodiesIO.rigStatesAddr + sizeof(TrbState)*pushBodiesIO.numRigStates,
			32, 10);

		PrefetchForwardIterator<SoftPushImpulse> itrTotalPushImp(
			&gPool,
			total_pushImpulseAddr + sizeof(SoftPushImpulse)*pushBodiesIO.rigStartState,
			total_pushImpulseAddr + sizeof(SoftPushImpulse)*(pushBodiesIO.rigStartState + pushBodiesIO.numRigStates),
			48, 11);

		for(u32 j=0;j < pushBodiesIO.numRigStates;j++, ++itrState, ++itrTotalPushImp) {
			TrbState& state = *itrState;
			state.setLinearVelocity(state.getLinearVelocity() + (*itrTotalPushImp).linearImpulse);
			state.setAngularVelocity(state.getAngularVelocity() + (*itrTotalPushImp).angularImpulse);
			(*itrTotalPushImp).linearImpulse = Vector3(0.0f);
			(*itrTotalPushImp).angularImpulse = Vector3(0.0f);
		}
	}
}

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

	switch(function) {
		case SOFTBODY_SOLVE_CONSTRAINTS:
		{
			s32 groupDmaTag = 8;

			spu_dma_get(&solverConstraintIO, addrIo, sizeof(IOParamSolveConstraintsSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			u32 commonBuffAddr = taskbuff[2];
			u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
			bool empty = false;
			while(!empty) {
				lock(commonBuff, commonBuffAddr);

				u32 groupAddr = commonBuff[1];
				u32 nextGroup = commonBuff[2];
				u32 maxGroup  = commonBuff[3];

				if(nextGroup < maxGroup) {
					spu_dma_get(&softbodyGroupIO, groupAddr + sizeof(IOParamSoftBodyGroups)*nextGroup, sizeof(IOParamSoftBodyGroups), groupDmaTag, 0, 0);
					commonBuff[2]++;
				} else
					empty = true;

				unlock(commonBuff, commonBuffAddr);

				if(!empty) {
					spu_dma_wait_tag_status_all(1<<groupDmaTag);
					solveConstraints();
				}
			}
			spu_dma_wait_tag_status_all(1<<groupDmaTag);

			DEALLOCATE(commonBuff);
		}
		break;

		case SOFTBODY_PUSH_BODIES:
		{
			spu_dma_get(&pushBodiesIO, addrIo, sizeof(IOParamPushBodiesSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			pushBodies();
		}
		break;
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}
