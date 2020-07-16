/*
 * main.cpp
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/prefetchiterator.h"
#include "base/heapmanager.h"
#include "base/safedma.h"

#include "softbody/common/softbodyconfig.h"
#include "softbody/common/softbodyio.h"
#include "softbody/common/softcontact.h"
#include "softbody/common/softstate.h"
#include "softbody/common/closestsoftbody.h"

#include "rigidbody/common/collobject.h"
#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/contact.h"

#define SPE_CACHE_NSETS				16
#define SPE_CACHELINE_SIZE			128
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
IOParamDetectContactsSoft detectContactsIO;

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

void detectContacts()
{
	SoftState *sftStates = (SoftState*)ALLOCATE(16, sizeof(SoftState)*softbodyGroupIO.curProperty.numParticles);
	spu_dma_large_get(sftStates, softbodyGroupIO.sftStatesAddr, sizeof(SoftState)*softbodyGroupIO.curProperty.numParticles, 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	{
		u32 numContacts = 0;

		PrefetchForwardIterator<SoftContactPair> itrPair(
			&gPool,
			softbodyGroupIO.contactPairsPclPclAddr,
			softbodyGroupIO.contactPairsPclPclAddr + sizeof(SoftContactPair)*softbodyGroupIO.numContactsPclPcl,
			PREFETCH_NUM>>1, 10);

		ReadOnlyPrefetchForwardIterator<SortData> itrSort(
			&gPool,
			softbodyGroupIO.contactSortsPclPclAddr,
			softbodyGroupIO.contactSortsPclPclAddr + sizeof(SortData)*softbodyGroupIO.numContactsPclPcl,
			PREFETCH_NUM, 11);

		for(u32 i=0;i < softbodyGroupIO.numContactsPclPcl;i++, ++itrSort) {
			SoftState& stateA = sftStates[getStateA(*itrSort)];
			SoftState& stateB = sftStates[getStateB(*itrSort)];

			SoftContactPair curPair;
			curPair.reset();

			closestContactPcl(curPair, stateA.getPosition(), softbodyGroupIO.curProperty.particleRadius, stateB.getPosition(), softbodyGroupIO.curProperty.particleRadius, CONTACT_EPSILON);

			if(curPair.numContacts > 0) {
				curPair.stateIndex[0] = getStateA(*itrSort);
				curPair.stateIndex[1] = getStateB(*itrSort);
				*itrPair = curPair;
				++itrPair;
				numContacts++;
			}
		}

		softbodyGroupIO.numContactsPclPcl = numContacts;
	}

	{
		u32 numContacts = 0;

		PrefetchForwardIterator<SoftContactPair> itrPair(
			&gPool,
			softbodyGroupIO.contactPairsPclRigAddr,
			softbodyGroupIO.contactPairsPclRigAddr + sizeof(SoftContactPair)*softbodyGroupIO.numContactsPclRig,
			PREFETCH_NUM>>1, 10);

		ReadOnlyPrefetchForwardIterator<SortData> itrSort(
			&gPool,
			softbodyGroupIO.contactSortsPclRigAddr,
			softbodyGroupIO.contactSortsPclRigAddr + sizeof(SortData)*softbodyGroupIO.numContactsPclRig,
			PREFETCH_NUM, 11);

		for(u32 i=0;i < softbodyGroupIO.numContactsPclRig;i++, ++itrSort) {
			TrbState stateB;
			CollObject collB;
			SoftState& stateA = sftStates[getStateA(*itrSort)];

			spu_cache_read((void*)&stateB, detectContactsIO.rigStatesAddr + sizeof(TrbState)*getStateB(*itrSort), sizeof(TrbState));
			spu_cache_read((void*)&collB, detectContactsIO.rigCollsAddr + sizeof(CollObject)*getBodyB(*itrSort), sizeof(CollObject));

			Transform3 tB(stateB.getOrientation(), stateB.getPosition());

			SoftContactPair curPair;
			curPair.reset();

			closestContactRig(curPair, stateA.getPosition(), softbodyGroupIO.curProperty.particleRadius, collB, tB, CONTACT_EPSILON);

			if(curPair.numContacts > 0) {
				curPair.stateIndex[0] = getStateA(*itrSort);
				curPair.stateIndex[1] = getStateB(*itrSort);
				*itrPair = curPair;
				++itrPair;
				numContacts++;
			}
		}

		softbodyGroupIO.numContactsPclRig = numContacts;
	}

	DEALLOCATE(sftStates);
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
		case SOFTBODY_DETECTCONTACTS:
		{
			s32 groupDmaTag = 8;

			spu_dma_get(&detectContactsIO, addrIo, sizeof(IOParamDetectContactsSoft), 0, 0, 0);
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
					spu_dma_wait_tag_status_all(1<<groupDmaTag);
					spu_dma_get(&softbodyGroupIO, groupAddr + sizeof(IOParamSoftBodyGroups)*nextGroup, sizeof(IOParamSoftBodyGroups), groupDmaTag, 0, 0);
					commonBuff[2]++;
				} else
					empty = true;

				unlock(commonBuff, commonBuffAddr);

				if(!empty) {
					spu_dma_wait_tag_status_all(1<<groupDmaTag);
					detectContacts();
					spu_dma_put(&softbodyGroupIO, groupAddr + sizeof(IOParamSoftBodyGroups)*nextGroup, sizeof(IOParamSoftBodyGroups), groupDmaTag, 0, 0);
				}
			}
			spu_dma_wait_tag_status_all(1<<groupDmaTag);

			DEALLOCATE(commonBuff);
		}
		break;
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}

