/*
 * main.cpp
 *
 *  Created on: Jan 28, 2014
 *      Author: mike
 */

#include "base/common.h"
#include "base/heapmanager.h"
#include "base/sortcommon.h"
#include "base/safedma.h"
#include "base/prefetchiterator.h"

#include "particle/common/particleconfig.h"
#include "particle/common/particleio.h"
#include "particle/common/pclcontact.h"
#include "particle/common/pclstate.h"

#define PREFETCH_NUM					128
#define STATIC_MEM						0
#define DYNAMIC_MEM						(210*1024)
#define HEAP_BYTES						(STATIC_MEM + DYNAMIC_MEM)

#define ALLOCATE(align, size) 			gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16)
#define DEALLOCATE(ptr) 				if(ptr) {gPool.deallocate((void*)ptr); ptr = NULL;}

IOParamNormalizePcl normalizeIOPcl;
IOParamCrossPcl crossIOPcl;

u32 taskId;
u32 barrier = 0;

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool,HEAP_BYTES);

void pclNormalize()
{
	s32 numBuffers = normalizeIOPcl.numInNormalBuffers;

	// input iterator
	ReadOnlyPrefetchForwardIterator<Vector3> *itrInNormals = (ReadOnlyPrefetchForwardIterator<Vector3>*)malloc(sizeof(ReadOnlyPrefetchForwardIterator<Vector3>)*numBuffers);
	for(s32 t=0;t < numBuffers;t++){
		itrInNormals[t] = ReadOnlyPrefetchForwardIterator<Vector3>(
			&gPool,
			normalizeIOPcl.inNormalBuffers[t] + sizeof(Vector3)*(normalizeIOPcl.startNormals),
			normalizeIOPcl.inNormalBuffers[t] + sizeof(Vector3)*(normalizeIOPcl.startNormals + normalizeIOPcl.numNormals),
			PREFETCH_NUM, 12);
	}

	{
		// output iterator
		WriteOnlyPrefetchForwardIterator<Vector3> itrOutNormals(
			&gPool,
			normalizeIOPcl.outNormalsAddr + sizeof(Vector3)*(normalizeIOPcl.startNormals),
			normalizeIOPcl.outNormalsAddr + sizeof(Vector3)*(normalizeIOPcl.startNormals + normalizeIOPcl.numNormals),
			PREFETCH_NUM, 12);

		// sum and normalize
		for(u32 i=0;i < normalizeIOPcl.numNormals;i++,++itrOutNormals) {
			Vector3& normal = *itrOutNormals;

			normal = *itrInNormals[0];
			++itrInNormals[0];

			for(s32 t=1;t < numBuffers;t++){
				normal += *itrInNormals[t];
				++itrInNormals[t];
			}

			normal = normalize(normal);
		}
	}

	for(s32 i=0;i < numBuffers;i++)
		itrInNormals[i].~ReadOnlyPrefetchForwardIterator();

	free(itrInNormals);
}

void pclCross(Vector3 *vtx, Vector3 *nml)
{

	ReadOnlyPrefetchForwardIterator<uint16_t> itrIndices(
		&gPool,
		crossIOPcl.indicesAddr ,
		(crossIOPcl.indicesAddr + sizeof(u16)*(crossIOPcl.numIndices + crossIOPcl.indicesAlign) + 15)&~0x0f,
		PREFETCH_NUM, 13);

	for(u32 i=0;i < crossIOPcl.indicesAlign;i++)
		++itrIndices;

	for(u32 i=0;i < crossIOPcl.numIndices;i+=3) {

		s32 idx0 = *(itrIndices);
		++itrIndices;
		s32 idx1 = *(itrIndices);
		++itrIndices;
		s32 idx2 = *(itrIndices);
		++itrIndices;


		ASSERT(idx0 >= 0);
		ASSERT(idx1 >= 0);
		ASSERT(idx2 >= 0);
		ASSERT(idx0 < (s32)crossIOPcl.numVertices);
		ASSERT(idx1 < (s32)crossIOPcl.numVertices);
		ASSERT(idx2 < (s32)crossIOPcl.numVertices);

		Vector3 edge0 = vtx[idx1] - vtx[idx0];
		Vector3 edge1 = vtx[idx2] - vtx[idx0];
		Vector3 normal = cross(edge0, edge1);

		nml[idx0] += normal;
		nml[idx1] += normal;
		nml[idx2] += normal;
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
	{
		switch(function) {
			case PARTICLE_NORMALIZE:
				spu_dma_get(&normalizeIOPcl, addrIo, sizeof(IOParamNormalizePcl), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				pclNormalize();
				break;
			case PARTICLE_CROSS:
				spu_dma_get(&crossIOPcl, addrIo, sizeof(IOParamCrossPcl), 0, 0, 0);
				spu_dma_wait_tag_status_all(1);
				Vector3 * nml = (Vector3*)ALLOCATE(128, sizeof(Vector3)*crossIOPcl.numVertices);
				Vector3 * vtx = (Vector3*)ALLOCATE(128, sizeof(Vector3)*crossIOPcl.numVertices);

				spu_dma_large_get(vtx, crossIOPcl.verticesAddr, sizeof(Vector3)*crossIOPcl.numVertices, 0, 0, 0);

				for(u32 i=0;i < crossIOPcl.numVertices;i++)
					nml[i] = Vector3(0.0f, 0.0f, 0.0f);

				spu_dma_wait_tag_status_all(1);

				pclCross(vtx, nml);

				DEALLOCATE(vtx);

				spu_dma_large_put(nml, crossIOPcl.normalsAddr, sizeof(Vector3)*crossIOPcl.numVertices, 0, 0, 0);
				spu_dma_wait_tag_status_all(1);

				DEALLOCATE(nml);
				break;
		}
	}

	while(mars_task_queue_try_push_begin(outQueueEa, taskbuff, 0) != MARS_SUCCESS) {}
	mars_task_queue_push_end(outQueueEa, 0);

	return MARS_SUCCESS;
}
