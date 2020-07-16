/*
 * main.cpp
 *
 *  Created on: Jun 12, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/safedma.h"
#include "base/sortcommon.h"
#include "base/heapmanager.h"
#include "base/prefetchiterator.h"

#include "softbody/common/softbodyconfig.h"
#include "softbody/common/softbodyio.h"
#include "softbody/common/softcontact.h"
#include "softbody/common/softstate.h"

#define STATIC_MEM				0
#define DYNAMIC_MEM				(96*1024)
#define HEAP_BYTES 				(STATIC_MEM + DYNAMIC_MEM)

#define PREFETCH_NUM			128

#define ALLOCATE(align,size) 	gPool.allocate((size), align == 128 ? HeapManager::ALIGN128 : HeapManager::ALIGN16);
#define DEALLOCATE(ptr) 		if(ptr) {gPool.deallocate(((void*)ptr)); ptr = NULL;}

ATTRIBUTE_ALIGNED128(u32 lockBuffer[32]);

ATTRIBUTE_ALIGNED128(u8 memPool[HEAP_BYTES]);
HeapManager gPool(memPool, HEAP_BYTES);

// IO
IOParamSoftBodyGroups softbodyGroupIO;
IOParamBuildMeshSoft buildMeshIO;


// -------------------------------------------------------
// SoftBodies Collision

ATTRIBUTE_ALIGNED16(struct) sftsftCollisionSetup
{
	u32 numVertices;
	u32 numIndices;

	Vector3 *sftVertices;
	u16 *sftIndices;

	Vector3 *pclPressureLocal; // Local Pressure
	Vector3 *pclPressureGlobal; // Total Pressure
	u16 *pclPressureCount; // Counting Pressure

	Vector3 pclbbMin;
	Vector3 pclbbMax;

	bool sftsftCollisionEnable;
	f32 pressureConst;
};

sftsftCollisionSetup sftCollisionSetupIO[2];

u32 taskId;
u32 barrier = 0;

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

ATTRIBUTE_ALIGNED16(struct) Idx24
{
	vec_ushort8 id[3];
};

void buildMesh(u32 groupAddr, u32 nextGroup)
{
	(void) groupAddr;

	u32 numVertices = softbodyGroupIO.numVertices;
	u32 numIndices = softbodyGroupIO.numIndices;
	f32 intVolume = softbodyGroupIO.curProperty.volume;
	f32 pressureConst = softbodyGroupIO.curProperty.pressureConst;
	s32 face_index = 0;

	if(numVertices == 0) return;

	ASSERT(numVertices <= softbodyGroupIO.curProperty.numParticles);

	Vector3 *sftVtx = (Vector3*)ALLOCATE(16, sizeof(Vector3)*numVertices);
	Vector3 *sftNml = (Vector3*)ALLOCATE(16, sizeof(Vector3)*numVertices);
	Vector3 *sftPress = (Vector3*)ALLOCATE(16, sizeof(Vector3)*numIndices/3);

	spu_dma_large_get(sftPress, softbodyGroupIO.pclPressureLocalAddr, sizeof(Vector3)*numIndices/3, 0, 0, 0);

	{
		PrefetchForwardIterator<SoftState> itrState(
			&gPool,
			softbodyGroupIO.sftStatesAddr,
			softbodyGroupIO.sftStatesAddr + sizeof(SoftState)*softbodyGroupIO.curProperty.numParticles,
			PREFETCH_NUM, 10);

		for(u32 i=0;i < numVertices;i++, ++itrState) {
			sftVtx[i] = (*itrState).getPosition();
			sftNml[i] = Vector3(0.0f);
		}
	}

	// ---------------------------
	// SoftBodies Collision Setup
	Vector3 bbMin = Vector3(FLT_MAX);
	Vector3 bbMax = Vector3(-FLT_MAX);
	for(u32 i=0;i < numVertices;i++) {
		bbMin = minPerElem(bbMin, sftVtx[i]);
		bbMax = maxPerElem(bbMax, sftVtx[i]);
	}

	// if volume is defined
	if(intVolume > 0.01) {
		u32 numIdx = ((softbodyGroupIO.numIndices + 23)/24)*24;
		PrefetchForwardIterator<Idx24> itrIdx(
			&gPool,
			softbodyGroupIO.indicesAddr,
			softbodyGroupIO.indicesAddr + sizeof(u16)*numIdx,
			PREFETCH_NUM, 10);

		u32 i = 0;
		for(;i < softbodyGroupIO.numIndices - 24;i+=24, ++itrIdx) {
			Idx24 idx24 = *itrIdx;

			Vector3 v[24], normal[8];
			v[0]  = sftVtx[spu_extract(idx24.id[0], 0)];
			v[1]  = sftVtx[spu_extract(idx24.id[0], 1)];
			v[2]  = sftVtx[spu_extract(idx24.id[0], 2)];
			v[3]  = sftVtx[spu_extract(idx24.id[0], 3)];
			v[4]  = sftVtx[spu_extract(idx24.id[0], 4)];
			v[5]  = sftVtx[spu_extract(idx24.id[0], 5)];
			v[6]  = sftVtx[spu_extract(idx24.id[0], 6)];
			v[7]  = sftVtx[spu_extract(idx24.id[0], 7)];
			v[8]  = sftVtx[spu_extract(idx24.id[1], 0)];
			v[9]  = sftVtx[spu_extract(idx24.id[1], 1)];
			v[10] = sftVtx[spu_extract(idx24.id[1], 2)];
			v[11] = sftVtx[spu_extract(idx24.id[1], 3)];
			v[12] = sftVtx[spu_extract(idx24.id[1], 4)];
			v[13] = sftVtx[spu_extract(idx24.id[1], 5)];
			v[14] = sftVtx[spu_extract(idx24.id[1], 6)];
			v[15] = sftVtx[spu_extract(idx24.id[1], 7)];
			v[16] = sftVtx[spu_extract(idx24.id[2], 0)];
			v[17] = sftVtx[spu_extract(idx24.id[2], 1)];
			v[18] = sftVtx[spu_extract(idx24.id[2], 2)];
			v[19] = sftVtx[spu_extract(idx24.id[2], 3)];
			v[20] = sftVtx[spu_extract(idx24.id[2], 4)];
			v[21] = sftVtx[spu_extract(idx24.id[2], 5)];
			v[22] = sftVtx[spu_extract(idx24.id[2], 6)];
			v[23] = sftVtx[spu_extract(idx24.id[2], 7)];

			normal[0] = cross(v[1] - v[0], v[2] - v[0]);
			normal[1] = cross(v[4] - v[3], v[5] - v[3]);
			normal[2] = cross(v[7] - v[6], v[8] - v[6]);
			normal[3] = cross(v[10] - v[9], v[11] - v[9]);
			normal[4] = cross(v[13] - v[12], v[14] - v[12]);
			normal[5] = cross(v[16] - v[15], v[17] - v[15]);
			normal[6] = cross(v[19] - v[18], v[20] - v[18]);
			normal[7] = cross(v[22] - v[21], v[23] - v[21]);


			// Calculate face pressure
			for(u32 k=0;k < 8;k++, face_index++) {
				Vector3 edge0 = v[3*k + 1] - v[3*k + 0];
				Vector3 edge1 = v[3*k + 2] - v[3*k + 0];
				f32 t0 = (f32)dot(edge0, edge0);
				f32 t1 = (f32)dot(edge1, edge1);
				f32 t2 = (f32)dot(edge0, edge1);
				f32 area = 0.5*sqrtf(t0*t1 - t2*t2);
				Vector3 reactiveForce = pressureConst*area*(1.0f/intVolume)*normalize(normal[k]);
				sftPress[face_index] = reactiveForce;
			}

			sftNml[spu_extract(idx24.id[0], 0)] += normal[0];
			sftNml[spu_extract(idx24.id[0], 1)] += normal[0];
			sftNml[spu_extract(idx24.id[0], 2)] += normal[0];
			sftNml[spu_extract(idx24.id[0], 3)] += normal[1];
			sftNml[spu_extract(idx24.id[0], 4)] += normal[1];
			sftNml[spu_extract(idx24.id[0], 5)] += normal[1];
			sftNml[spu_extract(idx24.id[0], 6)] += normal[2];
			sftNml[spu_extract(idx24.id[0], 7)] += normal[2];
			sftNml[spu_extract(idx24.id[1], 0)] += normal[2];
			sftNml[spu_extract(idx24.id[1], 1)] += normal[3];
			sftNml[spu_extract(idx24.id[1], 2)] += normal[3];
			sftNml[spu_extract(idx24.id[1], 3)] += normal[3];
			sftNml[spu_extract(idx24.id[1], 4)] += normal[4];
			sftNml[spu_extract(idx24.id[1], 5)] += normal[4];
			sftNml[spu_extract(idx24.id[1], 6)] += normal[4];
			sftNml[spu_extract(idx24.id[1], 7)] += normal[5];
			sftNml[spu_extract(idx24.id[2], 0)] += normal[5];
			sftNml[spu_extract(idx24.id[2], 1)] += normal[5];
			sftNml[spu_extract(idx24.id[2], 2)] += normal[6];
			sftNml[spu_extract(idx24.id[2], 3)] += normal[6];
			sftNml[spu_extract(idx24.id[2], 4)] += normal[6];
			sftNml[spu_extract(idx24.id[2], 5)] += normal[7];
			sftNml[spu_extract(idx24.id[2], 6)] += normal[7];
			sftNml[spu_extract(idx24.id[2], 7)] += normal[7];
		}

		{
			Idx24 idx24 = *itrIdx;
			for(u32 j=0;i < softbodyGroupIO.numIndices;i+=3, j+=3, face_index++) {
				u16 idx0 = spu_extract(idx24.id[(j + 0)>>3], (i + 0)&0x07);
				u16 idx1 = spu_extract(idx24.id[(j + 1)>>3], (i + 1)&0x07);
				u16 idx2 = spu_extract(idx24.id[(j + 2)>>3], (i + 2)&0x07);

				Vector3 edge0 = sftVtx[idx1] - sftVtx[idx0];
				Vector3 edge1 = sftVtx[idx2] - sftVtx[idx0];
				Vector3 normal = cross(edge0, edge1);

				sftNml[idx0] += normal;
				sftNml[idx1] += normal;
				sftNml[idx2] += normal;

				// calculate pressure value when it has volume

				f32 t0 = (f32)dot(edge0, edge0);
				f32 t1 = (f32)dot(edge1, edge1);
				f32 t2 = (f32)dot(edge0, edge1);
				f32 area = 0.5*sqrtf(t0*t1 - t2*t2);
				Vector3 reactiveForce = pressureConst*area*(1.0f/intVolume)*normalize(normal);
				sftPress[face_index] = reactiveForce;
			}
		}
	} else {
		// if volume is not defined
		u32 numIdx = ((softbodyGroupIO.numIndices + 23)/24)*24;
		PrefetchForwardIterator<Idx24> itrIdx(
			&gPool,
			softbodyGroupIO.indicesAddr,
			softbodyGroupIO.indicesAddr + sizeof(u16)*numIdx,
			PREFETCH_NUM, 10);

		u32 i = 0;
		for(;i < softbodyGroupIO.numIndices - 24;i+=24, ++itrIdx) {
			Idx24 idx24 = *itrIdx;

			Vector3 v[24], normal[8];
			v[0]  = sftVtx[spu_extract(idx24.id[0], 0)];
			v[1]  = sftVtx[spu_extract(idx24.id[0], 1)];
			v[2]  = sftVtx[spu_extract(idx24.id[0], 2)];
			v[3]  = sftVtx[spu_extract(idx24.id[0], 3)];
			v[4]  = sftVtx[spu_extract(idx24.id[0], 4)];
			v[5]  = sftVtx[spu_extract(idx24.id[0], 5)];
			v[6]  = sftVtx[spu_extract(idx24.id[0], 6)];
			v[7]  = sftVtx[spu_extract(idx24.id[0], 7)];
			v[8]  = sftVtx[spu_extract(idx24.id[1], 0)];
			v[9]  = sftVtx[spu_extract(idx24.id[1], 1)];
			v[10] = sftVtx[spu_extract(idx24.id[1], 2)];
			v[11] = sftVtx[spu_extract(idx24.id[1], 3)];
			v[12] = sftVtx[spu_extract(idx24.id[1], 4)];
			v[13] = sftVtx[spu_extract(idx24.id[1], 5)];
			v[14] = sftVtx[spu_extract(idx24.id[1], 6)];
			v[15] = sftVtx[spu_extract(idx24.id[1], 7)];
			v[16] = sftVtx[spu_extract(idx24.id[2], 0)];
			v[17] = sftVtx[spu_extract(idx24.id[2], 1)];
			v[18] = sftVtx[spu_extract(idx24.id[2], 2)];
			v[19] = sftVtx[spu_extract(idx24.id[2], 3)];
			v[20] = sftVtx[spu_extract(idx24.id[2], 4)];
			v[21] = sftVtx[spu_extract(idx24.id[2], 5)];
			v[22] = sftVtx[spu_extract(idx24.id[2], 6)];
			v[23] = sftVtx[spu_extract(idx24.id[2], 7)];

			normal[0] = cross(v[1] - v[0], v[2] - v[0]);
			normal[1] = cross(v[4] - v[3], v[5] - v[3]);
			normal[2] = cross(v[7] - v[6], v[8] - v[6]);
			normal[3] = cross(v[10] - v[9], v[11] - v[9]);
			normal[4] = cross(v[13] - v[12], v[14] - v[12]);
			normal[5] = cross(v[16] - v[15], v[17] - v[15]);
			normal[6] = cross(v[19] - v[18], v[20] - v[18]);
			normal[7] = cross(v[22] - v[21], v[23] - v[21]);

			sftNml[spu_extract(idx24.id[0], 0)] += normal[0];
			sftNml[spu_extract(idx24.id[0], 1)] += normal[0];
			sftNml[spu_extract(idx24.id[0], 2)] += normal[0];
			sftNml[spu_extract(idx24.id[0], 3)] += normal[1];
			sftNml[spu_extract(idx24.id[0], 4)] += normal[1];
			sftNml[spu_extract(idx24.id[0], 5)] += normal[1];
			sftNml[spu_extract(idx24.id[0], 6)] += normal[2];
			sftNml[spu_extract(idx24.id[0], 7)] += normal[2];
			sftNml[spu_extract(idx24.id[1], 0)] += normal[2];
			sftNml[spu_extract(idx24.id[1], 1)] += normal[3];
			sftNml[spu_extract(idx24.id[1], 2)] += normal[3];
			sftNml[spu_extract(idx24.id[1], 3)] += normal[3];
			sftNml[spu_extract(idx24.id[1], 4)] += normal[4];
			sftNml[spu_extract(idx24.id[1], 5)] += normal[4];
			sftNml[spu_extract(idx24.id[1], 6)] += normal[4];
			sftNml[spu_extract(idx24.id[1], 7)] += normal[5];
			sftNml[spu_extract(idx24.id[2], 0)] += normal[5];
			sftNml[spu_extract(idx24.id[2], 1)] += normal[5];
			sftNml[spu_extract(idx24.id[2], 2)] += normal[6];
			sftNml[spu_extract(idx24.id[2], 3)] += normal[6];
			sftNml[spu_extract(idx24.id[2], 4)] += normal[6];
			sftNml[spu_extract(idx24.id[2], 5)] += normal[7];
			sftNml[spu_extract(idx24.id[2], 6)] += normal[7];
			sftNml[spu_extract(idx24.id[2], 7)] += normal[7];
		}

		{
			Idx24 idx24 = *itrIdx;
			for(u32 j=0;i < softbodyGroupIO.numIndices;i+=3, j+=3, face_index++) {
				u16 idx0 = spu_extract(idx24.id[(j + 0)>>3], (i + 0)&0x07);
				u16 idx1 = spu_extract(idx24.id[(j + 1)>>3], (i + 1)&0x07);
				u16 idx2 = spu_extract(idx24.id[(j + 2)>>3], (i + 2)&0x07);

				Vector3 edge0 = sftVtx[idx1] - sftVtx[idx0];
				Vector3 edge1 = sftVtx[idx2] - sftVtx[idx0];
				Vector3 normal = cross(edge0, edge1);

				sftNml[idx0] += normal;
				sftNml[idx1] += normal;
				sftNml[idx2] += normal;

			}
		}
	}

	for(u32 i=0;i < numVertices;i++)
		sftNml[i] = normalize(sftNml[i]);

	sftCollisionSetupIO[0].pclbbMin = bbMin;
	sftCollisionSetupIO[0].pclbbMax = bbMax;
	spu_dma_large_put(&sftCollisionSetupIO[0], buildMeshIO.ioParamAddr + sizeof(sftsftCollisionSetup)*nextGroup, sizeof(sftsftCollisionSetup), 0, 0, 0);

	spu_dma_large_put(sftPress,softbodyGroupIO.pclPressureLocalAddr,sizeof(Vector3)*numIndices/3,0,0,0);
	spu_dma_large_put(sftPress,softbodyGroupIO.pclPressureGlobalAddr,sizeof(Vector3)*numIndices/3,0,0,0);
	spu_dma_large_put(sftVtx,softbodyGroupIO.verticesAddr,sizeof(Vector3)*numVertices,0,0,0);
	spu_dma_large_put(sftNml,softbodyGroupIO.normalsAddr,sizeof(Vector3)*numVertices,0,0,0);
	spu_dma_wait_tag_status_all(1);

	DEALLOCATE(sftPress);
	DEALLOCATE(sftNml);
	DEALLOCATE(sftVtx);

}

bool TrgTrgCollision(Vector3 vtx[][3], f32& cdepth, f32& cfraction)
{
	Vector3 normal[2];
	normal[0] = normalize(cross(vtx[0][1] - vtx[0][0], vtx[0][2] - vtx[0][0]));
	normal[1] = normalize(cross(vtx[1][1] - vtx[1][0], vtx[1][2] - vtx[1][0]));

	cfraction = 0.20f;

	// triangle intersection test
	for(u32 k=0;k < 3;k++) {

		Vector3 u = vtx[1][k] - vtx[0][0];
		Vector3 v = vtx[1][(k + 1)%3] - vtx[0][0];

		if(dot(u, normal[0])*dot(v, normal[0]) < 0.0f)
		{
			cdepth = dot((vtx[0][0] - vtx[1][k]), normal[0]);
			cdepth = cdepth/dot((vtx[1][(k + 1)%3] - vtx[1][k]), normal[0]);

			if(cdepth < 0.0f) continue;
			else if(cdepth > 1.0f) continue;

			Vector3 hitPos = vtx[1][k] + cdepth*(vtx[1][(k + 1)%3] - vtx[1][k]);

			Vector3 target;
			target = cross(vtx[0][0] - hitPos, vtx[0][1] - hitPos);
			if(dot(target, normal[0]) < 0.0f) continue;

			target = cross(vtx[0][1] - hitPos, vtx[0][2] - hitPos);
			if(dot(target, normal[0]) < 0.0f) continue;

			target = cross(vtx[0][2] - hitPos, vtx[0][0] - hitPos);
			if(dot(target, normal[0]) < 0.0f) continue;

			// contact point was found
			cfraction = 0.50f;
			return true;
		}
	}

	// triangle intersection test
	for(u32 k=0;k < 3;k++) {

		Vector3 u = vtx[0][k] - vtx[1][0];
		Vector3 v = vtx[0][(k + 1)%3] - vtx[1][0];

		if(dot(u, normal[1])*dot(v, normal[1]) < 0.0f)
		{
			cdepth = dot((vtx[1][0] - vtx[0][k]), normal[1]);
			cdepth = cdepth/dot((vtx[0][(k + 1)%3] - vtx[0][k]), normal[1]);

			if(cdepth < 0.0f) continue;
			else if(cdepth > 1.0f) continue;

			Vector3 hitPos = vtx[0][k] + cdepth*(vtx[0][(k + 1)%3] - vtx[0][k]);

			Vector3 target;
			target = cross(vtx[1][0] - hitPos, vtx[1][1] - hitPos);
			if(dot(target, normal[1]) < 0.0f) continue;

			target = cross(vtx[1][1] - hitPos, vtx[1][2] - hitPos);
			if(dot(target, normal[1]) < 0.0f) continue;

			target = cross(vtx[1][2] - hitPos, vtx[1][0] - hitPos);
			if(dot(target, normal[1]) < 0.0f) continue;

			// contact point was found
			cfraction = 0.50f;
			return true;
		}
	}

	// any contact points is not detected
	// but very near position
	// should need to add pressure force
	return true;
}

void sftsftCollisionDetect(sftsftCollisionSetup& ioParam0, sftsftCollisionSetup& ioParam1)
{
	u32 numOfParticle[2];
	u32 numOfIndex[2];
	Vector3 *pclVtxColl[2];
	u16 *pclIdxColl[2];
	Vector3 *pclPressureLocalAddr[2];
	Vector3 *pclPressureGlobalAddr[2];
	u16 *pclPressureCountAddr[2];
	Vector3 bbMin[2], bbMax[2];
	f32 pressureConst[2];

	{
		numOfParticle[0] = ioParam0.numVertices;
		numOfIndex[0] = ioParam0.numIndices;
		pclVtxColl[0] = ioParam0.sftVertices;
		pclIdxColl[0] = ioParam0.sftIndices;
		pclPressureLocalAddr[0] = ioParam0.pclPressureLocal;
		pclPressureGlobalAddr[0] = ioParam0.pclPressureGlobal;
		pclPressureCountAddr[0] = ioParam0.pclPressureCount;
		bbMin[0] = ioParam0.pclbbMin;
		bbMax[0] = ioParam0.pclbbMax;
		pressureConst[0] = ioParam0.pressureConst;

		numOfParticle[1] = ioParam1.numVertices;
		numOfIndex[1] = ioParam1.numIndices;
		pclVtxColl[1] = ioParam1.sftVertices;
		pclIdxColl[1] = ioParam1.sftIndices;
		pclPressureLocalAddr[1] = ioParam1.pclPressureLocal;
		pclPressureGlobalAddr[1] = ioParam1.pclPressureGlobal;
		pclPressureCountAddr[1] = ioParam1.pclPressureCount;
		bbMin[1] = ioParam1.pclbbMin;
		bbMax[1] = ioParam1.pclbbMax;
		pressureConst[1] = ioParam1.pressureConst;
	}

	// Bounding Box of SoftBody vs Bounding Box of SoftBody
	if(maxElem(bbMin[0] - bbMax[1]) > 0.0f) return;
	if(maxElem(bbMin[1] - bbMax[0]) > 0.0f) return;

	s32 groupDmaTag = 9;
	Vector3 *sftVtx[2];
	u16 *sftIdx[2];
	bool *cachedflag[2];
	u32 *commonBuff;

	commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
	cachedflag[0] = (bool*)ALLOCATE(16, sizeof(bool)*numOfIndex[0]/3);
	cachedflag[1] = (bool*)ALLOCATE(16, sizeof(bool)*numOfIndex[1]/3);
	sftVtx[0] = (Vector3*)ALLOCATE(16, sizeof(Vector3)*numOfParticle[0]);
	sftVtx[1] = (Vector3*)ALLOCATE(16, sizeof(Vector3)*numOfParticle[1]);
	//sftIdx[0] = (u16*)ALLOCATE(16, sizeof(u16)*numOfIndex[0]);
	sftIdx[1] = (u16*)ALLOCATE(16, sizeof(u16)*numOfIndex[1]);

	spu_dma_large_get(sftVtx[0], (u32)pclVtxColl[0], sizeof(Vector3)*numOfParticle[0], groupDmaTag, 0, 0);
	spu_dma_large_get(sftVtx[1], (u32)pclVtxColl[1], sizeof(Vector3)*numOfParticle[1], groupDmaTag, 0, 0);
	spu_dma_large_get(sftIdx[1], (u32)pclIdxColl[1], sizeof(u16)*numOfIndex[1], groupDmaTag, 0, 0);


	//
	// Calculate Overlap Region of Two Bounding Box
	//
	Vector3 overlapMin, overlapMax;
	overlapMin = maxPerElem(bbMin[0], bbMin[1]);
	overlapMax = minPerElem(bbMax[0], bbMax[1]);

	// Creating Cache Data
	// Overlap Region vs Triangle Check
	//
	for(u32 ob=0;ob < 2;ob++) {
		u32 face_count = 0;

		u32 numIdx = ((numOfIndex[ob] + 23)/24)*24;
		PrefetchForwardIterator<Idx24> itrIdx(
			&gPool,
			(u32)pclIdxColl[ob],
			(u32)pclIdxColl[ob] + sizeof(u16)*numIdx,
			PREFETCH_NUM, 10);

		u32 i = 0;
		for(;i< numOfIndex[ob] - 24;i+=24, ++itrIdx) {
			Idx24 idx24 = *itrIdx;

			Vector3 v[24];
			v[0]  = sftVtx[ob][spu_extract(idx24.id[0], 0)];
			v[1]  = sftVtx[ob][spu_extract(idx24.id[0], 1)];
			v[2]  = sftVtx[ob][spu_extract(idx24.id[0], 2)];
			v[3]  = sftVtx[ob][spu_extract(idx24.id[0], 3)];
			v[4]  = sftVtx[ob][spu_extract(idx24.id[0], 4)];
			v[5]  = sftVtx[ob][spu_extract(idx24.id[0], 5)];
			v[6]  = sftVtx[ob][spu_extract(idx24.id[0], 6)];
			v[7]  = sftVtx[ob][spu_extract(idx24.id[0], 7)];
			v[8]  = sftVtx[ob][spu_extract(idx24.id[1], 0)];
			v[9]  = sftVtx[ob][spu_extract(idx24.id[1], 1)];
			v[10] = sftVtx[ob][spu_extract(idx24.id[1], 2)];
			v[11] = sftVtx[ob][spu_extract(idx24.id[1], 3)];
			v[12] = sftVtx[ob][spu_extract(idx24.id[1], 4)];
			v[13] = sftVtx[ob][spu_extract(idx24.id[1], 5)];
			v[14] = sftVtx[ob][spu_extract(idx24.id[1], 6)];
			v[15] = sftVtx[ob][spu_extract(idx24.id[1], 7)];
			v[16] = sftVtx[ob][spu_extract(idx24.id[2], 0)];
			v[17] = sftVtx[ob][spu_extract(idx24.id[2], 1)];
			v[18] = sftVtx[ob][spu_extract(idx24.id[2], 2)];
			v[19] = sftVtx[ob][spu_extract(idx24.id[2], 3)];
			v[20] = sftVtx[ob][spu_extract(idx24.id[2], 4)];
			v[21] = sftVtx[ob][spu_extract(idx24.id[2], 5)];
			v[22] = sftVtx[ob][spu_extract(idx24.id[2], 6)];
			v[23] = sftVtx[ob][spu_extract(idx24.id[2], 7)];

			for(u32 j=0;j < 8;j++, face_count++) {
				Vector3 mMin, mMax;
				mMin = minPerElem(v[3*j + 0], minPerElem(v[3*j + 1], v[3*j + 2]));
				mMax = maxPerElem(v[3*j + 0], maxPerElem(v[3*j + 1], v[3*j + 2]));

				if(maxElem(overlapMin - mMax) > 0.0f)
					cachedflag[ob][face_count] = true;
				else if(maxElem(mMin - overlapMax) > 0.0f)
					cachedflag[ob][face_count] = true;
				else
					cachedflag[ob][face_count] = false;
			}
		} // end of -24 loop
		{ // left range calculation

			Idx24 idx24 = *itrIdx;
			for(u32 j=0;i < numOfIndex[ob];i+=3, j+=3, face_count++) {
				u16 idx0 = spu_extract(idx24.id[(j + 0)>>3], (i + 0)&0x07);
				u16 idx1 = spu_extract(idx24.id[(j + 1)>>3], (i + 1)&0x07);
				u16 idx2 = spu_extract(idx24.id[(j + 2)>>3], (i + 2)&0x07);

				Vector3 mMin, mMax;
				mMin = minPerElem(sftVtx[ob][idx0], minPerElem(sftVtx[ob][idx1], sftVtx[ob][idx2]));
				mMax = maxPerElem(sftVtx[ob][idx0], maxPerElem(sftVtx[ob][idx1], sftVtx[ob][idx2]));

				if(maxElem(overlapMin - mMax) > 0.0f)
					cachedflag[ob][face_count] = true;
				else if(maxElem(mMin - overlapMax) > 0.0f)
					cachedflag[ob][face_count] = true;
				else
					cachedflag[ob][face_count] = false;
			}
		}
	}

	spu_dma_wait_tag_status_all(1<<groupDmaTag);

	//
	// Rigrous Triangle vs Triangle Check
	//
	Vector3	vtx[2][3], min[2], max[2];
	{
		u32 face_count = 0;

		u32 numIdx = ((numOfIndex[0] + 23)/24)*24;
		PrefetchForwardIterator<Idx24> itrIdx(
			&gPool,
			(u32)pclIdxColl[0],
			(u32)pclIdxColl[0] + sizeof(u16)*numIdx,
			PREFETCH_NUM, 10);

		u32 i = 0;
		for(;i < numOfIndex[0] - 24;i+=24, ++itrIdx) {
			Idx24 idx24 = *itrIdx;

			Vector3 v[24];
			v[0]  = sftVtx[0][spu_extract(idx24.id[0], 0)];
			v[1]  = sftVtx[0][spu_extract(idx24.id[0], 1)];
			v[2]  = sftVtx[0][spu_extract(idx24.id[0], 2)];
			v[3]  = sftVtx[0][spu_extract(idx24.id[0], 3)];
			v[4]  = sftVtx[0][spu_extract(idx24.id[0], 4)];
			v[5]  = sftVtx[0][spu_extract(idx24.id[0], 5)];
			v[6]  = sftVtx[0][spu_extract(idx24.id[0], 6)];
			v[7]  = sftVtx[0][spu_extract(idx24.id[0], 7)];
			v[8]  = sftVtx[0][spu_extract(idx24.id[1], 0)];
			v[9]  = sftVtx[0][spu_extract(idx24.id[1], 1)];
			v[10] = sftVtx[0][spu_extract(idx24.id[1], 2)];
			v[11] = sftVtx[0][spu_extract(idx24.id[1], 3)];
			v[12] = sftVtx[0][spu_extract(idx24.id[1], 4)];
			v[13] = sftVtx[0][spu_extract(idx24.id[1], 5)];
			v[14] = sftVtx[0][spu_extract(idx24.id[1], 6)];
			v[15] = sftVtx[0][spu_extract(idx24.id[1], 7)];
			v[16] = sftVtx[0][spu_extract(idx24.id[2], 0)];
			v[17] = sftVtx[0][spu_extract(idx24.id[2], 1)];
			v[18] = sftVtx[0][spu_extract(idx24.id[2], 2)];
			v[19] = sftVtx[0][spu_extract(idx24.id[2], 3)];
			v[20] = sftVtx[0][spu_extract(idx24.id[2], 4)];
			v[21] = sftVtx[0][spu_extract(idx24.id[2], 5)];
			v[22] = sftVtx[0][spu_extract(idx24.id[2], 6)];
			v[23] = sftVtx[0][spu_extract(idx24.id[2], 7)];


			for(u32 j=0;j < 8;j++, face_count++) {
				if(cachedflag[0][face_count])
					continue;

				vtx[0][0] = v[3*j + 0];
				vtx[0][1] = v[3*j + 1];
				vtx[0][2] = v[3*j + 2];

				min[0] = minPerElem(vtx[0][0], minPerElem(vtx[0][1], vtx[0][2]));
				max[0] = maxPerElem(vtx[0][0], maxPerElem(vtx[0][1], vtx[0][2]));

				for(u32 k=0;k < numOfIndex[1];k+=3) {
					if(cachedflag[1][k/3]) continue;

					vtx[1][0] = sftVtx[1][sftIdx[1][k + 0]];
					vtx[1][1] = sftVtx[1][sftIdx[1][k + 1]];
					vtx[1][2] = sftVtx[1][sftIdx[1][k + 2]];

					min[1] = minPerElem(vtx[1][0], minPerElem(vtx[1][1], vtx[1][2]));
					max[1] = maxPerElem(vtx[1][0], maxPerElem(vtx[1][1], vtx[1][2]));

					//
					// Bounding Box of Triangle vs Bounding Box of Triangle
					//
					if(maxElem(min[0] - max[1]) > 0.0f)
						continue;
					else if(maxElem(min[1] - max[0]) > 0.0f)
						continue;
					else {
						// should avoid penetration
					}

					//
					// Pre Data Read
					//
					ATTRIBUTE_ALIGNED16(Vector3 LocalPress0);
					ATTRIBUTE_ALIGNED16(Vector3	LocalPress1);
					ATTRIBUTE_ALIGNED16(Vector3 GlobalPress0);
					ATTRIBUTE_ALIGNED16(Vector3	GlobalPress1);
					ATTRIBUTE_ALIGNED16(u16 PressCount0);
					ATTRIBUTE_ALIGNED16(u16 PressCount1);

					lock(commonBuff, buildMeshIO.commonbuf);
					spu_dma_get(&LocalPress0, (u32)pclPressureLocalAddr[0] + sizeof(Vector3)*face_count, sizeof(Vector3), groupDmaTag, 0, 0);
					spu_dma_get(&LocalPress1, (u32)pclPressureLocalAddr[1] + sizeof(Vector3)*k/3, sizeof(Vector3), groupDmaTag, 0, 0);
					spu_dma_get(&GlobalPress0, (u32)pclPressureGlobalAddr[0] +sizeof(Vector3)*face_count, sizeof(Vector3), groupDmaTag, 0, 0);
					spu_dma_get(&GlobalPress1, (u32)pclPressureGlobalAddr[1] + sizeof(Vector3)*k/3, sizeof(Vector3), groupDmaTag, 0, 0);
					PressCount0 = spu_dma_get_uint16((u32)pclPressureCountAddr[0] + sizeof(u16)*face_count, groupDmaTag, 0, 0);
					PressCount1 = spu_dma_get_uint16((u32)pclPressureCountAddr[1] + sizeof(u16)*(k/3),  groupDmaTag, 0, 0);

					//
					// Triangle vs Triangle Intersection Point
					//
					f32 cdepth = 0.0;
					f32 cfraction = 0.0;
					bool coll_flag = TrgTrgCollision(vtx, cdepth, cfraction);

					if(coll_flag) {
						spu_dma_wait_tag_status_all(1<<groupDmaTag);

						GlobalPress0 += cfraction*LocalPress1/pressureConst[1]*pressureConst[0];
						GlobalPress1 += cfraction*LocalPress0/pressureConst[0]*pressureConst[1];
						PressCount0++;
						PressCount1++;

						spu_dma_put(&GlobalPress0, (u32)pclPressureGlobalAddr[0] + sizeof(Vector3)*face_count, sizeof(Vector3), groupDmaTag, 0, 0);
						spu_dma_put(&GlobalPress1, (u32)pclPressureGlobalAddr[1] + sizeof(Vector3)*k/3, sizeof(Vector3), groupDmaTag, 0, 0);
						spu_dma_put_uint16(PressCount0, (u32)pclPressureCountAddr[0] + sizeof(u16)*face_count, groupDmaTag, 0, 0);
						spu_dma_put_uint16(PressCount1, (u32)pclPressureCountAddr[1] + sizeof(u16)*(k/3), groupDmaTag, 0, 0);

						spu_dma_wait_tag_status_all(1<<groupDmaTag);

						unlock(commonBuff, buildMeshIO.commonbuf);

					} else {
						spu_dma_wait_tag_status_all(1<<groupDmaTag);
						unlock(commonBuff,buildMeshIO.commonbuf);
					}

				}
			}
		}

		// end of -24 loop
		// left range calculation
		Idx24 idx24 = *itrIdx;
		for(u32 j=0;i < numOfIndex[0];i+=3, j+=3,face_count++) {
			if(cachedflag[0][face_count])
				continue;

			u16 idx0 = spu_extract(idx24.id[(j + 0)>>3],(i + 0)&0x07);
			u16 idx1 = spu_extract(idx24.id[(j + 1)>>3],(i + 1)&0x07);
			u16 idx2 = spu_extract(idx24.id[(j + 2)>>3],(i + 2)&0x07);

			vtx[0][0] = sftVtx[0][idx0];
			vtx[0][1] = sftVtx[0][idx1];
			vtx[0][2] = sftVtx[0][idx2];

			min[0] = minPerElem(vtx[0][0], minPerElem(vtx[0][1], vtx[0][2]));
			max[0] = maxPerElem(vtx[0][0], maxPerElem(vtx[0][1], vtx[0][2]));

			for(u32 k=0;k < numOfIndex[1];k+=3) {
				if(cachedflag[1][k/3])
					continue;

				vtx[1][0] = sftVtx[1][sftIdx[1][k + 0]];
				vtx[1][1] = sftVtx[1][sftIdx[1][k + 1]];
				vtx[1][2] = sftVtx[1][sftIdx[1][k + 2]];

				min[1] = minPerElem(vtx[1][0], minPerElem(vtx[1][1], vtx[1][2]));
				max[1] = maxPerElem(vtx[1][0], maxPerElem(vtx[1][1], vtx[1][2]));

				//
				// Bounding Box of Triangle vs Bounding Box of Triangle
				//
				if(maxElem(min[0] - max[1]) > 0.0f)
					continue;
				else if(maxElem(min[1] - max[0]) > 0.0f)
					continue;
				else {

					//
					// Pre Data Read
					//
					ATTRIBUTE_ALIGNED16(Vector3 LocalPress0);
					ATTRIBUTE_ALIGNED16(Vector3	LocalPress1);
					ATTRIBUTE_ALIGNED16(Vector3 GlobalPress0);
					ATTRIBUTE_ALIGNED16(Vector3	GlobalPress1);
					ATTRIBUTE_ALIGNED16(u16 PressCount0);
					ATTRIBUTE_ALIGNED16(u16 PressCount1);

					lock(commonBuff, buildMeshIO.commonbuf);
					spu_dma_get(&LocalPress0, (u32)pclPressureLocalAddr[0] + sizeof(Vector3)*face_count, sizeof(Vector3), 8, 0, 0);
					spu_dma_get(&LocalPress1, (u32)pclPressureLocalAddr[1] + sizeof(Vector3)*k/3, sizeof(Vector3), 8, 0, 0);
					spu_dma_get(&GlobalPress0, (u32)pclPressureGlobalAddr[0] + sizeof(Vector3)*face_count, sizeof(Vector3), 8, 0, 0);
					spu_dma_get(&GlobalPress1, (u32)pclPressureGlobalAddr[1] + sizeof(Vector3)*k/3, sizeof(Vector3), 8, 0, 0);
					PressCount0 = spu_dma_get_uint16((u32)pclPressureCountAddr[0] + sizeof(u16)*face_count, 8, 0, 0);
					PressCount1 = spu_dma_get_uint16((u32)pclPressureCountAddr[1] + sizeof(u16)*(k/3), 8, 0, 0);

					//
					// Triangle vs Triangle Intersection Point
					//
					f32 cdepth = 0.0;
					f32 cfraction = 0.0;
					bool coll_flag = TrgTrgCollision(vtx, cdepth, cfraction);

					if(coll_flag) {
						spu_dma_wait_tag_status_all(1<<groupDmaTag);

						GlobalPress0 += cfraction*LocalPress1/pressureConst[1]*pressureConst[0];
						GlobalPress1 += cfraction*LocalPress0/pressureConst[0]*pressureConst[1];

						PressCount0++;
						PressCount1++;

						spu_dma_put(&GlobalPress0, (u32)pclPressureGlobalAddr[0] + sizeof(Vector3)*face_count, sizeof(Vector3), groupDmaTag, 0, 0);
						spu_dma_put(&GlobalPress1, (u32)pclPressureGlobalAddr[1] + sizeof(Vector3)*k/3, sizeof(Vector3), groupDmaTag, 0, 0);
						spu_dma_put_uint16(PressCount0, (u32)pclPressureCountAddr[0] + sizeof(u16)*face_count, groupDmaTag, 0, 0);
						spu_dma_put_uint16(PressCount1, (u32)pclPressureCountAddr[1] + sizeof(u16)*(k/3), groupDmaTag, 0, 0);
						spu_dma_wait_tag_status_all(1<<groupDmaTag);

						unlock(commonBuff, buildMeshIO.commonbuf);
					}
					else {
						spu_dma_wait_tag_status_all(1<<groupDmaTag);
						unlock(commonBuff, buildMeshIO.commonbuf);
					}
				}
			}
		}
	}

	DEALLOCATE(sftIdx[1]);
//	DEALLOCATE(sftIdx[0]);
	DEALLOCATE(sftVtx[1]);
	DEALLOCATE(sftVtx[0]);
	DEALLOCATE(cachedflag[1]);
	DEALLOCATE(cachedflag[0]);
	DEALLOCATE(commonBuff);
}

void accumPressure(sftsftCollisionSetup ioParam)
{

	SoftState *sftStates = (SoftState*)ALLOCATE(16, sizeof(SoftState)*softbodyGroupIO.numVertices);
	Vector3 *glbPress = (Vector3*)ALLOCATE(16, sizeof(Vector3)*softbodyGroupIO.numIndices/3);
	u16 *glbCount = (u16*)ALLOCATE(16, sizeof(u16)*softbodyGroupIO.numIndices/3);

	spu_dma_large_get(sftStates, softbodyGroupIO.sftStatesAddr, sizeof(SoftState)*softbodyGroupIO.numVertices, 0, 0, 0);
	spu_dma_large_get(glbPress, (u32)ioParam.pclPressureGlobal, sizeof(Vector3)*ioParam.numIndices/3, 0, 0, 0);
	spu_dma_large_get(glbCount, (u32)ioParam.pclPressureCount, sizeof(u16)*ioParam.numIndices/3, 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	u32 face_count=0;
	u32 numIdx = ((softbodyGroupIO.numIndices + 23)/24)*24;
	PrefetchForwardIterator<Idx24> itrIdx(
		&gPool,
		softbodyGroupIO.indicesAddr,
		softbodyGroupIO.indicesAddr + sizeof(u16)*numIdx,
		PREFETCH_NUM, 10);

	u32 i = 0;
	for(;i < softbodyGroupIO.numIndices - 24;i+=24, ++itrIdx) {
		Idx24 idx24 = *itrIdx;

		u16 idx[24];
		idx[0]  = spu_extract(idx24.id[0], 0);
		idx[1]  = spu_extract(idx24.id[0], 1);
		idx[2]  = spu_extract(idx24.id[0], 2);
		idx[3]  = spu_extract(idx24.id[0], 3);
		idx[4]  = spu_extract(idx24.id[0], 4);
		idx[5]  = spu_extract(idx24.id[0], 5);
		idx[6]  = spu_extract(idx24.id[0], 6);
		idx[7]  = spu_extract(idx24.id[0], 7);
		idx[8]  = spu_extract(idx24.id[1], 0);
		idx[9]  = spu_extract(idx24.id[1], 1);
		idx[10] = spu_extract(idx24.id[1], 2);
		idx[11] = spu_extract(idx24.id[1], 3);
		idx[12] = spu_extract(idx24.id[1], 4);
		idx[13] = spu_extract(idx24.id[1], 5);
		idx[14] = spu_extract(idx24.id[1], 6);
		idx[15] = spu_extract(idx24.id[1], 7);
		idx[16] = spu_extract(idx24.id[2], 0);
		idx[17] = spu_extract(idx24.id[2], 1);
		idx[18] = spu_extract(idx24.id[2], 2);
		idx[19] = spu_extract(idx24.id[2], 3);
		idx[20] = spu_extract(idx24.id[2], 4);
		idx[21] = spu_extract(idx24.id[2], 5);
		idx[22] = spu_extract(idx24.id[2], 6);
		idx[23] = spu_extract(idx24.id[2], 7);

		for(u32 j=0;j < 8;j++, face_count++) {
			Vector3 pressForce = glbPress[face_count]/1; //(glbCount[face_count]+1);
			sftStates[idx[3*j + 0]].externalForce += pressForce;
			sftStates[idx[3*j + 1]].externalForce += pressForce;
			sftStates[idx[3*j + 2]].externalForce += pressForce;
		}
	}


	Idx24 idx24 = *itrIdx;
	for(u32 j=0;i < softbodyGroupIO.numIndices;i+=3, j+=3, face_count++) {
		u16 idx0 = spu_extract(idx24.id[(j + 0)>>3],(i + 0)&0x07);
		u16 idx1 = spu_extract(idx24.id[(j + 1)>>3],(i + 1)&0x07);
		u16 idx2 = spu_extract(idx24.id[(j + 2)>>3],(i + 2)&0x07);

		Vector3 pressForce = glbPress[face_count]/1; //(glbCount[face_count]+1);
		sftStates[idx0].externalForce += pressForce;
		sftStates[idx1].externalForce += pressForce;
		sftStates[idx2].externalForce += pressForce;
	}

	spu_dma_large_put(sftStates, softbodyGroupIO.sftStatesAddr, sizeof(SoftState)*softbodyGroupIO.numVertices, 0, 0, 0);
	spu_dma_wait_tag_status_all(1);

	DEALLOCATE(glbCount);
	DEALLOCATE(glbPress);
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
		case SOFTBODY_BUILDMESH:
		{
			s32 groupDmaTag = 8;

			spu_dma_get(&buildMeshIO, addrIo, sizeof(IOParamBuildMeshSoft), 0, 0, 0);
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
					spu_dma_get(&sftCollisionSetupIO[0], buildMeshIO.ioParamAddr + sizeof(sftsftCollisionSetup)*nextGroup, sizeof(sftsftCollisionSetup), groupDmaTag, 0, 0);
					commonBuff[2]++;
				} else
					empty = true;

				unlock(commonBuff, commonBuffAddr);

				if(!empty) {
					spu_dma_wait_tag_status_all(1<<groupDmaTag);
					buildMesh(groupAddr, nextGroup);
				}
			}
			spu_dma_wait_tag_status_all(1<<groupDmaTag);

			DEALLOCATE(commonBuff);
		}
		break;

		case SOFTBODY_SFTSFTCOLLISION:
		{
			s32 groupDmaTag = 8;

			spu_dma_get(&buildMeshIO, addrIo, sizeof(IOParamBuildMeshSoft), 0, 0, 0);
			spu_dma_wait_tag_status_all(1);

			u32 commonBuffAddr = taskbuff[2];
			u32 *commonBuff = (u32*)ALLOCATE(128, sizeof(u32)*32);
			bool empty = false;
			while(!empty) {
				lock(commonBuff, commonBuffAddr);

				//u32 groupAddr = commonBuff[1];
				u32 nextGroup = commonBuff[2];
				u32 maxGroup  = commonBuff[3]*commonBuff[3]; // Sft vs Sftのペア数


				u32 index0 = 0, index1 = 0;
				if(nextGroup < maxGroup) {
					index0 = nextGroup/commonBuff[3];
					index1 = nextGroup%commonBuff[3];
					spu_dma_get(&sftCollisionSetupIO[0], buildMeshIO.ioParamAddr + sizeof(sftsftCollisionSetup)*index0, sizeof(sftsftCollisionSetup), groupDmaTag, 0, 0);
					spu_dma_get(&sftCollisionSetupIO[1], buildMeshIO.ioParamAddr + sizeof(sftsftCollisionSetup)*index1, sizeof(sftsftCollisionSetup), groupDmaTag, 0, 0);
					commonBuff[2]++;
				} else
					empty = true;

				unlock(commonBuff, commonBuffAddr);

				if(!empty) {
					spu_dma_wait_tag_status_all(1<<groupDmaTag);

					if(index0>=index1) {
						// do nothing
					}
					else if(sftCollisionSetupIO[0].sftsftCollisionEnable && sftCollisionSetupIO[1].sftsftCollisionEnable)
						sftsftCollisionDetect(sftCollisionSetupIO[0], sftCollisionSetupIO[1]);
				}
			}
			spu_dma_wait_tag_status_all(1<<groupDmaTag);

			DEALLOCATE(commonBuff);
		}
		break;

		case SOFTBODY_ACCUMPRESSURE:
		{
			s32 groupDmaTag = 8;

			spu_dma_get(&buildMeshIO, addrIo, sizeof(IOParamBuildMeshSoft), 0, 0, 0);
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
					spu_dma_get(&sftCollisionSetupIO[0], buildMeshIO.ioParamAddr + sizeof(sftsftCollisionSetup)*nextGroup, sizeof(sftsftCollisionSetup), groupDmaTag, 0, 0);
					commonBuff[2]++;
				} else
					empty = true;

				unlock(commonBuff, commonBuffAddr);

				if(!empty) {
					spu_dma_wait_tag_status_all(1<<groupDmaTag);

					if(softbodyGroupIO.curProperty.volume > 0.01f)
						accumPressure(sftCollisionSetupIO[0]);
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
