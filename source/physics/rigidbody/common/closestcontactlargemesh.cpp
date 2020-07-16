/*
 * closestcontactlargemesh.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"

#include "rigidbody/common/contact.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/trianglesboxcontacts.h"
#include "rigidbody/common/trianglesspherecontacts.h"
#include "rigidbody/common/trianglescapsulecontacts.h"
#include "rigidbody/common/trianglesconvexcontacts.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/subdata.h"

#ifdef __SPU__
	#include "base/prefetchiterator.h"
	#include "base/heapmanager.h"

	extern HeapManager gPool;

	#define AABB_PREFETCH_NUM 32
#endif

#define SET_CONTACT_POINT(contactPoint, dist, nml, pntA, trnsA, primA, pntB, trnsB, primB) \
	contactPoint.distance = dist;\
	contactPoint.setNormal(nml);\
	contactPoint.setA(pntA, trnsA, primA);\
	contactPoint.setB(pntB, trnsB, primB);\
	contactPoint.subData.type = SubData::SubDataNone;

///////////////////////////////////////////////////////////////////////////////
// largeMesh x Shape

#ifdef __SPU__

static inline u32 dmaGetBuffer(void* ls, u32 ea, s32 sz, u32 tag, u32 endEA)
{
	s32 sz_tmp = (ea + sz <= endEA) ? sz : endEA - ea;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_large_get(ls, ea, sz_tmp, tag, 0, 0);
	return sz_tmp;
}

s32 primContactsShapeLargeMesh(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	s32 numContacts = 0;

	ConvexMesh *meshA = NULL;
	LargeTriMesh largeMesh;
	primB.getLargeMeshNB(&largeMesh, 0);

	Transform3 transformBA;
	Matrix3 matrixBA;
	Vector3 offsetBA;

	transformBA = orthoInverse(primTransformB)*primTransformA;
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	Vector3 shapeHalf(0.0f);
	Vector3 shapeCenter = offsetBA;

	switch(primA.getType()) {
		case SPHERE:
			meshA = (ConvexMesh*)gPool.allocate(0);
			shapeHalf = primA.getSphere().getAABB();
			break;

		case CAPSULE:
			meshA = (ConvexMesh*)gPool.allocate(0);
			shapeHalf = primA.getCapsule().getAABB(matrixBA.getCol0());
			break;

		case BOX:
			meshA = (ConvexMesh*)gPool.allocate(0);
			shapeHalf = primA.getBox().getAABB(matrixBA);
			break;

		case CONVEXMESH:
			meshA = (ConvexMesh*)gPool.allocate(sizeof(ConvexMesh));
			primA.getConvexMesh(meshA);
			shapeHalf = meshA->getAABB(matrixBA);
			break;

		default:
			meshA = (ConvexMesh*)gPool.allocate(0);
			break;
	}

	spu_dma_wait_tag_status_all(1);

	VecInt3 aabbMinL = largeMesh.getLocalPosition((shapeCenter - shapeHalf));
	VecInt3 aabbMaxL = largeMesh.getLocalPosition((shapeCenter + shapeHalf));

	vec_uint4 ptn_mask0 = ((vec_uint4){0x02031213, 0x06071617, 0x0A0B1A1B, 0x80808080});
	vec_ushort8 vecAabbA;
	vecAabbA = spu_shuffle((vec_ushort8)aabbMinL.get128(), (vec_ushort8)aabbMaxL.get128(), (vec_uchar16)ptn_mask0);

	{
		ReadOnlyPrefetchForwardIterator<AABB16> itrAABB(
			&gPool,
			(u32)largeMesh.aabbList,
			(u32)(largeMesh.aabbList + largeMesh.numIslands),
			AABB_PREFETCH_NUM, 10);

		u32 numIslands = largeMesh.numIslands;
		for(u32 i=0;i < numIslands;i++, ++itrAABB) {
			AABB16 aabbB = *itrAABB;
			vec_ushort8 vecAabbB = (vec_ushort8)aabbB;
			vec_uint4 ptn_mask1 = ((vec_uint4){0x10110001 ,0x14150405, 0x18190809, 0x80808080});
			vec_ushort8 vecMin = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask1);
			vec_ushort8 vecMax = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask0);
			vec_ushort8 isGt = spu_cmpgt(vecMin, vecMax);
			if(spu_extract(spu_gather(isGt), 0) > 0) continue;

			TriMesh *island = (TriMesh*)gPool.allocate(sizeof(TriMesh), HeapManager::ALIGN16);
			spu_dma_get(island, (u64)(largeMesh.islands + i), sizeof(TriMesh), 3, 0, 0);
			spu_dma_wait_tag_status_all(1<<3);

			Vector3 testNormal[4];
			Point3 pointA[4], pointB[4];
			SubData subData[4];
			f32 distance[4];
			s32 numNewCp = 0;

			switch(primA.getType()) {
				case SPHERE:
					numNewCp = trianglesSphereContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, primA.getSphere(), primTransformA, objsInContactDist);
					break;

				case CAPSULE:
					numNewCp = trianglesCapsuleContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, primA.getCapsule(), primTransformA, objsInContactDist);
					break;

				case BOX:
					numNewCp = trianglesBoxContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, primA.getBox(), primTransformA, objsInContactDist);
					break;

				case CONVEXMESH:
					numNewCp = trianglesConvexContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, meshA, primTransformA, objsInContactDist);
					break;

				default:
					break;
			}

			gPool.deallocate(island);

			for(s32 k=0;k < numNewCp;k++) {
				if(distance[k] >= objsInContactDist) continue;

				Vector3 newCp(relTransformA*pointA[k]);

				s32 replaceId = findNearestContactPoint(cp, numContacts, newCp, testNormal[k]);

				if(replaceId < 0) {
					if(numContacts < 4)
						replaceId = numContacts++;
					else
						replaceId = sort4ContactPoints(cp, newCp, distance[k]);
				}

				SET_CONTACT_POINT(cp[replaceId], distance[k], -testNormal[k], pointA[k], relTransformA, primIndexA, pointB[k], relTransformB, primIndexB);
				cp[replaceId].subData = subData[k];
				cp[replaceId].subData.setIslandIndex(i);
			}
		}
	}

	gPool.deallocate(meshA);

	return numContacts;
}

s32 primContactsLargeMeshShape(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	s32 numContacts = 0;

	ConvexMesh *meshB = NULL;
	LargeTriMesh largeMesh;
	primA.getLargeMeshNB(&largeMesh, 0);

	Transform3 transformAB;
	Matrix3 matrixAB;
	Vector3 offsetAB;

	transformAB = orthoInverse(primTransformA)*primTransformB;
	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();

	Vector3 shapeHalf(0.0f);
	Vector3 shapeCenter = offsetAB;

	switch(primB.getType()) {
		case SPHERE:
			meshB = (ConvexMesh*)gPool.allocate(0);
			shapeHalf = primB.getSphere().getAABB();
			break;

		case CAPSULE:
			meshB = (ConvexMesh*)gPool.allocate(0);
			shapeHalf = primB.getCapsule().getAABB(matrixAB.getCol0());
			break;

		case BOX:
			meshB = (ConvexMesh*)gPool.allocate(0);
			shapeHalf = primB.getBox().getAABB(matrixAB);
			break;

		case CONVEXMESH:
			meshB = (ConvexMesh*)gPool.allocate(sizeof(ConvexMesh));
			primB.getConvexMesh(meshB);
			shapeHalf = meshB->getAABB(matrixAB);
			break;

		default:
			meshB = (ConvexMesh*)gPool.allocate(0);
			break;
	}

	spu_dma_wait_tag_status_all(1);

	VecInt3 aabbMinL = largeMesh.getLocalPosition((shapeCenter - shapeHalf));
	VecInt3 aabbMaxL = largeMesh.getLocalPosition((shapeCenter + shapeHalf));

	vec_uint4 ptn_mask0 = ((vec_uint4){0x02031213, 0x06071617, 0x0A0B1A1B, 0x80808080});
	vec_ushort8 vecAabbA;
	vecAabbA = spu_shuffle((vec_ushort8)aabbMinL.get128(), (vec_ushort8)aabbMaxL.get128(), (vec_uchar16)ptn_mask0);

	{
		ReadOnlyPrefetchForwardIterator<AABB16> itrAABB(
			&gPool,
			(u32)largeMesh.aabbList,
			(u32)(largeMesh.aabbList + largeMesh.numIslands),
			AABB_PREFETCH_NUM, 10);

		u32 numIslands = largeMesh.numIslands;
		for(u32 i=0;i < numIslands;i++, ++itrAABB) {
			AABB16 aabbB = *itrAABB;
			vec_ushort8 vecAabbB = (vec_ushort8)aabbB;
			vec_uint4 ptn_mask1 = ((vec_uint4){0x10110001, 0x14150405, 0x18190809, 0x80808080});
			vec_ushort8 vecMin = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask1);
			vec_ushort8 vecMax = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask0);
			vec_ushort8 isGt = spu_cmpgt(vecMin, vecMax);
			if(spu_extract(spu_gather(isGt), 0) > 0) continue;

			TriMesh *island = (TriMesh*)gPool.allocate(sizeof(TriMesh), HeapManager::ALIGN16);
			spu_dma_get(island, (u64)(largeMesh.islands + i), sizeof(TriMesh), 3, 0, 0);
			spu_dma_wait_tag_status_all(1<<3);

			Vector3 testNormal[4];
			Point3 pointA[4], pointB[4];
			SubData subData[4];
			f32 distance[4];
			s32 numNewCp = 0;

			switch(primB.getType()) {
				case SPHERE:
					numNewCp = trianglesSphereContacts(testNormal, subData, pointA, pointB, distance, island, primTransformA, primB.getSphere(), primTransformB, objsInContactDist);
					break;

				case CAPSULE:
					numNewCp = trianglesCapsuleContacts(testNormal, subData, pointA, pointB, distance, island, primTransformA, primB.getCapsule(), primTransformB, objsInContactDist);
					break;

				case BOX:
					numNewCp = trianglesBoxContacts(testNormal, subData, pointA, pointB, distance, island, primTransformA, primB.getBox(), primTransformB, objsInContactDist);
					break;

				case CONVEXMESH:
					numNewCp = trianglesConvexContacts(testNormal, subData, pointA, pointB, distance, island, primTransformA, meshB, primTransformB, objsInContactDist);
					break;

				default:
					break;
			}

			gPool.deallocate(island);

			for(s32 k=0;k < numNewCp;k++) {
				if(distance[k] >= objsInContactDist) continue;

				Vector3 newCp(relTransformA*pointA[k]);

				s32 replaceId = findNearestContactPoint(cp, numContacts, newCp, testNormal[k]);

				if(replaceId < 0) {
					if(numContacts < 4)
						replaceId = numContacts++;
					else
						replaceId = sort4ContactPoints(cp, newCp, distance[k]);
				}

				SET_CONTACT_POINT(cp[replaceId], distance[k], testNormal[k], pointA[k], relTransformA, primIndexA, pointB[k], relTransformB, primIndexB);
				cp[replaceId].subData = subData[k];
				cp[replaceId].subData.setIslandIndex(i);
			}
		}
	}

	gPool.deallocate(meshB);

	return numContacts;
}

#else // __SPU__

s32 primContactsShapeLargeMesh(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	s32 numContacts = 0;

	Transform3 transformBA;
	Matrix3 matrixBA;
	Vector3 offsetBA;

	transformBA = orthoInverse(primTransformB)*primTransformA;
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	Vector3 shapeHalf(0.0f);
	Vector3 shapeCenter = offsetBA;

	switch(primA.getType()) {
		case SPHERE:
			shapeHalf = primA.getSphere().getAABB();
			break;

		case CAPSULE:
			shapeHalf = primA.getCapsule().getAABB(matrixBA.getCol0());
			break;

		case BOX:
			shapeHalf = primA.getBox().getAABB(matrixBA);
			break;

		case CONVEXMESH:
			shapeHalf = primA.getConvexMesh()->getAABB(matrixBA);
			break;

		default:
			break;
	}

	LargeTriMesh *largeMesh = primB.getLargeMesh();

	VecInt3 aabbMinL = largeMesh->getLocalPosition((shapeCenter - shapeHalf));
	VecInt3 aabbMaxL = largeMesh->getLocalPosition((shapeCenter + shapeHalf));

	u32 numIslands = largeMesh->numIslands;
	for(u32 i=0;i < numIslands;i++) {
		AABB16 aabbB = largeMesh->aabbList[i];
		if(aabbMaxL.getX() < getXMin(aabbB) || aabbMinL.getX() > getXMax(aabbB)) continue;
		if(aabbMaxL.getY() < getYMin(aabbB) || aabbMinL.getY() > getYMax(aabbB)) continue;
		if(aabbMaxL.getZ() < getZMin(aabbB) || aabbMinL.getZ() > getZMax(aabbB)) continue;

		TriMesh *island = &largeMesh->islands[i];
		Vector3 testNormal[4];
		Point3 pointA[4], pointB[4];
		SubData subData[4];
		f32 distance[4];
		s32 numNewCp = 0;

		switch(primA.getType()) {
			case SPHERE:
				numNewCp = trianglesSphereContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, primA.getSphere(), primTransformA, objsInContactDist);
				break;

			case CAPSULE:
				numNewCp = trianglesCapsuleContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, primA.getCapsule(), primTransformA, objsInContactDist);
				break;

			case BOX:
				numNewCp = trianglesBoxContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, primA.getBox(), primTransformA, objsInContactDist);
				break;

			case CONVEXMESH:
				numNewCp = trianglesConvexContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, primA.getConvexMesh(), primTransformA, objsInContactDist);
				break;

			default:
				break;
		}

		for(s32 k=0;k < numNewCp;k++) {
			if(distance[k] >= objsInContactDist) continue;

			Vector3 newCp(relTransformA*pointA[k]);

			s32 replaceId = findNearestContactPoint(cp, numContacts, newCp, testNormal[k]);

			if(replaceId < 0) {
				if(numContacts < 4)
					replaceId = numContacts++;
				else
					replaceId = sort4ContactPoints(cp, newCp, distance[k]);
			}

			SET_CONTACT_POINT(cp[replaceId], distance[k], -testNormal[k], pointA[k], relTransformA, primIndexA, pointB[k], relTransformB, primIndexB);
			cp[replaceId].subData = subData[k];
			cp[replaceId].subData.setIslandIndex(i);
		}
	}

	return numContacts;
}

s32 primContactsLargeMeshShape(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	s32 numContacts = 0;

	Transform3 transformAB;
	Matrix3 matrixAB;
	Vector3 offsetAB;

	transformAB = orthoInverse(primTransformA)*primTransformB;
	matrixAB = transformAB.getUpper3x3();
	offsetAB = transformAB.getTranslation();

	Vector3 shapeHalf(0.0f);
	Vector3 shapeCenter = offsetAB;

	switch(primB.getType()) {
		case SPHERE:
			shapeHalf = primB.getSphere().getAABB();
			break;

		case CAPSULE:
			shapeHalf = primB.getCapsule().getAABB(matrixAB.getCol0());
			break;

		case BOX:
			shapeHalf = primB.getBox().getAABB(matrixAB);
			break;

		case CONVEXMESH:
			shapeHalf = primB.getConvexMesh()->getAABB(matrixAB);
			break;

		default:
			break;
	}

	LargeTriMesh *largeMesh = primA.getLargeMesh();

	VecInt3 aabbMinL = largeMesh->getLocalPosition((shapeCenter - shapeHalf));
	VecInt3 aabbMaxL = largeMesh->getLocalPosition((shapeCenter + shapeHalf));

	u32 numIslands = largeMesh->numIslands;
	for(u32 i=0;i < numIslands;i++) {
		AABB16 aabbB = largeMesh->aabbList[i];
		if(aabbMaxL.getX() < getXMin(aabbB) || aabbMinL.getX() > getXMax(aabbB)) continue;
		if(aabbMaxL.getY() < getYMin(aabbB) || aabbMinL.getY() > getYMax(aabbB)) continue;
		if(aabbMaxL.getZ() < getZMin(aabbB) || aabbMinL.getZ() > getZMax(aabbB)) continue;

		TriMesh *island = &largeMesh->islands[i];
		Vector3 testNormal[4];
		Point3 pointA[4], pointB[4];
		SubData subData[4];
		f32 distance[4];
		s32 numNewCp = 0;

		switch(primB.getType()) {
			case SPHERE:
			 numNewCp = trianglesSphereContacts(testNormal, subData, pointA, pointB, distance, island, primTransformA, primB.getSphere(), primTransformB, objsInContactDist);
			 break;

			case CAPSULE:
				numNewCp = trianglesCapsuleContacts(testNormal, subData, pointA, pointB, distance, island, primTransformA, primB.getCapsule(), primTransformB, objsInContactDist);
				break;

			case BOX:
				numNewCp = trianglesBoxContacts(testNormal, subData, pointA, pointB, distance, island, primTransformA, primB.getBox(), primTransformB, objsInContactDist);
				break;

			case CONVEXMESH:
				numNewCp = trianglesConvexContacts(testNormal, subData, pointA, pointB, distance, island, primTransformA, primB.getConvexMesh(), primTransformB, objsInContactDist);
				break;

			default:
				break;
		}

		for(s32 k=0;k < numNewCp;k++) {
			if(distance[k] >= objsInContactDist) continue;

			Vector3 newCp(relTransformA*pointA[k]);

			s32 replaceId = findNearestContactPoint(cp, numContacts, newCp, testNormal[k]);

			if(replaceId < 0) {
				if(numContacts < 4)
					replaceId = numContacts++;
				else
					replaceId = sort4ContactPoints(cp, newCp, distance[k]);
			}

			SET_CONTACT_POINT(cp[replaceId], distance[k], testNormal[k], pointA[k], relTransformA, primIndexA, pointB[k], relTransformB, primIndexB);
			cp[replaceId].subData = subData[k];
			cp[replaceId].subData.setIslandIndex(i);
		}
	}

	return numContacts;
}

#endif // __SPU__
