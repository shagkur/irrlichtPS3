/*
 * closestparticle.cpp
 *
 *  Created on: Jan 27, 2014
 *      Author: mike
 */

#include "base/common.h"
#include "base/heapmanager.h"
#include "base/prefetchiterator.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/subdata.h"
#include "rigidbody/common/contact.h"
#include "rigidbody/common/spherespheredistance.h"
#include "rigidbody/common/boxspheredistance.h"
#include "rigidbody/common/capsulespheredistance.h"
#include "rigidbody/common/convexspheredistance.h"
#include "rigidbody/common/trianglesspherecontacts.h"
#include "rigidbody/common/contactheightfield.h"
#include "rigidbody/common/heightfieldfunction.h"

#include "particle/common/closestparticle.h"

#define AABB_PREFETCH_NUM			32

extern HeapManager gPool;

///////////////////////////////////////////////////////////////////////////////
// Particle Contact Function Table

#define PCLCONTACTFUNC(funcName) \
	s32 funcName(ContactPoint *cp, const Sphere &sphereA, const Transform3& primTransformA, const CollPrim &primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist);

typedef s32 (*PclContacts)(ContactPoint *cp, const Sphere& sphereA, const Transform3& primTransformA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist);

PCLCONTACTFUNC(pclContactsSphere)
PCLCONTACTFUNC(pclContactsBox)
PCLCONTACTFUNC(pclContactsCapsule)
PCLCONTACTFUNC(pclContactsHeightField)
PCLCONTACTFUNC(pclContactsConvex)
PCLCONTACTFUNC(pclContactsLargeMesh)
PCLCONTACTFUNC(pclContactsDummy)

PclContacts funcTbl_pclContacts[PRIM_COUNT] = {
	pclContactsSphere, pclContactsBox, pclContactsCapsule, pclContactsHeightField, pclContactsConvex, pclContactsDummy, pclContactsLargeMesh
};

#define SET_CONTACT_POINT(contactPoint, dist, nml, pntA, pntB, trnsB, primB) \
	contactPoint.distance = dist;\
	contactPoint.setNormal(nml);\
	contactPoint.setA(pntA, Transform3::identity(), 0);\
	contactPoint.setB(pntB, trnsB, primB);\
	contactPoint.subData.type = SubData::SubDataNone;

class CachedConvexMesh
{
public:
	ConvexMesh *mesh;

	CachedConvexMesh(const CollPrim& prim)
	{
#ifdef __SPU__
		mesh = (ConvexMesh*)gPool.allocate(sizeof(ConvexMesh));
		prim.getConvexMesh(mesh);
#else
		mesh = prim.getConvexMesh();
#endif
	}

	~CachedConvexMesh()
	{
#ifdef __SPU__
		gPool.deallocate(mesh);
#endif
	}

	ConvexMesh *getMesh()
	{
		return mesh;
	}
};

s32 pclContactsDummy(ContactPoint *cp, const Sphere& sphereA, const Transform3& primTransformA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	(void) cp;
	(void) sphereA;
	(void) primTransformA;
	(void) primB;
	(void) primTransformB;
	(void) relTransformB;
	(void) primIndexB;
	(void) objsInContactDist;

	return 0;
}

s32 pclContactsBox(ContactPoint *cp, const Sphere& sphereA, const Transform3& primTransformA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal;
	SpherePoint spherePointA;
	BoxPoint boxPointB;
	f32 distance = boxSphereDistance(testNormal, boxPointB, spherePointA, primB.getBox(), primTransformB, sphereA, primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, spherePointA.localPoint, boxPointB.localPoint, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 pclContactsCapsule(ContactPoint *cp, const Sphere& sphereA, const Transform3& primTransformA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal;
	SpherePoint spherePointA;
	CapsulePoint capsulePointB;
	f32 distance = capsuleSphereDistance(testNormal, capsulePointB, spherePointA, primB.getCapsule(), primTransformB, sphereA, primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, spherePointA.localPoint, capsulePointB.localPoint, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 pclContactsSphere(ContactPoint *cp, const Sphere& sphereA, const Transform3& primTransformA, const CollPrim& primB, const Transform3& primTransformB, const Transform3 &relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal(0.0f);
	SpherePoint spherePointA;
	SpherePoint spherePointB;
	f32 distance = sphereSphereDistance(testNormal, spherePointA, spherePointB, sphereA, primTransformA, primB.getSphere(), primTransformB);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, spherePointA.localPoint, spherePointB.localPoint, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 pclContactsConvex(ContactPoint *cp, const Sphere& sphereA, const Transform3& primTransformA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal;
	Point3 pointA,pointB;
	CachedConvexMesh meshB(primB);
	f32 distance = closestConvexSphere(testNormal, pointB, pointA, meshB.getMesh(), primTransformB, sphereA, primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, pointA, pointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 pclContactsHeightField(ContactPoint *cp, const Sphere& sphereA, const Transform3& primTransformA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	(void) primTransformB;
	(void) objsInContactDist;

	Point3 cpB(0.0f);
	Vector3 nml(0.0f);
	f32 distance = 0.0f;
	bool ret =false;
	HeightField *heightfield;

#ifdef __SPU__
	initializeHeightFieldCache();

	HeightField heightfield_;
	primB.getHeightField(&heightfield_);
	heightfield = &heightfield_;
#else
	heightfield = primB.getHeightField();
#endif

	Transform3 transformBA = orthoInverse(primTransformB)*primTransformA;
	Transform3 transformAB = orthoInverse(primTransformA)*primTransformB;

	Point3 spherePoint = transformBA.getTranslation() + Point3(0.0f, -sphereA.radius, 0.0f);
	if(contactHeightField(heightfield, spherePoint, cpB, nml, distance) && distance < cp[0].distance) {
		Point3 cpA = transformAB * spherePoint;
		SET_CONTACT_POINT(cp[0], distance, primTransformB.getUpper3x3()*nml, cpA, cpB, relTransformB, primIndexB);
		ret = true;
	}

#ifdef __SPU__
	releaseHeightFieldCache();
#endif

	return ret ? 1 : 0;
}

static inline u32 dmaGetBuffer(void* ls, u32 ea, s32 sz, u32 tag, u32 endEA)
{
	s32 sz_tmp = (ea + sz <= endEA) ? sz : endEA - ea;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_large_get(ls, ea, sz_tmp, tag, 0, 0);
	return sz_tmp;
}

s32 pclContactsLargeMesh(ContactPoint *cp, const Sphere& sphereA, const Transform3& primTransformA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	s32 numContacts = 0;
	LargeTriMesh largeMesh;
	primB.getLargeMeshNB(&largeMesh,0);

	Transform3 transformBA;
	Matrix3 matrixBA;
	Vector3 offsetBA;

	transformBA = orthoInverse(primTransformB)*primTransformA;
	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	Vector3 shapeHalf(sphereA.radius);
	Vector3 shapeCenter = offsetBA;

	spu_dma_wait_tag_status_all(1);

	VecInt3 aabbMinL = largeMesh.getLocalPosition((shapeCenter - shapeHalf));
	VecInt3 aabbMaxL = largeMesh.getLocalPosition((shapeCenter + shapeHalf));

	vec_uint4 ptn_mask0 = {0x02031213, 0x06071617, 0x0A0B1A1B, 0x80808080};
	vec_ushort8 vecAabbA;
	vecAabbA = spu_shuffle((vec_ushort8)aabbMinL.get128(), (vec_ushort8)aabbMaxL.get128(), (vec_uchar16)ptn_mask0);
	{
		ReadOnlyPrefetchForwardIterator<AABB16> itrAABB(
			&gPool,
			(u32)largeMesh.aabbList,
			(u32)(largeMesh.aabbList + largeMesh.numIslands),
			AABB_PREFETCH_NUM, 10);

		for(u8 i=0;i < largeMesh.numIslands;i++,++itrAABB) {
			AABB16 aabbB = *itrAABB;
			vec_ushort8 vecAabbB = (vec_ushort8)aabbB;
			vec_uint4 ptn_mask1 = {0x10110001, 0x14150405, 0x18190809, 0x80808080};
			vec_ushort8 vecMin = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask1);
			vec_ushort8 vecMax = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask0);
			vec_ushort8 isGt = spu_cmpgt(vecMin, vecMax);
			if(spu_extract(spu_gather(isGt), 0) > 0) continue;

			TriMesh *island = (TriMesh*)gPool.allocate(sizeof(TriMesh), HeapManager::ALIGN16);
			spu_dma_get(island, (u64)(largeMesh.islands + i), sizeof(TriMesh), 3, 0, 0);
			spu_dma_wait_tag_status_all(1<<3);

			Vector3 testNormal[4];
			Point3 pointA[4],pointB[4];
			SubData subData[4];
			f32 distance[4];
			s32 numNewCp = trianglesSphereContacts(testNormal, subData, pointB, pointA, distance, island, primTransformB, sphereA, primTransformA, objsInContactDist);

			gPool.deallocate(island);

			for(s32 k=0;k < numNewCp;k++) {
				if(distance[k] >= objsInContactDist) continue;

				Vector3 newCp(pointA[k]);

				s32 replaceId = findNearestContactPoint(cp, numContacts, newCp, testNormal[k]);

				if(replaceId < 0) {
					if(numContacts < 4)
						replaceId = numContacts++;
					else
						replaceId = sort4ContactPoints(cp, newCp, distance[k]);
				}

				SET_CONTACT_POINT(cp[replaceId], distance[k], -testNormal[k], pointA[k], pointB[k], relTransformB, primIndexB);
				cp[replaceId].subData = subData[k];
				cp[replaceId].subData.type = SubData::SubDataFacetLocal;
				cp[replaceId].subData.setIslandIndex(i);
			}
		}
	}

	return numContacts;
}

///////////////////////////////////////////////////////////////////////////////
// Detect closest contact

// Particle x Particle
bool closestContactPcl(PclContactPair& contactPair, const Vector3& posA, f32 radA, const Vector3& posB, f32 radB, f32 objsInContactDist)
{
	Vector3 testNormal(0.0f);
	SpherePoint spherePointA;
	SpherePoint spherePointB;
	f32 distance = sphereSphereDistance(testNormal, spherePointA, spherePointB, Sphere(radA), Transform3(Matrix3::identity(), posA), Sphere(radB), Transform3(Matrix3::identity(), posB));

	if(distance < objsInContactDist) {
		contactPair.numContacts = 1;
		contactPair.contactPoint.distance = distance;
		contactPair.contactPoint.normal = -testNormal;
		contactPair.contactPoint.localPoint[0] = Vector3(spherePointA.localPoint);
		contactPair.contactPoint.localPoint[1] = Vector3(spherePointB.localPoint);
		return true;
	}

	return false;
}

// Particle x Rigid Body
bool closestContactRig(PclContactPair& contactPair, const Vector3& posA, f32 radA, const CollObject& objB, const Transform3& transformB, f32 objsInContactDist)
{
	contactPair.numContacts = 0;
	f32 maxDepth = FLT_MAX;
	Sphere sphereA(radA);
	Transform3 primTransformA(Matrix3::identity(), posA);

	PrimIterator itrPrim(objB);
	for(s32 j=0;j < objB.getNumPrims();j++,++itrPrim) {
		const CollPrim& primB = *itrPrim;
		Transform3 relTransformB = primB.getObjectRelTransform();
		Transform3 primTransformB = transformB*relTransformB;

		ContactPoint cp[NUMCONTACTS_PER_BODIES];
		s32 newContacts = funcTbl_pclContacts[primB.getType()](cp, sphereA, primTransformA, primB, primTransformB, relTransformB, j, objsInContactDist);

		for(s32 k=0;k < newContacts;k++) {
			if(maxDepth < cp[k].distance) continue;
			maxDepth = cp[k].getDistance();
			contactPair.numContacts = 1;
			contactPair.contactPoint.distance = cp[k].getDistance();
			contactPair.contactPoint.normal = cp[k].getNormal();
			contactPair.contactPoint.localPoint[0] = cp[k].getLocalPointA();
			contactPair.contactPoint.localPoint[1] = cp[k].getLocalPointB();
		}
	}

	return contactPair.numContacts > 0;
}

