/*
 * closestcontact.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/simplestack.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/intersectfunction.h"
#include "rigidbody/common/subdata.h"
#include "rigidbody/common/closestcontact.h"
#include "rigidbody/common/contactcache.h"
#include "rigidbody/common/boxboxdistance.h"
#include "rigidbody/common/boxcapsuledistance.h"
#include "rigidbody/common/boxspheredistance.h"
#include "rigidbody/common/capsulecapsuledistance.h"
#include "rigidbody/common/capsulespheredistance.h"
#include "rigidbody/common/spherespheredistance.h"

#ifdef __SPU__
	#include "base/prefetchiterator.h"
	#include "base/heapmanager.h"
	extern HeapManager gPool;

	#define AABB_PREFETCH_NUM 				32
#endif

///////////////////////////////////////////////////////////////////////////////
// ContactPair Function Table

#define PRIMCONTACTFUNC(funcName) \
s32 funcName(\
				ContactPoint *cp,\
				const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA,\
				const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB,\
				f32 objsInContactDist);

typedef s32 (*PrimContacts)(
				ContactPoint *cp,
				const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA,
				const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB,
				f32 objsInContactDist);

PRIMCONTACTFUNC(primContactsDummy				)

PRIMCONTACTFUNC(primContactsSphereSphere		)
PRIMCONTACTFUNC(primContactsSphereBox			)
PRIMCONTACTFUNC(primContactsSphereCapsule		)
PRIMCONTACTFUNC(primContactsBoxSphere			)
PRIMCONTACTFUNC(primContactsBoxBox				)
PRIMCONTACTFUNC(primContactsBoxCapsule			)
PRIMCONTACTFUNC(primContactsCapsuleSphere		)
PRIMCONTACTFUNC(primContactsCapsuleBox			)
PRIMCONTACTFUNC(primContactsCapsuleCapsule		)
PRIMCONTACTFUNC(primContactsShapeHeightField	)
PRIMCONTACTFUNC(primContactsHeightFieldShape	)
PRIMCONTACTFUNC(primContactsLargeMeshShape		)
PRIMCONTACTFUNC(primContactsShapeLargeMesh		)

PRIMCONTACTFUNC(primContactsSphereConvex		)
PRIMCONTACTFUNC(primContactsBoxConvex			)
PRIMCONTACTFUNC(primContactsCapsuleConvex		)
PRIMCONTACTFUNC(primContactsHeightFieldConvex	)
PRIMCONTACTFUNC(primContactsConvexConvex		)
PRIMCONTACTFUNC(primContactsConvexSphere		)
PRIMCONTACTFUNC(primContactsConvexBox			)
PRIMCONTACTFUNC(primContactsConvexCapsule		)
PRIMCONTACTFUNC(primContactsConvexHeightField	)

PrimContacts funcTbl_primContacts[PRIM_COUNT][PRIM_COUNT] = {
	{primContactsSphereSphere		,primContactsSphereBox			,primContactsSphereCapsule		,primContactsShapeHeightField		,primContactsSphereConvex		,primContactsDummy,	primContactsShapeLargeMesh},
	{primContactsBoxSphere			,primContactsBoxBox				,primContactsBoxCapsule			,primContactsShapeHeightField		,primContactsBoxConvex			,primContactsDummy,	primContactsShapeLargeMesh},
	{primContactsCapsuleSphere		,primContactsCapsuleBox			,primContactsCapsuleCapsule		,primContactsShapeHeightField		,primContactsCapsuleConvex		,primContactsDummy,	primContactsShapeLargeMesh},
	{primContactsHeightFieldShape	,primContactsHeightFieldShape	,primContactsHeightFieldShape	,primContactsDummy					,primContactsHeightFieldConvex	,primContactsDummy,	primContactsDummy},
	{primContactsConvexSphere		,primContactsConvexBox			,primContactsConvexCapsule		,primContactsConvexHeightField		,primContactsConvexConvex		,primContactsDummy,	primContactsShapeLargeMesh},
	{primContactsDummy				,primContactsDummy				,primContactsDummy				,primContactsDummy					,primContactsDummy				,primContactsDummy,	primContactsDummy},
	{primContactsLargeMeshShape		,primContactsLargeMeshShape		,primContactsLargeMeshShape		,primContactsDummy					,primContactsLargeMeshShape		,primContactsDummy,	primContactsDummy},
};

///////////////////////////////////////////////////////////////////////////////
// ContactPair Function

#define SET_CONTACT_POINT(contactPoint, dist, nml, pntA, trnsA, primA, pntB, trnsB, primB) \
	contactPoint.distance = dist;\
	contactPoint.setNormal(nml);\
	contactPoint.setA(pntA.localPoint, trnsA, primA);\
	contactPoint.setB(pntB.localPoint, trnsB, primB);\
	contactPoint.subData.type = SubData::SubDataNone;

s32 primContactsDummy(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB,const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	(void) cp;
	(void) primA;
	(void) primTransformA;
	(void) relTransformA;
	(void) primIndexA;
	(void) primB;
	(void) primTransformB;
	(void) relTransformB;
	(void) primIndexB;
	(void) objsInContactDist;

	return 0;
}

s32 primContactsBoxBox(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal;
	BoxPoint boxPointA;
	BoxPoint boxPointB;
	f32 distance = boxBoxDistance(testNormal, boxPointA, boxPointB, primA.getBox(), primTransformA, primB.getBox(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, boxPointA, relTransformA, primIndexA, boxPointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsBoxCapsule(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal;
	BoxPoint boxPointA;
	CapsulePoint capsulePointB;
	f32 distance = boxCapsuleDistance(testNormal, boxPointA, capsulePointB, primA.getBox(), primTransformA, primB.getCapsule(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, boxPointA, relTransformA, primIndexA, capsulePointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsBoxSphere(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal;
	BoxPoint boxPointA;
	SpherePoint spherePointB;
	f32 distance = boxSphereDistance(testNormal, boxPointA, spherePointB, primA.getBox(), primTransformA, primB.getSphere(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, boxPointA, relTransformA, primIndexA, spherePointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsCapsuleBox(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	CapsulePoint capsulePointA;
	BoxPoint boxPointB;
	f32 distance = boxCapsuleDistance(testNormal, boxPointB, capsulePointA, primB.getBox(), primTransformB, primA.getCapsule(), primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, capsulePointA, relTransformA, primIndexA, boxPointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsCapsuleCapsule(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	CapsulePoint capsulePointA;
	CapsulePoint capsulePointB;
	f32 distance = capsuleCapsuleDistance(testNormal, capsulePointA, capsulePointB, primA.getCapsule(), primTransformA, primB.getCapsule(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, capsulePointA, relTransformA, primIndexA, capsulePointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsCapsuleSphere(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	CapsulePoint capsulePointA;
	SpherePoint spherePointB;
	f32 distance = capsuleSphereDistance(testNormal, capsulePointA, spherePointB, primA.getCapsule(), primTransformA, primB.getSphere(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, capsulePointA, relTransformA, primIndexA, spherePointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsSphereBox(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB,int primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal;
	SpherePoint spherePointA;
	BoxPoint boxPointB;
	f32 distance = boxSphereDistance(testNormal, boxPointB, spherePointA, primB.getBox(), primTransformB, primA.getSphere(), primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, spherePointA, relTransformA, primIndexA, boxPointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsSphereCapsule(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal;
	SpherePoint spherePointA;
	CapsulePoint capsulePointB;
	f32 distance = capsuleSphereDistance(testNormal, capsulePointB, spherePointA, primB.getCapsule(), primTransformB, primA.getSphere(), primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, spherePointA, relTransformA, primIndexA, capsulePointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsSphereSphere(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	Vector3 testNormal(0.0f);
	SpherePoint spherePointA;
	SpherePoint spherePointB;
	f32 distance = sphereSphereDistance(testNormal, spherePointA, spherePointB, primA.getSphere(), primTransformA, primB.getSphere(), primTransformB);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, spherePointA, relTransformA, primIndexA, spherePointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

///////////////////////////////////////////////////////////////////////////////
// Contact Filter

inline bool isCollidablePrims(const CollPrim& primA, const CollPrim& primB)
{
	return (primA.getContactFilterSelf()&primB.getContactFilterTarget()) &&
		   (primA.getContactFilterTarget()&primB.getContactFilterSelf());
}

///////////////////////////////////////////////////////////////////////////////
// closestContact

bool closestContact(ContactPair& contactPair, const CollObject& objA, const Transform3& transformA, const CollObject& objB, const Transform3& transformB, f32 objsInContactDist)
{
	contactPair.numContacts = 0;
	{
		PrimIterator itrPrimA(objA);
		for(s32 i=0;i < objA.getNumPrims();i++, ++itrPrimA) {
			const CollPrim& primA = *itrPrimA;
			Transform3 relTransformA = primA.getObjectRelTransform();
			Transform3 primTransformA = transformA*relTransformA;

			PrimIterator itrPrimB(objB);
			for(s32 j=0;j < objB.getNumPrims();j++, ++itrPrimB) {
				const CollPrim& primB = *itrPrimB;

				if(isCollidablePrims(primA, primB)) {
					Transform3 relTransformB = primB.getObjectRelTransform();
					Transform3 primTransformB = transformB*relTransformB;

					ContactPoint cp[NUMCONTACTS_PER_BODIES];
					s32 newContacts = funcTbl_primContacts[primA.getType()][primB.getType()](cp, primA, primTransformA, relTransformA, i, primB, primTransformB, relTransformB, j, objsInContactDist);

					for(s32 k=0;k < newContacts;k++) {
						s32 replaceId = findNearestContactPoint(contactPair.contactPoints, contactPair.numContacts, cp[k].getLocalPointA() ,cp[k].getNormal());

						if(replaceId < 0) {
							if(contactPair.numContacts < 4)
								replaceId = contactPair.numContacts++;
							else
								replaceId = sort4ContactPoints(contactPair.contactPoints, cp[k].getLocalPointA(), cp[k].distance);
						}

						contactPair.contactPoints[replaceId] = cp[k];
					}
				}
			}
		}
	}

	return (contactPair.numContacts > 0);
}

///////////////////////////////////////////////////////////////////////////////
// closestContactCCD

#define CCD_STACK_COUNT 10

struct CcdStackData
{
	f32 t0;
	f32 t1;
	s32 flag;

	CcdStackData(f32 t0_, f32 t1_, s32 f)
	{
		t0 = t0_;
		t1 = t1_;
		flag = f;
	}

	CcdStackData()
	{
		t0 = t1 = 0.0f;
		flag = 0;
	}
};

s32 testCcdPrim(ContactPair& contactPair, const CollObject& objA, const Transform3& tA0, const Transform3& tA1, const CollObject& objB, const Transform3& tB0, const Transform3& tB1, f32 objsInContactDist)
{
	CcdStackData lastToi;
	s32 loopCount = 0;

	Vector3 distA = tA1.getTranslation() - tA0.getTranslation();
	Vector3 distB = tB1.getTranslation() - tB0.getTranslation();
	Vector3 dir = distA - distB;
	f32 len = length(dir);
	f32 rA = objA.getCcdRadius();

	{
		ContactPoint cp[NUMCONTACTS_PER_BODIES];

		SimpleStack<CcdStackData> cs;

		cs.push(CcdStackData(0.0f, 1.0f, 0));

		s32 collisionCount = 0;

		bool collide = false;

		do {
			CcdStackData sd = cs.pop();

			f32 mid = 0.5f*(sd.t0 + sd.t1);
			f32 r = (mid - sd.t0)*len + rA;

			CollPrim sphere;
			sphere.setSphere(Sphere(r));
			sphere.setObjectRelTransform(Transform3::identity());

			Transform3 trA = Transform3::translation(mid*dir)*tA0;
			collisionCount = 0;

			{
				PrimIterator itrPrim(objB);
				for(s32 j=0;j < objB.getNumPrims();j++, ++itrPrim) {
					const CollPrim& primB = *itrPrim;
					Transform3 relTransformB = primB.getObjectRelTransform();
					Transform3 primTransformB = tB0*relTransformB;

					collisionCount += funcTbl_primContacts[sphere.getType()][primB.getType()](cp, sphere, trA, Transform3::identity(), 0, primB, primTransformB, relTransformB, j, objsInContactDist);
				}
			}

			if(collisionCount == 0)
				continue;

			loopCount++;

			if(loopCount > 1) {
				collide = true;
				lastToi = sd;
			}

			if((sd.t1 - sd.t0) < CCD_THRESHOLD_MIN)
				break;

			if(sd.flag == 1)
				cs.pop();

			if(cs.getStackCount() < CCD_STACK_COUNT - 1) {
				cs.push(CcdStackData(mid, sd.t1, 0));
				cs.push(CcdStackData(sd. t0,mid, 1));
			}
		} while(!cs.isEmpty());

		if(!collide || (lastToi.t1 - lastToi.t0) > CCD_THRESHOLD_MAX) return 0;
	}

	f32 mid = 0.5f*(lastToi.t0 + lastToi.t1);
	{
		Transform3 trA = interpTransform(mid, tA0, tA1);
		closestContact(contactPair, objA, trA, objB, tB0, objsInContactDist);
	}

	if(contactPair.numContacts == 0) {
		ContactPoint cp[NUMCONTACTS_PER_BODIES];

		f32 r = (mid - lastToi.t0)*len + rA;

		CollPrim sphere;
		sphere.setSphere(Sphere(r));
		sphere.setObjectRelTransform(Transform3::identity());

		Transform3 trA = Transform3::translation(mid*dir)*tA0;
		contactPair.numContacts = 0;

		PrimIterator itrPrim(objB);
		for(s32 j=0;j < objB.getNumPrims();j++, ++itrPrim) {
			const CollPrim& primB = *itrPrim;

			Transform3 relTransformB = primB.getObjectRelTransform();
			Transform3 primTransformB = tB0*relTransformB;

			s32 newContacts = funcTbl_primContacts[sphere.getType()][primB.getType()](cp, sphere, trA, Transform3::identity(), 0, primB, primTransformB, relTransformB, j, objsInContactDist);

			for(s32 k=0;k < newContacts;k++) {
				s32 replaceId = findNearestContactPoint(contactPair.contactPoints, contactPair.numContacts, cp[k].getLocalPointA(), cp[k].getNormal());

				if(replaceId < 0) {
					if(contactPair.numContacts < 4)
						replaceId = contactPair.numContacts++;
					else
						replaceId = sort4ContactPoints(contactPair.contactPoints, cp[k].getLocalPointA(), cp[k].distance);
				}

				contactPair.contactPoints[replaceId] = cp[k];
			}
		}
	}

	return contactPair.numContacts;
}

extern bool triangleSphereContact(ContactCache& contacts,u32 facetId, const Vector3& normal, const Vector3& p0, const Vector3& p1,const Vector3& p2, const f32 thickness, const f32 angle0, const f32 angle1, const f32 angle2, u32 edgeChk, f32 sphereRadius, const Vector3& spherePos);

// A:Primitive (Dynamic)
// B:LargeMesh (Static)
s32 testCcdLargeMesh(ContactPair& contactPair, const CollObject& objA, const Transform3& tA0, const Transform3& tA1, const CollObject& objB, const Transform3& tB0, const Transform3& tB1, f32 objsInContactDist)
{
	(void) objsInContactDist;
	(void) tB1;

	contactPair.numContacts = 0;

#ifdef __SPU__
	LargeTriMesh largeMesh_;
	LargeTriMesh *largeMesh = &largeMesh_;
	objB.getDefPrim().getLargeMeshNB(largeMesh, 0);
#else
	LargeTriMesh *largeMesh = objB.getDefPrim().getLargeMesh();
#endif

	Transform3 transformB, transformBA0, transformBA1;

	transformB = tB0*objB.getDefPrim().getObjectRelTransform();
	transformBA0 = orthoInverse(transformB)*tA0;
	transformBA1 = orthoInverse(transformB)*tA1;

	Vector3 dir = transformBA1.getTranslation() - transformBA0.getTranslation();
	f32 len = length(dir);
	f32 rA = objA.getCcdRadius();

#ifdef __SPU__
	spu_dma_wait_tag_status_all(1);
#endif

	VecInt3 aabbMinL = largeMesh->getLocalPosition(minPerElem(transformBA0.getTranslation(), transformBA1.getTranslation()) - Vector3(rA));
	VecInt3 aabbMaxL = largeMesh->getLocalPosition(maxPerElem(transformBA0.getTranslation(), transformBA1.getTranslation()) + Vector3(rA));

	{
#ifdef __SPU__
		vec_uint4 ptn_mask0 = ((vec_uint4){0x02031213, 0x06071617, 0x0A0B1A1B, 0x80808080});
		vec_ushort8 vecAabbA;
		vecAabbA = spu_shuffle((vec_ushort8)aabbMinL.get128(), (vec_ushort8)aabbMaxL.get128(), (vec_uchar16)ptn_mask0);

		ReadOnlyPrefetchForwardIterator<AABB16> itrAABB(
			&gPool,
			(u32)largeMesh->aabbList,
			(u32)(largeMesh->aabbList + largeMesh->numIslands),
			AABB_PREFETCH_NUM, 10);

		for(u32 i=0;i < largeMesh->numIslands;i++, ++itrAABB) {
			AABB16 aabbB = *itrAABB;

			vec_ushort8 vecAabbB = (vec_ushort8)aabbB;
			vec_uint4 ptn_mask1 = ((vec_uint4){0x10110001, 0x14150405, 0x18190809, 0x80808080});
			vec_ushort8 vecMin = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask1);
			vec_ushort8 vecMax = spu_shuffle(vecAabbA, vecAabbB, (vec_uchar16)ptn_mask0);
			vec_ushort8 isGt = spu_cmpgt(vecMin, vecMax);
			if(spu_extract(spu_gather(isGt), 0) > 0) continue;

			TriMesh *island = (TriMesh*)gPool.allocate(sizeof(TriMesh), HeapManager::ALIGN16);
			spu_dma_get(island, (u64)(largeMesh->islands + i), sizeof(TriMesh), 3, 0, 0);
			spu_dma_wait_tag_status_all(1<<3);
#else
		for(u32 i=0;i < largeMesh->numIslands;i++) {
			AABB16 aabbB = largeMesh->aabbList[i];
			if(aabbMaxL.getX() < getXMin(aabbB) || aabbMinL.getX() > getXMax(aabbB)) continue;
			if(aabbMaxL.getY() < getYMin(aabbB) || aabbMinL.getY() > getYMax(aabbB)) continue;
			if(aabbMaxL.getZ() < getZMin(aabbB) || aabbMinL.getZ() > getZMax(aabbB)) continue;

			TriMesh *island = &largeMesh->islands[i];
#endif

			for(u32 f=0;f < island->numFacets;f++) {
				const MeshFacet& facet = island->facets[f];

				const Vector3 facetNormal = read_Vector3(facet.normal);

				const Vector3 facetPnts[3] =
				{
					island->verts[facet.vertIndices[0]],
					island->verts[facet.vertIndices[1]],
					island->verts[facet.vertIndices[2]]
				};

				const MeshEdge *edge[3] =
				{
					&island->edges[facet.edgeIndices[0]],
					&island->edges[facet.edgeIndices[1]],
					&island->edges[facet.edgeIndices[2]]
				};

				u32 edgeChk = ((edge[0]->angleType == EDGE_CONVEX) ? 0x00 : 0x01) |
							  ((edge[1]->angleType == EDGE_CONVEX) ? 0x00 : 0x02) |
							  ((edge[2]->angleType == EDGE_CONVEX) ? 0x00 : 0x04);

				CcdStackData lastToi;
				s32 loopCount = 0;
				bool collide = false;

				SimpleStack<CcdStackData> cs;

				cs.push(CcdStackData(0.0f, 1.0f, 0));

				do {
					CcdStackData sd = cs.pop();

					f32 mid = 0.5f*(sd.t0 + sd.t1);
					f32 r = (mid - sd.t0)*len + rA;
					Vector3 posA = mid*dir + transformBA0.getTranslation();
					Vector3 pntOnA(0.0f);

					distancePointAndTriangle(facetPnts[0], facetPnts[1], facetPnts[2], posA, pntOnA);

					if(lengthSqr(pntOnA - posA) > r*r)
						continue;

					loopCount++;

					if(loopCount > 1) {
						collide = true;
						lastToi = sd;
					}

					if((sd.t1 - sd.t0) < CCD_THRESHOLD_MIN)
						break;

					if(sd.flag == 1)
						cs.pop();

					if(cs.getStackCount() < CCD_STACK_COUNT - 1) {
						cs.push(CcdStackData(mid, sd.t1, 0));
						cs.push(CcdStackData(sd.t0, mid, 1));
					}
				} while(!cs.isEmpty());

				if(!collide || (lastToi.t1 - lastToi.t0) > CCD_THRESHOLD_MAX) continue;

				f32 mid = 0.5f*(lastToi.t0 + lastToi.t1);
				{
					f32 r = (mid - lastToi.t0)*len + rA;

					Transform3 trA = Transform3::translation(mid*dir)*transformBA0;

					ContactCache contacts;

					if(triangleSphereContact(contacts, f, facetNormal, facetPnts[0], facetPnts[1], facetPnts[2], facet.thickness, 0.5f*PI*(edge[0]->tilt/255.0f), 0.5f*PI*(edge[1]->tilt/255.0f), 0.5f*PI*(edge[2]->tilt/255.0f), edgeChk, r, trA.getTranslation())) {
						Vector3 pntA = contacts.getPointB(0);
						Vector3 pntB = contacts.getPointA(0);
						Vector3 nml = contacts.getNormal(0);

						ContactPoint cp;

						f32 s, t;
						get_ST(s, t, facetPnts[1] - facetPnts[0], facetPnts[2] - facetPnts[0], pntB - facetPnts[0]);
						SubData sub;
						sub.type = SubData::SubDataFacetLocal;
						sub.setIslandIndex(i);
						sub.setFacetIndex(f);
						sub.setFacetLocalS(s);
						sub.setFacetLocalT(t);

						cp.distance = contacts.getDistance(0);
						cp.setNormal(-(transformB.getUpper3x3()*nml));
						Point3 pA(orthoInverse(trA)*Point3(pntA));
						Point3 pB(pntB);
						cp.setA(pA, Transform3::identity(), 0);
						cp.setB(pB, objB.getDefPrim().getObjectRelTransform(), 0);
						cp.subData = sub;

						s32 replaceId = findNearestContactPoint(contactPair.contactPoints, contactPair.numContacts, cp.getLocalPointA(), cp.getNormal());

						if(replaceId < 0) {
							if(contactPair.numContacts < 4)
								replaceId = contactPair.numContacts++;
							else
								replaceId = sort4ContactPoints(contactPair.contactPoints, cp.getLocalPointA(), cp.distance);
						}

						contactPair.contactPoints[replaceId] = cp;
					}
				}
			}
#ifdef __SPU__
			gPool.deallocate(island);
#endif
		}
	}

	return contactPair.numContacts;
}

u32 findContactCCD(ContactPair& contactPair, const CollObject& objA,const Transform3& tA0, const Transform3& tA1, bool useCcdA, const CollObject& objB, const Transform3& tB0, const Transform3& tB1, bool useCcdB, f32 objsInContactDist)
{
	s32 count = 0;

	Vector3 distA = tA1.getTranslation() - tA0.getTranslation();
	Vector3 distB = tB1.getTranslation() - tB0.getTranslation();

	Vector3 relativeDistance = distA - distB;
	f32 d = length(relativeDistance);

	f32 checkDA = useCcdA ? CCD_ENABLE_DISTANCE*objA.getCcdRadius() : 0.0f;
	f32 checkDB = useCcdB ? CCD_ENABLE_DISTANCE*objB.getCcdRadius() : 0.0f;

	if((useCcdA && d > checkDA) || (useCcdB && d > checkDB)) {
		if(objA.getDefPrim().getType() == LARGEMESH) {
			count = testCcdLargeMesh(contactPair, objB, tB0, tB1, objA, tA0, tA1, objsInContactDist);
			for(s32 i=0;i < contactPair.numContacts;i++)
				contactPair.contactPoints[i].exchange();
		} else if(objB.getDefPrim().getType() == LARGEMESH)
			count = testCcdLargeMesh(contactPair, objA, tA0, tA1, objB, tB0, tB1, objsInContactDist);
		else
			count = testCcdPrim(contactPair, objA, tA0, tA1, objB, tB0, tB1, objsInContactDist);
	} else
		return closestContact(contactPair, objA, tA0, objB, tB0, objsInContactDist);

	(void) count;

	return (contactPair.numContacts > 0);
}
