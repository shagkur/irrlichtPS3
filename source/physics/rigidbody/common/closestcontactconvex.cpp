/*
 * closestcontactconvex.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"

#include "rigidbody/common/contact.h"
#include "rigidbody/common/collobject.h"

#include "rigidbody/common/convexspheredistance.h"
#include "rigidbody/common/convexboxdistance.h"
#include "rigidbody/common/convexcapsuledistance.h"
#include "rigidbody/common/convexconvexdistance.h"
#include "rigidbody/common/contactheightfield.h"
#include "rigidbody/common/heightfieldfunction.h"

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

class CachedConvexMesh
{
public:
	ConvexMesh *mesh;

	CachedConvexMesh(const CollPrim &prim)
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

///////////////////////////////////////////////////////////////////////////////

s32 primContactsSphereConvex(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA, pointB;

	CachedConvexMesh meshB(primB);

	f32 distance = closestConvexSphere(testNormal, pointB, pointA, meshB.getMesh(), primTransformB, primA.getSphere(), primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, pointA, relTransformA, primIndexA, pointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsConvexSphere(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA, pointB;

	CachedConvexMesh meshA(primA);

	f32 distance = closestConvexSphere(testNormal, pointA, pointB, meshA.getMesh(), primTransformA, primB.getSphere(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, pointA, relTransformA, primIndexA, pointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsBoxConvex(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA, pointB;

	CachedConvexMesh meshB(primB);

	f32 distance = closestConvexBox(testNormal, pointB, pointA, meshB.getMesh(), primTransformB, primA.getBox(), primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, pointA, relTransformA, primIndexA, pointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsConvexBox(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA, pointB;

	CachedConvexMesh meshA(primA);

	f32 distance = closestConvexBox(testNormal, pointA, pointB, meshA.getMesh(), primTransformA, primB.getBox(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, pointA, relTransformA, primIndexA, pointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsCapsuleConvex(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA, pointB;

	CachedConvexMesh meshB(primB);

	f32 distance = closestConvexCapsule(testNormal, pointB, pointA, meshB.getMesh(), primTransformB, primA.getCapsule(), primTransformA, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, -testNormal, pointA, relTransformA, primIndexA, pointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsConvexCapsule(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA, pointB;

	CachedConvexMesh meshA(primA);

	f32 distance = closestConvexCapsule(testNormal, pointA, pointB, meshA.getMesh(), primTransformA, primB.getCapsule(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, pointA, relTransformA, primIndexA, pointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}

s32 primContactsConvexHeightField(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	(void) primTransformB;
	(void) objsInContactDist;

	bool ret =false;

	HeightField *heightfield;

	CachedConvexMesh meshA(primA);

#ifdef __SPU__
	initializeHeightFieldCache();

	HeightField heightfield_;
	primB.getHeightField(&heightfield_);
	heightfield = &heightfield_;
#else
	heightfield = primB.getHeightField();
#endif

	Transform3 transformBA = orthoInverse(primTransformB)*primTransformA;

	for(u32 i=0;i < meshA.getMesh()->numVerts;i++) {
		Point3 cpB(0.0f);
		Vector3 nml(0.0f);
		f32 dist = 0.0f;

		if (contactHeightField(heightfield, transformBA*Point3(meshA.getMesh()->verts[i]), cpB, nml, dist) && dist < cp[0].distance) {
			Point3 cpA(meshA.getMesh()->verts[i]);

			SET_CONTACT_POINT(cp[0], dist, primTransformB.getUpper3x3()*nml, cpA, relTransformA, primIndexA, cpB, relTransformB, primIndexB);
			ret = true;
		}
	}

#ifdef __SPU__
	releaseHeightFieldCache();
#endif

	return ret ? 1 : 0;
}

s32 primContactsHeightFieldConvex(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	(void) primTransformA;
	(void) objsInContactDist;

	bool ret =false;

	HeightField *heightfield;

	CachedConvexMesh meshB(primB);

#ifdef __SPU__
	initializeHeightFieldCache();

	HeightField heightfield_;
	primA.getHeightField(&heightfield_);
	heightfield = &heightfield_;
#else
	heightfield = primA.getHeightField();
#endif

	Transform3 transformAB = orthoInverse(primTransformA)*primTransformB;

	for(u32 i=0;i < meshB.getMesh()->numVerts;i++) {
		Point3 cpA(0.0f);
		Vector3 nml(0.0f);
		f32 dist = 0.0f;

		if(contactHeightField(heightfield, transformAB*Point3(meshB.getMesh()->verts[i]), cpA, nml, dist) && dist < cp[0].distance) {
			Point3 cpB(meshB.getMesh()->verts[i]);

			SET_CONTACT_POINT(cp[0], dist, -primTransformA.getUpper3x3()*nml, cpA, relTransformA, primIndexA, cpB, relTransformB, primIndexB);
			ret = true;
		}
	}

#ifdef __SPU__
	releaseHeightFieldCache();
#endif

	return ret ? 1 : 0;
}

s32 primContactsConvexConvex(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{

	Vector3 testNormal;
	Point3 pointA, pointB;

	CachedConvexMesh meshA(primA);
	CachedConvexMesh meshB(primB);

	f32 distance = closestConvexConvex(testNormal, pointA, pointB, meshA.getMesh(), primTransformA, meshB.getMesh(), primTransformB, objsInContactDist);

	if(distance < objsInContactDist) {
		SET_CONTACT_POINT(cp[0], distance, testNormal, pointA, relTransformA, primIndexA, pointB, relTransformB, primIndexB);
		return 1;
	}

	return 0;
}
