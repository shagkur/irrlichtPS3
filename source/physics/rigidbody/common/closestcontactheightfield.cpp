/*
 * closestcontactheightfield.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/contact.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/contactheightfield.h"
#include "rigidbody/common/heightfieldfunction.h"

#ifdef __SPU__
	#include "base/heapmanager.h"
	extern HeapManager gPool;
#endif

#define SET_CONTACT_POINT(contactPoint, dist, nml, pntA, trnsA, primA, pntB, trnsB, primB) \
	contactPoint.distance = dist;\
	contactPoint.setNormal(nml);\
	contactPoint.setA(pntA, trnsA, primA);\
	contactPoint.setB(pntB, trnsB, primB);\
	contactPoint.subData.type = SubData::SubDataNone;

s32 primContactsShapeHeightField(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
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

	if(primA.getType() == SPHERE) {
		Sphere sphere = primA.getSphere();
		Point3 spherePoint = transformBA.getTranslation() + Point3(0.0f, -sphere.radius, 0.0f);
		if(contactHeightField(heightfield, spherePoint, cpB, nml, distance) && distance < cp[0].distance) {
			Point3 cpA = transformAB*spherePoint;
			SET_CONTACT_POINT(cp[0], distance, primTransformB.getUpper3x3()*nml, cpA, relTransformA, primIndexA, cpB, relTransformB, primIndexB);
			ret = true;
		}
	}
	else if(primA.getType() == BOX) {
		Box box = primA.getBox();
		Point3 boxPoint[8];
		boxPoint[0] = (Point3)box.half;
		boxPoint[1] = mulPerElem(Point3(-1.0f, 1.0f, 1.0f), (Point3)box.half);
		boxPoint[2] = mulPerElem(Point3(-1.0f, 1.0f,-1.0f), (Point3)box.half);
		boxPoint[3] = mulPerElem(Point3( 1.0f, 1.0f,-1.0f), (Point3)box.half);
		boxPoint[4] = mulPerElem(Point3( 1.0f,-1.0f, 1.0f), (Point3)box.half);
		boxPoint[5] = mulPerElem(Point3(-1.0f,-1.0f, 1.0f), (Point3)box.half);
		boxPoint[6] = mulPerElem(Point3(-1.0f,-1.0f,-1.0f), (Point3)box.half);
		boxPoint[7] = mulPerElem(Point3( 1.0f,-1.0f,-1.0f), (Point3)box.half);
		for(s32 i=0;i < 8;i++) {
			if(contactHeightField(heightfield, transformBA*boxPoint[i], cpB, nml, distance) && distance < cp[0].distance) {
				Point3 cpA = boxPoint[i];
				SET_CONTACT_POINT(cp[0], distance,primTransformB.getUpper3x3()*nml, cpA, relTransformA, primIndexA, cpB, relTransformB, primIndexB);
				ret = true;
			}
		}
	}
	else if(primA.getType() == CAPSULE) {
		Capsule capsule = primA.getCapsule();
		Point3 capsulePoint[2];
		capsulePoint[0] = transformBA*Point3(-capsule.hLength, 0.0f, 0.0f) + Vector3(0.0f, -capsule.radius, 0.0f);
		capsulePoint[1] = transformBA*Point3( capsule.hLength, 0.0f, 0.0f) + Vector3(0.0f, -capsule.radius, 0.0f);
		for(s32 i=0;i < 2;i++) {
			if(contactHeightField(heightfield, capsulePoint[i], cpB, nml, distance) && distance < cp[0].distance) {
				Point3 cpA = transformAB*capsulePoint[i];
				SET_CONTACT_POINT(cp[0], distance, primTransformB.getUpper3x3()*nml, cpA, relTransformA, primIndexA, cpB, relTransformB, primIndexB);
				ret = true;
			}
		}
	}

#ifdef __SPU__
	releaseHeightFieldCache();
#endif

	return ret ? 1 : 0;
}

s32 primContactsHeightFieldShape(ContactPoint *cp, const CollPrim& primA, const Transform3& primTransformA, const Transform3& relTransformA, s32 primIndexA, const CollPrim& primB, const Transform3& primTransformB, const Transform3& relTransformB, s32 primIndexB, f32 objsInContactDist)
{
	(void) primTransformA;
	(void) objsInContactDist;

	Point3 cpA(0.0f);
	Vector3 nml(0.0f);
	f32 distance = 0.0f;
	bool ret = false;

	HeightField *heightfield;

#ifdef __SPU__
	initializeHeightFieldCache();

	HeightField heightfield_;
	primA.getHeightField(&heightfield_);
	heightfield = &heightfield_;
#else
	heightfield = primA.getHeightField();
#endif

	Transform3 transformBA = orthoInverse(primTransformB)*primTransformA;
	Transform3 transformAB = orthoInverse(primTransformA)*primTransformB;

	if(primB.getType() == SPHERE) {
		Sphere sphere = primB.getSphere();
		Point3 spherePoint = transformAB.getTranslation() + Point3(0.0f, -sphere.radius, 0.0f);
		if(contactHeightField(heightfield, spherePoint, cpA, nml, distance) && distance < cp[0].distance) {
			Point3 cpB = transformBA*spherePoint;
			SET_CONTACT_POINT(cp[0], distance, -primTransformA.getUpper3x3()*nml, cpA, relTransformA, primIndexA, cpB, relTransformB, primIndexB);
			ret = true;
		}
	}
	else if(primB.getType() == BOX) {
		Box box = primB.getBox();
		Point3 boxPoint[8];
		boxPoint[0] = (Point3)box.half;
		boxPoint[1] = mulPerElem(Point3(-1.0f, 1.0f, 1.0f), (Point3)box.half);
		boxPoint[2] = mulPerElem(Point3(-1.0f, 1.0f,-1.0f), (Point3)box.half);
		boxPoint[3] = mulPerElem(Point3( 1.0f, 1.0f,-1.0f), (Point3)box.half);
		boxPoint[4] = mulPerElem(Point3( 1.0f,-1.0f, 1.0f), (Point3)box.half);
		boxPoint[5] = mulPerElem(Point3(-1.0f,-1.0f, 1.0f), (Point3)box.half);
		boxPoint[6] = mulPerElem(Point3(-1.0f,-1.0f,-1.0f), (Point3)box.half);
		boxPoint[7] = mulPerElem(Point3( 1.0f,-1.0f,-1.0f), (Point3)box.half);
		for(s32 i=0;i < 8;i++) {
			if(contactHeightField(heightfield, transformAB*boxPoint[i], cpA, nml, distance) && distance < cp[0].distance) {
				Point3 cpB = boxPoint[i];
				SET_CONTACT_POINT(cp[0], distance, -primTransformA.getUpper3x3()*nml, cpA, relTransformA, primIndexA, cpB, relTransformB, primIndexB);
				ret = true;
			}
		}
	}
	else if(primB.getType() == CAPSULE) {
		Capsule capsule = primB.getCapsule();
		Point3 capsulePoint[2];
		capsulePoint[0] = transformAB*Point3(-capsule.hLength, 0.0f, 0.0f) + Vector3(0.0f, -capsule.radius, 0.0f);
		capsulePoint[1] = transformAB*Point3( capsule.hLength, 0.0f, 0.0f) + Vector3(0.0f, -capsule.radius, 0.0f);
		for(s32 i=0;i < 2;i++) {
			if(contactHeightField(heightfield, capsulePoint[i], cpA, nml, distance) && distance < cp[0].distance) {
				Point3 cpB = transformBA*capsulePoint[i];
				SET_CONTACT_POINT(cp[0], distance, -primTransformA.getUpper3x3()*nml, cpA, relTransformA, primIndexA, cpB, relTransformB, primIndexB);
				ret = true;
			}
		}
	}

#ifdef __SPU__
	releaseHeightFieldCache();
#endif

	return ret?1:0;
}
