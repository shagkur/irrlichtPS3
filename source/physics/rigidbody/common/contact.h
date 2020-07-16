/*
 * contact.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CONTACT_H_
#define CONTACT_H_

#include "base/common.h"
#include "base/sortcommon.h"
#include "base/simdfunc.h"

#include "rigidbody/common/collobject.h"
#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/subdata.h"
#include "rigidbody/common/constraintcache.h"

#define NO_CONTACT_DISTANCE				999.0f
#define CONTACT_EPSILON					0.0f
#define SWAP(type, x, y) 				do {type t; t = x; x = y; y = t;} while (0)

///////////////////////////////////////////////////////////////////////////////
// Contact Point

struct ContactPoint
{
	ConstraintCache constraints[3];

	u8 duration;
	u8 primIdxA;
	u8 primIdxB;
	SubData subData;

	f32 distance;
	f32 localVelocityA[3];
	f32 localVelocityB[3];
	f32 localPointA[3];
	f32 localPointB[3];

	// --------------------------------------------

	ContactPoint()
	{
		reset();
	}

	void reset()
	{
#if defined(__SPU__) && defined(USE_DEEP_OPTIMIZE)
		ASSERT_MSG((s32)&constraints[0]%16 == 0);
		ASSERT_MSG(10*sizeof(vec_uint4) == sizeof(ContactPoint));
		vec_uint4 *pCP = (vec_uint4*)&constraints[0];
		vec_uint4 zero = ((vec_uint4){0, 0, 0, 0});
		*(pCP+0) = zero;
		*(pCP+1) = zero;
		*(pCP+2) = zero;
		*(pCP+3) = zero;
		*(pCP+4) = zero;
		*(pCP+5) = zero;
		*(pCP+6) = zero;
		*(pCP+7) = zero;
		*(pCP+8) = zero;
		*(pCP+9) = zero;
		distance = FLT_MAX;
#else
		distance = FLT_MAX;
		duration = 0;
		subData.type = 0;
		localVelocityA[0] = 0.0f;
		localVelocityA[1] = 0.0f;
		localVelocityA[2] = 0.0f;
		localVelocityB[0] = 0.0f;
		localVelocityB[1] = 0.0f;
		localVelocityB[2] = 0.0f;
		constraints[0].normal[0] = 0.0f;
		constraints[0].normal[1] = 0.0f;
		constraints[0].normal[2] = 0.0f;
		constraints[0].accumImpulse = 0.0f;
		constraints[1].accumImpulse = 0.0f;
		constraints[2].accumImpulse = 0.0f;
#endif
	}

	void exchange()
	{
		SWAP(f32, localPointA[0], localPointB[0]);
		SWAP(f32, localPointA[1], localPointB[1]);
		SWAP(f32, localPointA[2], localPointB[2]);
		SWAP(u8, primIdxA, primIdxB);
		SWAP(f32, localVelocityA[0], localVelocityB[0]);
		SWAP(f32, localVelocityA[1], localVelocityB[1]);
		SWAP(f32, localVelocityA[2], localVelocityB[2]);
		setNormal(-getNormal());
	}

	void setA(Point3& contactPoint, const Transform3& objectRelativeTransform, u8 primIdx_)
	{
		Vector3 p(objectRelativeTransform*contactPoint);
		setLocalPointA(p);
		primIdxA = primIdx_;
	}

	void setB(Point3& contactPoint, const Transform3& objectRelativeTransform, u8 primIdx_)
	{
		Vector3 p(objectRelativeTransform*contactPoint);
		setLocalPointB(p);
		primIdxB = primIdx_;
	}

	void setLocalPointA(const Vector3& p) {store_Vector3(p, localPointA);}
	void setLocalPointB(const Vector3& p) {store_Vector3(p, localPointB);}
	void setLocalVelocityA(const Vector3& v) {store_Vector3(v, localVelocityA);}
	void setLocalVelocityB(const Vector3& v) {store_Vector3(v, localVelocityB);}
	void setNormal(const Vector3& n) {store_Vector3(n, constraints[0].normal);}
	void setDistance(f32 d) {distance = d;}

	Vector3 getLocalPointA() const {return read_Vector3(localPointA);}
	Vector3 getLocalPointB() const {return read_Vector3(localPointB);}
	Vector3 getLocalVelocityA() const {return read_Vector3(localVelocityA);}
	Vector3 getLocalVelocityB() const {return read_Vector3(localVelocityB);}
	Vector3 getNormal() const {return read_Vector3(constraints[0].normal);}
	f32   getDistance() const {return distance;}

	Vector3 getTangent1() const {return read_Vector3(constraints[1].normal);}
	Vector3 getTangent2() const {return read_Vector3(constraints[2].normal);}

	Vector3 getWorldPointA(const Vector3& pos, const Quat& rot) const
	{
		return pos + rotate(rot,getLocalPointA());
	}

	Vector3 getWorldPointB(const Vector3& pos,const Quat& rot) const
	{
		return pos + rotate(rot,getLocalPointB());
	}

	f32 getMaxImpulse() const
	{
		return constraints[0].accumImpulse;
	}
};

///////////////////////////////////////////////////////////////////////////////
// Contact Pair (Contact Manifold)

ATTRIBUTE_ALIGNED128(struct) ContactPair
{
	u32 duration;
	u16 numContacts;
	u16 stateIndex[2];

	f32	compositeFriction;
	f32	massInvA,massInvB;
	f32	inertiaInvA[9];
	f32	inertiaInvB[9];

	ContactPoint contactPoints[NUMCONTACTS_PER_BODIES];

	// --------------------------------------------

	void reset()
	{
		numContacts = 0;
		duration = 0;
		for(s32 i=0;i < NUMCONTACTS_PER_BODIES;i++)
			contactPoints[i].reset();
	}

	void exchange()
	{
		for(s32 i=0;i < numContacts;i++)
			contactPoints[i].exchange();

		SWAP(u16, stateIndex[0], stateIndex[1]);
	}

	ContactPoint& getContactPoint(s32 idx) {return contactPoints[idx];}

	void setA(s32 idx, Point3& localPoint, const Transform3& objectRelativeTransform, u8 primIdx)
	{
		contactPoints[idx].setA(localPoint, objectRelativeTransform, primIdx);
	}

	void setB(s32 idx, Point3& localPoint, const Transform3& objectRelativeTransform, u8 primIdx)
	{
		contactPoints[idx].setB(localPoint, objectRelativeTransform, primIdx);
	}

	f32 getMassInvA() {return massInvA;}
	f32 getMassInvB() {return massInvB;}

	Matrix3 getInertiaInvA() {return Matrix3(read_Vector3(&inertiaInvA[0]), read_Vector3(&inertiaInvA[3]), read_Vector3(&inertiaInvA[6]));}
	Matrix3 getInertiaInvB() {return Matrix3(read_Vector3(&inertiaInvB[0]), read_Vector3(&inertiaInvB[3]), read_Vector3(&inertiaInvB[6]));}

	void setMassInvA(f32 massInv) {massInvA = massInv;}
	void setMassInvB(f32 massInv) {massInvB = massInv;}
	void setInertiaInvA(const Matrix3& inertiaInv)
	{
		store_Vector3(inertiaInv[0], &inertiaInvA[0]);
		store_Vector3(inertiaInv[1], &inertiaInvA[3]);
		store_Vector3(inertiaInv[2], &inertiaInvA[6]);
	}

	void setInertiaInvB(const Matrix3& inertiaInv)
	{
		store_Vector3(inertiaInv[0], &inertiaInvB[0]);
		store_Vector3(inertiaInv[1], &inertiaInvB[3]);
		store_Vector3(inertiaInv[2], &inertiaInvB[6]);
	}

	// --------------------------------------------

	u32 merge(ContactPair& contactPair);
	void refreshContactPoints(const Vector3& pA, const Quat& qA, const Vector3& pB, const Quat& qB);
	void removeContactPoint(s32 idx)
	{
		contactPoints[idx] = contactPoints[numContacts - 1];
		numContacts--;
	}
};

s32 findNearestContactPoint(ContactPoint *cp, s32 numContacts, const Vector3& newCP, const Vector3& newNml);
s32 sort4ContactPoints(ContactPoint *cp, const Vector3& newCP, f32 newDistance);

#endif /* CONTACT_H_ */
