/*
 * trbstatevec.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef TRBSTATEVEC_H_
#define TRBSTATEVEC_H_

#include "base/common.h"
#include "base/simdfunc.h"

// Move Type
enum {
	MoveTypeFixed    = 0,
	MoveTypeActive   = 1,
	MoveTypeKeyframe = 3,
	MoveTypeOneWay   = 5,
	MoveTypeTrigger  = 7,
	MoveTypeCount
};

#define MOVE_TYPE_CAN_SLEEP		((1<<MoveTypeActive)|(1<<MoveTypeKeyframe)|(1<<MoveTypeOneWay))
#define MOVE_TYPE_DYNAMIC		((1<<MoveTypeActive)|(1<<MoveTypeOneWay))
#define MOVE_TYPE_NODYNAMIC		~MOVE_TYPE_DYNAMIC

//
// Rigid Body state
//

ATTRIBUTE_ALIGNED128(class) TrbState
{
public:
	TrbState()
	{
		setMoveType(MoveTypeActive);
		contactFilterSelf = contactFilterTarget = 0xffffffff;
		deleted = 0;
		sleeping = 0;
		useSleep = 1;
		trbBodyIdx = 0;
		sleepCount = 0;
		useCcd = 0;
		useContactCallback = 0;
		useSleepCallback = 0;
		linearDamping = 1.0f;
		angularDamping = 0.99f;
	}

	TrbState(const u8 m, const Vector3 x, const Quat q, const Vector3 v, const Vector3 omega);

	u16	sleepCount;
	u8 moveType;
	u8 deleted            : 1;
	u8 sleeping           : 1;
	u8 useSleep           : 1;
	u8 useCcd		      : 1;
	u8 useContactCallback : 1;
	u8 useSleepCallback   : 1;

	u16	trbBodyIdx;
	u32	contactFilterSelf;
	u32	contactFilterTarget;

	f32	center[3];		// AABB center(World)
	f32	half[3];		// AABB half(World)

	f32	linearDamping;
	f32	angularDamping;

	f32	deltaLinearVelocity[3];
	f32	deltaAngularVelocity[3];

	f32 fX[3];				// position
	f32 fQ[4];				// orientation
	f32 fV[3];				// velocity
	f32 fOmega[3];			// angular velocity

	inline void setZero();      // Zeroes out the elements
	inline void setIdentity();  // Sets the rotation to identity and zeroes out the other elements

	bool isDeleted() const {return deleted == 1;}

	u32	getContactFilterSelf() const {return contactFilterSelf;}
	void setContactFilterSelf(u32 filter) {contactFilterSelf = filter;}

	u32	getContactFilterTarget() const {return contactFilterTarget;}
	void setContactFilterTarget(u32 filter) {contactFilterTarget = filter;}

	u8 getMoveType() const {return moveType;}
	void setMoveType(u8 t) {moveType = t; sleeping = 0; sleepCount = 0;}

	u8 getMoveTypeBits() const {return (1<<moveType)|(1<<(moveType + sleeping));}

	bool isAsleep() const {return sleeping == 1;}
	bool isAwake() const {return sleeping == 0;}

	void wakeup() {sleeping = 0; sleepCount = 0;}
	void sleep() {if(useSleep) {sleeping = 1; sleepCount = 0;}}

	u8 getUseSleep() const {return useSleep;}
	void setUseSleep(u8 b) {useSleep = b;}

	u8 getUseCcd() const {return useCcd;}
	void setUseCcd(u8 b) {useCcd = b;}

	u8 getUseContactCallback() const {return useContactCallback;}
	void setUseContactCallback(u8 b) {useContactCallback = b;}

	u8 getUseSleepCallback() const {return useSleepCallback;}
	void setUseSleepCallback(u8 b) {useSleepCallback = b;}

	void incrementSleepCount() {sleepCount++;}
	void resetSleepCount() {sleepCount = 0;}
	u16	getSleepCount() const {return sleepCount;}

	Vector3 getPosition() const {return read_Vector3(fX);}
	Quat getOrientation() const {return read_Quat(fQ);}
	Vector3 getLinearVelocity() const {return read_Vector3(fV);}
	Vector3 getAngularVelocity() const {return read_Vector3(fOmega);}
	Vector3 getDeltaLinearVelocity() const {return read_Vector3(deltaLinearVelocity);}
	Vector3 getDeltaAngularVelocity() const {return read_Vector3(deltaAngularVelocity);}

	void setPosition(const Vector3& pos) {store_Vector3(pos, fX);}
	void setLinearVelocity(const Vector3& vel) {store_Vector3(vel, fV);}
	void setAngularVelocity(const Vector3& vel) {store_Vector3(vel, fOmega);}
	void setDeltaLinearVelocity(const Vector3& vel) {store_Vector3(vel, deltaLinearVelocity);}
	void setDeltaAngularVelocity(const Vector3& vel) {store_Vector3(vel, deltaAngularVelocity);}
	void setOrientation(const Quat& rot) {store_Quat(rot, fQ);}

	inline void setAuxils(const Vector3& centerLocal, const Vector3& halfLocal);
	inline void	setAuxilsCcd(const Vector3& centerLocal, const Vector3& halfLocal, f32 timeStep);
};

inline TrbState::TrbState(const u8 m, const Vector3 x, const Quat q, const Vector3 v, const Vector3 omega)
{
	setMoveType(m);
	fX[0] = x[0];
	fX[1] = x[1];
	fX[2] = x[2];
	fQ[0] = q[0];
	fQ[1] = q[1];
	fQ[2] = q[2];
	fQ[3] = q[3];
	fV[0] = v[0];
	fV[1] = v[1];
	fV[2] = v[2];
	fOmega[0] = omega[0];
	fOmega[1] = omega[1];
	fOmega[2] = omega[2];
	contactFilterSelf = contactFilterTarget = 0xffff;
	trbBodyIdx = 0;
	sleeping = 0;
	deleted = 0;
	useSleep = 1;
	useCcd = 0;
	useContactCallback = 0;
	useSleepCallback = 0;
	sleepCount = 0;
	linearDamping = 1.0f;
	angularDamping = 0.99f;
}

inline void TrbState::setIdentity()
{
	fX[0] = 0.0f;
	fX[1] = 0.0f;
	fX[2] = 0.0f;
	fQ[0] = 0.0f;
	fQ[1] = 0.0f;
	fQ[2] = 0.0f;
	fQ[3] = 1.0f;
	fV[0] = 0.0f;
	fV[1] = 0.0f;
	fV[2] = 0.0f;
	fOmega[0] = 0.0f;
	fOmega[1] = 0.0f;
	fOmega[2] = 0.0f;
}

inline void TrbState::setZero()
{
	fX[0] = 0.0f;
	fX[1] = 0.0f;
	fX[2] = 0.0f;
	fQ[0] = 0.0f;
	fQ[1] = 0.0f;
	fQ[2] = 0.0f;
	fQ[3] = 0.0f;
	fV[0] = 0.0f;
	fV[1] = 0.0f;
	fV[2] = 0.0f;
	fOmega[0] = 0.0f;
	fOmega[1] = 0.0f;
	fOmega[2] = 0.0f;
}

inline void TrbState::setAuxils(const Vector3& centerLocal, const Vector3& halfLocal)
{
	Vector3 centerW = getPosition() + rotate(getOrientation(), centerLocal);
	Vector3 halfW = absPerElem(Matrix3(getOrientation()))*halfLocal;
	center[0] = centerW[0];
	center[1] = centerW[1];
	center[2] = centerW[2];
	half[0] = halfW[0];
	half[1] = halfW[1];
	half[2] = halfW[2];
}

inline void TrbState::setAuxilsCcd(const Vector3& centerLocal, const Vector3& halfLocal, f32 timeStep)
{
	Vector3 centerW = getPosition() + rotate(getOrientation(), centerLocal);
	Vector3 halfW = absPerElem(Matrix3(getOrientation()))*halfLocal;

	Vector3 diffvec = getLinearVelocity()*timeStep;

	Vector3 newCenter = centerW + diffvec;
	Vector3 aabbMin = minPerElem(newCenter - halfW, centerW - halfW);
	Vector3 aabbMax = maxPerElem(newCenter + halfW, centerW + halfW);

	centerW = 0.5f*(aabbMin + aabbMax);
	halfW = 0.5f*(aabbMax - aabbMin);

	center[0] = centerW[0];
	center[1] = centerW[1];
	center[2] = centerW[2];

	half[0] = halfW[0];
	half[1] = halfW[1];
	half[2] = halfW[2];
}

#endif /* TRBSTATEVEC_H_ */
