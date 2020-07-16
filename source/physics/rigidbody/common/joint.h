/*
 * joint.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef JOINT_H_
#define JOINT_H_

#include "rigidbody/common/rigidbodyconfig.h"
#include "rigidbody/common/subdata.h"
#include "rigidbody/common/constraintcache.h"

// Joint Type
enum JointType {
	JointTypeBall = 0,
	JointTypeChain,
	JointTypeSlider,
	JointTypeHinge,
	JointTypeFix,
	JointTypeUniversal,
	JointTypeAnimation,
	JointTypeDistance,
	JointTypeCount
};

// Joint Axis
enum {
	JointAxisFree = 0x00,
	JointAxisLock = 0x01,
	JointAxisLimit = 0x02,
};

#define GET_AXIS_LOCK(flag, axisId)      		((flag>>(axisId<<2))&0x03)
#define SET_AXIS_LOCK(flag, axisId, lock) 		((flag&~(0x03<<(axisId<<2)))|(lock<<(axisId<<2)))

#define CHECK_WARM_STARTING(flag, axisId)   	((flag>>((axisId<<2) + 2))&0x01)
#define ENABLE_WARM_STARTING(flag, axisId)  	(flag|(0x01<<((axisId<<2) + 2)))
#define DISABLE_WARM_STARTING(flag, axisId) 	(flag&~(0x01<<((axisId<<2) + 2)))

ATTRIBUTE_ALIGNED128(struct) Joint
{
	Vector3 rA,rB;
	Matrix3	inertiaInvA, inertiaInvB;

	f32 breakableLimit;
	f32 massInvA, massInvB;

	ConstraintCache constraints[6];

	f32 linearImpulseWeight;
	f32 angularImpulseWeight;
	u32 jointFlag : 26;
	u8  jointType : 4;

	u16 stateIndexA;
	u16 stateIndexB;

	f32 lowerLimit[6];
	f32 upperLimit[6];

	f32 linearDamping;
	f32 angularDamping;

	f32 maxLinearImpulse;
	f32 maxAngularImpulse;

	f32 linearBias;
	f32 angularBias;

	SubData subData;

	Vector3 anchorA;
	Vector3 anchorB;

	Matrix3 frameA;
	Matrix3 frameB;
	Matrix3 targetFrame;

	Vector3 localVelocityA,localVelocityB;

	void reset()
	{
		jointType = 0;
		jointFlag = 0x2333333;
		for(s32 i=0;i < 6;i++) {
			lowerLimit[i] = upperLimit[i] = 0.0f;
			constraints[i].accumImpulse = 0.0f;
		}
		breakableLimit = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		maxLinearImpulse = 10000.0f;
		maxAngularImpulse = 10000.0f;
		linearImpulseWeight = 1.0f;
		angularImpulseWeight = 1.0f;
		linearBias = 0.2f;
		angularBias = 0.2f;

		localVelocityA = localVelocityB = Vector3(0.0f);
		subData.type = 0;
	}

	void enableBreakable() {jointFlag |= (1<<24);}
	void enableActive() {jointFlag |= (1<<25);}
	void disableBreakable() {jointFlag = ~(~jointFlag|(1<<24));}
	void disableActive() {jointFlag = ~(~jointFlag|(1<<25));}

	bool isBreakable() const {return (jointFlag&(1<<24)) != 0;}
	bool isActive() const {return (jointFlag&(1<<25)) != 0;}

	void setFree(s32 axisId)  {jointFlag = SET_AXIS_LOCK(jointFlag, axisId, JointAxisFree);}
	void setLimit(s32 axisId) {jointFlag = SET_AXIS_LOCK(jointFlag, axisId, JointAxisLimit);}
	void setLock(s32 axisId)  {jointFlag = SET_AXIS_LOCK(jointFlag, axisId, JointAxisLock);}

	bool isFree(s32 axisId)  const {return JointAxisFree  == GET_AXIS_LOCK(jointFlag, axisId);}
	bool isLimit(s32 axisId) const {return JointAxisLimit == GET_AXIS_LOCK(jointFlag, axisId);}
	bool isLock(s32 axisId)  const {return JointAxisLock  == GET_AXIS_LOCK(jointFlag, axisId);}

	void enableWarmStarting(s32 axisId) {jointFlag = ENABLE_WARM_STARTING(jointFlag, axisId);}
	void disableWarmStarting(s32 axisId) {jointFlag = DISABLE_WARM_STARTING(jointFlag, axisId);}
	bool isWarmStarting(s32 axisId) const {return CHECK_WARM_STARTING(jointFlag, axisId) != 0;}
};

#endif /* JOINT_H_ */
