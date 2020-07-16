/*
 * softbodysolver.h
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#ifndef SOFTBODYSOLVER_H_
#define SOFTBODYSOLVER_H_

#include "softbody/common/softcontact.h"
#include "softbody/common/softjoint.h"
#include "softbody/common/softstate.h"

#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/trbstatevec.h"

struct SoftContactSolverConfig
{
	f32 timeStep;
	f32 separateBias;
	f32 particleRadius;
};

struct SoftJointSolverConfig
{
	f32 timeStep;
};

static inline void applyImpulsePcl(SoftContactPair& pair, SoftState& particleA, SoftState& particleB, const SoftContactSolverConfig& config)
{
	SoftContactPoint& cp = pair.contactPoint;
	cp.maxImpulse = 0.0f;

	f32 distance = length((particleA.fX + particleA.accumulation) - (particleB.fX + particleB.accumulation)) - (config.particleRadius + config.particleRadius);

	f32 posErr = MIN(0.0f, distance);

	f32 correction = -config.separateBias*posErr;
	particleA.accumulation += 0.1f*correction*cp.getNormal();
	particleB.accumulation -= 0.1f*correction*cp.getNormal();
}

static inline void applyImpulseRig(SoftContactPair& pair, SoftState& particleA, TrbState& rigidbodyB, const SoftContactSolverConfig& config, TrbDynBody& bodyB)
{
	(void) bodyB;

	SoftContactPoint& cp = pair.contactPoint;
	cp.maxImpulse = 0.0f;

	Vector3 pA = (particleA.fX + particleA.accumulation) - config.particleRadius*cp.getNormal();
	Vector3 pB = rigidbodyB.getPosition() + rotate(rigidbodyB.getOrientation(), cp.localPoint[1]);

	f32 posErr = MIN(0.0f, dot(pA - pB, cp.getNormal()));

	f32 correction = -config.separateBias*posErr;
	particleA.accumulation += 0.1f*correction*cp.getNormal();
}

static inline void applyImpulseRigTwoWay(SoftContactPair& pair, SoftState& particleA, TrbState& rigidbodyB, const SoftContactSolverConfig& config, TrbDynBody& bodyB, Vector3& forceImp, Vector3& rotImp)
{
	SoftContactPoint &cp = pair.contactPoint;
	cp.maxImpulse = 0.0f;

	Vector3 pA = (particleA.fX + particleA.accumulation) - config.particleRadius*cp.getNormal();
	Vector3 pB = rigidbodyB.getPosition() + rotate(rigidbodyB.getOrientation(), cp.localPoint[1]);

	f32 posErr = MIN(0.0f, dot(pA - pB, cp.getNormal()));

	f32 correction = -config.separateBias*posErr;
	particleA.accumulation += 0.1f*correction*cp.getNormal();

	forceImp = Vector3(0.0f);
	rotImp = Vector3(0.0f);

	if((rigidbodyB.getMoveTypeBits()&MOVE_TYPE_DYNAMIC) == 0)
		return;

	if(rigidbodyB.isAsleep()) rigidbodyB.wakeup();

	Vector3 wvB = rotate(rigidbodyB.getOrientation(), cp.localPoint[1]);
	Vector3 compositeImpulse =  0.1f*correction*cp.getNormal()/config.timeStep*particleA.fMass;

	forceImp = -1.0f*compositeImpulse*bodyB.getMassInv();

	if(dot(wvB, compositeImpulse)*dot(wvB, compositeImpulse)/(dot(wvB, wvB)*dot(compositeImpulse, compositeImpulse)) > 0.95f*0.95f)
		return;

	Vector3 checkVec = cross(wvB, compositeImpulse);
	rotImp = -1.0f*bodyB.getBodyInertiaInv()*checkVec*0.5f;
}

static inline void applyJointPcl(SoftJoint& joint, SoftState& particleA, SoftState& particleB, const SoftJointSolverConfig& config)
{
	(void) config;

	Vector3 dir(1.0f, 0.0f, 0.0f);
	Vector3 positionA = particleA.fX + particleA.accumulation;
	Vector3 positionB = particleB.fX + particleB.accumulation;
	Vector3 distance = positionA - positionB;

	f32 distSqr = lengthSqr(distance);

	if(distSqr > 0.00001f)
		dir = distance/sqrtf(distSqr);

	f32 posErr = joint.length - dot(distance, dir);

	f32 correction = 0.1f*joint.bias*posErr;

	Vector3 correctionVector = correction*dir;

	particleA.accumulation += correctionVector;
	particleB.accumulation -= correctionVector;
}

static inline void applyJointRig(SoftJoint& joint, SoftState& particleA, TrbState& rigidbodyB, const SoftJointSolverConfig& config)
{
	(void) config;

	Vector3 dir(1.0f, 0.0f, 0.0f);
	Vector3 vecB = rotate(rigidbodyB.getOrientation(), joint.localPosB);
	Vector3 positionA = particleA.fX + particleA.accumulation;
	Vector3 positionB = rigidbodyB.getPosition() + vecB;

	Vector3 distance = positionA - positionB;

	f32 distSqr = lengthSqr(distance);

	if(distSqr > 0.00001f)
		dir = distance/sqrtf(distSqr);

	f32 posErr = joint.length - dot(distance, dir);

	f32 correction = 0.1f*joint.bias*posErr;

	Vector3 correctionVector = correction*dir;

	particleA.accumulation += correctionVector;
}

#endif /* SOFTBODYSOLVER_H_ */
