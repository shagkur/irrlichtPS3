/*
 * particlesolver.cpp
 *
 *  Created on: Jan 28, 2014
 *      Author: mike
 */

#include "particlesolver.h"
#include "particleconfig.h"

#define CONTACT_EPSILON				0.0f

void applyImpulsePcl(PclContactPair& pair, PclState& particleA, PclState& particleB, const PclContactSolverConfig& config)
{
	PclContactPoint& cp = pair.contactPoint;
	cp.maxImpulse = 0.0f;

#ifdef PARTICLE_VELOCITY_BASE
	{
		Vector3 velocityA = particleA.fV;
		Vector3 velocityB = particleB.fV;

		Vector3 relativeVelocity = velocityA - velocityB;

		f32 normalVelocity = dot(relativeVelocity, cp.getNormal());
		if(normalVelocity > 0.0f) return;

		f32 separate = (-config.separateBias*MIN(0.0f, cp.distance))/config.timeStep;
		f32 impulse;

		f32 num = - (1.0f + pair.compositeElasticity)*normalVelocity;
		impulse = (num + separate)*cp.impulseDen;

		Vector3 compositeImpulse = cp.getNormal()*impulse;

		particleA.fV += compositeImpulse/particleA.getMass();
		particleB.fV -= compositeImpulse/particleB.getMass();

		Vector3 tangent = cross(cp.getNormal(), cross(cp.getNormal(), relativeVelocity));
		if(lengthSqr(tangent) > CONTACT_EPSILON) {
			tangent = normalize(tangent);
			f32 maxFriction = pair.compositeFriction*impulse;
			f32 tangentVelocity = dot(relativeVelocity, tangent);
			f32 friction = -tangentVelocity*cp.impulseDen;
			friction = CLAMP(friction, -maxFriction, maxFriction);

			compositeImpulse = tangent*friction;

			particleA.fV += compositeImpulse/particleA.getMass();
			particleB.fV -= compositeImpulse/particleB.getMass();
		}

		cp.maxImpulse = MAX(cp.maxImpulse, fabsf(impulse));
	}
#else
	f32 distance = length(particleA.fX - particleB.fX) - (particleA.getRadius() + particleB.getRadius());
	f32 posErr = MIN(0.0f, distance);
	f32 correction = -config.separateBias*posErr;

	particleA.fX += 0.1f*correction*cp.getNormal();
	particleB.fX -= 0.1f*correction*cp.getNormal();
#endif
}

void applyImpulseRig(PclContactPair& pair, PclState& particleA, TrbState& rigidbodyB, const PclContactSolverConfig& config, TrbDynBody& bodyB, f32 timeStep, bool twoWayEnable, Vector3& ForceImp, Vector3& RotImp)
{
	PclContactPoint& cp = pair.contactPoint;
	cp.maxImpulse = 0.0f;

#ifdef PARTICLE_VELOCITY_BASE
	Vector3 velocityA = particleA.getVelocity();
	Vector3 velocityB = rigidbodyB.getLinearVelocity() + cross(rigidbodyB.getAngularVelocity(), rotate(rigidbodyB.getOrientation(), cp.localPoint[1]));

	Vector3 relativeVelocity = velocityA - velocityB;

	float normalVelocity = dot(relativeVelocity, cp.getNormal());

	if(normalVelocity > 0.0f) return;

	f32 separate = (-config.separateBias*MIN(0.0f, cp.distance))/config.timeStep;
	f32 impulse;

	f32 num = -(1.0f + pair.compositeElasticity)*normalVelocity;
	impulse = (num + separate)*cp.impulseDen;

	Vector3 compositeImpulse = cp.getNormal()*impulse;

	particleA.fV += compositeImpulse/particleA.getMass();

	Vector3 tangent = cross(cp.getNormal(), cross(cp.getNormal(), relativeVelocity));
	if(lengthSqr(tangent) > CONTACT_EPSILON) {
		tangent = normalize(tangent);
		f32 maxFriction = pair.compositeFriction*impulse;
		f32 tangentVelocity = dot(relativeVelocity, tangent);
		f32 friction = -tangentVelocity*cp.impulseDen;
		friction = CLAMP(friction, -maxFriction, maxFriction);

		compositeImpulse = tangent*friction;

		particleA.fV += compositeImpulse/particleA.getMass();
	}

	cp.maxImpulse = MAX(cp.maxImpulse, fabsf(impulse));

	f32 correction = -MIN(0.0f, cp.distance);
	particleA.fX += 0.1f*correction*cp.getNormal();
#else // position base
	Vector3 pA = particleA.getPosition() - particleA.getRadius()*cp.getNormal();
	Vector3 pB = rigidbodyB.getPosition() + rotate(rigidbodyB.getOrientation(), cp.localPoint[1]);
	f32 posErr = MIN(0.0f, dot(pA - pB, cp.getNormal()));
	f32 correction = -config.separateBias*posErr;
	particleA.fX += 0.1f*correction*cp.getNormal();
#endif

	// Two Way Interaction
	if(twoWayEnable == false)
		return;

	ForceImp = Vector3(0, 0, 0);
	RotImp = Vector3(0, 0, 0);

	if((rigidbodyB.getMoveTypeBits()&MOVE_TYPE_DYNAMIC) == 0)
		return;

	if(rigidbodyB.isAsleep()) rigidbodyB.wakeup();

	Vector3 wvB = rotate(rigidbodyB.getOrientation(), cp.localPoint[1]);
	Vector3 impulseRig =  0.1f*correction/timeStep*cp.getNormal()*particleA.fMass;

	ForceImp = -1.0*impulseRig*bodyB.getMassInv();

	if(dot(wvB, impulseRig)*dot(wvB, impulseRig)/(dot(wvB, wvB)*dot(impulseRig, impulseRig)) > 0.95f*0.95f)
		return;

	Vector3 checkVec = cross(wvB, impulseRig);
	RotImp = -1.0*bodyB.getBodyInertiaInv()*checkVec*0.5f;
}

void applyJointPcl(PclJoint& joint, PclState& particleA, PclState& particleB, const PclJointSolverConfig& config)
{
#ifdef PARTICLE_VELOCITY_BASE
	joint.maxImpulse = 0.0f;

	{
		Vector3 dir(1.0f, 0.0f, 0.0f);
		Vector3 positionA = particleA.fX;
		Vector3 positionB = particleB.fX;
		Vector3 velocityA = particleA.fV;
		Vector3 velocityB = particleB.fV;

		Vector3 distance = positionA - positionB;

		f32 distSqr = lengthSqr(distance);

		if(distSqr > 0.00001f)
			dir = distance/sqrtf(distSqr);

		f32 velErr = dot(velocityA - velocityB, dir);
		f32 posErr = joint.length - dot(distance, dir);

		f32 impulse = (-joint.damping*velErr+joint.bias*posErr/config.timeStep)*joint.impulseDen;

		Vector3 impulseVector = impulse*dir;
		particleA.fV += impulseVector/particleA.getMass();
		particleB.fV -= impulseVector/particleB.getMass();

		joint.maxImpulse = MAX(joint.maxImpulse, fabsf(impulse));
	}
#else // position base
	(void) config;
	{
		Vector3 dir(1.0f, 0.0f, 0.0f);
		Vector3 positionA = particleA.fX;
		Vector3 positionB = particleB.fX;
		Vector3 distance = positionA - positionB;

		f32 distSqr = lengthSqr(distance);

		if(distSqr > 0.00001f)
			dir = distance/sqrtf(distSqr);

		f32 posErr = joint.length - dot(distance, dir);
		f32 correction = 0.1f*joint.bias*posErr;

		Vector3 correctionVector = correction*dir;
		particleA.fX += correctionVector;
		particleB.fX -= correctionVector;
	}
#endif
}

void applyJointRig(PclJoint& joint, PclState& particleA, TrbState& rigidbodyB, const PclJointSolverConfig& config)
{
#ifdef PARTICLE_VELOCITY_BASE
	joint.maxImpulse = 0.0f;

	{
		Vector3 dir(1.0f, 0.0f, 0.0f);
		Vector3 vecB = rotate(rigidbodyB.getOrientation(), joint.localPosB);
		Vector3 positionA = particleA.getPosition();
		Vector3 positionB = rigidbodyB.getPosition() + vecB;
		Vector3 velocityA = particleA.getVelocity();
		Vector3 velocityB = rigidbodyB.getLinearVelocity() + cross(rigidbodyB.getAngularVelocity(), vecB);

		Vector3 distance = positionA - positionB;

		f32 distSqr = lengthSqr(distance);

		if(distSqr > 0.00001f)
			dir = distance/sqrtf(distSqr);

		f32 velErr = dot(velocityA - velocityB, dir);
		f32 posErr = joint.length - dot(distance, dir);

		f32 impulse = (-joint.damping*velErr+joint.bias*posErr/config.timeStep)*joint.impulseDen;

		Vector3 impulseVector = impulse*dir;
		particleA.fV += impulseVector/particleA.getMass();

		joint.maxImpulse = MAX(joint.maxImpulse, fabsf(impulse));
	}
#else // position base
	(void) config;
	{
		Vector3 dir(1.0f, 0.0f, 0.0f);
		Vector3 vecB = rotate(rigidbodyB.getOrientation(), joint.localPosB);
		Vector3 positionA = particleA.fX;
		Vector3 positionB = rigidbodyB.getPosition() + vecB;

		Vector3 distance = positionA - positionB;

		f32 distSqr = lengthSqr(distance);

		if(distSqr > 0.00001f)
			dir = distance/sqrtf(distSqr);

		f32 posErr = joint.length - dot(distance, dir);
		f32 correction = 0.1f*joint.bias*posErr;
		Vector3 correctionVector = correction*dir;
		particleA.fX += correctionVector;
	}
#endif
}
