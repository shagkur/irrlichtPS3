/*
 * jointsolver.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/jointsolver.h"
#include "rigidbody/common/vec_utils.h"
#include "rigidbody/common/constraintrowsolver.h"
#include "rigidbody/common/rigidbodyconfig.h"

///////////////////////////////////////////////////////////////////////////////
// Calc Joint Angle

static inline void calcJointAngleStd(Matrix3& worldFrameA, Matrix3& worldFrameB, f32 *angle, Vector3 *axis)
{
	Matrix3 frameBA = transpose(worldFrameA)*worldFrameB;

	Quat swing, twist, qBA(frameBA);
	swing = Quat::rotation(Vector3(1, 0, 0), frameBA.getCol0());
	twist = qBA*conj(swing);

	if(dot(twist, Quat::rotationX(0.0f)) < 0.0f)
		twist = -twist;

	angleAxis(normalize(twist), angle[0], axis[0]);
	angleAxis(normalize(swing), angle[1], axis[1]);

	if(angle[1] < 0.00001f)
		axis[1] = Vector3(0, 1, 0);

	if(dot(axis[0], frameBA.getCol0()) < 0.0f) {
		axis[0] = -axis[0];
		angle[0] = -angle[0];
	}

	axis[0] = worldFrameA*axis[0];
	axis[1] = worldFrameA*axis[1];
	axis[2] = cross(axis[0], axis[1]);
	angle[2] = 0.0f;
}

static inline void calcJointAngleUniversal(Matrix3& worldFrameA, Matrix3& worldFrameB, f32 *angle, Vector3 *axis)
{
	Matrix3 frameBA = transpose(worldFrameA)*worldFrameB;

	Quat swing, swing1, swing2, twist, qBA(frameBA);
	Vector3 Pxy(frameBA.getCol0());
	Pxy[2] = 0.0f;
	swing1 = Quat::rotation(Vector3(1, 0, 0), normalize(Pxy));
	swing = Quat::rotation(Vector3(1, 0, 0), frameBA.getCol0());
	swing2 = swing*conj(swing1);
	twist = qBA*conj(swing);

	if(dot(twist, Quat::rotationX(0.0f)) < 0.0f)
		twist = -twist;

	angleAxis(normalize(twist), angle[0], axis[0]);
	angleAxis(normalize(swing1), angle[1], axis[1]);
	angleAxis(normalize(swing2), angle[2], axis[2]);

	if(axis[1].getZ() < 0.0f) {
		axis[1] = -axis[1];
		angle[1] = -angle[1];
	}

	Vector3 chkY = cross(Vector3(0, 0, 1), frameBA.getCol0());
	if(dot(chkY,axis[2]) < 0.0f) {
		axis[2] = -axis[2];
		angle[2] = -angle[2];
	}

	if(dot(axis[0], frameBA.getCol0()) < 0.0f)
		angle[0] = -angle[0];

	axis[0] = worldFrameB.getCol0();
	axis[1] = worldFrameA*axis[1];
	axis[2] = worldFrameA*axis[2];
}


///////////////////////////////////////////////////////////////////////////////
// Calc Joint Limit

static inline void calcLinearLimit(Joint& joint, s32 c, f32& posErr, f32& velocityAmp, f32& lowerLimit, f32& upperLimit)
{
	if(joint.isFree(c)) {
		posErr = 0.0f;
		velocityAmp *= joint.linearDamping;
	} else if(joint.isLimit(c)) {
		if(posErr >= joint.lowerLimit[c] && posErr <= joint.upperLimit[c]) {
			posErr = 0.0f;
			velocityAmp *= joint.linearDamping;
		} else {
			if(posErr < joint.lowerLimit[c]) {
				posErr = posErr - joint.lowerLimit[c];
				posErr = MIN(0.0f, posErr + JOINT_LIN_SLOP);
				upperLimit = MIN(0.0f, upperLimit);
				velocityAmp = 1.0f;
			} else { // posErr > joint.upperLimit[c]
				posErr = posErr - joint.upperLimit[c];
				posErr = MAX(0.0f, posErr - JOINT_LIN_SLOP);
				lowerLimit = MAX(0.0f, lowerLimit);
				velocityAmp = 1.0f;
			}
		}
	}
}

static inline void calcAngularLimit(Joint& joint, s32 c, f32& posErr, f32& velocityAmp, f32& lowerLimit, f32& upperLimit)
{
	if(joint.isFree(c)) {
		posErr = 0.0f;
		velocityAmp *= joint.angularDamping;
	} else if(joint.isLimit(c)) {
		if(posErr >= joint.lowerLimit[c] && posErr <= joint.upperLimit[c]) {
			posErr = 0.0f;
			velocityAmp *= joint.angularDamping;
		} else {
			if(posErr < joint.lowerLimit[c]) {
				posErr = posErr - joint.lowerLimit[c];
				posErr = MIN(0.0f, posErr + JOINT_ANG_SLOP);
				upperLimit = MIN(0.0f, upperLimit);
				velocityAmp = 1.0f;
			} else { // posErr > joint.upperLimit[c]
				posErr = posErr - joint.upperLimit[c];
				posErr =  MAX(0.0f, posErr - JOINT_ANG_SLOP);
				lowerLimit =  MAX(0.0f, lowerLimit);
				velocityAmp = 1.0f;
			}
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
// Pre Joint

void preJointFixAndFix(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config)
{
	(void)stateA;(void)bodyA;(void)stateB;(void)bodyB;(void)config;
	joint.massInvA = 0.0f;
	joint.inertiaInvA = Matrix3(0.0f);
	joint.massInvB = 0.0f;
	joint.inertiaInvB = Matrix3(0.0f);
}

void preJointMovAndMov(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config)
{
	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());

	Vector3 rA = rotate(stateA.getOrientation(), joint.anchorA);
	Vector3 rB = rotate(stateB.getOrientation(), joint.anchorB);

	Vector3 vA = stateA.getLinearVelocity() + cross(stateA.getAngularVelocity(), rA);
	Vector3 vB = stateB.getLinearVelocity() + cross(stateB.getAngularVelocity(), rB);
	Vector3 vAB = vA - vB;

	Vector3 distance = (stateA.getPosition() + rA) - (stateB.getPosition() + rB);

	Matrix3 worldFrameA, worldFrameB;
	worldFrameB = mB*joint.frameB;
	if(joint.jointType == JointTypeAnimation)
		worldFrameA = mA*joint.frameA*joint.targetFrame;
	else
		worldFrameA = mA*joint.frameA;

	f32 massInvA = bodyA.getMassInv();
	f32 massInvB = bodyB.getMassInv();
	Matrix3 inertiaInvA = mA*bodyA.getBodyInertiaInv()*transpose(mA);
	Matrix3 inertiaInvB = mB*bodyB.getBodyInertiaInv()*transpose(mB);

	// Linear
	Matrix3 K = Matrix3::scale(Vector3(massInvA + massInvB)) - crossMatrix(rA)*inertiaInvA*crossMatrix(rA) - crossMatrix(rB)*inertiaInvB * crossMatrix(rB);

	s32 lcnt = (joint.jointType != JointTypeDistance) ? 3 : 1;
	for(s32 c=0;c < lcnt;c++) {
		Vector3 normal = (joint.jointType != JointTypeDistance) ? worldFrameA[c] : normalize((stateA.getPosition() + joint.rA) - (stateB.getPosition() + joint.rB));

		f32 posErr = dot(distance, -normal);
		f32 lowerLimit = -joint.maxLinearImpulse;
		f32 upperLimit =  joint.maxLinearImpulse;
		f32 velocityAmp = 1.0f;

		calcLinearLimit(joint, c, posErr, velocityAmp, lowerLimit, upperLimit);

		f32 denom = dot(K*normal, normal);

		ConstraintCache& constraint = joint.constraints[c];

		constraint.rhs = -velocityAmp*dot(vAB, normal);
		constraint.rhs -= DIVF((joint.linearBias*(-posErr)), config.timeStep);
		constraint.rhs *= DIVF(joint.linearImpulseWeight, denom);
		constraint.jacDiagInv = DIVF(joint.linearImpulseWeight*velocityAmp, denom);
		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal, constraint.normal);

		if(joint.isWarmStarting(c)) {
			f32 deltaImpulse = constraint.accumImpulse;
			stateA.setDeltaLinearVelocity(stateA.getDeltaLinearVelocity() + deltaImpulse*massInvA*normal);
			stateA.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() + deltaImpulse*inertiaInvA*cross(rA, normal));
			stateB.setDeltaLinearVelocity(stateB.getDeltaLinearVelocity() - deltaImpulse*massInvB*normal);
			stateB.setDeltaAngularVelocity(stateB.getDeltaAngularVelocity() - deltaImpulse*inertiaInvB*cross(rB, normal));
		} else
			constraint.accumImpulse = 0.0f;
	}

	Vector3 axis[3];
	f32 angle[3];

	if(joint.jointType == JointTypeUniversal)
		calcJointAngleUniversal(worldFrameA, worldFrameB, angle, axis);
	else
		calcJointAngleStd(worldFrameA, worldFrameB, angle, axis);

	// Angular
	for(s32 c=0;c < 3;c++) {
		Vector3 normal = axis[c];

		f32 posErr = angle[c];
		f32 lowerLimit = -joint.maxAngularImpulse;
		f32 upperLimit =  joint.maxAngularImpulse;
		f32 velocityAmp = 1.0f;

		calcAngularLimit(joint, c + 3, posErr, velocityAmp, lowerLimit, upperLimit);

		f32 denom = dot((inertiaInvA + inertiaInvB)*normal, normal);

		ConstraintCache &constraint = joint.constraints[c + 3];

		constraint.rhs = -velocityAmp*dot(stateA.getAngularVelocity() - stateB.getAngularVelocity(), normal); // velocity error
		constraint.rhs -= DIVF((joint.angularBias*(-posErr)), config.timeStep); // position error
		constraint.rhs *= DIVF(joint.angularImpulseWeight, denom);
		constraint.jacDiagInv = DIVF(joint.angularImpulseWeight*velocityAmp, denom);

		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal, constraint.normal);

		if(joint.isWarmStarting(c + 3)) {
			f32 deltaImpulse = constraint.accumImpulse;
			stateA.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() + deltaImpulse*inertiaInvA*normal);
			stateB.setDeltaAngularVelocity(stateB.getDeltaAngularVelocity() - deltaImpulse*inertiaInvB*normal);
		} else
			constraint.accumImpulse = 0.0f;
	}

	joint.rA = rA;
	joint.rB = rB;
	joint.massInvA = massInvA;
	joint.inertiaInvA = inertiaInvA;
	joint.massInvB = massInvB;
	joint.inertiaInvB = inertiaInvB;
}

void preJointFixAndMov(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config)
{
	(void) bodyA;

	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());

	Vector3 rA = rotate(stateA.getOrientation(), joint.anchorA);
	Vector3 rB = rotate(stateB.getOrientation(), joint.anchorB);

	Vector3 vA = stateA.getLinearVelocity() + cross(stateA.getAngularVelocity(), rA);
	Vector3 vB = stateB.getLinearVelocity() + cross(stateB.getAngularVelocity(), rB);
	if(UNLIKELY(config.deformMeshEnable && joint.subData.type == SubData::SubDataFacetLocal))
		vA += mA*joint.localVelocityA;

	Vector3 vAB = vA - vB;

	Vector3 distance = (stateA.getPosition() + rA) - (stateB.getPosition() + rB);

	Matrix3 worldFrameA, worldFrameB;
	worldFrameB = mB*joint.frameB;
	if(joint.jointType == JointTypeAnimation)
		worldFrameA = mA*joint.frameA*joint.targetFrame;
	else
		worldFrameA = mA*joint.frameA;

	f32 massInvB = bodyB.getMassInv();
	Matrix3 inertiaInvB = mB*bodyB.getBodyInertiaInv()*transpose(mB);

	// Linear
	Matrix3 K = Matrix3::scale(Vector3(massInvB)) - crossMatrix(rB)*inertiaInvB*crossMatrix(rB);

	s32 lcnt = (joint.jointType != JointTypeDistance) ? 3 : 1;
	for(s32 c=0;c < lcnt;c++) {
		Vector3 normal = (joint.jointType != JointTypeDistance) ? worldFrameA[c] : normalize((stateB.getPosition() + joint.rB) - (stateA.getPosition() + joint.rA));

		f32 posErr = dot(distance, -normal);
		f32 lowerLimit = -joint.maxLinearImpulse;
		f32 upperLimit =  joint.maxLinearImpulse;
		f32 velocityAmp = 1.0f;

		calcLinearLimit(joint, c, posErr, velocityAmp, lowerLimit, upperLimit);

		f32 denom = dot(K*normal, normal);

		ConstraintCache& constraint = joint.constraints[c];

		constraint.rhs = -velocityAmp*dot(vAB, normal);
		constraint.rhs -= DIVF((joint.linearBias*(-posErr)), config.timeStep);
		constraint.rhs *= DIVF(joint.linearImpulseWeight, denom);
		constraint.jacDiagInv = DIVF(joint.linearImpulseWeight*velocityAmp, denom);

		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal, constraint.normal);

		if(joint.isWarmStarting(c)) {
			f32 deltaImpulse = constraint.accumImpulse;
			stateB.setDeltaLinearVelocity(stateB.getDeltaLinearVelocity() - deltaImpulse*massInvB*normal);
			stateB.setDeltaAngularVelocity(stateB.getDeltaAngularVelocity() - deltaImpulse*inertiaInvB*cross(rB, normal));
		} else
			constraint.accumImpulse = 0.0f;
	}

	Vector3 axis[3];
	f32 angle[3];

	if(joint.jointType == JointTypeUniversal)
		calcJointAngleUniversal(worldFrameA, worldFrameB, angle, axis);
	else
		calcJointAngleStd(worldFrameA, worldFrameB, angle, axis);

	// Angular
	for(s32 c=0;c < 3;c++) {
		Vector3 normal = axis[c];

		f32 posErr = angle[c];
		f32 lowerLimit = -joint.maxAngularImpulse;
		f32 upperLimit =  joint.maxAngularImpulse;
		f32 velocityAmp = 1.0f;

		calcAngularLimit(joint, c + 3, posErr, velocityAmp, lowerLimit, upperLimit);

		f32 denom = dot(inertiaInvB*normal, normal);

		ConstraintCache &constraint = joint.constraints[c + 3];

		constraint.rhs = -velocityAmp*dot(stateA.getAngularVelocity() - stateB.getAngularVelocity(), normal); // velocity error
		constraint.rhs -= DIVF((joint.angularBias*(-posErr)), config.timeStep); // position error
		constraint.rhs *= DIVF(joint.angularImpulseWeight, denom);
		constraint.jacDiagInv = DIVF(joint.angularImpulseWeight*velocityAmp, denom);

		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal, constraint.normal);

		if(joint.isWarmStarting(c + 3)) {
			f32 deltaImpulse = constraint.accumImpulse;
			stateB.setDeltaAngularVelocity(stateB.getDeltaAngularVelocity() - deltaImpulse*inertiaInvB*normal);
		} else
			constraint.accumImpulse = 0.0f;
	}

	joint.rA = rA;
	joint.rB = rB;
	joint.massInvA = 0.0f;
	joint.inertiaInvA = Matrix3(0.0f);
	joint.massInvB = massInvB;
	joint.inertiaInvB = inertiaInvB;
}

void preJointMovAndFix(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config)
{
	(void) bodyB;

	Matrix3 mA(stateA.getOrientation());
	Matrix3 mB(stateB.getOrientation());

	Vector3 rA = rotate(stateA.getOrientation(), joint.anchorA);
	Vector3 rB = rotate(stateB.getOrientation(), joint.anchorB);

	Vector3 vA = stateA.getLinearVelocity() + cross(stateA.getAngularVelocity(), rA);
	Vector3 vB = stateB.getLinearVelocity() + cross(stateB.getAngularVelocity(), rB);
	if(UNLIKELY(config.deformMeshEnable && joint.subData.type == SubData::SubDataFacetLocal))
		vB += mB*joint.localVelocityB;

	Vector3 vAB = vA - vB;

	Vector3 distance = (stateA.getPosition() + rA) - (stateB.getPosition() + rB);

	Matrix3 worldFrameA, worldFrameB;
	worldFrameB = mB*joint.frameB;
	if(joint.jointType == JointTypeAnimation)
		worldFrameA = mA*joint.frameA*joint.targetFrame;
	else
		worldFrameA = mA*joint.frameA;

	f32 massInvA = bodyA.getMassInv();
	Matrix3 inertiaInvA = mA*bodyA.getBodyInertiaInv()*transpose(mA);

	// Linear
	Matrix3 K = Matrix3::scale(Vector3(massInvA)) - crossMatrix(rA)*inertiaInvA*crossMatrix(rA);

	s32 lcnt = (joint.jointType != JointTypeDistance) ? 3 : 1;
	for(s32 c=0;c < lcnt;c++) {
		Vector3 normal = (joint.jointType != JointTypeDistance) ? worldFrameA[c] : normalize((stateB.getPosition() + joint.rB) - (stateA.getPosition() + joint.rA));

		f32 posErr = dot(distance, -normal);
		f32 lowerLimit = -joint.maxLinearImpulse;
		f32 upperLimit =  joint.maxLinearImpulse;
		f32 velocityAmp = 1.0f;

		calcLinearLimit(joint, c, posErr, velocityAmp, lowerLimit, upperLimit);

		f32 denom = dot(K*normal, normal);

		ConstraintCache& constraint = joint.constraints[c];

		constraint.rhs = -velocityAmp*dot(vAB, normal);
		constraint.rhs -= DIVF((joint.linearBias*(-posErr)), config.timeStep);
		constraint.rhs *= DIVF(joint.linearImpulseWeight, denom);
		constraint.jacDiagInv = DIVF(joint.linearImpulseWeight*velocityAmp, denom);

		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal, constraint.normal);

		if(joint.isWarmStarting(c)) {
			f32 deltaImpulse = constraint.accumImpulse;
			stateA.setDeltaLinearVelocity(stateA.getDeltaLinearVelocity() + deltaImpulse*massInvA*normal);
			stateA.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() + deltaImpulse*inertiaInvA*cross(rA, normal));
		} else
			constraint.accumImpulse = 0.0f;
	}

	Vector3 axis[3];
	f32 angle[3];

	if(joint.jointType == JointTypeUniversal)
		calcJointAngleUniversal(worldFrameA, worldFrameB, angle, axis);
	else
		calcJointAngleStd(worldFrameA, worldFrameB, angle, axis);

	// Angular
	for(s32 c=0;c < 3;c++) {
		Vector3 normal = axis[c];

		f32 posErr = angle[c];
		f32 lowerLimit = -joint.maxAngularImpulse;
		f32 upperLimit =  joint.maxAngularImpulse;
		f32 velocityAmp = 1.0f;

		calcAngularLimit(joint, c + 3, posErr, velocityAmp, lowerLimit, upperLimit);

		f32 denom = dot(inertiaInvA*normal, normal);

		ConstraintCache &constraint = joint.constraints[c+3];

		constraint.rhs = -velocityAmp*dot(stateA.getAngularVelocity() - stateB.getAngularVelocity(), normal); // velocity error
		constraint.rhs -= DIVF((joint.angularBias*(-posErr)), config.timeStep); // position error
		constraint.rhs *= DIVF(joint.angularImpulseWeight, denom);
		constraint.jacDiagInv = DIVF(joint.angularImpulseWeight*velocityAmp, denom);

		constraint.lowerLimit = lowerLimit;
		constraint.upperLimit = upperLimit;
		store_Vector3(normal, constraint.normal);

		if(joint.isWarmStarting(c + 3)) {
			f32 deltaImpulse = constraint.accumImpulse;
			stateA.setDeltaAngularVelocity(stateA.getDeltaAngularVelocity() + deltaImpulse*inertiaInvA*normal);
		} else
			constraint.accumImpulse = 0.0f;
	}

	joint.rA = rA;
	joint.rB = rB;
	joint.massInvA = massInvA;
	joint.inertiaInvA = inertiaInvA;
	joint.massInvB = 0.0f;
	joint.inertiaInvB = Matrix3(0.0f);
}

///////////////////////////////////////////////////////////////////////////////
// Apply Joint Impulse

void applyJointMovAndMov(Joint& joint, TrbState& stateA, TrbState& stateB, const JointSolverConfig& config)
{
	(void)config;

	f32 maxImpulse = -FLT_MAX;

#ifdef UNROLL_ENABLE //optimize unroll
	// Linear
	Vector3 dLVA(stateA.getDeltaLinearVelocity());
	Vector3 dLVB(stateB.getDeltaLinearVelocity());
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	f32 accumImpulse0 = joint.constraints[0].accumImpulse;
	f32 accumImpulse1 = joint.constraints[1].accumImpulse;
	f32 accumImpulse2 = joint.constraints[2].accumImpulse;
	f32 accumImpulse3 = joint.constraints[3].accumImpulse;
	f32 accumImpulse4 = joint.constraints[4].accumImpulse;
	f32 accumImpulse5 = joint.constraints[5].accumImpulse;

	if(joint.jointType != JointTypeDistance){
		solveLinearConstraintRowMovAndMovDelta(dLVA, dLVB, dAVA, dAVB, accumImpulse0, joint.constraints[0], joint.massInvA, joint.inertiaInvA, joint.rA, joint.massInvB, joint.inertiaInvB, joint.rB);
		solveLinearConstraintRowMovAndMovDelta(dLVA, dLVB, dAVA, dAVB, accumImpulse1, joint.constraints[1], joint.massInvA, joint.inertiaInvA, joint.rA, joint.massInvB, joint.inertiaInvB, joint.rB);
		solveLinearConstraintRowMovAndMovDelta(dLVA, dLVB, dAVA, dAVB, accumImpulse2, joint.constraints[2], joint.massInvA, joint.inertiaInvA, joint.rA, joint.massInvB, joint.inertiaInvB, joint.rB);

		// Angular
		solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse3, joint.constraints[0 + 3], joint.inertiaInvA, joint.inertiaInvB);
		solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse4, joint.constraints[1 + 3], joint.inertiaInvA, joint.inertiaInvB);
		solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse5, joint.constraints[2 + 3], joint.inertiaInvA, joint.inertiaInvB);

		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse0));
		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse1));
		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse2));
		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse3));
		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse4));
		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse5));

	} else{
		solveLinearConstraintRowMovAndMovDelta(dLVA, dLVB, dAVA, dAVB, accumImpulse0, joint.constraints[0], joint.massInvA, joint.inertiaInvA, joint.rA, joint.massInvB, joint.inertiaInvB, joint.rB);

		solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse3, joint.constraints[0 + 3], joint.inertiaInvA, joint.inertiaInvB);
		solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse4, joint.constraints[1 + 3], joint.inertiaInvA, joint.inertiaInvB);
		solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse5, joint.constraints[2 + 3], joint.inertiaInvA, joint.inertiaInvB);

		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse0));
		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse3));
		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse4));
		maxImpulse = MAX(maxImpulse, fabsf(accumImpulse5));
	}
	stateA.setDeltaLinearVelocity(dLVA);
	stateA.setDeltaAngularVelocity(dAVA);
	stateB.setDeltaLinearVelocity(dLVB);
	stateB.setDeltaAngularVelocity(dAVB);
	joint.constraints[0].accumImpulse = accumImpulse0;
	joint.constraints[1].accumImpulse = accumImpulse1;
	joint.constraints[2].accumImpulse = accumImpulse2;
	joint.constraints[3].accumImpulse = accumImpulse3;
	joint.constraints[4].accumImpulse = accumImpulse4;
	joint.constraints[5].accumImpulse = accumImpulse5;

#else
	// Linear
	s32 lcnt = (joint.jointType != JointTypeDistance) ? 3 : 1;
	for(s32 c=0;c < lcnt;c++) {
		solveLinearConstraintRowMovAndMov(joint.constraints[c], stateA, joint.massInvA, joint.inertiaInvA, joint.rA, stateB, joint.massInvB, joint.inertiaInvB, joint.rB);
		maxImpulse = MAX(maxImpulse, fabsf(joint.constraints[c].accumImpulse));
	}

	// Angular
	for(s32 c=0;c < 3;c++) {
		solveAngularConstraintRowMovAndMov(joint.constraints[c + 3], stateA, joint.inertiaInvA, stateB, joint.inertiaInvB);
		maxImpulse = MAX(maxImpulse, fabsf(joint.constraints[c + 3].accumImpulse));
	}

#endif
	if(UNLIKELY(joint.isBreakable() && maxImpulse > joint.breakableLimit))
		joint.disableActive();
}


