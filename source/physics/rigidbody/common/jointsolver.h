/*
 * jointsolver.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef JOINTSOLVER_H_
#define JOINTSOLVER_H_

#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/joint.h"

struct JointSolverConfig
{
	f32 timeStep;
	bool deformMeshEnable;
};

typedef void (*PreJoint)(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config);
typedef void (*ApplyJoint)(Joint& joint, TrbState& stateA, TrbState& stateB, const JointSolverConfig& config);

void preJointFixAndMov(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config);
void preJointMovAndFix(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config);
void preJointMovAndMov(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config);
void preJointFixAndFix(Joint& joint, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const JointSolverConfig& config);

void applyJointMovAndMov(Joint& joint, TrbState& stateA, TrbState& stateB, const JointSolverConfig& config);

#define UNROLL_ENABLE //optimize unroll

#ifdef UNROLL_ENABLE //optimize unroll

#include "rigidbody/common/constraintrowsolver.h"

static inline void applyJointMovAndMovInline(Joint& joint, TrbState& stateA, TrbState& stateB, const JointSolverConfig& config)
{
	(void)config;

	f32 maxImpulse = -FLT_MAX;

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

	solveLinearConstraintRowMovAndMovDelta(dLVA, dLVB, dAVA, dAVB, accumImpulse0, joint.constraints[0], joint.massInvA, joint.inertiaInvA, joint.rA, joint.massInvB, joint.inertiaInvB, joint.rB);
	solveLinearConstraintRowMovAndMovDelta(dLVA, dLVB, dAVA, dAVB, accumImpulse1, joint.constraints[1], joint.massInvA, joint.inertiaInvA, joint.rA, joint.massInvB, joint.inertiaInvB, joint.rB);
	solveLinearConstraintRowMovAndMovDelta(dLVA, dLVB, dAVA, dAVB, accumImpulse2, joint.constraints[2], joint.massInvA, joint.inertiaInvA, joint.rA, joint.massInvB, joint.inertiaInvB, joint.rB);

	// Angular
	solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse3, joint.constraints[0 + 3], joint.inertiaInvA, joint.inertiaInvB);
	solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse4, joint.constraints[1 + 3], joint.inertiaInvA, joint.inertiaInvB);
	solveAngularConstraintRowMovAndMovDelta(dAVA, dAVB, accumImpulse5, joint.constraints[2 + 3], joint.inertiaInvA, joint.inertiaInvB);

	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse0));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse1));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse2));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse3));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse4));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse5));

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

#if 1 //optimize  branch elimination
	bool jointBreakableFlag1 = ((joint.jointFlag&(1<<24)) !=0 );
	bool jointBreakableFlag2 = (maxImpulse > joint.breakableLimit);

	joint.jointFlag = (jointBreakableFlag1 && jointBreakableFlag2) ? ~(~joint.jointFlag|(1<<25)) : joint.jointFlag;
#else //optimize  branch elimination
	if(UNLIKELY(joint.isBreakable() && maxImpulse > joint.breakableLimit))
		joint.disableActive();
#endif //optimize  branch elimination
}

static inline void applyJointMovAndMovInlineDouble(Joint& joint1, TrbState& stateA1, TrbState& stateB1, Joint& joint2, TrbState& stateA2, TrbState& stateB2, const JointSolverConfig& config)
{
	(void)config;

	f32 maxImpulse = -FLT_MAX;

	// Linear
	Vector3 dLVA1(stateA1.getDeltaLinearVelocity());
	Vector3 dLVB1(stateB1.getDeltaLinearVelocity());
	Vector3 dAVA1(stateA1.getDeltaAngularVelocity());
	Vector3 dAVB1(stateB1.getDeltaAngularVelocity());
	Vector3 dLVA2(stateA2.getDeltaLinearVelocity());
	Vector3 dLVB2(stateB2.getDeltaLinearVelocity());
	Vector3 dAVA2(stateA2.getDeltaAngularVelocity());
	Vector3 dAVB2(stateB2.getDeltaAngularVelocity());

	f32 accumImpulse10 = joint1.constraints[0].accumImpulse;
	f32 accumImpulse11 = joint1.constraints[1].accumImpulse;
	f32 accumImpulse12 = joint1.constraints[2].accumImpulse;
	f32 accumImpulse13 = joint1.constraints[3].accumImpulse;
	f32 accumImpulse14 = joint1.constraints[4].accumImpulse;
	f32 accumImpulse15 = joint1.constraints[5].accumImpulse;
	f32 accumImpulse20 = joint2.constraints[0].accumImpulse;
	f32 accumImpulse21 = joint2.constraints[1].accumImpulse;
	f32 accumImpulse22 = joint2.constraints[2].accumImpulse;
	f32 accumImpulse23 = joint2.constraints[3].accumImpulse;
	f32 accumImpulse24 = joint2.constraints[4].accumImpulse;
	f32 accumImpulse25 = joint2.constraints[5].accumImpulse;

	solveLinearConstraintRowMovAndMovDelta(dLVA1, dLVB1, dAVA1, dAVB1, accumImpulse10, joint1.constraints[0], joint1.massInvA, joint1.inertiaInvA, joint1.rA, joint1.massInvB, joint1.inertiaInvB, joint1.rB);
	solveLinearConstraintRowMovAndMovDelta(dLVA2, dLVB2, dAVA2, dAVB2, accumImpulse20, joint2.constraints[0], joint2.massInvA, joint2.inertiaInvA, joint2.rA, joint2.massInvB, joint2.inertiaInvB, joint2.rB);
	solveLinearConstraintRowMovAndMovDelta(dLVA1, dLVB1, dAVA1, dAVB1, accumImpulse11, joint1.constraints[1], joint1.massInvA, joint1.inertiaInvA, joint1.rA, joint1.massInvB, joint1.inertiaInvB, joint1.rB);
	solveLinearConstraintRowMovAndMovDelta(dLVA2, dLVB2, dAVA2, dAVB2, accumImpulse21, joint2.constraints[1], joint2.massInvA, joint2.inertiaInvA, joint2.rA, joint2.massInvB, joint2.inertiaInvB, joint2.rB);
	solveLinearConstraintRowMovAndMovDelta(dLVA1, dLVB1, dAVA1, dAVB1, accumImpulse12, joint1.constraints[2], joint1.massInvA, joint1.inertiaInvA, joint1.rA, joint1.massInvB, joint1.inertiaInvB, joint1.rB);
	solveLinearConstraintRowMovAndMovDelta(dLVA2, dLVB2, dAVA2, dAVB2, accumImpulse22, joint2.constraints[2], joint2.massInvA, joint2.inertiaInvA, joint2.rA, joint2.massInvB, joint2.inertiaInvB, joint2.rB);

	solveAngularConstraintRowMovAndMovDelta(dAVA1, dAVB1, accumImpulse13, joint1.constraints[0 + 3], joint1.inertiaInvA, joint1.inertiaInvB);
	solveAngularConstraintRowMovAndMovDelta(dAVA2, dAVB2, accumImpulse23, joint2.constraints[0 + 3], joint2.inertiaInvA, joint2.inertiaInvB);
	solveAngularConstraintRowMovAndMovDelta(dAVA1, dAVB1, accumImpulse14, joint1.constraints[1 + 3], joint1.inertiaInvA, joint1.inertiaInvB);
	solveAngularConstraintRowMovAndMovDelta(dAVA2, dAVB2, accumImpulse24, joint2.constraints[1 + 3], joint2.inertiaInvA, joint2.inertiaInvB);
	solveAngularConstraintRowMovAndMovDelta(dAVA1, dAVB1, accumImpulse15, joint1.constraints[2 + 3], joint1.inertiaInvA, joint1.inertiaInvB);
	solveAngularConstraintRowMovAndMovDelta(dAVA2, dAVB2, accumImpulse25, joint2.constraints[2 + 3], joint2.inertiaInvA, joint2.inertiaInvB);

	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse10));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse11));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse12));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse13));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse14));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse15));

	stateA1.setDeltaLinearVelocity(dLVA1);
	stateB1.setDeltaLinearVelocity(dLVB1);
	stateA1.setDeltaAngularVelocity(dAVA1);
	stateB1.setDeltaAngularVelocity(dAVB1);
	joint1.constraints[0].accumImpulse = accumImpulse10;
	joint1.constraints[1].accumImpulse = accumImpulse11;
	joint1.constraints[2].accumImpulse = accumImpulse12;
	joint1.constraints[3].accumImpulse = accumImpulse13;
	joint1.constraints[4].accumImpulse = accumImpulse14;
	joint1.constraints[5].accumImpulse = accumImpulse15;

    bool joint1BreakableFlag1 = ((joint1.jointFlag&(1<<24)) != 0);
	bool joint1BreakableFlag2 = (maxImpulse > joint1.breakableLimit);
	joint1.jointFlag = (joint1BreakableFlag1 && joint1BreakableFlag2) ? ~(~joint1.jointFlag|(1<<25)) : joint1.jointFlag;

	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse20));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse21));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse22));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse23));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse24));
	maxImpulse = MAXF(maxImpulse, fabsf(accumImpulse25));

	stateA2.setDeltaLinearVelocity(dLVA2);
	stateB2.setDeltaLinearVelocity(dLVB2);
	stateA2.setDeltaAngularVelocity(dAVA2);
	stateB2.setDeltaAngularVelocity(dAVB2);
	joint2.constraints[0].accumImpulse = accumImpulse20;
	joint2.constraints[1].accumImpulse = accumImpulse21;
	joint2.constraints[2].accumImpulse = accumImpulse22;
	joint2.constraints[3].accumImpulse = accumImpulse23;
	joint2.constraints[4].accumImpulse = accumImpulse24;
	joint2.constraints[5].accumImpulse = accumImpulse25;

    bool joint2BreakableFlag1 = ((joint2.jointFlag&(1<<24)) != 0);
	bool joint2BreakableFlag2 = (maxImpulse > joint2.breakableLimit);
	joint2.jointFlag = (joint2BreakableFlag1 && joint2BreakableFlag2) ? ~(~joint2.jointFlag|(1<<25)) : joint2.jointFlag;
}

#endif // UNROLL_ENABLE //optimize unroll

#endif /* JOINTSOLVER_H_ */
