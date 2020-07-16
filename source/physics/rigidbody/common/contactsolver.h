/*
 * contactsolver.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef CONTACTSOLVER_H_
#define CONTACTSOLVER_H_

#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/collobject.h"
#include "rigidbody/common/contact.h"

struct ContactSolverConfig {
	f32 timeStep;
	f32 separateBias;
	bool deformMeshEnable;
};

typedef void (*PreResponse)(ContactPair& pair, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const ContactSolverConfig& config);
typedef void (*ApplyImpulse)(ContactPair& pair, TrbState& stateA, TrbState& stateB, const ContactSolverConfig& config);
void preResponseFixAndMov(ContactPair& pair, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const ContactSolverConfig& config);
void preResponseMovAndFix(ContactPair& pair, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const ContactSolverConfig& config);
void preResponseMovAndMov(ContactPair& pair, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const ContactSolverConfig& config);
void preResponseFixAndFix(ContactPair& pair, TrbState& stateA, TrbDynBody& bodyA, TrbState& stateB, TrbDynBody& bodyB, const ContactSolverConfig& config);
void applyImpulseMovAndMov(ContactPair& pair, TrbState& stateA, TrbState& stateB, const ContactSolverConfig& config);

#include "rigidbody/common/constraintrowsolver.h"

static inline void applyImpulseMovAndMovInline(ContactPair& pair, TrbState& stateA, TrbState& stateB, const ContactSolverConfig& config)
{
	(void)config;

	Matrix3 inertiaInvA(pair.getInertiaInvA());
	Matrix3 inertiaInvB(pair.getInertiaInvB());

	Quat qA(stateA.getOrientation());
	Quat qB(stateB.getOrientation());

	Vector3 dLVA(stateA.getDeltaLinearVelocity());
	Vector3 dLVB(stateB.getDeltaLinearVelocity());
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	Vector3 dAVB(stateB.getDeltaAngularVelocity());

	for(s32 c=0;c < pair.numContacts;c++) {

		ContactPoint& cp = pair.contactPoints[c];

		f32 accumImpulse0 = cp.constraints[0].accumImpulse;
		f32 accumImpulse1 = cp.constraints[1].accumImpulse;
		f32 accumImpulse2 = cp.constraints[2].accumImpulse;
		f32 lowerLimit0 = cp.constraints[0].lowerLimit;
		f32 upperLimit0 = cp.constraints[0].upperLimit;

		Vector3 rA = rotate(qA, cp.getLocalPointA());
		Vector3 rB = rotate(qB, cp.getLocalPointB());

		solveLinearConstraintRowMovAndMovImpulse(dLVA, dLVB, dAVA, dAVB, accumImpulse0, cp.constraints[0], lowerLimit0, upperLimit0, stateA, pair.getMassInvA(), inertiaInvA, rA, stateB, pair.getMassInvB(), inertiaInvB, rB);

		// Update Friction Limit
		f32 mf = pair.compositeFriction*fabsf(accumImpulse0);
		f32 lowerLimit1 = -mf;
		f32 upperLimit1 =  mf;
		f32 lowerLimit2 = -mf;
		f32 upperLimit2 =  mf;

		solveLinearConstraintRowMovAndMovImpulse(dLVA, dLVB, dAVA, dAVB, accumImpulse1, cp.constraints[1], lowerLimit1, upperLimit1, stateA, pair.getMassInvA(), inertiaInvA, rA, stateB, pair.getMassInvB(), inertiaInvB, rB);

		solveLinearConstraintRowMovAndMovImpulse(dLVA, dLVB, dAVA, dAVB, accumImpulse2, cp.constraints[2], lowerLimit2, upperLimit2, stateA, pair.getMassInvA(), inertiaInvA, rA, stateB, pair.getMassInvB(), inertiaInvB, rB);

		cp.constraints[0].accumImpulse = accumImpulse0;
		cp.constraints[1].accumImpulse = accumImpulse1;
		cp.constraints[2].accumImpulse = accumImpulse2;
		cp.constraints[1].lowerLimit = lowerLimit1;
		cp.constraints[1].upperLimit = upperLimit1;
		cp.constraints[2].lowerLimit = lowerLimit2;
		cp.constraints[2].upperLimit = upperLimit2;

	}

	stateA.setDeltaLinearVelocity(dLVA);
	stateA.setDeltaAngularVelocity(dAVA);
	stateB.setDeltaLinearVelocity(dLVB);
	stateB.setDeltaAngularVelocity(dAVB);

}

#endif /* CONTACTSOLVER_H_ */
