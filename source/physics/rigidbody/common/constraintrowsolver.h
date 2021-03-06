/*
 * constraintrowsolver.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef CONSTRAINTROWSOLVER_H_
#define CONSTRAINTROWSOLVER_H_

#include "base/simdfunc.h"

#include "rigidbody/common/trbstatevec.h"
#include "rigidbody/common/constraintcache.h"

///////////////////////////////////////////////////////////////////////////////
// Constraint Row Solver

static inline void solveLinearConstraintRowMovAndMovImpulse(Vector3& dLVA, Vector3& dLVB, Vector3& dAVA, Vector3& dAVB, f32& accumImpulse, ConstraintCache& constraint, const f32& lowerLimit, const f32& upperLimit, TrbState& stateA, const f32& massInvA, const Matrix3& inertiaInvA, const Vector3& rA, TrbState& stateB, const f32& massInvB, const Matrix3& inertiaInvB, const Vector3& rB)
{
	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	Vector3 dVA = dLVA + cross(dAVA, rA);
	Vector3 dVB = dLVB + cross(dAVB, rB);
	deltaImpulse -= constraint.jacDiagInv*dot(normal, dVA - dVB);
	f32 oldImpulse = accumImpulse;
	accumImpulse = CLAMPF(oldImpulse + deltaImpulse, lowerLimit, upperLimit);
	deltaImpulse = accumImpulse - oldImpulse;
	dLVA = dLVA + deltaImpulse*massInvA*normal;
	dAVA = dAVA + deltaImpulse*inertiaInvA*cross(rA, normal);
	dLVB = dLVB - deltaImpulse*massInvB*normal;
	dAVB = dAVB - deltaImpulse*inertiaInvB*cross(rB, normal);
}

static inline void solveLinearConstraintRowMovAndMovDelta(Vector3& dLVA, Vector3& dLVB, Vector3& dAVA, Vector3& dAVB, f32& accumImpulse, ConstraintCache& constraint, f32 massInvA, const Matrix3& inertiaInvA, const Vector3& rA, f32 massInvB, const Matrix3& inertiaInvB, const Vector3& rB)
{
	const Vector3 normal(read_Vector3(constraint.normal));
	Vector3 dVA = dLVA + cross(dAVA, rA);
	Vector3 dVB = dLVB + cross(dAVB, rB);
	f32 deltaImpulse = constraint.rhs - constraint.jacDiagInv*dot(normal, dVA - dVB);
	f32 oldImpulse = accumImpulse;
	accumImpulse = CLAMPF(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = accumImpulse - oldImpulse;
	dLVA = (dLVA + deltaImpulse*massInvA*normal);
	dAVA = (dAVA + deltaImpulse*inertiaInvA*cross(rA, normal));
	dLVB = (dLVB - deltaImpulse*massInvB*normal);
	dAVB = (dAVB - deltaImpulse*inertiaInvB*cross(rB, normal));
}

static inline void solveLinearConstraintRowMovAndMov(ConstraintCache& constraint, TrbState& stateA, f32 massInvA, const Matrix3& inertiaInvA,const Vector3& rA, TrbState& stateB, f32 massInvB, const Matrix3& inertiaInvB, const Vector3& rB)
{
#ifdef __SPU__
	const qword cX000Mask = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
	qword v_normal = si_lqd(si_from_ptr(constraint.normal), 0);
	qword v_deltaImpulse = si_and(si_rotqbyi(v_normal, 12 ), cX000Mask);
	Vector3 dLVA(stateA.getDeltaLinearVelocity());
	Vector3 dLVB(stateB.getDeltaLinearVelocity());
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	Vector3 dVA = dLVA + cross(dAVA, rA);
	Vector3 dVB = dLVB + cross(dAVB, rB);

	qword v_jacDiagInv = si_lqd(si_from_ptr(constraint.normal), 16);
	v_deltaImpulse = si_fs(v_deltaImpulse,
						   si_fm( si_and(v_jacDiagInv, cX000Mask), (qword)_vmathVfDot3((vec_float4)v_normal, (dVA - dVB).get128()))
						  );
	qword v_lowerLimit = si_and(si_rotqbyi(v_jacDiagInv, 4 ), cX000Mask);
	qword v_upperLimit = si_and(si_rotqbyi(v_jacDiagInv, 8 ), cX000Mask);
	qword v_oldImpulse = si_and(si_rotqbyi(v_jacDiagInv, 12 ), cX000Mask);
	qword v_newImpulse = si_fa(v_oldImpulse, v_deltaImpulse);
	qword upperMask = si_fcgt(v_newImpulse, v_upperLimit); // 1 means new > upperLimit
	qword lowerMask = si_fcgt(v_lowerLimit, v_newImpulse); // 1 means lowerLimit > new
	v_newImpulse = si_selb(v_newImpulse, v_upperLimit, upperMask);
	v_newImpulse = si_selb(v_newImpulse, v_lowerLimit, lowerMask);
	qword c000Xmask = si_rotqbyi(cX000Mask, 4);
	v_jacDiagInv = si_or(si_and(v_jacDiagInv, si_nor(c000Xmask, c000Xmask)), si_rotqbyi(v_newImpulse, 4));
	si_stqd(v_jacDiagInv, si_from_ptr(constraint.normal), 16);
	f32 deltaImpulse = si_to_float(si_fs(v_newImpulse, v_oldImpulse));

	stateA.setDeltaLinearVelocity(dLVA + deltaImpulse*massInvA*Vector3((vec_float4)v_normal));
	stateA.setDeltaAngularVelocity(dAVA + deltaImpulse*inertiaInvA*cross(rA, Vector3((vec_float4)v_normal)));
	stateB.setDeltaLinearVelocity(dLVB - deltaImpulse*massInvB*Vector3((vec_float4)v_normal));
	stateB.setDeltaAngularVelocity(dAVB - deltaImpulse*inertiaInvB*cross(rB, Vector3((vec_float4)v_normal)));
#else
	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	Vector3 dLVA(stateA.getDeltaLinearVelocity());
	Vector3 dLVB(stateB.getDeltaLinearVelocity());
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	Vector3 dVA = dLVA + cross(dAVA, rA);
	Vector3 dVB = dLVB + cross(dAVB, rB);
	deltaImpulse -= constraint.jacDiagInv*dot(normal,dVA-dVB);
	f32 oldImpulse = constraint.accumImpulse;
	constraint.accumImpulse = CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = constraint.accumImpulse - oldImpulse;
	stateA.setDeltaLinearVelocity(dLVA + deltaImpulse*massInvA*normal);
	stateA.setDeltaAngularVelocity(dAVA + deltaImpulse*inertiaInvA*cross(rA, normal));
	stateB.setDeltaLinearVelocity(dLVB - deltaImpulse*massInvB*normal);
	stateB.setDeltaAngularVelocity(dAVB - deltaImpulse*inertiaInvB*cross(rB, normal));
#endif
}

static inline
void solveLinearConstraintRowMovAndFix(ConstraintCache& constraint, TrbState& stateA, f32 massInvA, const Matrix3& inertiaInvA, const Vector3& rA, TrbState& stateB, f32 massInvB, const Matrix3& inertiaInvB,const Vector3& rB)
{
	(void) stateB;
	(void) massInvB;
	(void) inertiaInvB;
	(void) rB;
#ifdef __SPU__
	const qword cX000Mask = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
	qword v_normal = si_lqd(si_from_ptr(constraint.normal), 0);
	qword v_deltaImpulse = si_and(si_rotqbyi(v_normal, 12 ), cX000Mask);
	Vector3 dLVA(stateA.getDeltaLinearVelocity());
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	Vector3 dVA = dLVA + cross(dAVA, rA);

	qword v_jacDiagInv = si_lqd(si_from_ptr(constraint.normal), 16);
	v_deltaImpulse = si_fs(v_deltaImpulse,
						   si_fm(si_and(v_jacDiagInv, cX000Mask), (qword)_vmathVfDot3((vec_float4)v_normal, dVA.get128()))
						  );
	qword v_lowerLimit = si_and(si_rotqbyi(v_jacDiagInv, 4 ), cX000Mask);
	qword v_upperLimit = si_and(si_rotqbyi(v_jacDiagInv, 8 ), cX000Mask);
	qword v_oldImpulse = si_and(si_rotqbyi(v_jacDiagInv, 12 ), cX000Mask);
	qword v_newImpulse = si_fa(v_oldImpulse, v_deltaImpulse);
	qword upperMask = si_fcgt(v_newImpulse, v_upperLimit); // 1 means new > upperLimit
	qword lowerMask = si_fcgt(v_lowerLimit, v_newImpulse); // 1 means lowerLimit > new
	v_newImpulse = si_selb(v_newImpulse, v_upperLimit, upperMask);
	v_newImpulse = si_selb(v_newImpulse, v_lowerLimit, lowerMask);
	qword c000Xmask = si_rotqbyi(cX000Mask, 4);
	v_jacDiagInv = si_or(si_and(v_jacDiagInv, si_nor(c000Xmask, c000Xmask)), si_rotqbyi(v_newImpulse, 4));
	si_stqd( v_jacDiagInv, si_from_ptr(constraint.normal), 16);
	f32 deltaImpulse = si_to_float(si_fs( v_newImpulse, v_oldImpulse) );
	stateA.setDeltaLinearVelocity(dLVA + deltaImpulse*massInvA*Vector3((vec_float4)v_normal));
	stateA.setDeltaAngularVelocity(dAVA + deltaImpulse*inertiaInvA*cross(rA, Vector3((vec_float4)v_normal)));
#else
	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	Vector3 dLVA(stateA.getDeltaLinearVelocity());
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	Vector3 dVA = dLVA + cross(dAVA, rA);
	deltaImpulse -= constraint.jacDiagInv*dot(normal, dVA);

	f32 oldImpulse = constraint.accumImpulse;
	constraint.accumImpulse = PFX_CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = constraint.accumImpulse - oldImpulse;

	stateA.setDeltaLinearVelocity(dLVA + deltaImpulse*massInvA*normal);
	stateA.setDeltaAngularVelocity(dAVA + deltaImpulse*inertiaInvA*cross(rA, normal));
#endif
}

static inline void solveLinearConstraintRowFixAndMovDelta(Vector3& dLVB, Vector3& dAVB,f32& accumImpulse, ConstraintCache& constraint, f32 massInvA, const Matrix3& inertiaInvA, const Vector3& rA, f32 massInvB, const Matrix3& inertiaInvB, const Vector3& rB)
{
	(void) massInvA;
	(void) inertiaInvA;
	(void) rA;
	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	Vector3 dVB = dLVB + cross(dAVB, rB);
	deltaImpulse -= constraint.jacDiagInv*dot(normal, -dVB);

	f32 oldImpulse = accumImpulse;
	accumImpulse = CLAMPF(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = accumImpulse - oldImpulse;

	dLVB = (dLVB - deltaImpulse*massInvB*normal);
	dAVB = (dAVB - deltaImpulse*inertiaInvB*cross(rB, normal));
}


static inline void solveLinearConstraintRowFixAndMov(ConstraintCache& constraint, TrbState& stateA, f32 massInvA, const Matrix3& inertiaInvA, const Vector3& rA, TrbState& stateB, f32 massInvB,const Matrix3& inertiaInvB,const Vector3& rB)
{
	(void) stateA;
	(void) massInvA;
	(void) inertiaInvA;
	(void) rA;
#ifdef __SPU__
	const qword cX000Mask = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
	qword v_normal = si_lqd(si_from_ptr(constraint.normal), 0);
	qword v_deltaImpulse = si_and(si_rotqbyi(v_normal, 12 ), cX000Mask);
	Vector3 dLVB(stateB.getDeltaLinearVelocity());
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	Vector3 dVB = dLVB + cross(dAVB, rB);

	qword v_jacDiagInv = si_lqd(si_from_ptr(constraint.normal), 16);
	v_deltaImpulse = si_fs(v_deltaImpulse,
						   si_fm(si_and(v_jacDiagInv, cX000Mask), (qword)_vmathVfDot3((vec_float4)v_normal, (-dVB).get128()))
						  );
	qword v_lowerLimit = si_and(si_rotqbyi(v_jacDiagInv, 4 ), cX000Mask);
	qword v_upperLimit = si_and(si_rotqbyi(v_jacDiagInv, 8 ), cX000Mask);
	qword v_oldImpulse = si_and(si_rotqbyi(v_jacDiagInv, 12 ), cX000Mask);
	qword v_newImpulse = si_fa(v_oldImpulse, v_deltaImpulse);
	qword upperMask = si_fcgt(v_newImpulse, v_upperLimit); // 1 means new > upperLimit
	qword lowerMask = si_fcgt(v_lowerLimit, v_newImpulse); // 1 means lowerLimit > new
	v_newImpulse = si_selb(v_newImpulse, v_upperLimit, upperMask);
	v_newImpulse = si_selb(v_newImpulse, v_lowerLimit, lowerMask);
	qword c000Xmask = si_rotqbyi(cX000Mask, 4);
	v_jacDiagInv = si_or(si_and(v_jacDiagInv, si_nor(c000Xmask, c000Xmask)), si_rotqbyi(v_newImpulse, 4));
	si_stqd(v_jacDiagInv, si_from_ptr(constraint.normal), 16);
	f32 deltaImpulse = si_to_float(si_fs( v_newImpulse, v_oldImpulse) );
	stateB.setDeltaLinearVelocity(dLVB - deltaImpulse*massInvB*Vector3((vec_float4)v_normal));
	stateB.setDeltaAngularVelocity(dAVB - deltaImpulse*inertiaInvB*cross(rB, Vector3((vec_float4)v_normal)));
#else
	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	Vector3 dLVB(stateB.getDeltaLinearVelocity());
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	Vector3 dVB = dLVB + cross(dAVB, rB);
	deltaImpulse -= constraint.jacDiagInv*dot(normal, -dVB);

	f32 oldImpulse = constraint.accumImpulse;
	constraint.accumImpulse = CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = constraint.accumImpulse - oldImpulse;

	stateB.setDeltaLinearVelocity(dLVB - deltaImpulse*massInvB*normal);
	stateB.setDeltaAngularVelocity(dAVB - deltaImpulse*inertiaInvB*cross(rB, normal));
#endif
}

static inline void solveAngularConstraintRowMovAndMovDelta(Vector3& dAVA, Vector3& dAVB,f32& accumImpulse, ConstraintCache& constraint, const Matrix3& inertiaInvA, const Matrix3& inertiaInvB)
{
	const Vector3 normal(read_Vector3(constraint.normal));
  	f32 deltaImpulse = constraint.rhs - constraint.jacDiagInv*dot(normal,dAVA - dAVB);

	f32 oldImpulse = accumImpulse;
	accumImpulse = CLAMPF(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = accumImpulse - oldImpulse;

	dAVA = (dAVA + deltaImpulse*inertiaInvA*normal);
	dAVB = (dAVB - deltaImpulse*inertiaInvB*normal);
}

static inline void solveAngularConstraintRowMovAndMov(ConstraintCache& constraint, TrbState& stateA, const Matrix3& inertiaInvA, TrbState& stateB, const Matrix3& inertiaInvB)
{
#ifdef __SPU__
	const qword cX000Mask = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
	qword v_normal = si_lqd(si_from_ptr(constraint.normal), 0);
	qword v_deltaImpulse = si_and(si_rotqbyi(v_normal, 12 ), cX000Mask);
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	qword v_jacDiagInv = si_lqd(si_from_ptr(constraint.normal), 16);
	v_deltaImpulse = si_fs(v_deltaImpulse,
						   si_fm(si_and(v_jacDiagInv, cX000Mask), (qword)_vmathVfDot3((vec_float4)v_normal, (dAVA-dAVB).get128()))
						  );
	qword v_lowerLimit = si_and(si_rotqbyi(v_jacDiagInv, 4 ), cX000Mask);
	qword v_upperLimit = si_and(si_rotqbyi(v_jacDiagInv, 8 ), cX000Mask);
	qword v_oldImpulse = si_and(si_rotqbyi(v_jacDiagInv, 12 ), cX000Mask);
	qword v_newImpulse = si_fa(v_oldImpulse, v_deltaImpulse);
	qword upperMask = si_fcgt(v_newImpulse, v_upperLimit); // 1 means new > upperLimit
	qword lowerMask = si_fcgt(v_lowerLimit, v_newImpulse); // 1 means lowerLimit > new
	v_newImpulse = si_selb(v_newImpulse, v_upperLimit, upperMask);
	v_newImpulse = si_selb(v_newImpulse, v_lowerLimit, lowerMask);
	qword c000Xmask = si_rotqbyi(cX000Mask, 4);
	v_jacDiagInv = si_or(si_and(v_jacDiagInv, si_nor(c000Xmask, c000Xmask)), si_rotqbyi(v_newImpulse, 4));
	si_stqd(v_jacDiagInv, si_from_ptr(constraint.normal), 16);
	f32 deltaImpulse = si_to_float(si_fs(v_newImpulse, v_oldImpulse) );
	stateA.setDeltaAngularVelocity(dAVA + deltaImpulse*inertiaInvA*Vector3((vec_float4)v_normal));
	stateB.setDeltaAngularVelocity(dAVB - deltaImpulse*inertiaInvB*Vector3((vec_float4)v_normal));
#else
	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	deltaImpulse -= constraint.jacDiagInv*dot(normal,dAVA - dAVB);

	f32 oldImpulse = constraint.accumImpulse;
	constraint.accumImpulse = CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = constraint.accumImpulse - oldImpulse;

	stateA.setDeltaAngularVelocity(dAVA + deltaImpulse*inertiaInvA*normal);
	stateB.setDeltaAngularVelocity(dAVB - deltaImpulse*inertiaInvB*normal);
#endif
}

static inline void solveAngularConstraintRowMovAndFix(ConstraintCache& constraint, TrbState& stateA, const Matrix3& inertiaInvA, TrbState& stateB, const Matrix3& inertiaInvB)
{
	(void) stateB;
	(void) inertiaInvB;

#ifdef __SPU__
	const qword cX000Mask = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
	qword v_normal = si_lqd(si_from_ptr(constraint.normal), 0);
	qword v_deltaImpulse = si_and(si_rotqbyi(v_normal, 12 ), cX000Mask);
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	qword v_jacDiagInv = si_lqd(si_from_ptr(constraint.normal), 16);
	v_deltaImpulse = si_fs(v_deltaImpulse,
						   si_fm(si_and(v_jacDiagInv, cX000Mask), (qword)_vmathVfDot3((vec_float4)v_normal, dAVA.get128()))
						  );
	qword v_lowerLimit = si_and(si_rotqbyi(v_jacDiagInv, 4 ), cX000Mask);
	qword v_upperLimit = si_and(si_rotqbyi(v_jacDiagInv, 8 ), cX000Mask);
	qword v_oldImpulse = si_and(si_rotqbyi(v_jacDiagInv, 12 ), cX000Mask);
	qword v_newImpulse = si_fa(v_oldImpulse, v_deltaImpulse);
	qword upperMask = si_fcgt(v_newImpulse, v_upperLimit); // 1 means new > upperLimit
	qword lowerMask = si_fcgt(v_lowerLimit, v_newImpulse); // 1 means lowerLimit > new
	v_newImpulse = si_selb(v_newImpulse, v_upperLimit, upperMask);
	v_newImpulse = si_selb(v_newImpulse, v_lowerLimit, lowerMask);
	qword c000Xmask = si_rotqbyi(cX000Mask, 4);
	v_jacDiagInv = si_or(si_and(v_jacDiagInv, si_nor(c000Xmask, c000Xmask)), si_rotqbyi(v_newImpulse, 4));
	si_stqd(v_jacDiagInv, si_from_ptr(constraint.normal), 16);
	f32 deltaImpulse = si_to_float(si_fs(v_newImpulse, v_oldImpulse) );
	stateA.setDeltaAngularVelocity(dAVA + deltaImpulse*inertiaInvA*Vector3((vec_float4)v_normal));
#else
	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	Vector3 dAVA(stateA.getDeltaAngularVelocity());
	deltaImpulse -= constraint.jacDiagInv*dot(normal,dAVA);

	f32 oldImpulse = constraint.accumImpulse;
	constraint.accumImpulse = CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = constraint.accumImpulse - oldImpulse;

	stateA.setDeltaAngularVelocity(dAVA + deltaImpulse*inertiaInvA*normal);
#endif
}

static inline void solveAngularConstraintRowFixAndMovDelta(Vector3& dAVB,f32& accumImpulse, ConstraintCache& constraint, const Matrix3& inertiaInvA, const Matrix3& inertiaInvB)
{
	(void) inertiaInvA;

	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	deltaImpulse -= constraint.jacDiagInv*dot(normal, -dAVB);

	f32 oldImpulse = accumImpulse;
	accumImpulse = CLAMPF(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = accumImpulse - oldImpulse;

	dAVB = (dAVB - deltaImpulse*inertiaInvB*normal);
}

static inline void solveAngularConstraintRowFixAndMov(ConstraintCache& constraint, TrbState& stateA, const Matrix3& inertiaInvA, TrbState& stateB, const Matrix3& inertiaInvB)
{
	(void) stateA;
	(void) inertiaInvA;

#ifdef __SPU__
	const qword cX000Mask = ((qword)(vec_uchar16){0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
	qword v_normal = si_lqd(si_from_ptr(constraint.normal), 0);
	qword v_deltaImpulse = si_and(si_rotqbyi(v_normal, 12 ), cX000Mask);
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	qword v_jacDiagInv = si_lqd(si_from_ptr(constraint.normal), 16);
	v_deltaImpulse = si_fs(v_deltaImpulse,
						   si_fm(si_and(v_jacDiagInv, cX000Mask), (qword)_vmathVfDot3((vec_float4)v_normal, (-dAVB).get128()))
						  );
	qword v_lowerLimit = si_and(si_rotqbyi(v_jacDiagInv, 4 ), cX000Mask);
	qword v_upperLimit = si_and(si_rotqbyi(v_jacDiagInv, 8 ), cX000Mask);
	qword v_oldImpulse = si_and(si_rotqbyi(v_jacDiagInv, 12 ), cX000Mask);
	qword v_newImpulse = si_fa(v_oldImpulse, v_deltaImpulse);
	qword upperMask = si_fcgt(v_newImpulse, v_upperLimit); // 1 means new > upperLimit
	qword lowerMask = si_fcgt(v_lowerLimit, v_newImpulse); // 1 means lowerLimit > new
	v_newImpulse = si_selb(v_newImpulse, v_upperLimit, upperMask);
	v_newImpulse = si_selb(v_newImpulse, v_lowerLimit, lowerMask);
	qword c000Xmask = si_rotqbyi(cX000Mask, 4);
	v_jacDiagInv = si_or(si_and(v_jacDiagInv, si_nor(c000Xmask, c000Xmask)), si_rotqbyi(v_newImpulse, 4));
	si_stqd( v_jacDiagInv, si_from_ptr(constraint.normal), 16);
	f32 deltaImpulse = si_to_float(si_fs( v_newImpulse, v_oldImpulse) );
	stateB.setDeltaAngularVelocity(dAVB - deltaImpulse*inertiaInvB*Vector3((vec_float4)v_normal));
#else
	const Vector3 normal(read_Vector3(constraint.normal));
	f32 deltaImpulse = constraint.rhs;
	Vector3 dAVB(stateB.getDeltaAngularVelocity());
	deltaImpulse -= constraint.jacDiagInv*dot(normal, -dAVB);

	f32 oldImpulse = constraint.accumImpulse;
	constraint.accumImpulse = CLAMP(oldImpulse + deltaImpulse, constraint.lowerLimit, constraint.upperLimit);
	deltaImpulse = constraint.accumImpulse - oldImpulse;

	stateB.setDeltaAngularVelocity(dAVB - deltaImpulse*inertiaInvB*normal);
#endif
}

#endif /* CONSTRAINTROWSOLVER_H_ */
