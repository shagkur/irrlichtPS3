/*
 * particlesolver.h
 *
 *  Created on: Jan 28, 2014
 *      Author: mike
 */

#ifndef PARTICLESOLVER_H_
#define PARTICLESOLVER_H_

#include "particle/common/pclcontact.h"
#include "particle/common/pcljoint.h"
#include "particle/common/pclstate.h"

#include "rigidbody/common/trbdynbody.h"
#include "rigidbody/common/trbstatevec.h"

struct PclContactSolverConfig
{
	f32 timeStep;
	f32 separateBias;
};

struct PclJointSolverConfig
{
	f32 timeStep;
};

void applyImpulsePcl(PclContactPair& pair, PclState& particleA, PclState& particleB, const PclContactSolverConfig& config);
void applyImpulseRig(PclContactPair& pair, PclState& particleA, TrbState& rigidbodyB, const PclContactSolverConfig& config, TrbDynBody& bodyB, f32 timeStep, bool twoWayEnable, Vector3& ForceImp, Vector3& RotImp);

void applyJointPcl(PclJoint& joint, PclState& particleA, PclState& particleB, const PclJointSolverConfig& config);
void applyJointRig(PclJoint& joint, PclState& particleA, TrbState& rigidbodyB, const PclJointSolverConfig& config);

#endif /* PARTICLESOLVER_H_ */
