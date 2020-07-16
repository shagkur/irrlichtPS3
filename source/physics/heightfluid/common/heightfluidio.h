/*
 * heightfluidio.h
 *
 *  Created on: Jun 11, 2013
 *      Author: mike
 */

#ifndef HEIGHTFLUIDIO_H_
#define HEIGHTFLUIDIO_H_

#include "base/common.h"

enum
{
	HEIGHTFLUID_CALCWAVE = 0,
	HEIGHTFLUID_SETMOTION_STAGE1,
	HEIGHTFLUID_SETMOTION_STAGE2,
	HEIGHTFLUID_CALCMESH,
	HEIGHTFLUID_APPLY_FORCES
};

struct IOParamCalcWave
{
	f32	timeStep;
	f32	cConst;
	f32	kConst;
	f32	deltaX;
	f32	heightMin;
	f32	heightMax;
};

struct IOParamSetMotion
{
	f32	reducedWater;
};

struct IOParamCalcSurface
{
	u32	vtxAddr;
	u32	nmlAddr;
	f32	amplitude;
};

struct IOParamApplyForces
{
	u32 commonAddr;
	u32 forcesAddr;
	u32 statesAddr;
	u32 collsAddr;
	u32 startState;
	u32 numStates;
	f32 buoyPower;
	f32 limitHeight;
	f32 amplitude;
	f32 buoyDamping;
	f32 downCurrWave;
	f32 downPrevWave;
};

ATTRIBUTE_ALIGNED16(struct) HeightFluidIO
{
	u32 numSPU;
	u32 debugFlag;
	u32 fieldAddr;
	u32 fieldWidth;
	u32 fieldDepth;
	Vector3 fieldScale;
	Vector3 fieldPosition;
	Quat fieldOrientation;
	union {
		IOParamCalcWave	calcWave;
		IOParamSetMotion setMotion;
		IOParamCalcSurface calcSurface;
		IOParamApplyForces applyForces;
	};
};

#endif /* HEIGHTFLUIDIO_H_ */
