/*
 * parallelgroup.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef PARALLELGROUP_H_
#define PARALLELGROUP_H_

#include "base/common.h"

///////////////////////////////////////////////////////////////////////////////
// Parallel Pair Group

#define MAX_SOLVER_PHASES 			64
#define MAX_SOLVER_GROUPS 			32
#define MAX_SOLVER_PAIRS  			64
#define MIN_SOLVER_PAIRS  			16

ATTRIBUTE_ALIGNED128(struct) SolverGroup
{
	u16 pairIndices[MAX_SOLVER_PAIRS];
};

ATTRIBUTE_ALIGNED16(struct) SolverInfo
{
	u16 numPhases;
	u16 numGroups[MAX_SOLVER_PHASES];
	u16 numPairs[MAX_SOLVER_PHASES*MAX_SOLVER_GROUPS];
};

#endif /* PARALLELGROUP_H_ */
