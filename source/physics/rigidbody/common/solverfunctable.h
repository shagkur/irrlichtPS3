/*
 * solverfunctable.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef SOLVERFUNCTABLE_H_
#define SOLVERFUNCTABLE_H_

/*
	Response Func Table
				FIX			ACTIVE		ACTIVE(S)	KEYFRAME	KEYFRAME(S)	ONEWAY		ONEWAY(S)	TRIGGER
	FIX			FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	ACTIVE		MovAndFix	MovAndMov	MovAndFix	MovAndFix	MovAndFix	FixAndMov	FixAndFix	FixAndFix
	ACTIVE(S)	FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	KEYFRAME	FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	KEYFRAME(S)	FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	ONEWAY		MovAndFix	MovAndFix	MovAndFix	MovAndFix	MovAndFix	FixAndFix	FixAndFix	FixAndFix
	ONEWAY(S)	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix
	TRIGGER		FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix
*/

static PreResponse funcTbl_preResponse[MoveTypeCount][MoveTypeCount] =
{
	{preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseMovAndFix,preResponseMovAndMov,preResponseMovAndFix,preResponseMovAndFix,preResponseMovAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndMov,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseMovAndFix,preResponseMovAndFix,preResponseMovAndFix,preResponseMovAndFix,preResponseMovAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix},
	{preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix,preResponseFixAndFix},
};

/*
	Joint Func Table
				FIX			ACTIVE		ACTIVE(S)	KEYFRAME	KEYFRAME(S)	ONEWAY		ONEWAY(S)	TRIGGER
	FIX			FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	ACTIVE		MovAndFix	MovAndMov	MovAndMov	MovAndFix	MovAndFix	FixAndMov	FixAndMov	FixAndFix
	ACTIVE(S)	FixAndFix	MovAndMov	FixAndFix	MovAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	KEYFRAME	FixAndFix	FixAndMov	FixAndMov	FixAndFix	FixAndFix	FixAndMov	FixAndMov	FixAndFix
	KEYFRAME(S)	FixAndFix	FixAndMov	FixAndFix	FixAndFix	FixAndFix	FixAndMov	FixAndFix	FixAndFix
	ONEWAY		MovAndFix	MovAndFix	MovAndFix	MovAndFix	MovAndFix	MovAndMov	MovAndMov	FixAndFix
	ONEWAY(S)	FixAndFix	MovAndFix	FixAndFix	MovAndFix	FixAndFix	MovAndMov	FixAndFix	FixAndFix
	TRIGGER		FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix	FixAndFix
*/

static PreJoint funcTbl_preJoint[MoveTypeCount][MoveTypeCount] =
{
	{preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix},
	{preJointMovAndFix,preJointMovAndMov,preJointMovAndMov,preJointMovAndFix,preJointMovAndFix,preJointFixAndMov,preJointFixAndMov,preJointFixAndFix},
	{preJointFixAndFix,preJointMovAndMov,preJointFixAndFix,preJointMovAndFix,preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix},
	{preJointFixAndFix,preJointFixAndMov,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix,preJointFixAndMov,preJointFixAndMov,preJointFixAndFix},
	{preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndMov,preJointFixAndFix,preJointFixAndFix},
	{preJointMovAndFix,preJointMovAndFix,preJointMovAndFix,preJointMovAndFix,preJointMovAndFix,preJointMovAndMov,preJointMovAndMov,preJointFixAndFix},
	{preJointFixAndFix,preJointMovAndFix,preJointFixAndFix,preJointMovAndFix,preJointFixAndFix,preJointMovAndMov,preJointFixAndFix,preJointFixAndFix},
	{preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix,preJointFixAndFix},
};

#endif /* SOLVERFUNCTABLE_H_ */
