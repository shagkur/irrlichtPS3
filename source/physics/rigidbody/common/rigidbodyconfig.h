/*
 * rigidbodyconfig.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef RIGIDBODYCONFIG_H_
#define RIGIDBODYCONFIG_H_

//---------------------------------------------------------------------------
// Rigid Body Dynamics
//---------------------------------------------------------------------------

// Rigid Body
#define NUMPRIMS					64

// Simulation Settings
#define JOINT_IMPULSE_EPSILON		0.0001f
#define NUMCONTACTS_PER_BODIES		4
#define CONTACT_THRESHOLD_NORMAL	0.01f
#define CONTACT_THRESHOLD_TANGENT	0.01f
#define CONTACT_BATCH				16
#define CCD_THRESHOLD_MIN			0.01f
#define CCD_THRESHOLD_MAX			0.25f
#define CCD_ENABLE_DISTANCE			0.5f
#define CONTACT_SLOP				0.001f
#define JOINT_LIN_SLOP				0.01f
#define JOINT_ANG_SLOP				0.01f

#define ODE_RUNGEKUTTA

// Height Field
#define BLOCK_SIZE					16
#define HEIGHTFIELD_CACHE_COUNT 	4
#define BLOCK_SIZE_B 				(BLOCK_SIZE+1)

#endif /* RIGIDBODYCONFIG_H_ */
