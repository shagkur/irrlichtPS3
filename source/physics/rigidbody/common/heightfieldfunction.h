/*
 * heightfieldfunction.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef HEIGHTFIELDFUNCTION_H_
#define HEIGHTFIELDFUNCTION_H_

#include "base/common.h"

#include "rigidbody/common/heightfield.h"
#include "rigidbody/common/rigidbodyconfig.h"

///////////////////////////////////////////////////////////////////////////////
// Height Field Function

#ifdef __SPU__

void  initializeHeightFieldCache();
void  releaseHeightFieldCache();

#endif

bool getHeight(const HeightField& heightfield, f32 x, f32 z, f32& h);
bool getNormal(const HeightField& heightfield, f32 x, f32 z, Vector3& nml);

#endif /* HEIGHTFIELDFUNCTION_H_ */
