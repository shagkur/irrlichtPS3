/*
 * heightfield.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/heightfield.h"

///////////////////////////////////////////////////////////////////////////////
// Height Field Class

HeightField::HeightField() : fieldScale(1.0f)
{
	fieldData.numBlockX = 0;
	fieldData.numBlockZ = 0;
	fieldData.numFieldX = 0;
	fieldData.numFieldZ = 0;
	fieldData.fieldDepth = 0;
	fieldData.fieldWidth = 0;
	fieldData.maxHeight = fieldData.minHeight = 0.0f;
}

#ifndef __SPU__

void HeightField::setHeightFieldData(const HeightFieldData &field)
{
	fieldData = field;
}

#endif

