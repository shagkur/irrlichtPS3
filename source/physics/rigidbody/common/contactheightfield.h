/*
 * contactheightfield.h
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#ifndef CONTACTHEIGHTFIELD_H_
#define CONTACTHEIGHTFIELD_H_

#include "rigidbody/common/heightfield.h"

bool contactHeightField(const HeightField *heightfield, const Point3& checkPoint, Point3& fieldPoint, Vector3& fieldNormal, f32& dist);

#endif /* CONTACTHEIGHTFIELD_H_ */
