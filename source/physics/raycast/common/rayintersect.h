/*
 * rayintersect.h
 *
 *  Created on: Jun 7, 2013
 *      Author: mike
 */

#ifndef RAYINTERSECT_H_
#define RAYINTERSECT_H_

#include "raycast/common/ray.h"
#include "rigidbody/common/collobject.h"

bool rayIntersect(const CollObject& obj, const Transform3& transform, Ray& ray, f32& t);

#endif /* RAYINTERSECT_H_ */
