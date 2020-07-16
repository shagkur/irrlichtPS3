/*
 * rayintersectheightfield.h
 *
 *  Created on: Jun 10, 2013
 *      Author: mike
 */

#ifndef RAYINTERSECTHEIGHTFIELD_H_
#define RAYINTERSECTHEIGHTFIELD_H_

bool rayIntersectHeightField(const CollPrim& prim, const Transform3& transform, Ray& ray, f32& t);

#endif /* RAYINTERSECTHEIGHTFIELD_H_ */
