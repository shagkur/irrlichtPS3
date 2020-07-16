/*
 * sphere.h
 *
 *  Created on: Apr 26, 2013
 *      Author: mike
 */

#ifndef SPHERE_H_
#define SPHERE_H_

#include "rigidbody/common/box.h"

class Sphere
{
public:
	f32 radius;

	Sphere() {}
	Sphere(f32 r) : radius(r) {}

	void set(f32 r) { radius = r; }
	Vector3 getAABB();
};

inline Vector3 Sphere::getAABB()
{
	return Vector3(radius);
}

class SpherePoint
{
public:
	SpherePoint() : localPoint(0.0f) {}

	Point3 localPoint;
	FeatureType featureType;
	s32 featureIdx;

	inline void setVertexFeature();
};

inline void SpherePoint::setVertexFeature()
{
	featureType = V;
	featureIdx = 0;
}

#endif /* SPHERE_H_ */
