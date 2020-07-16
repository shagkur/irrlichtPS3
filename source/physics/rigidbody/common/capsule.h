/*
 * capsule.h
 *
 *  Created on: Apr 27, 2013
 *      Author: mike
 */

#ifndef CAPSULE_H_
#define CAPSULE_H_

#include "rigidbody/common/box.h"

class Capsule
{
public:
	f32 hLength;
	f32 radius;

	Capsule() {}
	Capsule(f32 hLength_, f32 radius_);

	void set(f32 hLength_, f32 radius_);
	Vector3 getAABB(const Vector3& direction) const;
};

inline Capsule::Capsule(f32 hLength_, f32 radius_)
{
	hLength = hLength_;
	radius = radius_;
}

inline void Capsule::set(f32 hLength_, f32 radius_)
{
	hLength = hLength_;
	radius = radius_;
}

inline Vector3 Capsule::getAABB(const Vector3& direction) const
{
	return absPerElem(direction)*hLength + Vector3(radius);
}

class CapsulePoint
{
public:
	CapsulePoint() : localPoint(0.0f) {}

	Point3 localPoint;
	FeatureType featureType;
	s32 featureIdx;
	f32 lineParam;

	inline void setVertexFeature(s32 plus);
	inline void setEdgeFeature();

	inline void getVertexFeature(s32& plus) const;
};

inline void CapsulePoint::setVertexFeature(s32 plus)
{
	featureType = V;
	featureIdx = plus;
}

inline void CapsulePoint::setEdgeFeature()
{
	featureType = E;
	featureIdx = 0;
}

inline void CapsulePoint::getVertexFeature(s32& plus) const
{
	plus = featureIdx;
}

#endif /* CAPSULE_H_ */
