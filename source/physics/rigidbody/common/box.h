/*
 * box.h
 *
 *  Created on: Apr 26, 2013
 *      Author: mike
 */

#ifndef BOX_H_
#define BOX_H_

#include "base/common.h"

enum FeatureType { F, E, V };

class Box
{
public:
	Vector3 half;

	inline Box() {}
	inline Box(const Vector3& half_);
	inline Box(f32 hx, f32 hy, f32 hz);

	inline void set(const Vector3& half_);
	inline void set(f32 hx, f32 hy, f32 hz);

	inline Vector3 getAABB(const Matrix3& rotation) const;
};

inline Box::Box(const Vector3& half_)
{
	set(half_);
}

inline Box::Box(f32 hx, f32 hy, f32 hz)
{
	set(hx, hy, hz);
}

inline void Box::set(const Vector3& half_)
{
	half = half_;
}

inline void Box::set(f32 hx, f32 hy, f32 hz)
{
	half = Vector3(hx, hy, hz);
}

inline Vector3 Box::getAABB(const Matrix3& rotation) const
{
	return absPerElem(rotation)*half;
}

class BoxPoint
{
public:
	BoxPoint() : localPoint(0.0f) {}

	Point3 localPoint;
	FeatureType featureType;
	s32 featureIdx;

	inline void setVertexFeature(s32 plusX, s32 plusY, s32 plusZ);
	inline void setEdgeFeature(s32 dim0, s32 plus0, s32 dim1, s32 plus1);
	inline void setFaceFeature(s32 dim, s32 plus);

	inline void getVertexFeature(s32& plusX, s32& plusY, s32& plusZ) const;
	inline void getEdgeFeature(s32& dim0, s32& plus0, s32& dim1, s32& plus1) const;
	inline void getFaceFeature(s32& dim, s32& plus) const;
};

inline void BoxPoint::setVertexFeature(s32 plusX, s32 plusY, s32 plusZ)
{
	featureType = V;
	featureIdx = plusX<<2 | plusY<<1 | plusZ;
}

inline void BoxPoint::setEdgeFeature(s32 dim0, s32 plus0, s32 dim1, s32 plus1)
{
	featureType = E;
	if(dim0 > dim1)
		featureIdx = plus1<<5 | dim1<<3 | plus0<<2 | dim0;
	else
		featureIdx = plus0<<5 | dim0<<3 | plus1<<2 | dim1;
}

inline void BoxPoint::setFaceFeature(s32 dim, s32 plus)
{
	featureType = F;
	featureIdx = plus<<2 | dim;
}

inline void BoxPoint::getVertexFeature(s32& plusX, s32& plusY, s32& plusZ) const
{
	plusX = featureIdx>>2;
	plusY = featureIdx>>1&1;
	plusZ = featureIdx&1;
}

inline void BoxPoint::getEdgeFeature(s32& dim0, s32& plus0, s32& dim1, s32& plus1) const
{
	plus0 = featureIdx>>5;
	dim0 = featureIdx>>3&3;
	plus1 = featureIdx>>2&1;
	dim1 = featureIdx&3;
}

inline void BoxPoint::getFaceFeature(s32& dim, s32& plus) const
{
	plus = featureIdx>>2;
	dim = featureIdx&3;
}

#endif /* BOX_H_ */
