/*
 * worldvolume.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef WORLDVOLUME_H_
#define WORLDVOLUME_H_

#include "base/common.h"
#include "base/vecint3.h"

ATTRIBUTE_ALIGNED16(class) WorldVolume {
public:
	Vector3 origin;
	Vector3 extent;

	inline void	setWorldSize(const Vector3& origin_, const Vector3& extent_);
	inline Vector3 localToWorldPosition(const VecInt3& localPosition);
	inline VecInt3 worldToLocalPosition(const Vector3& worldPosition);
	inline void worldToLocalPosition(const Vector3& worldMinPosition, const Vector3& worldMaxPosition, VecInt3& localMinPosition, VecInt3& localMaxPosition);
};

inline void WorldVolume::setWorldSize(const Vector3& origin_, const Vector3& extent_)
{
	origin = origin_;
	extent = extent_;
}

inline Vector3 WorldVolume::localToWorldPosition(const VecInt3& localPosition)
{
	const Vector3 sz(65535.0f);
	Vector3 q = divPerElem((Vector3)localPosition, sz);
	return mulPerElem(q, 2.0f*extent) + origin - extent;
}

inline VecInt3 WorldVolume::worldToLocalPosition(const Vector3& worldPosition)
{
	const Vector3 sz(65535.0f);
	Vector3 q = divPerElem(worldPosition - origin + extent, 2.0f*extent);
	q = minPerElem(maxPerElem(q, Vector3(0.0f)), Vector3(1.0f)); // clamp 0.0 - 1.0
	q = mulPerElem(q, sz);
	return VecInt3(q);
}

inline void WorldVolume::worldToLocalPosition(const Vector3& worldMinPosition, const Vector3& worldMaxPosition, VecInt3& localMinPosition, VecInt3& localMaxPosition)
{
	const Vector3 sz(65535.0f);
	Vector3 qmin = divPerElem(worldMinPosition - origin + extent, 2.0f*extent);
	qmin = minPerElem(maxPerElem(qmin, Vector3(0.0f)), Vector3(1.0f)); // clamp 0.0 - 1.0
	qmin = mulPerElem(qmin, sz);
	Vector3 qmax = divPerElem(worldMaxPosition - origin + extent, 2.0f*extent);
	qmax = minPerElem(maxPerElem(qmax, Vector3(0.0f)), Vector3(1.0f)); // clamp 0.0 - 1.0
	qmax = mulPerElem(qmax, sz);
	localMinPosition = VecInt3(floorf(qmin[0]), floorf(qmin[1]), floorf(qmin[2]));
	localMaxPosition = VecInt3(ceilf(qmax[0]), ceilf(qmax[1]), ceilf(qmax[2]));
}

#endif /* WORLDVOLUME_H_ */
