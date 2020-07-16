/*
 * contactcache.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/contactcache.h"

s32 ContactCache::findNearestContactPoint(const Vector3& newPoint)
{
	s32 nearestIdx = -1;
	for(u32 i=0;i < numContacts;i++) {
		Vector3 dist = pointA[i] - newPoint;
		f32 diff = lengthSqr(dist);
		if(diff < CONTACT_THRESHOLD)
			nearestIdx = i;
	}
	return nearestIdx;
}

s32 ContactCache::sort4ContactPoints(const Vector3& newPoint, f32 newDistance)
{
	s32 maxPenetrationIndex = -1;
	f32 maxPenetration = newDistance;

	for(s32 i=0;i < CONTACT_CACHE_SIZE;i++) {
		if(distance[i] < maxPenetration) {
			maxPenetrationIndex = i;
			maxPenetration = distance[i];
		}
	}

	f32 res[4] = {0.0f};

	if(maxPenetrationIndex != 0) {
		Vector3 a0 = newPoint - pointA[1];
		Vector3 b0 = pointA[3] - pointA[2];
		res[0] = lengthSqr(cross(a0, b0));
	}

	if(maxPenetrationIndex != 1) {
		Vector3 a1 = newPoint - pointA[0];
		Vector3 b1 = pointA[3] - pointA[2];
		res[1] = lengthSqr(cross(a1, b1));
	}

	if(maxPenetrationIndex != 2) {
		Vector3 a2 = newPoint - pointA[0];
		Vector3 b2 = pointA[3] - pointA[1];
		res[2] = lengthSqr(cross(a2, b2));
	}

	if(maxPenetrationIndex != 3) {
		Vector3 a3 = newPoint - pointA[0];
		Vector3 b3 = pointA[2] - pointA[1];
		res[3] = lengthSqr(cross(a3, b3));
	}

	s32 maxIndex = 0;
	f32 maxVal = res[0];

	if(res[1] > maxVal) {
		maxIndex = 1;
		maxVal = res[1];
	}

	if(res[2] > maxVal) {
		maxIndex = 2;
		maxVal = res[2];
	}

	if(res[3] > maxVal) {
		maxIndex = 3;
		maxVal = res[3];
	}

	return maxIndex;
}

void ContactCache::add(f32 newDistance, const Vector3& newNormal, const Vector3& newPointA, const Vector3& newPointB, u32 newInfo)
{
	s32 replaceId = findNearestContactPoint(newPointA);
	if(replaceId >= 0 && distance[replaceId] < newDistance)
		return;
	else {
		if(numContacts < 4)
			replaceId = numContacts++;
		else
			replaceId = sort4ContactPoints(newPointA, newDistance);
	}
	distance[replaceId] = newDistance;
	normal[replaceId] = newNormal;
	pointA[replaceId] = newPointA;
	pointB[replaceId] = newPointB;
	info[replaceId] = newInfo;
}
