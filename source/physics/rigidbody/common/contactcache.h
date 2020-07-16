/*
 * contactcache.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef CONTACTCACHE_H_
#define CONTACTCACHE_H_

#include "base/common.h"

#include "rigidbody/common/subdata.h"

#define CONTACT_CACHE_SIZE 				4
#define CONTACT_THRESHOLD  				0.002f

class ContactCache
{
private:
	u32 numContacts;
	u32 info[CONTACT_CACHE_SIZE];
	f32   distance[CONTACT_CACHE_SIZE];
	Vector3 normal[CONTACT_CACHE_SIZE];
	Vector3 pointA[CONTACT_CACHE_SIZE];
	Vector3 pointB[CONTACT_CACHE_SIZE];

	int findNearestContactPoint(const Vector3 &newPoint);
	int sort4ContactPoints(const Vector3 &newPoint,f32 newDistance);

public:
	ContactCache() : numContacts(0)
	{
	}

	void add(f32 newDistance, const Vector3& newNormal, const Vector3& newPointA, const Vector3& newPointB, u32 newInfo);

	u32 getNumContacts() {return numContacts;}

	u32 getInfo(s32 i) {return info[i];}
	f32    getDistance(s32 i) const {return distance[i];}
	Vector3  getNormal(s32 i) const {return normal[i];}
	Vector3  getPointA(s32 i) const {return pointA[i];}
	Vector3  getPointB(s32 i) const {return pointB[i];}
};

#endif /* CONTACTCACHE_H_ */
