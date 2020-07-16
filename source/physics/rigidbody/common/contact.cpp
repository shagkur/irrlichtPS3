/*
 * contact.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/contact.h"

s32 findNearestContactPoint(ContactPoint *cp, s32 numContacts, const Vector3& newCP, const Vector3& newNml)
{
	s32 nearestIdx = -1;
	for(s32 i=0;i < numContacts;i++) {
		Vector3 dist = cp[i].getLocalPointA() - newCP;
		f32 diff = lengthSqr(dist);
		if(diff < CONTACT_THRESHOLD_TANGENT && dot(newNml, cp[i].getNormal()) > 0.99f)
			nearestIdx = i;
	}
	return nearestIdx;
}

s32  sort4ContactPoints(ContactPoint *cp, const Vector3& newCP, f32 newDistance)
{
	s32 maxPenetrationIndex = -1;
	f32 maxPenetration = newDistance;

	for(s32 i=0;i < NUMCONTACTS_PER_BODIES;i++) {
		if(cp[i].distance < maxPenetration) {
			maxPenetrationIndex = i;
			maxPenetration = cp[i].distance;
		}
	}

	f32 res[4] = {0.0f};

	if(maxPenetrationIndex != 0) {
		Vector3 a0 = newCP - cp[1].getLocalPointA();
		Vector3 b0 = cp[3].getLocalPointA() - cp[2].getLocalPointA();
		res[0] = lengthSqr(cross(a0, b0));
	}

	if(maxPenetrationIndex != 1) {
		Vector3 a1 = newCP - cp[0].getLocalPointA();
		Vector3 b1 = cp[3].getLocalPointA() - cp[2].getLocalPointA();
		res[1] = lengthSqr(cross(a1, b1));
	}

	if(maxPenetrationIndex != 2) {
		Vector3 a2 = newCP - cp[0].getLocalPointA();
		Vector3 b2 = cp[3].getLocalPointA() - cp[1].getLocalPointA();
		res[2] = lengthSqr(cross(a2, b2));
	}

	if(maxPenetrationIndex != 3) {
		Vector3 a3 = newCP - cp[0].getLocalPointA();
		Vector3 b3 = cp[2].getLocalPointA() - cp[1].getLocalPointA();
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

u32 ContactPair::merge(ContactPair& contactPair)
{
	if(stateIndex[0] != contactPair.stateIndex[0])
		contactPair.exchange();

	u32 ret = 0;

	for(s32 i=0;i < contactPair.numContacts;i++) {
		s32 idx = findNearestContactPoint(contactPoints, numContacts, contactPair.contactPoints[i].getLocalPointA(), contactPair.contactPoints[i].getNormal());

		if(idx >= 0) {
#if 1
			Vector3 nml1(read_Vector3(contactPoints[idx].constraints[0].normal));
			Vector3 nml2(read_Vector3(contactPair.contactPoints[i].constraints[0].normal));
			//float chk = dot(nml1,nml2);
			//if(contactPoints[idx].distance > contactPair.contactPoints[i].distance) {
				if(fabsf(dot(nml1, nml2)) > 0.99f ) {
					contactPoints[idx].distance = contactPair.contactPoints[i].distance;
					contactPoints[idx].localPointA[0] = contactPair.contactPoints[i].localPointA[0];
					contactPoints[idx].localPointA[1] = contactPair.contactPoints[i].localPointA[1];
					contactPoints[idx].localPointA[2] = contactPair.contactPoints[i].localPointA[2];
					contactPoints[idx].localPointB[0] = contactPair.contactPoints[i].localPointB[0];
					contactPoints[idx].localPointB[1] = contactPair.contactPoints[i].localPointB[1];
					contactPoints[idx].localPointB[2] = contactPair.contactPoints[i].localPointB[2];
					contactPoints[idx].constraints[0].normal[0] = contactPair.contactPoints[i].constraints[0].normal[0];
					contactPoints[idx].constraints[0].normal[1] = contactPair.contactPoints[i].constraints[0].normal[1];
					contactPoints[idx].constraints[0].normal[2] = contactPair.contactPoints[i].constraints[0].normal[2];
				} else {
					contactPoints[idx] = contactPair.contactPoints[i];
				}
			//}
#else
			if(contactPoints[idx].distance > contactPair.contactPoints[i].distance) {
				u8 d = contactPoints[idx].duration;
				f32 accumN  = contactPoints[idx].constraints[0].accumImpulse;
				f32 accumF1 = contactPoints[idx].constraints[1].accumImpulse;
				f32 accumF2 = contactPoints[idx].constraints[2].accumImpulse;
				contactPoints[idx] = contactPair.contactPoints[i];
				contactPoints[idx].constraints[0].accumImpulse = accumN ;
				contactPoints[idx].constraints[1].accumImpulse = accumF1;
				contactPoints[idx].constraints[2].accumImpulse = accumF2;
				contactPoints[idx].duration = d;
			}
#endif

			ret |= 1;
			continue;
		}

		if(numContacts < NUMCONTACTS_PER_BODIES) {
			contactPoints[numContacts++] = contactPair.contactPoints[i];
			ret |= 2;
		} else {
			s32 newIdx = sort4ContactPoints(contactPoints, contactPair.contactPoints[i].getLocalPointA(), contactPair.contactPoints[i].distance);

			contactPoints[newIdx] = contactPair.contactPoints[i];
			ret |= 4;
		}
	}

	return ret;
}

void ContactPair::refreshContactPoints(const Vector3& pA, const Quat& qA, const Vector3& pB, const Quat& qB)
{
	for(s32 i=0;i < (s32)numContacts;i++) {
		if(contactPoints[i].duration > 0) {
			Vector3 cpA = contactPoints[i].getWorldPointA(pA, qA);
			Vector3 cpB = contactPoints[i].getWorldPointB(pB, qB);

			contactPoints[i].distance = dot(contactPoints[i].getNormal(), (cpA - cpB));
			if(contactPoints[i].distance > CONTACT_THRESHOLD_NORMAL) {
				removeContactPoint(i);
				i--;
				continue;
			}

			cpA = cpA - contactPoints[i].distance*contactPoints[i].getNormal();
			f32 distanceAB = lengthSqr(cpA - cpB);
			if(distanceAB > CONTACT_THRESHOLD_TANGENT*CONTACT_THRESHOLD_TANGENT) {
				removeContactPoint(i);
				i--;
				continue;
			}
		}
		if(contactPoints[i].duration < 255) contactPoints[i].duration++;
	}
	duration++;
}



