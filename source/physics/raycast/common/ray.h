/*
 * ray.h
 *
 *  Created on: Jun 7, 2013
 *      Author: mike
 */

#ifndef RAY_H_
#define RAY_H_

#include "rigidbody/common/subdata.h"

enum FaceType
{
	FacetTypeFront = 0,
	FacetTypeBack = 1,
	FacetTypeBoth = 2
};

ATTRIBUTE_ALIGNED16(struct) Ray
{
	// input
	Vector3 startPos;
	Vector3 endPos;
	u32 contactFilterSelf;
	u32 contactFilterTarget;
	u8 rayGroup;
	u8 facetType : 2; // LargeTriMesh Only

	// output
	bool contactFlag : 1;
	u16 contactInstance;
	f32 t;
	Vector3 contactPoint;
	Vector3 contactNormal;
	Vector3 rayDir; // endPos - startPos

	u8 primIdx;
	SubData subData;

	Ray()
	{
		reset();
	}

	void reset()
	{
		primIdx = 0;
		subData.type = 0;
		contactFlag = false;
		contactFilterSelf = contactFilterTarget = 0xffffffff;
		rayGroup = 0;
		facetType = FacetTypeFront;
		t = 1.0f;
	}
};

#endif /* RAY_H_ */
