/*
 * contactheightfield.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "rigidbody/common/heightfieldfunction.h"

bool contactHeightField(const HeightField *heightfield, const Point3& checkPoint, Point3& fieldPoint, Vector3& fieldNormal, f32& dist)
{
	Vector3 localPoint = heightfield->worldToLocalPosition((Vector3)checkPoint);

	if(localPoint[1] < heightfield->getMinHeight() || localPoint[1] > heightfield->getMaxHeight())
		return false;

	f32 h;
	if(!getHeight((*heightfield), localPoint[0], localPoint[2], h))
		return false;

	dist = heightfield->getScale()[1]*(localPoint[1] - h);

	if(dist < 0.0f) {
		localPoint[1] = h;
		fieldPoint = (Point3)mulPerElem(heightfield->getScale(), (localPoint - 0.5f*Vector3(heightfield->getFieldWidth(), 0.0f, heightfield->getFieldDepth())));
		fieldNormal = Vector3(0, 1, 0);
		return true;
	}

	return false;
}
