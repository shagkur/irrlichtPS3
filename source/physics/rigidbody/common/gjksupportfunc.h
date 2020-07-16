/*
 * gjksupportfunc.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef GJKSUPPORTFUNC_H_
#define GJKSUPPORTFUNC_H_

#include "base/common.h"

void getSupportVertexTriangle(void *shape, Vector3 seperatingAxis, Vector3& supportVertex);
void getSupportVertexTriangleWithThickness(void *shape, Vector3 seperatingAxis, Vector3& supportVertex);
void getSupportVertexConvex(void *shape, Vector3 seperatingAxis, Vector3& supportVertex);
void getSupportVertexBox(void *shape, Vector3 seperatingAxis, Vector3& supportVertex);
void getSupportVertexCapsule(void *shape, Vector3 seperatingAxis, Vector3& supportVertex);
void getSupportVertexSphere(void *shape, Vector3 seperatingAxis, Vector3& supportVertex);

// not supported yet
//void getSupportVertexCylinder(void *shape, Vector3 seperatingAxis, Vector3& supportVertex);

#endif /* GJKSUPPORTFUNC_H_ */
