/*
 * convexconvexdistance.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/simdfunc.h"

#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/intersectfunction.h"
#include "rigidbody/common/sat_mesh_utils.h"

#include "rigidbody/common/gjksolver.h"
#include "rigidbody/common/gjksupportfunc.h"

f32 closestConvexConvex( Vector3& normal, Point3& pointA, Point3& pointB, const ConvexMesh *meshA, const Transform3& transformA, const ConvexMesh *meshB, const Transform3& transformB, f32 distanceThreshold)
{
	(void) distanceThreshold;

	GJKSolver gjk((void*)meshA, (void*)meshB, getSupportVertexConvex, getSupportVertexConvex);

	return gjk.collide(normal, pointA, pointB, transformA, transformB, FLT_MAX);
}
