/*
 * trianglesconvexcontacts.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/simdfunc.h"

#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/intersectfunction.h"
#include "rigidbody/common/contactcache.h"
#include "rigidbody/common/sat_mesh_utils.h"
#include "rigidbody/common/subdata.h"

#include "rigidbody/common/gjksolver.h"
#include "rigidbody/common/gjksupportfunc.h"

bool triangleConvexContact(ContactCache& contacts, u32 facetId, const Vector3& normal, const Vector3& p0, const Vector3& p1, const Vector3& p2, const f32 thickness, const f32 angle0, const f32 angle1, const f32 angle2, u32 edgeChk, const ConvexMesh *meshB)
{
	Vector3 facetPnts[6] =
	{
		p0,
		p1,
		p2,
		p0 - thickness*normal,
		p1 - thickness*normal,
		p2 - thickness*normal
	};

	if(angle0 > 0.0f || angle1 > 0.0f || angle2 > 0.0f) {
		Vector3 sideNml[3] =
		{
			normalize(cross((facetPnts[1] - facetPnts[0]), normal)),
			normalize(cross((facetPnts[2] - facetPnts[1]), normal)),
			normalize(cross((facetPnts[0] - facetPnts[2]), normal)),
		};
		sideNml[0] = cosf(angle0)*sideNml[0] - sinf(angle0)*normal;
		sideNml[1] = cosf(angle1)*sideNml[1] - sinf(angle1)*normal;
		sideNml[2] = cosf(angle2)*sideNml[2] - sinf(angle2)*normal;
		Matrix3 mtx(0.0f);
		Vector3 vec(0.0f);
		mtx.setRow(0, sideNml[0]);
		mtx.setRow(1, sideNml[1]);
		mtx.setRow(2, sideNml[2]);
		vec[0] = dot(facetPnts[0], sideNml[0]);
		vec[1] = dot(facetPnts[1], sideNml[1]);
		vec[2] = dot(facetPnts[2], sideNml[2]);
		Vector3 intersection = inverse(mtx)*vec;
		f32 dist = -dot(intersection - facetPnts[0], normal);
		if(thickness < dist) {
			f32 t = thickness/dist;
			facetPnts[3] = facetPnts[0] +t*(intersection - facetPnts[0]);
            facetPnts[4] = facetPnts[1] +t*(intersection - facetPnts[1]);
            facetPnts[5] = facetPnts[2] +t*(intersection - facetPnts[2]);
		} else {
			facetPnts[3] = intersection;
            facetPnts[4] = intersection;
            facetPnts[5] = intersection;
		}
	}

	GJKSolver gjk((void*)facetPnts, (void*)meshB, getSupportVertexTriangleWithThickness, getSupportVertexConvex);

	Point3 pA(0.0f), pB(0.0f);
	Vector3 nml(0.0f);

	f32 d = gjk.collide(nml, pA, pB, Transform3::identity(), Transform3::identity(), FLT_MAX);
	if(d >= 0.0f) return false;

	Vector3 pointsOnTriangle = Vector3(pA);
	Vector3 pointsOnConvex = Vector3(pB);
	Vector3 axis = nml;

	if(((edgeChk&0x01) && pointOnLine(pointsOnTriangle, p0, p1)) ||
	   ((edgeChk&0x02) && pointOnLine(pointsOnTriangle, p1, p2)) ||
	   ((edgeChk&0x04) && pointOnLine(pointsOnTriangle, p2, p0)))
	{
		axis = -normal;
	}

	contacts.add(-length(pointsOnTriangle - pointsOnConvex), axis, pointsOnTriangle, pointsOnConvex, facetId);

	return true;
}

int trianglesConvexContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, f32 *distance, const TriMesh *meshA, const Transform3& transformA, const ConvexMesh *meshB, const Transform3& transformB, f32 distanceThreshold)
{
	(void) distanceThreshold;

	Transform3 transformAB, transformBA;
	Matrix3 matrixBA;
	Vector3 offsetBA;

	transformAB = orthoInverse(transformA)*transformB;

	transformBA = orthoInverse(transformAB);

	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	u8 selFacets[NUMMESHFACETS] = {0};
	u8 numSelFacets = 0;

	// ※Convex座標系
	Vector3 aabbB = read_Vector3(meshB->half);
	gatherFacets(meshA, (f32*)&aabbB, offsetBA, matrixBA, selFacets, numSelFacets);

	if(numSelFacets == 0)
		return 0;

	ContactCache contacts;

	for(u32 f=0;f < numSelFacets;f++) {
		const MeshFacet& facet = meshA->facets[selFacets[f]];

		Vector3 facetNormal = matrixBA*read_Vector3(facet.normal);

		Vector3 facetPntsA[3] =
		{
			offsetBA + matrixBA*meshA->verts[facet.vertIndices[0]],
			offsetBA + matrixBA*meshA->verts[facet.vertIndices[1]],
			offsetBA + matrixBA*meshA->verts[facet.vertIndices[2]]
		};

		Vector3 axis, pointsOnTriangle, pointsOnConvex;

		const MeshEdge *edge[3] =
		{
			&meshA->edges[facet.edgeIndices[0]],
			&meshA->edges[facet.edgeIndices[1]],
			&meshA->edges[facet.edgeIndices[2]]
		};

		u32 edgeChk = ((edge[0]->angleType == EDGE_CONVEX) ? 0x00 : 0x01) |
					  ((edge[1]->angleType == EDGE_CONVEX) ? 0x00 : 0x02) |
					  ((edge[2]->angleType == EDGE_CONVEX) ? 0x00 : 0x04);

		triangleConvexContact(contacts, selFacets[f], facetNormal, facetPntsA[0], facetPntsA[1], facetPntsA[2], facet.thickness, 0.5f*PI*(edge[0]->tilt/255.0f), 0.5f*PI*(edge[1]->tilt/255.0f), 0.5f*PI*(edge[2]->tilt/255.0f), edgeChk, meshB);
	}

	for(u32 i=0;i < contacts.getNumContacts();i++) {
		normal[i] = transformB.getUpper3x3()*contacts.getNormal(i);
		pointA[i] = transformAB*Point3(contacts.getPointA(i));
		pointB[i] = Point3(contacts.getPointB(i));
		distance[i] = contacts.getDistance(i);

		const MeshFacet& facet = meshA->facets[contacts.getInfo(i)];
		Vector3 facetPnts[3] =
		{
			meshA->verts[facet.vertIndices[0]],
			meshA->verts[facet.vertIndices[1]],
			meshA->verts[facet.vertIndices[2]]
		};
		f32 s, t;
		get_ST(s, t, facetPnts[1] - facetPnts[0], facetPnts[2] - facetPnts[0], Vector3(pointA[i]) - facetPnts[0]);
		subData[i].type = SubData::SubDataFacetLocal;
		subData[i].setFacetIndex((u8)contacts.getInfo(i));
		subData[i].setFacetLocalS(s);
		subData[i].setFacetLocalT(t);
	}

	return contacts.getNumContacts();
}
