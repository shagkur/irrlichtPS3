/*
 * trianglesspherecontacts.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/simdfunc.h"

#include "rigidbody/common/sphere.h"
#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/intersectfunction.h"
#include "rigidbody/common/sat_mesh_utils.h"
#include "rigidbody/common/contactcache.h"
#include "rigidbody/common/subdata.h"

bool triangleSphereContact(ContactCache& contacts, u32 facetId, const Vector3& normal, const Vector3& p0, const Vector3& p1, const Vector3& p2, const f32 thickness, const f32 angle0, const f32 angle1, const f32 angle2, u32 edgeChk, f32 sphereRadius, const Vector3& spherePos)
{
	Vector3 facetPnts[3] =
	{
		p0,
		p1,
		p2
	};

	{
		Plane planeA(normal, p0);
		f32 len1 = planeA.onPlane(spherePos);

		if(len1 >= sphereRadius || len1 < -thickness - sphereRadius) return false;

		if(angle0 > 0.0f || angle1 > 0.0f || angle2 > 0.0f) {
			Vector3 sideNml[3];
			sideNml[0] = normalize(cross((facetPnts[1] - facetPnts[0]), normal));
			sideNml[1] = normalize(cross((facetPnts[2] - facetPnts[1]), normal));
			sideNml[2] = normalize(cross((facetPnts[0] - facetPnts[2]), normal));
			sideNml[0] = cosf(angle0)*sideNml[0] - sinf(angle0)*normal;
			sideNml[1] = cosf(angle1)*sideNml[1] - sinf(angle1)*normal;
			sideNml[2] = cosf(angle2)*sideNml[2] - sinf(angle2)*normal;
			for(s32 i=0;i < 3;i++) {
				Plane plane(sideNml[i], facetPnts[i]);
				f32 len2 = plane.onPlane(spherePos);
				if(len2 >= sphereRadius) return false;
			}
		}
	}

	{
		Vector3 pntA;
		s32 feature = distancePointAndTriangle(p0, p1, p2, spherePos, pntA);

		Vector3 distVec = pntA - spherePos;
		f32 l = length(distVec);

		if(feature != 6 && l >= sphereRadius) return false;

		Vector3 sepAxis = (l < 0.00001f || feature == 6) ? -normal : distVec/l;

		Vector3 pointsOnSphere = spherePos + sphereRadius*sepAxis;
		Vector3 pointsOnTriangle = pntA;

		if(((edgeChk&0x01) && pointOnLine(pointsOnTriangle, p0, p1)) ||
		   ((edgeChk&0x02) && pointOnLine(pointsOnTriangle, p1, p2)) ||
		   ((edgeChk&0x04) && pointOnLine(pointsOnTriangle, p2, p0)))
		{
			sepAxis = -normal;
		}

		contacts.add(-length(pointsOnSphere - pointsOnTriangle), sepAxis, pointsOnTriangle, pointsOnSphere, facetId);
	}

	return true;
}

s32 trianglesSphereContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, f32 *distance, const TriMesh *meshA, const Transform3& transformA, Sphere sphereB, const Transform3& transformB, f32 distanceThreshold)
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

	Vector3 aabbB(sphereB.radius);
	gatherFacets(meshA, (f32*)&aabbB, offsetBA, matrixBA, selFacets, numSelFacets);

	if(numSelFacets == 0)
		return 0;

	ContactCache contacts;
	{
		for(u32 f=0;f < numSelFacets;f++ ) {
			const MeshFacet& facet = meshA->facets[selFacets[f]];

			const Vector3 facetNormal = read_Vector3(facet.normal);

			const Vector3 facetPnts[3] =
			{
				meshA->verts[facet.vertIndices[0]],
				meshA->verts[facet.vertIndices[1]],
				meshA->verts[facet.vertIndices[2]]
			};

			const MeshEdge *edge[3] =
			{
				&meshA->edges[facet.edgeIndices[0]],
				&meshA->edges[facet.edgeIndices[1]],
				&meshA->edges[facet.edgeIndices[2]]
			};

			Vector3 sepAxis, pntA, pntB;

			u32 edgeChk = ((edge[0]->angleType == EDGE_CONVEX) ? 0x00 : 0x01) |
						  ((edge[1]->angleType == EDGE_CONVEX) ? 0x00 : 0x02) |
						  ((edge[2]->angleType == EDGE_CONVEX) ? 0x00 : 0x04);

			triangleSphereContact(contacts, selFacets[f], facetNormal, facetPnts[0], facetPnts[1], facetPnts[2], facet.thickness, 0.5f*PI*(edge[0]->tilt/255.0f), 0.5f*PI*(edge[1]->tilt/255.0f), 0.5f*PI*(edge[2]->tilt/255.0f), edgeChk, sphereB.radius, transformAB.getTranslation());
		}
	}

	for(u32 i=0;i < contacts.getNumContacts();i++) {
		normal[i] = transformA.getUpper3x3()*contacts.getNormal(i);
		pointA[i] = Point3(contacts.getPointA(i));
		pointB[i] = transformBA*Point3(contacts.getPointB(i));
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
