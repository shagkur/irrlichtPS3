/*
 * trianglescapsulecontacts.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/simdfunc.h"

#include "rigidbody/common/box.h"
#include "rigidbody/common/capsule.h"
#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/intersectfunction.h"
#include "rigidbody/common/contactcache.h"
#include "rigidbody/common/sat_mesh_utils.h"
#include "rigidbody/common/subdata.h"

static inline bool checkSAT(const Vector3& axis, f32 AMin, f32 AMax, f32 BMin, f32 BMax, f32& distMin, Vector3& axisMin)
{
	if(BMax <= AMin)
		return true;
	else if(AMax <= BMin)
		return true;

	if(BMin < AMin && AMax < BMax) {
		f32 d = AMin - BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	} else if(AMin < BMin && BMax < AMax) {
		f32 d = AMin - BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	} else if(BMin < AMin && AMin < BMax) {
		f32 d = AMin - BMax;
		if(distMin < d) {
			distMin = d;
			axisMin = axis;
		}
	} else if(AMin < BMin && BMin < AMax) {
		f32 d = BMin - AMax;
		if(distMin < d) {
			distMin = d;
			axisMin = -axis;
		}
	}

	return false;
}

bool triangleCapsuleContact(ContactCache& contacts, u32 facetId, const Vector3& normal, const Vector3& p0, const Vector3& p1, const Vector3& p2, const f32 thickness, const f32 angle0, const f32 angle1, const f32 angle2, u32 edgeChk, f32 capsuleRadius, const Vector3& capP0, const Vector3& capP1)
{
	const f32 epsilon = 0.00001f;

	f32 distMin = -FLT_MAX;
	Vector3 axisMin(0.0f);

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

		Vector3 sideNml[3] =
		{
			normalize(cross((facetPnts[1] - facetPnts[0]),normal)),
			normalize(cross((facetPnts[2] - facetPnts[1]),normal)),
			normalize(cross((facetPnts[0] - facetPnts[2]),normal))
		};

		if(angle0 > 0.0f || angle1 > 0.0f || angle2 > 0.0f) {
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
				facetPnts[3] = facetPnts[0] + t*(intersection - facetPnts[0]);
                facetPnts[4] = facetPnts[1] + t*(intersection - facetPnts[1]);
                facetPnts[5] = facetPnts[2] + t*(intersection - facetPnts[2]);
			} else {
				facetPnts[3] = intersection;
                facetPnts[4] = intersection;
                facetPnts[5] = intersection;
			}
		}

		const Vector3 capPnts[2] =
		{
			capP0,
			capP1
		};

		{
			const Vector3 &sepAxis = normal;

			Plane plane(sepAxis, facetPnts[0]);

			f32 test1, test2, BMin, BMax;
			test1 = plane.onPlane(capP0);
			test2 = plane.onPlane(capP1);
			BMax = MAX(test1, test2) + capsuleRadius;
			BMin = MIN(test1, test2) - capsuleRadius;

			if(BMin > 0.0f || BMax < -thickness)
				return false;

			if(distMin < BMin) {
				distMin = BMin;
				axisMin = -sepAxis;
			}
		}

		{
			for(s32 e=0;e < 3;e++) {
				Vector3 sepAxis = cross(capP1 - capP0, facetPnts[(e + 1)%3] - facetPnts[e]);
				f32 l = length(sepAxis);
				if(l < 0.00001f) continue;
				sepAxis /= l;

				if(dot(normal, sepAxis) > 0.0f)
					sepAxis = -sepAxis;

				f32 AMin = FLT_MAX, AMax = -FLT_MAX;
				getProjAxisPnts6(facetPnts, sepAxis, AMin, AMax);

				f32 BMin = FLT_MAX, BMax = -FLT_MAX;
				getProjAxisPnts2(capPnts, sepAxis, BMin, BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(checkSAT(sepAxis, AMin, AMax, BMin, BMax, distMin, axisMin))
					return false;
			}
		}

		{
			for(s32 e=0;e < 3;e++) {
				Vector3 edge = facetPnts[(e + 1)%3] - facetPnts[e];
				Vector3 sepAxis = normalize(cross(cross(capP0 - facetPnts[e], edge), edge));

				if(dot(normal, sepAxis) > 0.0f)
					sepAxis = -sepAxis;

				f32 AMin = FLT_MAX, AMax = -FLT_MAX;
				getProjAxisPnts6(facetPnts, sepAxis, AMin, AMax);

				f32 BMin = FLT_MAX, BMax = -FLT_MAX;
				getProjAxisPnts2(capPnts, sepAxis, BMin, BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(checkSAT(sepAxis, AMin, AMax, BMin, BMax, distMin, axisMin))
					return false;
			}
		}

		{
			for(s32 e=0;e < 3;e++) {
				Vector3 edge = facetPnts[(e + 1)%3] - facetPnts[e];
				Vector3 sepAxis = normalize(cross(cross(capP1 - facetPnts[e], edge), edge));

				if(dot(normal, sepAxis) > 0.0f)
					sepAxis = -sepAxis;

				f32 AMin = FLT_MAX, AMax = -FLT_MAX;
				getProjAxisPnts6(facetPnts, sepAxis, AMin, AMax);

				f32 BMin = FLT_MAX, BMax = -FLT_MAX;
				getProjAxisPnts2(capPnts, sepAxis, BMin, BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(checkSAT(sepAxis, AMin, AMax, BMin, BMax, distMin, axisMin))
					return false;
			}
		}

		{
			for(s32 e=0;e < 3;e++) {
				Vector3 capdir = capP1 - capP0;
				Vector3 sepAxis = cross(cross(facetPnts[e] - capP0, capdir), capdir);
				f32 l = length(sepAxis);
				if(l < 0.00001f) continue;
				sepAxis /= l;

				if(dot(normal, sepAxis) > 0.0f)
					sepAxis = -sepAxis;

				f32 AMin = FLT_MAX, AMax = -FLT_MAX;
				getProjAxisPnts6(facetPnts, sepAxis, AMin, AMax);

				f32 BMin = FLT_MAX, BMax = -FLT_MAX;
				getProjAxisPnts2(capPnts, sepAxis, BMin, BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(checkSAT(sepAxis, AMin, AMax, BMin, BMax, distMin, axisMin))
					return false;
			}
		}

		{
			for(s32 e=0;e < 3;e++) {
				Vector3 sepAxis = normalize(facetPnts[e] - capP0);

				f32 AMin = FLT_MAX, AMax = -FLT_MAX;
				getProjAxisPnts6(facetPnts ,sepAxis, AMin, AMax);

				f32 BMin = FLT_MAX, BMax = -FLT_MAX;
				getProjAxisPnts2(capPnts, sepAxis, BMin, BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(checkSAT(sepAxis, AMin, AMax, BMin, BMax, distMin, axisMin))
					return false;
			}
		}

		{
			for(s32 e=0;e < 3;e++) {
				Vector3 sepAxis = normalize(facetPnts[e] - capP1);

				f32 AMin = FLT_MAX, AMax = -FLT_MAX;
				getProjAxisPnts6(facetPnts, sepAxis, AMin, AMax);

				f32 BMin = FLT_MAX, BMax = -FLT_MAX;
				getProjAxisPnts2(capPnts, sepAxis, BMin, BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(checkSAT(sepAxis, AMin, AMax, BMin, BMax, distMin, axisMin))
					return false;
			}
		}

		if(thickness > 0.0f) {
			for(s32 i=0;i < 3;i++) {
				Plane plane(sideNml[i], facetPnts[i]);

				f32 test1, test2, BMin;
				test1 = plane.onPlane(capP0);
				test2 = plane.onPlane(capP1);
				BMin = MIN(test1, test2) - capsuleRadius;

				if(BMin > 0.0f)
					return false;
			}

			for(s32 e=0;e < 3;e++) {
				Vector3 edgeVec = normalize(cross(sideNml[(e + 1)%3], sideNml[e]));
				Vector3 capVec = capP1 - capP0;

				if(isSameDirection(edgeVec, capVec)) continue;

				Vector3 sepAxis = normalize(cross(edgeVec, capVec));

				f32 triMin, triMax;
				getProjAxisPnts3(facetPnts, sepAxis, triMin, triMax);

				f32 BMin = FLT_MAX, BMax = -FLT_MAX;
				getProjAxisPnts2(capPnts, sepAxis, BMin, BMax);
				BMin -= capsuleRadius;
				BMax += capsuleRadius;

				if(triMax < BMin || BMax < triMin)
					return false;
			}
		}
	}

	{
		Vector3 sepAxis = 1.1f*fabsf(distMin)*axisMin;

		const Vector3 facetPnts[3] =
		{
			p0 + sepAxis,
			p1 + sepAxis,
			p2 + sepAxis
		};

		ClosestPoints cp;
		Vector3 sA,sB;

		f32 distSqr;

		distanceTwoLines(capP0, capP1, facetPnts[0], facetPnts[1], sB, sA);
		distSqr = lengthSqr(sA - sB);
		cp.add(sA, sB + normalize(sA - sB)*capsuleRadius, distSqr);

		distanceTwoLines(capP0, capP1, facetPnts[1], facetPnts[2], sB, sA);
		distSqr = lengthSqr(sA - sB);
		cp.add(sA, sB + normalize(sA - sB)*capsuleRadius, distSqr);

		distanceTwoLines(capP0, capP1, facetPnts[2], facetPnts[0], sB, sA);
		distSqr = lengthSqr(sA - sB);
		cp.add(sA, sB + normalize(sA - sB)*capsuleRadius, distSqr);

		distancePointAndTriangle(facetPnts[0], facetPnts[1], facetPnts[2], capP0, sA);
		distSqr = lengthSqr(sA - capP0);
		cp.add(sA, capP0 + normalize(sA - capP0)*capsuleRadius, distSqr);

		distancePointAndTriangle(facetPnts[0], facetPnts[1], facetPnts[2], capP1, sA);
		distSqr = lengthSqr(sA - capP1);
		cp.add(sA, capP1 + normalize(sA - capP1)*capsuleRadius, distSqr);

		for(s32 i=0;i < cp.numPoints;i++) {
			if(cp.distSqr[i] < cp.closestDistSqr + epsilon) {
				cp.pA[i] -= sepAxis;
				if(((edgeChk&0x01) && pointOnLine(cp.pA[i], p0, p1)) ||
				   ((edgeChk&0x02) && pointOnLine(cp.pA[i], p1, p2)) ||
				   ((edgeChk&0x04) && pointOnLine(cp.pA[i], p2, p0)))
				{
					axisMin = -normal;
				}
				contacts.add(-length(cp.pB[i] - cp.pA[i]), axisMin, cp.pA[i], cp.pB[i], facetId);
			}
		}
	}

	return true;
}

s32 trianglesCapsuleContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, f32 *distance, const TriMesh *meshA, const Transform3& transformA, Capsule capB, const Transform3& transformB, f32 distanceThreshold)
{
	(void) distanceThreshold;

	Transform3 transformAB, transformBA;
	Matrix3 matrixBA;
	Vector3 offsetBA;

	transformAB = orthoInverse(transformA)*transformB;

	transformBA = orthoInverse(transformAB);

	matrixBA = transformBA.getUpper3x3();
	offsetBA = transformBA.getTranslation();

	u8 selFacets[NUMMESHFACETS] = { 0 };
	u8 numSelFacets = 0;

	Vector3 aabbB = capB.getAABB(Vector3(1,0,0));
	gatherFacets(meshA, (f32*)&aabbB, offsetBA, matrixBA, selFacets, numSelFacets);

	if(numSelFacets == 0)
		return 0;

	Vector3 vCapAB[2] =
	{
		Vector3(-capB.hLength, 0.0f, 0.0f),
		Vector3( capB.hLength, 0.0f, 0.0f)
	};

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

		const MeshEdge *edge[3] =
		{
			&meshA->edges[facet.edgeIndices[0]],
			&meshA->edges[facet.edgeIndices[1]],
			&meshA->edges[facet.edgeIndices[2]]
		};

		u32 edgeChk = ((edge[0]->angleType == EDGE_CONVEX) ? 0x00 : 0x01) |
					  ((edge[1]->angleType == EDGE_CONVEX) ? 0x00 : 0x02) |
					  ((edge[2]->angleType == EDGE_CONVEX) ? 0x00 : 0x04);

		triangleCapsuleContact(contacts, selFacets[f], facetNormal, facetPntsA[0], facetPntsA[1], facetPntsA[2], facet.thickness, 0.5f*PI*(edge[0]->tilt/255.0f), 0.5f*PI*(edge[1]->tilt/255.0f), 0.5f*PI*(edge[2]->tilt/255.0f), edgeChk, capB.radius, vCapAB[0], vCapAB[1]);
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
