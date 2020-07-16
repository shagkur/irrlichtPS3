/*
 * trianglesboxcontacts.cpp
 *
 *  Created on: Jun 4, 2013
 *      Author: mike
 */

#include "base/common.h"
#include "base/simdfunc.h"

#include "rigidbody/common/box.h"
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

bool triangleBoxContact(ContactCache& contacts, u32 facetId, const Vector3& normal, const Vector3& p0, const Vector3& p1, const Vector3& p2, const f32 thickness, const f32 angle0, const f32 angle1, const f32 angle2, u32 edgeChk, const Vector3& boxHalf)
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

		{
			const Vector3 sepAxis = normal;

			Plane planeA(sepAxis, p0);

			f32 r = dot(boxHalf, absPerElem(sepAxis));
			f32 boxOffset = planeA.onPlane(Vector3(0.0f));
			f32 boxMax = boxOffset + r;
			f32 boxMin = boxOffset - r;

			if(boxMin > 0.0f || boxMax < -thickness)
				return false;

			if(distMin < boxMin) {
				distMin = boxMin;
				axisMin = -sepAxis;
			}
		}

		// Box -> Triangles
		for(u8 bf=0;bf < 3;bf++) {
			Vector3 sepAxis(0.0f);
			sepAxis[bf] = 1.0f;

			if(dot(normal, sepAxis) > 0.0f)
				sepAxis = -sepAxis;

			f32 triMin, triMax;
			getProjAxisPnts6(facetPnts, sepAxis, triMin, triMax);

			f32 boxMin = -boxHalf[bf];
			f32 boxMax =  boxHalf[bf];

			if(checkSAT(sepAxis, triMin, triMax, boxMin, boxMax, distMin, axisMin))
				return false;
		}

		for(u8 e=0;e < 3;e++) {
			Vector3 dir = normalize(facetPnts[(e + 1)%3] - facetPnts[e]);

			for(s32 i=0;i < 3;i++) {
				Vector3 boxEdge(0.0f);
				boxEdge[i] = 1.0f;

				if(isSameDirection(dir, boxEdge)) continue;

				Vector3 sepAxis = normalize(cross(dir, boxEdge));

				if(dot(normal, sepAxis) > 0.0f)
					sepAxis = -sepAxis;

				f32 triMin, triMax;
				getProjAxisPnts6(facetPnts, sepAxis, triMin, triMax);

				f32 r = dot(boxHalf, absPerElem(sepAxis));
				f32 boxMin = -r;
				f32 boxMax = r;

				if(checkSAT(sepAxis, triMin, triMax, boxMin, boxMax, distMin, axisMin))
					return false;
			}
		}

		if(thickness > 0.0f) {
			for(s32 i=0;i < 3;i++) {
				Plane planeA(sideNml[i], facetPnts[i]);

				f32 r = dot(boxHalf, absPerElem(sideNml[i]));
				f32 boxOffset = planeA.onPlane(Vector3(0.0f));
				f32 boxMin = boxOffset - r;

				if(boxMin > 0.0f)
					return false;
			}

			for(s32 e=0;e < 3;e++) {
				Vector3 edgeVec = normalize(cross(sideNml[(e + 1)%3], sideNml[e]));

				for(s32 i=0;i < 3;i++) {
					Vector3 boxEdge(0.0f);
					boxEdge[i] = 1.0f;

					if(isSameDirection(edgeVec, boxEdge)) continue;

					Vector3 sepAxis = normalize(cross(edgeVec, boxEdge));

					f32 triMin, triMax;
					getProjAxisPnts3(facetPnts, sepAxis, triMin, triMax);

					f32 r = dot(boxHalf, absPerElem(sepAxis));
					f32 boxMin = -r;
					f32 boxMax = r;

					if(triMax < boxMin || boxMax < triMin)
						return false;
				}
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

		const Vector3 boxPnts[8] =
		{
			Vector3(-boxHalf[0], -boxHalf[1], -boxHalf[2]),
			Vector3(-boxHalf[0], -boxHalf[1],  boxHalf[2]),
			Vector3( boxHalf[0], -boxHalf[1],  boxHalf[2]),
			Vector3( boxHalf[0], -boxHalf[1], -boxHalf[2]),
			Vector3(-boxHalf[0],  boxHalf[1], -boxHalf[2]),
			Vector3(-boxHalf[0],  boxHalf[1],  boxHalf[2]),
			Vector3( boxHalf[0],  boxHalf[1],  boxHalf[2]),
			Vector3( boxHalf[0],  boxHalf[1], -boxHalf[2])
		};

		ClosestPoints cp;
		Vector3 sA,sB;
		{
			s32 boxIds[] =
			{
				0, 1,
				1, 2,
				2, 3,
				3, 0,
				4, 5,
				5, 6,
				6, 7,
				7, 4,
				0, 4,
				3, 7,
				2, 6,
				1, 5
			};

			for(s32 i=0;i < 3;i++) {
				for(s32 j=0;j < 12;j++) {
					distanceTwoLines(facetPnts[i], facetPnts[(i + 1)%3], boxPnts[boxIds[j*2]], boxPnts[boxIds[j*2 + 1]], sA, sB);
					f32 distSqr = lengthSqr(sA - sB);
					cp.add(sA, sB, distSqr);
				}
			}
		}

		{
			closestPointAndAABB(facetPnts[0], boxHalf, sB);
			f32 distSqr = lengthSqr(sB - facetPnts[0]);
			cp.add(facetPnts[0], sB, distSqr);

			closestPointAndAABB(facetPnts[1], boxHalf, sB);
			distSqr = lengthSqr(sB - facetPnts[1]);
			cp.add(facetPnts[1], sB, distSqr);

			closestPointAndAABB(facetPnts[2], boxHalf, sB);
			distSqr = lengthSqr(sB - facetPnts[2]);
			cp.add(facetPnts[2], sB, distSqr);
		}

		for(s32 i=0;i < 8;i++) {
			distancePointAndTriangle(facetPnts[0], facetPnts[1], facetPnts[2], boxPnts[i], sA);
			f32 distSqr = lengthSqr(sA - boxPnts[i]);
			cp.add(sA, boxPnts[i], distSqr);
		}

		for(s32 i=0;i < cp.numPoints;i++) {
			if(cp.distSqr[i] < cp.closestDistSqr + epsilon) {
				cp.pA[i] -= sepAxis;
				if(((edgeChk&0x01) && pointOnLine(cp.pA[i], p0, p1)) ||
				   ((edgeChk&0x02) && pointOnLine(cp.pA[i], p1, p2)) ||
				   ((edgeChk&0x04) && pointOnLine(cp.pA[i], p2, p0)))
				{
					axisMin=-normal;
				}
				contacts.add(-length(cp.pB[i] - cp.pA[i]), axisMin, cp.pA[i], cp.pB[i], facetId);
			}
		}
	}

	return true;
}

int trianglesBoxContacts(Vector3 *normal, SubData *subData, Point3 *pointA, Point3 *pointB, f32 *distance, const TriMesh *meshA, const Transform3& transformA, Box boxB, const Transform3& transformB, f32 distanceThreshold)
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

	Vector3 aabbB = boxB.half;
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

		const MeshEdge *edge[3] =
		{
			&meshA->edges[facet.edgeIndices[0]],
			&meshA->edges[facet.edgeIndices[1]],
			&meshA->edges[facet.edgeIndices[2]]
		};

		u32 edgeChk = ((edge[0]->angleType == EDGE_CONVEX) ? 0x00 : 0x01) |
					  ((edge[1]->angleType == EDGE_CONVEX) ? 0x00 : 0x02) |
					  ((edge[2]->angleType == EDGE_CONVEX) ? 0x00 : 0x04);

		triangleBoxContact(contacts, selFacets[f], facetNormal, facetPntsA[0], facetPntsA[1], facetPntsA[2], facet.thickness, 0.5f*PI*(edge[0]->tilt/255.0f), 0.5f*PI*(edge[1]->tilt/255.0f), 0.5f*PI*(edge[2]->tilt/255.0f), edgeChk, boxB.half);
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
