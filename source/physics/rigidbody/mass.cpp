/*
 * mass.cpp
 *
 *  Created on: Jun 6, 2013
 *      Author: mike
 */

#include "base/common.h"

#include "rigidbody/mass.h"

f32 calcMassBox(f32 density, const Vector3& size)
{
	return density*size[0]*size[1]*size[2]*8;
}

void calcInertiaBox(const Vector3& size, f32 mass, Matrix3& inertia)
{
	const f32 ratio = 1.2f;
	f32 sqrwidth = size[0]*2.0f*ratio;
	f32 sqrheight = size[1]*2.0f*ratio;
	f32 sqrdepth = size[2]*2.0f*ratio;

	sqrwidth *= sqrwidth;
	sqrheight *= sqrheight;
	sqrdepth *= sqrdepth;
	inertia[0][0] = (mass*(sqrheight + sqrdepth))/12.0f;
	inertia[1][1] = (mass*(sqrwidth + sqrdepth))/12.0f;
	inertia[2][2] = (mass*(sqrwidth + sqrheight))/12.0f;
}

f32 calcMassSphere(f32 density, f32 radius)
{
	return (4.0f/3.0f)*PI*radius*radius*radius*density;
}

void calcInertiaSphere(f32 radius, f32 mass, Matrix3& inertia)
{
	const f32 ratio = 1.2f;

	inertia = Matrix3::identity();
	inertia[0][0] = inertia[1][1] = inertia[2][2] = 0.4f*mass*radius*radius*ratio*ratio;
}

f32 calcMassCylinder(f32 density, f32 radius, f32 halfHeight)
{
	return PI*radius*radius*2.0f*halfHeight*density;
}

void calcInertiaCylinder(f32 radius, f32 halfHeight, f32 mass, Matrix3& inertia, s32 axis)
{
	inertia = Matrix3::identity();
	inertia[0][0] = inertia[1][1] = inertia[2][2] = 0.25f*mass*radius*radius + 0.33f*mass*halfHeight*halfHeight;
	inertia[axis][axis] = 0.5f*mass*radius*radius;
}

void calcInertiaCylinderX(f32 radius, f32 halfHeight, f32 mass, Matrix3& inertia)
{
	calcInertiaCylinder(radius, halfHeight, mass, inertia, 0);
}

void calcInertiaCylinderY(f32 radius, f32 halfHeight, f32 mass, Matrix3& inertia)
{
	calcInertiaCylinder(radius, halfHeight, mass, inertia, 1);
}

void calcInertiaCylinderZ(f32 radius, f32 halfHeight, f32 mass, Matrix3& inertia)
{
	calcInertiaCylinder(radius, halfHeight, mass, inertia, 2);
}

void massTranslate(f32 mass, Matrix3& inertia, const Vector3& translation)
{
	Matrix3 m = crossMatrix(translation);
	inertia = inertia + mass*(-m*m);
}

inline f32 calcTrigonalVolume(const Vector3& p1, const Vector3& p2, const Vector3& p3)
{
	return 	dot(cross(p1, p2), p3)/6.0f;
}

Matrix3 calcTrigonalInertia(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,float mass)
{
	Matrix3 retI;

	retI[0][0] = (p1[1]*p1[1] + p1[2]*p1[2] +
				  p2[1]*p2[1] + p2[2]*p2[2] +
				  p3[1]*p3[1] + p3[2]*p3[2] +
				  p1[1]*p2[1] + p1[2]*p2[2] +
				  p2[1]*p3[1] + p2[2]*p3[2] +
				  p3[1]*p1[1] + p3[2]*p1[2])*mass*0.1f;

	retI[1][0] = retI[0][1] = (-2.0f*p1[0]*p1[1] -
							   	     p1[0]*p2[1] -
							   	     p1[0]*p3[1] -
							   	     p2[0]*p1[1] -
							    2.0f*p2[0]*p2[1] -
							    	 p2[0]*p3[1] -
							    	 p3[0]*p1[1] -
							    	 p3[0]*p2[1] -
							    2.0f*p3[0]*p3[1])*mass*0.05f;
	retI[2][0] = retI[0][2] = (-2.0f*p1[0]*p1[2] -
									 p1[0]*p2[2] -
									 p1[0]*p3[2] -
									 p2[0]*p1[2] -
							   -2.0f*p2[0]*p2[2] -
							   	     p2[0]*p3[2] -
							   	     p3[0]*p1[2] -
							   	     p3[0]*p2[2] -
							   -2.0f*p3[0]*p3[2])*mass*0.05f;
	retI[1][1] = (p1[2]*p1[2] + p1[0]*p1[0] +
				  p2[2]*p2[2] + p2[0]*p2[0] +
				  p3[2]*p3[2] + p3[0]*p3[0] +
				  p1[2]*p2[2] + p1[0]*p2[0] +
				  p2[2]*p3[2] + p2[0]*p3[0] +
				  p3[2]*p1[2] + p3[0]*p1[0])*mass*0.1f;
	retI[2][1] = retI[1][2] = (-2.0f*p1[0]*p1[1] -
									 p1[0]*p2[1] -
									 p1[0]*p3[1] -
									 p2[0]*p1[1] -
							    2.0f*p2[0]*p2[1] -
							         p2[0]*p3[1] -
							         p3[0]*p1[1] -
							         p3[0]*p2[1] -
							    2.0f*p3[0]*p3[1])*mass*0.05f;
	retI[2][2] = (p1[0]*p1[0] + p1[1]*p1[1] +
				  p2[0]*p2[0] + p2[1]*p2[1] +
				  p3[0]*p3[0] + p3[1]*p3[1] +
				  p1[0]*p2[0] + p1[1]*p2[1] +
				  p2[0]*p3[0] + p2[1]*p3[1] +
				  p3[0]*p1[0] + p3[1]*p1[1])*mass*0.1f;

	return retI;
}

f32 calcVolumeMesh(const TriMesh& mesh)
{
	f32 allVolume = 0.0f;

	for(u32 f=0;f < mesh.numFacets;f++) {
		const MeshFacet& facet = mesh.facets[f];
		allVolume = calcTrigonalVolume(mesh.verts[facet.vertIndices[0]], mesh.verts[facet.vertIndices[1]], mesh.verts[facet.vertIndices[2]]);
	}

	return allVolume;
}

f32 calcMassMesh(f32 density, const TriMesh& mesh)
{
	return calcVolumeMesh(mesh)*density;
}

void calcInertiaMesh(const TriMesh& mesh, f32 mass, Matrix3& inertia)
{
	f32 allVolume = calcVolumeMesh(mesh);
	Matrix3 allInertia(0.0f);

	for(u32 f=0;f < mesh.numFacets;f++) {
		const MeshFacet& facet = mesh.facets[f];

		float tV = calcTrigonalVolume(mesh.verts[facet.vertIndices[0]], mesh.verts[facet.vertIndices[1]], mesh.verts[facet.vertIndices[2]]);
		Matrix3 tI = calcTrigonalInertia(mesh.verts[facet.vertIndices[0]], mesh.verts[facet.vertIndices[1]], mesh.verts[facet.vertIndices[2]], mass*(tV/allVolume));

		allInertia += tI;
	}

	inertia = allInertia;
}

f32 calcVolumeMesh(const f32 *verts, s32 numVerts, const u16 *indices, s32 numIndices)
{
	(void) numVerts;
	f32 allVolume = 0.0f;
	for(u32 f=0;f < (u32)numIndices/3;f++) {
		f32 tV = calcTrigonalVolume(
									Vector3(verts[indices[f*3 + 0]*3 + 0], verts[indices[f*3 + 0]*3 + 1], verts[indices[f*3 + 0]*3 + 2]),
									Vector3(verts[indices[f*3 + 1]*3 + 0], verts[indices[f*3 + 1]*3 + 1], verts[indices[f*3 + 1]*3 + 2]),
									Vector3(verts[indices[f*3 + 2]*3 + 0], verts[indices[f*3 + 2]*3 + 1], verts[indices[f*3 + 2]*3 + 2])
								   );
		allVolume += tV;
	}

	return allVolume;
}

f32 calcVolumeMesh(const ConvexMesh& mesh)
{
	f32 allVolume = 0.0f;
	for(u32 f=0;f < mesh.numIndices/3;f++) {
		f32 tV = calcTrigonalVolume(
									mesh.verts[mesh.indices[f*3 + 0]],
									mesh.verts[mesh.indices[f*3 + 1]],
									mesh.verts[mesh.indices[f*3 + 2]]
								   );
		allVolume += tV;
	}

	return allVolume;
}

f32 calcMassMesh(f32 density, const ConvexMesh& mesh)
{
	return calcVolumeMesh(mesh)*density;
}

void calcInertiaMesh(const ConvexMesh& mesh, f32 mass, Matrix3& inertia)
{
	f32 allVolume = calcVolumeMesh(mesh);
	Matrix3 allInertia(0.0f);

	for(u32 f=0;f < mesh.numIndices/3;f++) {
		f32 tV = calcTrigonalVolume(
									mesh.verts[mesh.indices[f*3 + 0]],
									mesh.verts[mesh.indices[f*3 + 1]],
									mesh.verts[mesh.indices[f*3 + 2]]
								   );
		Matrix3 tI = calcTrigonalInertia(
										 mesh.verts[mesh.indices[f*3 + 0]],
										 mesh.verts[mesh.indices[f*3 + 1]],
										 mesh.verts[mesh.indices[f*3 + 2]],
										 mass*(tV/allVolume)
										);

		allInertia += tI;
	}

	inertia = allInertia;
}

f32 calcMassMesh(f32 density, const f32 *verts, s32 numVerts, const u16 *indices, s32 numIndices)
{
	return calcVolumeMesh(verts, numVerts, indices, numIndices)*density;
}

void calcInertiaMesh(const f32 *verts, s32 numVerts, const u16 *indices, s32 numIndices, f32 mass, Matrix3& inertia)
{
	f32 allVolume = calcVolumeMesh(verts, numVerts, indices, numIndices);
	Matrix3 allInertia(0.0f);

	for(u32 f=0;f < (u32)numIndices/3;f++) {
		f32 tV = calcTrigonalVolume(
									Vector3(verts[indices[f*3 + 0]*3 + 0], verts[indices[f*3 + 0]*3 + 1], verts[indices[f*3 + 0]*3 + 2]),
									Vector3(verts[indices[f*3 + 1]*3 + 0], verts[indices[f*3 + 1]*3 + 1], verts[indices[f*3 + 1]*3 + 2]),
									Vector3(verts[indices[f*3 + 2]*3 + 0], verts[indices[f*3 + 2]*3 + 1], verts[indices[f*3 + 2]*3 + 2])
								   );
		Matrix3 tI = calcTrigonalInertia(
										 Vector3(verts[indices[f*3 + 0]*3 + 0], verts[indices[f*3 + 0]*3 + 1], verts[indices[f*3 + 0]*3 + 2]),
										 Vector3(verts[indices[f*3 + 1]*3 + 0], verts[indices[f*3 + 1]*3 + 1], verts[indices[f*3 + 1]*3 + 2]),
										 Vector3(verts[indices[f*3 + 2]*3 + 0], verts[indices[f*3 + 2]*3 + 1], verts[indices[f*3 + 2]*3 + 2]),
										 mass*(tV/allVolume)
										);

		allInertia += tI;
	}

	inertia = allInertia;
}

void massRotate(Matrix3& inertia, const Matrix3& rotate)
{
	inertia = rotate*inertia*transpose(rotate);
}

void massMerge(f32 mass, Matrix3& inertia, Vector3& p, f32 mass1, const Matrix3& inertia1, const Vector3& p1, f32 mass2, const Matrix3& inertia2, const Vector3& p2)
{
	p = (mass1*p1 + mass2*p2)/(mass1 + mass2);
	mass = mass1 + mass2;
	inertia = inertia1 + inertia2;
}
