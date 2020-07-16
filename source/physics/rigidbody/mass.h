/*
 * mass.h
 *
 *  Created on: Jun 6, 2013
 *      Author: mike
 */

#ifndef MASS_H_
#define MASS_H_

#include "rigidbody/common/trimesh.h"

f32 calcMassBox(f32 density, const Vector3& size);
void calcInertiaBox(const Vector3& size, f32 mass, Matrix3& inertia);

f32 calcMassSphere(f32 density, f32 radius);
void calcInertiaSphere(f32 radius, f32 mass, Matrix3& inertia);

f32 calcMassCylinder(f32 density, f32 radius, f32 halfHeight);
void calcInertiaCylinder(f32 radius, f32 halfHeight, f32 mass, Matrix3& inertia, s32 axis);

void calcInertiaCylinderX(f32 radius, f32 halfHeight, f32 mass, Matrix3& inertia);
void calcInertiaCylinderY(f32 radius, f32 halfHeight, f32 mass, Matrix3& inertia);
void calcInertiaCylinderZ(f32 radius, f32 halfHeight, f32 mass, Matrix3& inertia);

f32 calcMassMesh(f32 density, const TriMesh& mesh);
void calcInertiaMesh(const TriMesh& mesh, f32 mass, Matrix3& inertia);

f32 calcMassMesh(f32 density, const ConvexMesh& mesh);
void calcInertiaMesh(const ConvexMesh& mesh, f32 mass, Matrix3& inertia);

f32 calcMassMesh(f32 density, const f32 *verts, s32 numVerts, const u16 *indices);
void calcInertiaMesh(const f32 *verts, s32 numVerts, const u16 *indices, s32 numIndices, f32 mass, Matrix3& inertia);

void massTranslate(f32 mass, Matrix3& inertia, const Vector3& translation);
void massRotate(Matrix3& inertia, const Matrix3& rotate);
void massMerge(f32 mass, Matrix3& inertia, Vector3& p, f32 mass1, const Matrix3& inertia1, const Vector3& p1, f32 mass2, const Matrix3& inertia2, const Vector3& p2);

#endif /* MASS_H_ */
