/*
 * collobject.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "rigidbody/common/collobject.h"
#include "rigidbody/common/heightfield.h"
#include "rigidbody/common/trimesh.h"

CollObject::CollObject()
{
	clear();
}

CollObject::~CollObject()
{
}

void CollObject::clear()
{
	mNumPrims = 0;
	mMaxPrims = 1;
	mCenter[0] = 0.0f;
	mCenter[1] = 0.0f;
	mCenter[2] = 0.0f;
	mHalf[0] = 0.0f;
	mHalf[1] = 0.0f;
	mHalf[2] = 0.0f;
	mCcdRadius = 0.0f;
}

void CollObject::addBox(Box box, const Transform3& objectRelTransform)
{
	if(mNumPrims < mMaxPrims) {
		CollPrim& newPrim = getNewPrim();
		newPrim.setBox(box);
		newPrim.setObjectRelTransform(objectRelTransform);
	}
}

void CollObject::addCapsule(Capsule capsule, const Transform3& objectRelTransform)
{
	if(mNumPrims < mMaxPrims) {
		CollPrim &newPrim = getNewPrim();
		newPrim.setCapsule(capsule);
		newPrim.setObjectRelTransform(objectRelTransform);
	}
}

void CollObject::addSphere(Sphere sphere, const Transform3& objectRelTransform)
{
	if(mNumPrims < mMaxPrims) {
		CollPrim &newPrim = getNewPrim();
		newPrim.setSphere(sphere);
		newPrim.setObjectRelTransform(objectRelTransform);
	}
}

void CollObject::addConvex(const ConvexMesh *convexMesh, const Transform3& objectRelTransform)
{
	ASSERT(convexMesh);

	if(mNumPrims < mMaxPrims) {
		CollPrim &newPrim = getNewPrim();
		newPrim.setConvexMesh(convexMesh);
		newPrim.setObjectRelTransform(objectRelTransform);
	}
}

void CollObject::finish()
{
#ifndef __SPU__
	if(mNumPrims == 0 || mDefPrim.getType() == HEIGHTFIELD || mDefPrim.getType() == LARGEMESH)
		return;

	// compute AABB
	Vector3 vcenter(0.0f), vhalf(0.0f);
	mCcdRadius = 0.0f;

	Vector3 halfMax(-FLT_MAX), halfMin(FLT_MAX);

	PrimIterator itrPrim(*this);
	for(u32 i=0;i < getNumPrims();i++, ++itrPrim) {
		const CollPrim& prim = *itrPrim;
		Vector3 primHalf(0.0f);

		if(prim.getType() == BOX)
			primHalf = prim.getBox().getAABB(prim.getObjectRelTransform().getUpper3x3());
		else if(prim.getType() == CAPSULE)
			primHalf = prim.getCapsule().getAABB(prim.getObjectRelTransform().getCol(0));
		else if(prim.getType() == SPHERE)
			primHalf = prim.getSphere().getAABB();
		else if(prim.getType() == CONVEXMESH)
			primHalf = prim.getConvexMesh()->getAABB(prim.getObjectRelTransform().getUpper3x3());

		Vector3 relPos = prim.getObjectRelTransform().getTranslation();

		halfMax = maxPerElem(halfMax, relPos + primHalf);
		halfMin = minPerElem(halfMin, relPos - primHalf);
	}

	vcenter = (halfMin + halfMax)*0.5f;
	vhalf = (halfMax - halfMin)*0.5f;
	mCenter[0] = vcenter[0];
	mCenter[1] = vcenter[1];
	mCenter[2] = vcenter[2];
	mHalf[0] = vhalf[0];
	mHalf[1] = vhalf[1];
	mHalf[2] = vhalf[2];
	mCcdRadius = length(vhalf);
#endif
}

void CollObject::setHeightField(const HeightField *heightfield)
{
	ASSERT(heightfield);

	mDefPrim.setHeightField(heightfield);
	mDefPrim.setObjectRelTransform(Transform3::identity());
	mNumPrims = 1;

	Vector3 vhalf(0.0f), vcenter(0.0f);

	vhalf =	0.5f*Vector3(heightfield->getFieldWidth(), (heightfield->getMaxHeight() - heightfield->getMinHeight()), heightfield->getFieldDepth());

	vhalf =  mulPerElem(heightfield->getScale(), vhalf);
	vcenter = Vector3(0.0f, 0.5f*heightfield->getScale()[1]*(heightfield->getMaxHeight() + heightfield->getMinHeight()), 0.0f);
	mCenter[0] = vcenter[0];
	mCenter[1] = vcenter[1];
	mCenter[2] = vcenter[2];
	mHalf[0] = vhalf[0];
	mHalf[1] = vhalf[1];
	mHalf[2] = vhalf[2];
	mCcdRadius = 0.0f;
}

void CollObject::setLargeMesh(const LargeTriMesh *largeMesh)
{
	ASSERT(largeMesh);

	mDefPrim.setLargeMesh(largeMesh);
	mDefPrim.setObjectRelTransform(Transform3::identity());
	mDefPrim.viData[1] = mDefPrim.viData[2] = 0;
	mNumPrims = 1;

	if(largeMesh->numIslands == 0)
		return;

	VecInt3 halfMax(-0xffff), halfMin(0xffff);
	for(u16 i=0;i < largeMesh->numIslands;i++) {
		AABB16 aabb = largeMesh->aabbList[i];
		VecInt3 aabbMin((s32)getXMin(aabb),(s32)getYMin(aabb),(s32)getZMin(aabb));
		VecInt3 aabbMax((s32)getXMax(aabb),(s32)getYMax(aabb),(s32)getZMax(aabb));
		halfMax = maxPerElem(halfMax, aabbMax);
		halfMin = minPerElem(halfMin, aabbMin);
	}

	Vector3 vHalfMin = largeMesh->getWorldPosition(halfMin);
	Vector3 vHalfMax = largeMesh->getWorldPosition(halfMax);
	Vector3 vhalf = (vHalfMax - vHalfMin)*0.5f;
	Vector3 vcenter = (vHalfMin + vHalfMax)*0.5f;
	mCenter[0] = vcenter[0];
	mCenter[1] = vcenter[1];
	mCenter[2] = vcenter[2];
	mHalf[0] = vhalf[0];
	mHalf[1] = vhalf[1];
	mHalf[2] = vhalf[2];
	mCcdRadius = 0.0f;
}

