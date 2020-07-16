/*
 * collobject.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef COLLOBJECT_H_
#define COLLOBJECT_H_

#include "rigidbody/common/collprim.h"
#include "rigidbody/common/rigidbodyconfig.h"

///////////////////////////////////////////////////////////////////////////////
// Collidable Object

ATTRIBUTE_ALIGNED128(class) CollObject
{
	friend class RigidBodies;
	friend class PrimIterator;

private:
	ATTRIBUTE_PTR32(CollPrim *mPrimBase);
	u16 mPrimIds[NUMPRIMS];
	u8 mNumPrims;
	u8 mMaxPrims;

	f32 mCcdRadius;
	f32 mCenter[3];	// AABB center (Local)
	f32 mHalf[3];		// AABB half (Local)

	CollPrim mDefPrim;

	CollPrim& getNewPrim()
	{
		ASSERT(mNumPrims <= mMaxPrims);
		if(mNumPrims == 0) {
			mNumPrims++;
			return mDefPrim;
		} else {
			mNumPrims++;
			return mPrimBase[mPrimIds[mNumPrims - 2]];
		}
	}

public:
	CollObject();
	~CollObject();

	void addBox(Box box, const Transform3& objectRelTransform);
	void addCapsule(Capsule capsule, const Transform3& objectRelTransform);
	void addSphere(Sphere sphere, const Transform3& objectRelTransform);
	void addConvex(const ConvexMesh *convexMesh, const Transform3& objectRelTransform);
	void setHeightField(const HeightField *heightfield);
	void setLargeMesh(const LargeTriMesh *largeMesh);
	void setPreLargeMesh(const LargeTriMesh *largeMesh);

	void finish();
	void clear(); // clear all the prims

	inline u8 getNumPrims() const;
	const CollPrim& getDefPrim() const {return mDefPrim;}
	CollPrim& getDefPrim() {return mDefPrim;}

#ifndef __SPU__
	inline const CollPrim& getPrim(s32 i) const;
	inline CollPrim& getPrim(s32 i);
#endif

	inline f32 getRadius() const;
	inline Vector3 getHalf() const;
	inline Vector3 getCenter() const;

	inline void setCcdRadius(f32 r) {mCcdRadius = r;}
	inline f32 getCcdRadius() const {return mCcdRadius;}

};

inline u8 CollObject::getNumPrims() const
{
	return mNumPrims;
}

#ifndef __SPU__

inline const CollPrim& CollObject::getPrim(s32 i) const
{
	ASSERT(i < mNumPrims);
	if(i > 0)
		return mPrimBase[mPrimIds[i - 1]];
	else
		return mDefPrim;
}

inline CollPrim& CollObject::getPrim(s32 i)
{
	ASSERT(i < mNumPrims);
	if(i > 0)
		return mPrimBase[mPrimIds[i - 1]];
	else
		return mDefPrim;
}

#endif

inline f32 CollObject::getRadius() const
{
	return length(getHalf());
}

inline Vector3 CollObject::getHalf() const
{
	return read_Vector3(mHalf);
}

inline Vector3 CollObject::getCenter() const
{
	return read_Vector3(mCenter);
}

inline void CollObject::setPreLargeMesh(const LargeTriMesh *largeMesh)
{
	if(mDefPrim.getType() == LARGEMESH)
		mDefPrim.setPreLargeMesh(largeMesh);
}

///////////////////////////////////////////////////////////////////////////////
// Primitive Iterator

#ifdef __SPU__

class PrimIterator
{
private:
	static const u32 mBatch = 3;
	static const u32 mDmaTag = 15;

	u32 mNumPrims;
	CollPrim *mPrimBase;
	const u16 *mPrimIds;
	CollPrim *mCurPrim;
	u32 mLoopId;
	u32 mIndex;
	u32 mSwap;

	CollPrim mPrims[2][mBatch];
	spu_dma_list_element mDmaList[2][mBatch];

public:
	PrimIterator(const CollObject& coll) : mPrimIds(coll.mPrimIds)
	{
		mNumPrims = coll.mNumPrims;
		mPrimBase = coll.mPrimBase;
		mPrims[0][0] = coll.mDefPrim;
		mCurPrim = mPrims[0];

		if(mNumPrims > 1) {
			mLoopId = 0;
			mIndex = 0;
			mSwap = 0;

			u32 j;
			for(j=0;j < mBatch;j++) {
				mDmaList[0][j].notify = 0;
				mDmaList[0][j].size = sizeof(CollPrim);
				mDmaList[1][j].notify = 0;
				mDmaList[1][j].size = sizeof(CollPrim);
			}

			for(j=0;j < mBatch - 1 && mLoopId < mNumPrims - 1;j++, mLoopId++)
				mDmaList[0][j].eal = (u64)(mPrimBase + mPrimIds[mLoopId]);

			spu_dma_list_get(&mPrims[0][1], 0, mDmaList[0], sizeof(spu_dma_list_element)*j, mDmaTag, 0, 0);

			spu_dma_wait_tag_status_all(1<<mDmaTag);

			for(j=0;j < mBatch && mLoopId < mNumPrims - 1;j++, mLoopId++)
				mDmaList[1][j].eal = (u64)(mPrimBase + mPrimIds[mLoopId]);

			spu_dma_list_get(&mPrims[1], 0, mDmaList[1], sizeof(spu_dma_list_element)*j, mDmaTag, 0, 0);

			mCurPrim = mPrims[mSwap];
		}
	}

	~PrimIterator()
	{
		if(mNumPrims > 1)
			spu_dma_wait_tag_status_all(1<<mDmaTag);
	}

	inline PrimIterator& operator++()
	{
		if(mNumPrims > 1) {
			mIndex++;
			mCurPrim++;
			if(UNLIKELY(mIndex >= mBatch)) {
				spu_dma_wait_tag_status_all(1<<mDmaTag);

				u32 j;
				for(j=0;j < mBatch && mLoopId < mNumPrims - 1;j++, mLoopId++) {
					mDmaList[mSwap][j].eal = (u64)(mPrimBase + mPrimIds[mLoopId]);
				}
				spu_dma_list_get(&mPrims[mSwap], 0, mDmaList[mSwap], sizeof(spu_dma_list_element)*j, mDmaTag, 0, 0);

				mSwap = 1 - mSwap;
				mIndex = 0;
				mCurPrim = mPrims[mSwap];
			}
		}
		return *this;
	}

	const CollPrim& operator*() const {return *mCurPrim;}
};

#else

class PrimIterator
{
private:
	u32 mNumPrims;
	CollPrim *mPrimBase;
	const u16 *mPrimIds;
	const CollPrim *mCurPrim;
	u32 mIndex;

public:
	PrimIterator(const CollObject& coll) : mPrimIds(coll.mPrimIds)
	{
		mNumPrims = coll.mNumPrims;
		mPrimBase = coll.mPrimBase;
		mIndex = 0;
		mCurPrim = &coll.mDefPrim;
	}

	~PrimIterator() {}

	inline PrimIterator& operator++()
	{
		mCurPrim = &mPrimBase[mPrimIds[mIndex++]];
		return *this;
	}

	const CollPrim& operator*() const {return *mCurPrim;}
};

#endif

#endif /* COLLOBJECT_H_ */
