/*
 * heightfield.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef HEIGHTFIELD_H_
#define HEIGHTFIELD_H_

#include "base/common.h"
#include "rigidbody/common/rigidbodyconfig.h"

struct HeightFieldBlock
{
	ATTRIBUTE_ALIGNED128(f32 heightBuf[ALIGN16(BLOCK_SIZE_B*BLOCK_SIZE_B, sizeof(f32))]);
};

struct HeightFieldData
{
	u16 numBlockX, numBlockZ;
	u16 numFieldX, numFieldZ;
	f32 maxHeight, minHeight;
	f32 fieldWidth, fieldDepth;

	ATTRIBUTE_ALIGNED16_PTR32(HeightFieldBlock *blocks);

	HeightFieldData()
	{
		blocks = NULL;
	}
};

///////////////////////////////////////////////////////////////////////////////
// Height Field Class

ATTRIBUTE_ALIGNED16(class) HeightField
{
private:
	Vector3 fieldScale;

public:
	HeightField();

#ifndef __SPU__
	void setHeightFieldData(const HeightFieldData& field);
#endif

	HeightFieldData fieldData;

	f32	getMaxHeight() const {return fieldData.maxHeight;}
	f32 getMinHeight() const {return fieldData.minHeight;}
	f32 getFieldWidth() const {return fieldData.fieldWidth;}
	f32 getFieldDepth() const {return fieldData.fieldDepth;}

	const Vector3&	getScale() const {return fieldScale;}
	void setScale(const Vector3& scale) {fieldScale = scale;}

	inline Vector3 localToWorldPosition(const Vector3& localPosition) const;
	inline Vector3 worldToLocalPosition(const Vector3& worldPosition) const;
};

inline Vector3 HeightField::localToWorldPosition(const Vector3& localPosition) const
{
	return mulPerElem(fieldScale, (localPosition - 0.5f*Vector3(fieldData.fieldWidth, 0.0f, fieldData.fieldDepth)));
}

inline Vector3 HeightField::worldToLocalPosition(const Vector3& worldPosition) const
{
	return divPerElem(worldPosition, fieldScale) + 0.5f*Vector3(fieldData.fieldWidth, 0.0f, fieldData.fieldDepth);
}

///////////////////////////////////////////////////////////////////////////////
//  Inline Function

#ifndef __SPU__

inline f32 getFieldData(const HeightFieldData& fieldData, s32 i, s32 j)
{
	i = MIN(fieldData.numFieldX - 1, i);
	j = MIN(fieldData.numFieldZ - 1, j);
	s32 bi = i/BLOCK_SIZE;
	s32 bj = j/BLOCK_SIZE;
	s32 ci = i - bi*BLOCK_SIZE;
	s32 cj = j - bj*BLOCK_SIZE;

	return fieldData.blocks[bj*fieldData.numBlockX + bi].heightBuf[cj*BLOCK_SIZE_B + ci];
}

inline void setFieldData(HeightFieldData& fieldData, s32 i, s32 j, f32 v)
{
	i = MIN(fieldData.numFieldX - 1, i);
	j = MIN(fieldData.numFieldZ - 1, j);
	s32 bi = i/BLOCK_SIZE;
	s32 bj = j/BLOCK_SIZE;
	s32 ci = i - bi*BLOCK_SIZE;
	s32 cj = j - bj*BLOCK_SIZE;

	fieldData.blocks[bj*fieldData.numBlockX + bi].heightBuf[cj*BLOCK_SIZE_B + ci] = v;

	if(ci == 0 && cj == 0) {
		if(bi > 0)
			fieldData.blocks[bj*fieldData.numBlockX + (bi - 1)].heightBuf[cj*BLOCK_SIZE_B + BLOCK_SIZE] = v;

		if(bj > 0)
			fieldData.blocks[(bj - 1)*fieldData.numBlockX + bi].heightBuf[BLOCK_SIZE*BLOCK_SIZE_B + ci] = v;

		if(bi > 0 && bj > 0)
			fieldData.blocks[(bj - 1)*fieldData.numBlockX + (bi - 1)].heightBuf[BLOCK_SIZE*BLOCK_SIZE_B + BLOCK_SIZE] = v;
	} else if(ci == 0) {
		if(bi > 0)
			fieldData.blocks[bj*fieldData.numBlockX + (bi - 1)].heightBuf[cj*BLOCK_SIZE_B + BLOCK_SIZE] = v;
	} else if(cj == 0) {
		if(bj > 0)
			fieldData.blocks[(bj - 1)*fieldData.numBlockX + bi].heightBuf[BLOCK_SIZE*BLOCK_SIZE_B + ci] = v;
	}
}

#endif

#endif /* HEIGHTFIELD_H_ */
