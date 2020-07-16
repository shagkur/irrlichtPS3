/*
 * heightfluidsurface.h
 *
 *  Created on: Jun 11, 2013
 *      Author: mike
 */

#ifndef HEIGHTFLUIDSURFACE_H_
#define HEIGHTFLUIDSURFACE_H_

#include "base/common.h"

#include "heightfluid/common/heightfluidconfig.h"

class HeightFluidSurface
{
	friend class HeightFluid;

public:
	u32 fieldWidth;
	u32 fieldDepth;

	ATTRIBUTE_ALIGNED16(FieldPoint *fields[2]);

	ATTRIBUTE_ALIGNED16(Vector3 *hfVertex[2]);
	ATTRIBUTE_ALIGNED16(Vector3 *hfNormal[2]);
	ATTRIBUTE_ALIGNED16(f32 *hfTexCoords);
	ATTRIBUTE_ALIGNED16(u16 *hfMeshIndices);

private:
	f32 cConst;
	f32 kConst;
	f32 deltaX;
	f32 downCurrWave;
	f32 downPrevWave;
	f32 limitHeight;

	f32 buoyPower;
	f32 buoyDamping;

	f32 heightMin;
	f32 heightMax;
	f32 amplitude;

	Vector3 fieldScale;
	Vector3 fieldPosition;
	Quat fieldOrientation;

	u8 writeBuffer;
	u8 readBuffer;

	f32 totalWater;

	void swapBuffers()
	{
		writeBuffer = 1 - writeBuffer;
		readBuffer = 1 - readBuffer;
	}

	void copyFields();

public:
	HeightFluidSurface()
	{
		fields[0] = NULL;
		fields[1] = NULL;
		hfVertex[0] = NULL;
		hfVertex[1] = NULL;
		hfNormal[0] = NULL;
		hfNormal[1] = NULL;
		hfTexCoords = NULL;
		hfMeshIndices = NULL;

		buoyPower = 1.0f;
		buoyDamping = 0.1f;
		fieldScale = Vector3(1.0f);
		fieldPosition = Vector3(0.0f);
		fieldOrientation = Quat::identity();
		cConst = 10.0f;
		kConst = 0.5f;
		deltaX = 1.0f;
		downPrevWave = 0.05f;
		downCurrWave = 0.10f;
		limitHeight = 0.7f;

		writeBuffer = 0;
		readBuffer = 1;

		amplitude = 1.0f;
		heightMin = 0.0f;
		heightMax = 2.0f;
	}

	void initialize();

	inline Vector3 localToWorldPosition(const Vector3& localPosition);
	inline Vector3 worldToLocalPosition(const Vector3& worldPosition);
	inline Vector3 localToWorldVector(const Vector3& localVector);
	inline Vector3 worldToLocalVector(const Vector3& worldVector);

	inline s32 getVertexCount() {return fieldWidth*fieldDepth;}
	inline s32 getIndexCount() {return 6*(fieldWidth - 1)*(fieldDepth - 1);}

	inline Vector3 *getVertices() {return hfVertex[readBuffer];}
	inline Vector3 *getNormals() {return hfNormal[readBuffer];}
	inline f32 *getTexCoords() {return hfTexCoords;}
	inline u16 *getIndices() {return hfMeshIndices;}

	inline f32 getHeight(s32 i, s32 j);

	inline f32 getFieldHeight(s32 i, s32 j);
	inline void	setFieldHeight(s32 i, s32 j, f32 h);

	inline f32 getOffsetHeight(s32 i, s32 j);
	inline void	setOffsetHeight(s32 i, s32 j, f32 h);

	inline u8 getFixedSurface(s32 i, s32 j);
	inline void setFixedSurface(s32 i, s32 j, u8 b);

	inline Vector3 getPosition() {return fieldPosition;}
	inline void setPosition(const Vector3& position) {fieldPosition = position;}

	inline Quat getOrientation() {return fieldOrientation;}
	inline void setOrientation(const Quat& orientation) {fieldOrientation = orientation;}

	inline f32 getScaleW() {return fieldScale[0];}
	inline void setScaleW(f32 scaleW) {fieldScale[0] = fieldScale[2] = scaleW;}

	inline f32 getScaleH() {return fieldScale[1];}
	inline void setScaleH(f32 scaleH) {fieldScale[1] = scaleH;}

	inline f32 getWaveVelocity() {return cConst;}
	inline void setWaveVelocity(f32 value) {cConst = value;}

	inline f32 getWaveDamping() {return kConst;}
	inline void setWaveDamping(f32 value) {kConst = value;}

	inline f32 getDownCurrWave() {return downCurrWave;}
	inline void setDownCurrWave(f32 value) {downCurrWave = value;}

	inline f32 getDownPrevWave() {return downPrevWave;}
	inline void setDownPrevWave(f32 value) {downPrevWave = value;}

	inline f32 getLimitHeight() {return limitHeight;}
	inline void setLimitHeight(f32 value) {limitHeight = value;}

	inline f32 getBuoyancyPower() {return buoyPower;}
	inline void setBuoyancyPower(f32 value) {buoyPower = value;}

	inline f32 getBuoyancyDamping() {return buoyDamping;}
	inline void setBuoyancyDamping(f32 value) {buoyDamping = value;}

	inline f32 getAmplitude() {return amplitude;}
	inline void setAmplitude(f32 value) {amplitude = value;}

	inline f32 getHeightMin() {return heightMin;}
	inline void setHeightMin(f32 value) {heightMin = value;}

	inline f32 getHeightMax() {return heightMax;}
	inline void setHeightMax(f32 value) {heightMax = value;}
};

inline f32 HeightFluidSurface::getHeight(s32 i, s32 j)
{
	return getOffsetHeight(i, j) + getFieldHeight(i, j);
}

inline void HeightFluidSurface::setOffsetHeight(s32 i, s32 j, f32 h)
{
	fields[readBuffer][j*fieldWidth + i].offset = (u8)((h/amplitude + 0.5f)*255.0f);
}

inline f32 HeightFluidSurface::getOffsetHeight(s32 i, s32 j)
{
	u32 offset = fields[readBuffer][j*fieldWidth + i].offset;
	return ((offset/255.0f) - 0.5f)*amplitude;
}

inline void HeightFluidSurface::setFieldHeight(s32 i, s32 j, f32 h)
{
	FieldPoint& fp = fields[readBuffer][j*fieldWidth + i];
	fp.height[0] = fp.height[1] = fp.height[2] = h;
}

inline f32 HeightFluidSurface::getFieldHeight(s32 i, s32 j)
{
	return fields[readBuffer][j*fieldWidth + i].height[2];
}

inline void HeightFluidSurface::setFixedSurface(s32 i, s32 j, u8 b)
{
	fields[readBuffer][j*fieldWidth + i].flag = b;
}

inline u8 HeightFluidSurface::getFixedSurface(s32 i, s32 j)
{
	return fields[readBuffer][j*fieldWidth + i].flag;
}

inline Vector3 HeightFluidSurface::localToWorldPosition(const Vector3& localPosition)
{
	return fieldPosition + rotate(fieldOrientation, mulPerElem(fieldScale, (localPosition - 0.5f*Vector3(fieldWidth, 0.0f, fieldDepth))));
}

inline Vector3 HeightFluidSurface::worldToLocalPosition(const Vector3& worldPosition)
{
	return divPerElem(rotate(conj(fieldOrientation), (worldPosition - fieldPosition)), fieldScale) + 0.5f*Vector3(fieldWidth, 0.0f, fieldDepth);
}

inline Vector3 HeightFluidSurface::localToWorldVector(const Vector3& localVector)
{
	return rotate(fieldOrientation, mulPerElem(fieldScale, localVector));
}

inline Vector3 HeightFluidSurface::worldToLocalVector(const Vector3& worldVector)
{
	return divPerElem(rotate(conj(fieldOrientation), worldVector), fieldScale);
}

#endif /* HEIGHTFLUIDSURFACE_H_ */
