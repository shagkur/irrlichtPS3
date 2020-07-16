/*
 * vecint3.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef VECINT3_H_
#define VECINT3_H_

#include "common.h"

#ifndef __SPU__

ATTRIBUTE_ALIGNED16(class) VecInt3
{
private:
	s32 mX,mY,mZ,mW;

public:
	VecInt3() {mX = mY = mZ = mW = 0;}
	VecInt3(const Vector3& vec) {mX = (s32)vec[0]; mY = (s32)vec[1]; mZ = (s32)vec[2]; mW = 0;}
	VecInt3(f32 fx, f32 fy, f32 fz) {mX = (s32)fx; mY = (s32)fy; mZ = (s32)fz; mW = 0;}
	VecInt3(s32 iv) {mX = mY = mZ = iv; mW = 0;}
	VecInt3(s32 ix,s32 iy,s32 iz) {mX = ix; mY = iy; mZ = iz; mW = 0;}

    inline VecInt3& operator=(const VecInt3& vec);

	inline s32 get(s32 i) const {return *(&mX + i);}
	inline s32 getX() const {return mX;}
	inline s32 getY() const {return mY;}
	inline s32 getZ() const {return mZ;}
	inline void set(s32 i, s32 v) {*(&mX + i) = v;}
	inline void setX(s32 v) {mX = v;}
	inline void setY(s32 v) {mY = v;}
	inline void setZ(s32 v) {mZ = v;}

    inline const VecInt3 operator+(const VecInt3& vec) const;
    inline const VecInt3 operator-(const VecInt3& vec) const;
    inline const VecInt3 operator*(s32 scalar) const;
    inline const VecInt3 operator/(s32 scalar) const;

    inline VecInt3& operator+=(const VecInt3& vec);
    inline VecInt3& operator-=(const VecInt3& vec);
    inline VecInt3& operator*=(s32 scalar);
    inline VecInt3& operator/=(s32 scalar);

    inline const VecInt3 operator-() const;

	operator Vector3() const
	{
		return Vector3((f32)mX, (f32)mY, (f32)mZ);
	}
};

inline VecInt3& VecInt3::operator=(const VecInt3& vec)
{
    mX = vec.mX;
    mY = vec.mY;
    mZ = vec.mZ;
    return *this;
}

inline const VecInt3 VecInt3::operator+(const VecInt3& vec) const
{
    return VecInt3(mX + vec.mX, mY + vec.mY, mZ + vec.mZ);
}

inline const VecInt3 VecInt3::operator-(const VecInt3& vec) const
{
    return VecInt3(mX - vec.mX, mY - vec.mY, mZ - vec.mZ);
}

inline const VecInt3 VecInt3::operator*(s32 scalar) const
{
    return VecInt3(mX*scalar, mY*scalar, mZ*scalar);
}

inline const VecInt3 VecInt3::operator/(s32 scalar) const
{
    return VecInt3(mX/scalar, mY/scalar, mZ/scalar);
}

inline VecInt3& VecInt3::operator+=(const VecInt3& vec)
{
    *this = *this + vec;
    return *this;
}

inline VecInt3& VecInt3::operator-=(const VecInt3& vec)
{
    *this = *this - vec;
    return *this;
}

inline VecInt3& VecInt3::operator*=(s32 scalar)
{
    *this = *this*scalar;
    return *this;
}

inline VecInt3& VecInt3::operator/=(s32 scalar)
{
    *this = *this/scalar;
    return *this;
}

inline const VecInt3 VecInt3::operator-() const
{
	return VecInt3(-mX, -mY, -mZ);
}

inline const VecInt3 operator*(s32 scalar, const VecInt3& vec)
{
    return vec*scalar;
}

inline const VecInt3 mulPerElem( const VecInt3& vec0, const VecInt3& vec1)
{
	return VecInt3(vec0.getX()*vec1.getX(), vec0.getY()*vec1.getY(), vec0.getZ()*vec1.getZ());
}

inline const VecInt3 divPerElem(const VecInt3& vec0, const VecInt3& vec1)
{
	return VecInt3(vec0.getX()/vec1.getX(), vec0.getY()/vec1.getY(), vec0.getZ()/vec1.getZ());
}

inline const VecInt3 absPerElem(const VecInt3& vec)
{
	return VecInt3(abs(vec.getX()), abs(vec.getY()), abs(vec.getZ()));
}

inline const VecInt3 maxPerElem(const VecInt3& vec0, const VecInt3& vec1)
{
    return VecInt3(
        (vec0.getX() > vec1.getX()) ? vec0.getX() : vec1.getX(),
        (vec0.getY() > vec1.getY()) ? vec0.getY() : vec1.getY(),
        (vec0.getZ() > vec1.getZ()) ? vec0.getZ() : vec1.getZ()
    );
}

inline const VecInt3 minPerElem(const VecInt3& vec0, const VecInt3& vec1)
{
    return VecInt3(
        (vec0.getX() < vec1.getX()) ? vec0.getX() : vec1.getX(),
        (vec0.getY() < vec1.getY()) ? vec0.getY() : vec1.getY(),
        (vec0.getZ() < vec1.getZ()) ? vec0.getZ() : vec1.getZ()
    );
}

#else // __SPU__

ATTRIBUTE_ALIGNED16(class) VecInt3
{
private:
	vec_int4 mVec128;

public:
	VecInt3() {mVec128 = spu_splats(0);}
	VecInt3(const Vector3& vec) {mVec128 = spu_convts(vec.get128(), 0);}
	VecInt3(f32 fx, f32 fy, f32 fz) {mVec128 = ((vec_int4){(s32)fx, (s32)fy, (s32)fz, 0});}
	VecInt3(s32 iv) {mVec128 = spu_splats(iv);}
	VecInt3(s32 ix, s32 iy, s32 iz) {mVec128 = ((vec_int4){ix, iy, iz, 0});}
	VecInt3(vec_int4 vec) {mVec128 = vec;}

    inline VecInt3& operator=(const VecInt3& vec);

	inline s32 get(s32 i) const {return spu_extract(mVec128, i);}
	inline s32 getX() const {return spu_extract(mVec128, 0);}
	inline s32 getY() const {return spu_extract(mVec128, 1);}
	inline s32 getZ() const {return spu_extract(mVec128, 2);}
	inline void set(s32 i, s32 v) {spu_insert(v, mVec128, i);}
	inline void setX(s32 v) {spu_insert(v, mVec128, 0);}
	inline void setY(s32 v) {spu_insert(v, mVec128, 1);}
	inline void setZ(s32 v) {spu_insert(v, mVec128, 2);}

    inline const VecInt3 operator+(const VecInt3& vec) const;
    inline const VecInt3 operator-(const VecInt3& vec) const;
    inline const VecInt3 operator*(s32 scalar) const;
    inline const VecInt3 operator/(s32 scalar) const;

    inline VecInt3 & operator+=(const VecInt3& vec);
    inline VecInt3 & operator-=(const VecInt3& vec);
    inline VecInt3 & operator*=(s32 scalar);
    inline VecInt3 & operator/=(s32 scalar);

    inline const VecInt3 operator-() const;

	operator Vector3() const
	{
		return Vector3(spu_convtf(mVec128, 0));
	}

	inline vec_int4 get128( ) const
	{
		return mVec128;
	}
};

inline VecInt3& VecInt3::operator=(const VecInt3& vec)
{
	mVec128 = vec.mVec128;
    return *this;
}

inline const VecInt3 VecInt3::operator+(const VecInt3& vec) const
{
    return VecInt3(spu_add(mVec128, vec.mVec128));
}

inline const VecInt3 VecInt3::operator-(const VecInt3& vec) const
{
    return VecInt3(spu_sub(mVec128, vec.mVec128));
}

inline const VecInt3 VecInt3::operator*(s32 scalar) const
{
   vec_short8 lhs_short = (vec_short8)mVec128;
   vec_short8 rhs_short = (vec_short8)spu_splats(scalar);

   return VecInt3((vec_int4)spu_add(spu_add(spu_mulh(lhs_short, rhs_short),
                                            spu_mulh(rhs_short, lhs_short)),
                                  (vec_int4)spu_mulo((vec_ushort8)lhs_short, (vec_ushort8)rhs_short)));
}

inline const VecInt3 VecInt3::operator/(s32 scalar) const
{
	return VecInt3(divi4(mVec128, spu_splats(scalar)).quot);
}

inline VecInt3 &VecInt3::operator+=(const VecInt3& vec)
{
    *this = *this + vec;
    return *this;
}

inline VecInt3& VecInt3::operator-=(const VecInt3& vec)
{
    *this = *this - vec;
    return *this;
}

inline VecInt3& VecInt3::operator*=(s32 scalar)
{
    *this = *this*scalar;
    return *this;
}

inline VecInt3& VecInt3::operator/=(s32 scalar)
{
    *this = *this/scalar;
    return *this;
}

inline const VecInt3 VecInt3::operator-() const
{
    return VecInt3(spu_sub((s32)0, mVec128));
}

inline const VecInt3 operator*(s32 scalar, const VecInt3& vec)
{
    return vec*scalar;
}

inline const VecInt3 mulPerElem(const VecInt3& vec0, const VecInt3& vec1)
{
   vec_short8 lhs_short = (vec_short8)vec0.get128();
   vec_short8 rhs_short = (vec_short8)vec1.get128();

   return VecInt3((vec_int4)spu_add(spu_add(spu_mulh(lhs_short, rhs_short),
                                            spu_mulh(rhs_short, lhs_short)),
                                  (vec_int4)spu_mulo((vec_ushort8)lhs_short, (vec_ushort8)rhs_short)));
}

inline const VecInt3 divPerElem(const VecInt3& vec0, const VecInt3& vec1)
{
	return VecInt3(divi4(vec0.get128(), vec1.get128()).quot);
}

inline const VecInt3 absPerElem(const VecInt3 & vec)
{
	return VecInt3(absi4(vec.get128()));
}

inline const VecInt3 maxPerElem(const VecInt3& vec0, const VecInt3& vec1)
{
	return VecInt3(spu_sel(vec1.get128(), vec0.get128(), spu_cmpgt(vec0.get128(), vec1.get128())));
}

inline const VecInt3 minPerElem(const VecInt3& vec0, const VecInt3& vec1)
{
	return VecInt3(spu_sel(vec0.get128(), vec1.get128(), spu_cmpgt(vec0.get128(), vec1.get128())));
}

#endif // __SPU__

#endif /* VECINT3_H_ */
