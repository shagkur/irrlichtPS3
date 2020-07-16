/*
 * dimension2d.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef DIMENSION2D_H_
#define DIMENSION2D_H_

#include "irrtypes.h"

namespace irr
{
	namespace core
	{
		template< class T >
		class vector2d;

		template< class T >
		class dimension2d
		{
		public:
			dimension2d() : width(0), height(0) {};
			dimension2d(const dimension2d<T>& other) : width(other.width), height(other.height) {};
			dimension2d(const T& w,const T& h) : width(w), height(h) {};

			dimension2d(const vector2d<T>& other);

			template< class U >
			explicit dimension2d(const dimension2d<U>& other) : width((T)other.width), height((T)other.height) {};

			template< class U >
			dimension2d<T>& operator=(const dimension2d<U>& other)
			{
				width = (T)other.width;
				height = (T)other.height;
				return *this;
			}

			bool operator==(const dimension2d<T>& other) const
			{
				return (width==other.width && height==other.height);
			}

			bool operator!=(const dimension2d<T>& other) const
			{
				return !(*this==other);
			}

			bool operator==(const vector2d<T>& other) const;

			bool operator!=(const vector2d<T>& other) const
			{
				return !(*this == other);
			}

			dimension2d<T>& set(const T& w,const T& h)
			{
				width = w;
				height = h;
				return *this;
			}

			dimension2d<T>& operator/=(const T& scale)
			{
				width /= scale;
				height /= scale;
				return *this;
			}

			dimension2d<T> operator/(const T& scale) const
			{
				return dimension2d<T>(width/scale,height/scale);
			}

			dimension2d<T>& operator*=(const T& scale)
			{
				width *= scale;
				height *= scale;
				return *this;
			}

			dimension2d<T> operator*(const T& scale) const
			{
				return dimension2d<T>(width*scale,height*scale);
			}

			T getArea() const
			{
				return width*height;
			}

			dimension2d<T> getInterpolated(const dimension2d<T>& other,f32 d) const
			{
				T inv = (T)(1.0f - d);
				return dimension2d<T>(other.width*inv +  width*d,other.height*inv + height*d);
			}

			dimension2d<T> getOptimalSize(bool requirePowerOfTwo = true, bool requireSquare = false, bool large = true, u32 maxValue = 0) const
			{
				u32 i = 1;
				u32 j = 1;

				if(requirePowerOfTwo) {
					while(i < (u32)width) i <<= 1;
					if(!large && i != 1 && i != (u32)width) i >>= 1;

					while(j < (u32)height) j <<= 1;
					if(!large && j != 1 && j != (u32)height) j >>= 1;
				} else {
					i = (u32)width;
					j = (u32)height;
				}

				if(requireSquare) {
					if((large && i > j) || (!large && i < j))
						j = i;
					else
						i = j;
				}

				if(maxValue > 0 && i > maxValue) i = maxValue;
				if(maxValue > 0 && j > maxValue) j = maxValue;

				return dimension2d<T>((T)i, (T)j);
			}

			T width;
			T height;
		};

		typedef dimension2d<f32> dimension2df;
		typedef dimension2d<s32> dimension2di;
	}
}

#endif /* DIMENSION2D_H_ */
