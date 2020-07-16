/*
 * vector2d.h
 *
 *  Created on: Feb 21, 2013
 *      Author: mike
 */

#ifndef VECTOR2D_H_
#define VECTOR2D_H_

#include "irrmath.h"
#include "dimension2d.h"

namespace irr
{
	namespace core
	{
		template< class T >
		class vector2d
		{
		public:
			vector2d() : X(0), Y(0) {}
			vector2d(T nx, T ny) : X(nx), Y(ny) {}
			vector2d(const vector2d<T>& other) : X(other.X), Y(other.Y) {}
			vector2d(const dimension2d<T>& other) : X(other.width), Y(other.height) {}

			explicit vector2d(T n) : X(n), Y(n) {}

			vector2d<T> operator -() const { return vector2d<T>(-X, -Y); }

			vector2d<T>& operator = (const vector2d<T>& other) { X = other.X; Y = other.Y; return *this; }
			vector2d<T>& operator = (const dimension2d<T>& other) { X = other.width; Y = other.height; return *this; }

			vector2d<T> operator + (const vector2d<T>& other) const { return vector2d<T>(X + other.X, Y + other.Y); }
			vector2d<T> operator + (const dimension2d<T>& other) const { return vector2d<T>(X + other.width, Y + other.height); }
			vector2d<T> operator + (const T v) const { return vector2d<T>(X + v, Y + v); }
			vector2d<T>& operator += (const vector2d<T>& other) { X += other.X; Y += other.Y; return *this; }
			vector2d<T>& operator += (const dimension2d<T>& other) { X += other.width; Y += other.height; return *this; }
			vector2d<T>& operator += (const T v) { X += v; Y += v; return *this; }

			vector2d<T> operator - (const vector2d<T>& other) const { return vector2d<T>(X - other.X, Y - other.Y); }
			vector2d<T> operator - (const dimension2d<T>& other) const { return vector2d<T>(X - other.width, Y - other.height); }
			vector2d<T> operator - (const T v) const { return vector2d<T>(X - v, Y - v); }
			vector2d<T>& operator -= (const vector2d<T>& other) { X -= other.X; Y -= other.Y; return *this; }
			vector2d<T>& operator -= (const dimension2d<T>& other) { X -= other.width; Y -= other.height; return *this; }
			vector2d<T>& operator -= (const T v) { X -= v; Y -= v; return *this; }

			vector2d<T> operator * (const vector2d<T>& other) const { return vector2d<T>(X*other.X, Y*other.Y); }
			vector2d<T> operator * (const T v) const { return vector2d<T>(X*v, Y*v); }
			vector2d<T>& operator *= (const vector2d<T>& other) { X *= other.X; Y *= other.Y; return *this; }
			vector2d<T>& operator *= (const T v) { X *= v; Y *= v; return *this; }

			vector2d<T> operator / (const vector2d<T>& other) const { return vector2d<T>(X/other.X, Y/other.Y); }
			vector2d<T> operator / (const T v) const { return vector2d<T>(X/v, Y/v); }
			vector2d<T>& operator /= (const vector2d<T>& other) { X /= other.X; Y /= other.Y; return *this; }
			vector2d<T>& operator /= (const T v) { X /= v; Y /= v; return *this; }

			bool operator <= (const vector2d<T>& other) const
			{
				return (X < other.X || core::equals(X, other.X)) ||
					   (core::equals(X, other.X) && (Y < other.Y || core::equals(Y, other.Y)));
			}

			bool operator >= (const vector2d<T>& other) const
			{
				return (X > other.X || core::equals(X, other.X)) ||
					   (core::equals(X, other.X) && (Y > other.Y || core::equals(Y, other.Y)));
			}

			bool operator < (const vector2d<T>& other) const
			{
				return (X < other.X && !core::equals(X, other.X)) ||
					   (core::equals(X, other.X) && Y < other.Y && !core::equals(Y, other.Y));
			}

			bool operator > (const vector2d<T>& other) const
			{
				return (X > other.X && !core::equals(X, other.X)) ||
					   (core::equals(X, other.X) && Y > other.Y && !core::equals(Y, other.Y));
			}

			bool operator == (const vector2d<T>& other) const { return equals(other); }
			bool operator != (const vector2d<T>& other) const { return !equals(other); }

			bool equals(const vector2d<T>& other) const
			{
				return core::equals(X, other.X) && core::equals(Y, other.Y);
			}

			T getLength() const { return core::sqrtf32(X*X + Y*Y); }
			T getLengthSQ() const { return X*X + Y*Y; }

			T dotProduct(const vector2d<T>& other) const
			{
				return X*other.X + Y*other.Y;
			}

			T getDistanceFrom(const vector2d<T>& other) const
			{
				return vector2d<T>(X - other.X, Y - other.Y).getLength();
			}

			T getDistanceFromSQ(const vector2d<T>& other) const
			{
				return vector2d<T>(X - other.X, Y - other.Y).getLengthSQ();
			}

			vector2d<T>& normalize()
			{
				f32 length = (f32)(X*X + Y*Y);

				if(length == 0) return *this;

				length = core::reciprocal_sqrtf32(length);

				X = (T)(X * length);
				Y = (T)(Y * length);

				return *this;
			}

			union {
				T V[2];
				struct {
					T X,Y;
				};
			};
		};

		typedef vector2d<f32> vector2df;
		typedef vector2d<s32> vector2di;

		template< class S, class T >
		vector2d<T> operator * (const S scalar, const vector2d<T>& vector) { return vector*scalar; }

		template< class T >
		dimension2d<T>::dimension2d(const vector2d<T>& other) : width(other.X), height(other.Y) {}

		template< class T >
		bool dimension2d<T>::operator == (const vector2d<T>& other) const { return (width == other.X && height == other.Y); }
	}
}

#endif /* VECTOR2D_H_ */
