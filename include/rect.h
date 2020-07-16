/*
 * rect.h
 *
 *  Created on: Feb 21, 2013
 *      Author: mike
 */

#ifndef RECT_H_
#define RECT_H_

#include "irrtypes.h"
#include "dimension2d.h"
#include "position2d.h"

namespace irr
{
	namespace core
	{
		template< class T >
		class rect
		{
		public:
			rect() : upperLeftCorner(0, 0), lowerRightCorner(0, 0) {}
			rect(T x0, T y0, T x1, T y1) : upperLeftCorner(x0, y0), lowerRightCorner(x1, y1) {}
			rect(const position2d<T>& upperLeft, const position2d<T>& lowerRight) : upperLeftCorner(upperLeft), lowerRightCorner(lowerRight) {}

			template< class U >
			rect(const position2d<T>& pos, const dimension2d<U>& size) : upperLeftCorner(pos), lowerRightCorner(pos.X + size.width, pos.Y + size.height) {}

			rect<T> operator + (const position2d<T>& pos) const
			{
				rect<T> ret(*this);
				return ret += pos;
			}

			rect<T>& operator += (const position2d<T>& pos)
			{
				upperLeftCorner += pos;
				lowerRightCorner += pos;
				return *this;
			}

			rect<T> operator - (const position2d<T>& pos) const
			{
				rect<T> ret(*this);
				return ret -= pos;
			}

			rect<T>& operator -= (const position2d<T>& pos)
			{
				upperLeftCorner -= pos;
				lowerRightCorner -= pos;
				return *this;
			}

			bool operator == (const rect<T>& other) const
			{
				return (upperLeftCorner == other.upperLeftCorner && lowerRightCorner == other.lowerRightCorner);
			}

			bool operator != (const rect<T>& other) const
			{
				return (upperLeftCorner != other.upperLeftCorner || lowerRightCorner != other.lowerRightCorner);
			}

			bool operator < (const rect<T>& other) const
			{
				return getArea() < other.getArea();
			}

			T getArea() const
			{
				return getWidth()*getHeight();
			}

			bool isPointInside(const position2d<T>& pos) const
			{
				return (upperLeftCorner.X <= pos.X &&
						upperLeftCorner.X <= pos.Y &&
						lowerRightCorner.X >= pos.X &&
						lowerRightCorner.Y >= pos.Y);
			}

			bool isRectCollided(const rect<T>& other) const
			{
				return (lowerRightCorner.Y > other.upperLeftCorner.Y &&
						upperLeftCorner.Y < other.lowerRightCorner.Y &&
						lowerRightCorner.X > other.upperLeftCorner.X &&
						upperLeftCorner.X < other.lowerRightCorner.X);
			}

			void clipAgainst(const rect<T>& other)
			{
				if(other.lowerRightCorner.X < lowerRightCorner.X)
					lowerRightCorner.X = other.lowerRightCorner.X;
				if(other.lowerRightCorner.Y < lowerRightCorner.Y)
					lowerRightCorner.Y = other.lowerRightCorner.Y;

				if(other.upperLeftCorner.X > upperLeftCorner.X)
					upperLeftCorner.X = other.upperLeftCorner.X;
				if(other.upperLeftCorner.Y > upperLeftCorner.Y)
					upperLeftCorner.Y = other.upperLeftCorner.Y;

				if(upperLeftCorner.Y > lowerRightCorner.Y)
					upperLeftCorner.Y = lowerRightCorner.Y;
				if(upperLeftCorner.X > lowerRightCorner.X)
					upperLeftCorner.X = lowerRightCorner.X;
			}

			bool constrainTo(const rect<T>& other)
			{
				if(other.getWidth() < getWidth() || other.getHeight() < getHeight())
					return false;

				T diff = other.lowerRightCorner.X - lowerRightCorner.X;
				if(diff < 0) {
					lowerRightCorner.X += diff;
					upperLeftCorner.X += diff;
				}

				diff = other.lowerRightCorner.Y - lowerRightCorner.Y;
				if(diff < 0) {
					lowerRightCorner.Y += diff;
					upperLeftCorner.Y += diff;
				}

				diff = upperLeftCorner.X - other.upperLeftCorner.X;
				if(diff < 0) {
					upperLeftCorner.X -= diff;
					lowerRightCorner.X -= diff;
				}

				diff = upperLeftCorner.Y - other.upperLeftCorner.Y;
				if(diff < 0) {
					upperLeftCorner.Y -= diff;
					lowerRightCorner.Y -= diff;
				}

				return true;
			}

			T getWidth() const
			{
				return lowerRightCorner.X - upperLeftCorner.X;
			}

			T getHeight() const
			{
				return lowerRightCorner.Y - upperLeftCorner.Y;
			}

			void repair()
			{
				if(lowerRightCorner.X < upperLeftCorner.X) {
					T t = lowerRightCorner.X;
					lowerRightCorner.X = upperLeftCorner.X;
					upperLeftCorner.X = t;
				}

				if(lowerRightCorner.Y < upperLeftCorner.Y) {
					T t = lowerRightCorner.Y;
					lowerRightCorner.Y = upperLeftCorner.Y;
					upperLeftCorner.Y = t;
				}
			}

			bool isValid() const
			{
				return (lowerRightCorner.X >= upperLeftCorner.X && lowerRightCorner.Y >= upperLeftCorner.Y);
			}

			position2d<T> getSize() const
			{
				return dimension2d<T>(getWidth(), getHeight());
			}

			void addInternalPoint(const position2d<T>& p)
			{
				addInternalPoint(p.X, p.Y);
			}

			void addInternalPoint(T x, T y)
			{
				if(x > lowerRightCorner.X)
					lowerRightCorner.X = x;
				if(y > lowerRightCorner.Y)
					lowerRightCorner.Y = y;

				if(x < upperLeftCorner.X)
					upperLeftCorner.X = x;
				if(y < upperLeftCorner.Y)
					upperLeftCorner.Y = y;
			}

			position2d<T> upperLeftCorner;
			position2d<T> lowerRightCorner;
		};

		typedef rect<f32> rectf;
		typedef rect<s32> recti;
	}
}


#endif /* RECT_H_ */
