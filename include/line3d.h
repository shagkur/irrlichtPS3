#ifndef __LINE3D_H__
#define __LINE3D_H__ 

#include "irrtypes.h"
#include "vector3d.h"

namespace irr
{
	namespace core
	{
		class line3df
		{
		public:
			line3df() : start(0,0,0),end(1,1,1) {};
			line3df(f32 xa,f32 ya,f32 za,f32 xb,f32 yb,f32 zb) : start(xa,ya,za),end(xb,yb,zb) {};
			line3df(const vector3df& p1,const vector3df& p2) : start(p1),end(p2) {};

			line3df operator+(const vector3df& point) const { return line3df((start+point),(end+point)); }
			line3df& operator+=(const vector3df& point) { start += point; end += point; return *this; }

			line3df operator-(const vector3df& point) const { return line3df((start-point),(end-point)); }
			line3df& operator-=(const vector3df& point) { start -= point; end -= point; return *this; }

			bool operator==(const line3df& other) const
			{ return ((start==other.start && end==other.end) || (end==other.start && start==other.end)); }
			bool operator!=(const line3df& other) const
			{ return !((start==other.start && end==other.end) || (end==other.start && start==other.end)); }

			bool isPointBetweenStartAndEnd(const vector3df& point) const
			{
				f32 f = lengthSqr(end - start);
				return (lengthSqr(point - start) < f && lengthSqr(point - end) < f);
			}

			bool getIntersectionWithSphere(vector3df sorigin,f32 sradius,f32& outD) const
			{
				const vector3df q = sorigin - start;
				f32 c = length(q);
				f32 v = dot(q, normalize(getVector()));
				f32 d = sradius*sradius - (c*c - v*v);

				if(d<(f32)0.0f)
					return false;

				outD = v - sqrtf32((f32)d);
				return true;
			}

			void setLine(const f32& xa,const f32& ya,const f32& za,const f32& xb,const f32& yb,const f32& zb)
			{ start = vector3df(xa,ya,za); end = vector3df(xb,yb,zb); }
			
			void setLine(const vector3df& nstart,const vector3df& nend)
			{ start = nstart; end = nend; }

			void setLine(const line3df& other)
			{ start = other.start; end = other.end; }

			f32 getLength() const { return length(start - end); }
			f32 getLengthSQ() const { return lengthSqr(start - end); }

			vector3df getMiddle() const
			{
				return (start + end)*(f32)0.5f;
			}

			vector3df getVector() const
			{
				return (end - start);
			}

			vector3df getClosestPoint(const vector3df& point) const
			{
				vector3df c = point - start;
				vector3df v = end - start;
				f32 t,d = (f32)length(v);

				v /= d;
				t = dot(v, c);
				
				if(t<(f32)0.0f)
					return start;
				if(t>d)
					return end;

				v *= t;
				return start+v;
			}

			vector3df start,end;
		};
	}
}

#endif
