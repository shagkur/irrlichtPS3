#ifndef __plane3df_H__
#define __plane3df_H__

#include "irrmath.h"
#include "vector3d.h"

namespace irr
{
	namespace core
	{
		//! Enumeration for intersection relations of 3d objects
		enum EIntersectionRelation3D
		{
			ISREL3D_FRONT = 0,
			ISREL3D_BACK,
			ISREL3D_PLANAR,
			ISREL3D_SPANNING,
			ISREL3D_CLIPPED
		};

		class plane3df
		{
		public:
			plane3df() : normal(0,1,0) { recalculateD(vector3df(0,0,0)); }
			plane3df(const vector3df& mpoint,const vector3df& nrm) : normal(nrm) { recalculateD(mpoint); }
			plane3df(f32 px,f32 py,f32 pz,f32 nx,f32 ny,f32 nz) : normal(nx,ny,nz) { recalculateD(vector3df(px,py,pz)); }
			plane3df(const vector3df& point1,const vector3df& point2,const vector3df& point3) { setPlane(point1,point2,point3); }

			inline bool operator==(const plane3df& other) const { return (d==other.d && normal==other.normal); }
			inline bool operator!=(const plane3df& other) const { return !(d==other.d && normal==other.normal); }

			void setPlane(const vector3df& point,const vector3df& nvector)
			{
				normal = nvector;
				recalculateD(point);
			}

			void setPlane(const vector3df& nvect,f32 dist)
			{
				normal = nvect;
				d = dist;
			}

			void setPlane(const vector3df& point1,const vector3df& point2,const vector3df& point3)
			{
				normal = normalize(cross((point2 - point1), (point3 - point1)));
				recalculateD(point1);
			}

			bool getIntersectionWithLine(const vector3df& linePnt,const vector3df& lineVec,vector3df& outIntersect) const
			{
				f32 t,t2 = dot(normal, lineVec);

				if(t2==0.0f) return false;

				t = -(dot(normal, linePnt) + d)/t2;
				outIntersect = linePnt + (lineVec*t);
				return true;
			}

			bool getIntersectionWithPlane(const plane3df& other,vector3df& outLinePnt,vector3df& outLintVec) const
			{
				const f32 fn00 = length(normal);
				const f32 fn01 = dot(normal, other.normal);
				const f32 fn11 = length(other.normal);
				const f32 det = fn00*fn11 - fn01*fn01;

				if(fabsf((f32)det)<ROUNDING_ERROR_f32) return false;

				const f32 invdet = reciprocalf32(det);
				const f32 fc0 = (fn11*-d + fn01*other.d) * invdet;
				const f32 fc1 = (fn00*-other.d + fn01*d) * invdet;

				outLintVec = cross(normal, other.normal);
				outLinePnt = normal*fc0 + other.normal*fc1;
				return true;
			}

			bool getIntersectionWithPlanes(const plane3df& o1,const plane3df& o2,vector3df& outPnt) const
			{
				vector3df linePnt,lineVec;
				
				if(getIntersectionWithPlane(o1,linePnt,lineVec))
					return o2.getIntersectionWithLine(linePnt,lineVec,outPnt);

				return false;
			}

			void recalculateD(const vector3df& mpoint)
			{
				d = -dot(mpoint, normal);
			}

			vector3df getMemberPoint() const
			{
				return (normal * -d);
			}

			EIntersectionRelation3D classifyPointRelation(const vector3df& point) const
			{
				const f32 D = dot(normal, point) + d;

				if(D<-ROUNDING_ERROR_f32)
					return ISREL3D_BACK;
				if(D>ROUNDING_ERROR_f32)
					return ISREL3D_FRONT;

				return ISREL3D_PLANAR;
			}

			f32 d;
			vector3df normal;
		};
	}
}

#endif
