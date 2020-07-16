#ifndef __AABBOX3D_H__
#define __AABBOX3D_H__

#include "irrmath.h"
#include "plane3d.h"
#include "line3d.h"

#define _VECTORMATH_PERM_XBZW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_B, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_W })
#define _VECTORMATH_PERM_XYCW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_C, _VECTORMATH_PERM_W })
#define _VECTORMATH_PERM_XBCW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_X, _VECTORMATH_PERM_B, _VECTORMATH_PERM_C, _VECTORMATH_PERM_W })
#define _VECTORMATH_PERM_AYZW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_A, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_W })
#define _VECTORMATH_PERM_ABZW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_A, _VECTORMATH_PERM_B, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_W })
#define _VECTORMATH_PERM_AYCW ((vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_A, _VECTORMATH_PERM_Y, _VECTORMATH_PERM_C, _VECTORMATH_PERM_W })

namespace irr
{
	namespace core
	{
		class aabbox3df
		{
		public:
			aabbox3df() : minEdge(-1.0f, -1.0f, -1.0f),maxEdge(1.0f, 1.0f, 1.0f) {};
			aabbox3df(const vector3df& min,const vector3df& max) : minEdge(min), maxEdge(max) {};
			aabbox3df(const vector3df& init) : minEdge(init), maxEdge(init) {};
			aabbox3df(f32 minx,f32 miny,f32 minz,f32 maxx,f32 maxy,f32 maxz) : minEdge(minx, miny, minz), maxEdge(maxx, maxy, maxz) {};

			inline bool operator == (const aabbox3df& other) const { return (minEdge == other.minEdge && maxEdge == other.maxEdge); }
			inline bool operator != (const aabbox3df& other) const { return !(minEdge == other.minEdge && maxEdge == other.maxEdge); }

			void reset(f32 x, f32 y, f32 z)
			{
				maxEdge = vector3df(x, y, z);
				minEdge = maxEdge;
			}

			void reset(const vector3df& initVec)
			{
				maxEdge = initVec;
				minEdge = initVec;
			}

			void reset(const aabbox3df& other)
			{
				*this = other;
			}

			void addInternalBox(const aabbox3df& b)
			{
				addInternalPoint(b.maxEdge);
				addInternalPoint(b.minEdge);
			}

			void addInternalPoint(const vector3df& p)
			{
				maxEdge = vector3df(vec_sel(maxEdge.get128(), p.get128(), vec_cmpgt(p.get128(), maxEdge.get128())));
				minEdge = vector3df(vec_sel(minEdge.get128(), p.get128(), vec_cmplt(p.get128(), minEdge.get128())));
			}

			void addInternalPoint(f32 x, f32 y, f32 z)
			{
				addInternalPoint(vector3df(x, y, z));
			}

			void repair()
			{
				vec_uint4 sel = (vec_uint4)vec_cmpgt(minEdge.get128(), maxEdge.get128());
				vec_float4 min = vec_sel(minEdge.get128(), maxEdge.get128(), sel);
				vec_float4 max = vec_sel(maxEdge.get128(), minEdge.get128(), sel);
				minEdge = vector3df(min);
				maxEdge = vector3df(max);
			}

			bool intersectsWithBox(const aabbox3df& other) const
			{
				return (minEdge <= other.maxEdge && maxEdge >= other.minEdge);
			}

			bool isEmpty() const
			{
				return (minEdge == maxEdge);
			}

			vector3df getCenter() const
			{
				return (minEdge - maxEdge)/2;
			}
			
			vector3df getExtent() const
			{
				return (maxEdge - minEdge);
			}

			void getEdges(vector3df *edges) const
			{
				const vector3df middle = getCenter();
				const vector3df diag = middle - maxEdge;
				vec_float4 tl = (middle + diag).get128();
				vec_float4 br = (middle - diag).get128();

				edges[0] = vector3df(tl);
				edges[1] = vector3df(vec_perm(tl, br, _VECTORMATH_PERM_XBZW));
				edges[2] = vector3df(vec_perm(tl, br, _VECTORMATH_PERM_XYCW));
				edges[3] = vector3df(vec_perm(tl, br, _VECTORMATH_PERM_XBCW));
				edges[4] = vector3df(vec_perm(tl, br, _VECTORMATH_PERM_AYZW));
				edges[5] = vector3df(vec_perm(tl, br, _VECTORMATH_PERM_ABZW));
				edges[6] = vector3df(vec_perm(tl, br, _VECTORMATH_PERM_AYCW));
				edges[7] = vector3df(br);
			}

			bool isPointInside(const vector3df& p) const
			{
				return (p >= minEdge && p <= maxEdge);
			}

			bool isPointTotalInside(const vector3df& p) const
			{
				return (p > minEdge && p < maxEdge);
			}

			bool isValid() const
			{
				return !(minEdge > maxEdge);
			}
			
 			vector3df minEdge;
			vector3df maxEdge;
		};
	}
}

#endif
