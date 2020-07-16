/*
 * s3dvertex.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef S3DVERTEX_H_
#define S3DVERTEX_H_

#include "vector3d.h"
#include "vector2d.h"
#include "scolor.h"

namespace irr
{
	namespace video
	{
		enum E_VERTEX_TYPE
		{
			//! Base vertex type with only position and normal
			/** Usually used stencil shadow volumes and other special geometries. */
			EVT_BASE = 0,

			//! Standard vertex type used by the Irrlicht engine, video::S3DVertex.
			EVT_STANDARD,

			//! Vertex with two texture coordinates, video::S3DVertex2TCoords.
			/** Usually used for geometry with lightmaps or other special materials. */
			EVT_2TCOORDS,

			//! Vertex with a tangent and binormal vector, video::S3DVertexTangents.
			/** Usually used for tangent space normal mapping. */
			EVT_TANGENTS
		};

		struct S3DVertexBase
		{
			S3DVertexBase() {}
			S3DVertexBase(f32 x, f32 y, f32 z, f32 nx, f32 ny, f32 nz)
			: pos(x, y, z), nrm(nx, ny, nz) {}
			S3DVertexBase(const core::vector3df& p, const core::vector3df& n)
			: pos(p), nrm(n) {}

			bool operator == (const S3DVertexBase& other) const
			{
				return (pos == other.pos && nrm == other.nrm);
			}

			bool operator != (const S3DVertexBase& other) const
			{
				return (pos != other.pos || nrm != other.nrm);
			}

			bool operator < (const S3DVertexBase& other) const
			{
				return ((pos < other.pos) || (pos == other.pos && nrm < other.nrm));
			}

			E_VERTEX_TYPE getType() const
			{
				return EVT_BASE;
			}

			core::vector3df pos;
			core::vector3df nrm;
		};

		struct S3DVertexStandard : public S3DVertexBase
		{
			S3DVertexStandard() {}
			S3DVertexStandard(f32 x, f32 y, f32 z, f32 nx, f32 ny, f32 nz, video::SColor c, f32 tu, f32 tv)
			: S3DVertexBase(x, y, z, nx, ny, nz), col(c), tcoords(tu, tv) {}
			S3DVertexStandard(const core::vector3df& p, const core::vector3df& n, video::SColor c, const core::vector2df& t)
			: S3DVertexBase(p, n), col(c), tcoords(t) {}

			bool operator == (const S3DVertexStandard& other) const
			{
				return (static_cast<S3DVertexBase>(*this) == other && col == other.col && tcoords == other.tcoords);
			}

			bool operator != (const S3DVertexStandard& other) const
			{
				return (static_cast<S3DVertexBase>(*this) != other || col != other.col || tcoords != other.tcoords);
			}

			bool operator < (const S3DVertexStandard& other) const
			{
				return ((static_cast<S3DVertexBase>(*this) < other) ||
						(static_cast<S3DVertexBase>(*this) == other && col < other.col) ||
						(static_cast<S3DVertexBase>(*this) == other && col == other.col && tcoords < other.tcoords));
			}

			E_VERTEX_TYPE getType() const
			{
				return EVT_STANDARD;
			}

			video::SColor col;

			core::vector2df tcoords;
		};

		struct S3DVertex2TCoords : public S3DVertexStandard
		{
			S3DVertex2TCoords() : S3DVertexStandard() {}
			S3DVertex2TCoords(f32 x, f32 y, f32 z, SColor c, f32 tu, f32 tv, f32 tu2, f32 tv2)
			: S3DVertexStandard(x, y, z, 0.0f, 0.0f, 0.0f, c, tu, tv), tcoords2(tu2, tv2) {}

			S3DVertex2TCoords(const core::vector3df& pos, SColor c, const core::vector2df& tc, const core::vector2df& tc2)
			: S3DVertexStandard(pos, core::vector3df(), c, tc), tcoords2(tc2) {}

			S3DVertex2TCoords(const core::vector3df& pos, const core::vector3df& nrm, SColor c, const core::vector2df& tc, const core::vector2df& tc2)
			: S3DVertexStandard(pos, nrm, c, tc), tcoords2(tc2) {}

			S3DVertex2TCoords(f32 x, f32 y, f32 z, f32 nx, f32 ny, f32 nz, SColor c, f32 tu, f32 tv, f32 tu2, f32 tv2)
			: S3DVertexStandard(x, y, z, nx, ny, nz, c, tu, tv), tcoords2(tu2, tv2) {}

			S3DVertex2TCoords(const core::vector3df& pos, const core::vector3df& nrm, SColor c, const core::vector2df& tc)
			: S3DVertexStandard(pos, nrm, c, tc), tcoords2(tc) {}

			S3DVertex2TCoords(f32 x, f32 y, f32 z, f32 nx, f32 ny, f32 nz, SColor c, f32 tu, f32 tv)
			: S3DVertexStandard(x, y, z, nx, ny, nz, c, tu, tv), tcoords2(tu, tv) {}

			S3DVertex2TCoords(S3DVertexStandard& o) : S3DVertexStandard(o) {}

			bool operator == (const S3DVertex2TCoords& other) const
			{
				return (static_cast<S3DVertexStandard>(*this) == other && tcoords2 == other.tcoords2);
			}

			bool operator != (const S3DVertex2TCoords& other) const
			{
				return (static_cast<S3DVertexStandard>(*this) != other || tcoords2 != other.tcoords2);
			}

			bool operator < (const S3DVertex2TCoords& other) const
			{
				return ((static_cast<S3DVertexStandard>(*this) < other) ||
						(static_cast<S3DVertexStandard>(*this) == other && tcoords2 < other.tcoords2));
			}

			E_VERTEX_TYPE getType() const
			{
				return EVT_2TCOORDS;
			}

			core::vector2df tcoords2;
		};

		struct S3DVertexTangents : public S3DVertexStandard
		{
			S3DVertexTangents() : S3DVertexStandard() {}
			S3DVertexTangents(f32 x, f32 y, f32 z, f32 nx = 0.0f, f32 ny = 0.0f, f32 nz = 0.0f, SColor c = 0xffffffff,
							  f32 tu = 0.0f, f32 tv = 0.0f,
							  f32 tanx = 0.0f, f32 tany = 0.0f, f32 tanz = 0.0f,
							  f32 bx = 0.0f, f32 by = 0.0f, f32 bz = 0.0f)
			: S3DVertexStandard(x, y, z, nx, ny, nz, c, tu, tv), tangent(tanx, tany, tanz), binormal(bx, by, bz) {}

			S3DVertexTangents(const core::vector3df& pos, SColor c, const core::vector2df& tcoords)
			: S3DVertexStandard(pos, core::vector3df(), c, tcoords) {}

			S3DVertexTangents(const core::vector3df& pos, const core::vector3df& normal, SColor c, const core::vector2df& tcoords,
							  const core::vector3df& tan = core::vector3df(), const core::vector3df& binrm = core::vector3df())
			: S3DVertexStandard(pos, normal, c, tcoords), tangent(tan), binormal(binrm) {}

			bool operator == (const S3DVertexTangents& other) const
			{
				return (static_cast<S3DVertexStandard>(*this) == other &&
						tangent == other.tangent &&
						binormal == other.binormal);
			}

			bool operator != (const S3DVertexTangents& other) const
			{
				return (static_cast<S3DVertexStandard>(*this) != other ||
						tangent != other.tangent ||
						binormal != other.binormal);
			}

			bool operator < (const S3DVertexTangents& other) const
			{
				return ((static_cast<S3DVertexStandard>(*this) < other) ||
						(static_cast<S3DVertexStandard>(*this) == other && tangent < other.tangent) ||
						(static_cast<S3DVertexStandard>(*this) == other && tangent == other.tangent && binormal < other.binormal));
			}

			E_VERTEX_TYPE getType() const
			{
				return EVT_TANGENTS;
			}

			core::vector3df tangent;
			core::vector3df binormal;
		};

		inline u32 getVertexPitchFromType(E_VERTEX_TYPE vertexType)
		{
			switch(vertexType) {
				case EVT_STANDARD:
					return sizeof(video::S3DVertexStandard);
				case EVT_2TCOORDS:
					return sizeof(video::S3DVertex2TCoords);
				case EVT_TANGENTS:
					return sizeof(video::S3DVertexTangents);
				default:
					return sizeof(video::S3DVertexBase);
			}
		}
	}
}


#endif /* S3DVERTEX_H_ */
