/*
 * sviewfrustum.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef SVIEWFRUSTUM_H_
#define SVIEWFRUSTUM_H_

#include "vector3d.h"
#include "aabbox3d.h"
#include "matrix4.h"
#include "plane3d.h"
#include "ivideodriver.h"

namespace irr
{
	namespace scene
	{
		struct SViewFrustum
		{
			enum VFPLANES
			{
				//! Far plane of the frustum. That is the plane farest away from the eye.
				VF_FAR_PLANE = 0,
				//! Near plane of the frustum. That is the plane nearest to the eye.
				VF_NEAR_PLANE,
				//! Left plane of the frustum.
				VF_LEFT_PLANE,
				//! Right plane of the frustum.
				VF_RIGHT_PLANE,
				//! Bottom plane of the frustum.
				VF_BOTTOM_PLANE,
				//! Top plane of the frustum.
				VF_TOP_PLANE,

				//! Amount of planes enclosing the view frustum. Should be 6.
				VF_PLANE_COUNT
			};

			//! Hold a copy of important transform matrices
			enum E_TRANSFORMATION_STATE_FRUSTUM
			{
				ETS_VIEW = 0,
				ETS_PROJECTION = 1,
				ETS_ORTHOGRAPHIC = 2,
				ETS_COUNT_FRUSTUM
			};

			SViewFrustum() {};
			SViewFrustum(const SViewFrustum& other);
			SViewFrustum(const core::matrix4& mat);

			inline void setFrom(const core::matrix4& mat);

			inline void recalculateBoundingBox();
			const core::aabbox3df& getBoundingBox() const;

			void transform(const core::matrix4& mat);

			core::vector3df getFarLeftUp() const;
			core::vector3df getFarLeftDown() const;
			core::vector3df getFarRightUp() const;
			core::vector3df getFarRightDown() const;

			core::matrix4& getTransform(video::E_TRANSFORMATION_STATE state);
			const core::matrix4& getTransform(video::E_TRANSFORMATION_STATE state) const;

			core::vector3df camPos;
			core::aabbox3df boundingBox;
			core::plane3df planes[VF_PLANE_COUNT];
			core::matrix4 matrices[ETS_COUNT_FRUSTUM];
		};

		inline SViewFrustum::SViewFrustum(const SViewFrustum& other)
		{
			u32 i;

			camPos = other.camPos;
			boundingBox = other.boundingBox;

			for(i=0;i < VF_PLANE_COUNT;i++)
				planes[i] = other.planes[i];
			for(i=0;i < ETS_COUNT_FRUSTUM;i++)
				matrices[i] = other.matrices[i];
		}

		inline SViewFrustum::SViewFrustum(const core::matrix4& mat)
		{
			setFrom(mat);
		}

		inline const core::aabbox3df& SViewFrustum::getBoundingBox() const
		{
			return boundingBox;
		}

		inline void SViewFrustum::recalculateBoundingBox()
		{
			boundingBox.reset(camPos);

			boundingBox.addInternalPoint(getFarLeftUp());
			boundingBox.addInternalPoint(getFarRightUp());
			boundingBox.addInternalPoint(getFarLeftDown());
			boundingBox.addInternalPoint(getFarRightDown());
		}

		inline void SViewFrustum::setFrom(const core::matrix4& mat)
		{
			// left clipping plane
			planes[VF_LEFT_PLANE].normal.setX(mat.getCol(0).getW() + mat.getCol(0).getX());
			planes[VF_LEFT_PLANE].normal.setY(mat.getCol(1).getW() + mat.getCol(1).getX());
			planes[VF_LEFT_PLANE].normal.setZ(mat.getCol(2).getW() + mat.getCol(2).getX());
			planes[VF_LEFT_PLANE].d			= mat.getCol(3).getW() + mat.getCol(3).getX();

			// right clipping plane
			planes[VF_RIGHT_PLANE].normal.setX(mat.getCol(0).getW() - mat.getCol(0).getX());
			planes[VF_RIGHT_PLANE].normal.setY(mat.getCol(1).getW() - mat.getCol(1).getX());
			planes[VF_RIGHT_PLANE].normal.setZ(mat.getCol(2).getW() - mat.getCol(2).getX());
			planes[VF_RIGHT_PLANE].d		 = mat.getCol(3).getW() - mat.getCol(3).getX();

			// top clipping plane
			planes[VF_TOP_PLANE].normal.setX(mat.getCol(0).getW() - mat.getCol(0).getY());
			planes[VF_TOP_PLANE].normal.setY(mat.getCol(1).getW() - mat.getCol(1).getY());
			planes[VF_TOP_PLANE].normal.setZ(mat.getCol(2).getW() - mat.getCol(2).getY());
			planes[VF_TOP_PLANE].d		   = mat.getCol(3).getW() - mat.getCol(3).getY();

			// bottom clipping plane
			planes[VF_BOTTOM_PLANE].normal.setX(mat.getCol(0).getW() + mat.getCol(0).getY());
			planes[VF_BOTTOM_PLANE].normal.setY(mat.getCol(1).getW() + mat.getCol(1).getY());
			planes[VF_BOTTOM_PLANE].normal.setZ(mat.getCol(2).getW() + mat.getCol(2).getY());
			planes[VF_BOTTOM_PLANE].d		  = mat.getCol(3).getW() + mat.getCol(3).getY();

			// far clipping plane.
			planes[VF_FAR_PLANE].normal.setX(mat.getCol(0).getW() - mat.getCol(0).getZ());
			planes[VF_FAR_PLANE].normal.setY(mat.getCol(1).getW() - mat.getCol(1).getZ());
			planes[VF_FAR_PLANE].normal.setZ(mat.getCol(2).getW() - mat.getCol(2).getZ());
			planes[VF_FAR_PLANE].d		   = mat.getCol(3).getW() - mat.getCol(3).getZ();

			// near clipping plane.
			planes[VF_NEAR_PLANE].normal.setX(mat.getCol(0).getW() + mat.getCol(0).getZ());
			planes[VF_NEAR_PLANE].normal.setY(mat.getCol(1).getW() + mat.getCol(1).getZ());
			planes[VF_NEAR_PLANE].normal.setZ(mat.getCol(2).getW() + mat.getCol(2).getZ());
			planes[VF_NEAR_PLANE].d			= mat.getCol(3).getW() + mat.getCol(3).getZ();

			u32 i;
			for(i=0;i < VF_PLANE_COUNT;i++) {
				const f32 len = -core::reciprocal_sqrtf32(lengthSqr(planes[i].normal));

				planes[i].normal *= len;
				planes[i].d *= len;
			}
			recalculateBoundingBox();
		}

		inline void SViewFrustum::transform(const core::matrix4& mat)
		{
			u32 i;

			for(i=0;i < VF_PLANE_COUNT;i++)
				core::transformPlane(mat, planes[i]);

			core::transformVect(mat, camPos);
			recalculateBoundingBox();
		}

		inline core::vector3df SViewFrustum::getFarLeftUp() const
		{
			core::vector3df p;
			planes[VF_FAR_PLANE].getIntersectionWithPlanes(planes[VF_TOP_PLANE],planes[VF_LEFT_PLANE],p);
			return p;
		}

		inline core::vector3df SViewFrustum::getFarLeftDown() const
		{
			core::vector3df p;
			planes[VF_FAR_PLANE].getIntersectionWithPlanes(planes[VF_BOTTOM_PLANE],planes[VF_LEFT_PLANE],p);
			return p;
		}

		inline core::vector3df SViewFrustum::getFarRightUp() const
		{
			core::vector3df p;
			planes[VF_FAR_PLANE].getIntersectionWithPlanes(planes[VF_TOP_PLANE],planes[VF_RIGHT_PLANE],p);
			return p;
		}

		inline core::vector3df SViewFrustum::getFarRightDown() const
		{
			core::vector3df p;
			planes[VF_FAR_PLANE].getIntersectionWithPlanes(planes[VF_BOTTOM_PLANE],planes[VF_RIGHT_PLANE],p);
			return p;
		}

		inline core::matrix4& SViewFrustum::getTransform(video::E_TRANSFORMATION_STATE state)
		{
			u32 index = 0;

			switch(state) {
				case video::ETS_VIEW:			index = SViewFrustum::ETS_VIEW; break;
				case video::ETS_PROJECTION:		index = SViewFrustum::ETS_PROJECTION; break;
				case video::ETS_ORTHOGRAPHIC:	index = SViewFrustum::ETS_ORTHOGRAPHIC; break;
				default:						break;
			}
			return matrices[index];
		}

		inline const core::matrix4& SViewFrustum::getTransform(video::E_TRANSFORMATION_STATE state) const
		{
			u32 index = 0;

			switch(state) {
				case video::ETS_VIEW:			index = SViewFrustum::ETS_VIEW; break;
				case video::ETS_PROJECTION:		index = SViewFrustum::ETS_PROJECTION; break;
				case video::ETS_ORTHOGRAPHIC:	index = SViewFrustum::ETS_ORTHOGRAPHIC; break;
				default:						break;
			}
			return matrices[index];
		}
	}
}

#endif /* SVIEWFRUSTUM_H_ */
