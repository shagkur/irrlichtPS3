/*
 * triangle3d.h
 *
 *  Created on: Jul 17, 2013
 *      Author: mike
 */

#ifndef TRIANGLE3D_H_
#define TRIANGLE3D_H_

#include "vector3d.h"
#include "line3d.h"
#include "plane3d.h"
#include "aabbox3d.h"

namespace irr
{
	namespace core
	{
		class triangle3df
		{
		public:
			triangle3df() {}
			triangle3df(const vector3df& v1, const vector3df& v2, const vector3df& v3) : pointA(v1), pointB(v2), pointC(v3) {}

			bool operator == (const triangle3df& other) const
			{
				return (other.pointA == pointA && other.pointB == pointB && other.pointC == pointC);
			}

			bool operator != (const triangle3df& other) const
			{
				return !(*this == other);
			}

			bool isTotalInsideBox(const aabbox3df& box) const
			{
				return (box.isPointInside(pointA) && box.isPointInside(pointB) && box.isPointInside(pointC));
			}

			vector3df getNormal() const
			{
				return cross((pointB - pointA), (pointC - pointA));
			}

			bool isFrontFacing(const vector3df& lookDir) const
			{
				const vector3df n = normalize(getNormal());
				const f32 d = dot(n, lookDir);
				return (d <= 0.0f);
			}

			f32 getArea() const
			{
				return length(cross((pointB - pointA), (pointC - pointA)))*0.5f;
			}

			vector3df pointA;
			vector3df pointB;
			vector3df pointC;
		};
	}
}

#endif /* TRIANGLE3D_H_ */
