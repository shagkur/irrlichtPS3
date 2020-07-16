/*
 * simplexsolver.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef SIMPLEXSOLVER_H_
#define SIMPLEXSOLVER_H_

#include "base/common.h"

///////////////////////////////////////////////////////////////////////////////
// Voronoi Simplex Solver

ATTRIBUTE_ALIGNED16(struct) BarycentricCoords
{
	Vector3 closest;
#ifdef __SPU__
	vec_float4 barycentricCoords;
#else
	f32 barycentricCoords[4];
#endif
	u32 usedVertices;

	void reset()
	{
#ifdef __SPU__
		barycentricCoords = spu_splats(0.0f);
#else
		barycentricCoords[0] = 0.0f;
		barycentricCoords[1] = 0.0f;
		barycentricCoords[2] = 0.0f;
		barycentricCoords[3] = 0.0f;
#endif
		usedVertices = 0;
	}

	bool isValid()
	{
#ifdef __SPU__
		vec_uint4  v_retGt = spu_cmpgt(barycentricCoords, spu_splats(0.0f));
		vec_uint4  v_retEq = spu_cmpeq(barycentricCoords, spu_splats(0.0f));
		return spu_extract(spu_gather(spu_or(v_retGt, v_retEq)), 0) > 0;
#else
		return   (barycentricCoords[0] >= 0.0f) &&
				 (barycentricCoords[1] >= 0.0f) &&
				 (barycentricCoords[2] >= 0.0f) &&
				 (barycentricCoords[3] >= 0.0f);
#endif
	}

#ifdef __SPU__
	void setBarycentricCoordinates(vec_float4 v_abcd)
	{
		barycentricCoords = v_abcd;
		vec_uint4 v_retEq = spu_cmpeq(v_abcd, spu_splats(0.0f));
		usedVertices = spu_extract(spu_gather(spu_nand(v_retEq, v_retEq)), 0);
	}

	void setBarycentricCoordinates(f32 a, f32 b, f32 c, f32 d)
	{
		vec_float4 v_abcd = ((vec_float4){a, b, c, d});
		setBarycentricCoordinates(v_abcd);
	}
#else
	void setBarycentricCoordinates(f32 a, f32 b, f32 c, f32 d)
	{
		barycentricCoords[0] = a;
		barycentricCoords[1] = b;
		barycentricCoords[2] = c;
		barycentricCoords[3] = d;
		if(a != 0.0f) usedVertices |= 1<<3;
		if(b != 0.0f) usedVertices |= 1<<2;
		if(c != 0.0f) usedVertices |= 1<<1;
		if(d != 0.0f) usedVertices |= 1;
	}
#endif
};

class SimplexSolver
{
private:
	const static s32 MAX_VERTS = 4;

public:
	s32	numVertices;
	Vector3	W[MAX_VERTS];
	Vector3	P[MAX_VERTS];
	Vector3	Q[MAX_VERTS];

	BarycentricCoords bc;

	inline void	removeVertex(s32 index);
	inline void	reduceVertices();

	inline bool	originOutsideOfPlane(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d);
	bool closestPointTetrahedronFromOrigin(Vector3 a, Vector3 b, Vector3 c, Vector3 d, BarycentricCoords& result);
	bool closestPointTriangleFromOrigin(Vector3 a, Vector3 b, Vector3 c, BarycentricCoords& result);

public:
	void reset()
	{
		numVertices = 0;
		bc.reset();
	}

	inline void addVertex(const Vector3& w_, const Vector3& p_, const Vector3& q_);

	bool closest(Vector3& v);

	bool fullSimplex() const
	{
		return (numVertices == 4);
	}

	bool inSimplex(const Vector3& w);
};

inline void SimplexSolver::removeVertex(s32 index)
{
	ASSERT(numVertices > 0);
	numVertices--;
	W[index] = W[numVertices];
	P[index] = P[numVertices];
	Q[index] = Q[numVertices];
}

inline void SimplexSolver::reduceVertices()
{
	if ((numVertices >= 4) && (!(bc.usedVertices&0x01)))
		removeVertex(3);

	if ((numVertices >= 3) && (!(bc.usedVertices&0x02)))
		removeVertex(2);

	if ((numVertices >= 2) && (!(bc.usedVertices&0x04)))
		removeVertex(1);

	if ((numVertices >= 1) && (!(bc.usedVertices&0x08)))
		removeVertex(0);
}

inline void SimplexSolver::addVertex(const Vector3& w, const Vector3& p, const Vector3& q)
{
	W[numVertices] = w;
	P[numVertices] = p;
	Q[numVertices] = q;
	numVertices++;
}

inline bool SimplexSolver::originOutsideOfPlane(const Vector3& a, const Vector3& b, const Vector3& c, const Vector3& d)
{
	Vector3 normal = cross((b - a), (c - a));

    f32 signp = dot(-a, normal);
    f32 signd = dot((d - a), normal);

	return signp*signd < 0.0f;
}

#endif /* SIMPLEXSOLVER_H_ */
