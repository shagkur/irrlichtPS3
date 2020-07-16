/*
 * gjksupportfunc.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#include "base/simdfunc.h"

#include "rigidbody/common/gjksolver.h"
#include "rigidbody/common/gjksupportfunc.h"
#include "rigidbody/common/box.h"
#include "rigidbody/common/capsule.h"
#include "rigidbody/common/sphere.h"
#include "rigidbody/common/trimesh.h"

///////////////////////////////////////////////////////////////////////////////
// Support Function

void getSupportVertexTriangle(void *shape, Vector3 seperatingAxis, Vector3& supportVertex)
{
	Vector3 *vtx = (Vector3*)shape;

#ifdef __SPU__
	vec_float4 p0 = vec_dot3(vtx[0].get128(), seperatingAxis.get128());
	vec_float4 p1 = vec_dot3(vtx[1].get128(), seperatingAxis.get128());
	vec_float4 p2 = vec_dot3(vtx[2].get128(), seperatingAxis.get128());
	const vec_int4 i0 = ((vec_int4){0, 0, 0, 0});
	const vec_int4 i1 = ((vec_int4){1, 0, 0, 0});
	const vec_int4 i2 = ((vec_int4){2, 0, 0, 0});
	vec_uint4  retGt01 = spu_cmpgt(p0, p1);
	vec_float4 pmax01 = spu_sel(p1, p0, retGt01);
	vec_int4 imax01 = spu_sel(i1, i0, retGt01);
	vec_uint4 retGt012 = spu_cmpgt(pmax01, p2);
	vec_int4 imax012 = spu_sel(i2, imax01, retGt012);
	s32 reti = spu_extract(imax012, 0);
#else
	f32 d0 = dot(vtx[0], seperatingAxis);
	f32 d1 = dot(vtx[1], seperatingAxis);
	f32 d2 = dot(vtx[2], seperatingAxis);

	s32 reti = 2;

	if(d0 > d1 && d0 > d2)
		reti = 0;
	else if(d1 > d2)
		reti = 1;
#endif

	supportVertex = vtx[reti] + GJK_MARGIN*normalize(seperatingAxis);
}

void getSupportVertexTriangleWithThickness(void *shape, Vector3 seperatingAxis, Vector3& supportVertex)
{
	Vector3 *vtx = (Vector3*)shape;

#ifdef __SPU__
	vec_float4 axis = seperatingAxis.get128();
	vec_float4 p0 = vec_dot3(vtx[0].get128(), axis);
	vec_float4 p1 = vec_dot3(vtx[1].get128(), axis);
	vec_float4 p2 = vec_dot3(vtx[2].get128(), axis);
	vec_float4 p3 = vec_dot3(vtx[3].get128(), axis);
	vec_float4 p4 = vec_dot3(vtx[4].get128(), axis);
	vec_float4 p5 = vec_dot3(vtx[5].get128(), axis);
	const vec_int4 i0 = ((vec_int4){0, 0, 0, 0});
	const vec_int4 i1 = ((vec_int4){1, 0, 0, 0});
	const vec_int4 i2 = ((vec_int4){2, 0, 0, 0});
	const vec_int4 i3 = ((vec_int4){3, 0, 0, 0});
	const vec_int4 i4 = ((vec_int4){4, 0, 0, 0});
	const vec_int4 i5 = ((vec_int4){5, 0, 0, 0});
	vec_uint4  retGt;
	vec_float4 pmax;
	vec_int4   imax;
	retGt = spu_cmpgt(p0, p1);
	pmax = spu_sel(p1, p0, retGt);
	imax = spu_sel(i1, i0, retGt);
	retGt = spu_cmpgt(pmax, p2);
	pmax = spu_sel(p2, pmax, retGt);
	imax = spu_sel(i2, imax, retGt);
	retGt = spu_cmpgt(pmax, p3);
	pmax = spu_sel(p3, pmax, retGt);
	imax = spu_sel(i3, imax, retGt);
	retGt = spu_cmpgt(pmax, p4);
	pmax = spu_sel(p4, pmax, retGt);
	imax = spu_sel(i4, imax, retGt);
	retGt = spu_cmpgt(pmax, p5);
	imax = spu_sel(i5, imax, retGt);
	s32 reti = spu_extract(imax, 0);
#else
	f32 d[6];
	d[0] = dot(vtx[0], seperatingAxis);
	d[1] = dot(vtx[1], seperatingAxis);
	d[2] = dot(vtx[2], seperatingAxis);
	d[3] = dot(vtx[3], seperatingAxis);
	d[4] = dot(vtx[4], seperatingAxis);
	d[5] = dot(vtx[5], seperatingAxis);

	s32 reti = 0;
	for(s32 i=1;i < 6;i++) {
		if(d[reti] < d[i])
			reti = i;
	}
#endif

	supportVertex = vtx[reti] + GJK_MARGIN*normalize(seperatingAxis);
}

void getSupportVertexConvex(void *shape, Vector3 seperatingAxis, Vector3& supportVertex)
{
	ConvexMesh *mesh = (ConvexMesh*)shape;
	s32 reti = 0;
#ifdef __SPU__
	vec_float4 v_sepAxis = seperatingAxis.get128();
	vec_float4 v_distMax = ((vec_float4){-FLT_MAX, 0, 0, 0});
	vec_int4 v_idxMax = ((vec_int4){-999, 0, 0, 0});
	ATTRIBUTE_ALIGNED16(s32 v) = 0;
	ATTRIBUTE_ALIGNED16(vec_float4 *pVtx) = (vec_float4*)mesh->verts;

	const vec_uchar16 vmask1 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80});
	const vec_uchar16 vmask2 = ((vec_uchar16){0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13});

	for(;v < (s32)mesh->numVerts - 7;v+=7) {
		vec_float4 vp1, vp2;
		{
			vec_float4 p0 = vec_dot3((vec_float4)si_lqd(si_from_ptr(pVtx++), 0), v_sepAxis);
			vec_float4 p1 = vec_dot3((vec_float4)si_lqd(si_from_ptr(pVtx++), 0), v_sepAxis);
			vec_float4 p2 = vec_dot3((vec_float4)si_lqd(si_from_ptr(pVtx++), 0), v_sepAxis);
			vec_float4 p3 = vec_dot3((vec_float4)si_lqd(si_from_ptr(pVtx++), 0), v_sepAxis);
			vec_float4 p4 = vec_dot3((vec_float4)si_lqd(si_from_ptr(pVtx++), 0), v_sepAxis);
			vec_float4 p5 = vec_dot3((vec_float4)si_lqd(si_from_ptr(pVtx++), 0), v_sepAxis);
			vec_float4 p6 = vec_dot3((vec_float4)si_lqd(si_from_ptr(pVtx++), 0), v_sepAxis);

			vp1 = spu_or(spu_shuffle(p0, p1, vmask1), spu_shuffle(p2, p3,vmask2));
			vp2 = spu_or(spu_shuffle(p4, p5, vmask1), spu_shuffle(p6, v_distMax, vmask2));
		}

		vec_int4 vi1 = ((vec_int4){v + 0, v + 1, v + 2, v + 3});
		vec_int4 vi2 = spu_insert(spu_extract(v_idxMax, 0), spu_add(vi1, 4), 3);

		// Stage1
		{
			vec_uint4 retGt = spu_cmpgt(vp1, vp2);
			vec_float4 ptmp = spu_sel(vp2, vp1, retGt);
			vp1 = ptmp;
			vp2 = spu_slqwbyte(ptmp, 8);
			vec_int4   itmp = spu_sel(vi2, vi1, retGt);
			vi1 = itmp;
			vi2 = spu_slqwbyte(itmp, 8);
		}

		// Stage2
		{
			vec_uint4 retGt = spu_cmpgt(vp1, vp2);
			vec_float4 ptmp = spu_sel(vp2, vp1, retGt);
			vp1 = ptmp;
			vp2 = spu_slqwbyte(ptmp, 4);
			vec_int4   itmp = spu_sel(vi2, vi1, retGt);
			vi1 = itmp;
			vi2 = spu_slqwbyte(itmp, 4);
		}

		// Stage3
		{
			vec_uint4 retGt = spu_cmpgt(vp1, vp2);
			v_distMax = spu_sel(vp2, vp1, retGt);
			v_idxMax  = spu_sel(vi2, vi1, retGt);
		}
	}
	for(;v < (s32)mesh->numVerts;v++) {
		const vec_int4 i = (vec_int4)si_from_int(v);
		vec_float4 p = vec_dot3((vec_float4)si_lqd(si_from_ptr(pVtx++), 0), v_sepAxis);
		vec_uint4  retGtMax = spu_cmpgt(v_distMax, p);
		v_distMax = spu_sel(p, v_distMax, retGtMax);
		v_idxMax = spu_sel(i, v_idxMax, retGtMax);
	}
	reti = spu_extract(v_idxMax, 0);
#else
	f32 dmax = dot(mesh->verts[0], seperatingAxis);
	for(s32 i=1;i < mesh->numVerts;i++) {
		f32 d = dot(mesh->verts[i], seperatingAxis);
		if(d > dmax) {
			dmax =d;
			reti = i;
		}
	}
#endif
	supportVertex = mesh->verts[reti] + GJK_MARGIN*normalize(seperatingAxis);
}

void getSupportVertexBox(void *shape, Vector3 seperatingAxis, Vector3& supportVertex)
{
	Box *box = (Box*)shape;
	Vector3 boxHalf = box->half + Vector3(GJK_MARGIN);
	supportVertex[0] = seperatingAxis[0] > 0.0f ? boxHalf[0] : -boxHalf[0];
	supportVertex[1] = seperatingAxis[1] > 0.0f ? boxHalf[1] : -boxHalf[1];
	supportVertex[2] = seperatingAxis[2] > 0.0f ? boxHalf[2] : -boxHalf[2];
}

void getSupportVertexCapsule(void *shape, Vector3 seperatingAxis, Vector3& supportVertex)
{
	Capsule *capsule = (Capsule*)shape;
	Vector3 u(1.0f, 0.0f, 0.0f);

	float udotv = dot(seperatingAxis, u);
	Vector3 dir = u*(udotv > 0.0f ? capsule->hLength : -capsule->hLength);
	supportVertex = dir + normalize(seperatingAxis)*(capsule->radius + GJK_MARGIN);
}

void getSupportVertexSphere(void *shape, Vector3 seperatingAxis, Vector3& supportVertex)
{
	Sphere *sphere = (Sphere*)shape;
	supportVertex = normalize(seperatingAxis)*(sphere->radius + GJK_MARGIN);
}

/*
void getSupportVertexCylinder(void *shape, Vector3 seperatingAxis, Vector3& supportVertex)
{
	Cylinder *cylinder = (Cylinder*)shape;
	Vector3 u(1.0f, 0.0f, 0.0f);

	float udotv = dot(seperatingAxis, u);
	Vector3 dir = u*(udotv > 0.0f ? capsule->hLength : -capsule->hLength);
	supportVertex = dir + normalize(seperatingAxis)*GJK_MARGIN;
}
*/

