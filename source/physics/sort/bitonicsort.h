/*
 * bitonicsort.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef BITONICSORT_H_
#define BITONICSORT_H_

#include "base/common.h"

#ifdef __SPU__

SIMD_FORCE_INLINE void bitonicmerge8_ascend(vec_uint4 *keys, vec_uint4 *ids)
{
	vec_uint4 key1 = keys[0];
	vec_uint4 key2 = keys[1];
	vec_uint4 id1 = ids[0];
	vec_uint4 id2 = ids[1];
	vec_uint4 vtmp0, vtmp1, vtmp2, vtmp3;
	vec_uint4 vcmp0, vcmp1;

	// Phase4
	vcmp0 = spu_cmpgt(key1, key2);
	vtmp1 = key1;
	vtmp2 = id1;
	key1 = spu_sel(vtmp1, key2, vcmp0);
	key2 = spu_sel(key2, vtmp1, vcmp0);
	id1 = spu_sel(vtmp2, id2, vcmp0);
	id2 = spu_sel(id2, vtmp2, vcmp0);

	// Phase5
	const vec_uchar16 mask_ptn0101 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07});
	const vec_uchar16 mask_ptn2323 = ((vec_uchar16){0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f});
	const vec_uchar16 mask_ptn2301 = ((vec_uchar16){0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0101);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn2323);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0101);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn2323);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn2301), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn2301), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn2301), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn2301), vcmp1);

	// Phase6
	const vec_uchar16 mask_ptn0022 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x08, 0x09, 0x0a, 0x0b, 0x08, 0x09, 0x0a, 0x0b});
	const vec_uchar16 mask_ptn1133 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x04, 0x05, 0x06, 0x07, 0x0c, 0x0d, 0x0e, 0x0f, 0x0c, 0x0d, 0x0e, 0x0f});
	const vec_uchar16 mask_ptn1032 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x0c, 0x0d, 0x0e, 0x0f, 0x08, 0x09, 0x0a, 0x0b});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0022);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn1133);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0022);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn1133);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn1032), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn1032), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn1032), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn1032), vcmp1);

	keys[0] = key1;
	keys[1] = key2;
	ids[0] = id1;
	ids[1] = id2;
}

SIMD_FORCE_INLINE void bitonicmerge8_descend(vec_uint4 *keys, vec_uint4 *ids)
{
	vec_uint4 key1 = keys[0];
	vec_uint4 key2 = keys[1];
	vec_uint4 id1 = ids[0];
	vec_uint4 id2 = ids[1];
	vec_uint4 vtmp0, vtmp1, vtmp2, vtmp3;
	vec_uint4 vcmp0, vcmp1;

	// Phase4
	vcmp0 = spu_cmpgt(key2, key1);
	vtmp1 = key1;
	vtmp2 = id1;
	key1 = spu_sel(vtmp1, key2, vcmp0);
	key2 = spu_sel(key2, vtmp1, vcmp0);
	id1 = spu_sel(vtmp2, id2, vcmp0);
	id2 = spu_sel(id2, vtmp2, vcmp0);

	// Phase5
	const vec_uchar16 mask_ptn0101 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07});
	const vec_uchar16 mask_ptn2323 = ((vec_uchar16){0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f});
	const vec_uchar16 mask_ptn2301 = ((vec_uchar16){0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0101);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn2323);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0101);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn2323);
	vcmp0 = spu_cmpgt(vtmp1, vtmp0);
	vcmp1 = spu_cmpgt(vtmp3, vtmp2);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn2301), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn2301), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn2301), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn2301), vcmp1);

	// Phase6
	const vec_uchar16 mask_ptn0022 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x08, 0x09, 0x0a, 0x0b, 0x08, 0x09, 0x0a, 0x0b});
	const vec_uchar16 mask_ptn1133 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x04, 0x05, 0x06, 0x07, 0x0c, 0x0d, 0x0e, 0x0f, 0x0c, 0x0d, 0x0e, 0x0f});
	const vec_uchar16 mask_ptn1032 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x0c, 0x0d, 0x0e, 0x0f, 0x08, 0x09, 0x0a, 0x0b});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0022);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn1133);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0022);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn1133);
	vcmp0 = spu_cmpgt(vtmp1, vtmp0);
	vcmp1 = spu_cmpgt(vtmp3, vtmp2);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn1032), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn1032), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn1032), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn1032), vcmp1);

	keys[0] = key1;
	keys[1] = key2;
	ids[0] = id1;
	ids[1] = id2;
}

void bitonicmerge_ascend(vec_uint4 *keys, vec_uint4 *ids, u32 n)
{
	if(n == 8)
		bitonicmerge8_ascend(keys, ids);
	else { // 16,32,64,...
		u32 vnum = n>>3;

		for(u32 i=0;i < vnum;i++) {
			vec_uint4 key1 = keys[i];
			vec_uint4 key2 = keys[i + vnum];
			vec_uint4 id1 = ids[i];
			vec_uint4 id2 = ids[i + vnum];

			vec_uint4 vcmp = spu_cmpgt(key1, key2);

			keys[i] = spu_sel(key1, key2, vcmp);
			keys[i + vnum] = spu_sel(key2, key1, vcmp);
			ids[i] = spu_sel(id1 ,id2, vcmp);
			ids[i + vnum] = spu_sel(id2, id1, vcmp);
		}

		u32 k = n>>1;
		bitonicmerge_ascend(keys, ids, k);
		bitonicmerge_ascend(keys + (k>>2), ids + (k>>2), k);
	}
}

void bitonicmerge_descend(vec_uint4 *keys, vec_uint4 *ids, u32 n)
{
	if(n == 8)
		bitonicmerge8_descend(keys, ids);
	else { // 16,32,64,...
		u32 vnum = n>>3;
		for(u32 i=0;i < vnum;i++) {
			vec_uint4 key1 = keys[i];
			vec_uint4 key2 = keys[i + vnum];
			vec_uint4 id1 = ids[i];
			vec_uint4 id2 = ids[i + vnum];

			vec_uint4 vcmp = spu_cmpgt(key2, key1);

			keys[i] = spu_sel(key1, key2, vcmp);
			keys[i + vnum] = spu_sel(key2, key1, vcmp);
			ids[i] = spu_sel(id1, id2, vcmp);
			ids[i + vnum] = spu_sel(id2, id1, vcmp);
		}

		u32 k = n>>1;
		bitonicmerge_descend(keys, ids, k);
		bitonicmerge_descend(keys + (k>>2), ids + (k>>2), k);
	}
}

SIMD_FORCE_INLINE void bitonicsort8_ascend(vec_uint4 *keys, vec_uint4 *ids)
{
	vec_uint4 key1 = keys[0];
	vec_uint4 key2 = keys[1];
	vec_uint4 id1 = ids[0];
	vec_uint4 id2 = ids[1];
	vec_uint4 vtmp0,vtmp1,vtmp2,vtmp3;
	vec_uint4 vcmp0,vcmp1;

	// Phase1
	const vec_uchar16 mask_ptn0033 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x0c, 0x0d, 0x0e, 0x0f, 0x0c, 0x0d, 0x0e, 0x0f});
	const vec_uchar16 mask_ptn1122 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x08, 0x09, 0x0a, 0x0b});
	const vec_uchar16 mask_ptn1032 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x0c, 0x0d, 0x0e, 0x0f, 0x08, 0x09, 0x0a, 0x0b});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0033);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn1122);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0033);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn1122);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn1032), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn1032), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn1032), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn1032), vcmp1);

	// Phase2
	const vec_uchar16 mask_ptn0101 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07});
	const vec_uchar16 mask_ptn2323 = ((vec_uchar16){0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f});
	const vec_uchar16 mask_ptn2301 = ((vec_uchar16){0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0101);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn2323);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn2323);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn0101);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn2301), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn2301), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn2301), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn2301), vcmp1);

	// Phase3
	const vec_uchar16 mask_ptn0022 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x08, 0x09, 0x0a, 0x0b, 0x08, 0x09, 0x0a, 0x0b});
	const vec_uchar16 mask_ptn1133 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x04, 0x05, 0x06, 0x07, 0x0c, 0x0d, 0x0e, 0x0f, 0x0c, 0x0d, 0x0e, 0x0f});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0022);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn1133);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn1133);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn0022);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn1032), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn1032), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn1032), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn1032), vcmp1);

	// Phase4
	vcmp0 = spu_cmpgt(key1, key2);
	vtmp1 = key1;
	vtmp2 = id1;
	key1 = spu_sel(vtmp1, key2, vcmp0);
	key2 = spu_sel(key2, vtmp1, vcmp0);
	id1 = spu_sel(vtmp2, id2, vcmp0);
	id2 = spu_sel(id2, vtmp2, vcmp0);

	// Phase5
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0101);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn2323);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0101);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn2323);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn2301), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn2301), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn2301), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn2301), vcmp1);

	// Phase6
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0022);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn1133);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0022);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn1133);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn1032), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn1032), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn1032), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn1032), vcmp1);

	keys[0] = key1;
	keys[1] = key2;
	ids[0] = id1;
	ids[1] = id2;
}

SIMD_FORCE_INLINE void bitonicsort8_descend(vec_uint4 *keys, vec_uint4 *ids)
{
	vec_uint4 key1 = keys[0];
	vec_uint4 key2 = keys[1];
	vec_uint4 id1 = ids[0];
	vec_uint4 id2 = ids[1];
	vec_uint4 vtmp0,vtmp1,vtmp2,vtmp3;
	vec_uint4 vcmp0,vcmp1;

	// Phase1
	const vec_uchar16 mask_ptn0033 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x00, 0x01, 0x02, 0x03, 0x0c, 0x0d, 0x0e,0x0f, 0x0c, 0x0d, 0x0e, 0x0f});
	const vec_uchar16 mask_ptn1122 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a,0x0b, 0x08, 0x09, 0x0a, 0x0b});
	const vec_uchar16 mask_ptn1032 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x0c, 0x0d, 0x0e,0x0f, 0x08, 0x09, 0x0a, 0x0b});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0033);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn1122);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0033);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn1122);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn1032), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn1032), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn1032), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn1032), vcmp1);

	// Phase2
	const vec_uchar16 mask_ptn0101 = ((vec_uchar16){0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07, 0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07});
	const vec_uchar16 mask_ptn2323 = ((vec_uchar16){0x08,0x09,0x0a,0x0b, 0x0c,0x0d,0x0e,0x0f, 0x08,0x09,0x0a,0x0b, 0x0c,0x0d,0x0e,0x0f});
	const vec_uchar16 mask_ptn2301 = ((vec_uchar16){0x08,0x09,0x0a,0x0b, 0x0c,0x0d,0x0e,0x0f, 0x00,0x01,0x02,0x03, 0x04,0x05,0x06,0x07});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0101);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn2323);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn2323);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn0101);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn2301), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn2301), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn2301), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn2301), vcmp1);

	// Phase3
	const vec_uchar16 mask_ptn0022 = ((vec_uchar16){0x00,0x01,0x02,0x03, 0x00,0x01,0x02,0x03, 0x08,0x09,0x0a,0x0b, 0x08,0x09,0x0a,0x0b});
	const vec_uchar16 mask_ptn1133 = ((vec_uchar16){0x04,0x05,0x06,0x07, 0x04,0x05,0x06,0x07, 0x0c,0x0d,0x0e,0x0f, 0x0c,0x0d,0x0e,0x0f});
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0022);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn1133);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn1133);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn0022);
	vcmp0 = spu_cmpgt(vtmp0, vtmp1);
	vcmp1 = spu_cmpgt(vtmp2, vtmp3);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn1032), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn1032), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn1032), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn1032), vcmp1);

	// Phase4
	vcmp0 = spu_cmpgt(key2, key1);
	vtmp1 = key1;
	vtmp2 = id1;
	key1 = spu_sel(vtmp1, key2, vcmp0);
	key2 = spu_sel(key2, vtmp1, vcmp0);
	id1 = spu_sel(vtmp2, id2, vcmp0);
	id2 = spu_sel(id2, vtmp2, vcmp0);

	// Phase5
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0101);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn2323);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0101);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn2323);
	vcmp0 = spu_cmpgt(vtmp1, vtmp0);
	vcmp1 = spu_cmpgt(vtmp3, vtmp2);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn2301), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn2301), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn2301), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn2301), vcmp1);

	// Phase6
	vtmp0 = spu_shuffle(key1, key1, mask_ptn0022);
	vtmp1 = spu_shuffle(key1, key1, mask_ptn1133);
	vtmp2 = spu_shuffle(key2, key2, mask_ptn0022);
	vtmp3 = spu_shuffle(key2, key2, mask_ptn1133);
	vcmp0 = spu_cmpgt(vtmp1, vtmp0);
	vcmp1 = spu_cmpgt(vtmp3, vtmp2);
	key1 = spu_sel(key1, spu_shuffle(key1, key1, mask_ptn1032), vcmp0);
	key2 = spu_sel(key2, spu_shuffle(key2, key2, mask_ptn1032), vcmp1);
	id1 = spu_sel(id1, spu_shuffle(id1, id1, mask_ptn1032), vcmp0);
	id2 = spu_sel(id2, spu_shuffle(id2, id2, mask_ptn1032), vcmp1);

	keys[0] = key1;
	keys[1] = key2;
	ids[0] = id1;
	ids[1] = id2;
}

void bitonicsort_internal_ascend(vec_uint4 *keys, vec_uint4 *ids, u32 n);
void bitonicsort_internal_descend(vec_uint4 *keys, vec_uint4 *ids, u32 n);

void bitonicsort_internal_ascend(vec_uint4 *keys, vec_uint4 *ids, u32 n)
{
	if(n == 8)
		bitonicsort8_ascend(keys, ids);
	else {
		u32 k = n>>1;
		bitonicsort_internal_ascend(keys, ids, k);
		bitonicsort_internal_descend(keys + (k>>2), ids + (k>>2), k);
		bitonicmerge_ascend(keys, ids, n);
	}
}

void bitonicsort_internal_descend(vec_uint4 *keys, vec_uint4 *ids, u32 n)
{
	if(n == 8)
		bitonicsort8_descend(keys, ids);
	else {
		u32 k = n>>1;
		bitonicsort_internal_ascend(keys, ids, k);
		bitonicsort_internal_descend(keys + (k>>2), ids + (k>>2), k);
		bitonicmerge_descend(keys, ids, n);
	}
}

void bitonicsort(SortData *d,SortData *buff,u32 n)
{
	if(n > 1) {
		const u32 n8 = ((n + 7)/8)*8;
		const u32 vecn = (n8 + 3)/4;
		ATTRIBUTE_ALIGNED16(vec_uint4 ids[vecn]);
		ATTRIBUTE_ALIGNED16(vec_uint4 keys[vecn]);
		u32 i=0;
		for(;i < n;i++) {
			((u32*)ids)[i] = (u32)i;
			((u32*)keys)[i] = getKey(d[i]);
			//PRINTF("%u ids %u:%x keys %u:%x\n",i,i,((uint32_t*)ids+i),((uint32_t*)keys)[i],((uint32_t*)keys)+i);
		}
		for(;i < vecn*4;i++) {
			((u32*)ids)[i] = 0;
			((u32*)keys)[i] = 0xffffffff;
		}

		// n>=8
		bitonicsort_internal_ascend(keys, ids, vecn*4);

		for(i=0;i<n;i++)
			buff[i] = d[((u32*)ids)[i]];
	} else if(n == 1)
		buff[0] = d[0];
}

#else

void bitonicmerge(SortData *d, u32 n, s32 dir)
{
	if(n > 1) {
		u32 k = n>>1;
		for(u32 i=0;i < k;i++) {
			if(dir==0 && Key(d[i]) > Key(d[i + k])) {
				SortData t = d[i + k];
				d[i + k] = d[i];
				d[i] = t;
			}
			if(dir==1 && Key(d[i]) < Key(d[i + k])) {
				SortData t = d[i + k];
				d[i + k] = d[i];
				d[i] = t;
			}
		}
		bitonicmerge(d, k, dir);
		bitonicmerge(d + k, k, dir);
	}
}

void bitonicsort_internal(SortData *d, u32 n, u32 dir = 0)
{
	if(n > 1) {
		u32 k = n>>1;
		bitonicsort_internal(d, k, 0);
		bitonicsort_internal(d + k, k, 1);
		bitonicmerge(d, n, dir);
	}
}

void bitonicsort(SortData *d, SortData *buff, u32 n)
{
	bitonicsort_internal(d, n, 0);
	memcpy(buff, d, sizeof(SortData)*n);
}

#endif

#endif /* BITONICSORT_H_ */
