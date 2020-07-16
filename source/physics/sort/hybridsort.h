/*
 * hybridsort.h
 *
 *  Created on: Jun 3, 2013
 *      Author: mike
 */

#ifndef HYBRIDSORT_H_
#define HYBRIDSORT_H_

#include "base/common.h"

struct Div2n
{
	u32 id;
	u32 num;

	Div2n() {}

	Div2n(u32 i, u32 n)
	{
		id = i;
		num = n;
	}
};

void merge_two_buffers(SortData* d1, s32 n1, SortData* d2, s32 n2, SortData *buff)
{
	const vec_uint4 vlimit = ((vec_uint4){(u32)n1, (u32)n2, 0xffffffff, 0xffffffff});
	vec_uint4 vi = ((vec_uint4){0, 0, 0, 0});
	vec_uint4 vj = ((vec_uint4){0, 0, 0, 0});
	vec_uint4 vflag = ((vec_uint4){0, 0, 0, 0});

	while(LIKELY(spu_extract(vflag, 0) == 0)) {
		const vec_uint4 vd1 = spu_splats(getKey(d1[spu_extract(vi, 0)]));
		const vec_uint4 vd2 = spu_splats(getKey(d2[spu_extract(vj, 0)]));
		const vec_uint4 cmp = spu_cmpgt(vd1, vd2);

		buff[spu_extract(spu_add(vi, vj), 0)] = d1[spu_extract(spu_sel(vi, spu_add(vj, vlimit), cmp), 0)];

		const vec_uint4 vadd0 = ((vec_uint4){0, 0, 0, 0});
		const vec_uint4 vadd1 = ((vec_uint4){1, 1, 1, 1});
		vi = spu_add(vi, spu_sel(vadd1, vadd0, cmp));
		vj = spu_add(vj, spu_sel(vadd0, vadd1, cmp));

		const vec_uchar16 mask_chk = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x14, 0x15, 0x16, 0x17, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80});
		const vec_uint4 mask_ffff = ((vec_uint4){0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff});
		const vec_uint4 vchk = spu_shuffle(vi, vj, mask_chk);
		vflag = spu_gather(spu_xor(spu_cmpgt(vlimit, vchk), mask_ffff)); // i<n1 && j<n2 -> i>=n1 || j>=n2
	}

	u32 i = spu_extract(vi, 0);
	u32 j = spu_extract(vj, 0);
	if((s32)i < n1) {
		for(u32 k=0;k < n1 - i;k++)
			buff[i + j + k] = d1[i + k];
	} else if((s32)j < n2) {
		for(u32 k=0;k < n2 - j;k++)
			buff[i + j + k] = d2[j + k];
	}

	{
		s32 k = 0;
		s32 sum = n1 + n2;
		for(;k < sum - 8;k+=8) {
			d1[k + 0] = buff[k + 0];
			d1[k + 1] = buff[k + 1];
			d1[k + 2] = buff[k + 2];
			d1[k + 3] = buff[k + 3];
			d1[k + 4] = buff[k + 4];
			d1[k + 5] = buff[k + 5];
			d1[k + 6] = buff[k + 6];
			d1[k + 7] = buff[k + 7];
		}
		for(;k < sum;k++) {
			d1[k] = buff[k];
		}
	}
}

void hybridsort(SortData *data, SortData *buff, u32 n)
{
	u32 numDiv = 0;
	Div2n divData[32];

	u32 id = 0;
	u32 rest = n;
	u32 mask = 0x01;
	while(rest > 0) {
		if((mask&n) > 0) {
			divData[numDiv++] = Div2n(id, mask);
			bitonicsort(data + id, buff + id, mask);
			rest ^= mask;
			id += mask;
		}
		mask <<= 1;
	}

	if(numDiv == 1) {
		for(u32 i=0;i < n;i++)
			data[i] = buff[i];
	} else {
		for(u32 i=1;i < numDiv;i++) {
			merge_two_buffers(buff + divData[i - 1].id, divData[i - 1].num, buff + divData[i].id, divData[i].num, data);
			divData[i].id = divData[i - 1].id;
			divData[i].num += divData[i - 1].num;
		}
	}
}

#endif /* HYBRIDSORT_H_ */
