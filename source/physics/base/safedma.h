/*
 * safedma.h
 *
 *  Created on: Jun 1, 2013
 *      Author: mike
 */

#ifndef SAFEDMA_H_
#define SAFEDMA_H_

#include "common.h"

static inline u32 dmaGetBuffer(void* ls, u32 start, u32 end, s32 sz, u32 tag)
{
	s32 sz_tmp = (start + sz <= end) ? sz : end - start;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_get(ls, start, sz_tmp, tag, 0, 0);
	return (u32)sz_tmp;
}

static inline u32 dmaGetBufferf(void* ls, u32 start, u32 end, s32 sz, u32 tag)
{
	s32 sz_tmp = (start + sz <= end) ? sz : end - start;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_getf(ls, start, sz_tmp, tag, 0, 0);
	return (u32)sz_tmp;
}


static inline u32 dmaGetBufferb(void* ls, u32 start, u32 end, s32 sz, u32 tag)
{
	s32 sz_tmp = (start + sz <= end) ? sz : end - start;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_getb(ls, start, sz_tmp, tag, 0, 0);
	return (u32)sz_tmp;
}

static inline u32 dmaPutBuffer(void* ls, u32 start, u32 end, s32 sz, u32 tag)
{
	s32 sz_tmp = (start + sz <= end) ? sz : end - start;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_put(ls, start, sz_tmp, tag, 0, 0);
	return (u32)sz_tmp;
}

static inline u32 dmaPutBufferf(void* ls, u32 start, u32 end, s32 sz, u32 tag)
{
	s32 sz_tmp = (start + sz <= end) ? sz : end - start;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_putf(ls, start, sz_tmp, tag, 0, 0);
	return (u32)sz_tmp;
}

static inline u32 dmaPutBufferb(void* ls, u32 start, u32 end, s32 sz, u32 tag)
{
	s32 sz_tmp = (start + sz <= end) ? sz : end - start;
	if(sz_tmp < 0) sz_tmp = 0;
	spu_dma_putb(ls, start, sz_tmp, tag, 0, 0);
	return (u32)sz_tmp;
}

#endif /* SAFEDMA_H_ */
