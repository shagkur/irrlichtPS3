/*
 * heapmanager.h
 *
 *  Created on: Apr 26, 2013
 *      Author: mike
 */

#ifndef HEAPMANAGER_H_
#define HEAPMANAGER_H_

#include "common.h"

#ifdef __SPU__
	#define HEAP_STACK_SIZE			32
#else
	#define HEAP_STACK_SIZE			64
#endif

#define MIN_ALLOC_SIZE				16

class HeapManager
{
private:
	ATTRIBUTE_ALIGNED16(u8 *mHeap);
	ATTRIBUTE_ALIGNED16(u8 *mPoolStack[HEAP_STACK_SIZE]);
	ATTRIBUTE_ALIGNED16(u32 mHeapBytes);
	ATTRIBUTE_ALIGNED16(u32 mCurrStack);

public:
	enum { ALIGN16, ALIGN128 };

	HeapManager(u8 *buf, s32 bytes)
	{
		mHeap = buf;
		mHeapBytes = bytes;
		clear();
	}

	~HeapManager()
	{
	}

	s32 getAllocated()
	{
		return (s32)(mPoolStack[mCurrStack] - mHeap);
	}

	s32 getRest()
	{
		return mHeapBytes - getAllocated();
	}

	void* allocate(size_t bytes, s32 alignment = ALIGN16)
	{
#ifdef __SPU__
		const vec_uint4 mask_align = ((vec_uint4){ALIGN16, ALIGN16, ALIGN128, ALIGN128});
		const vec_uint4 vmin_bytes = ((vec_uint4){MIN_ALLOC_SIZE, MIN_ALLOC_SIZE, MIN_ALLOC_SIZE, MIN_ALLOC_SIZE});
		const vec_uint4 valign = (vec_uint4)spu_splats(alignment);
		const vec_uint4 aligncmp = spu_cmpeq(valign, mask_align);

		const vec_uint4 vaddr = spu_splats((u32)mPoolStack[mCurrStack]);
		vec_uint4 vbytes = spu_splats((u32)bytes);
		vbytes = spu_sel(vmin_bytes, vbytes, spu_cmpgt(vbytes, vmin_bytes));

		const vec_uchar16 mask_ptn0404 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13, 0x00, 0x01, 0x02, 0x03, 0x10, 0x11, 0x12, 0x13});
		vec_uint4 alignedAddr = spu_shuffle(vaddr, vbytes, mask_ptn0404);

		const vec_uint4 mask1 = ((vec_uint4){15, 15, 127, 127});
		const vec_uint4 mask2 = ((vec_uint4){0xfffffff0, 0xfffffff0, 0xffffff80, 0xffffff80});

		alignedAddr = spu_and(spu_add(mask1, alignedAddr), mask2);

		const vec_uchar16 mask_ptn01 = ((vec_uchar16){0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80});
		const vec_uchar16 mask_ptn23 = ((vec_uchar16){0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80});
		const vec_uchar16 mask_ptn10 = ((vec_uchar16){0x04, 0x05, 0x06, 0x07, 0x00, 0x01, 0x02, 0x03, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80});

		alignedAddr = spu_shuffle(alignedAddr, alignedAddr, spu_sel(mask_ptn23, mask_ptn01, (vec_uchar16)aligncmp));

		vec_uint4 nextAddr = spu_add(alignedAddr, spu_shuffle(alignedAddr, alignedAddr, mask_ptn10));

		const vec_uint4 limitcheck = spu_cmpgt(nextAddr, spu_splats(mHeapBytes + (u32)mHeap));

		if(spu_extract(spu_gather(limitcheck), 0) > 0) {
			PRINTF("SPU: HMERR2\n");
			HALT();
		}

		mPoolStack[++mCurrStack] = (u8*)(spu_extract(nextAddr, 0));
		return (void*)spu_extract(alignedAddr, 0);
#else
		if(bytes < MIN_ALLOC_SIZE) bytes = MIN_ALLOC_SIZE;
		if(mCurrStack == HEAP_STACK_SIZE - 1) {
			PRINTF("HMERR1\n");
			HALT();
		}

		u32 p = (u32)((u64)mPoolStack[mCurrStack]);
		if(alignment == ALIGN128) {
			p = ((p + 0x7f)&~0x7f);
			bytes = (bytes + 0x7f)&~0x7f;
		} else {
			bytes = (bytes + 0x0f)&~0x0f;
		}

		if(bytes > (mHeapBytes - (p - (u32)(u64)mHeap))) {
			PRINTF("HMERR2\n");
			HALT();
		}

		mPoolStack[++mCurrStack] = (u8*)(p + bytes);
		return (void*)(u64)p;
#endif
	}

	void deallocate(void *ptr)
	{
		(void)ptr;
		mCurrStack--;
	}

	void clear()
	{
		mPoolStack[0] = mHeap;
		mCurrStack = 0;
	}

	void printStack()
	{
		for(u32 i=0;i < mCurrStack;i++)
			PRINTF("memStack %2d 0x%x\n", i, (u32)((u64)mPoolStack[i]));
	}
};

#endif /* HEAPMANAGER_H_ */
