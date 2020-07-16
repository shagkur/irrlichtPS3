/*
 * prefetchiterator.h
 *
 *  Created on: Jun 1, 2013
 *      Author: mike
 */

#ifndef PREFETCHITERATOR_H_
#define PREFETCHITERATOR_H_

#include "common.h"
#include "heapmanager.h"

///////////////////////////////////////////////////////////////////////////////
// ReadOnlyPrefetchForwardIterator

template< class T >
class ReadOnlyPrefetchForwardIterator
{
private:
	ATTRIBUTE_ALIGNED16(HeapManager *mPool);

	ATTRIBUTE_ALIGNED16(s32 mDmaTag);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchNum);
	ATTRIBUTE_ALIGNED16(vec_uint4 szData);
	ATTRIBUTE_ALIGNED16(vec_uint4 mPrefetchSize);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEA);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEndEA);
	ATTRIBUTE_ALIGNED16(s32 mIndex);
	ATTRIBUTE_ALIGNED16(s32 mCurrentBuffer);
	ATTRIBUTE_ALIGNED16(T *mPtrBuf);
	ATTRIBUTE_ALIGNED128(T *mBuffer[2]);

	void dmaGetBuffer(T* ls,const vec_uint4 ea,const vec_uint4 end,const vec_uint4 sz,u32 tag)
	{
		vec_uint4 outsz;
		const vec_uint4 v0 = ((vec_uint4){0, 0, 0, 0});
		outsz = spu_sel(sz,spu_sub(end, ea), spu_cmpgt(spu_add(ea, sz), end));
		outsz = spu_sel(outsz, v0, spu_cmpgt(ea, end));
		spu_dma_get(ls, spu_extract(ea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

public:
	ReadOnlyPrefetchForwardIterator(HeapManager *pool, u32 initialEA, u32 endEA, s32 prefetchNum, s32 tag = 0);
	~ReadOnlyPrefetchForwardIterator();

	ReadOnlyPrefetchForwardIterator& operator++();
	const T& operator*() const {return *mPtrBuf;}
	T* getPtr() {return mPtrBuf;}
	u32 getEA() const {return spu_extract(mEA, 0);}
};

template< class T >
inline ReadOnlyPrefetchForwardIterator<T>::ReadOnlyPrefetchForwardIterator(HeapManager *pool, u32 initialEA, u32 endEA, s32 prefetchNum, s32 tag)
: mPool(pool), mDmaTag(tag), mPrefetchNum(prefetchNum), mIndex(0), mCurrentBuffer(0)
{
	szData = spu_splats((u32)sizeof(T));
	mEA = spu_splats(initialEA);
	mEndEA = spu_splats(endEA);
	mPrefetchSize = spu_splats((u32)sizeof(T)*prefetchNum);
	mBuffer[0] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mBuffer[1] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;

	dmaGetBuffer(mBuffer[0], mEA, mEndEA, mPrefetchSize, mDmaTag);
	spu_dma_wait_tag_status_all(1<<mDmaTag);
	dmaGetBuffer(mBuffer[1], spu_add(mEA, mPrefetchSize), mEndEA, mPrefetchSize, mDmaTag);
}

template< class T >
inline ReadOnlyPrefetchForwardIterator<T>::~ReadOnlyPrefetchForwardIterator()
{
	spu_dma_wait_tag_status_all(1<<mDmaTag);

	mPool->deallocate(mBuffer[1]);
	mPool->deallocate(mBuffer[0]);
}

template< class T >
inline ReadOnlyPrefetchForwardIterator<T>& ReadOnlyPrefetchForwardIterator<T>::operator++()
{
	mEA = spu_add(mEA, szData);
	mIndex++;
	mPtrBuf++;
	if(UNLIKELY(mIndex >= mPrefetchNum)) {
		spu_dma_wait_tag_status_all(1<<mDmaTag);
		dmaGetBuffer(mBuffer[mCurrentBuffer], spu_add(mEA, mPrefetchSize), mEndEA, mPrefetchSize, mDmaTag);
		mCurrentBuffer = 1 - mCurrentBuffer;
		mIndex = 0;
		mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;
	}
	return *this;
}

///////////////////////////////////////////////////////////////////////////////
// WriteOnlyPrefetchForwardIterator

template< class T >
class WriteOnlyPrefetchForwardIterator
{
private:
	ATTRIBUTE_ALIGNED16(HeapManager *mPool);

	ATTRIBUTE_ALIGNED16(s32 mDmaTag);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchNum);
	ATTRIBUTE_ALIGNED16(vec_uint4 szData);
	ATTRIBUTE_ALIGNED16(vec_uint4 mPrefetchSize);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEA);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEndEA);
	ATTRIBUTE_ALIGNED16(s32 mIndex);
	ATTRIBUTE_ALIGNED16(s32 mCurrentBuffer);
	ATTRIBUTE_ALIGNED16(T *mPtrBuf);
	ATTRIBUTE_ALIGNED128(T *mBuffer[2]);

	void dmaPutBuffer(T* ls, const vec_uint4 ea, const vec_uint4 end, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz;
		const vec_uint4 v0 = ((vec_uint4){0, 0, 0, 0});
		outsz = spu_sel(sz, spu_sub(end, ea), spu_cmpgt(spu_add(ea, sz), end));
		outsz = spu_sel(outsz, v0, spu_cmpgt(ea, end));
		spu_dma_put(ls, spu_extract(ea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

public:
	WriteOnlyPrefetchForwardIterator(HeapManager *pool, u32 initialEA, u32 endEA, s32 prefetchNum, s32 tag = 0);
	~WriteOnlyPrefetchForwardIterator();

	WriteOnlyPrefetchForwardIterator& operator++();
	T& operator*() const {return *mPtrBuf;}
	T* getPtr() {return mPtrBuf;}
	u32 getEA() const {return spu_extract(mEA, 0);}
};

template< class T >
inline WriteOnlyPrefetchForwardIterator<T>::WriteOnlyPrefetchForwardIterator(HeapManager *pool, u32 initialEA, u32 endEA, s32 prefetchNum, s32 tag)
: mPool(pool), mDmaTag(tag), mPrefetchNum(prefetchNum), mIndex(0), mCurrentBuffer(0)
{
	szData = spu_splats((u32)sizeof(T));
	mEA = spu_splats(initialEA);
	mEndEA = spu_splats(endEA);
	mPrefetchSize = spu_splats((u32)sizeof(T)*prefetchNum);
	mBuffer[0] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mBuffer[1] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;
}

template< class T >
inline WriteOnlyPrefetchForwardIterator<T>::~WriteOnlyPrefetchForwardIterator()
{
	vec_uint4 putsz = spu_splats(mIndex*(u32)sizeof(T));
	vec_uint4 putea = spu_sub(mEA, putsz);
	dmaPutBuffer(mBuffer[mCurrentBuffer], putea, mEndEA, putsz, mDmaTag);
	spu_dma_wait_tag_status_all(1<<mDmaTag);

	mPool->deallocate(mBuffer[1]);
	mPool->deallocate(mBuffer[0]);
}

template< class T >
inline WriteOnlyPrefetchForwardIterator<T>& WriteOnlyPrefetchForwardIterator<T>::operator++()
{
	mEA = spu_add(mEA, szData);
	mIndex++;
	mPtrBuf++;
	if(UNLIKELY(mIndex >= mPrefetchNum)) {
		spu_dma_wait_tag_status_all(1<<mDmaTag);
		dmaPutBuffer(mBuffer[mCurrentBuffer], spu_sub(mEA, mPrefetchSize), mEndEA, mPrefetchSize, mDmaTag);
		mCurrentBuffer = 1 - mCurrentBuffer;
		mIndex = 0;
		mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;
	}
	return *this;
}

///////////////////////////////////////////////////////////////////////////////
// PrefetchForwardIterator

template< class T >
class PrefetchForwardIterator
{
private:
	ATTRIBUTE_ALIGNED16(HeapManager *mPool);

	ATTRIBUTE_ALIGNED16(s32 mDmaTag);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchNum);
	ATTRIBUTE_ALIGNED16(vec_uint4 szData);
	ATTRIBUTE_ALIGNED16(vec_uint4 mPrefetchSize);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEA);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEndEA);
	ATTRIBUTE_ALIGNED16(s32 mIndex);
	ATTRIBUTE_ALIGNED16(s32 mCurrentBuffer);
	ATTRIBUTE_ALIGNED16(T *mPtrBuf);
	ATTRIBUTE_ALIGNED128(T *mBuffer[2]);

	void dmaGetBuffer(T* ls, const vec_uint4 ea, const vec_uint4 end, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz;
		const vec_uint4 v0 = {0, 0, 0, 0};
		outsz = spu_sel(sz, spu_sub(end, ea),spu_cmpgt(spu_add(ea, sz), end));
		outsz = spu_sel(outsz, v0, spu_cmpgt(ea, end));
		spu_dma_get(ls, spu_extract(ea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

	void dmaGetBufferf(T* ls, const vec_uint4 ea, const vec_uint4 end, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz;
		const vec_uint4 v0 = {0, 0, 0, 0};
		outsz = spu_sel(sz, spu_sub(end, ea), spu_cmpgt(spu_add(ea, sz), end));
		outsz = spu_sel(outsz, v0, spu_cmpgt(ea, end));
		spu_dma_getf(ls, spu_extract(ea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

	void dmaPutBuffer(T* ls, const vec_uint4 ea, const vec_uint4 end, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz;
		const vec_uint4 v0 = {0, 0, 0, 0};
		outsz = spu_sel(sz, spu_sub(end, ea), spu_cmpgt(spu_add(ea, sz), end));
		outsz = spu_sel(outsz, v0, spu_cmpgt(ea, end));
		spu_dma_put(ls, spu_extract(ea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

public:
	PrefetchForwardIterator(HeapManager *pool, u32 initialEA, u32 endEA, s32 prefetchNum, s32 tag = 0);
	~PrefetchForwardIterator();

	PrefetchForwardIterator& operator++();
	T& operator*() const {return *mPtrBuf;}
	T* getPtr() {return mPtrBuf;}
	u32 getEA() const {return spu_extract(mEA, 0);}
};

template< class T >
inline PrefetchForwardIterator<T>::PrefetchForwardIterator(HeapManager *pool, u32 initialEA, u32 endEA, s32 prefetchNum, s32 tag)
: mPool(pool), mDmaTag(tag), mPrefetchNum(prefetchNum), mIndex(0), mCurrentBuffer(0)
{
	szData = spu_splats((u32)sizeof(T));
	mEA = spu_splats(initialEA);
	mEndEA = spu_splats(endEA);
	mPrefetchSize = spu_splats((u32)sizeof(T)*prefetchNum);
	mBuffer[0] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mBuffer[1] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;

	dmaGetBuffer(mBuffer[0], mEA, mEndEA, mPrefetchSize, mDmaTag);
	spu_dma_wait_tag_status_all(1<<mDmaTag);
	dmaGetBuffer(mBuffer[1], spu_add(mEA, mPrefetchSize), mEndEA, mPrefetchSize, mDmaTag);
}

template< class T >
inline PrefetchForwardIterator<T>::~PrefetchForwardIterator()
{
	vec_uint4 putsz = spu_splats(mIndex*(u32)sizeof(T));
	vec_uint4 putea = spu_sub(mEA, putsz);
	dmaPutBuffer(mBuffer[mCurrentBuffer], putea, mEndEA, putsz, mDmaTag);
	spu_dma_wait_tag_status_all(1<<mDmaTag);

	mPool->deallocate(mBuffer[1]);
	mPool->deallocate(mBuffer[0]);
}

template< class T >
inline PrefetchForwardIterator<T>& PrefetchForwardIterator<T>::operator++()
{
	mEA = spu_add(mEA, szData);
	mIndex++;
	mPtrBuf++;
	if(UNLIKELY(mIndex >= mPrefetchNum)) {
		spu_dma_wait_tag_status_all(1<<mDmaTag);
		dmaPutBuffer(mBuffer[mCurrentBuffer], spu_sub(mEA, mPrefetchSize), mEndEA, mPrefetchSize, mDmaTag);
		dmaGetBufferf(mBuffer[mCurrentBuffer], spu_add(mEA, mPrefetchSize), mEndEA, mPrefetchSize, mDmaTag);
		mCurrentBuffer = 1 - mCurrentBuffer;
		mIndex = 0;
		mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;
	}
	return *this;
}

///////////////////////////////////////////////////////////////////////////////
// ReadOnlyPrefetchBackwardIterator

template< class T >
class ReadOnlyPrefetchBackwardIterator
{
private:
	ATTRIBUTE_ALIGNED16(HeapManager *mPool);

	ATTRIBUTE_ALIGNED16(s32 mDmaTag);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchNum);
	ATTRIBUTE_ALIGNED16(vec_uint4 szData);
	ATTRIBUTE_ALIGNED16(vec_uint4 mPrefetchSize);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEA);
	ATTRIBUTE_ALIGNED16(vec_uint4 mStartEA);
	ATTRIBUTE_ALIGNED16(s32 mIndex);
	ATTRIBUTE_ALIGNED16(s32 mCurrentBuffer);
	ATTRIBUTE_ALIGNED16(T *mPtrBuf);
	ATTRIBUTE_ALIGNED128(T *mBuffer[2]);

	void dmaGetBuffer(T* ls, const vec_uint4 ea, const vec_uint4 start, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz,outls,outea;
		const vec_uint4 v0 = {0, 0, 0, 0};
		const vec_uint4 vls = spu_splats((u32)ls);
		const vec_uint4 ea_sz = spu_add(ea, sz);
		vec_uint4 cmp1 = spu_cmpgt(start, ea);
		outls = spu_sel(vls, spu_add(vls, spu_sub(start, ea)), cmp1);
		outea = spu_sel(ea, start, cmp1);
		outsz = spu_sel(sz, spu_sub(ea_sz, start), cmp1);
		vec_uint4 cmp2 = spu_cmpgt(start, ea_sz);
		outsz = spu_sel(outsz, v0, cmp2);
		spu_dma_get((void*)spu_extract(outls, 0), spu_extract(outea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

public:
	ReadOnlyPrefetchBackwardIterator(HeapManager *pool, u32 initialEA, u32 startEA, s32 prefetchNum, s32 tag = 0);
	~ReadOnlyPrefetchBackwardIterator();

	ReadOnlyPrefetchBackwardIterator& operator--();
	const T& operator*() const {return *mPtrBuf;}
	u32 getEA() const {return spu_extract(mEA,0);}
};

template< class T >
inline ReadOnlyPrefetchBackwardIterator<T>::ReadOnlyPrefetchBackwardIterator(HeapManager *pool, u32 initialEA, u32 startEA, s32 prefetchNum, s32 tag)
: mPool(pool), mDmaTag(tag), mPrefetchNum(prefetchNum), mIndex(prefetchNum), mCurrentBuffer(0)
{
	szData = spu_splats((u32)sizeof(T));
	mEA = spu_splats(initialEA);
	mStartEA = spu_splats(startEA);
	mPrefetchSize = spu_splats((u32)sizeof(T)*prefetchNum);

	mBuffer[0] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mBuffer[1] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;

	dmaGetBuffer(mBuffer[0],spu_sub(mEA,mPrefetchSize),mStartEA,mPrefetchSize,mDmaTag);
	spu_dma_wait_tag_status_all(1<<mDmaTag);
	dmaGetBuffer(mBuffer[1],spu_sub(mEA,spu_add(mPrefetchSize,mPrefetchSize)),mStartEA,mPrefetchSize,mDmaTag);
}

template< class T >
inline ReadOnlyPrefetchBackwardIterator<T>::~ReadOnlyPrefetchBackwardIterator()
{
	spu_dma_wait_tag_status_all(1<<mDmaTag);

	mPool->deallocate(mBuffer[1]);
	mPool->deallocate(mBuffer[0]);
}

template< class T >
inline ReadOnlyPrefetchBackwardIterator<T>& ReadOnlyPrefetchBackwardIterator<T>::operator--()
{
	mEA = spu_sub(mEA, szData);
	mIndex--;
	mPtrBuf--;
	if(UNLIKELY(mIndex < 0)) {
		spu_dma_wait_tag_status_all(1<<mDmaTag);
		dmaGetBuffer(mBuffer[mCurrentBuffer], spu_add(spu_sub(mEA, spu_add(mPrefetchSize, mPrefetchSize)), szData), mStartEA, mPrefetchSize, mDmaTag);
		mCurrentBuffer = 1 - mCurrentBuffer;
		mIndex = mPrefetchNum - 1;
		mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;
	}
	return *this;
}

///////////////////////////////////////////////////////////////////////////////
// WriteOnlyPrefetchBackwardIterator

template< class T >
class WriteOnlyPrefetchBackwardIterator
{
private:
	ATTRIBUTE_ALIGNED16(HeapManager *mPool);

	ATTRIBUTE_ALIGNED16(s32 mDmaTag);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchNum);
	ATTRIBUTE_ALIGNED16(vec_uint4 szData);
	ATTRIBUTE_ALIGNED16(vec_uint4 mPrefetchSize);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEA);
	ATTRIBUTE_ALIGNED16(vec_uint4 mStartEA);
	ATTRIBUTE_ALIGNED16(s32 mIndex);
	ATTRIBUTE_ALIGNED16(s32 mCurrentBuffer);
	ATTRIBUTE_ALIGNED16(T *mPtrBuf);
	ATTRIBUTE_ALIGNED128(T *mBuffer[2]);

	void dmaPutBuffer(T* ls, const vec_uint4 ea, const vec_uint4 start, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz,outls,outea;
	 	const vec_uint4 v0 = {0, 0, 0, 0};
		const vec_uint4 vls = spu_splats((u32)ls);
		const vec_uint4 ea_sz = spu_add(ea, sz);
		vec_uint4 cmp1 = spu_cmpgt(start, ea);
		outls = spu_sel(vls, spu_add(vls, spu_sub(start, ea)), cmp1);
		outea = spu_sel(ea, start, cmp1);
		outsz = spu_sel(sz, spu_sub(ea_sz, start), cmp1);
		vec_uint4 cmp2 = spu_cmpgt(start, ea_sz);
		outsz = spu_sel(outsz, v0, cmp2);
		spu_dma_put((void*)spu_extract(outls, 0), spu_extract(outea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

public:
	WriteOnlyPrefetchBackwardIterator(HeapManager *pool, u32 initialEA, u32 startEA, s32 prefetchNum, s32 tag = 0);
	~WriteOnlyPrefetchBackwardIterator();

	WriteOnlyPrefetchBackwardIterator& operator--();
	T& operator*() const {return *mPtrBuf;}
	u32 getEA() const {return spu_extract(mEA,0);}
};

template< class T >
inline WriteOnlyPrefetchBackwardIterator<T>::WriteOnlyPrefetchBackwardIterator(HeapManager *pool, u32 initialEA, u32 startEA, s32 prefetchNum, s32 tag)
: mPool(pool), mDmaTag(tag), mPrefetchNum(prefetchNum), mIndex(prefetchNum), mCurrentBuffer(0)
{
	szData = spu_splats((u32)sizeof(T));
	mEA = spu_splats(initialEA);
	mStartEA = spu_splats(startEA);
	mPrefetchSize = spu_splats((u32)sizeof(T)*prefetchNum);

	mBuffer[0] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mBuffer[1] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;
}

template< class T >
inline WriteOnlyPrefetchBackwardIterator<T>::~WriteOnlyPrefetchBackwardIterator()
{
	vec_uint4 putsz = spu_sub(mPrefetchSize, spu_splats(mIndex*(u32)sizeof(T)));
	dmaPutBuffer(mBuffer[mCurrentBuffer] + mIndex, mEA, mStartEA, putsz, mDmaTag);
	spu_dma_wait_tag_status_all(1<<mDmaTag);

	mPool->deallocate(mBuffer[1]);
	mPool->deallocate(mBuffer[0]);
}

template< class T >
inline WriteOnlyPrefetchBackwardIterator<T>& WriteOnlyPrefetchBackwardIterator<T>::operator--()
{
	mEA = spu_sub(mEA, szData);
	mIndex--;
	mPtrBuf--;
	if(UNLIKELY(mIndex < 0)) {
		spu_dma_wait_tag_status_all(1<<mDmaTag);
		dmaPutBuffer(mBuffer[mCurrentBuffer], spu_add(mEA, szData), mStartEA, mPrefetchSize, mDmaTag);
		mCurrentBuffer = 1 - mCurrentBuffer;
		mIndex = mPrefetchNum - 1;
		mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;
	}
	return *this;
}

///////////////////////////////////////////////////////////////////////////////
// PrefetchBackwardIterator

template< class T >
class PrefetchBackwardIterator
{
private:
	ATTRIBUTE_ALIGNED16(HeapManager *mPool);

	ATTRIBUTE_ALIGNED16(s32 mDmaTag);
	ATTRIBUTE_ALIGNED16(s32 mPrefetchNum);
	ATTRIBUTE_ALIGNED16(vec_uint4 szData);
	ATTRIBUTE_ALIGNED16(vec_uint4 mPrefetchSize);
	ATTRIBUTE_ALIGNED16(vec_uint4 mEA);
	ATTRIBUTE_ALIGNED16(vec_uint4 mStartEA);
	ATTRIBUTE_ALIGNED16(s32 mIndex);
	ATTRIBUTE_ALIGNED16(s32 mCurrentBuffer);
	ATTRIBUTE_ALIGNED16(T *mPtrBuf);
	ATTRIBUTE_ALIGNED128(T *mBuffer[2]);

	void dmaGetBuffer(T* ls, const vec_uint4 ea, const vec_uint4 start, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz,outls,outea;
		const vec_uint4 v0 = {0, 0, 0, 0};
		const vec_uint4 vls = spu_splats((u32)ls);
		const vec_uint4 ea_sz = spu_add(ea, sz);
		vec_uint4 cmp1 = spu_cmpgt(start, ea);
		outls = spu_sel(vls, spu_add(vls, spu_sub(start, ea)), cmp1);
		outea = spu_sel(ea, start, cmp1);
		outsz = spu_sel(sz, spu_sub(ea_sz, start), cmp1);
		vec_uint4 cmp2 = spu_cmpgt(start, ea_sz);
		outsz = spu_sel(outsz, v0, cmp2);
		spu_dma_get((void*)spu_extract(outls, 0), spu_extract(outea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

	void dmaGetBufferf(T* ls, const vec_uint4 ea, const vec_uint4 start, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz,outls,outea;
		const vec_uint4 v0 = {0, 0, 0, 0};
		const vec_uint4 vls = spu_splats((u32)ls);
		const vec_uint4 ea_sz = spu_add(ea, sz);
		vec_uint4 cmp1 = spu_cmpgt(start, ea);
		outls = spu_sel(vls, spu_add(vls, spu_sub(start, ea)), cmp1);
		outea = spu_sel(ea,start, cmp1);
		outsz = spu_sel(sz, spu_sub(ea_sz, start), cmp1);
		vec_uint4 cmp2 = spu_cmpgt(start, ea_sz);
		outsz = spu_sel(outsz, v0, cmp2);
		spu_dma_getf((void*)spu_extract(outls, 0), spu_extract(outea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

	void dmaPutBuffer(T* ls, const vec_uint4 ea, const vec_uint4 start, const vec_uint4 sz, u32 tag)
	{
		vec_uint4 outsz,outls,outea;
	 	const vec_uint4 v0 = {0, 0, 0, 0};
		const vec_uint4 vls = spu_splats((u32)ls);
		const vec_uint4 ea_sz = spu_add(ea, sz);
		vec_uint4 cmp1 = spu_cmpgt(start, ea);
		outls = spu_sel(vls, spu_add(vls, spu_sub(start, ea)), cmp1);
		outea = spu_sel(ea, start, cmp1);
		outsz = spu_sel(sz, spu_sub(ea_sz, start), cmp1);
		vec_uint4 cmp2 = spu_cmpgt(start, ea_sz);
		outsz = spu_sel(outsz, v0, cmp2);
		spu_dma_put((void*)spu_extract(outls, 0), spu_extract(outea, 0), spu_extract(outsz, 0), tag, 0, 0);
	}

public:
	PrefetchBackwardIterator(HeapManager *pool, u32 initialEA, u32 startEA, s32 prefetchNum, s32 tag = 0);
	~PrefetchBackwardIterator();

	PrefetchBackwardIterator& operator--();
	T& operator*() const {return *mPtrBuf;}
	u32 getEA() const {return spu_extract(mEA, 0);}
};

template< class T >
inline PrefetchBackwardIterator<T>::PrefetchBackwardIterator(HeapManager *pool, u32 initialEA, u32 startEA, s32 prefetchNum, s32 tag)
: mPool(pool), mDmaTag(tag), mPrefetchNum(prefetchNum), mIndex(prefetchNum), mCurrentBuffer(0)
{
	szData = spu_splats((u32)sizeof(T));
	mEA = spu_splats(initialEA);
	mStartEA = spu_splats(startEA);
	mPrefetchSize = spu_splats((u32)sizeof(T)*prefetchNum);

	mBuffer[0] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mBuffer[1] = (T*)mPool->allocate(sizeof(T)*mPrefetchNum, HeapManager::ALIGN128);
	mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;

	dmaGetBuffer(mBuffer[0], spu_sub(mEA, mPrefetchSize), mStartEA, mPrefetchSize, mDmaTag);
	spu_dma_wait_tag_status_all(1<<mDmaTag);
	dmaGetBuffer(mBuffer[1], spu_sub(mEA, spu_add(mPrefetchSize, mPrefetchSize)), mStartEA, mPrefetchSize, mDmaTag);
}

template< class T >
inline PrefetchBackwardIterator<T>::~PrefetchBackwardIterator()
{
	vec_uint4 putsz = spu_sub(mPrefetchSize, spu_splats(mIndex*(u32)sizeof(T)));
	dmaPutBuffer(mBuffer[mCurrentBuffer] + mIndex, mEA, mStartEA, putsz, mDmaTag);
	spu_dma_wait_tag_status_all(1<<mDmaTag);

	mPool->deallocate(mBuffer[1]);
	mPool->deallocate(mBuffer[0]);
}

template< class T >
inline PrefetchBackwardIterator<T>& PrefetchBackwardIterator<T>::operator--()
{
	mEA = spu_sub(mEA, szData);
	mIndex--;
	mPtrBuf--;
	if(UNLIKELY(mIndex < 0)) {
		spu_dma_wait_tag_status_all(1<<mDmaTag);
		dmaPutBuffer(mBuffer[mCurrentBuffer], spu_add(mEA, szData), mStartEA, mPrefetchSize, mDmaTag);
		dmaGetBufferf(mBuffer[mCurrentBuffer], spu_add(spu_sub(mEA, spu_add(mPrefetchSize, mPrefetchSize)), szData), mStartEA, mPrefetchSize, mDmaTag);
		mCurrentBuffer = 1 - mCurrentBuffer;
		mIndex = mPrefetchNum - 1;
		mPtrBuf = mBuffer[mCurrentBuffer] + mIndex;
	}
	return *this;
}

#endif /* PREFETCHITERATOR_H_ */
