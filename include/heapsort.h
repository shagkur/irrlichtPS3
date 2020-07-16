#ifndef __HEAPSORT_H__
#define __HEAPSORT_H__

#include "irrtypes.h"

namespace irr
{
	namespace core
	{
		template< class T >
		inline void heapsink(T *array, s32 element, s32 max)
		{
			while((element<<1) < max) {
				s32 j = (element<<1);
				if(j + 1 < max && array[j] < array[j + 1]) j = j + 1;

				if(array[element] < array[j]) {
					T t = array[j];
					array[j] = array[element];
					array[element] = t;
					element = j;
				} else
					return;
			}
		}

		template< class T >
		inline void heapsort(T *_array, s32 size)
		{
			T *virt_array = _array - 1;
			s32 i, virt_size = size + 2;

			for(i=((size - 1)/2);i >= 0;--i)
				heapsink(virt_array, i + 1, virt_size - 1);

			for(i=size - 1;i >= 0;--i) {
				T t = _array[0];
				_array[0] = _array[i];
				_array[i] = t;
				heapsink(virt_array, 1, i + 1);
			}
		}
	}
}

#endif
