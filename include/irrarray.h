#ifndef __IRRARRAY_H__
#define __IRRARRAY_H__

#include "irrtypes.h"
#include "heapsort.h"
#include "irrallocator.h"
#include "irrheapallocator.h"

namespace irr
{
	namespace core
	{
		template< class T, typename TAlloc = irrallocator<T> >
		class array
		{
		public:
			array() : data(NULL),used(0),allocated(0),free_it(true),is_sorted(true),strategy(ALLOC_STRATEGY_DOUBLE) {};

			array(u32 start_count) : data(NULL),used(0),allocated(0),free_it(true),is_sorted(true),strategy(ALLOC_STRATEGY_DOUBLE)
			{
				reallocate(start_count);
			}

			array(const array<T,TAlloc> &other) : data(NULL)
			{
				*this = other;
			}

			~array()
			{
				clear();
			}

			const array<T, TAlloc>& operator = (const array<T, TAlloc>& other)
			{
				if(this == &other)
					return *this;

				strategy = other.strategy;

				if(data != NULL)
					clear();

				if(other.allocated == 0)
					data = NULL;
				else
					data = allocator.allocate(other.allocated);

				used = other.used;
				free_it = other.free_it;
				is_sorted = other.is_sorted;
				allocated = other.allocated;

				for(u32 i=0;i < other.used;i++)
					allocator.construct(&data[i], other.data[i]);

				return *this;
			}

			void reallocate(u32 new_size, bool can_shrink = true)
			{
				if(allocated == new_size)
					return;
				if(!can_shrink && new_size < allocated)
					return;

				T *old_data = data;

				data = allocator.allocate(new_size);
				allocated = new_size;

				s32 end = used < new_size ? used : new_size;
				for(s32 i=0;i < end;i++)
					allocator.construct(&data[i],old_data[i]);
				for(u32 j=0;j < used;j++)
					allocator.destruct(&old_data[j]);

				if(allocated < used)
					used = allocated;

				allocator.deallocate(old_data);
			}

			void setAllocStrategy(eAllocStrategy newStrategy = ALLOC_STRATEGY_DOUBLE)
			{
				strategy = newStrategy;
			}

			void push_back(const T &elem)
			{
				insert(elem, used);
			}

			void push_front(const T &elem)
			{
				insert(elem);
			}

			void insert(const T &elem,u32 index = 0)
			{
				if(used + 1 > allocated) {
					u32 newAlloc;
					const T e(elem);

					switch(strategy) {
						case ALLOC_STRATEGY_DOUBLE:
							newAlloc = used + 1 + (allocated < 500 ? (allocated < 5 ? 5 : used) : used>>2);
							break;
						default:
						case ALLOC_STRATEGY_SAFE:
							newAlloc = used + 1;
							break;
					}
					reallocate(newAlloc);

					for(u32 i=used;i > index;--i) {
						if(i < used)
							allocator.destruct(&data[i]);

						allocator.construct(&data[i], data[i - 1]);
					}

					if(used > index)
						allocator.destruct(&data[index]);

					allocator.construct(&data[index], e);
				} else {
					if(used > index) {
						allocator.construct(&data[used], data[used - 1]);

						for(u32 i=used - 1;i > index;--i)
							data[i] = data[i - 1];

						data[index] = elem;
					} else
						allocator.construct(&data[index], elem);
				}

				is_sorted = false;
				used++;
			}

			void erase(u32 index)
			{
				u32 i;

				for(i=index+1;i<used;i++) {
					allocator.destruct(&data[i-1]);
					allocator.construct(&data[i-1],data[i]);
				}
				allocator.destruct(&data[used-1]);
				used--;
			}

			T& getLast()
			{
				return data[used - 1];
			}
			
			const T& getLast() const
			{
				return data[used - 1];
			}

			T& operator [](u32 index)
			{
				return data[index];
			}

			const T& operator [](u32 index) const
			{
				return data[index];
			}

			T* pointer()
			{
				return data;
			}

			const T* const_pointer() const
			{
				return data;
			}

			u32 size() const
			{
				return used;
			}

			u32 allocated_size() const
			{
				return allocated;
			}

			bool empty() const
			{
				return used == 0;
			}

			void set_used(u32 new_used)
			{
				if(allocated < new_used)
					reallocate(new_used);

				used = new_used;
			}

			void clear()
			{
				if(free_it) {
					for(u32 i=0;i < used;i++)
						allocator.destruct(&data[i]);

					allocator.deallocate(data);
				}

				data = NULL;
				used = 0;
				allocated = 0;
				is_sorted = true;
			}

			void sort()
			{
				if(!is_sorted && used > 1)
					heapsort(data, used);
				is_sorted = true;
			}

			s32 binary_search(const T& elem)
			{
				sort();
				return binary_search(elem, 0, used - 1);
			}

			s32 binary_search_const(const T& elem) const
			{
				if(is_sorted)
					return binary_search(elem, 0, used - 1);
				else
					return linear_search(elem);
			}

			s32 binary_search(const T& elem, s32 left, s32 right) const
			{
				s32 m;

				if(!used) return -1;

				do {
					m = (left + right)>>1;
					if(elem < data[m])
						right = m - 1;
					else 
						left = m + 1;
				} while((elem < data[m] || data[m] < elem) && left <= right);

				if(!(elem < data[m]) && !(data[m] < elem))
					return m;

				return -1;
			}

			s32 linear_search(const T& elem) const
			{
				for(u32 i=0;i < used;i++) {
					if(elem == data[i]) return (s32)i;
				}
				return -1;
			}

			s32 linear_reverse_search(const T& elem) const
			{
				for(s32 i=used - 1;i >= 0;i--) {
					if(data[i] == elem) return i;
				}
				return -1;
			}

			void set_sorted(bool issorted)
			{
				is_sorted = issorted;
			}

		private:
			T *data;
			TAlloc allocator;
			u32 used;
			u32 allocated;
			bool free_it;
			bool is_sorted;
			eAllocStrategy strategy;
		};
	}
}

#endif
