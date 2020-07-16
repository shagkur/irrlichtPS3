#ifndef __IRRSTRING_H__
#define __IRRSTRING_H__

#include "irrtypes.h"
#include "irrmath.h"
#include "irrallocator.h"

namespace irr
{
	namespace core
	{
		template< typename T, typename TAlloc = irrallocator<T> >
		class string
		{
		public:
			string() : array(NULL),allocated(1),used(1)
			{
				array = allocator.allocate(1);
				array[0] = 0x0;
			}

			string(const string<T>& other) : array(NULL),allocated(0),used(0)
			{
				*this = other;
			}

			explicit string(int number) : array(NULL),allocated(0),used(0)
			{
				bool negativ = false;
				if(number<0) {
					number *= -1;
					negativ = true;
				}

				u32 idx = 15;
				char tmpbuf[16];
				tmpbuf[15] = 0;

				if(!number) {
					tmpbuf[14] = '0';
					*this = &tmpbuf[14];
					return;
				}

				while(number && idx) {
					idx--;
					tmpbuf[idx] = (char)('0' + (number%10));
					number /= 10;
				}

				if(negativ) {
					idx--;
					tmpbuf[idx] = '-';
				}
				*this = &tmpbuf[idx];
			}

			explicit string(unsigned int number) : array(NULL),allocated(0),used(0)
			{
				u32 idx = 15;
				char tmpbuf[16];
				tmpbuf[15] = 0;

				if(!number) {
					tmpbuf[14] = '0';
					*this = &tmpbuf[14];
					return;
				}

				while(number && idx) {
					idx--;
					tmpbuf[idx] = (char)('0' + (number%10));
					number /= 10;
				}

				*this = &tmpbuf[idx];
			}

			explicit string(const float number) : array(NULL),allocated(0),used(0)
			{
				char tmpbuf[255];
				snprintf(tmpbuf,255,"%0.6f",number);
				*this = tmpbuf;
			}

			~string()
			{
				allocator.deallocate(array);
			}

			template<class B>
			string(const B* const c,u32 len) : array(NULL),allocated(0),used(0)
			{
				u32 i;

				if(!c) {
					*this = "";
					return;
				}

				allocated = used = len+1;
				array = allocator.allocate(used);
				for(i=0;i<len;i++) array[i] = (T)c[i];

				array[len] = 0;
			}

			template<class B>
			string(const B* const c) : array(NULL),allocated(0),used(0)
			{
				*this = c;
			}

			string<T> operator+(const string<T>& other) const
			{
				string<T> cstr(*this);
				cstr.append(other);
				return cstr;
			}

			template<class B>
			string<T> operator+(const B* const str) const
			{
				string<T> cstr(*this);
				cstr.append(str);
				return cstr;
			}

			string<T>& operator=(const string<T>& other)
			{
				u32 i;

				if(this==&other) return *this;

				allocator.deallocate(array);
				allocated = used = other.size()+1;
				array = allocator.allocate(used);

				const T* p = other.c_str();
				for(i=0;i<used;i++,p++) array[i] = *p;

				return *this;
			}

			template<class B>
			string<T>& operator=(const B* const c)
			{
				u32 len,l;

				if(!c) {
					if(!array) {
						array = allocator.allocate(1);
						allocated = 1;
					}
					used = 1;
					array[0] = 0x0;
					return *this;
				}

				if((void*)c==(void*)array)
					return *this;

				len = 0;
				const B* p = c;
				while(*p) {
					++len;
					++p;
				}

				T* oldarray = array;

				++len;
				allocated = used = len;
				array = allocator.allocate(used);
				for(l=0;l<len;l++) array[l] = (T)c[l];

				allocator.deallocate(oldarray);
				return *this;
			}

			T& operator[](const u32 index)
			{
				return array[index];
			}

			const T& operator[](const u32 index) const
			{
				return array[index];
			}

			void reserve(u32 count)
			{
				if(count<allocated) return;
				reallocate(count);
			}

			void replace(T toReplace,T replaceWith)
			{
				u32 i;

				for(i=0;i<used;i++) {
					if(array[i]==toReplace)
						array[i] = replaceWith;
				}
			}

			s32 findFirst(T c) const
			{
				u32 i;

				for(i=0;i<used;i++) {
					if(array[i]==c) return i;
				}

				return -1;
			}

			s32 findFirstChar(const T* const c,u32 count) const
			{
				u32 i,j;

				if(!c) return -1;

				for(i=0;i<used-1;i++) {
					for(j=0;j<count;j++) {
						if(array[i]==c[j]) return i;
					}
				}
				
				return -1;
			}

			template<class B>
			s32 findFirstCharNotInList(const B* const c,u32 count) const
			{
				u32 i,j;

				for(i=0;i<used;i++) {
					for(j=0;j<count;j++) {
						if(array[i]==c[j]) break;
					}
					if(j==count) return i;
				}

				return -1;
			}

			template<class B>
			s32 findLastCharNotInList(const B* const c,u32 count) const
			{
				s32 i,j;

				for(i=(s32)(used-2);i>=0;i--) {
					for(j=0;j<count;j++) {
						if(array[i]==c[j]) break;
					}
					if(j==count) return i;
				}

				return -1;
			}

			s32 findNext(T c,u32 startPos) const
			{
				u32 i;

				for(i=startPos;i<used;i++) {
					if(array[i]==c) return i;
				}

				return -1;
			}

			s32 findLast(T c,s32 start = -1) const
			{
				s32 i;

				start = core::clamp(start<0 ? (s32)(used)-1 : start,0,(s32)(used)-1);
				for(i=start;i>=0;--i) {
					if(array[i]==c) return i;
				}
				return -1;
			}

			string<T> trim()
			{
				const u32 whitespacecount = 4;
				const char whitespace[] = " \t\n\r";

				const s32 begin = findFirstCharNotInList(whitespace,whitespacecount);
				if(begin==-1) return (*this="");

				const s32 end = findLastCharNotInList(whitespace,whitespacecount);
				return (*this = subString(begin,(end+1) - begin));
			}

			string<T> subString(u32 start,s32 len) const
			{
				s32 i;
				string<T> o;

				if((len+start)>size()) len = size()-start;
				if(len<0) return string<T>("");

				o.reserve(len+1);
				for(i=0;i<len;i++) o.array[i] = array[i];

				o.array[len] = 0;
				o.used = o.allocated;

				return o;
			}

			const T* c_str() const
			{
				return array;
			}

			void make_lower()
			{
				u32 i;
				
				for(i=0;i<used;i++)
					array[i] = ansi_lower(array[i]);
			}

			void make_upper()
			{
				u32 i;
				const T a = (T)'a';
				const T z = (T)'z';
				const T diff = (T)'A' - a;

				for(i=0;i<used;i++) {
					if(array[i]>=a && array[i]<=z)
						array[i] += diff;
				}
			}

			u32 size() const
			{
				return used-1;
			}

			void reallocate(u32 new_size)
			{
				u32 i,amount;
				T *old_array = array;

				array = allocator.allocate(new_size);
				allocated = new_size;
				
				amount = used<new_size ? used : new_size;
				for(i=0;i<amount;i++) array[i] = old_array[i];

				if(allocated<used) used = allocated;
				allocator.deallocate(old_array);
			}

			bool equals(const string<T>& other) const
			{
				u32 i;

				for(i=0;array[i] && other[i];i++) {
					if(array[i] != other[i]) return false;
				}

				return used == other.used;
			}

			bool equals_ignore_case(const string<T>& other) const
			{
				u32 i;

				for(i=0;array[i] && other[i];i++) {
					if(ansi_lower(array[i])!=ansi_lower(other[i])) return false;
				}

				return used==other.used;
			}
			
			bool equals_substring_ignore_case(const string<T>& other,const s32 sourcePos = 0) const
			{
				u32 i;

				if((u32)sourcePos>used) return false;

				for(i=0;array[sourcePos+i] && other[i];i++) {
					if(ansi_lower(array[sourcePos+i])!=ansi_lower(other[i])) return false;
				}
				return array[sourcePos+i]==0 && other[i]==0;
			}

			bool equalsn(const string<T>& other,u32 n) const
			{
				u32 i;

				for(i=0;array[i] && other[i] && i<n;i++) {
					if(array[i]!=other[i]) return false;
				}

				return (i==n) || (used==other.used);
			}

			bool equalsn(const T* const str,u32 n) const
			{
				u32 i;

				if(!str) return false;

				for(i=0;array[i] && str[i] && i<n;i++) {
					if(array[i]!=str[i]) return false;
				}

				return (i==n) || (array[i]==0 && str[i]==0);
			}

			void append(T ch)
			{
				if(used+1>allocated) reallocate(used+1);

				used++;
				array[used-2] = ch;
				array[used-1] = 0;
			}

			void append(const T* const str)
			{
				u32 i,len = 0;

				if(!str) return;
				
				const T *p = str;
				while(*p) {
					len++;
					p++;
				}

				if((used+len)>allocated) reallocate(used+len);

				used--;
				len++;

				for(i=0;i<len;i++) array[used+i] = *(str+i);

				used += len;
			}

			void append(const string<T>& other)
			{
				u32 i,len;

				used--;
				len = other.size()+1;

				if((used+len)>allocated) reallocate(used+len);

				for(i=0;i<len;i++) array[used+i] = other.array[i];

				used += len;
			}

			string<T, TAlloc>& validate()
			{
				for(u32 i=0;i < allocated;i++) {
					if(array[i] == 0) {
						used = i + 1;
						return *this;
					}
				}

				if(allocated > 0) {
					used = allocated;
					array[used - 1] = 0;
				} else
					used = 0;

				return *this;
			}

			bool operator==(const string<T>& other) const
			{
				u32 i;
				for(i=0;array[i] && other.array[i];i++) {
					if(array[i]!=other.array[i]) return false;
				}
				return used==other.used;
			}

			bool operator<(const string<T>& other) const
			{
				u32 i;
				for(i=0;array[i] && other.array[i];i++) {
					s32 diff = array[i] - other.array[i];
					if(diff) return diff<0;
				}
				return used<other.used;
			}

			string<T>& operator+=(T c)
			{
				append(c);
				return *this;
			}

			string<T>& operator+=(const T* const c)
			{
				append(c);
				return *this;
			}

			string<T>& operator+=(const string<T>& other)
			{
				append(other);
				return *this;
			}

			string<T>& operator+=(const int i)
			{
				append(string<T>(i));
				return *this;
			}

			string<T>& operator+=(const unsigned int i)
			{
				append(string<T>(i));
				return *this;
			}

			string<T>& operator+=(const float i)
			{
				append(string<T>(i));
				return *this;
			}

		private:
			inline T ansi_lower(u32 x) const
			{
				return x>='A' && x<='Z'?(T)x+0x20:(T)x;
			}

			T *array;
			u32 allocated;
			u32 used;
			TAlloc allocator;
		};

		typedef string<char> stringc;
	}
}

#endif
