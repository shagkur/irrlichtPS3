#ifndef __IRRLIST_H__
#define __IRRLIST_H__

#include "irrtypes.h"

namespace irr
{
	namespace core
	{
		template<class T>
		class list
		{
		private:
			struct SKListNode
			{
				SKListNode() : prev(NULL),next(NULL) {};

				SKListNode *prev;
				SKListNode *next;
				T elem;
			};

		public:
			class ConstIterator;

			class Iterator
			{
			public:
				Iterator() : curr(NULL) {};

				Iterator& operator++() { curr = curr->next; return *this; }
				Iterator& operator--() { curr = curr->prev; return *this; }

				Iterator operator++(s32) { Iterator tmp = *this; curr = curr->next; return tmp; }
				Iterator operator--(s32) { Iterator tmp = *this; curr = curr->prev; return tmp; }

				Iterator& operator+=(s32 num)
				{
					if(num>0) {
						while(num-- && this->curr!=NULL) ++(*this);
					} else {
						while(num++ && this->curr!=NULL) --(*this);
					}
					return *this;
				}

				Iterator operator+(s32 num) const { Iterator tmp = *this; return tmp += num; }
				Iterator operator-(s32 num) const { return (*this)+(-num); }
				Iterator& operator-=(s32 num) const { return (*this)+=(-num); }

				bool operator==(const Iterator& other) const { return (curr==other.curr); }
				bool operator!=(const Iterator& other) const { return (curr!=other.curr); }
				bool operator==(const ConstIterator& other) const { return (curr==other.curr); }
				bool operator!=(const ConstIterator& other) const { return (curr!=other.curr); }

				T& operator*() { return curr->elem; }
				T* operator->() { return &curr->elem; }

			private:
				Iterator(SKListNode *begin) : curr(begin) {};

				SKListNode *curr;

				friend class list<T>;
			};

			class ConstIterator
			{
			public:
				ConstIterator() : curr(NULL) {};

				ConstIterator& operator++() { curr = curr->next; return *this; }
				ConstIterator& operator--() { curr = curr->prev; return *this; }

				ConstIterator operator++(s32) { ConstIterator tmp = *this; curr = curr->next; return tmp; }
				ConstIterator operator--(s32) { ConstIterator tmp = *this; curr = curr->prev; return tmp; }

				ConstIterator& operator+=(s32 num)
				{
					if(num>0) {
						while(num-- && this->curr!=NULL) ++(*this);
					} else {
						while(num++ && this->curr!=NULL) --(*this);
					}
					return *this;
				}

				ConstIterator operator+(s32 num) const { ConstIterator tmp = *this; return tmp += num; }
				ConstIterator operator-(s32 num) const { return (*this)+(-num); }
				ConstIterator& operator-=(s32 num) const { return (*this)+=(-num); }

				bool operator==(const ConstIterator& other) const { return (curr==other.curr); }
				bool operator!=(const ConstIterator& other) const { return (curr!=other.curr); }
				bool operator==(const Iterator& other) const { return (curr==other.curr); }
				bool operator!=(const Iterator& other) const { return (curr!=other.curr); }

				const T& operator*() { return curr->elem; }
				const T* operator->() { return &curr->elem; }

				ConstIterator& operator=(const Iterator& it) { curr = it.curr; return *this; }

			private:
				ConstIterator(SKListNode *begin) : curr(begin) {};

				SKListNode *curr;

				friend class Iterator;
				friend class list<T>;
			};

			list() : _first(NULL),_last(NULL),_size(0) {};
			list(const list<T>& other) : _first(NULL),_last(NULL),_size(0)
			{
				*this = other;
			}

			~list()
			{
				clear();
			}

			void clear()
			{
				while(_first!=NULL) {
					SKListNode *next = _first->next;
					delete _first;
					_first = next;
				}

				_first = NULL;
				_last = NULL;
			}

			void push_back(const T& elem)
			{
				SKListNode *n = new SKListNode;

				n->elem = elem;
				n->prev = _last;
				if(_first==NULL) _first = n;
				if(_last!=NULL) _last->next = n;

				_last = n;
				_size++;
			}

			void push_front(const T& elem)
			{
				SKListNode *n = new SKListNode;

				n->elem = elem;
				if(_first==NULL) {
					_last = n;
					_first = n;
				} else {
					n->next = _first;
					_first->prev = n;
					_first = n;
				}
				_size++;
			}

			Iterator begin()
			{
				return Iterator(_first);
			}

			ConstIterator begin() const
			{
				return ConstIterator(_first);
			}

			Iterator end()
			{
				return Iterator(NULL);
			}

			ConstIterator end() const
			{
				return ConstIterator(NULL);
			}

			bool empty() const
			{
				return (_first==NULL);
			}

			void operator=(const list<T>& other)
			{
				if(&other==this) return;

				clear();

				SKListNode *node = other._first;
				while(node!=NULL) {
					push_back(node->elem);
					node = node->next;
				}
			}

			u32 getSize() const
			{
				return _size;
			}

			Iterator getLast()
			{
				return Iterator(_last);
			}

			ConstIterator getLast() const
			{
				return ConstIterator(_last);
			}

			void insert_after(const Iterator& it,const T& elem)
			{
				SKListNode *n = new SKListNode;

				n->elem = elem;
				n->next = it.curr->next;

				if(it.curr->next)
					it.curr->next->prev = n;

				n->prev = it.curr;
				it.curr->next = n;

				if(it.curr==_last)
					_last = n;

				_size++;
			}

			void insert_before(const Iterator& it,const T& elem)
			{
				SKListNode *n = new SKListNode;

				n->elem = elem;
				n->prev = it.curr->prev;

				if(it.curr->prev)
					it.curr->prev->next = n;

				n->next = it.curr;
				it.curr->prev = n;

				if(it.curr==_first)
					_first = n;

				_size++;
			}

			Iterator erase(Iterator& it)
			{
				Iterator ret(it);

				ret++;
				if(it.curr==_first) 
					_first = it.curr->next;
				else
					it.curr->prev->next = it.curr->next;

				if(it.curr==_last)
					_last = it.curr->prev;
				else
					it.curr->next->prev = it.curr->prev;

				delete it.curr;
				it.curr = NULL;

				_size--;
				return ret;
			}

		private:
			SKListNode *_first;
			SKListNode *_last;
			u32 _size;
		};
	}
}

#endif
