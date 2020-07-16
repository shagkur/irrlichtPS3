/*
 * irrmap.h
 *
 *  Created on: Feb 4, 2013
 *      Author: mike
 */

#ifndef IRRMAP_H_
#define IRRMAP_H_

#include "irrtypes.h"

namespace irr
{
	namespace core
	{
		template<class KeyType,class ValueType>
		class map
		{
			template<class KeyTypeRB,class ValueTypeRB>
			class RBTree
			{
			public:
				RBTree(const KeyTypeRB& k,const ValueTypeRB& v)
					: leftChild(NULL),rightChild(NULL),parent(NULL),key(k),value(v),_isRed(true) {};
				~RBTree() {};

				void setLeftChild(RBTree *p)
				{
					leftChild = p;
					if(p) p->setParent(this);
				}

				void setRightChild(RBTree *p)
				{
					rightChild = p;
					if(p) p->setParent(this);
				}

				void setParent(RBTree *p)
				{
					parent = p;
				}

				void setRed() { _isRed = true; }
				void setBlack() { _isRed = false; }
				void setValue(const ValueTypeRB& v) { value = v; }

				RBTree* getLeftChild() const { return leftChild; }
				RBTree* getRightChild() const { return rightChild; }
				RBTree* getParent() const { return parent; }

				ValueTypeRB getValue() const { return value; }
				KeyTypeRB getKey() const { return key; }

				ValueTypeRB& getValue() { return value; }
				KeyTypeRB& getKey() { return key; }

				bool isRoot() const { return (parent==NULL); }
				bool isLeftChild() const { return (parent!=NULL && parent->getLeftChild()==this); }
				bool isRightChild() const { return (parent!=NULL && parent->getRightChild()==this); }
				bool isLeaf() const { return (leftChild==NULL && rightChild==NULL); }
				bool isRed() const { return _isRed; }
				bool isBlack() const { return !_isRed; }

				u32 getLevel() const
				{
					if(isRoot())
						return 1;
					else
						return getParent()->getLevel()+1;
				}

			private:
				RBTree();

				RBTree *leftChild;
				RBTree *rightChild;
				RBTree *parent;

				KeyTypeRB key;
				ValueTypeRB value;

				bool _isRed;
			};

		public:
			typedef RBTree<KeyType,ValueType> Node;

			class Iterator
			{
			public:
				Iterator() : root(NULL),curr(NULL) {};

				Iterator(Node *n) : root(n),curr(NULL)
				{
					reset();
				}

				Iterator(const Iterator& src) : root(src.root), curr(src.curr) {};

				void reset(bool atLowest = true)
				{
					if(atLowest)
						curr = getMin(root);
					else
						curr = getMax(root);
				}

				bool atEnd() const { return (curr==NULL); }

				Node* getNode() { return curr; }

				Iterator& operator=(const Iterator& src)
				{
					root = src.root;
					curr = src.curr;
					return *this;
				}

				void operator++(s32)
				{
					inc();
				}

				void operator--(s32)
				{
					dec();
				}

				Node* operator->()
				{
					return getNode();
				}

				Node& operator*()
				{
					if(atEnd())
						throw "Iterator at end";
					return *getNode();
				}

			private:
				Node* getMin(Node *n)
				{
					while(n && n->getLeftChild())
						n = n->getLeftChild();
					return n;
				}

				Node* getMax(Node *n)
				{
					while(n && n->getRightChild())
						n = n->getRightChild();
					return n;
				}

				void inc()
				{
					if(curr==NULL) return;

					if(curr->getRightChild())
						curr = getMin(curr->getRightChild());
					else if(curr->isLeftChild())
						curr = curr->getParent();
					else {
						while(curr->isRightChild())
							curr = curr->getParent();
						curr = curr->getParent();
					}
				}

				void dec()
				{
					if(curr==NULL) return;

					if(curr->getLeftChild())
						curr = getMax(curr->getLeftChild());
					else if(curr->isRightChild())
						curr = curr->getParent();
					else {
						while(curr->isLeftChild())
							curr = curr->getParent();
						curr = curr->getParent();
					}
				}

				Node *root;
				Node *curr;
			};

			class ParentFirstIterator
			{
			public:
				ParentFirstIterator() : root(NULL),curr(NULL) {};
				explicit ParentFirstIterator(Node *n) : root(n),curr(NULL)
				{
					reset();
				}

				void reset()
				{
					curr = root;
				}

				bool atEnd() const { return (curr==NULL); }

				Node* getNode() { return curr; }

				ParentFirstIterator& operator=(const ParentFirstIterator& src)
				{
					root = src.root;
					curr = src.curr;
					return *this;
				}

				Node* operator->()
				{
					return getNode();
				}

				Node& operator*()
				{
					if(atEnd())
						throw "ParentFirstIterator at end";
					return *getNode();
				}

				void operator++(s32)
				{
					inc();
				}

			private:
				void inc()
				{
					if(curr==NULL) return;

					if(curr->getLeftChild())
						curr = curr->getLeftChild();
					else if(curr->getRightChild())
						curr = curr->getRightChild();
					else {
						while(curr!=NULL) {
							if(curr->isLeftChild() && curr->getParent()->getRightChild()) {
								curr = curr->getParent()->getRightChild();
								return;
							}
							curr = curr->getParent();
						}
					}

				}

				Node *root;
				Node *curr;
			};

			class ParentLastIterator
			{
			public:
				ParentLastIterator() : root(NULL),curr(NULL) {};
				explicit ParentLastIterator(Node *n) : root(n)
				{
					reset();
				}

				void reset()
				{
					curr = getMin(root);
				}

				bool atEnd() const { return (curr==NULL); }

				Node* getNode() { return curr; }

				ParentLastIterator& operator=(const ParentLastIterator& src)
				{
					root = src.root;
					curr = src.curr;
					return *this;
				}

				void operator++(int)
				{
					inc();
				}

				Node& operator*()
				{
					if(atEnd())
						throw "ParentLastIterator at end";
					return *getNode();
				}

				Node* operator->()
				{
					return getNode();
				}


			private:
				Node* getMin(Node *n)
				{
					while(n!=NULL && (n->getLeftChild()!=NULL || n->getRightChild()!=NULL)) {
						if(n->getLeftChild())
							n = n->getLeftChild();
						else
							n = n->getRightChild();
					}
					return n;
				}

				void inc()
				{
					if(curr==NULL) return;

					if(curr->isLeftChild() && curr->getParent()->getRightChild())
						curr = getMin(curr->getParent()->getRightChild());
					else
						curr = curr->getParent();
				}

				Node *root;
				Node *curr;
			};

			class AccessClass
			{
				friend class map<KeyType,ValueType>;

			public:
				void operator=(const ValueType& val)
				{
					tree.set(key,val);
				}

				operator ValueType()
				{
					Node *n = tree.find(key);

					if(n==NULL)
						return NULL;

					return n->getValue();
				}

			private:
				AccessClass();
				AccessClass(map& t,const KeyType& k) : tree(t),key(k) {};

				map& tree;
				const KeyType& key;
			};


			map() : _root(NULL),_size(0) {};
			~map()
			{
				clear();
			}

			void clear()
			{
				ParentLastIterator it(getParentLastIterator());

				while(!it.atEnd()) {
					Node *p = it.getNode();
					it++;

					delete p;
				}

				_root = NULL;
				_size = 0;
			}

			void set(const KeyType& k,const ValueType& v)
			{
				Node *p = find(k);

				if(p)
					p->setValue(v);
				else
					insert(k,v);
			}

			bool insert(const KeyType& k,const ValueType& v)
			{
				Node *n = new Node(k,v);

				if(!insert(n)) {
					delete n;
					return false;
				}

				while(!n->isRoot() && n->getParent()->isRed()) {
					if(n->getParent()->isLeftChild()) {
						Node *c = n->getParent()->getParent()->getRightChild();

						if(c!=NULL && c->isRed()) {
							n->getParent()->setBlack();
							c->setBlack();
							n->getParent()->getParent()->setRed();
							n = n->getParent()->getParent();
						} else {
							if(n->isRightChild()) {
								n = n->getParent();
								rotateLeft(n);
							}

							n->getParent()->setBlack();
							n->getParent()->getParent()->setRed();
							rotateRight(n->getParent()->getParent());
						}
					} else {
						Node *c = n->getParent()->getParent()->getLeftChild();

						if(c!=NULL && c->isRed()) {
							n->getParent()->setBlack();
							c->setBlack();
							n->getParent()->getParent()->setRed();
							n = n->getParent()->getParent();
						} else {
							if(n->isLeftChild()) {
								n = n->getParent();
								rotateRight(n);
							}

							n->getParent()->setBlack();
							n->getParent()->getParent()->setRed();
							rotateLeft(n->getParent()->getParent());
						}
					}
				}

				_root->setBlack();
				return true;
			}

			bool remove(const KeyType& k)
			{
				Node *left;
				Node *n = find(k);

				if(n==NULL) return false;

				while(n->getRightChild())
					rotateLeft(n);

				left = n->getLeftChild();
				if(n->isLeftChild())
					n->getParent()->setLeftChild(left);
				else if(n->isRightChild())
					n->getParent()->setRightChild(left);
				else
					setRoot(left);

				delete n;

				_size--;
				return true;
			}

			Node* delink(const KeyType& k)
			{
				Node *left;
				Node *n = find(k);

				if(n==NULL) return NULL;

				while(n->getRightChild())
					rotateLeft(n);

				left = n->getLeftChild();
				if(n->isLeftChild())
					n->getParent()->setLeftChild(left);
				else if(n->isRightChild())
					n->getParent()->setRightChild(left);
				else
					setRoot(left);

				_size--;
				return n;
			}

			bool isEmpty() const { return (_root==NULL); }
			u32 size() const { return _size; }
			Node* getRoot() const { return _root; }

			Iterator getIterator()
			{
				Iterator it(getRoot());
				return it;
			}

			ParentFirstIterator getParentFirstIterator()
			{
				ParentFirstIterator it(getRoot());
				return it;
			}

			ParentLastIterator getParentLastIterator()
			{
				ParentLastIterator it(getRoot());
				return it;
			}

			AccessClass operator[](const KeyType& k)
			{
				return AccessClass(*this,k);
			}

			Node* find(const KeyType& k) const
			{
				Node *n = _root;

				while(n!=NULL) {
					KeyType& key = n->getKey();
					if(k==key)
						return n;
					else if(k<key)
						n = n->getLeftChild();
					else
						n = n->getRightChild();
				}
				return NULL;
			}

		private:
			explicit map(const map& src);
			map& operator=(const map& src);

			bool insert(Node *n)
			{
				bool res = true;

				if(_root==NULL) {
					setRoot(n);
					_size = 1;
				} else {
					Node *p = _root;
					KeyType nk = n->getKey();
					while(p!=NULL) {
						KeyType k(p->getKey());

						if(nk==k) {
							res = false;
							p = NULL;
						} else if(nk<k) {
							if(p->getLeftChild()==NULL) {
								p->setLeftChild(n);
								p = NULL;
							} else
								p = p->getLeftChild();
						} else {
							if(p->getRightChild()==NULL) {
								p->setRightChild(n);
								p = NULL;
							} else
								p = p->getRightChild();
						}
					}
				}
				if(res) _size++;

				return res;
			}

			void setRoot(Node *n)
			{
				_root = n;
				if(_root!=NULL) _root->setParent(NULL);
			}

			void rotateLeft(Node *n)
			{
				Node *right = n->getRightChild();

				n->setRightChild(right->getLeftChild());
				if(n->isLeftChild())
					n->getParent()->setLeftChild(right);
				else if(n->isRightChild())
					n->getParent()->setRightChild(right);
				else
					setRoot(right);

				right->setLeftChild(n);
			}

			void rotateRight(Node *n)
			{
				Node *left = n->getLeftChild();

				n->setLeftChild(left->getRightChild());
				if(n->isLeftChild())
					n->getParent()->setLeftChild(left);
				else if(n->isRightChild())
					n->getParent()->setRightChild(left);
				else
					setRoot(left);

				left->setRightChild(n);
			}

			Node *_root;
			u32 _size;
		};
	}
}
#endif /* IRRMAP_H_ */
