/*
 * iscenenode.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef ISCENENODE_H_
#define ISCENENODE_H_

#include "irrtypes.h"
#include "irrarray.h"
#include "irrlist.h"
#include "irefcounter.h"
#include "vector3d.h"
#include "matrix4.h"
#include "aabbox3d.h"
#include "triangle3d.h"
#include "smaterial.h"
#include "ecullingtypes.h"
#include "escenenodetypes.h"
#include "iscenenodeanimator.h"

namespace irr
{
	namespace scene
	{
		class ISceneManager;

		typedef core::list< ISceneNode* > ISceneNodeList;
		typedef core::list< ISceneNodeAnimator* > ISceneNodeAnimatorList;

		class ISceneNode : public virtual IRefCounter
		{
		public:
			ISceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos = core::vector3df(0.0f, 0.0f, 0.0f), const core::vector3df& rot = core::vector3df(0.0f, 0.0f, 0.0f), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f))
			: _isVisible(true), _sceneManager(mgr), _relTranslation(pos), _relRotation(rot), _relScale(scale), _parent(parent), _automaticCullingState(EAC_BOX)
			{
				if(parent) parent->addChild(this);

				updateAbsolutePosition();
			}

			virtual ~ISceneNode()
			{
				removeAll();

				ISceneNodeAnimatorList::Iterator ait = _animators.begin();
				for(;ait!=_animators.end();ait++) (*ait)->drop();
			}

			virtual void render() = 0;

			virtual void addChild(ISceneNode *child)
			{
				if(child && child!=this) {
					child->grab();
					child->remove();
					_children.push_back(child);
					child->_parent = this;
				}
			}

			virtual bool removeChild(ISceneNode *child)
			{
				ISceneNodeList::Iterator it = _children.begin();
				for(;it!=_children.end();it++) {
					if((*it)==child) {
						(*it)->_parent = NULL;
						(*it)->drop();
						_children.erase(it);
						return true;
					}
				}
				return false;
			}

			virtual void removeAll()
			{
				ISceneNodeList::Iterator it = _children.begin();
				for(;it!=_children.end();it++) {
					(*it)->_parent = NULL;
					(*it)->drop();
				}
				_children.clear();
			}

			virtual void remove()
			{
				if(_parent) _parent->removeChild(this);
			}

			virtual void onRegisterSceneNode()
			{
				if(_isVisible) {
					ISceneNodeList::Iterator it = _children.begin();
					for(;it!=_children.end();it++) (*it)->onRegisterSceneNode();
				}
			}

			virtual void addAnimator(ISceneNodeAnimator *anim)
			{
				if(anim != NULL) {
					_animators.push_back(anim);
					anim->grab();
				}
			}

			virtual void removeAnimator(ISceneNodeAnimator* anim)
			{
				ISceneNodeAnimatorList::Iterator it = _animators.begin();
				for(;it!=_animators.end();it++) {
					if((*it) == anim) {
						(*it)->drop();
						_animators.erase(it);
						return;
					}
				}
			}

			virtual void removeAnimators()
			{
				ISceneNodeAnimatorList::Iterator it = _animators.begin();
				for(;it!=_animators.end();it++)
					(*it)->drop();

				_animators.clear();
			}

			const ISceneNodeAnimatorList& getAnimators() const
			{
				return _animators;
			}

			virtual void onAnimate(u32 timeMs)
			{
				if(_isVisible) {
					ISceneNodeAnimatorList::Iterator ait = _animators.begin();
					while(ait!=_animators.end()) {
						ISceneNodeAnimator *anim = *ait;
						++ait;
						anim->animateNode(this,timeMs);
					}

					updateAbsolutePosition();

					ISceneNodeList::Iterator it = _children.begin();
					for(;it!=_children.end();it++) (*it)->onAnimate(timeMs);
				}
			}

			virtual const core::vector3df& getPosition() const
			{
				return _relTranslation;
			}

			virtual void setPosition(const core::vector3df& pos)
			{
				_relTranslation = pos;
			}

			virtual const core::vector3df& getRotation() const
			{
				return _relRotation;
			}

			virtual void setRotation(const core::vector3df& rot)
			{
				_relRotation = rot;
			}

			virtual const core::vector3df& getScale() const
			{
				return _relScale;
			}

			virtual void setScale(const core::vector3df& scale)
			{
				_relScale = scale;
			}

			void setAutomaticCulling(u32 state)
			{
				_automaticCullingState = state;
			}

			u32 getAutomaticCulling() const
			{
				return _automaticCullingState;
			}

			virtual core::vector3df getAbsolutePosition() const
			{
				return _absTransformation.getTranslation();
			}

			virtual void updateAbsolutePosition()
			{
				if(_parent)
					_absTransformation = _parent->getAbsoluteTransformation()*getRelativeTransformation();
				else
					_absTransformation = getRelativeTransformation();
			}

			virtual const core::matrix4& getAbsoluteTransformation() const
			{
				return _absTransformation;
			}

			virtual const core::matrix4 getRelativeTransformation() const
			{
				core::matrix4 mat = core::matrix4::rotationZYX(_relRotation*core::DEGTORAD);

				mat.setTranslation(_relTranslation);

				if(_relScale != core::vector3df(1.0f, 1.0f, 1.0f)) {
					core::matrix4 smat = core::matrix4::scale(_relScale);
					mat *= smat;
				}

				return mat;
			}

			virtual const core::aabbox3df& getBoundingBox() const = 0;

			virtual const core::aabbox3df getTransformedBoundingBox() const
			{
				core::aabbox3df box = getBoundingBox();
				core::transformBoxEx(_absTransformation, box);
				return box;
			}

			virtual void getTransformedBoundingBoxEdges(core::array< core::vector3df >& edges) const
			{
				edges.set_used(8);
				getBoundingBox().getEdges(edges.pointer());
				for(u32 i=0;i < 8;i++)
					core::transformVect(_absTransformation, edges[i]);
			}

			virtual void setVisible(bool visible)
			{
				_isVisible = visible;
			}

			virtual bool isVisible() const
			{
				return _isVisible;
			}

			virtual bool isTrulyVisible() const
			{
				if(!_isVisible) return false;
				if(_parent == NULL) return true;

				return _parent->isTrulyVisible();
			}

			virtual video::SMaterial& getMaterial(u32 num)
			{
				return video::identityMaterial;
			}

			virtual u32 getMaterialCount() const
			{
				return 0;
			}

			void setMaterialFlag(video::E_MATERIAL_FLAG flag, bool newvalue)
			{
				for(u32 i=0;i < getMaterialCount();i++)
					getMaterial(i).setFlag(flag, newvalue);
			}

			void setMaterialTexture(u32 textureLayer, video::ITexture *texture)
			{
				if(textureLayer >= video::MATERIAL_MAX_TEXTURES)
					return;

				for(u32 i=0;i < getMaterialCount();i++) {
					getMaterial(i).setTexture(textureLayer, texture);
				}
			}

			void setMaterialType(video::E_MATERIAL_TYPE type)
			{
				for(u32 i=0;i < getMaterialCount();i++)
					getMaterial(i).materialType = type;
			}

			virtual ISceneNode* getParent() const
			{
				return _parent;
			}

			virtual void setParent(ISceneNode *parent)
			{
				grab();
				remove();

				_parent = parent;
				if(_parent) _parent->addChild(this);

				drop();
			}

			virtual ESCENE_NODE_TYPE getType() const
			{
				return ESNT_UNKNOWN;
			}

			virtual ISceneManager* getSceneManager() const { return _sceneManager; }

		protected:
			void setSceneManager(ISceneManager *mgr)
			{
				_sceneManager = mgr;

				ISceneNodeList::Iterator it = _children.begin();
				for(;it!=_children.end();it++)
					(*it)->setSceneManager(mgr);
			}

			bool _isVisible;
			ISceneManager *_sceneManager;
			core::matrix4 _absTransformation;
			core::vector3df _relTranslation;
			core::vector3df _relRotation;
			core::vector3df _relScale;
			ISceneNode *_parent;
			ISceneNodeList _children;
			ISceneNodeAnimatorList _animators;
			u32 _automaticCullingState;
		};
	}
}

#endif /* ISCENENODE_H_ */
