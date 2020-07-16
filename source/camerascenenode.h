/*
 * camerascenenode.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef CAMERASCENENODE_H_
#define CAMERASCENENODE_H_

#include "icamerascenenode.h"
#include "sviewfrustum.h"

namespace irr
{
	namespace scene
	{
		class CCameraSceneNode : public ICameraSceneNode
		{
		public:
			CCameraSceneNode(ISceneNode *parent,ISceneManager *mgr,const core::vector3df& pos = core::vector3df(0,0,0),const core::vector3df& lookAt = core::vector3df(0,0,100));
			virtual ~CCameraSceneNode();

			virtual void render();

			virtual void onRegisterSceneNode();

			virtual core::vector3df getAbsolutePosition() const;

			virtual const core::matrix4& getViewMatrix() const;
			virtual const core::matrix4& getProjectionMatrix() const;
			virtual core::vector3df getTarget() const;
			virtual core::vector3df getUpVector() const;
			virtual f32 getNearValue() const;
			virtual f32 getFarValue() const;
			virtual f32 getAspectRatio() const;
			virtual f32 getFOV() const;

			virtual void setProjectionMatrix(const core::matrix4& mat,bool isOrthogonal = false);
			virtual void setTarget(const core::vector3df& pos);
			virtual void setUpVector(const core::vector3df& up);
			virtual void setNearValue(f32 near);
			virtual void setFarValue(f32 far);
			virtual void setAspect(f32 aspect);
			virtual void setFOV(f32 fovy);

			virtual const SViewFrustum* getViewFrustum() const;

			virtual const core::aabbox3df& getBoundingBox() const;

			virtual ESCENE_NODE_TYPE getType() const { return ESNT_CAMERA; }

			virtual bool onEvent(const SEvent& event);

			virtual void setInputReceiverEnabled(bool enabled);
			virtual bool isInputReceiverEnabled() const;

			virtual void bindTargetAndRotation(bool bound);
			virtual bool getTargetAndRotationBinding() const;

		protected:
			void recalcProjectionMatrix();
			void recalcViewArea();

			core::vector3df _target;
			core::vector3df _upVector;

			f32 _fovy;
			f32 _aspect;
			f32 _znear;
			f32 _zfar;

			SViewFrustum _viewArea;

			bool _inputReceiverEnabled;
			bool _targetAndRotationBound;
		};
	}
}

#endif /* CAMERASCENENODE_H_ */
