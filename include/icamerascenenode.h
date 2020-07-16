/*
 * icamerascenenode.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef ICAMERASCENENODE_H_
#define ICAMERASCENENODE_H_

#include "iscenenode.h"
#include "ieventreceiver.h"

namespace irr
{
	namespace scene
	{
		struct SViewFrustum;

		class ICameraSceneNode : public ISceneNode, public IEventReceiver
		{
		public:
			ICameraSceneNode(ISceneNode *parent,ISceneManager *mgr,const core::vector3df& pos = core::vector3df(0,0,0),const core::vector3df& rot = core::vector3df(0,0,0),const core::vector3df& scale = core::vector3df(1.0f,1.0f,1.0f))
				: ISceneNode(parent,mgr,pos,rot,scale),_isOrthogonal(false)
			{
			}

			virtual ~ICameraSceneNode() {};

			virtual const core::matrix4& getViewMatrix() const = 0;
			virtual const core::matrix4& getProjectionMatrix() const = 0;
			virtual core::vector3df getTarget() const = 0;
			virtual core::vector3df getUpVector() const = 0;
			virtual f32 getNearValue() const = 0;
			virtual f32 getFarValue() const = 0;
			virtual f32 getAspectRatio() const = 0;
			virtual f32 getFOV() const = 0;

			virtual void setProjectionMatrix(const core::matrix4& mat,bool isOrthogonal = false) = 0;
			virtual void setTarget(const core::vector3df& pos) = 0;
			virtual void setUpVector(const core::vector3df& up) = 0;
			virtual void setNearValue(f32 near) = 0;
			virtual void setFarValue(f32 far) = 0;
			virtual void setAspect(f32 aspect) = 0;
			virtual void setFOV(f32 fovy) = 0;

			virtual const SViewFrustum* getViewFrustum() const = 0;

			virtual bool isOrthogonal() const
			{
				return _isOrthogonal;
			}

			virtual void setIsOrthogonal(bool ortho)
			{
				_isOrthogonal = ortho;
			}

			virtual bool onEvent(const SEvent& event) = 0;

			virtual void setInputReceiverEnabled(bool enabled) = 0;
			virtual bool isInputReceiverEnabled() const = 0;

			virtual void bindTargetAndRotation(bool bound) = 0;
			virtual bool getTargetAndRotationBinding() const = 0;

		protected:
			bool _isOrthogonal;
		};
	}
}

#endif /* ICAMERASCENENODE_H_ */
