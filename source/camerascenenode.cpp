/*
 * camerascenenode.cpp
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#include "camerascenenode.h"
#include "iscenemanager.h"
#include "ivideodriver.h"
#include "os.h"

namespace irr
{
	namespace scene
	{
		CCameraSceneNode::CCameraSceneNode(ISceneNode *parent,ISceneManager *mgr,const core::vector3df& pos,const core::vector3df& lookAt)
			: ICameraSceneNode(parent,mgr,pos),_target(lookAt),_upVector(0.0f,1.0f,0.0f),_znear(1.0f),_zfar(3000.0f),
			  _inputReceiverEnabled(true), _targetAndRotationBound(false)
		{
			_fovy = 45.0f*core::DEGTORAD; //core::PI/2.5f;
			_aspect = 4.0f/3.0f;

			recalcProjectionMatrix();
			recalcViewArea();
		}

		CCameraSceneNode::~CCameraSceneNode()
		{
		}

		void CCameraSceneNode::render()
		{
			core::vector3df pos = getAbsolutePosition();
			core::vector3df tgtv = normalize(_target - pos);
			core::vector3df up = normalize(_upVector);

			f32 dp = dot(tgtv, up);
			if(fabsf(dp)==1.0f) up.setX(up.getX() + 0.5f);

			_viewArea.getTransform(video::ETS_VIEW) = core::buildLookAtMatrix(pos,_target,up);
			recalcViewArea();

			video::IVideoDriver *driver = _sceneManager->getVideoDriver();
			if(driver != NULL) {
				driver->setTransform(video::ETS_PROJECTION, _viewArea.getTransform(video::ETS_PROJECTION));
				driver->setTransform(video::ETS_VIEW, _viewArea.getTransform(video::ETS_VIEW));
			}
		}

		void CCameraSceneNode::onRegisterSceneNode()
		{
			if(_sceneManager->getActiveCamera()==this)
				_sceneManager->registerNodeForRendering(this,ESNRP_CAMERA);

			ISceneNode::onRegisterSceneNode();
		}

		core::vector3df CCameraSceneNode::getAbsolutePosition() const
		{
			return _absTransformation.getTranslation();
		}

		const core::aabbox3df& CCameraSceneNode::getBoundingBox() const
		{
			return _viewArea.getBoundingBox();
		}

		const core::matrix4& CCameraSceneNode::getViewMatrix() const
		{
			return _viewArea.getTransform(video::ETS_VIEW);
		}

		const core::matrix4& CCameraSceneNode::getProjectionMatrix() const
		{
			return _viewArea.getTransform(video::ETS_PROJECTION);
		}

		core::vector3df CCameraSceneNode::getTarget() const
		{
			return _target;
		}

		core::vector3df CCameraSceneNode::getUpVector() const
		{
			return _upVector;
		}

		f32 CCameraSceneNode::getNearValue() const
		{
			return _znear;
		}

		f32 CCameraSceneNode::getFarValue() const
		{
			return _zfar;
		}

		f32 CCameraSceneNode::getAspectRatio() const
		{
			return _aspect;
		}

		f32 CCameraSceneNode::getFOV() const
		{
			return _fovy;
		}

		void CCameraSceneNode::setTarget(const core::vector3df& pos)
		{
			_target = pos;

			if(_targetAndRotationBound) {
				core::vector3df result;
				const core::vector3df toTarget = _target - getAbsolutePosition();

				core::horizontalAngle(toTarget, result);
				ISceneNode::setRotation(result);
			}
		}

		void CCameraSceneNode::setUpVector(const core::vector3df& up)
		{
			_upVector = up;
		}

		void CCameraSceneNode::setNearValue(f32 near)
		{
			_znear = near;
			recalcProjectionMatrix();
		}

		void CCameraSceneNode::setFarValue(f32 far)
		{
			_zfar = far;
			recalcProjectionMatrix();
		}

		void CCameraSceneNode::setAspect(f32 aspect)
		{
			_aspect = aspect;
			recalcProjectionMatrix();
		}

		void CCameraSceneNode::setFOV(f32 fovy)
		{
			_fovy = fovy;
			recalcProjectionMatrix();
		}

		const SViewFrustum* CCameraSceneNode::getViewFrustum() const
		{
			return &_viewArea;
		}

		void CCameraSceneNode::setProjectionMatrix(const core::matrix4& mat,bool isOrthogonal)
		{
			_isOrthogonal = isOrthogonal;
			_viewArea.getTransform(video::ETS_PROJECTION) = mat;
		}

		void CCameraSceneNode::recalcProjectionMatrix()
		{
			_viewArea.getTransform(video::ETS_PROJECTION) = core::buildPerspectiveProjectionMatrix(_fovy,_aspect,_znear,_zfar);
		}

		void CCameraSceneNode::recalcViewArea()
		{
			core::matrix4 m;

			_viewArea.camPos = getAbsolutePosition();
			m = _viewArea.getTransform(video::ETS_PROJECTION)*_viewArea.getTransform(video::ETS_VIEW);
			_viewArea.setFrom(m);
		}

		bool CCameraSceneNode::onEvent(const SEvent& event)
		{
			if(!_inputReceiverEnabled)
				return false;

			ISceneNodeAnimatorList::Iterator ait = _animators.begin();
			for(;ait != _animators.end();++ait) {
				if((*ait)->isEventReceiverEnabled() && (*ait)->onEvent(event))
					return true;
			}

			return false;
		}

		void CCameraSceneNode::setInputReceiverEnabled(bool enabled)
		{
			_inputReceiverEnabled = enabled;
		}

		bool CCameraSceneNode::isInputReceiverEnabled() const
		{
			return _inputReceiverEnabled;
		}

		void CCameraSceneNode::bindTargetAndRotation(bool bound)
		{
			_targetAndRotationBound = bound;
		}

		bool CCameraSceneNode::getTargetAndRotationBinding() const
		{
			return _targetAndRotationBound;
		}
	}
}



