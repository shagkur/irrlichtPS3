/*
 * lightscenenode.cpp
 *
 *  Created on: Feb 11, 2013
 *      Author: mike
 */

#include "lightscenenode.h"
#include "ivideodriver.h"
#include "iscenemanager.h"
#include "icamerascenenode.h"

#include "os.h"

namespace irr
{
	namespace scene
	{
		CLightSceneNode::CLightSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, video::SColorf color, f32 radius)
		: ILightSceneNode(parent, mgr, pos), _driverLightIndex(-1), _lightIsOn(true)
		{
			_lightData.diffuseColor = color;
			_lightData.specularColor = color.getInterpolated(video::SColor(255,255,255,255), 0.7f);

			setRadius(radius);
		}

		void CLightSceneNode::onRegisterSceneNode()
		{
			doLightRecalc();

			if(_isVisible) _sceneManager->registerNodeForRendering(this, ESNRP_LIGHT);

			ISceneNode::onRegisterSceneNode();
		}

		void CLightSceneNode::render()
		{
			video::IVideoDriver *driver = _sceneManager->getVideoDriver();
			if(driver == NULL) return;

			_driverLightIndex = driver->addDynamicLight(_lightData);

			setVisible(_lightIsOn);
		}

		void CLightSceneNode::setLightData(const video::SLight& light)
		{
			_lightData = light;
		}

		const video::SLight& CLightSceneNode::getLightData() const
		{
			return _lightData;
		}

		video::SLight& CLightSceneNode::getLightData()
		{
			return _lightData;
		}

		void CLightSceneNode::setVisible(bool isVisible)
		{
			ISceneNode::setVisible(isVisible);

			if(_driverLightIndex < 0) return;

			video::IVideoDriver *driver = _sceneManager->getVideoDriver();
			if(driver == NULL) return;

			_lightIsOn = isVisible;
		}

		const core::aabbox3df& CLightSceneNode::getBoundingBox() const
		{
			return _bbox;
		}

		void CLightSceneNode::setRadius(f32 radius)
		{
			_lightData.radius = radius;
			_lightData.attenuation = core::vector3df(0.0f, 1.0f/radius, 0.0f);

			doLightRecalc();
		}

		f32 CLightSceneNode::getRadius() const
		{
			return _lightData.radius;
		}

		void CLightSceneNode::setLightType(video::E_LIGHT_TYPE type)
		{
			_lightData.type = type;
		}

		video::E_LIGHT_TYPE CLightSceneNode::getLightType() const
		{
			return _lightData.type;
		}

		void CLightSceneNode::enableCastShadow(bool shadow)
		{
			_lightData.castShadows = shadow;
		}

		bool CLightSceneNode::getCastShadow() const
		{
			return _lightData.castShadows;
		}

		void CLightSceneNode::doLightRecalc()
		{
			if(_lightData.type == video::ELT_SPOT || _lightData.type == video::ELT_DIRECTIONAL) {
				core::vector3df direction(0.0f, 0.0f, -1.0f);
				core::rotateVect(getAbsoluteTransformation(), direction);
				_lightData.direction = normalize(direction);
			}
			if(_lightData.type == video::ELT_SPOT || _lightData.type == video::ELT_POINT) {
				const f32 r = _lightData.radius*_lightData.radius*0.5f;

				_bbox.maxEdge = core::vector3df(r, r, r);
				_bbox.minEdge = core::vector3df(-r, -r, -r);

				_lightData.position = getAbsolutePosition();

				setAutomaticCulling(EAC_OFF);
			}
			if(_lightData.type == video::ELT_DIRECTIONAL) {
				_bbox.reset(0, 0, 0);
				setAutomaticCulling(EAC_OFF);
			}
		}
	}
}


