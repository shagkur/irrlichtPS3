/*
 * particlecylinderemitter.h
 *
 *  Created on: Aug 10, 2013
 *      Author: mike
 */

#ifndef PARTICLECYLINDEREMITTER_H_
#define PARTICLECYLINDEREMITTER_H_

#include "iparticlecylinderemitter.h"
#include "irrarray.h"


namespace irr
{
	namespace scene
	{
		class CParticleCylinderEmitter : public IParticleCylinderEmitter
		{
		public:
			CParticleCylinderEmitter(const core::vector3df& center, f32 radius,
									 const core::vector3df& normal, f32 length,
									 bool outlineOnly = false, const core::vector3df& direction = core::vector3df(0.0f, 0.3f, 0.0f),
									 u32 minParticlesPerSecond = 20, u32 maxParticlesPerSecond = 40,
									 const video::SColor& minStartColor = video::SColor(0, 0, 0, 255),
									 const video::SColor& maxStartColor = video::SColor(255, 255, 255, 255),
									 u32 lifeTimeMin = 2000, u32 lifeTimeMax = 4000, s32 maxAngleDegrees = 0,
									 const core::dimension2df& minStartSize = core::dimension2df(5.0f, 5.0f),
									 const core::dimension2df& maxStartSize = core::dimension2df(5.0f, 5.0f));

			virtual s32 emitt(u32 now, u32 timeSinceLastCall, SParticle*& outArray);

			virtual void setCenter(const core::vector3df& center) { _center = center; }

			virtual void setNormal(const core::vector3df& normal) { _normal = normal; }

			virtual void setRadius(f32 radius) { _radius = radius; }

			virtual void setLength(f32 length) { _length = length; }

			virtual void setOutlineOnly(bool outlineOnly) { _outlineOnly = outlineOnly; }

			virtual void setDirection(const core::vector3df& newDirection) { _direction = newDirection; }

			virtual void setMinParticlesPerSecond(u32 minPPS) { _minParticlesPerSecond = minPPS; }

			virtual void setMaxParticlesPerSecond(u32 maxPPS) { _maxParticlesPerSecond = maxPPS; }

			virtual void setMinStartColor(const video::SColor& color) { _minStartColor = color; }

			virtual void setMaxStartColor(const video::SColor& color) { _maxStartColor = color; }

			virtual void setMinStartSize(const core::dimension2df& size) { _minStartSize = size; }

			virtual void setMaxStartSize(const core::dimension2df& size) { _maxStartSize = size; }

			virtual void setMinLifeTime(u32 lifeTime) { _minLifeTime = lifeTime; }

			virtual void setMaxLifeTime(u32 lifeTime) { _maxLifeTime = lifeTime; }

			virtual void setMaxAngleDegrees(s32 maxAngleDegrees) { _maxAngleDegrees = maxAngleDegrees; }

			virtual const core::vector3df& getCenter() const { return _center; }

			virtual const core::vector3df& getNormal() const { return _normal; }

			virtual f32 getRadius() const { return _radius; }

			virtual f32 getLength() const { return _length; }

			virtual bool getOutlineOnly() const { return _outlineOnly; }

			virtual const core::vector3df& getDirection() const { return _direction; }

			virtual u32 getMinParticlesPerSecond() const { return _minParticlesPerSecond; }

			virtual u32 getMaxParticlesPerSecond() const { return _maxParticlesPerSecond; }

			virtual void setMinStartColor(const video::SColor& color) { _minStartColor = color; }

			virtual void setMaxStartColor(const video::SColor& color) { _maxStartColor = color; }

			virtual void setMinStartSize(const core::dimension2df& size) { _minStartSize = size; }

			virtual void setMaxStartSize(const core::dimension2df& size) { _maxStartSize = size; }

			virtual void setMinLifeTime(u32 lifeTime) { _minLifeTime = lifeTime; }

			virtual void setMaxLifeTime(u32 lifeTime) { _maxLifeTime = lifeTime; }

			virtual void setMaxAngleDegrees(s32 degrees) { _maxAngleDegrees = degrees; }

		private:
			core::array< SParticle > _particles;
			core::vector3df _center;
			core::vector3df _normal;
			core::vector3df _direction;
			core::dimension2df _minStartSize, _maxStartSize;
			u32 _minParticlesPerSecond, _maxParticlesPerSecond;
			video::SColor _minStartColor, _maxStartColor;
			u32 _minLifeTime, _maxLifeTime;
			u32 _maxAngleDegrees;

			f32 _radius;
			f32 _length;
			bool _outlineOnly;

			u32 _time;
			u32 _emitted;
		};
	}
}

#endif /* PARTICLECYLINDEREMITTER_H_ */
