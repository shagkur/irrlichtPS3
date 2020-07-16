/*
 * particleboxemitter.h
 *
 *  Created on: Aug 9, 2013
 *      Author: mike
 */

#ifndef PARTICLEBOXEMITTER_H_
#define PARTICLEBOXEMITTER_H_

#include "iparticleboxemitter.h"
#include "irrarray.h"
#include "aabbox3d.h"

namespace irr
{
	namespace scene
	{
		class CParticleBoxEmitter : public IParticleBoxEmitter
		{
		public:
			CParticleBoxEmitter(const core::aabbox3df& box, const core::vector3df& direction = core::vector3df(0.0f, 0.3f, 0.0f),
								u32 minParticlesPerSecond = 20, u32 maxParticlesPerSecond = 40,
								video::SColor minStartColor = video::SColor(0, 0, 0, 255), video::SColor maxStartColor = video::SColor(255, 255, 255, 255),
								u32 lifeTimeMin = 2000, u32 lifeTimeMax = 4000, s32 angleDegrees = 0,
								const core::dimension2df& minStartSize = core::dimension2df(5.0f, 5.0f), const core::dimension2df& maxStartSize = core::dimension2df(5.0f, 5.0f));

			virtual s32 emitt(u32 now, u32 timeSinceLastCall, SParticle*& outArray);

			virtual void setDirection(const core::vector3df& newDirection) { _direction = newDirection; }

			virtual void setMinParticlesPerSecond(u32 minPPS) { _minParticlesPerSecond = minPPS; }

			virtual void setMaxParticlesPerSecond(u32 maxPPS) { _maxParticlesPerSecond = maxPPS; }

			virtual void setMinStartColor(const video::SColor& color) { _minStartColor = color; }

			virtual void setMaxStartColor(const video::SColor& color) { _maxStartColor = color; }

			virtual void setMinStartSize(const core::dimension2df& size) { _minStartSize = size; }

			virtual void setMaxStartSize(const core::dimension2df& size) { _maxStartSize = size; }

			virtual void setMinLifeTime(u32 lifeTime) { _minLifeTime = lifeTime; }

			virtual void setMaxLifeTime(u32 lifeTime) { _maxLifeTime = lifeTime; }

			virtual void setMaxAngleDegrees(s32 degrees) { _maxAngleDegrees = degrees; }

			virtual void setBox(const core::aabbox3df& box) { _box = box; }

			virtual const core::vector3df& getDirection() const { return _direction; }

			virtual u32 getMinParticlesPerSecond() const { return _minParticlesPerSecond; }

			virtual u32 getMaxParticlesPerSecond() const { return _maxParticlesPerSecond; }

			virtual const video::SColor& getMinStartColor() const { return _minStartColor; }

			virtual const video::SColor& getMaxStartColor() const { return _maxStartColor; }

			virtual const core::dimension2df& getMinStartSize() const { return _minStartSize; }

			virtual const core::dimension2df& getMaxStartSize() const { return _maxStartSize; }

			virtual u32 getMinLifeTime() const { return _minLifeTime; }

			virtual u32 getMaxLifeTime() const { return _maxLifeTime; }

			virtual s32 getMaxAngleDegrees() const { return _maxAngleDegrees; }

			virtual const core::aabbox3df& getBox() const { return _box; }

		private:
			core::array< SParticle > _particles;
			core::aabbox3df _box;
			core::vector3df _direction;
			core::dimension2df _minStartSize, _maxStartSize;
			u32 _minParticlesPerSecond, _maxParticlesPerSecond;
			video::SColor _minStartColor, _maxStartColor;
			u32 _minLifeTime, _maxLifeTime;
			s32 _maxAngleDegrees;

			u32 _time;
			u32 _emitted;
		};
	}
}

#endif /* PARTICLEBOXEMITTER_H_ */
