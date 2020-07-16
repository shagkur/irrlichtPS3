/*
 * particleboxemitter.cpp
 *
 *  Created on: Aug 9, 2013
 *      Author: mike
 */

#include "particleboxemitter.h"
#include "os.h"
#include "irrmath.h"

namespace irr
{
	namespace scene
	{
		CParticleBoxEmitter::CParticleBoxEmitter(const core::aabbox3df& box, const core::vector3df& direction,
												 u32 minParticlesPerSecond, u32 maxParticlesPerSecond,
												 video::SColor minStartColor, video::SColor maxStartColor,
												 u32 lifeTimeMin, u32 lifeTimeMax, s32 angleDegrees,
												 const core::dimension2df& minStartSize, const core::dimension2df& maxStartSize)
		: _box(box), _direction(direction),
		  _minStartSize(minStartSize), _maxStartSize(maxStartSize),
		  _minParticlesPerSecond(minParticlesPerSecond), _maxParticlesPerSecond(maxParticlesPerSecond),
		  _minStartColor(minStartColor), _maxStartColor(maxStartColor),
		  _minLifeTime(lifeTimeMin), _maxLifeTime(lifeTimeMax),
		  _maxAngleDegrees(angleDegrees)
		{

		}

		s32 CParticleBoxEmitter::emitt(u32 now, u32 timeSinceLastCall, SParticle*& outArray)
		{
			_time += timeSinceLastCall;

			const u32 pps = (_maxParticlesPerSecond - _minParticlesPerSecond);
			const f32 perSecond = pps ? ((f32)_minParticlesPerSecond + os::Randomizer::frand()*pps) : _minParticlesPerSecond;
			const f32 everyWhatMs = 1000.0f/perSecond;

			if(_time > everyWhatMs) {
				SParticle p;
				u32 amount = (u32)((_time/everyWhatMs) + 0.5f);
				const core::vector3df& extent = _box.getExtent();

				_time = 0;
				_particles.set_used(0);

				if(amount > _maxParticlesPerSecond*2)
					amount = _maxParticlesPerSecond*2;

				for(u32 i=0;i < amount;i++) {
					p.pos.setX(_box.minEdge.getX() + os::Randomizer::frand()*extent.getX());
					p.pos.setY(_box.minEdge.getY() + os::Randomizer::frand()*extent.getY());
					p.pos.setZ(_box.minEdge.getZ() + os::Randomizer::frand()*extent.getZ());

					p.startTime = now;
					p.vector = _direction;

					if(_maxAngleDegrees) {
						core::vector3df tgt = _direction;
					}

					p.endTime = now + _minLifeTime;
					if(_maxLifeTime != _minLifeTime)
						p.endTime += os::Randomizer::rand()%(_maxLifeTime - _minLifeTime);

					if(_minStartColor == _maxStartColor)
						p.color = _minStartColor;
					else
						p.color = _minStartColor.getInterpolated(_maxStartColor, os::Randomizer::frand());

					p.startColor = p.color;
					p.startVector = p.vector;

					if(_minStartSize == _maxStartSize)
						p.startSize = _minStartSize;
					else
						p.startSize = _minStartSize.getInterpolated(_maxStartSize, os::Randomizer::frand());

					p.size = p.startSize;

					_particles.push_back(p);
				}

				outArray = _particles.pointer();
				return _particles.size();
			}

			return 0;
		}
	}
}
