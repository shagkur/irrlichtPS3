/*
 * util.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: mike
 */

#include "util.h"

namespace ParticleUtil
{
	void createParticleGroup(ParticleGroup& particleGroup, ParticleGroupProperty& property)
	{
		particleGroup.property = property;

		particleGroup.pclStates[0] = (PclState*)memalign(128, sizeof(PclState)*property.numParticles);
		particleGroup.pclStates[1] = (PclState*)memalign(128, sizeof(PclState)*property.numParticles);
		particleGroup.pclJoints = (PclJoint*)memalign(128, sizeof(PclJoint)*property.numJoints);
		particleGroup.userData = (u32*)memalign(128, sizeof(u32)*property.numParticles);

		particleGroup.initialize();
	}

	void releaseParticleGroup(ParticleGroup& particleGroup)
	{
		free(particleGroup.pclStates[0]);
		free(particleGroup.pclStates[1]);
		free(particleGroup.pclJoints);
		free(particleGroup.userData);

		free(particleGroup.pclVertices);
		free(particleGroup.pclNormals);
		free(particleGroup.pclTexCoords);
		free(particleGroup.pclIndices);
	}
}
