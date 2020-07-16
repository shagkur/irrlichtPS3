/*
 * particlegroup.h
 *
 *  Created on: Jan 27, 2014
 *      Author: mike
 */

#ifndef PARTICLEGROUP_H_
#define PARTICLEGROUP_H_

#include "base/common.h"

#include "particle/common/particleconfig.h"

enum ParticleType {
	ParticleTypeGroup,
	ParticleTypeCloth,
	ParticleTypeSoftBody
};

struct ParticleProperty
{
	bool active;
	u32 userData;
	f32 mass;
	f32 restitution;
	f32 friction;
	f32 radius;
	Vector3 position;
	Vector3 velocity;

	ParticleProperty()
	{
		active = true;
		mass = 0.2f;
		restitution = 0.2f;
		friction = 0.0f;
		radius = 0.2f;
		position = velocity = Vector3(0.0f);
		userData = 0;
	}
};

struct ParticleJointProperty
{
	bool active;
	u16 type;
	u16 particleA;
	f32 length;
	f32 bias;
#ifdef PARTICLE_VELOCITY_BASE
	f32 damping
#endif

	union {
		u16 particleB;
		u16 rigidBodyB;
	};

	Vector3 localPosB;

	ParticleJointProperty()
	{
		active = true;
		type = PclJointTypePcl;
		length = 1.0f;
		bias = 0.2f;
#ifdef PARTICLE_VELOCITY_BASE
		damping = 0.2f;
#endif
	}
};

struct ParticleGroupProperty
{
	ParticleType particleType;

	u32	numParticles;
	u32	numJoints;
	u8 contactIteration;
	u8 jointIteration;
	u32	contactFilterSelf;
	u32	contactFilterTarget;
	f32 maxLinearVelocity;
	f32	linearDamping;
	f32	separateBias;
	Vector3	extraForce;
	Vector3	gravity;
	bool selfCollisionEnable;
	bool twoWayInteraction;

	ParticleGroupProperty()
	{
		particleType = ParticleTypeGroup;
		numParticles = 0;
		numJoints = 0;
		contactIteration = 1;
		jointIteration = 3;
		contactFilterSelf = 0xffffffff;
		contactFilterTarget = 0xffffffff;
		maxLinearVelocity = 500.0f;
		linearDamping = 1.0f;
		separateBias = 0.2f;
		extraForce = Vector3(0.0f);
		gravity = Vector3(0.0f, -9.8f, 0.0f);
		selfCollisionEnable = false;
		twoWayInteraction = false;
	}
};

ATTRIBUTE_ALIGNED16(class) ParticleGroup
{
	friend class Particles;

public:
	ParticleGroupProperty property;

	u8 writeBuffer;
	u8 readBuffer;

	// -------------------------------------------------------
	// Particles and Joints
	ATTRIBUTE_ALIGNED16(PclState *pclStates[2]);
	ATTRIBUTE_ALIGNED16(PclJoint *pclJoints);
	u32 *userData;

	// -------------------------------------------------------
	// Mesh

	u32	numVertices;
	u32	numIndices;
	ATTRIBUTE_ALIGNED16(Vector3	*pclVertices);
	ATTRIBUTE_ALIGNED16(Vector3	*pclNormals);
	ATTRIBUTE_ALIGNED16(f32	*pclTexCoords);
	ATTRIBUTE_ALIGNED16(u16	*pclIndices);

public:
	ParticleGroup()
	{
		numVertices = 0;
		numIndices = 0;
		pclVertices = NULL;
		pclNormals = NULL;
		pclTexCoords = NULL;
		pclIndices = NULL;
	}

	~ParticleGroup()
	{
	}

	void initialize()
	{
		writeBuffer = 0;
		readBuffer = 1;
		memset(pclStates[0], 0, sizeof(PclState)*property.numParticles);
		memset(pclStates[1], 0, sizeof(PclState)*property.numParticles);
		memset(pclJoints, 0, sizeof(PclJoint)*property.numJoints);
	}

	inline void initParticle(s32 i, ParticleProperty& param);
	inline void initJoint(s32 i, ParticleJointProperty& param);

	// -------------------------------------------------------
	// Particle Method

	Vector3	getExtraForce() { return property.extraForce; }
	void setExtraForce(const Vector3& f) { property.extraForce = f; }

	u8 getContactIteration() { return property.contactIteration; }
	void setContactIteration(u8 i) { property.contactIteration = i; }

	u8 getJointIteration() { return property.jointIteration; }
	void setJointIteration(u8 i) { property.jointIteration = i; }

	u32	getContactFilterSelf() const { return property.contactFilterSelf; }
	void setContactFilterSelf(u32 filter) { property.contactFilterSelf = filter; }

	u32	getContactFilterTarget() const { return property.contactFilterTarget; }
	void setContactFilterTarget(u32 filter) { property.contactFilterTarget = filter; }

	f32	getMaxLinearVelocity() { return property.maxLinearVelocity; }
	void setMaxLinearVelocity(f32 value) { property.maxLinearVelocity = value; }

	f32	getLinearDamping() { return property.linearDamping; }
	void setLinearDamping(f32 value) { property.linearDamping = value; }

	f32	getSeparateBias() { return property.separateBias; }
	void setSeparateBias(f32 value) { property.separateBias = value; }

	bool getSelfCollisionEnable() { return property.selfCollisionEnable; }
	void setSelfCollisionEnable(bool b) { property.selfCollisionEnable = b; }

	bool getTwoWayInteraction() { return property.twoWayInteraction; }
	void setTwoWayInteraction(bool b) { property.twoWayInteraction = b; }

	u32	getParticleCount() { return property.numParticles; }
	PclState* getParticle(s32 idx) { return &pclStates[readBuffer][idx]; }
	void setParticle(s32 idx, const PclState& particle) { pclStates[readBuffer][idx] = particle; }

	u32	getJointCount() { return property.numJoints; }
	PclJoint* getJoint(s32 idx) { return &pclJoints[idx]; }
	void setJoint(s32 idx, const PclJoint& joint) { pclJoints[idx] = joint; }

	u32	getIndexCount() { return numIndices; }
	u32	getVertexCount() { return numVertices; }

	u32	getUserData(int i) { return userData[i]; }
	void setUserData(s32 i, u32 data) { userData[i] = data; }
};

inline void ParticleGroup::initParticle(s32 i, ParticleProperty& param)
{
	PclState newParticle;

	newParticle.setActive(param.active);
	newParticle.setMass(param.mass);
	newParticle.setRadius(param.radius);
	newParticle.setPosition(param.position);
	newParticle.setVelocity(param.velocity);
	newParticle.setElasticity(param.restitution);
	newParticle.setFriction(param.friction);

	pclStates[0][i] = pclStates[1][i] = newParticle;
	userData[i] = param.userData;
}

inline void ParticleGroup::initJoint(s32 i, ParticleJointProperty& param)
{
	PclJoint newJoint;

	newJoint.active = param.active;
	newJoint.type = param.type;
	newJoint.bias = param.bias;
#ifdef PARTICLE_VELOCITY_BASE
	newJoint.damping = param.damping;
#endif
	if(param.type == PclJointTypePcl) {
		newJoint.stateIndexA = param.particleA;
		newJoint.stateIndexB = param.particleB;
		newJoint.length = param.length;
	} else {
		newJoint.active = true;
		newJoint.stateIndexA = param.particleA;
		newJoint.stateIndexB = param.rigidBodyB;
		newJoint.length = param.length;
		newJoint.localPosB = param.localPosB;
	}
	pclJoints[i] = newJoint;
}

#endif /* PARTICLEGROUP_H_ */
