/*
 * softbodygroup.h
 *
 *  Created on: Jun 12, 2013
 *      Author: mike
 */

#ifndef SOFTBODYGROUP_H_
#define SOFTBODYGROUP_H_

#include "base/common.h"

#include "softbody/common/softbodyconfig.h"

enum SoftBodyType
{
	SoftBodyTypeGroup,
	SoftBodyTypeCloth,
	SoftBodyTypeSoftBody,
};

struct SoftBodyParticleProperty
{
	bool active;
	f32 mass;
	Vector3	position;
	Vector3	velocity;

	SoftBodyParticleProperty()
	{
		active = true;
		mass = 0.2f;
		position = velocity = Vector3(0.0f);
	}
};

struct SoftBodyJointProperty
{
	bool active;
	u16	type;
	u16	particleA;
	f32 length;
	f32 bias;

	union {
		u16	particleB;
		u16	rigidbodyB;
	};

	Vector3 localPosB;

	SoftBodyJointProperty()
	{
		active = true;
		type = SoftJointTypePcl;
		length = 1.0f;
		bias = 0.2f;
	}
};

struct SoftBodyGroupProperty
{
	SoftBodyType softbodyType;

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
	f32	particleRadius;

	bool hasVolume;
	f32	volume;
	bool softBodyMeshCollision;
	f32	pressureConst;

	SoftBodyGroupProperty()
	{
		softbodyType = SoftBodyTypeGroup;
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
		particleRadius = 1.0f;
		softBodyMeshCollision = false;
		volume = 0.0;
		pressureConst = 100.0f;
		hasVolume = false;
	}
};

ATTRIBUTE_ALIGNED16(class) SoftBodyGroup
{
	friend class SoftBodies;

public:
	// -------------------------------------------------------
	// Simulation Parameter

	SoftBodyGroupProperty property;

	u8 writeBuffer;
	u8 readBuffer;

	bool active;

	// -------------------------------------------------------
	// SoftBodies and Joints

	ATTRIBUTE_ALIGNED16(SoftState *sftStates[2]);
	ATTRIBUTE_ALIGNED16(SoftJoint *sftJoints);
	ATTRIBUTE_ALIGNED16(Vector3  *sftExtraForce);

	// -------------------------------------------------------
	// Mesh

	u32	numVertices;
	u32	numIndices;
	ATTRIBUTE_ALIGNED16(Vector3	*sftVertices);
	ATTRIBUTE_ALIGNED16(Vector3	*sftNormals);
	ATTRIBUTE_ALIGNED16(f32	*sftTexCoords);
	ATTRIBUTE_ALIGNED16(u16	*sftIndices);

	// -------------------------------------------------------
	// SoftBodies Collision

	Vector3	bbMin;
	Vector3	bbMax;
	ATTRIBUTE_ALIGNED16(Vector3	*pclPressureLocal);
	ATTRIBUTE_ALIGNED16(Vector3	*pclPressureGlobal);
	ATTRIBUTE_ALIGNED16(u16	*pclPressureCount);

public:

	// -------------------------------------------------------
	// Method

	SoftBodyGroup()
	{
		numVertices = 0;
		numIndices = 0;
		sftVertices = NULL;
		sftNormals = NULL;
		sftTexCoords = NULL;
		sftIndices = NULL;
		sftExtraForce = NULL;
	}

	~SoftBodyGroup()
	{
	}

	void initialize()
	{
		active = true;
		writeBuffer = 0;
		readBuffer = 1;
		memset(sftStates[0], 0, sizeof(SoftState)*property.numParticles);
		memset(sftStates[1], 0, sizeof(SoftState)*property.numParticles);
		memset(sftJoints, 0, sizeof(SoftJoint)*property.numJoints);
		memset(sftExtraForce, 0, sizeof(Vector3)*property.numParticles);
	}

	// -------------------------------------------------------
	// SoftBody Initialize

	inline void initSoftBody(s32 i, SoftBodyParticleProperty& param);

	// -------------------------------------------------------
	// Joint Initialize

	inline void initJoint(s32 i, SoftBodyJointProperty& param);

	// -------------------------------------------------------
	// SoftBody Method

	Vector3	getExtraForce() {return property.extraForce;}
	void setExtraForce(const Vector3& f) {property.extraForce = f;}

	f32	getParticleRaius() {return property.particleRadius;}
	void setParticleRadius(f32 r) {property.particleRadius = r;}

	u8 getContactIteration() {return property.contactIteration;}
	void setContactIteration(u8 i) {property.contactIteration = i;}

	u8 getJointIteration() {return property.jointIteration;}
	void setJointIteration(u8 i) {property.jointIteration = i;}

	u32	getContactFilterSelf() const {return property.contactFilterSelf;}
	void setContactFilterSelf(u32 filter) {property.contactFilterSelf = filter;}

	u32	getContactFilterTarget() const {return property.contactFilterTarget;}
	void setContactFilterTarget(u32 filter) {property.contactFilterTarget = filter;}

	f32 getMaxLinearVelocity() {return property.maxLinearVelocity;}
	void setMaxLinearVelocity(f32 value) {property.maxLinearVelocity = value;}

	f32	getLinearDamping() {return property.linearDamping;}
	void setLinearDamping(f32 value) {property.linearDamping = value;}

	f32	getSeparateBias() {return property.separateBias;}
	void setSeparateBias(f32 value) {property.separateBias = value;}

	bool getSelfCollisionEnable() {return property.selfCollisionEnable;}
	void setSelfCollisionEnable(bool b)  {property.selfCollisionEnable = b;}

	bool getTwoWayInteraction() {return property.twoWayInteraction;}
	void setTwoWayInteraction(bool b)  {property.twoWayInteraction = b;}

	bool getSoftBodyMeshCollision() {return property.softBodyMeshCollision;}
	void setSoftBodyMeshCollision(bool b)  {property.softBodyMeshCollision = ((b & property.hasVolume) & (property.volume > 0));}

	bool getHasVolume() {return property.hasVolume;}
	void setHasVolume(bool b)  {property.hasVolume = b;}

	u32	getParticleCount() {return property.numParticles;}
	SoftState *getParticle(s32 idx) {return &sftStates[readBuffer][idx];}
	void setParticle(s32 idx, const SoftState& particle) {sftStates[readBuffer][idx] = particle;}

	u32	getJointCount() {return property.numJoints;}
	SoftJoint *getJoint(s32 idx) {return &sftJoints[idx];}
	void setJoint(s32 idx, const SoftJoint& joint) {sftJoints[idx] = joint;}

	u32	getIndexCount() {return numIndices;}
	u32	getVertexCount() {return numVertices;}

	Vector3 getExtraForce(u32 idx) {return sftExtraForce[idx];}
	void setExtraForce(u32 idx, const Vector3& f) {sftExtraForce[idx] = f;}

	bool isActive() {return active;}
	void activate() {active = true;}
	void deactivate() {active = false;}
};

inline void SoftBodyGroup::initSoftBody(s32 i, SoftBodyParticleProperty& param)
{
	SoftState newParticle;
	newParticle.setActive(param.active);
	newParticle.setMass(param.mass);
	newParticle.setPosition(param.position);
	newParticle.setOldPosition(param.position);
	newParticle.setVelocity(param.velocity);

	sftStates[0][i] = sftStates[1][i] = newParticle;
}

inline void SoftBodyGroup::initJoint(s32 i, SoftBodyJointProperty& param)
{
	SoftJoint newJoint;
	newJoint.active = param.active;
	newJoint.type = param.type;
	newJoint.bias = param.bias;
	if(param.type == SoftJointTypePcl) {
		newJoint.stateIndexA = param.particleA;
		newJoint.stateIndexB = param.particleB;
		newJoint.length = param.length;
	} else {
		newJoint.active = true;
		newJoint.stateIndexA = param.particleA;
		newJoint.stateIndexB = param.rigidbodyB;
		newJoint.length = param.length;
		newJoint.localPosB = param.localPosB;
	}
	sftJoints[i] = newJoint;
}

#endif /* SOFTBODYGROUP_H_ */
