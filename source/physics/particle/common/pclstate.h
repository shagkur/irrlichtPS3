/*
 * pclstate.h
 *
 *  Created on: Jan 26, 2014
 *      Author: mike
 */

#ifndef PCLSTATE_H_
#define PCLSTATE_H_

#include "base/common.h"

#include "particleconfig.h"

#ifdef PARTICLE_VELOCITY_BASE

ATTRIBUTE_ALIGNED16(class) PclState
{
public:
	bool active : 1;

	f32 fMass;
	f32 fElasticity;
	f32 fFriction;
	f32 fRadius;

	Vector3 fX;
	Vector3 fV;

	PclState()
	{
		active = true;
		fX = fV = Vector3(0.0f);
	}

	bool isActive() const { return active; }
	void setActive(bool b) { active = b; }

	f32 getMass() const { return fMass; }
	void setMass(f32 mass) { fMass = mass; }

	f32 getElasticity() const { return fElasticity; }
	void setElasiticity(f32 elasticity) { fElasticity = elasticity; }

	f32 getFriction() const { return fFriction; }
	void setFriction(f32 friction) { fFriction = friction; }

	f32 getRadius() const { return fRadius; }
	void setRadius(f32 radius) { fRadius = radius; }

	Vector3 getPosition() { return fX; }
	void setPosition(const Vector3& pos) { fX = pos; }

	Vector3 getVelocity() { return fV; }
	void setVelocity(const Vector3& vel) { fV = vel; }
};

#else

ATTRIBUTE_ALIGNED16(class) PclState
{
public:
	bool active : 1;

	f32 fMass;
	f32 fRadius;

	Vector3 fX;
	Vector3 oldX;

	PclState()
	{
		active = true;
		fX = Vector3(0.0f);
	}

	bool isActive() const { return active; }
	void setActive(bool b) { active = b; }

	f32 getMass() const { return fMass; }
	void setMass(f32 mass) { fMass = mass; }

	f32 getElasticity() const { return 0.0f; }
	void setElasticity(f32 elasticity) { (void)elasticity; }

	f32 getFriction() const { return 0.0f; }
	void setFriction(f32 friction) { (void)friction; }

	f32 getRadius() const { return fRadius; }
	void setRadius(f32 radius) { fRadius = radius; }

	Vector3 getPosition() { return fX; }
	void setPosition(const Vector3& pos) { fX = pos; }

	Vector3 getVelocity() { return fX - oldX; }
	void setVelocity(const Vector3& vel) { oldX = fX - vel; }
};

#endif

#endif /* PCLSTATE_H_ */
