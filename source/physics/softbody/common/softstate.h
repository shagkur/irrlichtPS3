/*
 * softstate.h
 *
 *  Created on: Jun 12, 2013
 *      Author: mike
 */

#ifndef SOFTSTATE_H_
#define SOFTSTATE_H_

#include "base/common.h"

#include "softbody/common/softbodyconfig.h"

ATTRIBUTE_ALIGNED16(class) SoftState
{
public:
	bool active : 1;

	f32 fMass;

	Vector3 fX;
	Vector3 oldX;

	Vector3 accumulation;
	Vector3 externalForce;

	SoftState()
	{
		active = true;
		fX = Vector3(0.0f);
		externalForce = Vector3(0.0f);
		accumulation = Vector3(0.0f);
	}

	bool isActive() { return active; }
	void setActive(bool b) { active = b; }

	f32 getMass() const { return fMass; }
	void setMass(f32 mass) { fMass = mass; }

	f32 getElasticity() const { return 0.0f; }
	void setElasticity(f32 elasticity) { (void)elasticity; }

	f32 getFriction() const { return 0.0f; }
	void setFriction(f32 friction) { (void)friction; }

	Vector3 getPosition() { return fX; }
	void setPosition(const Vector3& pos) { fX = pos; }

	Vector3 getOldPosition() { return oldX; }
	void setOldPosition(const Vector3& pos) { oldX = pos; }

	Vector3 getVelocity() { return fX - oldX; }
	void setVelocity(const Vector3& velocity) { oldX = fX - velocity; }
};

#endif /* SOFTSTATE_H_ */
