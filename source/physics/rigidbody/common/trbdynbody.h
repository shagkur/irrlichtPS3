/*
 * trbdynbody.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef TRBDYNBODY_H_
#define TRBDYNBODY_H_

#include "base/common.h"

#include "rigidbody/common/trbstatevec.h"

class CollObject;

ATTRIBUTE_ALIGNED16(class) TrbDynBody
{
public:
	TrbDynBody()
	{
		fMass   = 0.0f;
		fCollObject = NULL;
		fElasticity = 0.2f;
		fFriction = 0.8f;
	}

	// Get methods
	f32 getMass() const {return fMass;};
	f32 getElasticity() const {return fElasticity;}
	f32 getFriction() const {return fFriction;}
	CollObject* getCollObject() const {return fCollObject;}
	const Matrix3& getBodyInertia() const {return fIBody;}
	const Matrix3& getBodyInertiaInv() const {return fIBodyInv;}
	f32 getMassInv() const {return fMassInv;}

	// Set methods
	void setMass(f32 mass) {fMass = mass; fMassInv = mass > 0.0f ? 1.0f/mass : 0.0f;}
	void setBodyInertia(const Matrix3 bodyInertia) {fIBody = bodyInertia; fIBodyInv = inverse(bodyInertia);}
	void setElasticity(f32 elasticity) {fElasticity = elasticity;}
	void setFriction(f32 friction) {fFriction = friction;}
	void setCollObject(CollObject *collObj) {fCollObject = collObj;}

private:
	// Rigid Body constants
	f32 fMass;        // Rigid Body mass
	f32 fMassInv;     // Inverse of mass
	Matrix3 fIBody;       // Inertia matrix in body's coords
	Matrix3 fIBodyInv;    // Inertia matrix inverse in body's coords
	f32 fElasticity;  // Coefficient of restitution
	f32 fFriction;    // Coefficient of friction

public:
	ATTRIBUTE_PTR32(CollObject* fCollObject);  // Collision object corresponding the RB
};

#endif /* TRBDYNBODY_H_ */
