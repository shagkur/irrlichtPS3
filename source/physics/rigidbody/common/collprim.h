/*
 * collprim.h
 *
 *  Created on: Jun 2, 2013
 *      Author: mike
 */

#ifndef COLLPRIM_H_
#define COLLPRIM_H_

#include "base/simdfunc.h"

#include "rigidbody/common/box.h"
#include "rigidbody/common/capsule.h"
#include "rigidbody/common/sphere.h"
#include "rigidbody/common/trimesh.h"
#include "rigidbody/common/heightfield.h"

enum PrimType {
	SPHERE = 0,
	BOX,
	CAPSULE,
	HEIGHTFIELD,
	CONVEXMESH,
	TRIANGLEMESH,
	LARGEMESH,
	PRIM_COUNT
};

class HeightField;

ATTRIBUTE_ALIGNED16(class) CollPrim
{
	friend class CollObject;

private:
	PrimType type;
	f32 relativeOrientation[4];
	f32 relativePosition[3];
	u32 contactFilterSelf;
	u32 contactFilterTarget;
	union {
		f32 vfData[3];
		u32 viData[3];
	};

public:
	inline void reset()
	{
 		contactFilterSelf = contactFilterTarget = 0xffffffff;
 	}

	inline void setBox(Box box);
	inline void setCapsule(Capsule capsule);
	inline void setSphere(Sphere sphere);
	inline void setHeightField(const HeightField *heightfield);
	inline void setConvexMesh(const ConvexMesh *convexMesh);
	inline void setTriangleMesh(const TriMesh *triangleMesh);
	inline void setLargeMesh(const LargeTriMesh *largeMesh);
	inline void setObjectRelTransform(const Transform3& xfrm);

	inline PrimType getType() const;
	inline Box getBox() const;
	inline Capsule getCapsule() const;
	inline Sphere getSphere() const;
	inline Transform3  getObjectRelTransform() const;
	inline Quat  getObjectRelOrientation() const {return read_Quat(relativeOrientation);}
	inline Vector3 getObjectRelPosition() const {return read_Vector3(relativePosition);}

	inline void setObjectRelOrientation(Quat rot) {return store_Quat(rot, relativeOrientation);}
	inline void setObjectRelPosition(Vector3 pos) {return store_Vector3(pos, relativePosition);}

	inline void setPrimDataFloat(s32 i, f32 v) {vfData[i] = v;}
	inline void setPrimDataInteger(s32 i, u32 v) {viData[i] = v;}
	inline f32 getPrimDataFloat(s32 i) const {return vfData[i];}
	inline u32 getPrimDataInteger(s32 i) const {return viData[i];}

	inline f32 getBoundingRadius() const;

	void setContactFilterSelf(u32 filter) {contactFilterSelf = filter;}
	u32	getContactFilterSelf() const {return contactFilterSelf;}

	void setContactFilterTarget(u32 filter) {contactFilterTarget = filter;}
	u32	getContactFilterTarget() const {return contactFilterTarget;}

	// deform mesh
#ifndef __SPU__
	inline void setPreLargeMesh(const LargeTriMesh *largeMesh) {viData[1] = (u32)(u64)largeMesh;}
	inline LargeTriMesh *getPreLargeMesh() const {return (LargeTriMesh*)((u64)viData[1]);}
#else
	inline void setPreLargeMesh(const LargeTriMesh *largeMesh) {viData[1] = (u32)largeMesh;}
	inline LargeTriMesh *getPreLargeMesh() const {return (LargeTriMesh*)viData[1];}
#endif

#ifdef __SPU__
	inline void	getHeightField(HeightField *heightfield) const;
	inline void getConvexMesh(ConvexMesh *mesh) const;
	inline void getLargeMesh(LargeTriMesh *largeMesh) const;
	inline void	getHeightFieldNB(HeightField *heightfield, s32 tag) const;
	inline void getConvexMeshNB(ConvexMesh *mesh, s32 tag) const;
	inline void getLargeMeshNB(LargeTriMesh *largeMesh,s32 tag) const;
#else
	inline HeightField* getHeightField() const;
	inline ConvexMesh* getConvexMesh() const;
	inline LargeTriMesh* getLargeMesh() const;
#endif
};

inline void CollPrim::setBox(Box box)
{
	reset();
	vfData[0] = box.half[0];
	vfData[1] = box.half[1];
	vfData[2] = box.half[2];
	type = BOX;
}

inline void CollPrim::setCapsule(Capsule capsule)
{
	reset();
	vfData[0] = capsule.hLength;
	vfData[1] = capsule.radius;
	vfData[2] = 0.0f;
	type = CAPSULE;
}

inline void CollPrim::setSphere(Sphere sphere)
{
	reset();
	vfData[0] = sphere.radius;
	vfData[1] = 0.0f;
	vfData[2] = 0.0f;
	type = SPHERE;
}

inline void CollPrim::setHeightField(const HeightField *heightfield)
{
	reset();
#ifndef __SPU__
	viData[0] = (u32)(u64)heightfield;
#else
	viData[0] = (u32)heightfield;
#endif
	viData[1] = 0;
	viData[2] = 0;
	type = HEIGHTFIELD;
}

inline void CollPrim::setConvexMesh(const ConvexMesh *convexMesh)
{
	reset();
#ifndef __SPU__
	viData[0] = (u32)(u64)convexMesh;
#else
	viData[0] = (u32)convexMesh;
#endif
	viData[1] = 0;
	viData[2] = 0;
	type = CONVEXMESH;
}

inline void CollPrim::setTriangleMesh(const TriMesh *triangleMesh)
{
	reset();
#ifndef __SPU__
	viData[0] = (u32)(u64)triangleMesh;
#else
	viData[0] = (u32)triangleMesh;
#endif
	viData[1] = 0;
	viData[2] = 0;
	type = TRIANGLEMESH;
}

inline void CollPrim::setLargeMesh(const LargeTriMesh *largeMesh)
{
	reset();
#ifndef __SPU__
	viData[0] = (u32)(u64)largeMesh;
#else
	viData[0] = (u32)largeMesh;
#endif
	viData[1] = 0;
	viData[2] = 0;
	type = LARGEMESH;
}

inline void CollPrim::setObjectRelTransform(const Transform3 & xfrm)
{
	setObjectRelOrientation(Quat(xfrm.getUpper3x3()));
	setObjectRelPosition(xfrm.getTranslation());
}

inline PrimType CollPrim::getType() const
{
	return type;
}

inline Box CollPrim::getBox() const
{
	return Box(vfData[0], vfData[1], vfData[2]);
}

inline Capsule CollPrim::getCapsule() const
{
	return Capsule(vfData[0], vfData[1]);
}

inline Sphere CollPrim::getSphere() const
{
	return Sphere(vfData[0]);
}

#ifdef __SPU__

inline void CollPrim::getHeightField(HeightField *heightfield) const
{
	spu_dma_get(heightfield, viData[0], sizeof(HeightField), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);
}

inline void CollPrim::getConvexMesh(ConvexMesh *mesh) const
{
	spu_dma_get(mesh, viData[0], sizeof(ConvexMesh), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);
}

inline void CollPrim::getLargeMesh(LargeTriMesh *largeMesh) const
{
	spu_dma_get(largeMesh, viData[0], sizeof(LargeTriMesh), 0, 0, 0);
	spu_dma_wait_tag_status_all(1);
}

inline void CollPrim::getHeightFieldNB(HeightField *heightfield, s32 tag) const
{
	spu_dma_get(heightfield, viData[0], sizeof(HeightField), tag, 0, 0);
}

inline void CollPrim::getConvexMeshNB(ConvexMesh *mesh, s32 tag) const
{
	spu_dma_get(mesh, viData[0], sizeof(ConvexMesh), tag, 0, 0);
}

inline void CollPrim::getLargeMeshNB(LargeTriMesh *largeMesh, s32 tag) const
{
	spu_dma_get(largeMesh, viData[0], sizeof(LargeTriMesh), tag, 0, 0);
}

#else

inline HeightField* CollPrim::getHeightField() const
{
	return (HeightField*)((u64)viData[0]);
}

inline ConvexMesh* CollPrim::getConvexMesh() const
{
	return (ConvexMesh*)((u64)viData[0]);
}

inline LargeTriMesh* CollPrim::getLargeMesh() const
{
	return (LargeTriMesh*)((u64)viData[0]);
}

#endif

inline Transform3 CollPrim::getObjectRelTransform() const
{
	return Transform3(getObjectRelOrientation(), getObjectRelPosition());
}

inline f32 CollPrim::getBoundingRadius() const
{
	if ( getType() == BOX ) {
		return length(Vector3(vfData[0], vfData[1], vfData[2]));
	} else if ( getType() == CAPSULE ) {
		return vfData[0] + vfData[1];
	} else if ( getType() == SPHERE ) {
		return vfData[0];
	} else if ( getType() == CONVEXMESH ) {
#ifdef __SPU__
		ATTRIBUTE_ALIGNED16(struct) MiniConvex {
			u8 numVerts;
			u8 numIndices;
			f32 half[3];
		} mesh;
		spu_dma_get(&mesh, viData[0], sizeof(MiniConvex), 1, 0, 0);
		spu_dma_wait_tag_status_all(1);
		return length(read_Vector3(mesh.half));
#else
		return length(read_Vector3(getConvexMesh()->half));
#endif
	} else if ( getType() == LARGEMESH ) {
		return 1000.0f; // TODO
	}
	return 0.0f;
}

#endif /* COLLPRIM_H_ */
