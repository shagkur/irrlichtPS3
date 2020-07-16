/*
 * physicsmanager.h
 *
 *  Created on: Jun 6, 2013
 *      Author: mike
 */

#ifndef PHYSICSMANAGER_H_
#define PHYSICSMANAGER_H_

#include "iphysicsmanager.h"
#include "irrarray.h"

#include "rigidbody/rigidbodies.h"
#include "rigidbody/mass.h"

#include "raycast/raycast.h"

#include "util/threadutil.h"


#define HEAP_SIZE_RB			(10*1024*1024)
#define HEAP_SIZE_RC			(5*1024*1024)

namespace irr
{
	namespace scene
	{
		enum E_KDOP_TYPE
		{
			EKDPT_6DOP = 3,
			EKDPT_14DOP = 7,
			EKDPT_26DOP = 13
		};

		class IMesh;

		class CPhysicsManager : public IPhysicsManager, public ContactCallback, public SleepCallback
		{
			struct CHPlane
			{
				core::vector3df N, S, T;
				f32 d;

				CHPlane() {}
				CHPlane(const core::vector3df& n, const core::vector3df& q)
				{
					N = n;
					d = dot(n, q);
					getPlaneSpace(N, S, T);
				}

				f32 onPlane(const core::vector3df& p) const
				{
					return dot(p, N) - d;
				}

				void getPlaneSpace(const core::vector3df& n, core::vector3df& p, core::vector3df& q);
			};

			struct CHPolygon
			{
				CHPlane plane;
				core::array< core::vector3df > vertices;
			};

			struct CHMesh
			{
				core::array< u16 > indices;
				core::array< core::vector3df > vertices;

				u16 getVertexId(core::vector3df& v);
				void addTriangle(u16 vi0, u16 vi1, u16 vi2);
			};

		public:
			CPhysicsManager(ISceneManager *mgr);
			virtual ~CPhysicsManager();

			virtual void setWorldSize(const core::vector3df& center, const core::vector3df& extent);

			virtual void stepSimulation(f32 timeStep, s32 maxSubSteps = 1, f32 fixedTimeStep = 1.0f/60.0f);

			virtual IRigidBody* createRigidBodyBox(ISceneNode *node, f32 mass);

			virtual IRigidBody* createRigidBodySphere(ISceneNode *node, f32 mass);

			virtual IRigidBody* createRigidBodyCone(ISceneNode *node, f32 mass, E_BODY_ORIENT orientation = EBO_AXIS_AUTO);

			virtual IRigidBody* createRigidBodyConvexHull(IMeshSceneNode *node, f32 mass);

			virtual IRigidBody* createRigidBodyConvexHull(IAnimatedMeshSceneNode *node, f32 mass);

			virtual RigidBodies* getRigidBodies() const { return _rigidBodies; }

			virtual void onContact(ContactPair& pair);
			virtual void onSleep(u16 stateIndex);
			virtual void onActive(u16 stateIndex);

		private:
			static constexpr f32 CONVEX_HULL_EPSILON = 1e-04f;
			static constexpr u32 NUM_KDOP = 13;

			static const core::vector3df VECTOR_KDOP_TABLE[NUM_KDOP];

			void setupWorld();
			void setupRaycast();
			void updateRigidBodiesScene();
			void updateRigidBodiesPhysics();
			Transform3 getNodeTransform(ISceneNode *node);

			void createConvexHull(ConvexMesh& mesh, core::vector3df *vertices, s32 numVertices);
			void createConvexPolygons(E_KDOP_TYPE type, core::vector3df *vertices, s32 numVertices, core::array< CHPolygon >& polys);
			void shrinkVertices(core::vector3df *vertices, s32& numVertices, u16 *indices, s32& numIndices);
			void createConvexMesh(ConvexMesh& mesh, core::vector3df *vertices, s32 numVertices, u16 *indices, s32 numIndices);

			void convertPolygonToMesh(core::array< CHPolygon >& polys, CHMesh& mesh);
			f32 calcPolygonArea(CHPolygon& poly);
			void removeSameVertex(core::array< core::vector3df >& vertices);
			void sortGatherVertsByPlane(core::array< core::vector3df >& gatherVertices, CHPlane& plane);
			s32 slicePolygonByPlane(CHPolygon& poly, CHPlane& plane, core::array< core::vector3df >& gatherVertices);

			CollObject* createCollObjectFromMesh(IMesh *mesh, const core::vector3df& scale, f32 mass, Matrix3& inertia);

			f32 _localTime;

			PfxThread _pfxThread;

			HeapManager _poolRB;
			RigidBodies *_rigidBodies;

			HeapManager _poolRC;
			RayCast *_rayCast;

			MARSTaskManager *_taskManager;

			core::array<IRigidBody*> _rigidBodyList;

			ISceneManager *_sceneManager;

			mars_context *_marsContext;
		};
	}
}

#endif /* PHYSICSMANAGER2_H_ */
