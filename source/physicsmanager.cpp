/*
 * physicsmanager.cpp
 *
 *  Created on: Jun 6, 2013
 *      Author: mike
 */

#include "physicsmanager.h"
#include "rigidbody.h"
#include "imesh.h"
#include "imeshbuffer.h"
#include "imeshscenenode.h"
#include "ianimatedmeshscenenode.h"
#include "iscenemanager.h"
#include "ivideodriver.h"

#define NUM_MAX_SPU			5

static u8 memPoolRB[HEAP_SIZE_RB] __attribute__((aligned(128)));
static u8 memPoolRC[HEAP_SIZE_RC] __attribute__((aligned(128)));

namespace irr
{
	namespace scene
	{
		const core::vector3df CPhysicsManager::VECTOR_KDOP_TABLE[NUM_KDOP] =
		{
			// 6 DOP
			core::vector3df(1.0f, 0.0f, 0.0f),
			core::vector3df(0.0f, 1.0f, 0.0f),
			core::vector3df(0.0f, 0.0f, 1.0f),

			// 14 DOP
			normalize(core::vector3df( 1.0f,  1.0f, 1.0f)),
			normalize(core::vector3df(-1.0f,  1.0f, 1.0f)),
			normalize(core::vector3df(-1.0f, -1.0f, 1.0f)),
			normalize(core::vector3df( 1.0f, -1.0f, 1.0f)),

			// 26 DOP
			normalize(core::vector3df( 1.0f,  1.0f, 0.0f)),
			normalize(core::vector3df(-1.0f,  1.0f, 0.0f)),
			normalize(core::vector3df( 1.0f,  0.0f, 1.0f)),
			normalize(core::vector3df( 0.0f,  1.0f, 1.0f)),
			normalize(core::vector3df(-1.0f,  0.0f, 1.0f)),
			normalize(core::vector3df( 0.0f, -1.0f, 1.0f))
		};

		static void pfxThreadEntry(void *arg)
		{
			sys_event_t event;
			bool bExit = false;
			PfxThread *pfxThread = reinterpret_cast<PfxThread*>(arg);

			while(!bExit) {
				sysEventQueueReceive(pfxThread->sendQueue(), &event, 0);

				void *param = (void*)event.data_2;
				switch(event.data_1) {
					case PFXTHREAD_SIMULATION_START:
						break;
					case PFXTHREAD_SIMULATION_END:
						bExit = true;
						break;
				}
			}

			sysThreadExit(0);
		}

		void CPhysicsManager::CHPlane::getPlaneSpace(const core::vector3df& n, core::vector3df& p, core::vector3df& q)
		{
			if(fabsf(n[2]) > 0.707f) {
				f32 a = n[1]*n[1] + n[2]*n[2];
				f32 k = 1.0f/sqrtf(a);
				p[0] = 0;
				p[1] = -n[2]*k;
				p[2] = n[1]*k;
				// set q = n x p
				q[0] = a*k;
				q[1] = -n[0]*p[2];
				q[2] = n[0]*p[1];
			} else {
				// choose p in x-y plane
				f32 a = n[0]*n[0] + n[1]*n[1];
				f32 k = 1.0f/sqrtf(a);
				p[0] = -n[1]*k;
				p[1] = n[0]*k;
				p[2] = 0;
				// set q = n x p
				q[0] = -n[2]*p[1];
				q[1] = n[2]*p[0];
				q[2] = a*k;
			}
		}

		u16 CPhysicsManager::CHMesh::getVertexId(core::vector3df& v)
		{
			for(u16 i=0;i < vertices.size();i++) {
				f32 lenSqr = lengthSqr(v - vertices[i]);
				if(lenSqr < CONVEX_HULL_EPSILON*CONVEX_HULL_EPSILON)
					return i;
			}

			vertices.push_back(v);

			return vertices.size() - 1;
		}

		void CPhysicsManager::CHMesh::addTriangle(u16 vi0, u16 vi1, u16 vi2)
		{
			indices.push_back(vi0);
			indices.push_back(vi1);
			indices.push_back(vi2);
		}

		CPhysicsManager::CPhysicsManager(ISceneManager *mgr)
		: _localTime(0.0f), _poolRB(memPoolRB, HEAP_SIZE_RB), _poolRC(memPoolRC, HEAP_SIZE_RC),
		  _sceneManager(mgr)
		{
			if(_sceneManager != NULL) _sceneManager->grab();

			_marsContext = _sceneManager->getVideoDriver()->getMARSContext();

			_taskManager = new MARSTaskManager(_marsContext, NUM_MAX_SPU);
			_taskManager->initialize();

			_rigidBodies = new RigidBodies(_taskManager, 0, &_poolRB);
			_rayCast = new RayCast(_taskManager, 0, &_poolRC);

			setupWorld();
			setupRaycast();
		}

		CPhysicsManager::~CPhysicsManager()
		{
			if(_sceneManager != NULL) _sceneManager->drop();

			_taskManager->finalize();

			delete _rigidBodies;
			delete _rayCast;
			delete _taskManager;
		}

		void CPhysicsManager::setupWorld()
		{
			WorldProperty worldProperty;

			worldProperty.maxDynBodies = 550;
			worldProperty.maxInstances = 550;
			worldProperty.worldCenter = core::vector3df(0.0f, 90.0f, 0.0f);
			worldProperty.worldExtent = core::vector3df(1000.0f, 1000.0f, 1000.0f);
			worldProperty.ccdEnable = true;
			worldProperty.contactCallback = this;
			worldProperty.sleepCallback = this;
			worldProperty.separateBias = 0.2f;
			worldProperty.contactIteration = 10;

			_rigidBodies->setWorldProperty(worldProperty);
			_rigidBodies->reset();
		}

		void CPhysicsManager::setupRaycast()
		{
			RaycastProperty raycastProperty;

			raycastProperty.maxInstances = 550;
			raycastProperty.maxRayGroups = 10;
			raycastProperty.useRigidBodyWorldSize = true;

			_rayCast->setRaycastProperty(raycastProperty);
			_rayCast->reset();
		}

		void CPhysicsManager::setWorldSize(const core::vector3df& center, const core::vector3df& extent)
		{
			_rigidBodies->setWorldSize(center, extent);
		}

		void CPhysicsManager::stepSimulation(f32 timeStep, s32 maxSubSteps, f32 fixedTimeStep)
		{
			s32 numSimulationSubSteps = 0;

			if(maxSubSteps) {
				_localTime += timeStep;
				if(_localTime >= fixedTimeStep) {
					numSimulationSubSteps = s32(_localTime/fixedTimeStep);
					_localTime -= numSimulationSubSteps*fixedTimeStep;
				}
			} else {
				fixedTimeStep = timeStep;
				_localTime = timeStep;

				if(fabsf(timeStep) < FLT_EPSILON) {
					numSimulationSubSteps = 0;
					maxSubSteps = 0;
				} else {
					numSimulationSubSteps = 1;
					maxSubSteps = 1;
				}
			}

			if(numSimulationSubSteps) {
				s32 clampedSimulationSteps = core::min_(numSimulationSubSteps, maxSubSteps);

				timeStep = fixedTimeStep*clampedSimulationSteps;

				_rigidBodies->setSubStepCount(clampedSimulationSteps);
				_rigidBodies->setupSimulate();
				_rigidBodies->spuSimulate(timeStep, 0);
				_rigidBodies->finalizeSimulate();
			}

			updateRigidBodiesScene();
		}

		IRigidBody* CPhysicsManager::createRigidBodyBox(ISceneNode *node, f32 mass)
		{
			RigidBodyProperty bodyProperty;
			const core::vector3df& scale = node->getScale();
			CollObject *collObj = _rigidBodies->createCollObject();
			core::vector3df extent = node->getBoundingBox().getExtent();
			core::vector3df halfEdges = extent*0.5f;
			core::vector3df halfExtents = mulPerElem(scale, halfEdges);
			Transform3 nodeTransform = getNodeTransform(node);

			collObj->addBox(Box(halfExtents), Transform3::identity());
			collObj->finish();

			bodyProperty.mass = mass;
			bodyProperty.collObject = collObj;
			calcInertiaBox(halfExtents, bodyProperty.mass, bodyProperty.inertia);

			TrbDynBody *rigidBody = _rigidBodies->createRigidBody(bodyProperty);

			InstanceProperty instProperty;
			instProperty.moveType = MoveTypeActive;
			instProperty.position = nodeTransform.getTranslation();
			instProperty.orientation = Quat(nodeTransform.getUpper3x3());
			instProperty.rigidBody = rigidBody;

			s32 instance = _rigidBodies->createInstance(instProperty);

			IRigidBody *cmbody = new CRigidBody(ERBT_BOX, instance, node, this);
			_rigidBodyList.push_back(cmbody);

			return cmbody;
		}

		IRigidBody* CPhysicsManager::createRigidBodySphere(ISceneNode *node, f32 mass)
		{
			RigidBodyProperty bodyProperty;
			const core::vector3df& scale = node->getScale();
			CollObject *collObj = _rigidBodies->createCollObject();
			core::vector3df extent = node->getBoundingBox().getExtent();
			f32 radius = (extent.getY()*scale.getY())*0.5f;
			Transform3 nodeTransform = getNodeTransform(node);

			collObj->addSphere(Sphere(radius), Transform3::identity());
			collObj->finish();

			bodyProperty.mass = mass;
			bodyProperty.collObject = collObj;
			calcInertiaSphere(radius, bodyProperty.mass, bodyProperty.inertia);

			TrbDynBody *rigidBody = _rigidBodies->createRigidBody(bodyProperty);

			InstanceProperty instProperty;
			instProperty.moveType = MoveTypeActive;
			instProperty.position = nodeTransform.getTranslation();
			instProperty.orientation = Quat(nodeTransform.getUpper3x3());
			instProperty.rigidBody = rigidBody;

			s32 instance = _rigidBodies->createInstance(instProperty);

			IRigidBody *cmbody = new CRigidBody(ERBT_SPHERE, instance, node, this);
			_rigidBodyList.push_back(cmbody);

			return cmbody;
		}

		IRigidBody* CPhysicsManager::createRigidBodyCone(ISceneNode *node, f32 mass, E_BODY_ORIENT orientation)
		{
			f32 radius, height;
			core::vector3df edges[8];
			RigidBodyProperty bodyProperty;
			const core::vector3df& scale = node->getScale();
			CollObject *collObj = _rigidBodies->createCollObject();
			core::vector3df extent = node->getBoundingBox().getExtent();
			Transform3 nodeTransform = getNodeTransform(node);
			E_BODY_ORIENT coneOri = orientation;

			node->getBoundingBox().getEdges(edges);

			if(coneOri == EBO_AXIS_AUTO) {
				if(extent.getX() == extent.getY() && extent.getX() == extent.getZ())
					coneOri = EBO_AXIS_Y;
				else if(extent.getY() == extent.getZ())
					coneOri = EBO_AXIS_X;
				else if(extent.getX() == extent.getZ())
					coneOri = EBO_AXIS_Y;
				else if(extent.getX() == extent.getY())
					coneOri = EBO_AXIS_Z;
				else
					coneOri = EBO_AXIS_Y;
			}

			switch(coneOri) {
				case EBO_AXIS_X:
					radius = fabsf(edges[1].getY() - edges[0].getY())*0.5f;
					height = fabsf(edges[4].getX() - edges[0].getX());

					radius *= scale.getY();
					height *= scale.getX();
					break;
				case EBO_AXIS_Y:
					radius = fabsf(edges[4].getX() - edges[0].getX())*0.5f;
					height = fabsf(edges[1].getY() - edges[0].getY());

					radius *= scale.getX();
					height *= scale.getY();
					break;
				case EBO_AXIS_Z:
					radius = fabsf(edges[4].getX() - edges[0].getX())*0.5f;
					height = fabsf(edges[2].getZ() - edges[0].getZ());

					radius *= scale.getX();
					height *= scale.getZ();
					break;
				default:
					radius = fabsf(edges[4].getX() - edges[0].getX())*0.5f;
					height = fabsf(edges[1].getY() - edges[0].getY());

					radius *= scale.getX();
					height *= scale.getY();

					coneOri = EBO_AXIS_Y;
					break;
			}

			collObj->addCapsule(Capsule(height, radius), Transform3::identity());
			collObj->finish();

			bodyProperty.mass = mass;
			bodyProperty.collObject = collObj;
			calcInertiaCylinder(radius, height, bodyProperty.mass, bodyProperty.inertia, coneOri);

			TrbDynBody *rigidBody = _rigidBodies->createRigidBody(bodyProperty);

			InstanceProperty instProperty;
			instProperty.moveType = MoveTypeActive;
			instProperty.position = nodeTransform.getTranslation();
			instProperty.orientation = Quat(nodeTransform.getUpper3x3());
			instProperty.rigidBody = rigidBody;

			s32 instance = _rigidBodies->createInstance(instProperty);

			IRigidBody *cmbody = new CRigidBody(ERBT_CONE, instance, node, this);
			_rigidBodyList.push_back(cmbody);

			return cmbody;
		}

		IRigidBody* CPhysicsManager::createRigidBodyConvexHull(IMeshSceneNode *node, f32 mass)
		{
			Matrix3 inertia;
			RigidBodyProperty bodyProperty;
			Transform3 nodeTransform = getNodeTransform(node);
			CollObject *collObj = createCollObjectFromMesh(node->getMesh(), node->getScale(), mass, inertia);

			bodyProperty.mass = mass;
			bodyProperty.inertia = inertia;
			bodyProperty.collObject = collObj;

			TrbDynBody *rigidBody = _rigidBodies->createRigidBody(bodyProperty);

			InstanceProperty instProperty;
			instProperty.moveType = MoveTypeActive;
			instProperty.position = nodeTransform.getTranslation();
			instProperty.orientation = Quat(nodeTransform.getUpper3x3());
			instProperty.rigidBody = rigidBody;

			s32 instance = _rigidBodies->createInstance(instProperty);

			IRigidBody *cmbody = new CRigidBody(ERBT_CONVEX_HULL, instance, node, this);
			_rigidBodyList.push_back(cmbody);

			return cmbody;
		}

		IRigidBody* CPhysicsManager::createRigidBodyConvexHull(IAnimatedMeshSceneNode *node, f32 mass)
		{
			Matrix3 inertia;
			RigidBodyProperty bodyProperty;
			Transform3 nodeTransform = getNodeTransform(node);
			CollObject *collObj = createCollObjectFromMesh(node->getMesh(), node->getScale(), mass, inertia);

			bodyProperty.mass = mass;
			bodyProperty.inertia = inertia;
			bodyProperty.collObject = collObj;

			TrbDynBody *rigidBody = _rigidBodies->createRigidBody(bodyProperty);

			InstanceProperty instProperty;
			instProperty.moveType = MoveTypeActive;
			instProperty.position = nodeTransform.getTranslation();
			instProperty.orientation = Quat(nodeTransform.getUpper3x3());
			instProperty.rigidBody = rigidBody;

			s32 instance = _rigidBodies->createInstance(instProperty);

			IRigidBody *cmbody = new CRigidBody(ERBT_CONVEX_HULL, instance, node, this);
			_rigidBodyList.push_back(cmbody);

			return cmbody;
		}

		void CPhysicsManager::updateRigidBodiesScene()
		{
			for(u32 i=0;i < _rigidBodyList.size();i++) {
				IRigidBody *cmbody = _rigidBodyList[i];
				cmbody->updateScene();
			}
		}

		Transform3 CPhysicsManager::getNodeTransform(ISceneNode *node)
		{
			return Transform3(Matrix3::rotationZYX(node->getRotation()*core::DEGTORAD), node->getPosition());
		}

		CollObject* CPhysicsManager::createCollObjectFromMesh(IMesh *mesh, const core::vector3df& scale, f32 mass, Matrix3& inertia)
		{
			core::array< core::vector3df > scaledVertices;
			u32 mbCount = mesh->getMeshBufferCount();
			CollObject *collObj = _rigidBodies->createCollObject(mbCount);
			ConvexMesh *chMesh = (ConvexMesh*)memalign(128, sizeof(ConvexMesh));

			for(u32 i=0;i < mbCount; i++) {
				IMeshBuffer *mb = mesh->getMeshBuffer(i);

				if(mb->getVertexType() == video::EVT_STANDARD) {
					video::S3DVertexStandard *vertices = (video::S3DVertexStandard*)mb->getVertices();
					u16 *indices = mb->getIndices();
					u32 numIndices = mb->getIndexCount();

					for(u32 j=0;j < numIndices/3;j++) {
						scaledVertices.push_back(mulPerElem(vertices[indices[(j*3) + 0]].pos, scale));
						scaledVertices.push_back(mulPerElem(vertices[indices[(j*3) + 1]].pos, scale));
						scaledVertices.push_back(mulPerElem(vertices[indices[(j*3) + 2]].pos, scale));
					}

				}
			}

			createConvexHull(*chMesh, scaledVertices.pointer(), scaledVertices.size());
			calcInertiaMesh(*chMesh, mass, inertia);

			collObj->addConvex(chMesh, Transform3::identity());
			collObj->finish();

			return collObj;
		}

		void CPhysicsManager::createConvexMesh(ConvexMesh& mesh, core::vector3df *vertices, s32 numVertices, u16 *indices, s32 numIndices)
		{
			core::array< core::vector3df > newVerts(numVertices);
			memcpy(newVerts.pointer(), vertices, sizeof(core::vector3df)*numVertices);

			core::array< u16 > newIndices(numIndices);
			memcpy(newIndices.pointer(), indices, sizeof(u16)*numIndices);

			s32 newNumVerts = numVertices;
			s32 newNumIndices = numIndices;

			shrinkVertices(newVerts.pointer(), newNumVerts, newIndices.pointer(), newNumIndices);

			for(s32 i=newVerts.size() - 1;i >= newNumVerts;--i)
				newVerts.erase(i);
			for(s32 i=newIndices.size() - 1;i >= newNumIndices;--i)
				newIndices.erase(i);

			ASSERT(newNumVerts <= NUMMESHVERTICES);
			ASSERT(newNumIndices <= NUMMESHFACETS*3);

			mesh.numVerts = newNumVerts;
			for(u32 i=0;i < mesh.numVerts;i++)
				mesh.verts[i] = newVerts[i];

			mesh.numIndices = newNumIndices;
			for(u32 i=0;i < mesh.numIndices;i++)
				mesh.indices[i] = newIndices[i];

			mesh.updateAABB();
		}

		void CPhysicsManager::createConvexHull(ConvexMesh& mesh, core::vector3df *vertices, s32 numVertices)
		{
			CHMesh testMesh;
			E_KDOP_TYPE dop[3] = { EKDPT_26DOP, EKDPT_14DOP, EKDPT_6DOP };
			s32 n = 0;

			do {
				core::array< CHPolygon > polys;
				createConvexPolygons(dop[n++], vertices, numVertices, polys);
				convertPolygonToMesh(polys, testMesh);
			} while(testMesh.vertices.size() > NUMMESHVERTICES*3 || testMesh.indices.size() > NUMMESHFACETS*3);

			ASSERT(testMesh.vertices.size() <= NUMMESHVERTICES*3);
			ASSERT(testMesh.indices.size() <= NUMMESHFACETS*3);

			createConvexMesh(mesh, testMesh.vertices.pointer(), testMesh.vertices.size()/3, testMesh.indices.pointer(), testMesh.indices.size());
		}

		void CPhysicsManager::createConvexPolygons(E_KDOP_TYPE type, core::vector3df *vertices, s32 numVertices, core::array< CHPolygon >& polys)
		{
			f32 kdMin[13], kdMax[13];

			for(u32 i=0;i < NUM_KDOP;i++) {
				f32 vmin = FLT_MAX, vmax = -FLT_MAX;
				for(u32 j=0;j < (u32)numVertices;j++) {
					f32 proj = dot(vertices[j], VECTOR_KDOP_TABLE[i]);
					if(proj < vmin) vmin = proj;
					if(proj > vmax) vmax = proj;
				}
				kdMin[i] = vmin;
				kdMax[i] = vmax;
			}

			polys.clear();

			core::vector3df boxMin(kdMin[0], kdMin[1], kdMin[2]);
			core::vector3df boxMax(kdMax[0], kdMax[1], kdMax[2]);

			{ // -X
				CHPolygon poly;
				poly.plane = CHPlane(-VECTOR_KDOP_TABLE[0], kdMin[0]*VECTOR_KDOP_TABLE[0]);
				poly.vertices.push_back(Vector3(kdMin[0], kdMin[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMin[0], kdMin[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMin[0], kdMax[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMin[0], kdMax[1], kdMin[2]));
				polys.push_back(poly);
			}
			{ // +X
				CHPolygon poly;
				poly.plane = CHPlane(VECTOR_KDOP_TABLE[0], kdMax[0]*VECTOR_KDOP_TABLE[0]);
				poly.vertices.push_back(Vector3(kdMax[0], kdMin[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMax[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMax[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMin[1], kdMax[2]));
				polys.push_back(poly);
			}
			{ // -Y
				CHPolygon poly;
				poly.plane = CHPlane(-VECTOR_KDOP_TABLE[1], kdMin[1]*VECTOR_KDOP_TABLE[1]);
				poly.vertices.push_back(Vector3(kdMin[0], kdMin[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMin[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMin[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMin[0], kdMin[1], kdMax[2]));
				polys.push_back(poly);
			}
			{ // +Y
				CHPolygon poly;
				poly.plane = CHPlane(VECTOR_KDOP_TABLE[1], kdMax[1]*VECTOR_KDOP_TABLE[1]);
				poly.vertices.push_back(Vector3(kdMin[0], kdMax[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMin[0], kdMax[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMax[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMax[1], kdMin[2]));
				polys.push_back(poly);
			}
			{ // -Z
				CHPolygon poly;
				poly.plane = CHPlane(-VECTOR_KDOP_TABLE[2], kdMin[2]*VECTOR_KDOP_TABLE[2]);
				poly.vertices.push_back(Vector3(kdMin[0], kdMin[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMin[0], kdMax[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMax[1], kdMin[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMin[1], kdMin[2]));
				polys.push_back(poly);
			}
			{ // +Z
				CHPolygon poly;
				poly.plane = CHPlane(VECTOR_KDOP_TABLE[2], kdMax[2]*VECTOR_KDOP_TABLE[2]);
				poly.vertices.push_back(Vector3(kdMin[0], kdMin[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMin[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMax[0], kdMax[1], kdMax[2]));
				poly.vertices.push_back(Vector3(kdMin[0], kdMax[1], kdMax[2]));
				polys.push_back(poly);
			}

			for(u32 k=3;k < (u32)type;k++) {
				CHPlane planes[2] =
				{
					CHPlane(-VECTOR_KDOP_TABLE[k], kdMin[k]*VECTOR_KDOP_TABLE[k]),
					CHPlane( VECTOR_KDOP_TABLE[k], kdMax[k]*VECTOR_KDOP_TABLE[k]),
				};

				for(u32 p=0;p < 2;p++) {
					core::array< u32 > removeIndices;
					core::array< core::vector3df > gatherVertices;

					removeIndices.reallocate(polys.size());
					for(u32 i=0;i < polys.size();i++) {
						s32 ret = slicePolygonByPlane(polys[i], planes[p], gatherVertices);
						if(ret == -1)
							removeIndices.push_back(i);
					}
					for(u32 i=0;i < removeIndices.size();i++)
						polys.erase(removeIndices[i]);

					sortGatherVertsByPlane(gatherVertices, planes[p]);
					if(gatherVertices.size() >= 3) {
						CHPolygon poly;
						poly.plane = planes[p];
						poly.vertices = core::array< core::vector3df >(gatherVertices);
						polys.push_back(poly);
					}
				}
			}

			{
				core::array< u32 > removeIndices;

				removeIndices.reallocate(polys.size());

				for(u32 i=0;i < polys.size();i++) {
					f32 area = calcPolygonArea(polys[i]);
					if(area < CONVEX_HULL_EPSILON)
						removeIndices.push_back(i);
				}
				for(u32 i=0;i < removeIndices.size();i++)
					polys.erase(removeIndices[i]);
			}
		}

		void CPhysicsManager::convertPolygonToMesh(core::array< CHPolygon >& polys, CHMesh& mesh)
		{
			mesh.vertices.clear();
			mesh.indices.clear();

			for(u32 i=0;i < polys.size();i++) {
				u16 vi0 = mesh.getVertexId(polys[i].vertices[0]);
				for(u32 j=1;j < polys[i].vertices.size() - 1;j++) {
					u16 vi1 = mesh.getVertexId(polys[i].vertices[j + 0]);
					u16 vi2 = mesh.getVertexId(polys[i].vertices[j + 1]);
					mesh.addTriangle(vi0, vi1, vi2);
				}
			}
		}

		void CPhysicsManager::removeSameVertex(core::array< core::vector3df >& vertices)
		{
			if(vertices.empty()) return;

			core::array< u32 > removeIndices;

			removeIndices.reallocate(vertices.size());

			core::vector3df last = vertices[0];
			for(u32 i=1;i < vertices.size();i++) {
				if(length(last - vertices[i]) < CONVEX_HULL_EPSILON)
					removeIndices.push_back(i);

				last = vertices[i];
			}

			for(u32 i=0;i < removeIndices.size();i++)
				vertices.erase(removeIndices[i]);
		}

		s32 CPhysicsManager::slicePolygonByPlane(CHPolygon& poly, CHPlane& plane, core::array< core::vector3df >& gatherVertices)
		{
			u32 sliceEdgeId[2];
			u32 idcnt = 0;
			u32 upper = 0;
			u32 lower = 0;

			for(u32 i=0;i < poly.vertices.size();i++) {
				core::vector3df e1 = poly.vertices[i];
				core::vector3df e2 = poly.vertices[(i + 1)%poly.vertices.size()];

				f32 d1 = plane.onPlane(e1);
				f32 d2 = plane.onPlane(e2);

				if(d1*d2 < 0.0f || d1 == 0.0f)
					sliceEdgeId[idcnt++] = i;

				if(d1 > 0.0f)
					upper++;
				else
					lower++;
			}

			if(upper == poly.vertices.size()) return -1;
			if(lower == poly.vertices.size()) return -2;

			ASSERT(idcnt < 3);

			core::vector3df insertVtx[2];

			for(u32 i=0;i < idcnt;i++) {
				core::vector3df e1 = poly.vertices[sliceEdgeId[i]];
				core::vector3df e2 = poly.vertices[(sliceEdgeId[i] + 1)%poly.vertices.size()];

				core::vector3df dir = e2 - e1;
				f32 t = (plane.d - dot(plane.N, e1))/dot(plane.N, dir);

				if( t >= 0.0f && t <= 1.0f)
					insertVtx[i] = e1 + t*dir;
				else
					ASSERT(0);
			}

			core::array< core::vector3df > newVerts;

			for(u32 i=0;i < poly.vertices.size();i++) {
				f32 d = plane.onPlane(poly.vertices[i]);

				if(d <= 0.0f)
					newVerts.push_back(poly.vertices[i]);

				if(idcnt > 0 && sliceEdgeId[0] == i) {
					newVerts.push_back(insertVtx[0]);
					gatherVertices.push_back(insertVtx[0]);
				} else if(idcnt > 1 && sliceEdgeId[1] == i) {
					newVerts.push_back(insertVtx[1]);
					gatherVertices.push_back(insertVtx[1]);
				}
			}

			removeSameVertex(newVerts);

			poly.vertices = newVerts;

			if(poly.vertices.size() < 3) return -1;

			return 0;
		}

		void CPhysicsManager::sortGatherVertsByPlane(core::array< core::vector3df >& gatherVertices, CHPlane& plane)
		{
			if(gatherVertices.empty()) return;

			core::array< core::vector3df > newVerts;
			{
				f32 mint = FLT_MAX;
				s32 minId = 0;
				for(u32 i=0;i < gatherVertices.size();i++) {
					f32 t = dot(plane.S, gatherVertices[i]);
					if(t < mint) {
						mint = t;
						minId = i;
					}
				}

				core::vector3df minVtx = gatherVertices[minId];
				newVerts.push_back(minVtx);
				gatherVertices.erase(minId);

				while(!gatherVertices.empty()) {
					f32 minAng = FLT_MAX;
					for(u32 i=0;i < gatherVertices.size();i++) {
						core::vector3df v = gatherVertices[i] - minVtx;
						f32 lenSqr = lengthSqr(v);
						if(lenSqr < CONVEX_HULL_EPSILON*CONVEX_HULL_EPSILON) {
							minId = i;
							break;
						}

						f32 ang = dot(v/sqrtf(lenSqr), plane.T);
						if(ang < minAng) {
							minAng = ang;
							minId = i;
						}
					}
					newVerts.push_back(gatherVertices[minId]);
					gatherVertices.erase(minId);
				}
			}

			removeSameVertex(newVerts);

			gatherVertices = newVerts;
		}

		f32 CPhysicsManager::calcPolygonArea(CHPolygon& poly)
		{
			if(poly.vertices.size() < 3)
				return 0.0f;

			f32 area = 0.0f;
			core::vector3df p0 = poly.vertices[0];
			for(u32 i=1;i < poly.vertices.size()-1;i++) {
				core::vector3df p1 = poly.vertices[i + 0];
				core::vector3df p2 = poly.vertices[i + 1];
				core::vector3df v = cross(p2 - p0, p1 - p0);

				f32 lenSqr = lengthSqr(v);
				if(lenSqr > CONVEX_HULL_EPSILON*CONVEX_HULL_EPSILON)
					area += sqrtf(lenSqr);
			}

			return area;
		}

		void CPhysicsManager::shrinkVertices(core::vector3df *vertices, s32& numVertices, u16 *indices, s32& numIndices)
		{
			if(numVertices == 0 || numIndices == 0) return;

			for(s32 i=0;i < numIndices/3;i++) {
				Vector3 pnts[3] =
				{
					vertices[indices[i*3 + 0]],
					vertices[indices[i*3 + 1]],
					vertices[indices[i*3 + 2]]
				};

				f32 area = lengthSqr(cross(pnts[1] - pnts[0], pnts[2] - pnts[0]));

				if(area < 0.00001f) {
					for(s32 j=i + 1;j < numIndices/3;j++) {
						indices[(j - 1)*3 + 0] = indices[j*3 + 0];
						indices[(j - 1)*3 + 1] = indices[j*3 + 1];
						indices[(j - 1)*3 + 2] = indices[j*3 + 2];
					}
					numIndices -= 3;
					i--;
				}
			}

			for(s32 i=0;i < numVertices - 1;i++) {
				for(s32 j=i + 1;j < numVertices;j++) {
					f32 lenSqr = lengthSqr(vertices[i] - vertices[j]);

					if(lenSqr < 0.00001f) {
						for(s32 k=0;k < numIndices;k++) {
							if(indices[k] == j)
								indices[k] = i;
							else if(indices[k] > j)
								indices[k]--;
						}

						for(s32 k=j;k < numVertices - 1;k++)
							vertices[k] = vertices[k + 1];

						numVertices--;
						j--;
					}
				}
			}
		}

		void CPhysicsManager::onContact(ContactPair& pair)
		{

		}

		void CPhysicsManager::onSleep(u16 stateIndex)
		{

		}

		void CPhysicsManager::onActive(u16 stateIndex)
		{

		}
	}
}



