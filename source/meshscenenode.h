/*
 * meshscenenode.h
 *
 *  Created on: Feb 25, 2013
 *      Author: mike
 */

#ifndef MESHSCENENODE_H_
#define MESHSCENENODE_H_

#include "imeshscenenode.h"
#include "imesh.h"

namespace irr
{
	namespace scene
	{
		class CMeshSceneNode : public IMeshSceneNode
		{
		public:
			CMeshSceneNode(IMesh *mesh, ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos = core::vector3df(0, 0, 0), const core::vector3df& rot = core::vector3df(0, 0, 0), const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f));
			virtual ~CMeshSceneNode();

			virtual void onRegisterSceneNode();

			virtual void render();

			virtual const core::aabbox3df& getBoundingBox() const;

			virtual video::SMaterial& getMaterial(u32 i);

			virtual u32 getMaterialCount() const;

			virtual ESCENE_NODE_TYPE getType() const { return ESNT_MESH; }

			virtual void setMesh(IMesh *mesh);

			virtual IMesh* getMesh() { return _mesh; }

			virtual IShadowVolumeSceneNode* addShadowVolumeSceneNode(const IMesh* shadowMesh = NULL, bool zFailMethod = true, f32 infinity = 10000.0f);


			virtual void setReadOnlyMaterials(bool readonly);

			virtual bool isReadOnlyMaterials() const;

			virtual bool removeChild(ISceneNode *child);

		protected:
			void copyMaterials();

			core::array< video::SMaterial > _materials;
			core::aabbox3df _bbox;
			video::SMaterial _readOnlyMaterial;

			IMesh *_mesh;
			IShadowVolumeSceneNode *_shadow;

			s32 _passCount;
			bool _readOnlyMaterials;
		};
	}
}

#endif /* MESHSCENENODE_H_ */
