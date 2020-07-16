/*
 * imeshscenenode.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef IMESHSCENENODE_H_
#define IMESHSCENENODE_H_

#include "iscenenode.h"

namespace irr
{
	namespace scene
	{
		class IMesh;
		class IShadowVolumeSceneNode;

		class IMeshSceneNode : public ISceneNode
		{
		public:
			IMeshSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, const core::vector3df& rot, const core::vector3df& scale)
			: ISceneNode(parent, mgr, pos, rot, scale) {}

			virtual void setMesh(IMesh *mesh) = 0;
			virtual IMesh* getMesh() = 0;

			virtual IShadowVolumeSceneNode* addShadowVolumeSceneNode(const IMesh *shadowMesh = NULL, bool zFailMethod = true, f32 infinity = 10000.0f) = 0;

			virtual void setReadOnlyMaterials(bool readonly) = 0;
			virtual bool isReadOnlyMaterials() const = 0;
		};
	}
}

#endif /* IMESHSCENENODE_H_ */
