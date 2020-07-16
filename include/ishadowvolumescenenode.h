/*
 * ishadowvolumescenenode.h
 *
 *  Created on: Jul 16, 2013
 *      Author: mike
 */

#ifndef ISHADOWVOLUMESCENENODE_H_
#define ISHADOWVOLUMESCENENODE_H_

#include "iscenenode.h"

namespace irr
{
	namespace scene
	{
		class IMesh;

		class IShadowVolumeSceneNode : public ISceneNode
		{
		public:
			IShadowVolumeSceneNode(ISceneNode *parent, ISceneManager *mgr) : ISceneNode(parent, mgr) {}

			virtual void setShadowMesh(const IMesh *mesh) = 0;
		};
	}
}

#endif /* ISHADOWVOLUMESCENENODE_H_ */
