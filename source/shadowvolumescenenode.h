/*
 * shadowvolumescenenode.h
 *
 *  Created on: Jul 16, 2013
 *      Author: mike
 */

#ifndef SHADOWVOLUMESCENENODE_H_
#define SHADOWVOLUMESCENENODE_H_

#include "ishadowvolumescenenode.h"
#include "s3dvertex.h"

#include <vector>

namespace irr
{
	namespace scene
	{
		class CShadowVolumeSceneNode : public IShadowVolumeSceneNode
		{
			struct Edge
			{
				core::vector3df v1;
				core::vector3df v2;
				core::vector3df normal;

				Edge(const core::vector3df& p1, const core::vector3df& p2, const core::vector3df& nrm) : v1(p1), v2(p2), normal(nrm) {}

				bool operator == (const Edge& other) const
				{
					return (v1 == other.v1 && v2 == other.v2) || (v1 == other.v2 && v2 == other.v1);
				}
			};

		public:
			CShadowVolumeSceneNode(const IMesh *shadowMesh, ISceneNode *parent, ISceneManager *mgr, bool zFailMethod = true, f32 infinity = 10000.0f);
			virtual ~CShadowVolumeSceneNode();

			virtual void setShadowMesh(const IMesh *mesh);

			virtual void onRegisterSceneNode();

			virtual void render();

			virtual const core::aabbox3df& getBoundingBox() const;

			virtual ESCENE_NODE_TYPE getType() const { return ESNT_SHADOW_VOLUME; }

		private:
			void createShadowVolumeMesh();
			void processEdge(core::array< Edge >& pairs, const Edge& e);

			core::array< video::S3DVertexBase, core::irrheapallocator< video::S3DVertexBase > > _shadowVolume;

			core::aabbox3df _bbox;

			const scene::IMesh *_shadowMesh;

			f32 _infinity;

			bool _useZFailMethod;
		};
	}
}

#endif /* SHADOWVOLUMESCENENODE_H_ */
