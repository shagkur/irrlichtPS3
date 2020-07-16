/*
 * billboardscenenode.h
 *
 *  Created on: Feb 2, 2014
 *      Author: mike
 */

#ifndef BILLBOARDSCENENODE_H_
#define BILLBOARDSCENENODE_H_

#include "ibillboardscenenode.h"
#include "s3dvertex.h"

namespace irr
{
	namespace scene
	{
		class CBillboardSceneNode : public IBillboardSceneNode
		{
		public:
			CBillboardSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, const core::dimension2df& size, video::SColor topColor = video::SColor(0xffffffff), video::SColor bottomColor = video::SColor(0xffffffff));
			virtual ~CBillboardSceneNode();

			virtual void onRegisterSceneNode();
			virtual void render();

			virtual const core::aabbox3df& getBoundingBox() const;

			virtual void setSize(const core::dimension2df& size);
			virtual void setSize(f32 height, f32 bottomEdgeWidth, f32 topEdgeWidth);

			virtual const core::dimension2df& getSize() const;
			virtual void getSize(f32& height, f32& bottomEdgeWidth, f32& topEdgeWidth) const;

			virtual video::SMaterial& getMaterial(u32 i);
			virtual u32 getMaterialCount() const;

			virtual void setColor(const video::SColor& overallColor);
			virtual void setColor(const video::SColor& topColor, const video::SColor& bottomColor);

			virtual void getColor(video::SColor& topColor, video::SColor& bottomColor) const;

			virtual ESCENE_NODE_TYPE getType() const { return ESNT_BILLBOARD; }

		private:
			core::dimension2df _size;
			f32 _topEdgeWidth;
			core::aabbox3df _bbox;
			video::SMaterial _material;

			video::S3DVertexStandard *_vertices;
			u16 _indices[6];
		};
	}
}

#endif /* BILLBOARDSCENENODE_H_ */
