/*
 * billboardscenenode.cpp
 *
 *  Created on: Feb 2, 2014
 *      Author: mike
 */

#include "ivideodriver.h"
#include "iscenemanager.h"
#include "icamerascenenode.h"
#include "billboardscenenode.h"

#define _VECTORMATH_PERM_YXZW (vec_uchar16)(vec_uint4){ _VECTORMATH_PERM_Y, _VECTORMATH_PERM_X, _VECTORMATH_PERM_Z, _VECTORMATH_PERM_W }

namespace irr
{
	namespace scene
	{
		CBillboardSceneNode::CBillboardSceneNode(ISceneNode *parent, ISceneManager *mgr, const core::vector3df& pos, const core::dimension2df& size, video::SColor topColor, video::SColor bottomColor)
		: IBillboardSceneNode(parent, mgr, pos), _vertices((video::S3DVertexStandard*)IHeapManager::allocate(64, sizeof(video::S3DVertexStandard)*4))
		{
			setSize(size);

			_indices[0] = 0;
			_indices[1] = 1;
			_indices[2] = 2;
			_indices[3] = 2;
			_indices[4] = 3;
			_indices[5] = 0;

			_vertices[0].tcoords = core::vector2df(1.0f, 1.0f);
			_vertices[0].col = bottomColor;

			_vertices[1].tcoords = core::vector2df(1.0f, 0.0f);
			_vertices[1].col = topColor;

			_vertices[2].tcoords = core::vector2df(0.0f, 0.0f);
			_vertices[2].col = topColor;

			_vertices[3].tcoords = core::vector2df(0.0f, 1.0f);
			_vertices[3].col = bottomColor;
		}

		CBillboardSceneNode::~CBillboardSceneNode()
		{
			IHeapManager::deallocate(_vertices);
		}

		void CBillboardSceneNode::onRegisterSceneNode()
		{
			if(_isVisible)
				_sceneManager->registerNodeForRendering(this);

			ISceneNode::onRegisterSceneNode();
		}

		void CBillboardSceneNode::render()
		{
			video::IVideoDriver *driver = _sceneManager->getVideoDriver();
			ICameraSceneNode *camera = _sceneManager->getActiveCamera();

			if(camera == NULL || driver == NULL)
				return;

			core::vector3df pos = getAbsolutePosition();

			core::vector3df campos = camera->getAbsolutePosition();
			core::vector3df target = camera->getTarget();
			core::vector3df up = camera->getUpVector();
			core::vector3df view = normalize(target - campos);
			core::vector3df horizontal = cross(up, view);

			if(length(horizontal) == 0)
				horizontal = core::vector3df(vec_perm(up.get128(), up.get128(), _VECTORMATH_PERM_YXZW));

			horizontal = normalize(horizontal);

			core::vector3df topHorizontal = horizontal*_topEdgeWidth*0.5f;

			horizontal *= _size.width*0.5f;

			core::vector3df vertical = normalize(cross(horizontal, view));
			vertical *= _size.height*0.5f;

			view *= -1.0f;
			for(s32 i=0;i < 4;i++)
				_vertices[i].nrm = view;

			_vertices[0].pos = pos + horizontal + vertical;
			_vertices[1].pos = pos + topHorizontal - vertical;
			_vertices[2].pos = pos - topHorizontal - vertical;
			_vertices[3].pos = pos - horizontal + vertical;

			driver->setTransform(video::ETS_WORLD, core::identityMatrix);
			driver->setMaterial(_material);
			driver->drawIndexedTriangleList(_vertices, 4, _indices, 2);
		}

		const core::aabbox3df& CBillboardSceneNode::getBoundingBox() const
		{
			return _bbox;
		}

		void CBillboardSceneNode::setSize(const core::dimension2df& size)
		{
			_size = size;

			if(core::equals(_size.width, 0.0f))
				_size.width = 1.0f;
			_topEdgeWidth = _size.width;

			if(core::equals(_size.height, 0.0f))
				_size.height = 1.0f;

			const f32 avg = (_size.width + _size.height)/6;
			_bbox.minEdge = core::vector3df(-avg, -avg, -avg);
			_bbox.maxEdge = core::vector3df(avg, avg, avg);
		}

		void CBillboardSceneNode::setSize(f32 height, f32 bottomEdgeWidth, f32 topEdgeWidth)
		{
			_size.set(bottomEdgeWidth, height);
			_topEdgeWidth = topEdgeWidth;

			if(core::equals(_size.height, 0.0f))
				_size.height = 1.0f;

			if(core::equals(_size.width, 0.0f) && core::equals(_topEdgeWidth, 0.0f)) {
				_size.width = 1.0f;
				_topEdgeWidth = 1.0f;
			}

			const f32 avg = (core::max_(_size.width, _topEdgeWidth) + _size.height)/6;
			_bbox.minEdge = core::vector3df(-avg, -avg, -avg);
			_bbox.maxEdge = core::vector3df(avg, avg, avg);
		}

		video::SMaterial& CBillboardSceneNode::getMaterial(u32 i)
		{
			return _material;
		}

		u32 CBillboardSceneNode::getMaterialCount() const
		{
			return 1;
		}

		const core::dimension2df& CBillboardSceneNode::getSize() const
		{
			return _size;
		}

		void CBillboardSceneNode::getSize(f32& height, f32& bottomEdgeWidth, f32& topEdgeWidth) const
		{
			height = _size.height;
			bottomEdgeWidth = _size.width;
			topEdgeWidth = _topEdgeWidth;
		}

		void CBillboardSceneNode::setColor(const video::SColor& overallColor)
		{
			for(u32 i=0;i < 4;i++)
				_vertices[i].col = overallColor;
		}

		void CBillboardSceneNode::setColor(const video::SColor& topColor, const video::SColor& bottomColor)
		{
			_vertices[0].col = bottomColor;
			_vertices[1].col = topColor;
			_vertices[2].col = topColor;
			_vertices[3].col = bottomColor;
		}

		void CBillboardSceneNode::getColor(video::SColor& topColor, video::SColor& bottomColor) const
		{
			bottomColor = _vertices[0].col;
			topColor = _vertices[1].col;
		}
	}
}


