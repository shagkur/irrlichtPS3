/*
 * ivideodriver.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef IVIDEODRIVER_H_
#define IVIDEODRIVER_H_

#include "irefcounter.h"
#include "matrix4.h"
#include "scolor.h"
#include "smaterial.h"
#include "irrarray.h"
#include "itexture.h"
#include "imeshbuffer.h"
#include "edrivertypes.h"

namespace irr
{
	namespace io
	{
		class IReadFile;
	}

	namespace scene
	{
		class IMeshBuffer;
		class IMesh;
		class ISceneNode;
		class IMeshManipulator;
	}

	namespace video
	{
		struct SLight;
		struct S3DVertexStandard;
		struct S3DVertexTangents;
		class IImageLoader;
		class IMaterialRenderer;
		class IGPUProgrammingServices;

		enum E_TRANSFORMATION_STATE
		{
			//! View transformation
			ETS_VIEW = 0,
			//! World transformation
			ETS_WORLD,
			//! Projection transformation
			ETS_PROJECTION,
			//! Orthographic transformation
			ETS_ORTHOGRAPHIC,
			//! Texture transformation
			ETS_TEXTURE_0,
			//! Texture transformation
			ETS_TEXTURE_1,
			//! Texture transformation
			ETS_TEXTURE_2,
			//! Texture transformation
			ETS_TEXTURE_3,
			//! Texture transformation
			ETS_TEXTURE_4,
			//! Texture transformation
			ETS_TEXTURE_5,
			//! Texture transformation
			ETS_TEXTURE_6,
			//! Texture transformation
			ETS_TEXTURE_7,
			//! Multiplied modelviewprojection transformation
			ETS_MVP,
			//! Not used
			ETS_COUNT
		};

		enum E_RENDER_TARGET
		{
			ERT_FRAMEBUFFER = 0,

			ERT_AUXBUFFER0,
			ERT_AUXBUFFER1,
			ERT_AUXBUFFER2,
			ERT_AUXBUFFER3,
		};

		enum E_RENDER_CONTEXT
		{
			ERC_OPAQUE = 0,
			ERC_OPAQUE_LIGHTING,
			ERC_TRANSPARENT,
			ERC_TRANSPARENT_LIGHTING,

			ERC_COUNT
		};

		class IVideoDriver : public virtual IRefCounter
		{
		public:
			virtual bool beginScene(bool backBuffer = true,bool zBuffer = true, SColor color = SColor(0, 0, 0, 255)) = 0;

			virtual bool endScene() = 0;

			virtual ECOLOR_FORMAT getColorFormat() const = 0;

			virtual const core::dimension2d<u32>& getScreenSize() const = 0;
			virtual const core::dimension2d<u32>& getCurrentRenderTargetSize() const = 0;

			virtual void setTransform(E_TRANSFORMATION_STATE state,const core::matrix4& mat) = 0;

			virtual const core::matrix4& getTransform(E_TRANSFORMATION_STATE state) const = 0;

			virtual void setMaterial(const SMaterial& material) = 0;

			virtual ITexture* getTexture(const io::path& filename) = 0;
			virtual ITexture* getTexture(io::IReadFile *file) = 0;
			virtual ITexture* findTexture(const io::path& filename) = 0;
			virtual ITexture* addTexture(const io::path& name, IImage *image) = 0;
			virtual ITexture* addTexture(const core::dimension2d<u32>& size, const io::path& name, ECOLOR_FORMAT format = ECF_A8R8G8B8) = 0;
			virtual u32 getTextureCount() const = 0;

			virtual void setAmbientLight(const SColorf& ambientColor) = 0;
			virtual void setCamWorldPos(const core::vector3df& camWorldPos) = 0;

			virtual void setActiveLight(const SLight& light) = 0;
			virtual const SLight& getActiveLight() const = 0;

			virtual void setRenderContext(E_RENDER_CONTEXT context) = 0;
			virtual E_RENDER_CONTEXT getRenderContext() const = 0;

			virtual f32 getFPS() const = 0;
			virtual u32 getPrimitiveCountDrawn(u32 mode = 0) const = 0;

			virtual void deleteAllDynamicLights() = 0;
			virtual s32 addDynamicLight(const SLight& light) = 0;
			virtual u32 getMaximalDynamicLightAmount() const = 0;
			virtual u32 getDynamicLightCount() const = 0;
			virtual const SLight& getDynamicLight(u32 idx) const = 0;
			virtual void turnLightOn(s32 lightIndex, bool turnOn) = 0;

			virtual u32 getMaximalPrimitiveCount() const = 0;

			virtual s32 addMaterialRenderer(IMaterialRenderer *renderer, const char *cname = NULL) = 0;
			virtual IMaterialRenderer* getMaterialRenderer(u32 nr) = 0;

			virtual void setViewPort(const core::rect<s32>& area) = 0;
			virtual const core::rect<s32>& getViewPort() const = 0;

			virtual void drawMeshBuffer(const scene::IMeshBuffer *mb) = 0;

			virtual void drawVertexPrimitiveList(const void *vertices, u32 vertexCount, const void *indexList, u32 primCount, E_VERTEX_TYPE vType = EVT_STANDARD, scene::E_PRIMITIVE_TYPE pType = scene::EPT_TRIANGLES, E_INDEX_TYPE iType = EIT_16BIT) = 0;

			virtual void drawStencilShadowVolume(const void *vertices, u32 vertexCount, const core::vector4df& lightPos, f32 shadowExtDist = 10000.0f, bool zFail = true) = 0;

			void drawIndexedTriangleList(const S3DVertexBase *vertices, u32 vertexCount, const u16 *indices, u32 triangleCount)
			{
				drawVertexPrimitiveList(vertices, vertexCount, indices, triangleCount, EVT_BASE, scene::EPT_TRIANGLES, EIT_16BIT);
			}

			void drawIndexedTriangleList(const S3DVertexStandard *vertices, u32 vertexCount, const u16 *indices, u32 triangleCount)
			{
				drawVertexPrimitiveList(vertices, vertexCount, indices, triangleCount, EVT_STANDARD, scene::EPT_TRIANGLES, EIT_16BIT);
			}

			void drawIndexedTriangleList(const S3DVertex2TCoords *vertices, u32 vertexCount, const u16 *indices, u32 triangleCount)
			{
				drawVertexPrimitiveList(vertices, vertexCount, indices, triangleCount, EVT_2TCOORDS, scene::EPT_TRIANGLES, EIT_16BIT);
			}

			void drawIndexedTriangleList(const S3DVertexTangents *vertices, u32 vertexCount, const u16 *indices, u32 triangleCount)
			{
				drawVertexPrimitiveList(vertices, vertexCount, indices, triangleCount, EVT_TANGENTS, scene::EPT_TRIANGLES, EIT_16BIT);
			}

			virtual void addOcclusionQuery(scene::ISceneNode *node, const scene::IMesh *mesh = NULL) = 0;
			virtual void removeOcclusionQuery(scene::ISceneNode *node) = 0;
			virtual void runOcclusionQuery(scene::ISceneNode *node, bool visible = false) = 0;

			virtual void setTextureCreationFlag(E_TEXTURE_CREATION_FLAG flag, bool enabled = true) = 0;
			virtual bool getTextureCreationFlag(E_TEXTURE_CREATION_FLAG flag) const = 0;

			virtual IImage* createImageFromFile(const io::path& filename) = 0;
			virtual IImage* createImageFromFile(io::IReadFile *file) = 0;
			virtual IImage* createImageFromData(ECOLOR_FORMAT format, const core::dimension2d<u32>& size, void *data, bool ownForeignMemory = false, bool deleteMemory = true) = 0;
			virtual IImage* createImage(ECOLOR_FORMAT format, const core::dimension2d<u32>& size) = 0;

			virtual bool setRenderTarget(E_RENDER_TARGET target, bool clearTarget = true, bool clearZBuffer = true, SColor color = SColor(0, 0, 0, 0)) = 0;

			virtual void makeColorKeyTexture(video::ITexture *texture, video::SColor color) const = 0;
			virtual void makeColorKeyTexture(video::ITexture *texture, core::position2di colorKeyPixelPos) const = 0;
			virtual void makeNormalMapTexture(video::ITexture *texture, f32 amplitude = 1.0f) const = 0;

			virtual void setStencilShadowEnabled(bool enable) = 0;
			virtual bool isStencilShadowEnabled() const = 0;

			virtual scene::IMeshManipulator* getMeshManipulator() = 0;

			virtual IGPUProgrammingServices* getGPUProgrammingServices() = 0;

			virtual mars_context* getMARSContext() = 0;

			virtual E_DRIVER_TYPE getDriverType() const = 0;
		};
	}
}

#endif /* IVIDEODRIVER_H_ */
