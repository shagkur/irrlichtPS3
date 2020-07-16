#ifndef __NULLDRIVER_H__
#define __NULLDRIVER_H__

#include "ivideodriver.h"
#include "ifilesystem.h"
#include "irrmap.h"
#include "irrarray.h"
#include "imesh.h"
#include "imeshbuffer.h"
#include "imeshscenenode.h"
#include "imaterialrenderer.h"
#include "igpuprogrammingservices.h"
#include "slight.h"
#include "fpscounter.h"

#define SOBEL_KERNEL3x3			0
#define SOBEL_KERNEL5x5			1

namespace irr
{
	namespace video
	{
		class CNullDriver : public IVideoDriver, public IGPUProgrammingServices
		{
		public:
			CNullDriver(io::IFileSystem *fs);
			virtual ~CNullDriver();

			virtual bool beginScene(bool backBuffer = true,bool zBuffer = true, SColor color = SColor(0, 0, 0, 255));
			virtual bool endScene();

			virtual ECOLOR_FORMAT getColorFormat() const;

			virtual const core::dimension2d<u32>& getScreenSize() const;
			virtual const core::dimension2d<u32>& getCurrentRenderTargetSize() const;

			virtual void setTransform(E_TRANSFORMATION_STATE state,const core::matrix4& mat);
			virtual const core::matrix4& getTransform(E_TRANSFORMATION_STATE state) const;
			
			virtual void setMaterial(const SMaterial& material);

			virtual ITexture* getTexture(const io::path& filename);
			virtual ITexture* getTexture(io::IReadFile *file);
			virtual ITexture* findTexture(const io::path& filename);
			virtual ITexture* addTexture(const core::dimension2d<u32>& size, const io::path& name, ECOLOR_FORMAT format = ECF_A8R8G8B8);
			virtual u32 getTextureCount() const;

			virtual void deleteAllDynamicLights();
			virtual s32 addDynamicLight(const SLight& light);
			virtual void turnLightOn(s32 lightIndex, bool turnOn);
			virtual u32 getMaximalDynamicLightAmount() const;
			virtual u32 getDynamicLightCount() const;
			virtual const SLight& getDynamicLight(u32 idx) const;

			virtual f32 getFPS() const;
			virtual u32 getPrimitiveCountDrawn(u32 mode = 0) const;

			virtual u32 getMaximalPrimitiveCount() const;

			virtual s32 addMaterialRenderer(IMaterialRenderer *renderer, const char *cname = NULL);
			virtual IMaterialRenderer* getMaterialRenderer(u32 nr);

			virtual void setViewPort(const core::rect<s32>& area);
			virtual const core::rect<s32>& getViewPort() const;

			virtual void drawMeshBuffer(const scene::IMeshBuffer *mb);

			virtual void drawVertexPrimitiveList(const void *vertices, u32 vertexCount, const void *indexList, u32 primitiveCount, E_VERTEX_TYPE vType = EVT_STANDARD, scene::E_PRIMITIVE_TYPE pType = scene::EPT_TRIANGLES, E_INDEX_TYPE iType = EIT_16BIT);

			virtual void drawStencilShadowVolume(const void *vertices, u32 vertexCount, const core::vector4df& lightPos, f32 shadowExtDist = 10000.0f, bool zFail = true);

			virtual bool isHardwareBufferRecommended(const scene::IMeshBuffer *mb);

			virtual void addOcclusionQuery(scene::ISceneNode *node, const scene::IMesh *mesh = NULL);
			virtual void removeOcclusionQuery(scene::ISceneNode *node);
			virtual void runOcclusionQuery(scene::ISceneNode *node, bool visible = false);

			virtual void setTextureCreationFlag(E_TEXTURE_CREATION_FLAG flag, bool enabled = true);
			virtual bool getTextureCreationFlag(E_TEXTURE_CREATION_FLAG flag) const;

			virtual IImage* createImageFromFile(const io::path& filename);
			virtual IImage* createImageFromFile(io::IReadFile *file);
			virtual IImage* createImageFromData(ECOLOR_FORMAT format, const core::dimension2d<u32>& size, void *data, bool ownForeignMemory = false, bool deleteMemory = true);
			virtual IImage* createImage(ECOLOR_FORMAT format, const core::dimension2d<u32>& size);

			virtual bool setRenderTarget(video::E_RENDER_TARGET target, bool clearTarget, bool clearZBuffer, SColor color);

			virtual s32 addHighLevelShaderMaterial(const void *vertexShaderProgram, const void *fragmentShaderProgram, IShaderConstantSetCallback *callback = NULL, E_MATERIAL_TYPE baseMaterial = video::EMT_SOLID, s32 userData = 0);
			virtual s32 addHighLevelShaderMaterial(const io::path& vertexShaderProgram, const io::path& fragmentShaderProgram, IShaderConstantSetCallback *callback = NULL, E_MATERIAL_TYPE baseMaterial = video::EMT_SOLID, s32 userData = 0);
			virtual s32 addHighLevelShaderMaterial(io::IReadFile *vertexShaderProgram, io::IReadFile *fragmentShaderProgram, IShaderConstantSetCallback *callback = NULL, E_MATERIAL_TYPE baseMaterial = video::EMT_SOLID, s32 userData = 0);

			virtual void makeColorKeyTexture(video::ITexture *texture, video::SColor color) const;
			virtual void makeColorKeyTexture(video::ITexture *texture, core::position2di colorKeyPixelPos) const;
			virtual void makeNormalMapTexture(video::ITexture *texture, f32 amplitude = 1.0f) const;

			virtual void setStencilShadowEnabled(bool enable);
			virtual bool isStencilShadowEnabled() const;

			virtual scene::IMeshManipulator* getMeshManipulator();

			virtual IGPUProgrammingServices* getGPUProgrammingServices();

			virtual mars_context* getMARSContext() { return NULL; }

			virtual E_DRIVER_TYPE getDriverType() const { return EDT_NULL; }

		protected:
			ITexture* loadTextureFromFile(io::IReadFile *file, const io::path& hashName = "");

			struct SHWBufferLink
			{
				SHWBufferLink(const scene::IMeshBuffer *mb)
				: meshBuffer(mb), changedID_Vertex(0), changedID_Index(0), lastUsed(0),
				  mapping_Vertex(scene::EHM_NEVER), mapping_Index(scene::EHM_NEVER)
				{
					if(meshBuffer) meshBuffer->grab();
				}

				virtual ~SHWBufferLink()
				{
					if(meshBuffer) meshBuffer->drop();
				}

				const scene::IMeshBuffer *meshBuffer;
				u32 changedID_Vertex;
				u32 changedID_Index;
				u32 lastUsed;
				scene::E_HARDWARE_MAPPING mapping_Vertex;
				scene::E_HARDWARE_MAPPING mapping_Index;
			};

			struct SSobelKernelElement
			{
				s32 x, y;
				f32 w;
			};

			virtual SHWBufferLink* getBufferLink(const scene::IMeshBuffer *mb);

			virtual SHWBufferLink* createHardwareBuffer(const scene::IMeshBuffer *mb) { return NULL; }

			virtual bool updateHardwareBuffer(SHWBufferLink *hwBuffer) { return false; }

			virtual void drawHardwareBuffer(SHWBufferLink *hwBuffer) {}

			virtual void deleteHardwareBuffer(SHWBufferLink *hwBuffer);

			s32 addAndDropMaterialRenderer(IMaterialRenderer *m);

			void addTexture(ITexture *texture);

			virtual ITexture* addTexture(const io::path& name, IImage *image);

			virtual video::ITexture* createDeviceDependentTexture(IImage *surface, const io::path& name);

			void generateSobelKernel(s32 type);

			inline f32 nml32(s32 x, s32 y, s32 pitch, s32 height, s32 *p) const
			{
				u32 val = p[(core::max_(0, core::min_(height - 1, y))*pitch) + core::max_(0, core::min_(pitch - 1, x))];
				return video::SColor::fromA8R8G8B8(val).getLuminance()*ONE_OVER_255;
			}

			struct SSurface
			{
				video::ITexture *surface;

				bool operator < (const SSurface& other) const
				{
					return surface->getName() < other.surface->getName();
				}
			};

			struct SMaterialRenderer
			{
				core::stringc name;
				IMaterialRenderer *renderer;
			};

			struct SDummyTexture : public ITexture
			{
				SDummyTexture(const io::path& name) : ITexture(name), size(0, 0) {}

				virtual const core::dimension2d<u32>& getOriginalSize() const { return size; }
				virtual const core::dimension2d<u32>& getSize() const { return size; }

				virtual E_DRIVER_TYPE getDriverType() const { return EDT_NULL; }
				virtual ECOLOR_FORMAT getColorFormat() const { return video::ECF_A1R5G5B5; }

				virtual u32 getPitch() const { return 0; }
				virtual void regenerateMipMapLevels() {}

				virtual void* lock() { return NULL; }
				virtual void unlock() {}

				core::dimension2d<u32> size;
			};

			struct SOccQuery
			{
				SOccQuery(scene::ISceneNode *n, const scene::IMesh *m = NULL)
				: node(n), mesh(m), UID(0)
				{
					if(node != NULL) node->grab();
					if(mesh != NULL) mesh->grab();
				}

				~SOccQuery()
				{
					if(node != NULL) node->drop();
					if(mesh != NULL) mesh->drop();
				}

				bool operator == (const SOccQuery& other) const
				{
					return other.node == node;
				}

				scene::ISceneNode *node;
				const scene::IMesh *mesh;

				u32 UID;
			};

			static constexpr f32 ONE_OVER_255 = 1.0f/255.0f;

			core::array< SOccQuery > _occlusionQueries;
			core::array< SLight > _lights;
			core::array< SSurface > _textures;
			core::array< SMaterialRenderer > _materialRenderers;
			core::array< IImageLoader* > _surfaceLoader;

			core::map< const scene::IMeshBuffer*, SHWBufferLink* > _hwBufferMap;

			io::IFileSystem *_fileSystem;

			scene::IMeshManipulator *_meshManipulator;

			core::rect<s32> _viewPort;
			core::dimension2d<u32> _screenSize;
			core::dimension2d<u32> _mrtSize;
			core::matrix4 _transformationMatrix;

			CFPSCounter _fpsCounter;

			u32 _minVertexCountForVBO;

			u32 _primitivesDrawn;

			u32 _textureCreationFlags;

			bool _stencilShadowEnabled;

			SSobelKernelElement *_sobelKernel[2];
			u32 _numKernelElems;
		};
	}
}

#endif
