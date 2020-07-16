/*
 * rsxdriver.h
 *
 *  Created on: Feb 4, 2013
 *      Author: mike
 */

#ifndef RSXDRIVER_H_
#define RSXDRIVER_H_

#include "nulldriver.h"
#include "imaterialrendererservices.h"
#include "sirrcreationparameters.h"

#include "rsxfragmentshader.h"
#include "rsxvertexshader.h"

#include "debug/gcmdebugfontrenderer.h"


#define BUFFER_IDLE							0
#define BUFFER_BUSY							1

#define FRAME_BUFFER_COUNT					4

namespace irr
{
	class CIrrDevicePS3;
	class IrrlichtDevice;

	namespace video
	{
		class CRSXState;
		class CRSXTexture;
		class CRSXMaterial;
		class CRSXShaderPipeline;
		class CRSXShadowVolumeShaderDriver;

		class CRSXDriver : public CNullDriver, public IMaterialRendererServices
		{
			friend class CRSXTexture;

		public:
			CRSXDriver(const SIrrlichtCreationParameters& param, io::IFileSystem *fs, IrrlichtDevice *device);
			virtual ~CRSXDriver();

			virtual bool beginScene(bool backBuffer = true,bool zBuffer = true, SColor color = SColor(0, 0, 0, 255));
			virtual bool endScene();

			virtual ECOLOR_FORMAT getColorFormat() const;

			virtual const core::dimension2d<u32>& getCurrentRenderTargetSize() const;

			virtual const core::matrix4& getTransform(E_TRANSFORMATION_STATE state) const;
			virtual void setTransform(E_TRANSFORMATION_STATE state,const core::matrix4& mat);

			virtual void setAmbientLight(const SColorf& ambientColor);
			virtual void setCamWorldPos(const core::vector3df& camWorldPos);

			virtual void setActiveLight(const SLight& light);
			virtual const SLight& getActiveLight() const;

			virtual void setRenderContext(E_RENDER_CONTEXT context);
			virtual E_RENDER_CONTEXT getRenderContext() const;

			virtual void setMaterial(const SMaterial& material);

			virtual void deleteAllDynamicLights();
			virtual s32 addDynamicLight(const SLight& light);
			virtual u32 getMaximalDynamicLightAmount() const;
			virtual const SLight& getDynamicLight(u32 idx) const;
			virtual u32 getDynamicLightCount() const;

			virtual u32 getMaximalPrimitiveCount() const;

			virtual void setViewPort(const core::rect<s32>& area);

			virtual void drawVertexPrimitiveList(const void *vertices, u32 vertexCount, const void *indexList, u32 primitiveCount, E_VERTEX_TYPE vType = EVT_STANDARD, scene::E_PRIMITIVE_TYPE pType = scene::EPT_TRIANGLES, E_INDEX_TYPE iType = EIT_16BIT);

			virtual void drawStencilShadowVolume(const void *vertices, u32 vertexCount, const core::vector4df& lightPos, f32 shadowExtDist = 10000.0f, bool zFail = true);

			struct SHWBufferLink_RSX : public SHWBufferLink
			{
				SHWBufferLink_RSX(const scene::IMeshBuffer *mb)
				: SHWBufferLink(mb), vertices(NULL), vertexSize(0), vertexOffset(0),
				  indices(NULL), indexCount(0), indexOffset(0) {}

				void *vertices;
				u32 vertexSize;
				u32 vertexOffset;

				u16* indices;
				u32 indexCount;
				u32 indexOffset;
			};

			virtual SHWBufferLink* createHardwareBuffer(const scene::IMeshBuffer *mb);

			virtual bool updateHardwareBuffer(SHWBufferLink *hwBuffer);

			virtual void deleteHardwareBuffer(SHWBufferLink *hwBuffer);

			virtual void drawHardwareBuffer(SHWBufferLink *hwBuffer);

			virtual s32 getVertexShaderConstantID(const char *name);
			virtual s32 getPixelShaderConstantID(const char *name);

			virtual bool setVertexShaderConstant(s32 index, const float *floats, int count);
			virtual bool setPixelShaderConstant(s32 index, const float *floats, int count);

			virtual s32 addHighLevelShaderMaterial(const void *vertexShaderProgram, const void *fragmentShaderProgram, IShaderConstantSetCallback *callback = NULL, E_MATERIAL_TYPE baseMaterial = video::EMT_SOLID, s32 userData = 0);

			virtual IVideoDriver* getVideoDriver();

			virtual bool setRenderTarget(E_RENDER_TARGET target, bool clearTarget, bool clearZBuffer, SColor color);

			virtual void setBasicRenderStates(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates);

			virtual void setTextureRenderStates(const SMaterial& material, CRSXShaderPipeline *pipeline);

			virtual void addOcclusionQuery(scene::ISceneNode *node, scene::IMesh *mesh = NULL);
			virtual void runOcclusionQuery(scene::ISceneNode *node, bool visible = false);

			virtual mars_context* getMARSContext();

			virtual E_DRIVER_TYPE getDriverType() const { return EDT_RSX; }

			bool setActiveTexture(u32 stage, const video::ITexture *texture);
			bool disableTextures(u32 fromStage = 0);

			const SMaterial& getCurrentMaterial() const;

			const SColorf& getAmbientLight() const { return _ambientLight; }

			const core::vector3df& getCamWorldPos() const { return _camWorldPos; }

			gcmContextData* getGcmRootContext() const { return _gcmRootContext; }
			gcmContextData* getGcmContext() const { return _gcmContext; }
			void setGcmContext(gcmContextData *context) { _gcmContext = context; }

			void finish();

			CRSXState* getRSXState() const { return _rsxState; }

			CRSXVertexShader* getRSXVertexShader(const char *name);
			CRSXFragmentShader* getRSXFragmentShader(const char *name);

			CRSXMaterial* popRSXMaterial(const char *name);
			void pushRSXMaterial(CRSXMaterial *material);

			void waitRSXFinish();

		protected:
			virtual video::ITexture* createDeviceDependentTexture(IImage *surface, const io::path& name);

		private:
			void setRenderStates3DMode();
			bool genericDriverInit(const SIrrlichtCreationParameters& param);
			void waitRSXIdle();
			void setFramebufferTarget(u32 index);
			void setupViewport(const core::rect<s32>& vp);
			void initFlipEvent();
			void initDefaultStateCommands();
			void initIOBuffers(u32 hostBufferSize);
			void bindVertexStream(u8 index, u8 stride, u8 elems, u8 dtype, u32 offset, u8 location, bool *attr_set);
			void unbindVertexStream(s32 index);

			static void flipHandler(const u32 head);
			static void vblankHandler(const u32 head);

			bool updateVertexHardwareBuffer(SHWBufferLink_RSX *hwBuffer);
			bool updateIndexHardwareBuffer(SHWBufferLink_RSX *hwBuffer);

			void createMaterialRenderers();
			void renderArray(const void *indexList, u32 primitiveCount, scene::E_PRIMITIVE_TYPE pType, E_INDEX_TYPE iType);

			void clearBuffers(bool backBuffer, bool zBuffer, bool stencilBuffer, SColor color);

			void setVertexPrimitiveList(const void *vertices, u32 vertexCount, const void *indexList, u32 primitiveCount, E_VERTEX_TYPE vType = EVT_STANDARD, scene::E_PRIMITIVE_TYPE pType = scene::EPT_TRIANGLES, E_INDEX_TYPE iType = EIT_16BIT);

			void syncPPUGPU();

			u32 getTextureWrapMode(const u8 clamp);
			u32 getIndexCount(scene::E_PRIMITIVE_TYPE pType, u32 primitiveCount);

			void initDrawBufferSurface();
			void initVideoConfiguration();

			void initShaders();
			void initFpShader(const void *program, const core::stringc& name);
			void initVpShader(const void *program, const core::stringc& name);

			void initMaterials();

			struct RequestedLight
			{
				RequestedLight(const SLight& light)
				: lightData(light), hwLightIndex(-1)
				{}

				SLight lightData;
				s32 hwLightIndex;
			};

			core::matrix4 _matrices[ETS_COUNT];
			core::array< RequestedLight > _requestedLights;

			void *_hostAddress;
			gcmContextData *_gcmContext;
			gcmContextData *_gcmRootContext;
			u32 _rsxRefValue;

			void *_stateBufferAddress;
			u32 _stateBufferOffset;

			ECOLOR_FORMAT _colorFormat;

			videoResolution _videoResolution;
			core::dimension2d<u32> _currentRenderTargetSize;

			u32 _colorPitch;
			u32 _colorOffset[FRAME_BUFFER_COUNT];
			u32 _depthPitch;
			u32 _depthOffset;

			u32 _currentFB;
			u32 _fbOnDisplay;
			u32 _fbFlipped;
			bool _fbOnFlip;

			bool _resetRenderStates;

			CIrrDevicePS3 *_device;

			CRSXState *_rsxState;

			SColorf _ambientLight;
			SMaterial _material, _lastMaterial;

			u32 _maxTextureSize;

			SIrrlichtCreationParameters _params;

			core::vector3df _camWorldPos;

			SLight _activeLight;

			E_RENDER_CONTEXT _renderContext;

			class STextureStageCache
			{
				const ITexture *currentTexture[MATERIAL_MAX_TEXTURES];

			public:
				STextureStageCache()
				{
					for(u32 i=0;i < MATERIAL_MAX_TEXTURES;i++)
						currentTexture[i] = NULL;
				}

				~STextureStageCache()
				{
					clear();
				}

				void set(u32 stage, const ITexture *texture)
				{
					if(stage < MATERIAL_MAX_TEXTURES) {
						const ITexture *oldTexture = currentTexture[stage];

						if(texture != NULL) texture->grab();

						currentTexture[stage] = texture;

						if(oldTexture != NULL) oldTexture->drop();
					}
				}

				const ITexture* operator [](int stage) const
				{
					if((u32)stage < MATERIAL_MAX_TEXTURES)
						return currentTexture[stage];
					else
						return NULL;
				}

				void remove(ITexture *texture)
				{
					for(s32 i=MATERIAL_MAX_TEXTURES - 1;i >= 0;--i) {
						if(currentTexture[i] == texture) {
							texture->drop();
							currentTexture[i] = NULL;
						}
					}
				}

				void clear()
				{
					for(u32 i=0;i < MATERIAL_MAX_TEXTURES;i++) {
						if(currentTexture[i] != NULL) {
							currentTexture[i]->drop();
							currentTexture[i] = NULL;
						}
					}
				}
			};

			f32 _aspectRatio;

			STextureStageCache _currentTexture;

			sys_event_queue_t _flipEventQueue;
			sys_event_port_t _flipEventPort;

			gcmSurface _drawSurface;

			GCMDebugFontRenderer *_debugFontRenderer;

			S3DVertexStandard *_quad2DVertices;

			CRSXShaderPipeline *_pipeline;

			core::map<core::stringc, CRSXVertexShader*> _vpShaders;
			core::map<core::stringc, CRSXFragmentShader*> _fpShaders;
			core::map<core::stringc, CRSXMaterial*> _materials;

			static u32 sLabelVal;

			static CRSXDriver *instance;

			static const u16 quad2DIndices[4];
		};
	}
}


#endif /* RSXDRIVER_H_ */
