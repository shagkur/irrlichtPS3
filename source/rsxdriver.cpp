/*
 * rsxdriver.cpp
 *
 *  Created on: Feb 4, 2013
 *      Author: mike
 */

#include "irrtypes.h"
#include "irrdeviceps3.h"
#include "vector4d.h"
#include "rsxstate.h"
#include "rsxdriver.h"
#include "rsxtexture.h"
#include "rsxmaterial.h"
#include "rsxshaderpipeline.h"
#include "rsxmainheapmanager.h"
#include "rsxmaterialrenderer.h"
#include "rsxcgmaterialrenderer.h"

#include "vpshader_tex2d_vpo.h"
#include "vpshader_lighting_vpo.h"
#include "vpshader_ambient_emissive_vpo.h"
#include "vpshader_shadow_volume_vpo.h"
#include "vpshader_parallax_vpo.h"
#include "vpshader_parallax_lighting_vpo.h"

#include "fpshader_ambient_emissive_fpo.h"
#include "fpshader_lighting_fpo.h"
#include "fpshader_tex2d_fpo.h"
#include "fpshader_shadow_volume_fpo.h"
#include "fpshader_parallax_fpo.h"
#include "fpshader_parallax_lighting_fpo.h"

#define DEFUALT_CB_SIZE						0x80000		// 512Kb default command buffer size
#define HOST_STATE_CB_SIZE					0x10000		// 64Kb state command buffer size (used for resetting certain default states)
#define HOST_ADDR_ALIGNMENT					(1024*1024)

#define GCM_PREPARED_BUFFER_INDEX			65
#define GCM_BUFFER_STATUS_INDEX				66
#define GCM_WAIT_LABEL_INDEX				255

#define MAX_BUFFER_QUEUE_SIZE				1

#define OFFSETOF(type, pointer, memb)		((size_t)&(reinterpret_cast<type*>(pointer)->memb) - (size_t)pointer)

namespace irr
{
	namespace video
	{
		u32 CRSXDriver::sLabelVal = 1;
		CRSXDriver* CRSXDriver::instance = NULL;
		const u16 CRSXDriver::quad2DIndices[4] = { 0, 1, 2, 3 };

		static u32 resolutionIds[] = {
			VIDEO_RESOLUTION_960x1080,
			VIDEO_RESOLUTION_720,
			VIDEO_RESOLUTION_480,
			VIDEO_RESOLUTION_576
		};

		static u32 irrRSXPrimitiveMapping[] = {
			GCM_TYPE_POINTS,
			GCM_TYPE_LINE_STRIP,
			GCM_TYPE_LINE_LOOP,
			GCM_TYPE_LINES,
			GCM_TYPE_TRIANGLE_STRIP,
			GCM_TYPE_TRIANGLE_FAN,
			GCM_TYPE_TRIANGLES,
			GCM_TYPE_QUAD_STRIP,
			GCM_TYPE_QUADS,
			GCM_TYPE_POLYGON,
			0
		};

		CRSXDriver::CRSXDriver(const SIrrlichtCreationParameters& param, io::IFileSystem *fs, IrrlichtDevice *device)
		: CNullDriver(fs), _rsxRefValue(0), _colorFormat(ECF_R8G8B8), _currentRenderTargetSize(0, 0), _currentFB(0),
		  _fbOnDisplay(0), _fbFlipped(0), _fbOnFlip(false), _resetRenderStates(true),
		  _device(dynamic_cast<CIrrDevicePS3*>(device)), _maxTextureSize(4096), _params(param), _camWorldPos(0.0f, 0.0f, 0.0f),
		  _activeLight(SLight()), _renderContext(ERC_COUNT)
		{
			CRSXDriver::instance = this;

			genericDriverInit(param);
			createMaterialRenderers();
		}

		CRSXDriver::~CRSXDriver()
		{

		}

		bool CRSXDriver::genericDriverInit(const SIrrlichtCreationParameters& param)
		{
			s32 ret;
			u32 zs_depth = 4;
			u32 color_depth = 4;
			u32 bufferSize = rsxAlign(0x100000, (DEFUALT_CB_SIZE + HOST_STATE_CB_SIZE + param.hostBufferSize));
			u32 hostBufferSize = bufferSize - (DEFUALT_CB_SIZE + HOST_STATE_CB_SIZE);

			gcmInitDefaultFifoMode(GCM_DEFAULT_FIFO_MODE_CONDITIONAL);

			_hostAddress = memalign(HOST_ADDR_ALIGNMENT, bufferSize);
			ret = rsxInit(&_gcmRootContext, DEFUALT_CB_SIZE, bufferSize, _hostAddress);

			initIOBuffers(hostBufferSize);
			initDefaultStateCommands();
			setGcmContext(_gcmRootContext);

			_rsxState = new CRSXState(_gcmRootContext);
			_pipeline = new CRSXShaderPipeline(this);
			_debugFontRenderer = new GCMDebugFontRenderer(_gcmRootContext);

			_quad2DVertices = (S3DVertexStandard*)IHeapManager::allocate(64, sizeof(S3DVertexStandard)*4);
			_quad2DVertices[0] = S3DVertexStandard(core::vector3df(-1.0f,  1.0f, 0.0f), core::vector3df(0.0f, 0.0f, 0.0f), SColor(255, 255, 255, 255), core::vector2df(0.0f, 1.0f));
			_quad2DVertices[1] = S3DVertexStandard(core::vector3df( 1.0f,  1.0f, 0.0f), core::vector3df(0.0f, 0.0f, 0.0f), SColor(255, 255, 255, 255), core::vector2df(1.0f, 1.0f));
			_quad2DVertices[2] = S3DVertexStandard(core::vector3df( 1.0f, -1.0f, 0.0f), core::vector3df(0.0f, 0.0f, 0.0f), SColor(255, 255, 255, 255), core::vector2df(1.0f, 0.0f));
			_quad2DVertices[3] = S3DVertexStandard(core::vector3df(-1.0f, -1.0f, 0.0f), core::vector3df(0.0f, 0.0f, 0.0f), SColor(255, 255, 255, 255), core::vector2df(0.0f, 0.0f));

			initVideoConfiguration();

			_colorPitch = gcmGetTiledPitchSize(_videoResolution.width*color_depth);
			_depthPitch = gcmGetTiledPitchSize(_videoResolution.width*zs_depth);

			u32 bufferHeight = rsxAlign(GCM_ZCULL_ALIGN_HEIGHT, _videoResolution.height);
			u32 frameSize = bufferHeight*_colorPitch;
			u32 depthSize = bufferHeight*_depthPitch;

			_screenSize = core::dimension2d<s32>(_videoResolution.width, _videoResolution.height);
			_viewPort = core::rect<s32>(core::position2d<s32>(0, 0), _screenSize);

			waitRSXIdle();

			gcmSetFlipMode(GCM_FLIP_HSYNC);

			void *buffer;
			u32 regionSize, offset = 0;
			u32 gcmBufferOffset, tileIndex = 0, tagMemOffset = 0;
			for(u32 i=0;i < FRAME_BUFFER_COUNT;i++) {
				_colorOffset[i] = offset;
				offset += frameSize;
				offset = rsxAlign(8*_colorPitch, offset);
			}
			regionSize = offset + frameSize;
			regionSize = rsxAlign(32*_colorPitch, regionSize);
			regionSize = rsxAlign(GCM_TILE_ALIGN_OFFSET, regionSize);
			buffer = rsxMemalign(GCM_TILE_ALIGN_OFFSET, regionSize);

			rsxAddressToOffset(buffer, &gcmBufferOffset);
			gcmSetTile(tileIndex++, GCM_LOCATION_RSX, gcmBufferOffset, regionSize, _colorPitch, GCM_COMPMODE_DISABLED, tagMemOffset, 0);
			tagMemOffset += rsxAlign(GCM_TILE_ALIGN_OFFSET, regionSize);

			for(u32 i=0;i < FRAME_BUFFER_COUNT;i++) {
				_colorOffset[i] += gcmBufferOffset;
				gcmSetDisplayBuffer(i, _colorOffset[i], _colorPitch, _videoResolution.width, _videoResolution.height);
				printf("fb[%d] address %p (%08x) %d/%d %d\n", i, (u8*)buffer + (_colorOffset[i] - gcmBufferOffset), _colorOffset[i], _videoResolution.width, _videoResolution.height, _colorPitch);
			}

			regionSize = depthSize;
			regionSize = rsxAlign(32*_depthPitch, regionSize);
			regionSize = rsxAlign(GCM_TILE_ALIGN_OFFSET, regionSize);
			buffer = rsxMemalign(GCM_TILE_ALIGN_OFFSET, regionSize);

			rsxAddressToOffset(buffer, &_depthOffset);
			gcmSetTile(tileIndex++, GCM_LOCATION_RSX, _depthOffset, regionSize, _depthPitch, GCM_COMPMODE_Z32_SEPSTENCIL_REGULAR, tagMemOffset, 1);
			tagMemOffset += rsxAlign(GCM_TILE_ALIGN_OFFSET, regionSize);

			for(u32 i=0;i < FRAME_BUFFER_COUNT;i++) {
				*((vu32*) gcmGetLabelAddress(GCM_BUFFER_STATUS_INDEX + i)) = BUFFER_IDLE;
			}

			*((vu32*) gcmGetLabelAddress(GCM_PREPARED_BUFFER_INDEX)) = (_fbOnDisplay<<8);
			*((vu32*) gcmGetLabelAddress(GCM_BUFFER_STATUS_INDEX + _fbOnDisplay)) = BUFFER_BUSY;

			_currentFB = (_fbOnDisplay + 1)%FRAME_BUFFER_COUNT;

			initFlipEvent();

			initDrawBufferSurface();

			rsxSetWriteCommandLabel(_gcmRootContext, GCM_BUFFER_STATUS_INDEX + _currentFB, BUFFER_BUSY);

			DebugFont::init();
			DebugFont::setScreenRes(_videoResolution.width, _videoResolution.height);

			initShaders();
			initMaterials();

			return true;
		}

		void CRSXDriver::initIOBuffers(u32 hostBufferSize)
		{
			void *bufferAddress = NULL;
			void *heapBuffer = (void*)((u8*)_hostAddress + (DEFUALT_CB_SIZE + HOST_STATE_CB_SIZE));

			CRSXMainHeapManager::initialize(heapBuffer, hostBufferSize);

			_stateBufferAddress =  (void*)((u8*)_hostAddress + DEFUALT_CB_SIZE);
			rsxAddressToOffset(_stateBufferAddress, &_stateBufferOffset);
		}

		void CRSXDriver::initShaders()
		{
			initVpShader(vpshader_tex2d_vpo, "vpshader_tex2d");
			initVpShader(vpshader_lighting_vpo, "vpshader_lighting");
			initVpShader(vpshader_ambient_emissive_vpo, "vpshader_ambient_emissive");
			initVpShader(vpshader_parallax_vpo, "vpshader_parallax");
			initVpShader(vpshader_parallax_lighting_vpo, "vpshader_parallax_lighting");
			initVpShader(vpshader_shadow_volume_vpo, "vpshader_shadow_volume");

			initFpShader(fpshader_ambient_emissive_fpo, "fpshader_ambient_emissive");
			initFpShader(fpshader_lighting_fpo, "fpshader_lighting");
			initFpShader(fpshader_tex2d_fpo, "fpshader_tex2d");
			initFpShader(fpshader_parallax_fpo, "fpshader_parallax");
			initFpShader(fpshader_parallax_lighting_fpo, "fpshader_parallax_lighting");
			initFpShader(fpshader_shadow_volume_fpo, "fpshader_shadow_volume");
		}

		CRSXVertexShader* CRSXDriver::getRSXVertexShader(const char *name)
		{
			core::map<core::stringc, CRSXVertexShader*>::Node *n = _vpShaders.find(name);

			if(n != NULL)
				return n->getValue();

			return NULL;
		}

		CRSXFragmentShader* CRSXDriver::getRSXFragmentShader(const char *name)
		{
			core::map<core::stringc, CRSXFragmentShader*>::Node *n = _fpShaders.find(name);

			if(n != NULL)
				return n->getValue();

			return NULL;
		}

		void CRSXDriver::initVpShader(const void *program, const core::stringc& name)
		{
			CRSXVertexShader *tmpShader = new CRSXVertexShader(this);

			tmpShader->init(program, name);

			_vpShaders.insert(name, tmpShader);
		}

		void CRSXDriver::initFpShader(const void *program, const core::stringc& name)
		{
			CRSXFragmentShader *tmpShader = new CRSXFragmentShader(this);

			tmpShader->init(program, name);

			_fpShaders.insert(name, tmpShader);
		}

		void CRSXDriver::initMaterials()
		{
			CRSXMaterial *tmpMat;
			CRSXVertexShader *vShader;

			tmpMat = new CRSXMaterial(this);
			tmpMat->setVertexShader(_vpShaders["vpshader_ambient_emissive"]);
			tmpMat->setFragmentShader(_fpShaders["fpshader_ambient_emissive"]);
			_materials.insert("material_ambient_emissive", tmpMat);

			tmpMat = new CRSXMaterial(this);
			tmpMat->setVertexShader(_vpShaders["vpshader_lighting"]);
			tmpMat->setFragmentShader(_fpShaders["fpshader_lighting"]);
			_materials.insert("material_lighting", tmpMat);

			tmpMat = new CRSXMaterial(this);
			tmpMat->setVertexShader(_vpShaders["vpshader_tex2d"]);
			tmpMat->setFragmentShader(_fpShaders["fpshader_tex2d"]);
			_materials.insert("material_tex2d", tmpMat);

			tmpMat = new CRSXMaterial(this);
			tmpMat->setVertexShader(_vpShaders["vpshader_parallax_lighting"]);
			tmpMat->setFragmentShader(_fpShaders["fpshader_parallax_lighting"]);
			_materials.insert("material_parallax_lighting", tmpMat);

			tmpMat = new CRSXMaterial(this);
			tmpMat->setVertexShader(_vpShaders["vpshader_parallax"]);
			tmpMat->setFragmentShader(_fpShaders["fpshader_parallax"]);
			_materials.insert("material_parallax", tmpMat);

			tmpMat = new CRSXMaterial(this);
			tmpMat->setVertexShader(_vpShaders["vpshader_shadow_volume"]);
			tmpMat->setFragmentShader(_fpShaders["fpshader_shadow_volume"]);
			tmpMat->setDepthMask(GCM_FALSE);
			tmpMat->setDepthTestEnable(GCM_TRUE);
			tmpMat->setDepthFunc(GCM_LESS);
			tmpMat->setColorMask(0);
			tmpMat->setZMinMaxControl(GCM_FALSE, GCM_TRUE, GCM_FALSE);
			tmpMat->setStencilMask(0xff);
			tmpMat->setBackStencilMask(0xff);
			tmpMat->setStencilFunc(GCM_ALWAYS, 0, 0xffffffff);
			tmpMat->setBackStencilFunc(GCM_ALWAYS, 0, 0xffffffff);
			tmpMat->setPolygonOffsetFillEnable(GCM_TRUE);
			tmpMat->setPolygonOffset(0.0f, 1.0f);
			tmpMat->setCullFaceEnable(GCM_FALSE);
			_materials.insert("material_shadow_volume", tmpMat);
		}

		CRSXMaterial* CRSXDriver::popRSXMaterial(const char *name)
		{
			core::map<core::stringc, CRSXMaterial*>::Node *n = _materials.find(name);

			if(n != NULL) {
				CRSXMaterial *m = n->getValue();

				m->pushState();
				return m;
			}

			return NULL;
		}

		void CRSXDriver::pushRSXMaterial(CRSXMaterial *material)
		{
			if(material != NULL)
				material->popState();
		}

		void CRSXDriver::initVideoConfiguration()
		{
			s32 rval = 0;
			s32 resId = 0;

			for(u32 i=0;i < sizeof(resolutionIds)/sizeof(u32);i++) {
				rval = videoGetResolutionAvailability(VIDEO_PRIMARY, resolutionIds[i], VIDEO_ASPECT_AUTO, 0);
				if(rval != 1) continue;

				resId = resolutionIds[i];
				rval = videoGetResolution(resId, &_videoResolution);
				if(!rval) break;
			}

			if(rval) {
				printf("Error: videoGetResolutionAvailability failed. No usable resolution.\n");
				return;
			}

			videoConfiguration config = {
				(u8)resId,
				VIDEO_BUFFER_FORMAT_XRGB,
				VIDEO_ASPECT_AUTO,
				{0,0,0,0,0,0,0,0,0},
				gcmGetTiledPitchSize(_videoResolution.width*4)
			};

			rval = videoConfigure(VIDEO_PRIMARY, &config, NULL, 0);
			if(rval) {
				printf("Error: videoConfigure failed.\n");
				return;
			}

			videoState state;

			rval = videoGetState(VIDEO_PRIMARY, 0, &state);
			switch(state.displayMode.aspect) {
				case VIDEO_ASPECT_4_3:
					_aspectRatio = 4.0f/3.0f;
					break;
				case VIDEO_ASPECT_16_9:
					_aspectRatio = 16.0f/9.0f;
					break;
				default:
					printf("unknown aspect ratio %x\n", state.displayMode.aspect);
					_aspectRatio = 16.0f/9.0f;
					break;
			}
		}

		void CRSXDriver::initDefaultStateCommands()
		{
			rsxSetCurrentBuffer(&_gcmRootContext, (u32*)_stateBufferAddress, HOST_STATE_CB_SIZE);
			{
				rsxSetBlendEnable(_gcmRootContext, GCM_FALSE);
				rsxSetBlendFunc(_gcmRootContext, GCM_ONE, GCM_ZERO, GCM_ONE, GCM_ZERO);
				rsxSetBlendEquation(_gcmRootContext, GCM_FUNC_ADD, GCM_FUNC_ADD);
				rsxSetDepthWriteEnable(_gcmRootContext, GCM_TRUE);
				rsxSetDepthFunc(_gcmRootContext, GCM_LESS);
				rsxSetDepthTestEnable(_gcmRootContext, GCM_FALSE);
				rsxSetClearDepthStencil(_gcmRootContext,0xffffff00);
				rsxSetColorMaskMRT(_gcmRootContext, 0);
				rsxSetFrontFace(_gcmRootContext, GCM_FRONTFACE_CW);
				rsxSetClearReport(_gcmRootContext, GCM_ZPASS_PIXEL_CNT);
				rsxSetZControl(_gcmRootContext, GCM_TRUE, GCM_FALSE, GCM_FALSE);
				rsxSetZCullControl(_gcmRootContext, GCM_ZCULL_LESS, GCM_ZCULL_LONES);
				rsxSetSCullControl(_gcmRootContext, GCM_SCULL_SFUNC_LESS, 1, 0xff);
				rsxSetReturnCommand(_gcmRootContext);
			}
			rsxSetDefaultCommandBuffer(&_gcmRootContext);
		}

		mars_context* CRSXDriver::getMARSContext()
		{
			return _device->getMARSContext();
		}

		void CRSXDriver::finish()
		{
			rsxFinish(_gcmRootContext, _rsxRefValue++);

			u32 data = *(vu32*)gcmGetLabelAddress(GCM_PREPARED_BUFFER_INDEX);
			u32 lastBufferToFlip = data>>8;

			while(_fbOnDisplay != lastBufferToFlip) {
				usleep(1000);
			}

			gcmSetFlipHandler(NULL);
			gcmSetVBlankHandler(NULL);
		}

		void CRSXDriver::clearBuffers(bool backBuffer, bool zBuffer, bool stencilBuffer, SColor color)
		{
			u32 clear_mask = 0;

			if(backBuffer)
				clear_mask |= (GCM_CLEAR_A | GCM_CLEAR_R | GCM_CLEAR_G | GCM_CLEAR_B);

			if(zBuffer)
				clear_mask |= GCM_CLEAR_Z;

			if(stencilBuffer)
				clear_mask |= GCM_CLEAR_S;

			_rsxState->setClearColor(color.toA8R8G8B8());
			_rsxState->setClearSurface(clear_mask);
		}

		bool CRSXDriver::beginScene(bool backBuffer,bool zBuffer, SColor color)
		{
			CNullDriver::beginScene(backBuffer, zBuffer, color);

			rsxSetCallCommand(_gcmRootContext, _stateBufferOffset);

			setupViewport(_viewPort);

			for(u32 i=0;i < 8;i++)
				rsxSetViewportClip(_gcmRootContext, i, _videoResolution.width, _videoResolution.height);

			clearBuffers(backBuffer, zBuffer, false, color);

			return true;
		}

		bool CRSXDriver::endScene()
		{
			CNullDriver::endScene();

			s32 qid = gcmSetPrepareFlip(_gcmRootContext, _currentFB);
			while(qid < 0) {
				usleep(100);
				qid = gcmSetPrepareFlip(_gcmRootContext, _currentFB);
			}

			rsxSetWriteBackendLabel(_gcmRootContext, GCM_PREPARED_BUFFER_INDEX, ((_currentFB<<8)|qid));
			rsxFlushBuffer(_gcmRootContext);

			syncPPUGPU();

			_resetRenderStates = true;
			_currentFB = (_currentFB + 1)%FRAME_BUFFER_COUNT;

			rsxSetWaitLabel(_gcmRootContext, GCM_BUFFER_STATUS_INDEX + _currentFB, BUFFER_IDLE);
			rsxSetWriteCommandLabel(_gcmRootContext, GCM_BUFFER_STATUS_INDEX + _currentFB, BUFFER_BUSY);

			_rsxState->reset();

			setFramebufferTarget(_currentFB);

			return true;
		}

		ECOLOR_FORMAT CRSXDriver::getColorFormat() const
		{
			return _colorFormat;
		}

		const core::dimension2d<u32>& CRSXDriver::getCurrentRenderTargetSize() const
		{
			if(_currentRenderTargetSize.width == 0)
				return _screenSize;
			else
				return _currentRenderTargetSize;
		}

		const core::matrix4& CRSXDriver::getTransform(E_TRANSFORMATION_STATE state) const
		{
			return _matrices[state];
		}

		void CRSXDriver::setTransform(E_TRANSFORMATION_STATE state,const core::matrix4& mat)
		{
			_matrices[state] = mat;

			switch(state) {
			case ETS_WORLD:
				_matrices[ETS_MVP] = _matrices[ETS_PROJECTION]*_matrices[ETS_VIEW]*_matrices[ETS_WORLD];
				break;
			default:
				break;
			}
		}

		void CRSXDriver::setMaterial(const SMaterial& material)
		{
			_material = material;

			for(u32 i=0;i < MAX_TEXTURE_UNITS;i++)
				setActiveTexture(i, material.getTexture(i));
		}

		void CRSXDriver::setAmbientLight(const SColorf& ambientColor)
		{
			_ambientLight = ambientColor;
		}

		void CRSXDriver::setCamWorldPos(const core::vector3df& camWorldPos)
		{
			_camWorldPos = camWorldPos;
		}

		void CRSXDriver::setRenderContext(E_RENDER_CONTEXT context)
		{
			_renderContext = context;
		}

		E_RENDER_CONTEXT CRSXDriver::getRenderContext() const
		{
			return _renderContext;
		}

		void CRSXDriver::setActiveLight(const SLight& light)
		{
			_activeLight = light;
		}

		const SLight& CRSXDriver::getActiveLight() const
		{
			return _activeLight;
		}

		void CRSXDriver::deleteAllDynamicLights()
		{
			_requestedLights.clear();

			CNullDriver::deleteAllDynamicLights();
		}

		s32 CRSXDriver::addDynamicLight(const SLight& light)
		{
			CNullDriver::addDynamicLight(light);

			_requestedLights.push_back(RequestedLight(light));

			u32 newLightIndex = _requestedLights.size() - 1;

			return newLightIndex;
		}

		const SLight& CRSXDriver::getDynamicLight(u32 idx) const
		{
			if(idx < 0 || idx >= _requestedLights.size()) return *((SLight*)0);

			return _requestedLights[idx].lightData;
		}

		u32 CRSXDriver::getDynamicLightCount() const
		{
			return _requestedLights.size();
		}

		u32 CRSXDriver::getMaximalDynamicLightAmount() const
		{
			return MAX_LIGHTS;
		}

		ITexture* CRSXDriver::createDeviceDependentTexture(IImage *surface, const io::path& name)
		{
			return new CRSXTexture(surface, name, this);
		}

		void CRSXDriver::setViewPort(const core::rect<s32>& area)
		{
			if(area == _viewPort) return;

			core::rect<s32> vp = area;
			core::rect<s32> rendert(0, 0, getCurrentRenderTargetSize().width, getCurrentRenderTargetSize().height);

			vp.clipAgainst(rendert);
			if(vp.getHeight() > 0 && vp.getWidth() > 0) {
				setupViewport(vp);
				_viewPort = vp;
			}
		}

		u32 CRSXDriver::getMaximalPrimitiveCount() const
		{
			return 0x7fffffff;
		}

		void CRSXDriver::bindVertexStream(u8 index, u8 stride, u8 elems, u8 dtype, u32 offset, u8 location, bool *attr_set)
		{
			vertexDataArray_t *vtxArray = &_rsxState->state.vertexDataArray[index];

			if(elems == 0) return;

			if(vtxArray->stride != stride ||
			   vtxArray->size != elems ||
			   vtxArray->type != dtype ||
			   vtxArray->location != location ||
			   vtxArray->offset != offset)
			{
				rsxBindVertexArrayAttrib(_gcmContext, index, 0, offset, stride, elems, dtype, location);

				vtxArray->location = location;
				vtxArray->offset = offset;
				vtxArray->size = elems;
				vtxArray->stride = stride;
				vtxArray->type = dtype;
			}
			attr_set[index] = true;
		}

		void CRSXDriver::unbindVertexStream(s32 index)
		{
			vertexDataArray_t *vtxArray = &_rsxState->state.vertexDataArray[index];

			if(vtxArray->size == 0) return;

			rsxBindVertexArrayAttrib(_gcmContext, index, 0, 0, 0, 0, GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_RSX);

			vtxArray->frequency = 0;
			vtxArray->stride = 0;
			vtxArray->size = 0;
			vtxArray->type = GCM_VERTEX_DATA_TYPE_F32;
			vtxArray->location = GCM_LOCATION_RSX;
			vtxArray->offset = 0;
		}

		void CRSXDriver::setVertexPrimitiveList(const void *vertices, u32 vertexCount, const void *indexList, u32 primitiveCount, E_VERTEX_TYPE vType, scene::E_PRIMITIVE_TYPE pType, E_INDEX_TYPE iType)
		{
			if(!primitiveCount || !vertexCount)
				return;

			CNullDriver::drawVertexPrimitiveList(vertices, vertexCount, indexList, primitiveCount, vType, pType, iType);

			if(vertices != NULL) {
				u32 offset;

				rsxAddressToOffset(vertices, &offset);

				switch(vType) {
					case EVT_BASE:
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertexBase), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexBase, const_cast<void*>(vertices), pos), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertexBase), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexBase, const_cast<void*>(vertices), nrm), GCM_LOCATION_CELL, this));
						break;
					case EVT_STANDARD:
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertexStandard), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexStandard, const_cast<void*>(vertices), pos), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertexStandard), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexStandard, const_cast<void*>(vertices), nrm), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_COLOR0, sizeof(S3DVertexStandard), 4, GCM_VERTEX_DATA_TYPE_U8, offset + OFFSETOF(S3DVertexStandard, const_cast<void*>(vertices), col), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TEX0, sizeof(S3DVertexStandard), 2, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexStandard, const_cast<void*>(vertices), tcoords), GCM_LOCATION_CELL, this));
						break;
					case EVT_2TCOORDS:
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertex2TCoords), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(vertices), pos), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertex2TCoords), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(vertices), nrm), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_COLOR0, sizeof(S3DVertex2TCoords), 4, GCM_VERTEX_DATA_TYPE_U8, offset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(vertices), col), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TEX0, sizeof(S3DVertex2TCoords), 2, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(vertices), tcoords), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TEX1, sizeof(S3DVertex2TCoords), 2, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(vertices), tcoords2), GCM_LOCATION_CELL, this));
						break;
					case EVT_TANGENTS:
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertexTangents), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexTangents, const_cast<void*>(vertices), pos), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertexTangents), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexTangents, const_cast<void*>(vertices), nrm), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_COLOR0, sizeof(S3DVertexTangents), 4, GCM_VERTEX_DATA_TYPE_U8, offset + OFFSETOF(S3DVertexTangents, const_cast<void*>(vertices), col), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TEX0, sizeof(S3DVertexTangents), 2, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexTangents, const_cast<void*>(vertices), tcoords), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TANGENT, sizeof(S3DVertexTangents), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexTangents, const_cast<void*>(vertices), tangent), GCM_LOCATION_CELL, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_BINORMAL, sizeof(S3DVertexTangents), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexTangents, const_cast<void*>(vertices), binormal), GCM_LOCATION_CELL, this));
						break;
				}
			}

			if(indexList != NULL)
				_pipeline->setIndexStream(CRSXIndexStream(indexList, getIndexCount(pType, primitiveCount), irrRSXPrimitiveMapping[(s32)pType], iType == EIT_16BIT ? GCM_INDEX_TYPE_16B : GCM_INDEX_TYPE_32B, GCM_LOCATION_CELL, this));

			setRenderStates3DMode();
		}

		void CRSXDriver::drawVertexPrimitiveList(const void *vertices, u32 vertexCount, const void *indexList, u32 primitiveCount, E_VERTEX_TYPE vType, scene::E_PRIMITIVE_TYPE pType, E_INDEX_TYPE iType)
		{
			_pipeline->reset();
			setVertexPrimitiveList(vertices, vertexCount, indexList, primitiveCount, vType, pType, iType);
		}

		void CRSXDriver::drawStencilShadowVolume(const void *vertices, u32 vertexCount, const core::vector4df& lightPos, f32 shadowExtDist, bool zFail)
		{
			if(!vertexCount) return;

			u32 offset;
			const core::matrix4& mvp = getTransform(ETS_MVP);
			CRSXMaterial *material = popRSXMaterial("material_shadow_volume");

			rsxAddressToOffset(vertices, &offset);

			_pipeline->reset();

			_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertexBase), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexBase, const_cast<void*>(vertices), pos), GCM_LOCATION_CELL, this));
			_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertexBase), 3, GCM_VERTEX_DATA_TYPE_F32, offset + OFFSETOF(S3DVertexBase, const_cast<void*>(vertices), nrm), GCM_LOCATION_CELL, this));

			pushRSXMaterial(material);
/*
			u32 offset;
			S3DVertexBase *p = reinterpret_cast<S3DVertexBase*>(const_cast<void*>(vertices));

			if(static_cast<u32>(_material.materialType) < _materialRenderers.size()) {
				_materialRenderers[_material.materialType].renderer->onUnsetMaterial();
				_resetRenderStates = true;
			}


			rsxAddressToOffset(&p[0].pos, &offset);
			rsxBindVertexArrayAttrib(_gcmContext, GCM_VERTEX_ATTRIB_POS, 0, offset, sizeof(S3DVertexBase), 3, GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_CELL);

			rsxAddressToOffset(&p[0].nrm, &offset);
			rsxBindVertexArrayAttrib(_gcmContext, GCM_VERTEX_ATTRIB_NORMAL, 0, offset, sizeof(S3DVertexBase), 3, GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_CELL);

			rsxSetDepthWriteEnable(_gcmContext, GCM_FALSE);
			rsxSetDepthTestEnable(_gcmContext, GCM_TRUE);
			rsxSetDepthFunc(_gcmContext, GCM_LESS);

			rsxSetColorMask(_gcmContext, 0);

			rsxSetZControl(_gcmContext, GCM_FALSE, GCM_TRUE, GCM_FALSE);

			rsxSetStencilTestEnable(_gcmContext, GCM_TRUE);
			rsxSetTwoSidedStencilTestEnable(_gcmContext, GCM_TRUE);
			rsxSetStencilMask(_gcmContext, 0xff);
			rsxSetBackStencilMask(_gcmContext, 0xff);
			rsxSetStencilFunc(_gcmContext, GCM_ALWAYS, 0, 0xffffffff);
			rsxSetBackStencilFunc(_gcmContext, GCM_ALWAYS, 0, 0xffffffff);

			rsxSetPolygonOffsetFillEnable(_gcmContext, GCM_TRUE);
			rsxSetPolygonOffset(_gcmContext, 0.0f, 1.0f);

			_shadowVolumeShader->apply(getTransform(ETS_PROJECTION), getTransform(ETS_VIEW), getTransform(ETS_WORLD), lightPos, shadowExtDist);
			_shadowVolumeShader->bind();

			rsxSetCullFaceEnable(_gcmContext, GCM_FALSE);
			if(zFail) {
				rsxSetStencilOp(_gcmContext, GCM_KEEP, GCM_INCR_WRAP, GCM_KEEP);
				rsxSetBackStencilOp(_gcmContext, GCM_KEEP, GCM_DECR_WRAP, GCM_KEEP);
				rsxDrawVertexArray(_gcmContext, GCM_TYPE_TRIANGLES, 0, vertexCount);
			} else {
				rsxSetStencilOp(_gcmContext, GCM_KEEP, GCM_KEEP, GCM_DECR_WRAP);
				rsxSetBackStencilOp(_gcmContext, GCM_KEEP, GCM_KEEP, GCM_INCR_WRAP);
				rsxDrawVertexArray(_gcmContext, GCM_TYPE_TRIANGLES, 0, vertexCount);
			}

			// now restore to previous state
			rsxSetDepthWriteEnable(_gcmContext, _rsxState->state.depth_mask);
			rsxSetDepthTestEnable(_gcmContext, _rsxState->state.depth_test);
			rsxSetDepthFunc(_gcmContext, _rsxState->state.depth_func);

			rsxSetColorMask(_gcmContext, _rsxState->state.color_mask);

			rsxSetZControl(_gcmContext, _rsxState->state.cull_near_far_enable, _rsxState->state.zclamp_enable, _rsxState->state.cull_ignoreW);

			rsxSetCullFaceEnable(_gcmContext, _rsxState->state.cull_face_enable);

			rsxSetPolygonOffsetFillEnable(_gcmContext, _rsxState->state.polygon_offset_fill_enable);
			rsxSetPolygonOffset(_gcmContext, _rsxState->state.polygon_offset_factor, _rsxState->state.polygon_offset_units);

			rsxSetStencilTestEnable(_gcmContext, _rsxState->state.stencil_test_enable);
			rsxSetTwoSidedStencilTestEnable(_gcmContext, _rsxState->state.stencil_test_enable_two_sided);
			rsxSetStencilMask(_gcmContext, _rsxState->state.stencil_mask_mask);
			rsxSetBackStencilMask(_gcmContext, _rsxState->state.stencil_back_mask_mask);
			rsxSetStencilFunc(_gcmContext, _rsxState->state.stencil_func_func, _rsxState->state.stencil_func_ref, _rsxState->state.stencil_func_mask);
			rsxSetBackStencilFunc(_gcmContext, _rsxState->state.stencil_back_func_func, _rsxState->state.stencil_back_func_ref, _rsxState->state.stencil_back_func_mask);
			rsxSetStencilOp(_gcmContext, _rsxState->state.stencil_op_fail, _rsxState->state.stencil_op_depth_fail, _rsxState->state.stencil_op_depth_pass);
			rsxSetBackStencilOp(_gcmContext, _rsxState->state.stencil_back_op_fail, _rsxState->state.stencil_back_op_depth_fail, _rsxState->state.stencil_back_op_depth_pass);
*/
		}

		void CRSXDriver::drawHardwareBuffer(SHWBufferLink *hwBuffer)
		{
			if(hwBuffer == NULL) return;

			updateHardwareBuffer(hwBuffer);
			hwBuffer->lastUsed = 0;

			SHWBufferLink_RSX *buffer = (SHWBufferLink_RSX*)hwBuffer;
			const scene::IMeshBuffer *mb = buffer->meshBuffer;
			const E_VERTEX_TYPE vType = mb->getVertexType();
			const void *vertices = mb->getVertices();
			const u16 *indices = mb->getIndices();

			_pipeline->reset();

			if(buffer->mapping_Vertex != scene::EHM_NEVER) {
				const u8 location = buffer->mapping_Vertex == scene::EHM_STATIC ? GCM_LOCATION_RSX : GCM_LOCATION_CELL;

				switch(vType) {
					case EVT_BASE:
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertexBase), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexBase, const_cast<void*>(buffer->vertices), pos), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertexBase), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexBase, const_cast<void*>(buffer->vertices), nrm), location, this));
						break;
					case EVT_STANDARD:
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertexStandard), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexStandard, const_cast<void*>(buffer->vertices), pos), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertexStandard), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexStandard, const_cast<void*>(buffer->vertices), nrm), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_COLOR0, sizeof(S3DVertexStandard), 4, GCM_VERTEX_DATA_TYPE_U8, buffer->vertexOffset + OFFSETOF(S3DVertexStandard, const_cast<void*>(buffer->vertices), col), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TEX0, sizeof(S3DVertexStandard), 2, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexStandard, const_cast<void*>(buffer->vertices), tcoords), location, this));
						break;
					case EVT_2TCOORDS:
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertex2TCoords), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(buffer->vertices), pos), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertex2TCoords), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(buffer->vertices), nrm), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_COLOR0, sizeof(S3DVertex2TCoords), 4, GCM_VERTEX_DATA_TYPE_U8, buffer->vertexOffset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(buffer->vertices), col), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TEX0, sizeof(S3DVertex2TCoords), 2, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(buffer->vertices), tcoords), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TEX1, sizeof(S3DVertex2TCoords), 2, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertex2TCoords, const_cast<void*>(buffer->vertices), tcoords2), location, this));
						break;
					case EVT_TANGENTS:
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_POS, sizeof(S3DVertexTangents), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexTangents, const_cast<void*>(buffer->vertices), pos), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_NORMAL, sizeof(S3DVertexTangents), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexTangents, const_cast<void*>(buffer->vertices), nrm), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_COLOR0, sizeof(S3DVertexTangents), 4, GCM_VERTEX_DATA_TYPE_U8, buffer->vertexOffset + OFFSETOF(S3DVertexTangents, const_cast<void*>(buffer->vertices), col), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TEX0, sizeof(S3DVertexTangents), 2, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexTangents, const_cast<void*>(buffer->vertices), tcoords), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_TANGENT, sizeof(S3DVertexTangents), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexTangents, const_cast<void*>(buffer->vertices), tangent), location, this));
						_pipeline->addVertexStream(CRSXVertexStream(GCM_VERTEX_ATTRIB_BINORMAL, sizeof(S3DVertexTangents), 3, GCM_VERTEX_DATA_TYPE_F32, buffer->vertexOffset + OFFSETOF(S3DVertexTangents, const_cast<void*>(buffer->vertices), binormal), location, this));
						break;
				}
				vertices = NULL;		// discard since streams are now set
			}

			if(buffer->mapping_Index != scene::EHM_NEVER) {
				const u8 location = buffer->mapping_Index == scene::EHM_STATIC ? GCM_LOCATION_RSX : GCM_LOCATION_CELL;

				indices = NULL;
				_pipeline->setIndexStream(CRSXIndexStream(buffer->indexOffset, buffer->indexCount, GCM_TYPE_TRIANGLES, mb->getIndexType() == EIT_16BIT ? GCM_INDEX_TYPE_16B : GCM_INDEX_TYPE_32B, location, this));
			}

			setVertexPrimitiveList(vertices, mb->getVertexCount(), indices, mb->getIndexCount()/3, mb->getVertexType(), scene::EPT_TRIANGLES, mb->getIndexType());
		}

		CRSXDriver::SHWBufferLink* CRSXDriver::createHardwareBuffer(const scene::IMeshBuffer *mb)
		{
			if(mb == NULL || (mb->getHardwareMappingHint_Vertex() == scene::EHM_NEVER && mb->getHardwareMappingHint_Index() == scene::EHM_NEVER))
				return NULL;

			SHWBufferLink_RSX *hwBuffer = new SHWBufferLink_RSX(mb);

			_hwBufferMap.insert(hwBuffer->meshBuffer, hwBuffer);

			hwBuffer->changedID_Vertex = hwBuffer->meshBuffer->getChangedID_Vertex();
			hwBuffer->changedID_Index = hwBuffer->meshBuffer->getChangedID_Index();
			hwBuffer->mapping_Vertex = hwBuffer->meshBuffer->getHardwareMappingHint_Vertex();
			hwBuffer->mapping_Index = hwBuffer->meshBuffer->getHardwareMappingHint_Index();
			hwBuffer->lastUsed = 0;

			hwBuffer->vertices = NULL;
			hwBuffer->indices = NULL;
			hwBuffer->vertexSize = 0;
			hwBuffer->indexCount = 0;

			if(!updateHardwareBuffer(hwBuffer)) {
				deleteHardwareBuffer(hwBuffer);
				return NULL;
			}

			return hwBuffer;
		}

		void CRSXDriver::deleteHardwareBuffer(SHWBufferLink *hwBuffer)
		{
			if(hwBuffer == NULL) return;

			SHWBufferLink_RSX *buffer = (SHWBufferLink_RSX*)hwBuffer;

			if(buffer->vertices != NULL) rsxFree(buffer->vertices);
			buffer->vertices = NULL;

			if(buffer->indices != NULL) rsxFree(buffer->indices);
			buffer->indices = NULL;

			CNullDriver::deleteHardwareBuffer(hwBuffer);
		}

		bool CRSXDriver::updateHardwareBuffer(SHWBufferLink *hwBuffer)
		{
			if(hwBuffer == NULL) return false;

			if(hwBuffer->mapping_Vertex != scene::EHM_NEVER) {
				if(hwBuffer->changedID_Vertex != hwBuffer->meshBuffer->getChangedID_Vertex() ||
				   ((SHWBufferLink_RSX*)hwBuffer)->vertices == NULL)
				{
					hwBuffer->changedID_Vertex = hwBuffer->meshBuffer->getChangedID_Vertex();
					if(!updateVertexHardwareBuffer((SHWBufferLink_RSX*)hwBuffer)) return false;
				}
			}

			if(hwBuffer->mapping_Index != scene::EHM_NEVER) {
				if(hwBuffer->changedID_Index != hwBuffer->meshBuffer->getChangedID_Index() ||
				   ((SHWBufferLink_RSX*)hwBuffer)->indices == NULL)
				{
					hwBuffer->changedID_Index = hwBuffer->meshBuffer->getChangedID_Index();
					if(!updateIndexHardwareBuffer((SHWBufferLink_RSX*)hwBuffer)) return false;
				}
			}

			return true;
		}

		bool CRSXDriver::updateVertexHardwareBuffer(SHWBufferLink_RSX *hwBuffer)
		{
			if(hwBuffer == NULL) return false;

			const scene::IMeshBuffer *mb = hwBuffer->meshBuffer;
			const void *vertices = mb->getVertices();
			const u32 vertexCount = mb->getVertexCount();
			const E_VERTEX_TYPE vType = mb->getVertexType();
			const u32 vertexSize = getVertexPitchFromType(vType);
			const scene::E_HARDWARE_MAPPING mappingHint = hwBuffer->mapping_Vertex;

			if(hwBuffer->vertices == NULL || vertexSize*vertexCount != hwBuffer->vertexSize) {
				if(mappingHint == scene::EHM_STATIC) {
					if(hwBuffer->vertices != NULL) rsxFree(hwBuffer->vertices); //if we ever have to free a previous buffer when STATIC then something went wrong!
					hwBuffer->vertices = rsxMemalign(128, vertexSize*vertexCount);
				} else if(mappingHint == scene::EHM_DYNAMIC) {
					if(hwBuffer->vertices != NULL) IHeapManager::deallocate(hwBuffer->vertices);
					hwBuffer->vertices = IHeapManager::allocate(128, vertexSize*vertexCount);
				}
				if(hwBuffer->vertices == NULL) return false;

				rsxAddressToOffset(hwBuffer->vertices, &hwBuffer->vertexOffset);
				hwBuffer->vertexSize = vertexSize*vertexCount;
			}

			printf("vbo size: %dKB\n", (hwBuffer->vertexSize/1024));
			memcpy(hwBuffer->vertices, vertices, hwBuffer->vertexSize);

			return true;
		}

		bool CRSXDriver::updateIndexHardwareBuffer(SHWBufferLink_RSX *hwBuffer)
		{
			if(hwBuffer == NULL) return false;

			const scene::IMeshBuffer *mb = hwBuffer->meshBuffer;
			const u16 *indices = mb->getIndices();
			const u32 indexCount = mb->getIndexCount();
			const scene::E_HARDWARE_MAPPING mappingHint = hwBuffer->mapping_Index;

			if(hwBuffer->indices == NULL || indexCount != hwBuffer->indexCount) {
				if(mappingHint == scene::EHM_STATIC) {
					if(hwBuffer->indices != NULL) rsxFree(hwBuffer->indices);
					hwBuffer->indices = (u16*)rsxMemalign(128, indexCount*sizeof(u16));
				} else if(mappingHint == scene::EHM_DYNAMIC) {
					if(hwBuffer->indices != NULL) IHeapManager::deallocate(hwBuffer->indices);
					hwBuffer->indices = (u16*)IHeapManager::allocate(128, indexCount*sizeof(u16));
				}
				if(hwBuffer->indices == NULL) return false;

				rsxAddressToOffset(hwBuffer->indices, &hwBuffer->indexOffset);
				hwBuffer->indexCount = indexCount;
			}

			memcpy(hwBuffer->indices, indices, hwBuffer->indexCount*sizeof(u16));
			return true;
		}

		bool CRSXDriver::setActiveTexture(u32 stage, const video::ITexture *texture)
		{
			if(stage >= MATERIAL_MAX_TEXTURES) return false;

			if(_currentTexture[stage] == texture) return true;

			_currentTexture.set(stage, texture);

			if(texture == NULL)
				return true;
			else {

			}
			return true;
		}

		bool CRSXDriver::disableTextures(u32 fromStage)
		{
			bool result = true;

			for(u32 i=fromStage;i < MAX_TEXTURE_UNITS;i++)
				result &= setActiveTexture(i, NULL);

			return result;
		}

		s32 CRSXDriver::getVertexShaderConstantID(const char *name)
		{
			return -1;
		}

		s32 CRSXDriver::getPixelShaderConstantID(const char *name)
		{
			return -1;
		}

		bool CRSXDriver::setVertexShaderConstant(s32 index, const float *floats, int count)
		{
			return false;
		}

		bool CRSXDriver::setPixelShaderConstant(s32 index, const float *floats, int count)
		{
			return false;
		}

		const SMaterial& CRSXDriver::getCurrentMaterial() const
		{
			return _material;
		}

		void CRSXDriver::addOcclusionQuery(scene::ISceneNode *node, scene::IMesh *mesh)
		{
			CNullDriver::addOcclusionQuery(node, mesh);

			const s32 index = _occlusionQueries.linear_search(SOccQuery(node));
			if(index != -1 && _occlusionQueries[index].UID == 0) {
				_occlusionQueries[index].UID = 100;	//needs proper allocation scheme to handle the indices limit and usage
			}
		}

		void CRSXDriver::runOcclusionQuery(scene::ISceneNode *node, bool visible)
		{
			if(node == NULL) return;

			const s32 index = _occlusionQueries.linear_search(SOccQuery(node));
			if(index != -1) {
				if(_occlusionQueries[index].UID != 0) {
					rsxSetZPixelCountEnable(_gcmRootContext, GCM_TRUE);
				}

				CNullDriver::runOcclusionQuery(node, visible);

				if(_occlusionQueries[index].UID != 0) {
					rsxSetZPixelCountEnable(_gcmRootContext, GCM_FALSE);
					rsxSetReport(_gcmRootContext, GCM_ZPASS_PIXEL_CNT, _occlusionQueries[index].UID);
				}
			}
		}

		void CRSXDriver::setBasicRenderStates(const SMaterial& material, const SMaterial& lastMaterial, bool resetAllRenderStates)
		{
			if(resetAllRenderStates || _lastMaterial.gouraudShading != material.gouraudShading) {
				u32 shade_model;

				if(material.gouraudShading)
					shade_model = GCM_SHADE_MODEL_SMOOTH;
				else
					shade_model = GCM_SHADE_MODEL_FLAT;

				_rsxState->setShadeModel(shade_model);
			}

			if(resetAllRenderStates || lastMaterial.wireFrame != material.wireFrame || lastMaterial.pointCloud != material.pointCloud) {
				u32 polygon_mode = material.wireFrame ? GCM_POLYGON_MODE_LINE : (material.pointCloud ? GCM_POLYGON_MODE_POINT : GCM_POLYGON_MODE_FILL);

				_rsxState->setBackPolygonMode(polygon_mode);
				_rsxState->setFrontPolygonMode(polygon_mode);
			}

			if(resetAllRenderStates || lastMaterial.zBuffer != material.zBuffer) {
				u32 depth_test = GCM_FALSE;
				u32 depth_func = GCM_LESS;
				switch(material.zBuffer) {
					case ECFN_LESSEQUAL:
						depth_test = GCM_TRUE;
						depth_func = GCM_LEQUAL;
						break;
					case ECFN_EQUAL:
						depth_test = GCM_TRUE;
						depth_func = GCM_EQUAL;
						break;
					case ECFN_LESS:
						depth_test = GCM_TRUE;
						depth_func = GCM_LESS;
						break;
					case ECFN_NOTEQUAL:
						depth_test = GCM_TRUE;
						depth_func = GCM_NOTEQUAL;
						break;
					case ECFN_GREATEREQUAL:
						depth_test = GCM_TRUE;
						depth_func = GCM_GEQUAL;
						break;
					case ECFN_GREATER:
						depth_test = GCM_TRUE;
						depth_func = GCM_GREATER;
						break;
					case ECFN_ALWAYS:
						depth_test = GCM_TRUE;
						depth_func = GCM_ALWAYS;
						break;
					case ECFN_NEVER:
						depth_test = GCM_TRUE;
						depth_func = GCM_NEVER;
						break;
					default:
						break;
				}

				_rsxState->setDepthTestEnable(depth_test);
				_rsxState->setDepthFunc(depth_func);
			}

			{
				u32 depth_mask;

				if(material.zWriteEnable && !material.isTransparent())
					depth_mask = GCM_TRUE;
				else
					depth_mask = GCM_FALSE;

				_rsxState->setDepthWriteEnable(depth_mask);
			}

			if(resetAllRenderStates || lastMaterial.frontfaceCulling != material.frontfaceCulling || lastMaterial.backfaceCulling != material.backfaceCulling) {
				u32 cull_face_mode = GCM_CULL_BACK;
				u32 cull_face_enable = (material.frontfaceCulling || material.backfaceCulling);

				if(material.frontfaceCulling && material.backfaceCulling)
					cull_face_mode = GCM_CULL_ALL;
				else if(material.backfaceCulling)
					cull_face_mode = GCM_CULL_BACK;
				else if(material.frontfaceCulling)
					cull_face_mode = GCM_CULL_FRONT;

				_rsxState->setCullFace(cull_face_mode);
				_rsxState->setCullFaceEnable(cull_face_enable);
			}

			if(resetAllRenderStates || lastMaterial.colorMask != material.colorMask) {
				u32 color_mask = (((material.colorMask&ECP_ALPHA) ? GCM_COLOR_MASK_A : 0) |
						   	     ((material.colorMask&ECP_RED) ? GCM_COLOR_MASK_R : 0) |
						   	     ((material.colorMask&ECP_GREEN) ? GCM_COLOR_MASK_G : 0) |
						   	     ((material.colorMask&ECP_BLUE) ? GCM_COLOR_MASK_B : 0));


				_rsxState->setColorMask(color_mask);
			}

			if(resetAllRenderStates || lastMaterial.blendOperation != material.blendOperation) {
				u32 blend_enable;
				u32 blend_eq_color = GCM_FUNC_ADD;
				u32 blend_eq_alpha = GCM_FUNC_ADD;

				if(material.blendOperation == EBO_NONE)
					blend_enable = GCM_FALSE;
				else {
					blend_enable = GCM_TRUE;

					switch(material.blendOperation) {
						case EBO_ADD:
							blend_eq_color = GCM_FUNC_ADD;
							blend_eq_alpha = GCM_FUNC_ADD;
							break;
						case EBO_SUBTRACT:
							blend_eq_color = GCM_FUNC_SUBTRACT;
							blend_eq_alpha = GCM_FUNC_SUBTRACT;
							break;
						case EBO_REVSUBTRACT:
							blend_eq_color = GCM_FUNC_SUBTRACT;
							blend_eq_alpha = GCM_FUNC_SUBTRACT;
							break;
						case EBO_MIN:
							blend_eq_color = GCM_FUNC_MIN;
							blend_eq_alpha = GCM_FUNC_MIN;
							break;
						case EBO_MAX:
							blend_eq_color = GCM_FUNC_MIN;
							blend_eq_alpha = GCM_FUNC_MIN;
							break;
						default:
							break;
					}
				}

				_rsxState->setBlendEnable(blend_enable);
				_rsxState->setBlendEquation(blend_eq_color, blend_eq_alpha);
			}
		}

		void CRSXDriver::setTextureRenderStates(const SMaterial& material, CRSXShaderPipeline *pipeline)
		{
			for(s32 i=MAX_TEXTURE_UNITS - 1;i >= 0;--i) {
				u8 maxAniso = GCM_TEXTURE_MAX_ANISO_1;
				CRSXTexture *texture = (CRSXTexture*)_currentTexture[i];

				if(texture != NULL) {
					s16 lodBias = 0;
					u8 minFilter = (material.textureLayer[i].trilinearFilter ? GCM_TEXTURE_LINEAR_MIPMAP_LINEAR :
								   (material.textureLayer[i].bilinearFilter ? GCM_TEXTURE_LINEAR_MIPMAP_NEAREST :
										   GCM_TEXTURE_NEAREST_MIPMAP_NEAREST));
					u8 magFilter = (material.textureLayer[i].trilinearFilter || material.textureLayer[i].bilinearFilter ? GCM_TEXTURE_LINEAR : GCM_TEXTURE_NEAREST);

					if(material.textureLayer[i].LODBias) {
						const f32 tmp = core::clamp(material.textureLayer[i].LODBias*0.125f, -16.0f, 15.996f);
						lodBias = (tmp*256.0f);
					}

					maxAniso = (material.textureLayer[i].anisotropicFilter > 0 ? core::min_(MAX_ANISO, material.textureLayer[i].anisotropicFilter) : 0);

					texture->setTextureControl(GCM_TRUE, 0<<8, 12<<8, maxAniso);
					texture->setClamp(getTextureWrapMode(material.textureLayer[i].textureWrapU), getTextureWrapMode(material.textureLayer[i].textureWrapV), GCM_TEXTURE_MIRRORED_REPEAT);
					texture->setTextureFilter(0, minFilter, magFilter, GCM_TEXTURE_CONVOLUTION_QUINCUNX);

					pipeline->setTextureUnit(i, texture);
				} else
					pipeline->setTextureUnit(i, NULL);
			}
		}

		u32 CRSXDriver::getTextureWrapMode(const u8 clamp)
		{
			u32 mode = GCM_TEXTURE_REPEAT;

			switch(clamp) {
				case ETC_REPEAT:
					mode = GCM_TEXTURE_REPEAT;
					break;
				case ETC_CLAMP:
					mode = GCM_TEXTURE_CLAMP;
					break;
				case ETC_CLAMP_TO_EDGE:
					mode = GCM_TEXTURE_CLAMP_TO_EDGE;
					break;
				case ETC_CLAMP_TO_BORDER:
					mode = GCM_TEXTURE_BORDER;
					break;
				case ETC_MIRROR:
					mode = GCM_TEXTURE_MIRRORED_REPEAT;
					break;
				case ETC_MIRROR_CLAMP:
					mode = GCM_TEXTURE_MIRROR_ONCE_CLAMP;
					break;
				case ETC_MIRROR_CLAMP_TO_EDGE:
					mode = GCM_TEXTURE_MIRROR_ONCE_CLAMP_TO_EDGE;
					break;
				case ETC_MIRROR_CLAMP_TO_BORDER:
					mode = GCM_TEXTURE_MIRROR_ONCE_CLAMP_TO_BORDER;
					break;
			}

			return mode;
		}

		IVideoDriver* CRSXDriver::getVideoDriver()
		{
			return this;
		}

		void CRSXDriver::setRenderStates3DMode()
		{
			if(_lastMaterial.materialType != _material.materialType &&
			   static_cast<u32>(_lastMaterial.materialType) < _materialRenderers.size())
			{
				_materialRenderers[_lastMaterial.materialType].renderer->onUnsetMaterial();
			}

			if(static_cast<u32>(_material.materialType) < _materialRenderers.size())
				_materialRenderers[_material.materialType].renderer->onSetMaterial(_material, _lastMaterial, _resetRenderStates, _pipeline);

			_lastMaterial = _material;
			_resetRenderStates = false;

			if(static_cast<u32>(_material.materialType) < _materialRenderers.size())
				_materialRenderers[_material.materialType].renderer->onRender(_pipeline, video::EVT_STANDARD);
		}

		void CRSXDriver::renderArray(const void *indexList, u32 primitiveCount, scene::E_PRIMITIVE_TYPE pType, E_INDEX_TYPE iType)
		{
			if(indexList != NULL) {
				u32 offset;
				s32 ret = rsxAddressToOffset(indexList, &offset);

				if(ret) {
					if(iType == EIT_16BIT)
						rsxDrawInlineIndexArray16(_gcmContext, irrRSXPrimitiveMapping[(s32)pType], 0, getIndexCount(pType, primitiveCount), static_cast<const u16*>(indexList));
					else
						rsxDrawInlineIndexArray32(_gcmContext, irrRSXPrimitiveMapping[(s32)pType], 0, getIndexCount(pType, primitiveCount), static_cast<const u32*>(indexList));
				} else
					rsxDrawIndexArray(_gcmContext, irrRSXPrimitiveMapping[(s32)pType], offset, getIndexCount(pType, primitiveCount), iType == EIT_16BIT ? GCM_INDEX_TYPE_16B : GCM_INDEX_TYPE_32B, GCM_LOCATION_CELL);
			}
		}

		u32 CRSXDriver::getIndexCount(scene::E_PRIMITIVE_TYPE pType, u32 primitiveCount)
		{
			switch(pType) {
				case scene::EPT_POINTS:
				case scene::EPT_POINT_SPRITES:
				case scene::EPT_LINE_LOOP:
				case scene::EPT_POLYGON:
					return primitiveCount;
				case scene::EPT_LINE_STRIP:
					return primitiveCount + 1;
				case scene::EPT_LINES:
					return primitiveCount*2;
				case scene::EPT_TRIANGLE_STRIP:
				case scene::EPT_TRIANGLE_FAN:
					return primitiveCount + 2;
				case scene::EPT_TRIANGLES:
					return primitiveCount*3;
				case scene::EPT_QUAD_STRIP:
					return primitiveCount*2 + 2;
				case scene::EPT_QUADS:
					return primitiveCount*4;
			}
			return 0;
		}

		void CRSXDriver::createMaterialRenderers()
		{
			CRSXMaterialRenderer *lmr = new CRSXMaterialRenderer_SOLID(this);

			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);

			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);

			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addAndDropMaterialRenderer(new CRSXMaterialRenderer_TRANSPARENT_ADD_COLOR(this));
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);

			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);

			//addMaterialRenderer(lmr);
			addAndDropMaterialRenderer(new CRSXParallaxMaterialRenderer(this));
			addMaterialRenderer(lmr);
			addMaterialRenderer(lmr);

			lmr->drop();
		}

		s32 CRSXDriver::addHighLevelShaderMaterial(const void *vertexShaderProgram, const void *fragmentShaderProgram, IShaderConstantSetCallback *callback, E_MATERIAL_TYPE baseMaterial, s32 userData)
		{
			s32 nr = -1;

			CRSXCgMaterialRenderer *r = new CRSXCgMaterialRenderer(this, nr, vertexShaderProgram, fragmentShaderProgram, callback, baseMaterial, userData);
			r->drop();

			return nr;
		}

		void CRSXDriver::waitRSXIdle()
		{
			rsxSetWriteBackendLabel(_gcmRootContext,GCM_WAIT_LABEL_INDEX,sLabelVal);
			rsxSetWaitLabel(_gcmRootContext,GCM_WAIT_LABEL_INDEX,sLabelVal);

			++sLabelVal;

			waitRSXFinish();
		}

		void CRSXDriver::waitRSXFinish()
		{
			rsxSetWriteBackendLabel(_gcmRootContext,GCM_WAIT_LABEL_INDEX,sLabelVal);

			rsxFlushBuffer(_gcmRootContext);

			while(*(vu32*)gcmGetLabelAddress(GCM_WAIT_LABEL_INDEX)!=sLabelVal)
				usleep(30);

			++sLabelVal;
		}

		bool CRSXDriver::setRenderTarget(E_RENDER_TARGET target, bool clearTarget, bool clearZBuffer, SColor color)
		{
			return false;
		}

		void CRSXDriver::initDrawBufferSurface()
		{
			memset(&_drawSurface, 0, sizeof(gcmSurface));

			_drawSurface.colorFormat		= GCM_SURFACE_A8R8G8B8;
			_drawSurface.colorTarget		= GCM_SURFACE_TARGET_0;
			_drawSurface.colorLocation[0]	= GCM_LOCATION_RSX;
			_drawSurface.colorOffset[0]		= _colorOffset[_currentFB];
			_drawSurface.colorPitch[0]		= _colorPitch;

			for(u32 i=1; i< GCM_MAX_MRT_COUNT;i++) {
				_drawSurface.colorLocation[i]	= GCM_LOCATION_RSX;
				_drawSurface.colorOffset[i]		= _colorOffset[_currentFB];
				_drawSurface.colorPitch[i]		= 64;
			}

			_drawSurface.depthFormat		= GCM_SURFACE_ZETA_Z24S8;
			_drawSurface.depthLocation		= GCM_LOCATION_RSX;
			_drawSurface.depthOffset		= _depthOffset;
			_drawSurface.depthPitch			= _depthPitch;

			_drawSurface.type				= GCM_SURFACE_TYPE_LINEAR;
			_drawSurface.antiAlias			= GCM_SURFACE_CENTER_1;
			_drawSurface.width				= _videoResolution.width;
			_drawSurface.height				= _videoResolution.height;
			_drawSurface.x					= 0;
			_drawSurface.y					= 0;

			gcmSetZcull(0, _depthOffset, rsxAlign(64, _videoResolution.width), rsxAlign(64, _videoResolution.height), 0, GCM_ZCULL_Z24S8, GCM_SURFACE_CENTER_1, GCM_ZCULL_LESS, GCM_ZCULL_LONES, GCM_SCULL_SFUNC_LESS, 1, 0xff);
		}

		void CRSXDriver::setFramebufferTarget(u32 index)
		{
			_drawSurface.colorOffset[0] = _colorOffset[index];
			rsxSetSurface(_gcmRootContext,&_drawSurface);
		}

		void CRSXDriver::initFlipEvent()
		{
			sys_event_queue_attr_t queueAttr = { SYS_EVENT_QUEUE_PRIO, SYS_EVENT_QUEUE_PPU, "\0" };

			sysEventQueueCreate(&_flipEventQueue, &queueAttr, SYS_EVENT_QUEUE_KEY_LOCAL, 32);
			sysEventPortCreate(&_flipEventPort, SYS_EVENT_PORT_LOCAL, SYS_EVENT_PORT_NO_NAME);
			sysEventPortConnectLocal(_flipEventPort, _flipEventQueue);

			gcmSetFlipHandler(CRSXDriver::flipHandler);
			gcmSetVBlankHandler(CRSXDriver::vblankHandler);
		}

		void CRSXDriver::vblankHandler(const u32 head)
		{
			CRSXDriver *driver = CRSXDriver::instance;

			if(driver != NULL) {
				u32 data = *(vu32*)gcmGetLabelAddress(GCM_PREPARED_BUFFER_INDEX);
				u32 bufferToFlip = data>>8;
				u32 indexToFlip = data&0x07;

				if(!driver->_fbOnFlip) {
					if (bufferToFlip != driver->_fbOnDisplay) {
						s32 ret = gcmSetFlipImmediate(indexToFlip);
						if(ret != 0) return;

						driver->_fbFlipped = bufferToFlip;
						driver->_fbOnFlip = true;
					}
				}

			}
		}

		void CRSXDriver::flipHandler(const u32 head)
		{
			CRSXDriver *driver = CRSXDriver::instance;

			if(driver != NULL) {
				u32 v = driver->_fbFlipped;

				for(u32 i=driver->_fbOnDisplay;i != v;i=(i + 1)%FRAME_BUFFER_COUNT) {
					*(vu32*)gcmGetLabelAddress(GCM_BUFFER_STATUS_INDEX + i) = BUFFER_IDLE;
				}

				driver->_fbOnDisplay = v;
				driver->_fbOnFlip = false;

				sysEventPortSend(driver->_flipEventPort, 0, 0, 0);
			}
		}

		void CRSXDriver::syncPPUGPU()
		{
			vu32 *label = gcmGetLabelAddress(GCM_PREPARED_BUFFER_INDEX);

			while(((_currentFB + FRAME_BUFFER_COUNT - ((*label)>>8))%FRAME_BUFFER_COUNT) > MAX_BUFFER_QUEUE_SIZE) {
				sys_event_t event;

				sysEventQueueReceive(_flipEventQueue, &event, 0);
				sysEventQueueDrain(_flipEventQueue);
			}
		}

		void CRSXDriver::setupViewport(const core::rect<s32>& vp)
		{
			f32 min, max;
			u16 x, y, w, h;
			f32 scale[4], offset[4];

			x = vp.upperLeftCorner.X;
			y = vp.upperLeftCorner.Y;
			w = vp.getWidth();
			h = vp.getHeight();
			min = 0.0f;
			max = 1.0f;
			scale[0] = w*0.5f;
			scale[1] = h*-0.5f;
			scale[2] = (max - min)*0.5f;
			scale[3] = 0.0f;
			offset[0] = x + w*0.5f;
			offset[1] = y + h*0.5f;
			offset[2] = (max + min)*0.5f;
			offset[3] = 0.0f;

			rsxSetViewport(_gcmRootContext,x, y, w, h, min, max, scale, offset);
			rsxSetScissor(_gcmRootContext,x,y,w,h);
		}

		IVideoDriver* createRSXDriver(const SIrrlichtCreationParameters& param, io::IFileSystem *fs, IrrlichtDevice *device)
		{
			return new CRSXDriver(param, fs, device);
		}
	}
}
