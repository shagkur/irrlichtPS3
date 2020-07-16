#include "nulldriver.h"
#include "os.h"
#include "image.h"
#include "ireadfile.h"
#include "iimageloader.h"
#include "ianimatedmeshscenenode.h"
#include "meshmanipulator.h"

namespace irr
{
	namespace video
	{
		IImageLoader* createImageLoaderPNG();
		IImageLoader* createImageLoaderJPG();
		IImageLoader* createImageLoaderBMP();
		IImageLoader* createImageLoaderTGA();

		CNullDriver::CNullDriver(io::IFileSystem *fs)
		: _fileSystem(fs), _meshManipulator(NULL),  _viewPort(0, 0, 0, 0), _minVertexCountForVBO(500),
		  _primitivesDrawn(0), _textureCreationFlags(0), _stencilShadowEnabled(false)
		{
			setTextureCreationFlag(ETCF_ALWAYS_32_BIT, true);
			setTextureCreationFlag(ETCF_CREATE_MIP_MAPS, true);

			_meshManipulator = new scene::CMeshManipulator();
			if(_fileSystem) _fileSystem->grab();

			_surfaceLoader.push_back(createImageLoaderPNG());
			_surfaceLoader.push_back(createImageLoaderJPG());
			_surfaceLoader.push_back(createImageLoaderBMP());
			_surfaceLoader.push_back(createImageLoaderTGA());

			generateSobelKernel(SOBEL_KERNEL3x3);
		}

		CNullDriver::~CNullDriver()
		{
			if(_fileSystem) _fileSystem->drop();
			if(_meshManipulator) _meshManipulator->drop();

			if(_sobelKernel[0] != NULL) delete [] _sobelKernel[0];
			if(_sobelKernel[1] != NULL) delete [] _sobelKernel[1];
		}

		scene::IMeshManipulator* CNullDriver::getMeshManipulator()
		{
			return _meshManipulator;
		}

		bool CNullDriver::beginScene(bool backBuffer,bool zBuffer, SColor color)
		{
			_primitivesDrawn = 0;
			return true;
		}

		bool CNullDriver::endScene()
		{
			_fpsCounter.registerFrame(os::Timer::getRealTime(), _primitivesDrawn);
			return true;
		}

		ECOLOR_FORMAT CNullDriver::getColorFormat() const
		{
			return ECF_R5G6B5;
		}

		const core::dimension2d<u32>& CNullDriver::getScreenSize() const
		{
			return _screenSize;
		}

		const core::dimension2d<u32>& CNullDriver::getCurrentRenderTargetSize() const
		{
			return _mrtSize;
		}

		void CNullDriver::setTransform(E_TRANSFORMATION_STATE state,const core::matrix4& mat)
		{
		}

		const core::matrix4& CNullDriver::getTransform(E_TRANSFORMATION_STATE state) const
		{
			return _transformationMatrix;
		}

		void CNullDriver::setMaterial(const SMaterial& material)
		{

		}

		void CNullDriver::addOcclusionQuery(scene::ISceneNode *node, const scene::IMesh *mesh)
		{
			if(node == NULL) return;

			if(mesh == NULL) {
				if(node->getType() != scene::ESNT_MESH && node->getType() != scene::ESNT_ANIMATED_MESH)
					return;
				else if(node->getType() == scene::ESNT_MESH)
					mesh = static_cast<scene::IMeshSceneNode*>(node)->getMesh();
				else
					mesh = static_cast<scene::IAnimatedMeshSceneNode*>(node)->getMesh()->getMesh(0);

				if(mesh == NULL) return;
			}

			s32 index = _occlusionQueries.linear_search(SOccQuery(node));
			if(index != -1) {
				if(_occlusionQueries[index].mesh != mesh) {
					_occlusionQueries[index].mesh->drop();
					_occlusionQueries[index].mesh = mesh;
					mesh->grab();
				}
			} else {
				_occlusionQueries.push_back(SOccQuery(node, mesh));
				node->setAutomaticCulling(node->getAutomaticCulling() | scene::EAC_OCC_QUERY);
			}
		}

		void CNullDriver::removeOcclusionQuery(scene::ISceneNode *node)
		{
			s32 index = _occlusionQueries.linear_search(SOccQuery(node));
			if(index != -1) {
				node->setAutomaticCulling(node->getAutomaticCulling()&~scene::EAC_OCC_QUERY);
				_occlusionQueries.erase(index);
			}
		}

		void CNullDriver::runOcclusionQuery(scene::ISceneNode *node, bool visible)
		{
			if(node == NULL) return;

			s32 index = _occlusionQueries.linear_search(SOccQuery(node));
			if(index == -1) return;

			if(!visible) {
				SMaterial mat;

				mat.lighting = false;
				mat.colorMask = ECP_NONE;
				mat.gouraudShading = false;
				mat.zWriteEnable = false;
				setMaterial(mat);
			}

			setTransform(video::ETS_WORLD, node->getAbsoluteTransformation());

			const scene::IMesh *mesh = _occlusionQueries[index].mesh;
			for(u32 i=0;i < mesh->getMeshBufferCount();i++) {
				if(visible)
					setMaterial(mesh->getMeshBuffer(i)->getMaterial());

				drawMeshBuffer(mesh->getMeshBuffer(i));
			}
		}

		u32 CNullDriver::getTextureCount() const
		{
			return _textures.size();
		}

		ITexture* CNullDriver::getTexture(const io::path& filename)
		{
			const io::path absolutePath = _fileSystem->getAbsolutePath(filename);

			ITexture *texture = findTexture(absolutePath);
			if(texture != NULL) return texture;

			texture = findTexture(filename);
			if(texture != NULL) return texture;

			io::IReadFile *file = _fileSystem->createAndOpenFile(absolutePath);
			if(file == NULL) file = _fileSystem->createAndOpenFile(filename);

			if(file != NULL) {
				texture = findTexture(file->getFilename());
				if(texture != NULL) {
					file->drop();
					return texture;
				}

				texture = loadTextureFromFile(file);
				file->drop();

				if(texture != NULL) {
					addTexture(texture);
					texture->drop();
				}

				return texture;
			}

			return NULL;
		}

		ITexture* CNullDriver::getTexture(io::IReadFile *file)
		{
			ITexture *texture = NULL;

			if(file != NULL) {
				texture = findTexture(file->getFilename());
				if(texture != NULL) return texture;

				texture = loadTextureFromFile(file);
				if(texture != NULL) {
					addTexture(texture);
					texture->drop();
				}
			}

			return texture;
		}

		ITexture* CNullDriver::findTexture(const io::path& filename)
		{
			SSurface s;
			SDummyTexture dummy(filename);

			s.surface = &dummy;

			s32 index = _textures.binary_search(s);
			if(index != -1) return _textures[index].surface;

			return NULL;
		}

		ITexture* CNullDriver::addTexture(const io::path& name, IImage *image)
		{
			if(name.size() == 0 || image == NULL)
				return NULL;

			ITexture *t = createDeviceDependentTexture(image, name);
			if(t != NULL) {
				addTexture(t);
				t->drop();
			}

			return t;
		}

		ITexture* CNullDriver::addTexture(const core::dimension2d<u32>& size, const io::path& name, ECOLOR_FORMAT format)
		{
			if(name.size() == 0) return NULL;

			IImage *image = new CImage(format, size);
			ITexture *t = createDeviceDependentTexture(image, name);

			image->drop();
			addTexture(t);

			if(t != NULL) t->drop();

			return t;
		}

		ITexture* CNullDriver::createDeviceDependentTexture(IImage *surface, const io::path& name)
		{
			return new SDummyTexture(name);
		}

		void CNullDriver::addTexture(ITexture *texture)
		{
			if(texture != NULL) {
				SSurface s;

				s.surface = texture;
				texture->grab();

				_textures.push_back(s);
				_textures.sort();
			}
		}

		ITexture* CNullDriver::loadTextureFromFile(io::IReadFile *file, const io::path& hashName)
		{
			ITexture *texture = NULL;
			IImage *image = createImageFromFile(file);

			if(image != NULL) {
				texture = createDeviceDependentTexture(image, hashName.size() > 0 ? hashName : file->getFilename());
				image->drop();
			}

			return texture;
		}

		void CNullDriver::deleteAllDynamicLights()
		{
			_lights.set_used(0);
		}

		s32 CNullDriver::addDynamicLight(const SLight& light)
		{
			_lights.push_back(light);
			return _lights.size() - 1;
		}

		void CNullDriver::turnLightOn(s32 lightIndex, bool turnOn)
		{

		}

		u32 CNullDriver::getMaximalDynamicLightAmount() const
		{
			return 0;
		}

		u32 CNullDriver::getDynamicLightCount() const
		{
			return _lights.size();
		}

		const SLight& CNullDriver::getDynamicLight(u32 idx) const
		{
			if(idx < _lights.size())
				return _lights[idx];
			else
				return *((SLight*)0);
		}

		f32 CNullDriver::getFPS() const
		{
			return _fpsCounter.getFPS();
		}

		u32 CNullDriver::getPrimitiveCountDrawn(u32 mode) const
		{
			return (mode == 0) ? _fpsCounter.getPrimitive() : (mode == 1) ? _fpsCounter.getPrimitiveAverage() : _fpsCounter.getPrimitiveTotal();
		}

		u32 CNullDriver::getMaximalPrimitiveCount() const
		{
			return 0xffffffff;
		}

		s32 CNullDriver::addAndDropMaterialRenderer(IMaterialRenderer *m)
		{
			s32 i = addMaterialRenderer(m);
			if(m != NULL) m->drop();
			return i;
		}

		s32 CNullDriver::addMaterialRenderer(IMaterialRenderer *renderer, const char *cname)
		{
			SMaterialRenderer r;

			if(renderer == NULL) return -1;

			r.renderer = renderer;
			r.name = cname;

			_materialRenderers.push_back(r);
			renderer->grab();

			return _materialRenderers.size() - 1;
		}

		IMaterialRenderer* CNullDriver::getMaterialRenderer(u32 nr)
		{
			if(nr < _materialRenderers.size())
				return _materialRenderers[nr].renderer;
			else
				return NULL;
		}

		void CNullDriver::setViewPort(const core::rect<s32>& area)
		{
		}

		const core::rect<s32>& CNullDriver::getViewPort() const
		{
			return _viewPort;
		}

		void CNullDriver::drawMeshBuffer(const scene::IMeshBuffer *mb)
		{
			if(mb == NULL) return;

			SHWBufferLink *hwBuffer = getBufferLink(mb);
			if(hwBuffer != NULL)
				drawHardwareBuffer(hwBuffer);
			else
				drawVertexPrimitiveList(mb->getVertices(), mb->getVertexCount(), mb->getIndices(), mb->getIndexCount()/3, mb->getVertexType(), scene::EPT_TRIANGLES);
		}

		CNullDriver::SHWBufferLink* CNullDriver::getBufferLink(const scene::IMeshBuffer *mb)
		{
			if(mb == NULL || !isHardwareBufferRecommended(mb))
				return NULL;

			core::map< const scene::IMeshBuffer*, SHWBufferLink* >::Node *n = _hwBufferMap.find(mb);
			if(n != NULL) return n->getValue();

			return createHardwareBuffer(mb);
		}

		void CNullDriver::deleteHardwareBuffer(SHWBufferLink *hwBuffer)
		{
			if(hwBuffer == NULL) return;

			_hwBufferMap.remove(hwBuffer->meshBuffer);
			delete hwBuffer;
		}

		bool CNullDriver::isHardwareBufferRecommended(const scene::IMeshBuffer *mb)
		{
			if(mb == NULL || (mb->getHardwareMappingHint_Vertex() == scene::EHM_NEVER && mb->getHardwareMappingHint_Index() == scene::EHM_NEVER))
				return false;

			if(mb->getVertexCount() < _minVertexCountForVBO)
				return false;

			return true;
		}

		void CNullDriver::drawVertexPrimitiveList(const void *vertices, u32 vertexCount, const void *indexList, u32 primitiveCount, E_VERTEX_TYPE vType, scene::E_PRIMITIVE_TYPE pType, E_INDEX_TYPE iType)
		{
			_primitivesDrawn += primitiveCount;
		}

		void CNullDriver::drawStencilShadowVolume(const void *vertices, u32 vertexCount, const core::vector4df& lightPos, f32 shadowExtDist, bool zFail)
		{

		}

		void CNullDriver::setTextureCreationFlag(E_TEXTURE_CREATION_FLAG flag, bool enabled)
		{
			if(enabled && (flag == ETCF_ALWAYS_16_BIT || flag == ETCF_ALWAYS_32_BIT ||
						   flag == ETCF_OPTIMIZED_FOR_QUALITY || flag == ETCF_OPTIMIZED_FOR_SPEED))
			{
				setTextureCreationFlag(ETCF_ALWAYS_16_BIT, false);
				setTextureCreationFlag(ETCF_ALWAYS_32_BIT, false);
				setTextureCreationFlag(ETCF_OPTIMIZED_FOR_QUALITY, false);
				setTextureCreationFlag(ETCF_OPTIMIZED_FOR_SPEED, false);
			}
			_textureCreationFlags = (_textureCreationFlags&~flag) | ((((u32)!enabled) - 1)&flag);
		}

		bool CNullDriver::getTextureCreationFlag(E_TEXTURE_CREATION_FLAG flag) const
		{
			return (_textureCreationFlags&flag) != 0;
		}

		IImage* CNullDriver::createImageFromFile(const io::path& filename)
		{
			if(filename.size() == 0) return NULL;

			IImage *image = NULL;
			io::IReadFile *file = _fileSystem->createAndOpenFile(filename);
			if(file != NULL) {
				image = createImageFromFile(file);
				file->drop();
			}

			return image;
		}

		IImage* CNullDriver::createImageFromFile(io::IReadFile *file)
		{
			if(file == NULL) return NULL;

			s32 i;
			IImage *image = NULL;

			printf("filename: %s\n", file->getFilename().c_str());
			for(i=_surfaceLoader.size() - 1;i >= 0;--i) {
				if(_surfaceLoader[i]->isLoadableFileExtension(file->getFilename())) {
					file->seek(0);
					image = _surfaceLoader[i]->loadImage(file);
					if(image != NULL) return image;
				}
			}

			for(i=_surfaceLoader.size() - 1;i >= 0;--i) {
				if(_surfaceLoader[i]->isLoadableFileFormat(file)) {
					file->seek(0);
					image = _surfaceLoader[i]->loadImage(file);
					if(image != NULL) return image;
				}
			}

			return NULL;
		}

		IImage* CNullDriver::createImageFromData(ECOLOR_FORMAT format, const core::dimension2d<u32>& size, void *data, bool ownForeignMemory, bool deleteMemory)
		{
			return new CImage(format, size, data, ownForeignMemory, deleteMemory);
		}

		IImage* CNullDriver::createImage(ECOLOR_FORMAT format, const core::dimension2d<u32>& size)
		{
			return new CImage(format, size);
		}

		bool CNullDriver::setRenderTarget(video::E_RENDER_TARGET target, bool clearTarget, bool clearZBuffer, SColor color)
		{
			return false;
		}

		IGPUProgrammingServices* CNullDriver::getGPUProgrammingServices()
		{
			return this;
		}

		s32 CNullDriver::addHighLevelShaderMaterial(const void *vertexShaderProgram, const void *fragmentShaderProgram, IShaderConstantSetCallback *callback, E_MATERIAL_TYPE baseMaterial, s32 userData)
		{
			return -1;
		}

		s32 CNullDriver::addHighLevelShaderMaterial(const io::path& vertexShaderProgram, const io::path& fragmentShaderProgram, IShaderConstantSetCallback *callback, E_MATERIAL_TYPE baseMaterial, s32 userData)
		{
			io::IReadFile *vsFile = NULL;
			io::IReadFile *fsFile = NULL;

			if(vertexShaderProgram.size())
				vsFile = _fileSystem->createAndOpenFile(vertexShaderProgram);

			if(fragmentShaderProgram.size())
				fsFile = _fileSystem->createAndOpenFile(fragmentShaderProgram);

			s32 result = addHighLevelShaderMaterial(vsFile, fsFile, callback, baseMaterial, userData);

			if(vsFile) vsFile->drop();
			if(fsFile) fsFile->drop();

			return result;
		}

		s32 CNullDriver::addHighLevelShaderMaterial(io::IReadFile *vertexShaderProgram, io::IReadFile *fragmentShaderProgram, IShaderConstantSetCallback *callback, E_MATERIAL_TYPE baseMaterial, s32 userData)
		{
			u8 *vs = NULL;
			u8 *fs = NULL;

			if(vertexShaderProgram) {
				const long size = vertexShaderProgram->getSize();
				if(size) {
					vs = new u8[size];
					vertexShaderProgram->read(vs, size);
				}
			}

			if(fragmentShaderProgram) {
				const long size = fragmentShaderProgram->getSize();
				if(size) {
					fs = new u8[size];
					fragmentShaderProgram->read(fs, size);
				}
			}

			return addHighLevelShaderMaterial(vs, fs, callback, baseMaterial, userData);
		}

		void CNullDriver::makeColorKeyTexture(video::ITexture *texture, video::SColor color) const
		{

		}

		void CNullDriver::makeColorKeyTexture(video::ITexture *texture, core::position2di colorKeyPixelPos) const
		{

		}

		void CNullDriver::makeNormalMapTexture(video::ITexture *texture, f32 amplitude) const
		{
			if(texture->getColorFormat() != ECF_A1R5G5B5 && texture->getColorFormat() != ECF_A8R8G8B8)
				return;

			core::dimension2d<u32> size = texture->getSize();
			if(texture->getColorFormat() == ECF_A8R8G8B8) {
				s32 *p = (s32*)texture->lock();

				if(p == NULL) return;

				u32 pitch = texture->getPitch()/4;
				s32 *in = new s32[size.width*size.height];

				memcpy(in, p, size.height*size.width*4);
				for(s32 x=0;x < s32(pitch);++x) {
					for(s32 y=0;y < s32(size.height);++y) {
						f32 dX = 0.0f;
						f32 dY = 0.0f;

						for(u32 i=0;i < _numKernelElems;i++)
							dX += nml32(x + _sobelKernel[0][i].x, y + _sobelKernel[0][i].y, pitch, size.height, in)*_sobelKernel[0][i].w;

						for(u32 i=0;i < _numKernelElems;i++)
							dY += nml32(x + _sobelKernel[1][i].x, y + _sobelKernel[1][i].y, pitch, size.height, in)*_sobelKernel[1][i].w;

						core::vector3df n = normalize(core::vector3df(-dX*amplitude, -dY*amplitude, 1.0f));

						n += core::vector3df(1.0f, 1.0f, 1.0f);
						n *= 127.5f;

						u32 height = (u32)(nml32(x, y, pitch, size.height, in)*255.0f);
						p[y*pitch + x] = video::SColor((s32)n.getX(), (s32)n.getY(), (s32)n.getZ(), height).toA8R8G8B8();
					}
				}

				delete [] in;
				texture->unlock();
			}
			texture->regenerateMipMapLevels();
		}

		void CNullDriver::setStencilShadowEnabled(bool enable)
		{
			_stencilShadowEnabled = enable;
		}

		bool CNullDriver::isStencilShadowEnabled() const
		{
			return _stencilShadowEnabled;
		}

		void CNullDriver::generateSobelKernel(s32 type)
		{
			_numKernelElems = 0;
			_sobelKernel[0] = NULL;
			_sobelKernel[1] = NULL;

			switch(type) {
				case SOBEL_KERNEL3x3:
					_numKernelElems = 6;
					_sobelKernel[0] = new SSobelKernelElement[_numKernelElems];
					_sobelKernel[1] = new SSobelKernelElement[_numKernelElems];

					_sobelKernel[0][0].x = -1; _sobelKernel[0][0].y =  1; _sobelKernel[0][0].w = -1.0f;
					_sobelKernel[0][1].x = -1; _sobelKernel[0][1].y =  0; _sobelKernel[0][1].w = -2.0f;
					_sobelKernel[0][2].x = -1; _sobelKernel[0][2].y = -1; _sobelKernel[0][2].w = -1.0f;
					_sobelKernel[0][3].x =  1; _sobelKernel[0][3].y =  1; _sobelKernel[0][3].w =  1.0f;
					_sobelKernel[0][4].x =  1; _sobelKernel[0][4].y =  0; _sobelKernel[0][4].w =  2.0f;
					_sobelKernel[0][5].x =  1; _sobelKernel[0][5].y = -1; _sobelKernel[0][5].w =  1.0f;

					_sobelKernel[1][0].x = -1; _sobelKernel[1][0].y =  1; _sobelKernel[1][0].w =  1.0f;
					_sobelKernel[1][1].x =  0; _sobelKernel[1][1].y =  1; _sobelKernel[1][1].w =  2.0f;
					_sobelKernel[1][2].x =  1; _sobelKernel[1][2].y =  1; _sobelKernel[1][2].w =  1.0f;
					_sobelKernel[1][3].x = -1; _sobelKernel[1][3].y = -1; _sobelKernel[1][3].w = -1.0f;
					_sobelKernel[1][4].x =  0; _sobelKernel[1][4].y = -1; _sobelKernel[1][4].w = -2.0f;
					_sobelKernel[1][5].x =  1; _sobelKernel[1][5].y = -1; _sobelKernel[1][5].w = -1.0f;
					break;
				case SOBEL_KERNEL5x5:
					_numKernelElems = 20;
					_sobelKernel[0] = new SSobelKernelElement[_numKernelElems];
					_sobelKernel[1] = new SSobelKernelElement[_numKernelElems];

					_sobelKernel[0][ 0].x = -2; _sobelKernel[0][ 0].y =  2; _sobelKernel[0][ 0].w = -1.0f;
					_sobelKernel[0][ 1].x = -2; _sobelKernel[0][ 1].y =  1; _sobelKernel[0][ 1].w = -4.0f;
					_sobelKernel[0][ 2].x = -2; _sobelKernel[0][ 2].y =  0; _sobelKernel[0][ 2].w = -6.0f;
					_sobelKernel[0][ 3].x = -2; _sobelKernel[0][ 3].y = -1; _sobelKernel[0][ 3].w = -4.0f;
					_sobelKernel[0][ 4].x = -2; _sobelKernel[0][ 4].y = -2; _sobelKernel[0][ 4].w = -1.0f;
					_sobelKernel[0][ 5].x = -1; _sobelKernel[0][ 5].y =  2; _sobelKernel[0][ 5].w = -2.0f;
					_sobelKernel[0][ 6].x = -1; _sobelKernel[0][ 6].y =  1; _sobelKernel[0][ 6].w = -8.0f;
					_sobelKernel[0][ 7].x = -1; _sobelKernel[0][ 7].y =  0; _sobelKernel[0][ 7].w = -12.0f;
					_sobelKernel[0][ 8].x = -1; _sobelKernel[0][ 8].y = -1; _sobelKernel[0][ 8].w = -8.0f;
					_sobelKernel[0][ 9].x = -1; _sobelKernel[0][ 9].y = -2; _sobelKernel[0][ 9].w = -2.0f;
					_sobelKernel[0][10].x =  1; _sobelKernel[0][10].y =  2; _sobelKernel[0][10].w =  2.0f;
					_sobelKernel[0][11].x =  1; _sobelKernel[0][11].y =  1; _sobelKernel[0][11].w =  8.0f;
					_sobelKernel[0][12].x =  1; _sobelKernel[0][12].y =  0; _sobelKernel[0][12].w =  12.0f;
					_sobelKernel[0][13].x =  1; _sobelKernel[0][13].y = -1; _sobelKernel[0][13].w =  8.0f;
					_sobelKernel[0][14].x =  1; _sobelKernel[0][14].y = -2; _sobelKernel[0][14].w =  2.0f;
					_sobelKernel[0][15].x =  2; _sobelKernel[0][15].y =  2; _sobelKernel[0][15].w =  1.0f;
					_sobelKernel[0][16].x =  2; _sobelKernel[0][16].y =  1; _sobelKernel[0][16].w =  4.0f;
					_sobelKernel[0][17].x =  2; _sobelKernel[0][17].y =  0; _sobelKernel[0][17].w =  6.0f;
					_sobelKernel[0][18].x =  2; _sobelKernel[0][18].y = -1; _sobelKernel[0][18].w =  4.0f;
					_sobelKernel[0][19].x =  2; _sobelKernel[0][19].y = -2; _sobelKernel[0][19].w =  1.0f;

					_sobelKernel[1][ 0].x = -2; _sobelKernel[1][ 0].y =  2; _sobelKernel[1][ 0].w =  1.0f;
					_sobelKernel[1][ 1].x = -1; _sobelKernel[1][ 1].y =  2; _sobelKernel[1][ 1].w =  4.0f;
					_sobelKernel[1][ 2].x =  0; _sobelKernel[1][ 2].y =  2; _sobelKernel[1][ 2].w =  6.0f;
					_sobelKernel[1][ 3].x =  1; _sobelKernel[1][ 3].y =  2; _sobelKernel[1][ 3].w =  4.0f;
					_sobelKernel[1][ 4].x =  2; _sobelKernel[1][ 4].y =  2; _sobelKernel[1][ 4].w =  1.0f;
					_sobelKernel[1][ 5].x = -2; _sobelKernel[1][ 5].y =  1; _sobelKernel[1][ 5].w =  2.0f;
					_sobelKernel[1][ 6].x = -1; _sobelKernel[1][ 6].y =  1; _sobelKernel[1][ 6].w =  8.0f;
					_sobelKernel[1][ 7].x =  0; _sobelKernel[1][ 7].y =  1; _sobelKernel[1][ 7].w =  12.0f;
					_sobelKernel[1][ 8].x =  1; _sobelKernel[1][ 8].y =  1; _sobelKernel[1][ 8].w =  8.0f;
					_sobelKernel[1][ 9].x =  2; _sobelKernel[1][ 9].y =  1; _sobelKernel[1][ 9].w =  2.0f;
					_sobelKernel[1][10].x = -2; _sobelKernel[1][10].y = -1; _sobelKernel[1][10].w = -2.0f;
					_sobelKernel[1][11].x = -1; _sobelKernel[1][11].y = -1; _sobelKernel[1][11].w = -8.0f;
					_sobelKernel[1][12].x =  0; _sobelKernel[1][12].y = -1; _sobelKernel[1][12].w = -12.0f;
					_sobelKernel[1][13].x =  1; _sobelKernel[1][13].y = -1; _sobelKernel[1][13].w = -8.0f;
					_sobelKernel[1][14].x =  2; _sobelKernel[1][14].y = -1; _sobelKernel[1][14].w = -2.0f;
					_sobelKernel[1][15].x = -2; _sobelKernel[1][15].y = -2; _sobelKernel[1][15].w = -1.0f;
					_sobelKernel[1][16].x = -1; _sobelKernel[1][16].y = -2; _sobelKernel[1][16].w = -4.0f;
					_sobelKernel[1][17].x =  0; _sobelKernel[1][17].y = -2; _sobelKernel[1][17].w = -6.0f;
					_sobelKernel[1][18].x =  1; _sobelKernel[1][18].y = -2; _sobelKernel[1][18].w = -4.0f;
					_sobelKernel[1][19].x =  2; _sobelKernel[1][19].y = -2; _sobelKernel[1][19].w = -1.0f;
					break;
				default:
					break;
			}
		}
	}
}
