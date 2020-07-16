/*
 * rsxtexture.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#include "irrtypes.h"
#include "rsxtexture.h"
#include "rsxdriver.h"
#include "os.h"
#include "irrstring.h"

namespace irr
{
	namespace video
	{
		CRSXTexture::CRSXTexture(IImage *surface, const io::path& name, CRSXDriver *driver)
		: ITexture(name), _colorFormat(ECF_A8R8G8B8),  _textureData(NULL), _isCompressed(false),
		  _texturePitch(0), _driver(driver), _rsxState(driver->getRSXState()), _internalFormat(GCM_TEXTURE_FORMAT_A8R8G8B8),
		  _remapping(DEFAULT_RSX_REMAPPING), _numMipMaps(1)
		{
			_hasMipMaps = driver->getTextureCreationFlag(ETCF_CREATE_MIP_MAPS);

			getImageValues(surface);

			if(_colorFormat == ECF_DXT1 || _colorFormat == ECF_DXT2 || _colorFormat == ECF_DXT3 || _colorFormat == ECF_DXT4 || _colorFormat == ECF_DXT5) {
				if(_imageSize != _textureSize) {
					return;
				}

				_isCompressed = true;

			}

			textureControl.enable = GCM_TRUE;
			textureControl.minLod = 0<<8;
			textureControl.maxLod = 12<<8;
			textureControl.maxAniso = GCM_TEXTURE_MAX_ANISO_1;

			textureAddress.wraps = GCM_TEXTURE_REPEAT;
			textureAddress.wrapt = GCM_TEXTURE_REPEAT;
			textureAddress.wrapr = GCM_TEXTURE_MIRRORED_REPEAT;
			textureAddress.unsignedRemap = GCM_TEXTURE_UNSIGNED_REMAP_NORMAL;
			textureAddress.zfunc = GCM_TEXTURE_ZFUNC_GREATER;
			textureAddress.gamma = 0;

			textureFilter.bias = 0;
			textureFilter.min = GCM_TEXTURE_LINEAR_MIPMAP_LINEAR;
			textureFilter.mag = GCM_TEXTURE_LINEAR;
			textureFilter.conv = GCM_TEXTURE_CONVOLUTION_QUINCUNX;

			uploadTexture(surface);
		}

		CRSXTexture::CRSXTexture(const io::path& name, CRSXDriver *driver)
		: ITexture(name)
		{

		}

		CRSXTexture::~CRSXTexture()
		{
			if(_textureData != NULL) rsxFree(_textureData);
		}

		const core::dimension2d<u32>& CRSXTexture::getOriginalSize() const
		{
			return _imageSize;
		}

		const core::dimension2d<u32>& CRSXTexture::getSize() const
		{
			return _textureSize;
		}

		ECOLOR_FORMAT CRSXTexture::getColorFormat() const
		{
			return _colorFormat;
		}

		void* CRSXTexture::lock()
		{
			return _textureData;
		}

		void CRSXTexture::unlock()
		{
		}

		u32 CRSXTexture::getPitch() const
		{
			return _texturePitch;
		}

		bool CRSXTexture::hasMipMaps() const
		{
			return _hasMipMaps;
		}

		void CRSXTexture::regenerateMipMapLevels()
		{
			if(!_hasMipMaps) return;

			_numMipMaps = 1;

			u32 dstY = 0;
			u32 srcPitch = _texturePitch;
			u32 dstWidth = _textureSize.width>>1;
			u32 dstHeight = _textureSize.height>>1;
			while((dstWidth > 1 || dstHeight > 1) && _numMipMaps <= 13) {
				_numMipMaps++;

				dstY += dstHeight;
				uploadTextureScale(_colorFormat, _textureOffset, 0, 0, _textureSize.width, _textureSize.height, srcPitch, _textureOffset, 0, dstY, dstWidth, dstHeight, _texturePitch, 0, dstY, _textureSize.width, dstHeight, GCM_TRANSFER_LOCAL_TO_LOCAL);

				if(dstWidth > 1) dstWidth >>= 1;
				if(dstHeight > 1) dstHeight >>= 1;
			}

			_driver->waitRSXFinish();

			updateRSXTexture();
		}

		void CRSXTexture::init(void *buffer, u32 pitch, u32 width, u32 height, u32 numMips, ECOLOR_FORMAT format)
		{
			_textureData = buffer;
			_numMipMaps = numMips;
			_hasMipMaps = (numMips > 1);
			_texturePitch = pitch;
			_textureSize = core::dimension2d<u32>(width, height);
			_internalFormat = getRSXFormatAndParametersFromColorFormat(format, _remapping);

			rsxAddressToOffset(buffer, &_textureOffset);

			updateRSXTexture();
		}

		void CRSXTexture::init(u32 texOffs, u32 pitch, u32 width, u32 height, u32 numMips, ECOLOR_FORMAT format)
		{
			_textureOffset = texOffs;
			_numMipMaps = numMips;
			_hasMipMaps = (numMips > 1);
			_texturePitch = pitch;
			_textureSize = core::dimension2d<u32>(width, height);
			_internalFormat = getRSXFormatAndParametersFromColorFormat(format, _remapping);

			gcmIoOffsetToAddress(texOffs, &_textureData);

			updateRSXTexture();
		}

		void CRSXTexture::setTextureControl(u32 enable, u16 minLod, u16 maxLod, u8 maxAniso)
		{
			textureControl.enable = enable;
			textureControl.minLod = minLod;
			textureControl.maxLod = maxLod;
			textureControl.maxAniso = maxAniso;
		}

		void CRSXTexture::setClamp(u8 wraps, u8 wrapt, u8 wrapr)
		{
			textureAddress.wraps = wraps;
			textureAddress.wrapt = wrapt;
			textureAddress.wrapr = wrapr;
		}

		void CRSXTexture::setTextureFilter(u16 bias, u8 min, u8 mag, u8 conv)
		{
			textureFilter.bias = bias;
			textureFilter.min = min;
			textureFilter.mag = mag;
			textureFilter.conv = conv;
		}

		void CRSXTexture::addToCmdBuffer(u8 texUnit)
		{
			gcmContextData *context = _driver->getGcmContext();

			if(textureControl.enable != _rsxState->state.textureControl[texUnit].enable ||
			   textureControl.minLod != _rsxState->state.textureControl[texUnit].minLod ||
			   textureControl.maxLod != _rsxState->state.textureControl[texUnit].maxLod ||
			   textureControl.maxAniso != _rsxState->state.textureControl[texUnit].maxAniso)
			{
				rsxTextureControl(context, texUnit, textureControl.enable, textureControl.minLod, textureControl.maxLod, textureControl.maxAniso);

				_rsxState->state.textureControl[texUnit].enable = textureControl.enable;
				_rsxState->state.textureControl[texUnit].minLod = textureControl.minLod;
				_rsxState->state.textureControl[texUnit].maxLod = textureControl.maxLod;
				_rsxState->state.textureControl[texUnit].maxAniso = textureControl.maxAniso;
			}

			if(textureAddress.wraps != _rsxState->state.textureAddress[texUnit].wraps ||
			   textureAddress.wrapt != _rsxState->state.textureAddress[texUnit].wrapt ||
			   textureAddress.wrapr != _rsxState->state.textureAddress[texUnit].wrapr ||
			   textureAddress.unsignedRemap != _rsxState->state.textureAddress[texUnit].unsignedRemap ||
			   textureAddress.zfunc != _rsxState->state.textureAddress[texUnit].zfunc ||
			   textureAddress.gamma != _rsxState->state.textureAddress[texUnit].gamma)
			{
				rsxTextureWrapMode(context, texUnit, textureAddress.wraps, textureAddress.wrapt, textureAddress.wrapr, textureAddress.unsignedRemap, textureAddress.zfunc, textureAddress.gamma);

				_rsxState->state.textureAddress[texUnit].wraps = textureAddress.wraps;
				_rsxState->state.textureAddress[texUnit].wrapt = textureAddress.wrapt;
				_rsxState->state.textureAddress[texUnit].wrapr = textureAddress.wrapr;
				_rsxState->state.textureAddress[texUnit].unsignedRemap = textureAddress.unsignedRemap;
				_rsxState->state.textureAddress[texUnit].zfunc = textureAddress.zfunc;
				_rsxState->state.textureAddress[texUnit].gamma = textureAddress.gamma;
			}

			if(textureFilter.bias != _rsxState->state.textureFilter[texUnit].bias ||
			   textureFilter.min != _rsxState->state.textureFilter[texUnit].min ||
			   textureFilter.mag != _rsxState->state.textureFilter[texUnit].mag ||
			   textureFilter.conv != _rsxState->state.textureFilter[texUnit].conv)
			{
				rsxTextureFilter(context, texUnit, textureFilter.bias, textureFilter.min, textureFilter.mag, textureFilter.conv);

				_rsxState->state.textureFilter[texUnit].bias = textureFilter.bias;
				_rsxState->state.textureFilter[texUnit].min = textureFilter.min;
				_rsxState->state.textureFilter[texUnit].mag = textureFilter.mag;
				_rsxState->state.textureFilter[texUnit].conv = textureFilter.conv;
			}

			if(_rsxState->state.currentTexture[texUnit] != &_rsxTexture) {
				rsxLoadTexture(context, texUnit, &_rsxTexture);

				_rsxState->state.currentTexture[texUnit] = &_rsxTexture;
			}
		}

		const gcmTexture& CRSXTexture::getRSXTexture() const
		{
			return _rsxTexture;
		}

		void CRSXTexture::updateRSXTexture()
		{
			_rsxTexture.format		= (_internalFormat | GCM_TEXTURE_FORMAT_LIN);
			_rsxTexture.mipmap		= _numMipMaps;
			_rsxTexture.dimension	= GCM_TEXTURE_DIMS_2D;
			_rsxTexture.cubemap		= GCM_FALSE;
			_rsxTexture.remap		= _remapping;
			_rsxTexture.width		= _textureSize.width;
			_rsxTexture.height		= _textureSize.height;
			_rsxTexture.depth		= 1;
			_rsxTexture.location	= GCM_LOCATION_RSX;
			_rsxTexture.pitch		= _texturePitch;
			_rsxTexture.offset		= _textureOffset;
		}

		void CRSXTexture::getImageValues(IImage *image)
		{
			if(image == NULL) return;

			_imageSize = image->getDimension();
			if(_imageSize.width == 0 || _imageSize.height == 0) return;

			const s32 ratio = (f32)_imageSize.width/(f32)_imageSize.height;
			if(_imageSize.width > _driver->_maxTextureSize && ratio >= 1.0f) {
				_imageSize.width = _driver->_maxTextureSize;
				_imageSize.height = (u32)(_driver->_maxTextureSize/ratio);
			}
			else if(_imageSize.height > _driver->_maxTextureSize) {
				_imageSize.height = _driver->_maxTextureSize;
				_imageSize.width = (u32)(_driver->_maxTextureSize*ratio);
			}
			_textureSize = _imageSize.getOptimalSize();

			if(image->getColorFormat() == ECF_DXT1 || image->getColorFormat() == ECF_DXT2 || image->getColorFormat() == ECF_DXT3 || image->getColorFormat() == ECF_DXT4 || image->getColorFormat() == ECF_DXT5)
				_colorFormat = image->getColorFormat();
			else
				_colorFormat = getBestColorFormat(image->getColorFormat());
		}

		void CRSXTexture::uploadTexture(IImage *image)
		{
			if(image == NULL) return;

			u32 srcOffset;
			u32 textureByteSize;
			u8 *srcPtr = (u8*)image->lock();
			const u32 bpp = image->getBytesPerPixel();
			core::dimension2d<u32> imageSize = image->getDimension();

			_numMipMaps = 1;
			_texturePitch = _textureSize.width*bpp;
			_internalFormat = getRSXFormatAndParametersFromColorFormat(_colorFormat, _remapping);

			printf("imageSize: %dx%d\n", imageSize.width, imageSize.height);
			printf("_colorFormat = %d\n", _colorFormat);
			printf("_textureSize = %dx%d\n", _textureSize.width, _textureSize.height);

			textureByteSize = _textureSize.height*_texturePitch;
			if(_hasMipMaps) {
				if(core::ispoweroftwo(_textureSize.width) && core::ispoweroftwo(_textureSize.height))
					textureByteSize *= 2;
				else
					_hasMipMaps = false;
			}

			_textureData = rsxMemalign(128, textureByteSize);
			printf("textureData: %p\n", _textureData);
			rsxAddressToOffset(_textureData, &_textureOffset);

			rsxAddressToOffset(srcPtr, &srcOffset);

			u32 dstY = 0;
			u32 srcPitch = imageSize.width*bpp;
			u32 dstWidth = _textureSize.width;
			u32 dstHeight = _textureSize.height;
			do {
				uploadTextureScale(image->getColorFormat(), srcOffset, 0, 0, imageSize.width, imageSize.height, srcPitch, _textureOffset, 0, dstY, dstWidth, dstHeight, _texturePitch, 0, dstY, _textureSize.width, dstHeight, GCM_TRANSFER_MAIN_TO_LOCAL);
				if(!_hasMipMaps) break;

				dstY += dstHeight;

				if(dstWidth > 1) dstWidth >>= 1;
				if(dstHeight > 1) dstHeight >>= 1;

				_numMipMaps++;
			} while((dstWidth > 1 || dstHeight > 1) && _numMipMaps <= 13);

			_driver->waitRSXFinish();

			printf("numMipMaps = %d\n", _numMipMaps);
			updateRSXTexture();
			image->unlock();
		}

		void CRSXTexture::uploadTextureScale(ECOLOR_FORMAT srcFormat, const u32 srcOffset, const u32 srcX, const u32 srcY, const u32 srcWidth, const u32 srcHeight, const u32 srcPitch, const u32 dstOffset, const u32 dstX, const u32 dstY, const u32 dstWidth, const u32 dstHeight, const u32 dstPitch, const u32 clipX, const u32 clipY, const u32 clipWidth, const u32 clipHeight, const u32 transferMode)
		{
			gcmTransferScale srcParam;
			gcmTransferSurface dstParam;

			srcParam.conversion = GCM_TRANSFER_CONVERSION_DITHER;
			srcParam.format = getSourceScaleFormat(srcFormat);
			srcParam.origin = GCM_TRANSFER_ORIGIN_CORNER;
			srcParam.operation = GCM_TRANSFER_OPERATION_SRCCOPY;
			srcParam.interp = GCM_TRANSFER_INTERPOLATOR_LINEAR;
			srcParam.clipX = clipX;
			srcParam.clipY = clipY;
			srcParam.clipW = clipWidth;
			srcParam.clipH = clipHeight;
			srcParam.outX = dstX;
			srcParam.outY = dstY;
			srcParam.outW = dstWidth;
			srcParam.outH = dstHeight;
			srcParam.ratioX = rsxGetFixedSint32((f32)srcWidth/(f32)dstWidth);
			srcParam.ratioY = rsxGetFixedSint32((f32)srcHeight/(f32)dstHeight);
			srcParam.inX = rsxGetFixedUint16((f32)srcX);
			srcParam.inY = rsxGetFixedUint16((f32)srcY);
			srcParam.inW = srcWidth;
			srcParam.inH = srcHeight;
			srcParam.offset = srcOffset;
			srcParam.pitch = srcPitch;

			dstParam.offset = dstOffset;
			dstParam.pitch = dstPitch;
			dstParam.format = GCM_TRANSFER_SURFACE_FORMAT_A8R8G8B8;

			rsxSetTransferScaleMode(_driver->getGcmRootContext(), transferMode, GCM_TRANSFER_SURFACE);
			rsxSetTransferScaleSurface(_driver->getGcmRootContext(), &srcParam, &dstParam);
		}

		ECOLOR_FORMAT CRSXTexture::getBestColorFormat(ECOLOR_FORMAT format)
		{
			ECOLOR_FORMAT destFormat = ECF_A8R8G8B8;

			switch(format) {
				case ECF_A1R5G5B5:
					if(!_driver->getTextureCreationFlag(ETCF_ALWAYS_32_BIT))
						destFormat = ECF_A1R5G5B5;
					break;
				case ECF_R5G6B5:
					if(_driver->getTextureCreationFlag(ETCF_ALWAYS_32_BIT))
						destFormat = ECF_R5G6B5;
					break;
				case ECF_A8R8G8B8:
					if(_driver->getTextureCreationFlag(ETCF_ALWAYS_16_BIT) || _driver->getTextureCreationFlag(ETCF_OPTIMIZED_FOR_SPEED))
						destFormat = ECF_A1R5G5B5;
					break;
				case ECF_R8G8B8:
					if(_driver->getTextureCreationFlag(ETCF_ALWAYS_16_BIT) || _driver->getTextureCreationFlag(ETCF_OPTIMIZED_FOR_SPEED))
						destFormat = ECF_A1R5G5B5;
					break;
				default:
					break;
			}

			if(_driver->getTextureCreationFlag(ETCF_NO_ALPHA_CHANNEL)) {
				switch(destFormat) {
					case ECF_A1R5G5B5:
						destFormat = ECF_R5G6B5;
						break;
					case ECF_A8R8G8B8:
						destFormat = ECF_R8G8B8;
						break;
					default:
						break;
				}
			}

			return destFormat;
		}

		u32 CRSXTexture::getRSXFormatAndParametersFromColorFormat(ECOLOR_FORMAT format, u32& remap)
		{
			u32 internalformat = GCM_TEXTURE_FORMAT_A8R8G8B8;

			remap = DEFAULT_RSX_REMAPPING;
			switch(format) {
				case ECF_A1R5G5B5:
					internalformat = GCM_TEXTURE_FORMAT_A1R5G5B5;
					break;
				case ECF_R5G6B5:
					internalformat = GCM_TEXTURE_FORMAT_R5G6B5;
					remap = ((GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_B_SHIFT) |
							 (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_G_SHIFT) |
							 (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_R_SHIFT) |
							 (GCM_TEXTURE_REMAP_TYPE_ONE << GCM_TEXTURE_REMAP_TYPE_A_SHIFT) |
							 (GCM_TEXTURE_REMAP_COLOR_B << GCM_TEXTURE_REMAP_COLOR_B_SHIFT) |
							 (GCM_TEXTURE_REMAP_COLOR_G << GCM_TEXTURE_REMAP_COLOR_G_SHIFT) |
							 (GCM_TEXTURE_REMAP_COLOR_R << GCM_TEXTURE_REMAP_COLOR_R_SHIFT) |
							 (GCM_TEXTURE_REMAP_COLOR_A << GCM_TEXTURE_REMAP_COLOR_A_SHIFT));
					break;
				case ECF_R8G8B8:
					remap = ((GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_B_SHIFT) |
							 (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_G_SHIFT) |
							 (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_R_SHIFT) |
							 (GCM_TEXTURE_REMAP_TYPE_ONE << GCM_TEXTURE_REMAP_TYPE_A_SHIFT) |
							 (GCM_TEXTURE_REMAP_COLOR_B << GCM_TEXTURE_REMAP_COLOR_B_SHIFT) |
							 (GCM_TEXTURE_REMAP_COLOR_G << GCM_TEXTURE_REMAP_COLOR_G_SHIFT) |
							 (GCM_TEXTURE_REMAP_COLOR_R << GCM_TEXTURE_REMAP_COLOR_R_SHIFT) |
							 (GCM_TEXTURE_REMAP_COLOR_A << GCM_TEXTURE_REMAP_COLOR_A_SHIFT));
					break;
				case ECF_G16R16F:
					internalformat = GCM_TEXTURE_FORMAT_Y16_X16_FLOAT;
					break;
				case ECF_A8R8G8B8:
				default:
					break;
			}

			return internalformat;
		}

		u32 CRSXTexture::getSourceScaleFormat(ECOLOR_FORMAT format)
		{
			switch(format) {
				case ECF_A1R5G5B5:
					return GCM_TRANSFER_SCALE_FORMAT_A1R5G5B5;
				case ECF_R5G6B5:
					return GCM_TRANSFER_SCALE_FORMAT_R5G6B5;
				case ECF_R8G8B8:
					return GCM_TRANSFER_SCALE_FORMAT_X8R8G8B8;
				case ECF_A8R8G8B8:
					return GCM_TRANSFER_SCALE_FORMAT_A8R8G8B8;
				default:
					break;
			}

			return GCM_TRANSFER_SCALE_FORMAT_A8R8G8B8;
		}
	}
}

