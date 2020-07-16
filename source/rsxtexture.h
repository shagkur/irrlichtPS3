/*
 * rsxtexture.h
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#ifndef RSXTEXTURE_H_
#define RSXTEXTURE_H_

#include "itexture.h"
#include "iimage.h"
#include "smateriallayer.h"

#include "rsxstate.h"

#define DEFAULT_RSX_REMAPPING		((GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_B_SHIFT) | \
					 	 	 	 	 (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_G_SHIFT) | \
					 	 	 	 	 (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_R_SHIFT) | \
					 	 	 	 	 (GCM_TEXTURE_REMAP_TYPE_REMAP << GCM_TEXTURE_REMAP_TYPE_A_SHIFT) | \
					 	 	 	 	 (GCM_TEXTURE_REMAP_COLOR_B << GCM_TEXTURE_REMAP_COLOR_B_SHIFT) | \
					 	 	 	 	 (GCM_TEXTURE_REMAP_COLOR_G << GCM_TEXTURE_REMAP_COLOR_G_SHIFT) | \
					 	 	 	 	 (GCM_TEXTURE_REMAP_COLOR_R << GCM_TEXTURE_REMAP_COLOR_R_SHIFT) | \
					 	 	 	 	 (GCM_TEXTURE_REMAP_COLOR_A << GCM_TEXTURE_REMAP_COLOR_A_SHIFT))

namespace irr
{
	namespace video
	{
		class CRSXDriver;

		class CRSXTexture : public ITexture
		{
		public:
			CRSXTexture(const io::path& name, CRSXDriver *driver);
			CRSXTexture(IImage *surface, const io::path& name, CRSXDriver *driver);
			virtual ~CRSXTexture();

			virtual const core::dimension2d<u32>& getOriginalSize() const;
			virtual const core::dimension2d<u32>& getSize() const;

			virtual ECOLOR_FORMAT getColorFormat() const;

			virtual void* lock();
			virtual void unlock();

			virtual u32 getPitch() const;

			virtual bool hasMipMaps() const;

			virtual void regenerateMipMapLevels();

			virtual E_DRIVER_TYPE getDriverType() const { return EDT_RSX; }

			void init(void *buffer, u32 pitch, u32 width, u32 height, u32 numMips, ECOLOR_FORMAT format = ECF_A8R8G8B8);
			void init(u32 texOffs, u32 pitch, u32 width, u32 height, u32 numMips, ECOLOR_FORMAT format = ECF_A8R8G8B8);

			void setTextureControl(u32 enable, u16 minLod, u16 maxLod, u8 maxAniso);

			void setClamp(u8 wraps, u8 wrapt, u8 wrapr);

			void setTextureFilter(u16 bias, u8 min, u8 mag, u8 conv);

			void addToCmdBuffer(u8 texUnit);

			const gcmTexture& getRSXTexture() const;

		private:
			u32 getRSXFormatAndParametersFromColorFormat(ECOLOR_FORMAT format, u32& remap);

		protected:
			ECOLOR_FORMAT getBestColorFormat(ECOLOR_FORMAT format);

			void uploadTexture(IImage *image);

			void updateRSXTexture();

			void getImageValues(IImage *image);

			void uploadTextureScale(ECOLOR_FORMAT srcFormat, const u32 srcOffset, const u32 srcX, const u32 srcY, const u32 srcWidth, const u32 srcHeight, const u32 srcPitch, const u32 dstOffset, const u32 dstX, const u32 dstY, const u32 dstWidth, const u32 dstHeight, const u32 dstPitch, const u32 clipX, const u32 clipY, const u32 clipWidth, const u32 clipHeight, const u32 transferMode);

			u32 getSourceScaleFormat(ECOLOR_FORMAT format);

			core::dimension2d<u32> _imageSize;
			core::dimension2d<u32> _textureSize;

			ECOLOR_FORMAT _colorFormat;

			u32 _textureOffset;
			void *_textureData;

			bool _hasMipMaps;
			bool _isCompressed;

			u32 _texturePitch;

			gcmTexture _rsxTexture;

			CRSXDriver *_driver;

			CRSXState *_rsxState;

			textureControl_t textureControl;
			textureAddress_t textureAddress;
			textureFilter_t textureFilter;

		private:
			u32 _internalFormat;
			u32 _remapping;

			u32 _numMipMaps;
		};
	}
}
#endif /* RSXTEXTURE_H_ */
