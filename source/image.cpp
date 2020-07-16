/*
 * image.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#include "image.h"
#include "colorconverter.h"
#include "blit.h"
#include "os.h"

#define IMAGE_HEAP_PAGE_ALIGNMENT	(1024*1024)
#define IMAGE_HEAP_SIZE				(32*1024*1024)

namespace irr
{
	namespace video
	{
		CImage::CImage(ECOLOR_FORMAT format, const core::dimension2d<u32>& size)
		: _data(NULL), _size(size), _format(format), _isCompressed(false), _hasMipMaps(false), _deleteMemory(true)
		{
			initData();
		}

		CImage::CImage(ECOLOR_FORMAT format, const core::dimension2d<u32>& size, void *data, bool ownForeignMemory, bool deleteMemory, bool compressed, bool mipMaps)
		: _data(NULL), _size(size), _format(format), _isCompressed(compressed), _hasMipMaps(mipMaps), _deleteMemory(deleteMemory)
		{
			if(ownForeignMemory) {
				_data = (u8*)0x0badf00d;
				initData();
				_data = (u8*)data;
			} else {
				_data = NULL;
				initData();
				memcpy(_data, data, size.height*_pitch);
			}
		}

		CImage::~CImage()
		{
			if(_deleteMemory) IHeapManager::deallocate(_data);
		}

		const core::dimension2d<u32>& CImage::getDimension() const
		{
			return _size;
		}

		u32 CImage::getBitsPerPixel() const
		{
			return getBitsPerPixelFromFormat(_format);
		}

		u32 CImage::getBytesPerPixel() const
		{
			return _bytesPerPixel;
		}

		ECOLOR_FORMAT CImage::getColorFormat() const
		{
			return _format;
		}

		void CImage::copyToScaling(void *target, u32 width, u32 height, ECOLOR_FORMAT format, u32 pitch)
		{
			if(_isCompressed) return;

			if(target == NULL || width == 0 || height == 0) return;

			const u32 bpp = getBitsPerPixelFromFormat(format)/8;
			if(pitch == 0) pitch = width*bpp;

			if(_format == format && _size.width == width && _size.height == height) {
				if(pitch == _pitch) {
					memcpy(target, _data, height*pitch);
					return;
				} else {
					u8 *tgtpos = (u8*)target;
					u8 *srcpos = _data;
					const u32 bwidth = width*bpp;
					const u32 rest = pitch - bwidth;

					for(u32 y=0;y < height;y++) {
						memcpy(tgtpos, srcpos, bwidth);
						memset(tgtpos + bwidth, 0, rest);

						tgtpos += pitch;
						srcpos += _pitch;
					}

					return;
				}
			}

			f32 sy = 0.0f;
			s32 yval = 0, syval = 0;
			const f32 sourceXStep = (f32)_size.width/(f32)width;
			const f32 sourceYStep = (f32)_size.height/(f32)height;
			for(u32 y=0;y < height;y++) {
				f32 sx = 0.0f;
				for(u32 x=0;x < width;x++) {
					CColorConverter::convert_viaFormat(_data + syval + ((s32)sx)*_bytesPerPixel, _format, 1, ((u8*)target) + yval + (x*bpp), format);
					sx += sourceXStep;
				}

				sy += sourceYStep;
				syval = ((s32)sy)*_pitch;
				yval += pitch;
			}
		}

		void CImage::copyToScaling(IImage *target)
		{
			if(_isCompressed) return;

			if(target == NULL) return;

			const core::dimension2d<u32>& targetSize = target->getDimension();
			if(targetSize == _size) {
				copyTo(target);
				return;
			}

			copyToScaling(target->lock(), targetSize.width, targetSize.height, target->getColorFormat());
			target->unlock();
		}

		void CImage::copyTo(IImage *target, const core::position2d<s32>& pos)
		{
			if(_isCompressed) return;

			if(target == NULL) return;

			const core::dimension2d<u32>& targetSize = target->getDimension();
			copyToScaling(target->lock(), targetSize.width, targetSize.height, target->getColorFormat());
			target->unlock();
			//Blit(BLITTER_TEXTURE, target, NULL, &pos, this, NULL, 0);
		}

		void CImage::copyTo(IImage *target, const core::position2d<s32>& pos, const core::rect<s32>& sourceRect, const core::rect<s32> *clipRect)
		{
			if(_isCompressed) return;

			Blit(BLITTER_TEXTURE, target, clipRect, &pos, this, &sourceRect, 0);
		}

		void CImage::fill(const SColor& color)
		{

		}

		void CImage::initData()
		{
			_bytesPerPixel = getBitsPerPixelFromFormat(_format)/8;
			_pitch = _bytesPerPixel*_size.width;

			if(_data == NULL) {
				_deleteMemory = true;
				_data = (u8*)IHeapManager::allocate(64, _size.height*_pitch);
			}
		}
	}
}
