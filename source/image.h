/*
 * image.h
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include "iimage.h"

namespace irr
{
	namespace video
	{
		class CImage : public IImage
		{
		public:
			CImage(ECOLOR_FORMAT format, const core::dimension2d<u32>& size, void *data, bool ownForeignMemory = true, bool deleteMemory = true, bool compressed = false, bool mipMaps = false);
			CImage(ECOLOR_FORMAT format, const core::dimension2d<u32>& size);
			virtual ~CImage();

			virtual void* lock()
			{
				return _data;
			}

			virtual void unlock() {}

			virtual const core::dimension2d<u32>& getDimension() const;

			virtual u32 getBitsPerPixel() const;
			virtual u32 getBytesPerPixel() const;

			virtual ECOLOR_FORMAT getColorFormat() const;

			virtual u32 getPitch() const { return _pitch; }

			virtual void copyToScaling(void *target, u32 width, u32 height, ECOLOR_FORMAT format = ECF_A8R8G8B8, u32 pitch = 0);
			virtual void copyToScaling(IImage *target);

			virtual void copyTo(IImage *target, const core::position2d<s32>& pos = core::position2d<s32>(0, 0));
			virtual void copyTo(IImage *target, const core::position2d<s32>& pos, const core::rect<s32>& sourceRect, const core::rect<s32> *clipRect);

			virtual void fill(const SColor& color);

		private:
			void initData();

			static void initHeap();

			u8 *_data;
			core::dimension2d<u32> _size;
			u32 _bytesPerPixel;
			u32 _pitch;
			ECOLOR_FORMAT _format;

			bool _isCompressed;
			bool _hasMipMaps;

			bool _deleteMemory;
		};
	}
}

#endif /* IMAGE_H_ */
