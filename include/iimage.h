/*
 * iimage.h
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#ifndef IIMAGE_H_
#define IIMAGE_H_

#include "irefcounter.h"
#include "scolor.h"
#include "rect.h"
#include "position2d.h"
#include "iheapmanager.h"

namespace irr
{
	namespace video
	{
		class IImage : public IRefCounter
		{
		public:
			virtual void* lock() = 0;
			virtual void unlock() = 0;

			virtual const core::dimension2d<u32>& getDimension() const = 0;

			virtual u32 getBitsPerPixel() const = 0;
			virtual u32 getBytesPerPixel() const = 0;

			virtual ECOLOR_FORMAT getColorFormat() const = 0;

			virtual u32 getPitch() const = 0;

			virtual void copyToScaling(void *target, u32 width, u32 height, ECOLOR_FORMAT format = ECF_A8R8G8B8, u32 pitch = 0) = 0;
			virtual void copyToScaling(IImage *target) = 0;

			virtual void copyTo(IImage *target, const core::position2d<s32>& pos = core::position2d<s32>(0, 0)) = 0;
			virtual void copyTo(IImage *target, const core::position2d<s32>& pos, const core::rect<s32>& sourceRect, const core::rect<s32> *clipRect) = 0;

			virtual void fill(const SColor& color) = 0;

			static u32 getBitsPerPixelFromFormat(const ECOLOR_FORMAT format)
			{
				switch(format) {
					case ECF_DXT1:
					case ECF_R16F:
					case ECF_R5G6B5:
					case ECF_A1R5G5B5:
						return 16;
					case ECF_R8G8B8:
						return 24;
					case ECF_R32F:
					case ECF_DXT2:
					case ECF_DXT3:
					case ECF_DXT4:
					case ECF_DXT5:
					case ECF_G16R16F:
					case ECF_A8R8G8B8:
						return 32;
					case ECF_A16B16G16R16F:
					case ECF_G32R32F:
						return 64;
					case ECF_A32B32G32R32F:
						return 128;
					default:
						return 0;
				}
			}

			static bool isCompressed(const ECOLOR_FORMAT format)
			{
				switch(format) {
					case ECF_DXT1:
					case ECF_DXT2:
					case ECF_DXT3:
					case ECF_DXT4:
					case ECF_DXT5:
						return true;
					default:
						return false;
				}
			}
		};
	}  // namespace video
}  // namespace irr

#endif /* IIMAGE_H_ */
