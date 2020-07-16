/*
 * colorconverter.h
 *
 *  Created on: Feb 22, 2013
 *      Author: mike
 */

#ifndef COLORCONVERTER_H_
#define COLORCONVERTER_H_

#include "irrtypes.h"
#include "iimage.h"

namespace irr
{
	namespace video
	{
		class CColorConverter
		{
		public:
			static void convert1BitTo16Bit(const u8 *in, s16 *out, s32 width, s32 height, s32 linepad = 0, bool flip = false);
			static void convert1BitTo32Bit(const u8 *in, s32 *out, s32 width, s32 height, s32 linepad = 0, bool flip = false);
			static void convert4BitTo16Bit(const u8 *in, s16 *out, s32 width, s32 height, const s32 *palette, s32 linepad = 0, bool flip = false);
			static void convert4BitTo32Bit(const u8 *in, s32 *out, s32 width, s32 height, const s32 *palette, s32 linepad = 0, bool flip = false);
			static void convert8BitTo16Bit(const u8 *in, s16 *out, s32 width, s32 height, const s32 *palette, s32 linepad = 0, bool flip = false);
			static void convert8BitTo24Bit(const u8 *in, u8 *out, s32 width, s32 height, const u8 *palette, s32 linepad = 0, bool flip = false);
			static void convert8BitTo32Bit(const u8 *in, u8 *out, s32 width, s32 height, const u8 *palette, s32 linepad = 0, bool flip = false);
			static void convert16BitTo16Bit(const s16 *in, s16 *out, s32 width, s32 height, s32 linepad = 0, bool flip = false);
			static void convert16BitTo32Bit(const s16 *in, s32 *out, s32 width, s32 height, s32 linepad = 0, bool flip = false);
			static void convert24BitTo24Bit(const u8 *in, u8 *out, s32 width, s32 height, s32 linepad = 0, bool flip = false, bool bgr = false);
			static void convert24BitTo32Bit(const u8 *in, s32 *out, s32 width, s32 height, s32 linepad = 0, bool flip = false, bool bgr = false);
			static void convert16BitToA8R8G8B8AndResize(const s16 *in, s32 *out, s32 newWidth, s32 newHeight, s32 currentWidth, s32 currentHeight);
			static void convert32BitTo32Bit(const s32 *in, s32 *out, s32 width, s32 height, s32 linepad = 0, bool flip = false);

			static void convert_A1R5G5B5toR8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_A1R5G5B5toB8G8R8(const void *sP, s32 sN, void *dP);
			static void convert_A1R5G5B5toA8R8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_A1R5G5B5toA1R5G5B5(const void *sP, s32 sN, void *dP);
			static void convert_A1R5G5B5toR5G6B5(const void *sP, s32 sN, void *dP);

			static void convert_A8R8G8B8toR8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_A8R8G8B8toB8G8R8(const void *sP, s32 sN, void *dP);
			static void convert_A8R8G8B8toA8R8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_A8R8G8B8toA1R5G5B5(const void *sP, s32 sN, void *dP);
			static void convert_A8R8G8B8toR5G6B5(const void *sP, s32 sN, void *dP);

			static void convert_A8R8G8B8toR3G3B2(const void *sP, s32 sN, void *dP);
			static void convert_R8G8B8toR8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_R8G8B8toA8R8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_R8G8B8toA1R5G5B5(const void *sP, s32 sN, void *dP);
			static void convert_R8G8B8toR5G6B5(const void *sP, s32 sN, void *dP);
			static void convert_B8G8R8toA8R8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_B8G8R8A8toA8R8G8B8(const void *sP, s32 sN, void *dP);

			static void convert_R5G6B5toR5G6B5(const void *sP, s32 sN, void *dP);
			static void convert_R5G6B5toR8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_R5G6B5toB8G8R8(const void *sP, s32 sN, void *dP);
			static void convert_R5G6B5toA8R8G8B8(const void *sP, s32 sN, void *dP);
			static void convert_R5G6B5toA1R5G5B5(const void *sP, s32 sN, void *dP);

			static void convert_viaFormat(const void *sP, ECOLOR_FORMAT sF, s32 sN, void *dP, ECOLOR_FORMAT dF);
		};
	}
}
#endif /* COLORCONVERTER_H_ */
