/*
 * imageloaderbmp.h
 *
 *  Created on: Feb 13, 2014
 *      Author: mike
 */

#ifndef IMAGELOADERBMP_H_
#define IMAGELOADERBMP_H_

#include "iimageloader.h"

namespace irr
{
	namespace video
	{
		struct SBMPHeader
		{
			u16 id;
			u32 fileSize;
			u32 reserved;
			u32 bitmapDataOffset;
			u32 bitmapHeaderSize;
			u32 width;
			u32 height;
			u16 planes;
			u16 bpp;
			u32 compression;
			u32 bitmapDataSize;
			u32 pixelPerMeterX;
			u32 pixelPerMeterY;
			u32 colors;
			u32 importantColors;
		} __attribute__((packed));

		class CImageLoaderBMP : public IImageLoader
		{
		public:
			CImageLoaderBMP();
			virtual ~CImageLoaderBMP();

			virtual bool isLoadableFileExtension(const io::path& filename) const;

			virtual bool isLoadableFileFormat(io::IReadFile *file) const;

			virtual IImage* loadImage(io::IReadFile *file) const;

		private:
			void decompress4BitRLE(u8*& bmpData, s32 size, s32 width, s32 height, s32 pitch) const;
			void decompress8BitRLE(u8*& bmpData, s32 size, s32 width, s32 height, s32 pitch) const;
		};
	}
}

#endif /* IMAGELOADERBMP_H_ */
