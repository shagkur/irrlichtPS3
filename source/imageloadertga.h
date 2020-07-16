/*
 * imageloadertga.h
 *
 *  Created on: Feb 14, 2014
 *      Author: mike
 */

#ifndef IMAGELOADERTGA_H_
#define IMAGELOADERTGA_H_

#include "iimageloader.h"

namespace irr
{
	namespace video
	{
		struct STGAHeader
		{
			u8 idLength;
			u8 colorMapType;
			u8 imageType;
			u8 firstEntryIndex[2];
			u16 colorMapLength;
			u8 colorMapEntrySize;
			u8 xOrigin[2];
			u8 yOrigin[2];
			u16 imageWidth;
			u16 imageHeight;
			u8 pixelDepth;
			u8 imageDescriptor;
		} __attribute__((packed));

		struct STGAFooter
		{
			u32 extensionOffset;
			u32 developerOffset;
			char signature[18];
		} __attribute__((packed));

		class CImageLoaderTGA : public IImageLoader
		{
		public:
			CImageLoaderTGA();
			virtual ~CImageLoaderTGA();

			virtual bool isLoadableFileExtension(const io::path& filename) const;

			virtual bool isLoadableFileFormat(io::IReadFile *file) const;

			virtual IImage* loadImage(io::IReadFile *file) const;

		private:
			u8* loadCompressedImage(io::IReadFile *file, const STGAHeader& header) const;
		};
	}
}

#endif /* IMAGELOADERTGA_H_ */
