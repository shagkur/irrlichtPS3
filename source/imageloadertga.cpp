/*
 * imageloadertga.cpp
 *
 *  Created on: Feb 14, 2014
 *      Author: mike
 */

#include "colorconverter.h"
#include "ireadfile.h"
#include "scolor.h"
#include "image.h"
#include "os.h"

#include "imageloadertga.h"

namespace irr
{
	namespace video
	{
		CImageLoaderTGA::CImageLoaderTGA()
		{

		}

		CImageLoaderTGA::~CImageLoaderTGA()
		{

		}

		bool CImageLoaderTGA::isLoadableFileExtension(const io::path& filename) const
		{
			return core::hasFileExtension(filename, "tga");
		}

		bool CImageLoaderTGA::isLoadableFileFormat(io::IReadFile *file) const
		{
			if(file == NULL)
				return false;

			STGAFooter footer;

			memset(&footer, 0, sizeof(STGAFooter));

			file->seek(file->getSize() - sizeof(STGAFooter));
			file->read(&footer, sizeof(STGAFooter));

			return !strcmp(footer.signature, "TRUEVISION-XFILE.");		//very old TGAs are refused.
		}

		IImage* CImageLoaderTGA::loadImage(io::IReadFile *file) const
		{
			STGAHeader header;
			u32 *palette = NULL;

			file->read(&header, sizeof(STGAHeader));

			header.colorMapLength = os::ByteSwap::byteswap(header.colorMapLength);
			header.imageWidth = os::ByteSwap::byteswap(header.imageWidth);
			header.imageHeight = os::ByteSwap::byteswap(header.imageHeight);

			if(header.idLength)
				file->seek(header.idLength, true);

			if(header.colorMapType) {
				u8 *colorMap = new u8[header.colorMapEntrySize/8*header.colorMapLength];

				palette = new u32[header.colorMapLength];
				file->read(colorMap, header.colorMapEntrySize/8*header.colorMapLength);

				switch(header.colorMapEntrySize) {
					case 16:
						CColorConverter::convert_A1R5G5B5toA8R8G8B8(colorMap, header.colorMapLength, palette);
						break;
					case 24:
						CColorConverter::convert_B8G8R8toA8R8G8B8(colorMap, header.colorMapLength, palette);
						break;
					case 32:
						CColorConverter::convert_B8G8R8A8toA8R8G8B8(colorMap, header.colorMapLength, palette);
						break;
				}
				delete [] colorMap;
			}

			u8 *data = NULL;
			if(header.imageType == 1 || header.imageType == 2 || header.imageType == 3) {
				const s32 imageSize = header.imageHeight*header.imageWidth*header.pixelDepth/8;

				data = new u8[imageSize];
				file->read(data, imageSize);
			} else if(header.imageType == 10) {
				data = loadCompressedImage(file, header);
			} else {
				printf("unsupported TGA file type.\n");

				delete [] palette;
				return NULL;
			}

			IImage *image = NULL;
			switch(header.pixelDepth) {
				case 8:
					image = new CImage(ECF_A8R8G8B8, core::dimension2d<u32>(header.imageWidth, header.imageHeight));
					if(header.imageType == 3)
						CColorConverter::convert8BitTo32Bit((u8*)data, (u8*)image->lock(), header.imageWidth, header.imageHeight, NULL, 0, (header.imageDescriptor&0x20) == 0);
					else
						CColorConverter::convert8BitTo32Bit((u8*)data, (u8*)image->lock(), header.imageWidth, header.imageHeight, (u8*)palette, 0, (header.imageDescriptor&0x20) == 0);
					break;
				case 16:
					image = new CImage(ECF_A8R8G8B8, core::dimension2d<u32>(header.imageWidth, header.imageHeight));
					CColorConverter::convert16BitTo32Bit((s16*)data, (s32*)image->lock(), header.imageWidth, header.imageHeight, 0, (header.imageDescriptor&0x20) == 0);
					break;
				case 24:
					image = new CImage(ECF_A8R8G8B8, core::dimension2d<u32>(header.imageWidth, header.imageHeight));
					CColorConverter::convert24BitTo32Bit((u8*)data, (s32*)image->lock(), header.imageWidth, header.imageHeight, 0, (header.imageDescriptor&0x20) == 0, true);
					break;
				case 32:
					image = new CImage(ECF_A8R8G8B8, core::dimension2d<u32>(header.imageWidth, header.imageHeight));
					CColorConverter::convert32BitTo32Bit((s32*)data, (s32*)image->lock(), header.imageWidth, header.imageHeight, 0, (header.imageDescriptor&0x20) == 0);
					break;
				default:
					printf("unsupported TGA format.\n");
					break;
			}

			if(image != NULL) image->unlock();
			return image;
		}

		u8* CImageLoaderTGA::loadCompressedImage(io::IReadFile *file, const STGAHeader& header) const
		{
			s32 currentByte = 0;
			s32 bytesPerPixel = header.pixelDepth/8;
			s32 imageSize = header.imageHeight*header.imageWidth*bytesPerPixel;
			u8 *data = new u8[imageSize];

			while(currentByte < imageSize) {
				u8 chunkHeader = 0;

				file->read(&chunkHeader, sizeof(u8));

				if(chunkHeader < 128) {
					chunkHeader++;

					file->read(&data[currentByte], bytesPerPixel*chunkHeader);
					currentByte += bytesPerPixel*chunkHeader;
				} else {
					s32 dataOffset = currentByte;

					chunkHeader -= 127;

					file->read(&data[dataOffset], bytesPerPixel);
					currentByte += bytesPerPixel;

					for(s32 counter=1;counter < chunkHeader;counter++) {
						for(s32 elementCounter=0;elementCounter < bytesPerPixel;elementCounter++)
							data[currentByte + elementCounter] = data[dataOffset + elementCounter];

						currentByte += bytesPerPixel;
					}
				}
			}

			return data;
		}

		IImageLoader* createImageLoaderTGA()
		{
			return new CImageLoaderTGA();
		}
	}
}
