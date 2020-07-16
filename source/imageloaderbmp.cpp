/*
 * imageloaderbmp.cpp
 *
 *  Created on: Feb 13, 2014
 *      Author: mike
 */

#include "colorconverter.h"
#include "ireadfile.h"
#include "scolor.h"
#include "image.h"
#include "os.h"

#include "imageloaderbmp.h"

namespace irr
{
	namespace video
	{
		CImageLoaderBMP::CImageLoaderBMP()
		{

		}

		CImageLoaderBMP::~CImageLoaderBMP()
		{

		}

		bool CImageLoaderBMP::isLoadableFileExtension(const io::path& filename) const
		{
			return core::hasFileExtension(filename, "bmp");
		}

		bool CImageLoaderBMP::isLoadableFileFormat(io::IReadFile *file) const
		{
			u16 headerId;

			file->read(&headerId, sizeof(u16));
			headerId = os::ByteSwap::byteswap(headerId);

			return headerId == 0x4d42;
		}

		void CImageLoaderBMP::decompress4BitRLE(u8*& bmpData, s32 size, s32 width, s32 height, s32 pitch) const
		{
			u8 *p = bmpData;
			s32 lineWidth = (width + 1)/2 + pitch;
			u8 *newBmp = new u8[lineWidth*height];
			u8 *d = newBmp;
			u8 *destEnd = newBmp + (lineWidth*height);
			s32 line = 0;
			s32 shift = 4;

			while(bmpData - p < size && d < destEnd) {
				if(*p == 0) {
					++p;

					switch(*p) {
						case 0:
							++p;
							++line;
							d = newBmp + (line*lineWidth);
							shift = 4;
							break;
						case 1:
							delete [] bmpData;
							bmpData = newBmp;
							return;
						case 2:
						{
							++p;

							s32 x = *p; ++p;
							s32 y = *p; ++p;

							d += x/2 + y*lineWidth;
							shift = x%2 == 0 ? 4 : 0;
						}
						break;

						default:
						{
							s32 count = *p; ++p;
							s32 readAdditional = (2 - (count%2))%2;
							s32 readShift = 4;

							for(s32 i=0;i < count;i++) {
								s32 color = (*p>>readShift)&0x0f;

								readShift -= 4;
								if(readShift < 0) {
									++p;
									readShift = 4;
								}

								u8 mask = 0x0f<<shift;

								*d = ((*d&~mask)|((color<<shift)&mask));
								shift -= 4;
								if(shift < 0) {
									++d;
									shift = 4;
								}
							}

							for(s32 i=0;i < readAdditional;i++)
								++p;
						}
						break;
					}
				} else {
					s32 count = *p; ++p;
					s32 color1 = *p&0x0f;
					s32 color2 = (*p>>4)&0x0f;

					for(s32 i=0;i < count;i++) {
						u8 mask = 0x0f<<shift;
						u8 toSet = (shift == 0 ? color1 : color2)<<shift;

						*d = (*d&~mask)|(toSet&mask);
						shift -= 4;
						if(shift < 0) {
							++d;
							shift = 4;
						}
					}
				}
			}

			delete [] bmpData;
			bmpData = newBmp;
		}

		void CImageLoaderBMP::decompress8BitRLE(u8*& bmpData, s32 size, s32 width, s32 height, s32 pitch) const
		{
			u8 *p = bmpData;
			u8 *newBmp = new u8[(width + pitch)*height];
			u8 *d = newBmp;
			u8 *destEnd = newBmp + (width + pitch)*height;
			s32 line = 0;

			while(bmpData - p < size && d < destEnd) {
				if(*p == 0) {
					++p;

					switch(*p) {
						case 0:
							++p;
							++line;
							d = newBmp + (line*(width + pitch));
							break;
						case 1:
							delete [] bmpData;
							bmpData = newBmp;
							return;
						case 2:
							++p; d += *p;
							++p; d += *p*(width + pitch);
							++p;
							break;
						default:
						{
							s32 count = *p; ++p;
							s32 readAdditional = (2 - (count%2))%2;

							for(s32 i=0;i < count;i++) {
								*d = *p;
								++p;
								++d;
							}

							for(s32 i=0;i < readAdditional;i++)
								++p;
						}
						break;
					}
				} else {
					s32 count = *p; ++p;
					u8 color = *p; ++p;

					for(s32 i=0;i < count;i++) {
						*d = color;
						++d;
					}
				}
			}

			delete [] bmpData;
			bmpData = newBmp;
		}

		IImage* CImageLoaderBMP::loadImage(io::IReadFile *file) const
		{
			SBMPHeader header;

			file->read(&header, sizeof(SBMPHeader));

			header.id = os::ByteSwap::byteswap(header.id);
			header.fileSize = os::ByteSwap::byteswap(header.fileSize);
			header.bitmapDataOffset = os::ByteSwap::byteswap(header.bitmapDataOffset);
			header.bitmapHeaderSize = os::ByteSwap::byteswap(header.bitmapHeaderSize);
			header.width = os::ByteSwap::byteswap(header.width);
			header.height = os::ByteSwap::byteswap(header.height);
			header.planes = os::ByteSwap::byteswap(header.planes);
			header.bpp = os::ByteSwap::byteswap(header.bpp);
			header.compression = os::ByteSwap::byteswap(header.compression);
			header.bitmapDataSize = os::ByteSwap::byteswap(header.bitmapDataSize);
			header.pixelPerMeterX = os::ByteSwap::byteswap(header.pixelPerMeterX);
			header.pixelPerMeterY = os::ByteSwap::byteswap(header.pixelPerMeterY);
			header.colors = os::ByteSwap::byteswap(header.colors);
			header.importantColors = os::ByteSwap::byteswap(header.importantColors);

			s32 pitch = 0;

			if(header.id != 0x4d42)
				return NULL;

			if(header.compression > 2)
				return NULL;

			header.bitmapDataSize += (4 - (header.bitmapDataSize%4))%4;

			s32 *paletteData = NULL;
			long pos = file->getPos();
			s32 paletteSize = (header.bitmapDataOffset  - pos)/4;

			if(paletteSize > 0) {
				paletteData = new s32[paletteSize];

				file->read(paletteData, paletteSize*sizeof(s32));
				for(s32 i=0;i < paletteSize;i++)
					paletteData[i] = os::ByteSwap::byteswap(paletteData[i]);
			}

			if(!header.bitmapDataSize)
				header.bitmapDataSize = static_cast<u32>(file->getSize() - header.bitmapDataOffset);

			file->seek(header.bitmapDataOffset);

			f32 t = header.width*(header.bpp/8.0f);
			s32 widthInBytes = (s32)t;

			t -= widthInBytes;
			if(t != 0.0f)
				++widthInBytes;

			s32 lineData = widthInBytes + (4 - (widthInBytes%4))%4;

			pitch = lineData - widthInBytes;

			u8 *bmpData = new u8[header.bitmapDataSize];

			file->read(bmpData, header.bitmapDataSize);

			switch(header.compression) {
				case 1:
					decompress8BitRLE(bmpData, header.bitmapDataSize, header.width, header.height, pitch);
					break;
				case 2:
					decompress4BitRLE(bmpData, header.bitmapDataSize, header.width, header.height, pitch);
					break;
			}

			IImage *image = NULL;
			core::dimension2d<u32> size(header.width, header.height);

			switch(header.bpp) {
				case 1:
					image = new CImage(ECF_A8R8G8B8, size);
					CColorConverter::convert1BitTo32Bit(bmpData, (s32*)image->lock(), header.width, header.height, pitch, true);
					break;
				case 4:
					image = new CImage(ECF_A8R8G8B8, size);
					CColorConverter::convert4BitTo32Bit(bmpData, (s32*)image->lock(), header.width, header.height, paletteData, pitch, true);
					break;
				case 8:
					image = new CImage(ECF_A8R8G8B8, size);
					CColorConverter::convert8BitTo32Bit(bmpData, (u8*)image->lock(), header.width, header.height, (u8*)paletteData, pitch, true);
					break;
				case 16:
					image = new CImage(ECF_A8R8G8B8, size);
					CColorConverter::convert16BitTo32Bit((s16*)bmpData, (s32*)image->lock(), header.width, header.height, pitch, true);
					break;
				case 24:
					image = new CImage(ECF_A8R8G8B8, size);
					CColorConverter::convert24BitTo32Bit(bmpData, (s32*)image->lock(), header.width, header.height, pitch, true, true);
					break;
				case 32:
					image = new CImage(ECF_A8R8G8B8, size);
					CColorConverter::convert32BitTo32Bit((s32*)bmpData, (s32*)image->lock(), header.width, header.height, pitch, true);
					break;
			}

			if(image != NULL) image->unlock();

			delete [] paletteData;
			delete [] bmpData;

			return image;
		}

		IImageLoader* createImageLoaderBMP()
		{
			return new CImageLoaderBMP();
		}
	}
}
