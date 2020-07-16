/*
 * imageloaderpng.h
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#ifndef IMAGELOADERPNG_H_
#define IMAGELOADERPNG_H_

#include "iimageloader.h"

#define PNG_STREAMBUF_SIZE				4096

namespace irr
{
	namespace video
	{
		class CImageLoaderPNG : public IImageLoader
		{
		public:
			CImageLoaderPNG();
			virtual ~CImageLoaderPNG();

			virtual bool isLoadableFileExtension(const io::path& filename) const;

			virtual bool isLoadableFileFormat(io::IReadFile *file) const;

			virtual IImage* loadImage(io::IReadFile *file) const;

		private:
			s32 _pngDecHandle;
			u8 _pngStrmBuffer[PNG_STREAMBUF_SIZE];
		};
	}
}

#endif /* IMAGELOADERPNG_H_ */
