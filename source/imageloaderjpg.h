/*
 * imageloaderjpg.h
 *
 *  Created on: Feb 21, 2013
 *      Author: mike
 */

#ifndef IMAGELOADERJPG_H_
#define IMAGELOADERJPG_H_

#include "iimageloader.h"

#define JPG_STREAMBUF_SIZE			4096

namespace irr
{
	namespace video
	{
		class CImageLoaderJPG : public IImageLoader
		{
		public:
			CImageLoaderJPG();
			virtual ~CImageLoaderJPG();

			virtual bool isLoadableFileExtension(const io::path& filename) const;

			virtual bool isLoadableFileFormat(io::IReadFile *file) const;

			virtual IImage* loadImage(io::IReadFile *file) const;

		private:
			s32 _jpgDecHandle;
			vu32 _jpgDecCommand;
			u8 _jpgStrmBuffer[JPG_STREAMBUF_SIZE];
		};
	}
}

#endif /* IMAGELOADERJPEG_H_ */
