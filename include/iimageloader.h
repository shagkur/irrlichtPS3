/*
 * iimageloader.h
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#ifndef IIMAGELOADER_H_
#define IIMAGELOADER_H_

#include "irefcounter.h"
#include "iimage.h"
#include "path.h"

namespace irr
{
	namespace io
	{
		class IReadFile;
	}

	namespace video
	{
		class IImageLoader : public virtual IRefCounter
		{
		public:
			virtual bool isLoadableFileExtension(const io::path& filename) const = 0;

			virtual bool isLoadableFileFormat(io::IReadFile *file) const = 0;

			virtual IImage* loadImage(io::IReadFile *file) const = 0;
		};
	} // namespace video
}  // namespace irr

#endif /* IIMAGELOADER_H_ */
