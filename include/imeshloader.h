/*
 * imeshloader.h
 *
 *  Created on: Feb 18, 2013
 *      Author: mike
 */

#ifndef IMESHLOADER_H_
#define IMESHLOADER_H_

#include "irefcounter.h"
#include "path.h"

namespace irr
{
	namespace io
	{
		class IReadFile;
	}

	namespace scene
	{
		class IAnimatedMesh;

		class IMeshLoader : public virtual IRefCounter
		{
		public:
			virtual ~IMeshLoader() {}

			virtual bool isLoadableFileExtension(const io::path& filename)  const = 0;

			virtual IAnimatedMesh* createMesh(io::IReadFile *file) = 0;
		};
	}
}

#endif /* IMESHLOADER_H_ */
