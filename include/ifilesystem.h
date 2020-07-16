#ifndef __IFILESYSTEM_H__
#define __IFILESYSTEM_H__

#include "irefcounter.h"
#include "ireadfile.h"

namespace irr
{
	namespace video
	{
		class IVideoDriver;
	}

	namespace io
	{
		enum E_FILESYSTEM_TYPE
		{
			EFST_NATIVE = 0,

			EFST_VIRTUAL,

			EFST_COUNT
		};

		class IFileSystem : public virtual IRefCounter
		{
		public:
			virtual IReadFile* createAndOpenFile(const io::path& filename) = 0;
			virtual IReadFile* createMemoryReadFile(void *memory,s32 len,const io::path& filename,bool deleteMemoryWhenDropped = false) = 0;

			virtual io::path getFileDir(const io::path& filename) const = 0;

			virtual bool existFile(const io::path& fileName) const = 0;

			virtual io::path getFileBasename(const io::path& filename,bool keepExtension = true) const = 0;

			virtual const io::path& getWorkingDirectory() = 0;

			virtual bool changeWorkingDirectory(const io::path& newDirectory) = 0;

			virtual io::path getAbsolutePath(const io::path& filename) const = 0;
		};
	}
}

#endif
