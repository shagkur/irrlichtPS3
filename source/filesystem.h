#ifndef __FILESYSTEM_H__
#define __FILESYSTEM_H__

#include "irrarray.h"
#include "ifilesystem.h"

namespace irr
{
	namespace io
	{
		class CFileSystem : public IFileSystem
		{
		public:
			CFileSystem();
			virtual ~CFileSystem();

			virtual IReadFile* createAndOpenFile(const io::path& filename);
			virtual IReadFile* createMemoryReadFile(void *memory,s32 len,const io::path& filename,bool deleteMemoryWhenDropped = false);

			virtual io::path getFileDir(const io::path& filename) const;

			virtual bool existFile(const io::path& fileName) const;

			virtual io::path getFileBasename(const io::path& filename,bool keepExtension = true) const;

			virtual const io::path& getWorkingDirectory();

			virtual bool changeWorkingDirectory(const io::path& newDirectory);

			virtual io::path getAbsolutePath(const io::path& filename) const;

		private:
			E_FILESYSTEM_TYPE _fileSystemType;

			io::path _workingDirectory[EFST_COUNT];
		};
	}
}

#endif
