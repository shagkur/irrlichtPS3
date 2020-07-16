#ifndef __IREADFILE_H__
#define __IREADFILE_H__

#include "irefcounter.h"
#include "coreutil.h"

namespace irr
{
	namespace io
	{
		class IReadFile : public virtual IRefCounter
		{
		public:
			virtual s32 read(void *buffer,u32 sizeToRead) = 0;
			virtual bool seek(s32 finalPos,bool relMovment = false) = 0;

			virtual s32 getSize() const = 0;
			virtual s32 getPos() const = 0;

			virtual const io::path& getFilename() const = 0;
		};

		IReadFile* createReadFile(const io::path& filename);
		IReadFile* createMemoryReadFile(void *mem,s32 size,const io::path& filename,bool deleteMemoryWhenDropped);
		IReadFile* createLimitReadFile(const io::path& filename,IReadFile *file,s32 areaSize);
	}
}

#endif
