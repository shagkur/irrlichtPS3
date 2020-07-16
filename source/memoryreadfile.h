#ifndef __MEMORYREADFILE_H__
#define __MEMORYREADFILE_H__

#include "ireadfile.h"

namespace irr
{
	namespace io
	{
		class CMemoryReadFile : public IReadFile
		{
		public:
			CMemoryReadFile(void *ptr,s32 len,const io::path& filename,bool deleteMemoryWhenDropped);
			virtual ~CMemoryReadFile();

			virtual s32 read(void *buffer,u32 sizeToRead);
			virtual bool seek(s32 finalPos,bool relMovement = false);

			virtual s32 getSize() const;
			virtual s32 getPos() const;

			virtual const io::path& getFilename() const;

		private:
			io::path _fileName;
			void *_buffer;
			s32 _len;
			s32 _pos;
			bool _deleteMemoryWhenDropped;
		};
	}
}

#endif
