#ifndef __READFILE_H__
#define __READFILE_H__

#include <stdio.h>
#include "ireadfile.h"
#include "irrstring.h"

namespace irr
{
	namespace io
	{
		class CReadFile : public IReadFile
		{
		public:
			CReadFile(const io::path& filename);
			virtual ~CReadFile();

			virtual s32 read(void *buffer,u32 sizeToRead);
			virtual bool seek(s32 finalPos,bool relMovement = false);

			virtual s32 getSize() const;
			virtual s32 getPos() const;

			virtual const io::path& getFilename() const;

			virtual bool isOpen() const
			{
				return (_file!=NULL);
			}

		private:
			void openFile();

			io::path _fileName;
			FILE *_file;
			u32 _fileSize;
		};
	}
}

#endif
