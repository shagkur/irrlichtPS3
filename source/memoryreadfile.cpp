#include "memoryreadfile.h"
#include "irrstring.h"

namespace irr
{
	namespace io
	{
		CMemoryReadFile::CMemoryReadFile(void *ptr,s32 len,const io::path& filename,bool deleteMemoryWhenDropped)
		: _buffer(ptr),_len(len),_pos(0),_deleteMemoryWhenDropped(deleteMemoryWhenDropped)
		{
			_fileName = filename;
		}

		CMemoryReadFile::~CMemoryReadFile()
		{
			if(_deleteMemoryWhenDropped)
				delete [] (char*)_buffer;
		}

		s32 CMemoryReadFile::read(void *buffer,u32 sizeToRead)
		{
			char *p;
			s32 amount = static_cast<s32>(sizeToRead);

			if(_pos+amount>_len) amount -= (_pos+amount-_len);
			if(amount<=0) return 0;

			p = (char*)_buffer;
			memcpy(buffer,p+_pos,amount);

			_pos += amount;
			return amount;
		}

		bool CMemoryReadFile::seek(s32 finalPos,bool relMovement)
		{
			if(relMovement) {
				if((_pos+finalPos)>_len) return false;

				_pos += finalPos;
			} else {
				if(finalPos>_len) return false;

				_pos = finalPos;
			}
			return true;
		}

		s32 CMemoryReadFile::getSize() const
		{
			return _len;
		}

		s32 CMemoryReadFile::getPos() const
		{
			return _pos;
		}

		const io::path& CMemoryReadFile::getFilename() const
		{
			return _fileName;
		}

		IReadFile* createMemoryReadFile(void *mem,u32 size,const io::path& filename,bool deleteMemoryWhenDropped)
		{
			CMemoryReadFile *file = new CMemoryReadFile(mem,size,filename,deleteMemoryWhenDropped);
			return file;
		}
	}
}
