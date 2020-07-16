#include "readfile.h"

namespace irr
{
	namespace io
	{
		CReadFile::CReadFile(const io::path& filename) : _file(NULL),_fileSize(0)
		{
			_fileName = filename;
			openFile();
		}

		CReadFile::~CReadFile()
		{
			if(_file) fclose(_file);
		}

		s32 CReadFile::read(void *buffer,u32 sizeToRead)
		{
			if(!isOpen()) return 0;
			return (s32)fread(buffer,1,sizeToRead,_file);
		}

		bool CReadFile::seek(s32 finalPos,bool relMovement)
		{
			if(!isOpen()) return false;
			return fseek(_file,finalPos,relMovement?SEEK_CUR:SEEK_SET)==0;
		}

		s32 CReadFile::getSize() const
		{
			return _fileSize;
		}

		s32 CReadFile::getPos() const
		{
			if(!_file) return 0;
			return ftell(_file);
		}

		const io::path& CReadFile::getFilename() const
		{
			return _fileName;
		}

		void CReadFile::openFile()
		{
			if(_fileName.size()==0) {
				_file = NULL;
				return;
			}

			_file = fopen(_fileName.c_str(),"rb");
			if(_file) {
				fseek(_file,0,SEEK_END);
				_fileSize = ftell(_file);
				fseek(_file,0,SEEK_SET);
			}
		}

		IReadFile* createReadFile(const io::path& filename)
		{

			CReadFile *file = new CReadFile(filename);	
			if(file->isOpen()) return file;

			file->drop();
			return NULL;
		}
	}
}
