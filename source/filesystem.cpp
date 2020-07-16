#include "filesystem.h"
#include "readfile.h"
#include "memoryreadfile.h"

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace irr
{
	namespace io
	{
		CFileSystem::CFileSystem()
		{
		}

		CFileSystem::~CFileSystem()
		{
		}

		IReadFile* CFileSystem::createAndOpenFile(const io::path& filename)
		{
			IReadFile *file = NULL;
			file = createReadFile(filename);
			return file;
		}

		IReadFile* CFileSystem::createMemoryReadFile(void *memory,s32 len,const io::path& filename,bool deleteMemoryWhenDropped)
		{
			if(!memory) 
				return NULL;
			else
				return new CMemoryReadFile(memory,len,filename,deleteMemoryWhenDropped);
			return NULL;
		}

		io::path CFileSystem::getFileDir(const io::path& filename) const
		{
			s32 lastSlash = filename.findLast('/');
			s32 lastbackSlash = filename.findLast('\\');

			lastSlash = lastSlash>lastbackSlash ? lastSlash : lastbackSlash;
			if((u32)lastSlash<filename.size())
				return filename.subString(0,lastSlash);
			else
				return ".";
		}

		bool CFileSystem::existFile(const io::path& fileName) const
		{
			struct stat s;

			if(stat(fileName.c_str(), &s)) return false;

			return true;
		}

		io::path CFileSystem::getFileBasename(const io::path& filename,bool keepExtension) const
		{
			s32 end = 0;
			s32 lastSlash = filename.findLast('/');
			const s32 lastBackSlash = filename.findLast('\\');
			
			lastSlash = core::max_(lastSlash,lastBackSlash);
			if(!keepExtension) {
				end = filename.findLast('.');
				if(end==-1) end = 0;
				else end = filename.size() - end;
			}

			if((u32)lastSlash<filename.size())
				return filename.subString(lastSlash+1,filename.size()-lastSlash-1-end);
			else if(end!=0)
				return filename.subString(0,filename.size()-end);
			else
				return filename;
		}

		const io::path& CFileSystem::getWorkingDirectory()
		{
			E_FILESYSTEM_TYPE type = _fileSystemType;

			if(type != EFST_NATIVE)
				type = EFST_VIRTUAL;
			else {
				u32 pathSize = 256;
				char *tmpPath = new char[pathSize];

				while(pathSize < (1<<16) && !getcwd(tmpPath, pathSize)) {
					delete [] tmpPath;
					pathSize *= 2;
					tmpPath = new char[pathSize];
				}

				if(tmpPath) {
					_workingDirectory[EFST_NATIVE] = tmpPath;
					delete [] tmpPath;
				}

				_workingDirectory[type].validate();
			}

			return _workingDirectory[type];
		}

		bool CFileSystem::changeWorkingDirectory(const io::path& newDirectory)
		{
			bool success = false;

			if(_fileSystemType != EFST_NATIVE) {
				_workingDirectory[EFST_VIRTUAL] = newDirectory;
			} else {
				_workingDirectory[EFST_NATIVE] = newDirectory;

				success = (chdir(newDirectory.c_str()) == 0);
			}

			return success;
		}

		io::path CFileSystem::getAbsolutePath(const io::path& filename) const
		{
			return io::path(filename);
		}

		IFileSystem* createFileSystem()
		{
			return new CFileSystem();
		}
	}
}
