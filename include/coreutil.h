/*
 * coreutil.h
 *
 *  Created on: Feb 19, 2013
 *      Author: mike
 */

#ifndef COREUTIL_H_
#define COREUTIL_H_

#include "irrstring.h"
#include "path.h"

namespace irr
{
	namespace core
	{
		inline s32 isFileExtension(const io::path& filename, const io::path& ext0, const io::path& ext1, const io::path& ext2)
		{
			s32 extPos = filename.findLast('.');
			if(extPos < 0) return 0;

			extPos++;
			if(filename.equals_substring_ignore_case(ext0, extPos)) return 1;
			if(filename.equals_substring_ignore_case(ext1, extPos)) return 2;
			if(filename.equals_substring_ignore_case(ext2, extPos)) return 3;

			return 0;
		}

		inline bool hasFileExtension(const io::path& filename, const io::path& ext0, const io::path& ext1 = "", const io::path& ext2 = "")
		{
			return isFileExtension(filename, ext0, ext1, ext2) > 0;
		}

		inline io::path& cutFilenameExtension(io::path& dest, const io::path& source)
		{
			s32 endPos = source.findLast('.');
			dest = source.subString(0, endPos < 0 ? source.size() : endPos);
			return dest;
		}
	}
}

#endif /* COREUTIL_H_ */
