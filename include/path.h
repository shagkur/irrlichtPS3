/*
 * path.h
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#ifndef PATH_H_
#define PATH_H_

#include "irrstring.h"

namespace irr
{
	namespace io
	{
		typedef core::string<fschar_t> path;

		struct SNamedPath
		{
			SNamedPath() {}
			SNamedPath(const path& p) : _path(p), _internalName(pathToName(p))
			{
			}

			bool operator < (const SNamedPath& other) const
			{
				return _internalName < other._internalName;
			}

			void setPath(const path& p)
			{
				_path = p;
				_internalName = pathToName(p);
			}

			const path& getPath() const
			{
				return _path;
			}

			const path& getInternalName() const
			{
				return _internalName;
			}

			operator core::stringc() const
			{
				return core::stringc(getPath());
			}

		protected:
			path pathToName(const path& p) const
			{
				path name(p);
				name.replace('\\', '/');
				name.make_lower();
				return name;
			}

		private:
			path _path;
			path _internalName;
		};
	}
}

#endif /* PATH_H_ */
