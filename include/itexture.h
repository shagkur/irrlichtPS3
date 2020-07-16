/*
 * itexture.h
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#ifndef ITEXTURE_H_
#define ITEXTURE_H_

#include "irefcounter.h"
#include "dimension2d.h"
#include "path.h"
#include "iimage.h"
#include "edrivertypes.h"

namespace irr
{
	namespace video
	{
		enum E_TEXTURE_CREATION_FLAG
		{
			ETCF_ALWAYS_16_BIT = 0x00000001,

			ETCF_ALWAYS_32_BIT = 0x00000002,

			ETCF_OPTIMIZED_FOR_QUALITY = 0x00000004,

			ETCF_OPTIMIZED_FOR_SPEED = 0x00000008,

			ETCF_CREATE_MIP_MAPS = 0x00000010,

			ETCF_NO_ALPHA_CHANNEL = 0x00000020,

			ETCF_ALLOW_NON_POWER_2 = 0x00000040,

			ETCF_FORCE_32_BIT_DO_NOT_USE = 0x7fffffff
		};

		class ITexture : public virtual IRefCounter
		{
		public:
			ITexture(const io::path& name) : _namedPath(name)
			{
			}

			virtual const core::dimension2d<u32>& getOriginalSize() const = 0;
			virtual const core::dimension2d<u32>& getSize() const = 0;

			virtual ECOLOR_FORMAT getColorFormat() const = 0;

			virtual void* lock() = 0;
			virtual void unlock() = 0;

			virtual u32 getPitch() const = 0;

			virtual bool hasMipMaps() const { return false; }

			virtual bool hasAlpha() const
			{
				return (getColorFormat() == video::ECF_A8R8G8B8 || getColorFormat() == video::ECF_A1R5G5B5);
			}

			virtual void regenerateMipMapLevels() = 0;

			virtual E_DRIVER_TYPE getDriverType() const = 0;

			const io::SNamedPath& getName() const { return _namedPath; }

		protected:
			io::SNamedPath _namedPath;
		};
	}
}

#endif /* ITEXTURE_H_ */
