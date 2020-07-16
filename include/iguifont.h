/*
 * iguifont.h
 *
 *  Created on: Mar 8, 2013
 *      Author: mike
 */

#ifndef IGUIFONT_H_
#define IGUIFONT_H_

#include "irefcounter.h"
#include "scolor.h"
#include "rect.h"
#include "irrstring.h"

namespace irr
{
	namespace gui
	{
		enum EGUI_FONT_TYPE
		{
			EGFT_BITMAP = 0,

			EGFT_VECTOR,

			EGFT_OS,

			EGFT_CUSTOM
		};

		class IGUIFont : public virtual IRefCounter
		{
		public:
			virtual void draw(const core::stringc& text, const core::rect<s32>& position, video::SColor color, bool hcenter = false, bool vcenter = false, const core::rect<s32> *clip = NULL) = 0;

			virtual core::dimension2d<u32> getDimension(const char *text) const = 0;

			virtual s32 getCharacterFromPos(const char *text, s32 pixel_x) const = 0;

			virtual EGUI_FONT_TYPE getType() const { return EGFT_CUSTOM; }

			virtual void setKerningWidth(s32 kerning) = 0;

			virtual void setKerningHeight(s32 kerning) = 0;

			virtual s32 getKerningWidth(const char *thisLetter = NULL, const char *previousLetter = NULL) const = 0;

			virtual s32 getKerningHeight() const = 0;

			virtual void setInvisibleCharacters(const char *s) = 0;
		};
	}
}


#endif /* IGUIFONT_H_ */
