/*
 * debugfontrenderer.h
 *
 *  Created on: Jun 17, 2013
 *      Author: mike
 */

#ifndef DEBUGFONTRENDERER_H_
#define DEBUGFONTRENDERER_H_

#include "debug/debugfont.h"

class DebugFontRenderer
{
public:
	DebugFontRenderer();
	virtual ~DebugFontRenderer();

	virtual void init() = 0;
	virtual void shutdown() = 0;

	virtual void printStart(f32 r, f32 g, f32 b, f32 a) = 0;
	virtual void printPass(DebugFont::Position *pPositions, DebugFont::TexCoord *pTexCoords, DebugFont::Color *pColors, s32 numVerts) = 0;
	virtual void printEnd() = 0;

protected:
	static inline u8* getFontData()
	{
		return DebugFont::sFontData;
	}

	static f32 sR;
	static f32 sG;
	static f32 sB;
	static f32 sA;
};

#endif /* DEBUGFONTRENDERER_H_ */
