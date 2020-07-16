/*
 * debugfontrenderer.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: mike
 */

#include "debug/debugfont.h"
#include "debug/debugfontrenderer.h"

f32 DebugFontRenderer::sR;
f32 DebugFontRenderer::sG;
f32 DebugFontRenderer::sB;
f32 DebugFontRenderer::sA;

DebugFontRenderer::DebugFontRenderer()
{
	DebugFont::spRenderer = this;
}

DebugFontRenderer::~DebugFontRenderer()
{

}
