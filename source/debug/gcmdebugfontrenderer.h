/*
 * gcmdebugfontrenderer.h
 *
 *  Created on: Jun 17, 2013
 *      Author: mike
 */

#ifndef GCMDEBUGFONTRENDERER_H_
#define GCMDEBUGFONTRENDERER_H_

#include "debug/debugfontrenderer.h"

#ifdef DEBUGFONT_USE_QUADS
#define DEBUGFONT_PRIMITIVE					GCM_TYPE_QUADS
#define NUM_VERTS_PER_GLYPH					4
#else
#define DEBUGFONT_PRIMITIVE					GCM_TYPE_TRIANGLES
#define NUM_VERTS_PER_GLYPH					6
#endif

class GCMDebugFontRenderer : public DebugFontRenderer
{
public:
	GCMDebugFontRenderer();
	GCMDebugFontRenderer(gcmContextData *context);
	virtual ~GCMDebugFontRenderer();

	virtual void init();
	virtual void shutdown();
	virtual void printStart(f32 r, f32 g, f32 b, f32 a);
	virtual void printPass(DebugFont::Position *pPositions, DebugFont::TexCoord *pTexCoords, DebugFont::Color *pColors, s32 numVerts);
	virtual void printEnd();

private:
	void initShader();

	static gcmContextData *mContext;

	static u8 *spTextureData;

	static u8 *mPosition;
	static u8 *mTexCoord;
	static u8 *mColor;

	static u32 mPositionOffset;
	static u32 mTexCoordOffset;
	static u32 mColorOffset;

	static u8 *mpTexture;
	static rsxProgramAttrib *mPosIndex;
	static rsxProgramAttrib *mTexIndex;
	static rsxProgramAttrib *mColIndex;
	static rsxProgramAttrib *mTexUnit;

	static rsxVertexProgram *mRSXVertexProgram;
	static rsxFragmentProgram *mRSXFragmentProgram;

	static u32 mFragmentProgramOffset;
	static u32 mTextureOffset;

	static void *mVertexProgramUCode;				// this is sysmem
	static void *mFragmentProgramUCode;			// this is vidmem

	static vu32 *mLabel;
	static u32 mLabelValue;

	static const u32 sLabelId = 254;
};

#endif /* GCMDEBUGFONTRENDERER_H_ */
