/*
 * gcmdebugfontrenderer.cpp
 *
 *  Created on: Jun 17, 2013
 *      Author: mike
 */

#include "gcmdebugfontrenderer.h"

#include "iheapmanager.h"

#include "vpshader_dbgfont_vpo.h"
#include "fpshader_dbgfont_fpo.h"

u8* GCMDebugFontRenderer::spTextureData;

gcmContextData* GCMDebugFontRenderer::mContext = NULL;

u8* GCMDebugFontRenderer::mpTexture = NULL;
u8* GCMDebugFontRenderer::mPosition = NULL;
u8* GCMDebugFontRenderer::mTexCoord = NULL;
u8* GCMDebugFontRenderer::mColor = NULL;

rsxProgramAttrib* GCMDebugFontRenderer::mPosIndex = NULL;
rsxProgramAttrib* GCMDebugFontRenderer::mTexIndex = NULL;
rsxProgramAttrib* GCMDebugFontRenderer::mColIndex = NULL;
rsxProgramAttrib* GCMDebugFontRenderer::mTexUnit = NULL;

rsxVertexProgram* GCMDebugFontRenderer::mRSXVertexProgram;
rsxFragmentProgram* GCMDebugFontRenderer::mRSXFragmentProgram;

void* GCMDebugFontRenderer::mVertexProgramUCode;
void* GCMDebugFontRenderer::mFragmentProgramUCode;

vu32* GCMDebugFontRenderer::mLabel = NULL;
u32 GCMDebugFontRenderer::mLabelValue = 0;

u32 GCMDebugFontRenderer::mFragmentProgramOffset;
u32 GCMDebugFontRenderer::mTextureOffset;
u32 GCMDebugFontRenderer::mPositionOffset;
u32 GCMDebugFontRenderer::mTexCoordOffset;
u32 GCMDebugFontRenderer::mColorOffset;

using namespace irr;

GCMDebugFontRenderer::GCMDebugFontRenderer() : DebugFontRenderer()
{

}

GCMDebugFontRenderer::GCMDebugFontRenderer(gcmContextData *context) : DebugFontRenderer()
{
	mContext = context;
}

GCMDebugFontRenderer::~GCMDebugFontRenderer()
{

}

void GCMDebugFontRenderer::initShader()
{
	mRSXVertexProgram = (rsxVertexProgram*)vpshader_dbgfont_vpo;
	mRSXFragmentProgram = (rsxFragmentProgram*)fpshader_dbgfont_fpo;

	void *ucode;
	u32 ucodeSize;

	rsxFragmentProgramGetUCode(mRSXFragmentProgram, &ucode, &ucodeSize);

	mFragmentProgramUCode = rsxMemalign(64, ucodeSize);
	rsxAddressToOffset(mFragmentProgramUCode, &mFragmentProgramOffset);

	memcpy(mFragmentProgramUCode, ucode, ucodeSize);

	rsxVertexProgramGetUCode(mRSXVertexProgram, &mVertexProgramUCode, &ucodeSize);
}

void GCMDebugFontRenderer::init()
{
	mLabel = gcmGetLabelAddress(sLabelId);
	*mLabel = mLabelValue;

	initShader();

	mPosIndex = rsxVertexProgramGetAttrib(mRSXVertexProgram, "position");
	mTexIndex = rsxVertexProgramGetAttrib(mRSXVertexProgram, "texcoord");
	mColIndex = rsxVertexProgramGetAttrib(mRSXVertexProgram, "color");

	mTexUnit = rsxFragmentProgramGetAttrib(mRSXFragmentProgram, "texture");

	spTextureData = (u8*)rsxMemalign(128, DEBUGFONT_DATA_SIZE);
	mpTexture = (u8*)(((u64)spTextureData + 127)&~127);

	u8 *pFontData = (u8*)getFontData();

	for(s32 i=0;i < DEBUGFONT_DATA_SIZE;i++)
		mpTexture[i] = pFontData[i];

	rsxAddressToOffset(mpTexture, &mTextureOffset);

	mPosition = (u8*)IHeapManager::allocate(128, DEBUGFONT_MAX_CHAR_COUNT*NUM_VERTS_PER_GLYPH*sizeof(f32)*3);
	mTexCoord = (u8*)IHeapManager::allocate(128, DEBUGFONT_MAX_CHAR_COUNT*NUM_VERTS_PER_GLYPH*sizeof(f32)*2);
	mColor = (u8*)IHeapManager::allocate(128, DEBUGFONT_MAX_CHAR_COUNT*NUM_VERTS_PER_GLYPH*sizeof(f32)*4);

	rsxAddressToOffset(mPosition, &mPositionOffset);
	rsxAddressToOffset(mTexCoord, &mTexCoordOffset);
	rsxAddressToOffset(mColor, &mColorOffset);
}

void GCMDebugFontRenderer::shutdown()
{

}

void GCMDebugFontRenderer::printStart(f32 r, f32 g, f32 b, f32 a)
{
	sR = r;
	sG = g;
	sB = b;
	sA = a;

	rsxSetFrontFace(mContext, GCM_FRONTFACE_CCW);
	rsxSetCullFaceEnable(mContext, GCM_FALSE);

	rsxSetBlendFunc(mContext, GCM_SRC_ALPHA, GCM_ONE_MINUS_SRC_ALPHA, GCM_SRC_ALPHA, GCM_ONE_MINUS_SRC_ALPHA);
	rsxSetBlendEquation(mContext, GCM_FUNC_ADD, GCM_FUNC_ADD);
	rsxSetBlendEnable(mContext, GCM_TRUE);
	rsxSetLogicOpEnable(mContext, GCM_FALSE);

	rsxSetDepthTestEnable(mContext, GCM_FALSE);

	rsxLoadVertexProgram(mContext, mRSXVertexProgram, mVertexProgramUCode);
	rsxLoadFragmentProgramLocation(mContext, mRSXFragmentProgram, mFragmentProgramOffset, GCM_LOCATION_RSX);

	gcmTexture tex;
	tex.format = GCM_TEXTURE_FORMAT_B8|GCM_TEXTURE_FORMAT_LIN;
	tex.mipmap = 1;
	tex.dimension = GCM_TEXTURE_DIMS_2D;
	tex.cubemap = GCM_FALSE;
	tex.remap = GCM_TEXTURE_REMAP_TYPE_REMAP<<GCM_TEXTURE_REMAP_TYPE_B_SHIFT |
				GCM_TEXTURE_REMAP_TYPE_REMAP<<GCM_TEXTURE_REMAP_TYPE_G_SHIFT |
				GCM_TEXTURE_REMAP_TYPE_REMAP<<GCM_TEXTURE_REMAP_TYPE_R_SHIFT |
				GCM_TEXTURE_REMAP_TYPE_REMAP<<GCM_TEXTURE_REMAP_TYPE_A_SHIFT |
				GCM_TEXTURE_REMAP_COLOR_B<<GCM_TEXTURE_REMAP_COLOR_B_SHIFT |
				GCM_TEXTURE_REMAP_COLOR_B<<GCM_TEXTURE_REMAP_COLOR_G_SHIFT |
				GCM_TEXTURE_REMAP_COLOR_B<<GCM_TEXTURE_REMAP_COLOR_R_SHIFT |
				GCM_TEXTURE_REMAP_COLOR_B<<GCM_TEXTURE_REMAP_COLOR_A_SHIFT;
	tex.width = DEBUGFONT_TEXTURE_WIDTH;
	tex.height = DEBUGFONT_TEXTURE_HEIGHT;
	tex.depth = 1;
	tex.pitch = DEBUGFONT_TEXTURE_WIDTH;
	tex.location = GCM_LOCATION_RSX;
	tex.offset = mTextureOffset;
	rsxLoadTexture(mContext, mTexUnit->index, &tex);

	rsxTextureControl(mContext, mTexUnit->index, GCM_TRUE, 0<<8, 12<<8, GCM_TEXTURE_MAX_ANISO_1);
	rsxTextureFilter(mContext, mTexUnit->index, 0, GCM_TEXTURE_NEAREST_MIPMAP_LINEAR, GCM_TEXTURE_LINEAR, GCM_TEXTURE_CONVOLUTION_QUINCUNX);
	rsxTextureWrapMode(mContext, mTexUnit->index, GCM_TEXTURE_REPEAT, GCM_TEXTURE_REPEAT, GCM_TEXTURE_REPEAT, GCM_TEXTURE_UNSIGNED_REMAP_NORMAL, GCM_TEXTURE_ZFUNC_LESS, 0);
}

void GCMDebugFontRenderer::printPass(DebugFont::Position *pPositions, DebugFont::TexCoord *pTexCoords, DebugFont::Color *pColors, s32 numVerts)
{
	while(*mLabel != mLabelValue)
		usleep(10);

	mLabelValue++;

	memcpy(mPosition, pPositions, numVerts*sizeof(f32)*3);
	memcpy(mTexCoord, pTexCoords, numVerts*sizeof(f32)*2);
	memcpy(mColor, pColors, numVerts*sizeof(f32)*4);

	rsxBindVertexArrayAttrib(mContext, mPosIndex->index, 0, mPositionOffset, sizeof(f32)*3, 3, GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_CELL);
	rsxBindVertexArrayAttrib(mContext, mTexIndex->index, 0, mTexCoordOffset, sizeof(f32)*2, 2, GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_CELL);
	rsxBindVertexArrayAttrib(mContext, mColIndex->index, 0, mColorOffset, sizeof(f32)*4, 4, GCM_VERTEX_DATA_TYPE_F32, GCM_LOCATION_CELL);

	rsxDrawVertexArray(mContext, DEBUGFONT_PRIMITIVE, 0, numVerts);
	rsxInvalidateVertexCache(mContext);
	rsxSetWriteBackendLabel(mContext, sLabelId, mLabelValue);

	rsxFlushBuffer(mContext);
}

void GCMDebugFontRenderer::printEnd()
{
	rsxSetDepthTestEnable(mContext, GCM_TRUE);
	rsxSetBlendEnable(mContext, GCM_FALSE);
	rsxSetCullFaceEnable(mContext, GCM_TRUE);
	rsxSetFrontFace(mContext, GCM_FRONTFACE_CW);
}
