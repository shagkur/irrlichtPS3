/*
 * blit.h
 *
 *  Created on: Feb 21, 2013
 *      Author: mike
 */

#ifndef BLIT_H_
#define BLIT_H_

#include "irrtypes.h"
#include "matrix4.h"

namespace irr
{
	struct AbsRectangle
	{
		s32 x0;
		s32 y0;
		s32 x1;
		s32 y1;
	};

	struct SBlitJob
	{
		AbsRectangle dest;
		AbsRectangle source;

		u32 argb;

		void *src;
		void *dst;

		s32 width;
		s32 height;

		u32 srcPitch;
		u32 dstPitch;

		u32 srcPixelMul;
		u32 dstPixelMul;

		bool stretch;
		f32 x_stretch;
		f32 y_stretch;

		SBlitJob() : stretch(false) {}
	};

	enum eBlitter
	{
		BLITTER_INVALID = 0,
		BLITTER_COLOR,
		BLITTER_COLOR_ALPHA,
		BLITTER_TEXTURE,
		BLITTER_TEXTURE_ALPHA_BLEND,
		BLITTER_TEXTURE_ALPHA_COLOR_BLEND
	};

	typedef void (*tExecuteBlit)(const SBlitJob *job);

	struct blitterTable
	{
		eBlitter operation;
		s32 destFormat;
		s32 sourceFormat;
		tExecuteBlit func;
	};

	inline bool intersect(AbsRectangle& dest, const AbsRectangle& a, const AbsRectangle& b)
	{
		dest.x0 = core::s32_max(a.x0, b.x0);
		dest.y0 = core::s32_max(a.y0, b.y0);
		dest.x1 = core::s32_max(a.x1, b.x1);
		dest.y1 = core::s32_max(a.y1, b.y1);
		return dest.x0 < dest.x1 && dest.y0 < dest.y1;
	}

	inline u32 pixelLerp32(const u32 source, const u32 value)
	{
		u32 srcRB = source&0x00ff00ff;
		u32 srcXG = (source&0xff00ff00)>>8;

		srcRB *= value;
		srcXG *= value;

		srcRB >>= 8;

		srcXG &= 0xff00ff00;
		srcRB &= 0x00ff00ff;

		return srcRB | srcXG;
	}

	static inline u32 extractAlpha(const u32 c)
	{
		return (c>>24) + (c>>31);
	}

	static inline u32 packAlpha(const u32 c)
	{
		return (c > 127 ? c - 1 : c)<<24;
	}

	static void executeBlit_TextureCopy_x_to_x(const SBlitJob *job)
	{
		const u32 w = job->width;
		const u32 h = job->height;

		if(job->stretch) {
			const u32 *src = static_cast<const u32*>(job->src);
			u32 *dst = static_cast<u32*>(job->dst);
			const f32 wscale = 1.0f/job->x_stretch;
			const f32 hscale = 1.0f/job->y_stretch;

			for(u32 dy=0;dy < h;dy++) {
				const u32 src_y = (u32)(dy*hscale);

				src = (u32*)((u8*)(job->src) + job->srcPitch*src_y);
				for(u32 dx=0;dx < w;dx++) {
					const u32 src_x = (u32)(dx*wscale);
					dst[dx] = src[src_x];
				}

				dst = (u32*)((u8*)(dst) + job->dstPitch);
			}
		} else {
			const u32 widthPitch = job->width*job->dstPixelMul;
			const void *src = (void*)job->src;
			void *dst = (void*)job->dst;

			for(u32 dy=0;dy < h;dy++) {
				memcpy(dst, src, widthPitch);

				src = (void*)((u8*)(src) + job->srcPitch);
				dst = (void*)((u8*)(dst) + job->dstPitch);
			}
		}
	}

	static void executeBlit_TextureCopy_32_to_16(const SBlitJob *job)
	{
		const u32 w = job->width;
		const u32 h = job->height;
		const u32 *src = static_cast<const u32*>(job->src);
		u16 *dst = static_cast<u16*>(job->dst);

		if(job->stretch) {
			const f32 wscale = 1.0f/job->x_stretch;
			const f32 hscale = 1.0f/job->y_stretch;

			for(u32 dy=0;dy < h;dy++) {
				const u32 src_y = (u32)(dy*hscale);

				src = (u32*)((u8*)(job->src) + job->srcPitch*src_y);
				for(u32 dx=0;dx < w;dx++) {
					const u32 src_x = (u32)(dx*wscale);
					const u32 s = pixelLerp32(src[src_x]|0xff000000, extractAlpha(src[src_x]));

					dst[dx] = video::A8R8G8B8toA1R5G5B5(s);
				}

				dst = (u16*)((u8*)(dst) + job->dstPitch);
			}
		} else {
			for(u32 dy=0;dy < h;dy++) {
				for(u32 dx=0;dx < w;dx++) {
					const u32 s = pixelLerp32(src[dx]|0xff000000, extractAlpha(src[dx]));

					dst[dx] = video::A8R8G8B8toA1R5G5B5(s);
				}

				src = (u32*)((u8*)(src) + job->srcPitch);
				dst = (u16*)((u8*)(dst) + job->dstPitch);
			}
		}
	}

	static void executeBlit_TextureCopy_24_to_16(const SBlitJob *job)
	{
		const u32 w = job->width;
		const u32 h = job->height;
		const u8 *src = static_cast<const u8*>(job->src);
		u16 *dst = static_cast<u16*>(job->dst);

		if(job->stretch) {
			const f32 wscale = 3.0f/job->x_stretch;
			const f32 hscale = 1.0f/job->y_stretch;

			for(u32 dy=0;dy < h;++dy) {
				const u32 src_y = (u32)(dy*hscale);

				src = (u8*)(job->src) + job->srcPitch*src_y;
				for(u32 dx=0;dx < w;++dx) {
					const u8 *src_x = src + (u32)(dx*wscale);
					dst[dx] = video::RGBA16(src_x[0], src_x[1], src_x[2]);
				}
				dst = (u16*)((u8*)(dst) + job->dstPitch);
			}
		} else {
			for(u32 dy=0;dy < h;++dy) {
				const u8 *s = src;

				for(u32 dx=0;dx < w;++dx) {
					dst[dx] = video::RGBA16(s[0], s[1], s[2]);
					s += 3;
				}
				src = src + job->srcPitch;
				dst = (u16*)((u8*)(dst) + job->dstPitch);
			}
		}
	}

	static void executeBlit_TextureCopy_16_to_32(const SBlitJob *job)
	{
		const u32 w = job->width;
		const u32 h = job->height;
		const u16 *src = static_cast<const u16*>(job->src);
		u32 *dst = static_cast<u32*>(job->dst);

		if(job->stretch) {
			const f32 wscale = 1.0f/job->x_stretch;
			const f32 hscale = 1.0f/job->y_stretch;

			for(u32 dy=0;dy < h;++dy) {
				const u32 src_y = (u32)(dy*hscale);

				src = (u16*)((u8*)(job->src) + job->srcPitch*src_y);
				for(u32 dx=0;dx < w;++dx) {
					const u32 src_x = (u32)(dx*wscale);
					dst[dx] = video::A1R5G5B5toA8R8G8B8(src[src_x]);
				}
				dst = (u32*)((u8*)(dst) + job->dstPitch);
			}
		} else {
			for(u32 dy=0;dy < h;++dy) {
				for(u32 dx=0;dx < w;++dx) {
					dst[dx] = video::A1R5G5B5toA8R8G8B8(src[dx]);
				}
				src = (u16*)((u8*)(src) + job->srcPitch);
				dst = (u32*)((u8*)(dst) + job->dstPitch);
			}
		}
	}

	static void executeBlit_TextureCopy_16_to_24(const SBlitJob *job)
	{
		const u32 w = job->width;
		const u32 h = job->height;
		const u16 *src = static_cast<const u16*>(job->src);
		u8 *dst = static_cast<u8*>(job->dst);

		if(job->stretch) {
			const f32 wscale = 1.0f/job->x_stretch;
			const f32 hscale = 1.0f/job->y_stretch;

			for(u32 dy=0;dy < h;++dy) {
				const u32 src_y = (u32)(dy*hscale);

				src = (u16*)((u8*)(job->src) + job->srcPitch*src_y);
				for(u32 dx=0;dx < w;++dx) {
					const u32 src_x = (u32)(dx*wscale);
					u32 color = video::A1R5G5B5toA8R8G8B8(src[src_x]);
					u8 *writeTo = &dst[dx*3];

					*writeTo++ = (color>>16)&0xff;
					*writeTo++ = (color>>8)&0xff;
					*writeTo++ = color&0xff;
				}
				dst += job->dstPitch;
			}
		} else {
			for(u32 dy=0;dy < h;++dy) {
				for(u32 dx=0;dx < w;++dx) {
					u32 color = video::A1R5G5B5toA8R8G8B8(src[dx]);
					u8 *writeTo = &dst[dx*3];

					*writeTo++ = (color>>16)&0xff;
					*writeTo++ = (color>>8)&0xff;
					*writeTo++ = color&0xff;
				}
				src = (u16*)((u8*)(src) + job->srcPitch);
				dst += job->dstPitch;
			}
		}
	}

	static const blitterTable blitTable[] = {
		{ BLITTER_TEXTURE, -2, -2, executeBlit_TextureCopy_x_to_x },
		{ BLITTER_TEXTURE, video::ECF_A1R5G5B5, video::ECF_A8R8G8B8, executeBlit_TextureCopy_32_to_16 },
		{ BLITTER_TEXTURE, video::ECF_A1R5G5B5, video::ECF_R8G8B8, executeBlit_TextureCopy_24_to_16 },
		{ BLITTER_TEXTURE, video::ECF_A8R8G8B8, video::ECF_A1R5G5B5, executeBlit_TextureCopy_16_to_32 },
		{ BLITTER_TEXTURE, video::ECF_R8G8B8, video::ECF_A1R5G5B5, executeBlit_TextureCopy_16_to_24 },
		{ BLITTER_INVALID, -1, -1, NULL }
	};

	static inline tExecuteBlit getBlitter(eBlitter operation, const video::IImage *dest, const video::IImage *source)
	{
		video::ECOLOR_FORMAT sourceFormat = (video::ECOLOR_FORMAT)(source != NULL ? source->getColorFormat() : -1);
		video::ECOLOR_FORMAT destFormat = (video::ECOLOR_FORMAT)(dest != NULL ? dest->getColorFormat() : -1);
		const blitterTable *b = blitTable;

		while(b->operation != BLITTER_INVALID) {
			if(b->operation == operation) {
				if((b->destFormat == -1 || b->destFormat == destFormat) &&
				   (b->sourceFormat == -1 || b->sourceFormat == sourceFormat))
				{
					return b->func;
				}
				else if(b->destFormat == -2 && sourceFormat == destFormat)
					return b->func;
			}
			b++;
		}

		return NULL;
	}

	inline void setClip(AbsRectangle& out, const core::rect<s32> *clip, const video::IImage *tex, s32 passnative)
	{
		if(clip != NULL && tex == NULL && passnative) {
			out.x0 = clip->upperLeftCorner.X;
			out.x1 = clip->lowerRightCorner.X;
			out.y0 = clip->upperLeftCorner.Y;
			out.y1 = clip->lowerRightCorner.Y;
			return;
		}

		const s32 w = tex != NULL ? tex->getDimension().width : 0;
		const s32 h = tex != NULL ? tex->getDimension().height : 0;
		if(clip != NULL) {
			out.x0 = core::s32_clamp(clip->upperLeftCorner.X, 0, w);
			out.x1 = core::s32_clamp(clip->lowerRightCorner.X, out.x0, w);
			out.y0 = core::s32_clamp(clip->upperLeftCorner.Y, 0, h);
			out.y1 = core::s32_clamp(clip->lowerRightCorner.Y, out.y0, h);
		} else {
			out.x0 = 0;
			out.x1 = 0;
			out.y0 = 0;
			out.y1 = 0;
		}
	}

	static s32 Blit(eBlitter operation, video::IImage *dst, const core::rect<s32> *dstClipping, const core::position2d<s32> *dstPos, video::IImage* const src, const core::rect<s32> *srcClipping, u32 argb)
	{
		SBlitJob job;
		AbsRectangle v;
		AbsRectangle destClip;
		AbsRectangle sourceClip;
		tExecuteBlit blitter = getBlitter(operation, dst, src);

		setClip(sourceClip, srcClipping, src, 1);
		setClip(destClip, dstClipping, dst, 0);

		v.x0 = dstPos != NULL ? dstPos->X : 0;
		v.y0 = dstPos != NULL ? dstPos->Y : 0;
		v.x1 = v.x0 + (sourceClip.x1 - sourceClip.x0);
		v.y1 = v.y0 + (sourceClip.y1 - sourceClip.y0);

		if(!intersect(job.dest, destClip, v))
			return 0;

		job.width = job.dest.x1 - job.dest.x0;
		job.height = job.dest.y1 - job.dest.y0;

		job.source.x0 = sourceClip.x0 + (job.dest.x0 - v.x0);
		job.source.x1 = job.source.x0 + job.width;
		job.source.y0 = sourceClip.y0 + (job.dest.y0 - v.y0);
		job.source.y1 = job.source.y0 + job.height;

		job.argb = argb;

		if(src != NULL) {
			job.srcPitch = src->getPitch();
			job.srcPixelMul = src->getBytesPerPixel();
			job.src = (void*)((u8*)src->lock() + (job.source.y0*job.srcPitch) + (job.source.x0*job.srcPixelMul));
		} else {
			job.srcPitch = job.width*dst->getBytesPerPixel();
		}

		job.dstPitch = dst->getPitch();
		job.dstPixelMul = dst->getBytesPerPixel();
		job.dst = (void*)((u8*)dst->lock() + (job.dest.y0*job.dstPitch) + (job.dest.x0*job.dstPixelMul));

		blitter(&job);

		if(src != NULL) src->unlock();
		if(dst != NULL) dst->unlock();

		return 1;
	}
}


#endif /* BLIT_H_ */
