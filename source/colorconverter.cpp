/*
 * colorconverter.cpp
 *
 *  Created on: Feb 22, 2013
 *      Author: mike
 */

#include "colorconverter.h"
#include "scolor.h"
#include "os.h"

namespace irr
{
	namespace video
	{
		void CColorConverter::convert1BitTo16Bit(const u8 *in, s16 *out, s32 width, s32 height, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL) return;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				s32 shift = 7;

				if(flip) out -= width;

				for(s32 x=0;x < width;x++) {
					out[x] = *in>>shift&0x01 ? (s16)0xffff : (s16)0x8000;

					if((--shift) < 0) {
						shift = 7;
						++in;
					}
				}

				if(shift != 7) ++in;
				if(!flip) out += width;

				in += linepad;
			}
		}

		void CColorConverter::convert1BitTo32Bit(const u8 *in, s32 *out, s32 width, s32 height, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL) return;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				s32 shift = 7;

				if(flip) out -= width;

				for(s32 x=0;x < width;x++) {
					out[x] = *in>>shift&0x01 ? (s32)0xffffffff : (s32)0xff000000;

					if((--shift) < 0) {
						shift = 7;
						++in;
					}
				}

				if(shift != 7) ++in;
				if(!flip) out += width;

				in += linepad;
			}
		}

		void CColorConverter::convert4BitTo16Bit(const u8 *in, s16 *out, s32 width, s32 height, const s32 *palette, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL || palette == NULL) return;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				s32 shift = 4;

				if(flip) out -= width;

				for(s32 x=0;x < width;x++) {
					out[x] = X8R8G8B8toA1R5G5B5(palette[(u8)((*in>>shift)&0x0f)]);

					if(shift == 0) {
						shift = 4;
						++in;
					} else
						shift = 0;
				}

				if(shift == 0) ++in;
				if(!flip) out += width;

				in += linepad;
			}
		}

		void CColorConverter::convert4BitTo32Bit(const u8 *in, s32 *out, s32 width, s32 height, const s32 *palette, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL || palette == NULL) return;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				s32 shift = 4;

				if(flip) out -= width;

				for(s32 x=0;x < width;x++) {
					out[x] = palette[(u8)((*in>>shift)&0x0f)];

					if(shift == 0) {
						shift = 4;
						++in;
					} else
						shift = 0;
				}

				if(shift == 0) ++in;
				if(!flip) out += width;

				in += linepad;
			}
		}

		void CColorConverter::convert8BitTo16Bit(const u8 *in, s16 *out, s32 width, s32 height, const s32 *palette, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL || palette == NULL) return;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				if(flip) out -= width;

				for(s32 x=0;x < width;x++) {
					out[x] = X8R8G8B8toA1R5G5B5(palette[(u8)(*in)]);
					++in;
				}
				if(!flip) out += width;

				in += linepad;
			}
		}

		void CColorConverter::convert8BitTo24Bit(const u8 *in, u8 *out, s32 width, s32 height, const u8 *palette, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL) return;

			const s32 lineWidth = 3*width;

			if(flip) out += lineWidth*height;

			for(s32 y=0;y < height;y++) {
				if(flip) out -= lineWidth;

				for(s32 x=0;x < lineWidth;x+=3) {
					if(palette != NULL) {
						out[x + 0] = palette[(in[0]<<2) + 0];
						out[x + 1] = palette[(in[0]<<2) + 1];
						out[x + 2] = palette[(in[0]<<2) + 2];
					} else {
						out[x + 0] = in[0];
						out[x + 1] = in[0];
						out[x + 2] = in[0];
					}
					++in;
				}
				if(!flip) out += lineWidth;

				in += linepad;
			}
		}

		void CColorConverter::convert8BitTo32Bit(const u8 *in, u8 *out, s32 width, s32 height, const u8 *palette, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL) return;

			const u32 lineWidth = 4*width;

			if(flip) out += lineWidth*height;

			u32 x;
			register u32 c;
			for(s32 y=0;y < height;y++) {
				if(flip) out -= lineWidth;

				if(palette != NULL) {
					for(x=0;x < (u32)width;x++) {
						c = in[x];
						((u32*)out)[x] = ((u32*)palette)[c];
					}
				} else {
					for(x=0;x < (u32)width;x++) {
						c = in[x];
						((u32*)out)[x] = c<<24 | c<<16 | c<<8 | 0x000000ff;
					}
				}
				if(!flip) out += lineWidth;

				in += width + linepad;
			}
		}

		void CColorConverter::convert16BitTo16Bit(const s16 *in, s16 *out, s32 width, s32 height, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL) return;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				if(flip) out -= width;

				for(s32 x=0;x < width;x++)
					out[x] = os::ByteSwap::byteswap(in[x]);

				if(!flip) out += width;

				in += width;
				in += linepad;
			}
		}

		void CColorConverter::convert16BitTo32Bit(const s16 *in, s32 *out, s32 width, s32 height, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL) return;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				if(flip) out -= width;

				for(s32 x=0;x < width;x++)
					out[x] = A1R5G5B5toA8R8G8B8(os::ByteSwap::byteswap(in[x]));

				if(!flip) out += width;

				in += width;
				in += linepad;
			}
		}

		void CColorConverter::convert24BitTo24Bit(const u8 *in, u8 *out, s32 width, s32 height, s32 linepad, bool flip, bool bgr)
		{
			if(in == NULL || out == NULL) return;

			const s32 lineWidth = width*3;

			if(flip) out += lineWidth*height;

			for(s32 y=0;y < height;y++) {
				if(flip) out -= lineWidth;

				if(bgr) {
					for(s32 x=0;x < lineWidth;x++) {
						out[x + 0] = in[x + 2];
						out[x + 1] = in[x + 1];
						out[x + 2] = in[x + 0];
					}
				} else {
					memcpy(out, in, lineWidth);
				}

				if(!flip) out += lineWidth;

				in += lineWidth;
				in += linepad;
			}
		}

		void CColorConverter::convert24BitTo32Bit(const u8 *in, s32 *out, s32 width, s32 height, s32 linepad, bool flip, bool bgr)
		{
			if(in == NULL || out == NULL) return;

			const s32 lineWidth = width*3;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				if(flip) out -= width;

				if(bgr)
					convert_B8G8R8toA8R8G8B8(in, width, out);
				else
					convert_R8G8B8toA8R8G8B8(in, width, out);

				if(!flip) out += width;

				in += lineWidth;
				in += linepad;
			}
		}

		void CColorConverter::convert32BitTo32Bit(const s32 *in, s32 *out, s32 width, s32 height, s32 linepad, bool flip)
		{
			if(in == NULL || out == NULL) return;

			if(flip) out += width*height;

			for(s32 y=0;y < height;y++) {
				if(flip) out -= width;

				for(s32 x=0;x < width;x++)
					out[x] = os::ByteSwap::byteswap(in[x]);

				if(!flip) out += width;

				in += width;
				in += linepad;
			}
		}

		void CColorConverter::convert_A1R5G5B5toR8G8B8(const void *sP, s32 sN, void *dP)
		{
			u16 *sB = (u16*)sP;
			u8 *dB = (u8*)dP;

			for(s32 x=0;x < sN;x++) {
				dB[2] = (*sB&0x7c00)>>7;
				dB[1] = (*sB&0x03e0)>>2;
				dB[0] = (*sB&0x001f)<<3;

				sB += 1;
				dB += 3;
			}
		}

		void CColorConverter::convert_A1R5G5B5toB8G8R8(const void *sP, s32 sN, void *dP)
		{
			u16 *sB = (u16*)sP;
			u8 *dB = (u8*)dP;

			for(s32 x=0;x < sN;x++) {
				dB[0] = (*sB&0x7c00)>>7;
				dB[1] = (*sB&0x03e0)>>2;
				dB[2] = (*sB&0x001f)<<3;

				sB += 1;
				dB += 3;
			}
		}

		void CColorConverter::convert_A1R5G5B5toA8R8G8B8(const void *sP, s32 sN, void *dP)
		{
			u16 *sB = (u16*)sP;
			u32 *dB = (u32*)dP;

			for(s32 x=0;x < sN;x++)
				*dB++ = A1R5G5B5toA8R8G8B8(*sB++);
		}

		void CColorConverter::convert_A1R5G5B5toA1R5G5B5(const void *sP, s32 sN, void *dP)
		{
			memcpy(dP, sP, sN*2);
		}

		void CColorConverter::convert_A1R5G5B5toR5G6B5(const void *sP, s32 sN, void *dP)
		{
			u16 *sB = (u16*)sP;
			u16 *dB = (u16*)dP;

			for(s32 x=0;x < sN;x++)
				*dB++ = A1R5G5B5toR5G6B5(*sB++);
		}

		void CColorConverter::convert_A8R8G8B8toR8G8B8(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u8 *dB = (u8*)dP;

			for(s32 x=0;x < sN;x++) {
				dB[0] = sB[2];
				dB[1] = sB[1];
				dB[2] = sB[0];

				sB += 4;
				dB += 3;
			}
		}

		void CColorConverter::convert_A8R8G8B8toB8G8R8(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u8 *dB = (u8*)dP;

			for(s32 x=0;x < sN;x++) {
				dB[0] = sB[0];
				dB[1] = sB[1];
				dB[2] = sB[2];

				sB += 4;
				dB += 3;
			}
		}

		void CColorConverter::convert_A8R8G8B8toA8R8G8B8(const void *sP, s32 sN, void *dP)
		{
			memcpy(dP, sP, sN*4);
		}

		void CColorConverter::convert_A8R8G8B8toA1R5G5B5(const void *sP, s32 sN, void *dP)
		{
			u32 *sB = (u32*)sP;
			u16 *dB = (u16*)dP;

			for(s32 x=0;x < sN;x++)
				*dB++ = A8R8G8B8toA1R5G5B5(*sB++);
		}

		void CColorConverter::convert_A8R8G8B8toR3G3B2(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u8 *dB = (u8*)dP;

			for(s32 x=0;x < sN;x++) {
					u8 r = (sB[2]&0xe0);
					u8 g = (sB[1]&0xe0)>>3;
					u8 b = (sB[2]&0xe0)>>6;

					dB[0] = (r | g | b);

					sB += 4;
					dB += 1;
			}
		}

		void CColorConverter::convert_A8R8G8B8toR5G6B5(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u16 *dB = (u16*)dP;

			for(s32 x=0;x < sN;x++) {
				s32 r = sB[2]>>3;
				s32 g = sB[1]>>2;
				s32 b = sB[0]>>3;

				dB[0] = (r<<11) | (g<<5) | (b);

				sB += 4;
				dB += 1;
			}
		}

		void CColorConverter::convert_R8G8B8toR8G8B8(const void *sP, s32 sN, void *dP)
		{
			memcpy(dP, sP, sN*3);
		}

		void CColorConverter::convert_R8G8B8toA8R8G8B8(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u32 *dB = (u32*)dP;

			for(s32 x=0;x < sN;x++) {
				*dB = 0xff000000 | (sB[0]<<16) | (sB[1]<<8) | (sB[2]);

				sB += 3;
				dB += 1;
			}
		}

		void CColorConverter::convert_R8G8B8toA1R5G5B5(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u16 *dB = (u16*)dP;

			for(s32 x=0;x < sN;x++) {
				s32 r = sB[0]>>3;
				s32 g = sB[1]>>3;
				s32 b = sB[2]>>3;

				dB[0] = (0x8000) | (r<<10) | (g<<5) | (b);

				sB += 3;
				dB += 1;
			}
		}

		void CColorConverter::convert_B8G8R8toA8R8G8B8(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u32 *dB = (u32*)dP;

			for(s32 x=0;x < sN;x++) {
				*dB = 0xff000000 | (sB[2]<<16) | (sB[1]<<8) | (sB[0]);

				sB += 3;
				dB += 1;
			}
		}

		void CColorConverter::convert_B8G8R8A8toA8R8G8B8(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u8 *dB = (u8*)dP;

			for(s32 x=0;x < sN;x++) {
				dB[0] = sB[3];
				dB[1] = sB[2];
				dB[2] = sB[1];
				dB[3] = sB[0];

				sB += 4;
				dB += 4;
			}
		}

		void CColorConverter::convert_R8G8B8toR5G6B5(const void *sP, s32 sN, void *dP)
		{
			u8 *sB = (u8*)sP;
			u16 *dB = (u16*)dP;

			for(s32 x=0;x < sN;x++) {
				s32 r = sB[0]>>3;
				s32 g = sB[1]>>2;
				s32 b = sB[2]>>3;

				dB[0] = (r<<11) | (g<<5) | (b);

				sB += 3;
				dB += 1;
			}
		}

		void CColorConverter::convert_R5G6B5toR5G6B5(const void *sP, s32 sN, void *dP)
		{
			memcpy(dP, sP, sN*2);
		}

		void CColorConverter::convert_R5G6B5toR8G8B8(const void *sP, s32 sN, void *dP)
		{
			u16 *sB = (u16*)sP;
			u8 *dB = (u8*)dP;

			for(s32 x=0;x < sN;x++) {
				dB[0] = (*sB&0xf800)>>8;
				dB[1] = (*sB&0x07e0)>>3;
				dB[2] = (*sB&0x001f)<<3;

				sB += 1;
				dB += 3;
			}
		}

		void CColorConverter::convert_R5G6B5toB8G8R8(const void *sP, s32 sN, void *dP)
		{
			u16 *sB = (u16*)sP;
			u8 *dB = (u8*)dP;

			for(s32 x=0;x < sN;x++) {
				dB[2] = (*sB&0xf800)>>8;
				dB[1] = (*sB&0x07e0)>>3;
				dB[0] = (*sB&0x001f)<<3;

				sB += 1;
				dB += 3;
			}
		}

		void CColorConverter::convert_R5G6B5toA8R8G8B8(const void *sP, s32 sN, void *dP)
		{
			u16 *sB = (u16*)sP;
			u32 *dB = (u32*)dP;

			for(s32 x=0;x < sN;x++)
				*dB++ = R5G6B5toA8R8G8B8(*sB++);
		}

		void CColorConverter::convert_R5G6B5toA1R5G5B5(const void *sP, s32 sN, void *dP)
		{
			u16 *sB = (u16*)sP;
			u16 *dB = (u16*)dP;

			for(s32 x=0;x < sN;x++)
				*dB++ = R5G6B5toA1R5G5B5(*sB++);
		}

		void CColorConverter::convert_viaFormat(const void *sP, ECOLOR_FORMAT sF, s32 sN, void *dP, ECOLOR_FORMAT dF)
		{
			switch(sF) {
				case ECF_A1R5G5B5:
					switch(dF) {
						case ECF_A1R5G5B5:
							convert_A1R5G5B5toA1R5G5B5(sP, sN, dP);
							break;
						case ECF_R5G6B5:
							convert_A1R5G5B5toR5G6B5(sP, sN, dP);
							break;
						case ECF_A8R8G8B8:
							convert_A8R8G8B8toA1R5G5B5(sP, sN, dP);
							break;
						case ECF_R8G8B8:
							convert_R8G8B8toA1R5G5B5(sP, sN, dP);
							break;
						default:
							break;
					}
					break;
				case ECF_R5G6B5:
					switch(dF) {
						case ECF_A1R5G5B5:
							convert_R5G6B5toA1R5G5B5(sP, sN, dP);
							break;
						case ECF_R5G6B5:
							convert_R5G6B5toR5G6B5(sP, sN, dP);
							break;
						case ECF_A8R8G8B8:
							convert_R5G6B5toA8R8G8B8(sP, sN, dP);
							break;
						case ECF_R8G8B8:
							convert_R5G6B5toR8G8B8(sP, sN, dP);
							break;
						default:
							break;
					}
					break;
				case ECF_A8R8G8B8:
					switch(dF) {
						case ECF_A1R5G5B5:
							convert_A8R8G8B8toA1R5G5B5(sP, sN, dP);
							break;
						case ECF_R5G6B5:
							convert_A8R8G8B8toR5G6B5(sP, sN, dP);
							break;
						case ECF_A8R8G8B8:
							convert_A8R8G8B8toA8R8G8B8(sP, sN, dP);
							break;
						case ECF_R8G8B8:
							convert_A8R8G8B8toR8G8B8(sP, sN, dP);
							break;
						default:
							break;
					}
					break;
				case ECF_R8G8B8:
					switch(dF) {
						case ECF_A1R5G5B5:
							convert_R8G8B8toA1R5G5B5(sP, sN, dP);
							break;
						case ECF_R5G6B5:
							convert_R8G8B8toR5G6B5(sP, sN, dP);
							break;
						case ECF_A8R8G8B8:
							convert_R8G8B8toA8R8G8B8(sP, sN, dP);
							break;
						case ECF_R8G8B8:
							convert_R8G8B8toR8G8B8(sP, sN, dP);
							break;
						default:
							break;
					}
					break;
				default:
					break;
			}
		}
	}
}

