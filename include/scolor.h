#ifndef __SCOLOR_H__
#define __SCOLOR_H__

#include "irrtypes.h"
#include "irrmath.h"

namespace irr
{
	namespace video
	{
		enum ECOLOR_FORMAT
		{
				//! 16 bit color format used by the software driver.
				/** It is thus preferred by all other irrlicht engine video drivers.
				There are 5 bits for every color component, and a single bit is left
				for alpha information. */
				ECF_A1R5G5B5 = 0,

				//! Standard 16 bit color format.
				ECF_R5G6B5,

				//! 24 bit color, no alpha channel, but 8 bit for red, green and blue.
				ECF_R8G8B8,

				//! Default 32 bit color format. 8 bits are used for every component: red, green, blue and alpha.
				ECF_A8R8G8B8,

				/** Compressed image formats. **/

				//! DXT1 color format.
				ECF_DXT1,

				//! DXT2 color format.
				ECF_DXT2,

				//! DXT3 color format.
				ECF_DXT3,

				//! DXT4 color format.
				ECF_DXT4,

				//! DXT5 color format.
				ECF_DXT5,

				/** Floating Point formats. The following formats may only be used for render target textures. */

				//! 16 bit floating point format using 16 bits for the red channel.
				ECF_R16F,

				//! 32 bit floating point format using 16 bits for the red channel and 16 bits for the green channel.
				ECF_G16R16F,

				//! 64 bit floating point format 16 bits are used for the red, green, blue and alpha channels.
				ECF_A16B16G16R16F,

				//! 32 bit floating point format using 32 bits for the red channel.
				ECF_R32F,

				//! 64 bit floating point format using 32 bits for the red channel and 32 bits for the green channel.
				ECF_G32R32F,

				//! 128 bit floating point format. 32 bits are used for the red, green, blue and alpha channels.
				ECF_A32B32G32R32F,

				//! Unknown color format:
				ECF_UNKNOWN
		};

		inline u16 RGBA16(u32 r,u32 g,u32 b,u32 a = 0xff)
		{
			return (((a&0x80)<<8)|((r&0xf8)<<7)|((g&0xf8)<<2)|((b&0xf8)>>3));
		}

		inline u16 RGB16(u32 r,u32 g,u32 b)
		{
			return RGBA16(r,g,b);
		}

		inline u16 RGB16from16(u16 r, u16 g, u16 b)
		{
				return (0x8000|(r&0x1F)<<10|(g&0x1F)<<5|(b&0x1F));
		}

		inline u16 X8R8G8B8toA1R5G5B5(u32 color)
		{
			return (0x8000|(color&0x00f80000)>>9|(color&0x0000f800)>>6|(color&0x000000f8)>>3);
		}

		inline u16 A8R8G8B8toA1R5G5B5(u32 color)
		{
			return (( color & 0x80000000) >> 16|
				( color & 0x00F80000) >> 9 |
				( color & 0x0000F800) >> 6 |
				( color & 0x000000F8) >> 3);
		}

        inline u16 A8R8G8B8toR5G6B5(u32 color)
        {
                return (u16)(( color & 0x00F80000) >> 8 |
                        ( color & 0x0000FC00) >> 5 |
                        ( color & 0x000000F8) >> 3);
        }
	
		inline u32 A1R5G5B5toA8R8G8B8(u16 color)
		{
			return ( (( -( (s32) color & 0x00008000 ) >> (s32) 31 ) & 0xFF000000 ) |
					(( color & 0x00007C00 ) << 9) | (( color & 0x00007000 ) << 4) |
					(( color & 0x000003E0 ) << 6) | (( color & 0x00000380 ) << 1) |
					(( color & 0x0000001F ) << 3) | (( color & 0x0000001C ) >> 2)
					);
		}

		inline u32 R5G6B5toA8R8G8B8(u16 color)
		{
			return 0xFF000000|
				((color & 0xF800) << 8)|
				((color & 0x07E0) << 5)|
				((color & 0x001F) << 3);
		}

		//! Returns A1R5G5B5 Color from R5G6B5 color
		inline u16 R5G6B5toA1R5G5B5(u16 color)
		{
			return 0x8000|(((color&0xFFC0)>>1)|(color&0x1F));
		}

		inline u16 A1R5G5B5toR5G6B5(u16 color)
		{
			return (((color&0x7FE0)<<1)|(color&0x1F));
		}

		inline u32 getAlpha(u16 color)
		{
			return ((color>>15)&0x01);
		}

		inline u32 getRed(u16 color)
		{

			return ((color>>10)&0x1f);
		}

		inline u32 getGreen(u16 color)
		{
			return ((color>>5)&0x1f);
		}

		inline u32 getBlue(u16 color)
		{
			return (color&0x1f);
		}

		inline s32 getAverage(s16 color)
		{
			return ((getRed(color)<<3) + (getGreen(color)<<3) + (getBlue(color)<<3))/3;
		}

		class SColor
		{
		public:
			SColor() : color(0) {};
			SColor(u32 c) : color(c) {};
			SColor(u8 r, u8 g, u8 b, u8 a) : color(((r&0xff)<<24) | ((g&0xff)<<16) | ((b&0xff)<<8) | (a&0xff)) {};
			SColor(const SColor& other) : color(other.color) {};

			u8 getRed() const { return (color>>24)&0xff; }
			u8 getGreen() const { return (color>>16)&0xff; }
			u8 getBlue() const { return (color>>8)&0xff; }
			u8 getAlpha() const { return color&0xff; }

			void setRed(u8 c) { color = ((c&0xff)<<24) | (color&0x00ffffff); }
			void setGreen(u8 c) { color = ((c&0xff)<<16) | (color&0xff00ffff); }
			void setBlue(u8 c) { color = ((c&0xff)<<8) | (color&0xffff00ff); }
			void setAlpha(u8 c) { color = (c&0xff) | (color&0xffffff00); }

			u32 toA8R8G8B8()
			{
				return (((color&0x000000ff)<<24) | ((color&0xffffff00)>>8));
			}

			void set(u8 r, u8 g, u8 b, u8 a)
			{
				color = (((r&0xff)<<24) | ((g&0xff)<<16) | ((b&0xff)<<8) | (a&0xff));
			}

			void set(u32 col)
			{
				color = col;
			} 

            f32 getLightness() const
            {
                return 0.5f*(core::max_(core::max_(getRed(), getGreen()),getBlue()) + core::min_(core::min_(getRed(), getGreen()), getBlue()));
            }

			f32 getLuminance() const
			{
				return 0.299f*(f32)getRed() + 0.587f*(f32)getGreen() + 0.114f*(f32)getBlue();
			}

			u32 getAverage() const
			{
				return (getRed() + getGreen() + getBlue())/3;
			}

			SColor getInterpolated(const SColor& other,f32 d)
			{
				d = core::clamp(d, 0.0f, 1.0f);

				const f32 inv = 1.0f - d;
				return SColor((u32)(other.getRed()*inv + getRed()*d),
							  (u32)(other.getGreen()*inv + getGreen()*d),
							  (u32)(other.getBlue()*inv + getBlue()*d),
							  (u32)(other.getAlpha()*inv + getAlpha()*d));
			}

			bool operator!=(const SColor& other) const { return (other.color!=color); }
			bool operator==(const SColor& other) const { return (other.color==color); }

			bool operator<(const SColor& other) const { return (color<other.color); }

			SColor operator+(const SColor& other) const
			{
				return SColor(core::min_(getRed() + other.getRed(),255),
							  core::min_(getGreen() + other.getGreen(),255),
							  core::min_(getBlue() + other.getBlue(),255),
							  core::min_(getAlpha() + other.getAlpha(),255));
			}

			static SColor fromA8R8G8B8(u32 val)
			{
				return SColor(((val&0xff000000)>>24)|((val&0x00ffffff)<<8));
			}

			u32 color;
		};

		class SColorf
		{
		public:
			SColorf(f32 r = 0.0f,f32 g = 0.0f,f32 b = 0.0f,f32 a = 1.0f) : _r(r),_g(g),_b(b),_a(a) {};
			SColorf(const SColor& c)
			{
				const f32 inv = 1.0f/255.f;
				_r = (f32)c.getRed()*inv;
				_g = (f32)c.getGreen()*inv;
				_b = (f32)c.getBlue()*inv;
				_a = (f32)c.getAlpha()*inv;
			}

			SColor toSColor() const
			{
				return SColor((u8)(_r*255.0f), (u8)(_g*255.0f), (u8)(_b*255.0f), (u8)(_a*255.0f));
			}

			void set(f32 r,f32 g,f32 b) { _r = r; _g = g; _b = b; }
			void set(f32 a,f32 r,f32 g,f32 b) { _a = a; _r = r; _g = g; _b = b; }

			f32 getAlpha() const { return _a; }
			f32 getRed() const { return _r; }
			f32 getGreen() const { return _g; }
			f32 getBlue() const { return _b; }

			bool operator==(const SColorf& other) const
			{
				return (_r==other._r && _g==other._g && _b==other._b && _a==other._a);
			}

			bool operator!=(const SColorf& other) const
			{
				return (_r!=other._r || _g!=other._g || _b!=other._b || _a!=other._a);
			}

			bool operator<(const SColorf& other) const
			{
				return (_r<other._r || _g<other._g || _b<other._b || _a<other._a);
			}

			SColorf operator*(const SColorf& other) const
			{
				return SColorf(core::min_(getRed()*other.getRed(),1.0f),
							   core::min_(getGreen()*other.getGreen(),1.0f),
							   core::min_(getBlue()*other.getBlue(),1.0f),
							   core::min_(getAlpha()*other.getAlpha(),1.0f));
			}

			SColorf operator+(const SColorf& other) const
			{
				return SColorf(core::min_(getRed() + other.getRed(),1.0f),
							   core::min_(getGreen() + other.getGreen(),1.0f),
							   core::min_(getBlue() + other.getBlue(),1.0f),
							   core::min_(getAlpha() + other.getAlpha(),1.0f));
			}

			f32 getLuminance() const
			{
				return 0.3f*getRed()+0.59f*getGreen()+0.11f*getBlue();
			}

			SColorf getInterpolated(const SColorf& other,f32 d) const
			{
				d = core::clamp(d,0.0f,1.0f);
				const f32 inv = 1.0f - d;
				return SColorf(other._r*inv + _r*d,
							   other._g*inv + _g*d, 
							   other._b*inv + _b*d, 
							   other._a*inv + _a*d);
			}

			inline SColorf getInterpolated_quadratic(const SColorf& c1,const SColorf& c2,f32 d) const
			{
				d = core::clamp(d,0.0f,1.0f);
				// this*(1-d)*(1-d) + 2 * c1 * (1-d) + c2 * d * d;
				const f32 inv = 1.0f - d;
				const f32 mul0 = inv * inv;
				const f32 mul1 = 2.0f * d * inv;
				const f32 mul2 = d * d;

				return SColorf (_r * mul0 + c1._r * mul1 + c2._r * mul2,
								_g * mul0 + c1._g * mul1 + c2._g * mul2,
								_b * mul0 + c1._b * mul1 + c2._b * mul2,
								_a * mul0 + c1._a * mul1 + c2._a * mul2);
			}

			union
			{
				struct
				{
					f32 _r;
					f32 _g;
					f32 _b;
					f32 _a;
				};
				f32 color[4];
			};
		};
	}
}

#endif
