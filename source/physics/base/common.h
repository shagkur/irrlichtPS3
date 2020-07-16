/*
 * common.h
 *
 *  Created on: Apr 26, 2013
 *      Author: mike
 */

#ifndef COMMON_H_
#define COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <mars/task.h>
#include <vectormath/cpp/vectormath_aos.h>

#ifdef __SPU__

#include <dma/spu_dma.h>
#include <sys/spu_atomic.h>
#include <sys/spu_printf.h>

#define HALT()				spu_hcmpeq(0, 0)

#define PRINTF(...) 		spu_printf(__VA_ARGS__)

#define UNLIKELY(a)			__builtin_expect((a),0)
#define LIKELY(a)			__builtin_expect((a),1)

#define DIVF(a, b)			spu_extract(divf4(spu_splats((a)), spu_splats((b))), 0)
#define SQRTF(a)			spu_extract(sqrtf4(spu_splats((a))), 0)

#define ATTRIBUTE_PTR32(a) a
#define ATTRIBUTE_ALIGNED16_PTR32(a) a __attribute__((aligned(16)))
#define ATTRIBUTE_ALIGNED64_PTR32(a) a __attribute__((aligned(64)))
#define ATTRIBUTE_ALIGNED128_PTR32(a) a __attribute__((aligned(128)))

typedef int8_t				s8;
typedef uint8_t				u8;

typedef int16_t				s16;
typedef uint16_t			u16;

typedef int32_t				s32;
typedef uint32_t			u32;

typedef int64_t				s64;
typedef uint64_t			u64;

typedef float				f32;
typedef double				f64;

static inline f32 MINF(f32 a, f32 b)			{return spu_extract(spu_sel(spu_splats(a), spu_splats(b), (vector unsigned int)spu_cmpgt(spu_splats(a), spu_splats(b))), 0);}
static inline f32 MAXF(f32 a, f32 b)			{return spu_extract(spu_sel(spu_splats(b), spu_splats(a), (vector unsigned int)spu_cmpgt(spu_splats(a), spu_splats(b))), 0);}
static inline f32 CLAMPF(f32 v, f32 a, f32 b)	{return MAXF(a, MINF(v, b));}

#else

#include <ppu-types.h>

#define HALT()				abort()

#define PRINTF(...) 		printf(__VA_ARGS__)

#define UNLIKELY(a)			(a)
#define LIKELY(a)			(a)

#define DIVF(a, b)			(a)/(b)
#define SQRTF(a)			sqrtf(a)

#define ATTRIBUTE_PTR32(a) a __attribute__((mode(SI)))
#define ATTRIBUTE_ALIGNED16_PTR32(a) a __attribute__((aligned(16), mode(SI)))
#define ATTRIBUTE_ALIGNED64_PTR32(a) a __attribute__((aligned(64), mode(SI)))
#define ATTRIBUTE_ALIGNED128_PTR32(a) a __attribute__((aligned(128), mode(SI)))

static inline f32 MINF(f32 a, f32 b)			{return (((a) < (b)) ? (a) : (b));}
static inline f32 MAXF(f32 a, f32 b)			{return (((a) > (b)) ? (a) : (b));}
static inline f32 CLAMPF(f32 v, f32 a, f32 b)	{return MAXF(a, MINF(v, b));}

#endif


#define SIMD_FORCE_INLINE inline __attribute__((always_inline))
#define ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
#define ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
#define ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))

#ifndef ASSERT
#define ASSERT(check) if(!(check)) {PRINTF("Assert " __FILE__ ":%u ("#check")\n",__LINE__);HALT();}
#endif

#define CLAMP(v,a,b) (v<a?a:(b<v?b:v))
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define ALIGN16(count, size)		((((((count) * (size)) + 15) & (~15)) + (size) - 1)/(size))
#define ALIGN128(count, size)		((((((count) * (size)) + 127) & (~127)) + (size) - 1)/(size))

const f32 PI = 3.14159265358979f;

using namespace Vectormath::Aos;

#endif /* TPTYPES_H_ */
