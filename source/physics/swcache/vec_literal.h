/*
 * vec_literal.h
 *
 *  Created on: Jun 13, 2013
 *      Author: mike
 */

#ifndef VEC_LITERAL_H_
#define VEC_LITERAL_H_

#ifdef __SPU__
#include <spu_intrinsics.h>
#endif


#ifdef __ALTIVEC_LITERAL_STYLE__
/* Use altivec style.
 */
#define VEC_LITERAL(_type, ...) ((_type)(__VA_ARGS__))

#define VEC_SPLAT_U8(_val)      ((vector unsigned char)(_val))
#define VEC_SPLAT_S8(_val)      ((vector signed char)(_val))

#define VEC_SPLAT_U16(_val)     ((vector unsigned short)(_val))
#define VEC_SPLAT_S16(_val)     ((vector signed short)(_val))

#define VEC_SPLAT_U32(_val)     ((vector unsigned int)(_val))
#define VEC_SPLAT_S32(_val)     ((vector signed int)(_val))
#define VEC_SPLAT_F32(_val)     ((vector float)(_val))

#define VEC_SPLAT_U64(_val)     ((vector unsigned long long)(_val))
#define VEC_SPLAT_S64(_val)     ((vector signed long long)(_val))
#define VEC_SPLAT_F64(_val)     ((vector double)(_val))

#else
/* Use curly brace style.
 */
#define VEC_LITERAL(_type, ...) ((_type){__VA_ARGS__})

#define VEC_SPLAT_U8(_val)      ((vector unsigned char){_val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val})
#define VEC_SPLAT_S8(_val)      ((vector signed char){_val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val, _val})

#define VEC_SPLAT_U16(_val)     ((vector unsigned short){_val, _val, _val, _val, _val, _val, _val, _val})
#define VEC_SPLAT_S16(_val)     ((vector signed short){_val, _val, _val, _val, _val, _val, _val, _val})

#define VEC_SPLAT_U32(_val)     ((vector unsigned int){_val, _val, _val, _val})
#define VEC_SPLAT_S32(_val)     ((vector signed int){_val, _val, _val, _val})
#define VEC_SPLAT_F32(_val)     ((vector float){_val, _val, _val, _val})

#define VEC_SPLAT_U64(_val)     ((vector unsigned long long){_val, _val})
#define VEC_SPLAT_S64(_val)     ((vector signed long long){_val, _val})
#define VEC_SPLAT_F64(_val)     ((vector double){_val, _val})

#endif

#endif /* VEC_LITERAL_H_ */
