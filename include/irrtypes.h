/*
 * irrtypes.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef IRRTYPES_H_
#define IRRTYPES_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>
#include <math.h>
#include <setjmp.h>
#include <ppu-asm.h>
#include <ppu-types.h>

#include <io/pad.h>
#include <rsx/rsx.h>
#include <sys/spu.h>
#include <sysutil/sysutil.h>
#include <sysutil/video.h>
#include <sysmodule/sysmodule.h>

#include <sys/event_queue.h>

#include <font/font.h>
#include <font/fontFT.h>

#include <mars/context.h>

#include <vectormath/cpp/vectormath_aos.h>

#define NUM_SPUS				6

#ifndef ASSERT
#define ASSERT(check) if(!(check)) {printf("Assert " __FILE__ ":%u ("#check")\n",__LINE__);abort();}
#endif

using namespace Vectormath;
using namespace Vectormath::Aos;

namespace irr
{
	typedef char fschar_t;

	#if ( ((__GNUC__ > 4 ) || ((__GNUC__ == 4 ) && (__GNUC_MINOR__ >= 7))) && (defined(__GXX_EXPERIMENTAL_CXX0X) || __cplusplus >= 201103L) )
	#define _IRR_OVERRIDE_ override
	#endif
	
	//! creates four CC codes used in Irrlicht for simple ids
	/** some compilers can create those by directly writing the
	code like 'code', but some generate warnings so we use this macro here */
	#define MAKE_IRR_ID(c0, c1, c2, c3) \
			((u32)(u8)(c0) | ((u32)(u8)(c1) << 8) | \
			((u32)(u8)(c2) << 16) | ((u32)(u8)(c3) << 24 ))
}

#endif /* IRRTYPES_H_ */
