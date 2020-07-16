/*
 * ehardwarebufferflags.h
 *
 *  Created on: Feb 4, 2013
 *      Author: mike
 */

#ifndef EHARDWAREBUFFERFLAGS_H_
#define EHARDWAREBUFFERFLAGS_H_

namespace irr
{
	namespace scene
	{
		enum E_HARDWARE_MAPPING
		{
			//! Don't store on the hardware
			EHM_NEVER = 0,

			//! Rarely changed, usually stored completely on the hardware
			EHM_STATIC,

			//! Sometimes changed, driver optimized placement
			EHM_DYNAMIC,

			//! Always changed, cache optimizing on the GPU
			EHM_STREAM
		};

		enum E_BUFFER_TYPE
		{
			//! Does not change anything
			EBT_NONE = 0,
			//! Change the vertex mapping
			EBT_VERTEX,
			//! Change the index mapping
			EBT_INDEX,
			//! Change both vertex and index mapping to the same value
			EBT_VERTEX_AND_INDEX

		};
	}
}

#endif /* EHARDWAREBUFFERFLAGS_H_ */
