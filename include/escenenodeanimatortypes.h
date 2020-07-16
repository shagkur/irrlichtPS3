#ifndef __ESCENENODEANIMATORTYPES_H__
#define __ESCENENODEANIMATORTYPES_H__

namespace irr
{
	namespace scene
	{
		enum ESCENE_NODE_ANIMATOR_TYPE
		{
			//! Fly circle scene node animator
			ESNAT_FLY_CIRCLE = 0,

			//! Fly straight scene node animator
			ESNAT_FLY_STRAIGHT,

			//! Follow spline scene node animator
			ESNAT_FOLLOW_SPLINE,

			//! Rotation scene node animator
			ESNAT_ROTATION,

			//! Texture scene node animator
			ESNAT_TEXTURE,

			//! Deletion scene node animator
			ESNAT_DELETION,

			//! Collision respose scene node animator
			ESNAT_COLLISION_RESPONSE,

			//! FPS camera animator
			ESNAT_CAMERA_FPS,

			//! Maya camera animator
			ESNAT_CAMERA_MAYA,

			//! Amount of built-in scene node animators
			ESNAT_COUNT,

			//! Unknown scene node animator
			ESNAT_UNKNOWN,

			//! This enum is never used, it only forces the compiler to
			//! compile these enumeration values to 32 bit.
			ESNAT_FORCE_32_BIT = 0x7fffffff
		};
	}
}

#endif
