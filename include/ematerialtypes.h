/*
 * ematerialtypes.h
 *
 *  Created on: Feb 6, 2013
 *      Author: mike
 */

#ifndef EMATERIALTYPES_H_
#define EMATERIALTYPES_H_

namespace irr
{
	namespace video
	{
		enum E_MATERIAL_TYPE
		{
			EMT_SOLID = 0,

			EMT_SOLID_2_LAYER,

			EMT_LIGHTMAP,

			EMT_LIGHTMAP_ADD,

			EMT_LIGHTMAP_M2,

			EMT_LIGHTMAP_M4,

			EMT_LIGHTMAP_LIGHTING,

			EMT_LIGHTMAP_LIGHTING_M2,

			EMT_LIGHTMAP_LIGHTING_M4,

			EMT_DETAIL_MAP,

			EMT_SPHERE_MAP,

			EMT_REFLECTION_2_LAYER,

			EMT_TRANSPARENT_ADD_COLOR,

			EMT_TRANSPARENT_ALPHA_CHANNEL,

			EMT_TRANSPARENT_ALPHA_CHANNEL_REF,

			EMT_TRANSPARENT_VERTEX_ALPHA,

			EMT_TRANSPARENT_REFLECTION_2_LAYER,

			EMT_NORMAL_MAP_SOLID,

			EMT_NORMAL_MAP_TRANSPARENT_ADD_COLOR,

			EMT_NORMAL_MAP_TRANSPARENT_VERTEX_ALPHA,

			EMT_PARALLAX_MAP_SOLID,

			EMT_PARALLAX_MAP_TRANSPARENT_ADD_COLOR,

			EMT_PARALLAX_MAP_TRANSPARENT_VERTEX_ALPHA,

			EMT_ONETEXTURE_BLEND,

			EMT_FORCE_32BIT = 0x7fffffff
		};
	}
}

#endif /* EMATERIALTYPES_H_ */
