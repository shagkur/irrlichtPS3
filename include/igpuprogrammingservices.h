/*
 * igpuprogrammingservices.h
 *
 *  Created on: Jul 7, 2013
 *      Author: mike
 */

#ifndef IGPUPROGRAMMINGSERVICES_H_
#define IGPUPROGRAMMINGSERVICES_H_

#include "ematerialtypes.h"
#include "eprimitivetypes.h"
#include "path.h"

namespace irr
{
	namespace io
	{
		class IReadFile;
	}

	namespace video
	{
		class IVideoDriver;
		class IShaderConstantSetCallback;

		class IGPUProgrammingServices
		{
		public:
			virtual ~IGPUProgrammingServices() {}

			virtual s32 addHighLevelShaderMaterial(const void *vertexShaderProgram, const void *fragmentShaderProgram, IShaderConstantSetCallback *callback = NULL, E_MATERIAL_TYPE baseMaterial = video::EMT_SOLID, s32 userData = 0) = 0;

			virtual s32 addHighLevelShaderMaterial(const io::path& vertexShaderProgram, const io::path& fragmentShaderProgram, IShaderConstantSetCallback *callback = NULL, E_MATERIAL_TYPE baseMaterial = video::EMT_SOLID, s32 userData = 0) = 0;

			virtual s32 addHighLevelShaderMaterial(io::IReadFile *vertexShaderProgram, io::IReadFile *fragmentShaderProgram, IShaderConstantSetCallback *callback = NULL, E_MATERIAL_TYPE baseMaterial = video::EMT_SOLID, s32 userData = 0) = 0;
		};
	}
}
#endif /* IGPUPROGRAMMINGSERVICES_H_ */
