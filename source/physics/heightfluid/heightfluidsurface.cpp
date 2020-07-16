/*
 * heightfluidsurface.cpp
 *
 *  Created on: Jun 11, 2013
 *      Author: mike
 */

#include "heightfluid/heightfluidsurface.h"

void HeightFluidSurface::initialize()
{
	totalWater = 0.0;
	writeBuffer = 0;
	readBuffer = 1;

	for(u32 j=0;j < fieldDepth;j++) {
		for(u32 i=0;i < fieldWidth;i++) {
			FieldPoint fp;
			fp.height[0] = fp.height[1] = fp.height[2] = 1.0f;
			fp.flag = 0;
			fp.offset = 127;
			fp.userData = (u16)((i<<8)|(j&0xff)); // TEST
			fields[readBuffer][j*fieldWidth + i] = fields[writeBuffer][j*fieldWidth + i] = fp;
			totalWater += 1.0f;
		}
	}

	// initialize edge
	for(u32 i=0;i < fieldWidth;i++) {
		s32 id1 = i;
		s32 id2 = (fieldDepth - 1)*fieldWidth + i;
		fields[readBuffer][id1].flag = fields[readBuffer][id2].flag = 1;
		fields[writeBuffer][id1].flag = fields[writeBuffer][id2].flag = 1;
	}
	for(u32 j=0;j < fieldDepth;j++) {
		s32 id1 = j*fieldWidth;
		s32 id2 = j*fieldWidth + fieldWidth - 1;
		fields[readBuffer][id1].flag = fields[readBuffer][id2].flag = 1;
		fields[writeBuffer][id1].flag = fields[writeBuffer][id2].flag = 1;
	}

	for(u32 j=0;j < fieldDepth;j++) {
		for(u32 i=0;i < fieldWidth;i++) {
			// initialize position
			Vector3 pos(i, getHeight(i, j), j);
			pos = mulPerElem(fieldScale, (pos - 0.5f*Vector3(fieldWidth, 0.0f, fieldDepth)));

			s32 idx = j*fieldWidth + i;
			hfVertex[writeBuffer][idx] = hfVertex[readBuffer][idx] = pos;

			// initialize normal
			hfNormal[writeBuffer][idx] = hfNormal[readBuffer][idx] = Vector3(0.0f, 1.0f, 0.0f);

			// initialize texcoord
			idx = (j*fieldWidth + i)*2;
			hfTexCoords[idx + 0] = i/(f32)fieldWidth;
			hfTexCoords[idx + 1] = 1.0f - j/(f32)fieldDepth;
		}
	}

	// initialize indices
	s32 c=0;
	for(u32 i=0;i < fieldWidth - 1;i++) {
		for(u32 j=0;j < fieldDepth - 1;j++) {
			hfMeshIndices[c++] = j*fieldWidth + i;
			hfMeshIndices[c++] = (j + 1)*fieldWidth + i;
			hfMeshIndices[c++] = j*fieldWidth + (i + 1);
			hfMeshIndices[c++] = (j + 1)*fieldWidth + (i + 1);
			hfMeshIndices[c++] = j*fieldWidth + (i + 1);
			hfMeshIndices[c++] =  (j + 1)*fieldWidth + i;
		}
	}
}

void HeightFluidSurface::copyFields()
{
	memcpy(fields[writeBuffer], fields[readBuffer], sizeof(FieldPoint)*fieldWidth*fieldDepth);
}

