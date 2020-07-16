/*
 * rsxvertexstream.h
 *
 *  Created on: Feb 11, 2014
 *      Author: mike
 */

#ifndef RSXVERTEXSTREAM_H_
#define RSXVERTEXSTREAM_H_

#include "rsxstate.h"

namespace irr
{
	namespace video
	{
		class CRSXDriver;

		class CRSXVertexStream
		{
		public:
			CRSXVertexStream(u8 index, u8 stride, u8 elems, u8 dtype, u32 offset, u8 location, CRSXDriver *driver);
			CRSXVertexStream(const CRSXVertexStream& other);

			CRSXVertexStream& operator =(const CRSXVertexStream& other);

			void addToCmdBuffer();

			u8 getAttributeIndex() const
			{
				return _index;
			}

		private:
			u8 _index;
			u8 _stride;
			u8 _elems;
			u8 _type;
			u32 _offset;
			u8 _location;

			CRSXDriver *_driver;
			CRSXState *_gfxState;
		};
	}
}

#endif /* RSXVERTEXSTREAM_H_ */
