/*
 * rsxindexstream.h
 *
 *  Created on: Feb 11, 2014
 *      Author: mike
 */

#ifndef RSXINDEXSTREAM_H_
#define RSXINDEXSTREAM_H_

#include "irrtypes.h"

namespace irr
{
	namespace video
	{
		class CRSXDriver;

		class CRSXIndexStream
		{
		public:
			CRSXIndexStream() {}
			CRSXIndexStream(const void *indexList, u32 indexCount, u8 vtype, u8 itype, u8 location, CRSXDriver *driver);
			CRSXIndexStream(u32 indexOffset, u32 indexCount, u8 vtype, u8 itype, u8 location, CRSXDriver *driver);
			CRSXIndexStream(const CRSXIndexStream& other);

			CRSXIndexStream& operator =(const CRSXIndexStream& other);

			void addToCmdBuffer();

		private:
			const void *_indexList;
			u32 _indexCount;
			u8 _vtype;
			u8 _itype;
			u8 _location;
			bool _drawInline;
			CRSXDriver *_driver;

			u32 _offset;
		};
	}
}

#endif /* RSXINDEXSTREAM_H_ */
