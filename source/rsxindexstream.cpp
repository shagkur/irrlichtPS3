/*
 * rsxindexstream.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: mike
 */

#include "rsxdriver.h"
#include "rsxindexstream.h"

namespace irr
{
	namespace video
	{
		CRSXIndexStream::CRSXIndexStream(const void *indexList, u32 indexCount, u8 vtype, u8 itype, u8 location, CRSXDriver *driver)
		: _indexList(indexList), _indexCount(indexCount), _vtype(vtype), _itype(itype), _location(location),
		  _drawInline(false), _driver(driver)
		{
			s32 ret = rsxAddressToOffset(indexList, &_offset);
			if(ret) _drawInline = true;
		}

		CRSXIndexStream::CRSXIndexStream(u32 indexOffset, u32 indexCount, u8 vtype, u8 itype, u8 location, CRSXDriver *driver)
		: _indexList(NULL), _indexCount(indexCount), _vtype(vtype), _itype(itype), _location(location),
		  _drawInline(false), _driver(driver), _offset(indexOffset)
		{
		}

		CRSXIndexStream::CRSXIndexStream(const CRSXIndexStream& other)
		{
			*this = other;
		}

		CRSXIndexStream& CRSXIndexStream::operator =(const CRSXIndexStream& other)
		{
			_indexList = other._indexList;
			_indexCount = other._indexCount;
			_vtype = other._vtype;
			_itype = other._itype;
			_location = other._location;
			_drawInline = other._drawInline;
			_driver = other._driver;
			_offset = other._offset;

			return *this;
		}

		void CRSXIndexStream::addToCmdBuffer()
		{
			if(_drawInline) {
				if(_itype == GCM_INDEX_TYPE_16B)
					rsxDrawInlineIndexArray16(_driver->getGcmContext(), _vtype, 0, _indexCount, static_cast<const u16*>(_indexList));
				else
					rsxDrawInlineIndexArray32(_driver->getGcmContext(), _vtype, 0, _indexCount, static_cast<const u32*>(_indexList));
			} else
				rsxDrawIndexArray(_driver->getGcmContext(), _vtype, _offset, _indexCount, _itype, _location);
		}
	}
}
