/*
 * rsxvertexstream.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: mike
 */

#include "rsxdriver.h"
#include "rsxvertexstream.h"

namespace irr
{
	namespace video
	{
		CRSXVertexStream::CRSXVertexStream(u8 index, u8 stride, u8 elems, u8 dtype, u32 offset, u8 location, CRSXDriver *driver)
		: _index(index), _stride(stride), _elems(elems), _type(dtype), _offset(offset), _location(location),
		  _driver(driver), _gfxState(driver->getRSXState())
		{

		}

		CRSXVertexStream::CRSXVertexStream(const CRSXVertexStream& other)
		{
			*this = other;
		}

		CRSXVertexStream& CRSXVertexStream::operator =(const CRSXVertexStream& other)
		{
			_index = other._index;
			_stride = other._stride;
			_elems = other._elems;
			_type = other._type;
			_offset = other._offset;
			_location = other._location;
			_driver = other._driver;
			_gfxState = other._gfxState;

			return *this;
		}

		void CRSXVertexStream::addToCmdBuffer()
		{
			vertexDataArray_t *vtxArray = &_gfxState->state.vertexDataArray[_index];

			if(_stride != vtxArray->stride ||
			   _elems != vtxArray->size ||
			   _type != vtxArray->type ||
			   _offset != vtxArray->offset ||
			   _location != vtxArray->location)
			{
				rsxBindVertexArrayAttrib(_driver->getGcmContext(), _index, 0, _offset, _stride, _elems, _type, _location);

				vtxArray->location = _location;
				vtxArray->offset = _offset;
				vtxArray->size = _elems;
				vtxArray->stride = _stride;
				vtxArray->type = _type;
			}
		}
	}
}


