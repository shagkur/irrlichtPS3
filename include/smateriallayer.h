/*
 * smateriallayer.h
 *
 *  Created on: Feb 6, 2013
 *      Author: mike
 */

#ifndef SMATERIALLAYER_H_
#define SMATERIALLAYER_H_

#include "matrix4.h"
#include "irrallocator.h"

namespace irr
{
	namespace video
	{
		class ITexture;

		enum E_TEXTURE_CLAMP
		{
			ETC_REPEAT = 0,

			ETC_CLAMP,

			ETC_CLAMP_TO_EDGE,

			ETC_CLAMP_TO_BORDER,

			ETC_MIRROR,

			ETC_MIRROR_CLAMP,

			ETC_MIRROR_CLAMP_TO_EDGE,

			ETC_MIRROR_CLAMP_TO_BORDER
		};

		class SMaterialLayer
		{
		public:
			SMaterialLayer()
			: texture(NULL), textureWrapU(ETC_REPEAT), textureWrapV(ETC_REPEAT),
			  bilinearFilter(false), trilinearFilter(true), anisotropicFilter(0),
			  LODBias(0), _textureMatrix(NULL)
			{}

			SMaterialLayer(const SMaterialLayer& other)
			{
				_textureMatrix = NULL;
				*this = other;
			}

			~SMaterialLayer()
			{
				_matrixAllocator.destruct(_textureMatrix);
				_matrixAllocator.deallocate(_textureMatrix);
			}

			SMaterialLayer& operator = (const SMaterialLayer& other)
			{
				if(this == &other) return *this;

				texture = other.texture;
				if(_textureMatrix != NULL) {
					if(other._textureMatrix != NULL)
						*_textureMatrix = *other._textureMatrix;
					else {
						_matrixAllocator.destruct(_textureMatrix);
						_matrixAllocator.deallocate(_textureMatrix);
						_textureMatrix = NULL;
					}
				} else {
					if(other._textureMatrix != NULL) {
						_textureMatrix = _matrixAllocator.allocate(1);
						_matrixAllocator.construct(_textureMatrix, *other._textureMatrix);
					} else
						_textureMatrix = NULL;
				}

				textureWrapU = other.textureWrapU;
				textureWrapV = other.textureWrapV;
				bilinearFilter = other.bilinearFilter;
				trilinearFilter = other.trilinearFilter;
				anisotropicFilter = other.anisotropicFilter;
				LODBias = other.LODBias;

				return *this;
			}

			core::matrix4& getTextureMatrix()
			{
				if(_textureMatrix == NULL) {
					_textureMatrix = _matrixAllocator.allocate(1);
					_matrixAllocator.construct(_textureMatrix, core::identityMatrix);
				}
				return *_textureMatrix;
			}

			const core::matrix4& getTextureMatrix() const
			{
				if(_textureMatrix)
					return *_textureMatrix;
				else
					return core::identityMatrix;
			}

			void setTextureMatrix(const core::matrix4& mat)
			{
				if(_textureMatrix == NULL) {
					_textureMatrix = _matrixAllocator.allocate(1);
					_matrixAllocator.construct(_textureMatrix, mat);
				} else
					*_textureMatrix = mat;
			}

			inline bool operator != (const SMaterialLayer& other) const
			{
				bool different = texture != other.texture ||
								 textureWrapU != other.textureWrapU ||
								 textureWrapV != other.textureWrapV ||
								 bilinearFilter != other.bilinearFilter ||
								 trilinearFilter != other.trilinearFilter ||
								 anisotropicFilter != other.anisotropicFilter ||
								 LODBias != other.LODBias;

				if(different)
					return true;
				else
					different |= (_textureMatrix != other._textureMatrix &&
								  _textureMatrix != NULL && other._textureMatrix != NULL &&
								  *_textureMatrix != *(other._textureMatrix));

				return different;
			}

			inline bool operator == (const SMaterialLayer& other) const
			{
				return !(other != *this);
			}

			ITexture *texture;

			u8 textureWrapU;
			u8 textureWrapV;

			bool bilinearFilter : 1;
			bool trilinearFilter : 1;

			u8 anisotropicFilter;

			s8 LODBias;

		private:
			friend class SMaterial;

			core::matrix4 *_textureMatrix;
			core::irrallocator< core::matrix4 > _matrixAllocator;
		};
	}
}

#endif /* SMATERIALLAYER_H_ */
