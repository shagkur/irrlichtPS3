/*
 * smaterial.h
 *
 *  Created on: Feb 1, 2013
 *      Author: mike
 */

#ifndef SMATERIAL_H_
#define SMATERIAL_H_

#include "scolor.h"
#include "matrix4.h"
#include "irrarray.h"
#include "irrmath.h"
#include "ematerialtypes.h"
#include "smateriallayer.h"
#include "ematerialflags.h"

namespace irr
{
	namespace video
	{
		enum E_BLEND_FACTOR
		{
			EBF_ZERO = 0,
			EBF_ONE,
			EBF_DST_COLOR,
			EBF_ONE_MINUS_DST_COLOR,
			EBF_SRC_COLOR,
			EBF_ONE_MINUS_SRC_COLOR,
			EBF_SRC_ALPHA,
			EBF_ONE_MINUS_SRC_ALPHA,
			EBF_DST_ALPHA,
			EBF_ONE_MINUS_DST_ALPHA,
			EBF_SRC_ALPHA_SATURATE
		};

		enum E_BLEND_OPERATION
		{
			EBO_NONE = 0,
			EBO_ADD,
			EBO_SUBTRACT,
			EBO_REVSUBTRACT,
			EBO_MIN,
			EBO_MAX,
			EBO_MIN_FACTOR,
			EBO_MAX_FACTOR,
			EBO_MIN_ALPHA,
			EBO_MAX_ALPHA
		};

		enum E_MODULATE_FUNC
		{
			EMFN_MODULATE_1X = 1,
			EMFN_MODULATE_2X = 2,
			EMFN_MODULATE_4X = 4
		};

		enum E_COMPARISON_FUNC
		{
			ECFN_DISABLED = 0,

			ECFN_LESSEQUAL,

			ECFN_EQUAL,

			ECFN_LESS,

			ECFN_NOTEQUAL,

			ECFN_GREATEREQUAL,

			ECFN_GREATER,

			ECFN_ALWAYS,

			ECFN_NEVER
		};

		enum E_COLOR_PLANE
		{
			ECP_NONE = 0,

			ECP_ALPHA = 1,

			ECP_RED = 2,

			ECP_GREEN = 4,

			ECP_BLUE = 8,

			ECP_RGB = 14,

			ECP_ALL = 15
		};

		enum E_COLOR_MATERIAL
		{
			ECM_NONE = 0,

			ECM_DIFFUSE,

			ECM_AMBIENT,

			ECM_EMISSIVE,

			ECM_SPECULAR,

			ECM_DIFFUSE_AND_AMBIENT
		};

		const u32 MATERIAL_MAX_TEXTURES = 16;
		const u32 MATERIAL_MAX_PARAMS = 8;

		class SMaterial
		{
		public:
			SMaterial()
			: materialType(EMT_SOLID), ambientColor(255,255,255,255), diffuseColor(255,255,255,255),
			  emissiveColor(0,0,0,0), specularColor(255,255,255,255), shininess(0.0f), zBuffer(ECFN_LESSEQUAL),
			  colorMask(ECP_ALL), colorMaterial(ECM_DIFFUSE), blendOperation(EBO_NONE), wireFrame(false),
			  pointCloud(false), gouraudShading(true), lighting(true), zWriteEnable(true), backfaceCulling(true),
			  frontfaceCulling(false)
			{
				for(u32 i=0;i < MATERIAL_MAX_TEXTURES;i++)
					textureLayer[i]._textureMatrix = NULL;

				for(u32 i=0;i < MATERIAL_MAX_PARAMS;i++)
					materialTypeParams[i] = 0.0f;
			}

			SMaterial(const SMaterial& other)
			{
				for(u32 i=0;i < MATERIAL_MAX_TEXTURES;i++)
					textureLayer[i]._textureMatrix = NULL;

				for(u32 i=0;i < MATERIAL_MAX_PARAMS;i++)
					materialTypeParams[i] = 0.0f;

				*this = other;
			}

			SMaterial& operator = (const SMaterial& other)
			{
				if(this == &other) return *this;

				materialType = other.materialType;
				ambientColor = other.ambientColor;
				diffuseColor = other.diffuseColor;
				emissiveColor = other.emissiveColor;
				specularColor = other.specularColor;
				shininess = other.shininess;
				zBuffer = other.zBuffer;
				colorMask = other.colorMask;
				colorMaterial = other.colorMaterial;
				wireFrame = other.wireFrame;
				pointCloud = other.pointCloud;
				gouraudShading = other.gouraudShading;
				lighting = other.lighting;
				zWriteEnable = other.zWriteEnable;
				backfaceCulling = other.backfaceCulling;
				frontfaceCulling = other.frontfaceCulling;
				blendOperation = other.blendOperation;

				for(u32 i=0;i < MATERIAL_MAX_TEXTURES;i++)
					textureLayer[i] = other.textureLayer[i];

				for(u32 i=0; i< MATERIAL_MAX_PARAMS;i++)
					materialTypeParams[i] = other.materialTypeParams[i];

				return *this;
			}

			SMaterialLayer textureLayer[MATERIAL_MAX_TEXTURES];

			E_MATERIAL_TYPE materialType;

			SColor ambientColor;
			SColor diffuseColor;
			SColor emissiveColor;
			SColor specularColor;

			f32 shininess;
			f32 materialTypeParams[MATERIAL_MAX_PARAMS];

			E_COMPARISON_FUNC zBuffer;

			u8 colorMask : 4;
			u8 colorMaterial : 3;

			E_BLEND_OPERATION blendOperation;

			bool wireFrame : 1;
			bool pointCloud : 1;
			bool gouraudShading : 1;
			bool lighting : 1;
			bool zWriteEnable : 1;
			bool backfaceCulling : 1;
			bool frontfaceCulling : 1;

			core::matrix4& getTextureMatrix(u32 i)
			{
				return textureLayer[i].getTextureMatrix();
			}

			const core::matrix4& getTextureMatrix(u32 i) const
			{
				if(i < MATERIAL_MAX_TEXTURES)
					return textureLayer[i].getTextureMatrix();
				else
					return core::identityMatrix;
			}

			void setTextureMatrix(u32 i, const core::matrix4& mat)
			{
				if(i >= MATERIAL_MAX_TEXTURES) return;
				textureLayer[i].setTextureMatrix(mat);
			}

			ITexture* getTexture(u32 i) const
			{
				return i < MATERIAL_MAX_TEXTURES ? textureLayer[i].texture : NULL;
			}

			void setTexture(u32 i, ITexture *tex)
			{
				if(i >= MATERIAL_MAX_TEXTURES) return;
				textureLayer[i].texture = tex;
			}

			void setFlag(E_MATERIAL_FLAG flag, u32 value)
			{
				switch(flag) {
					case EMF_WIREFRAME:				wireFrame = (bool)value; break;
					case EMF_POINTCLOUD:			pointCloud = (bool)value; break;
					case EMF_GOURAUD_SHADING:		gouraudShading = (bool)value; break;
					case EMF_LIGHTING:				lighting = (bool)value; break;
					case EMF_ZBUFFER:				zBuffer = (E_COMPARISON_FUNC)value; break;
					case EMF_ZWRITE_ENABLE:			zWriteEnable = (bool)value; break;
					case EMF_BACK_FACE_CULLING:		backfaceCulling = (bool)value; break;
					default:
						break;
				}
			}

			u32 getFlag(E_MATERIAL_FLAG flag) const
			{
				return 0;
			}

			bool operator != (const SMaterial& other) const
			{
				bool different = materialType != other.materialType ||
								 ambientColor != other.ambientColor ||
								 diffuseColor != other.diffuseColor ||
								 emissiveColor != other.emissiveColor ||
								 specularColor != other.specularColor ||
								 shininess != other.shininess ||
								 zBuffer != other.zBuffer ||
								 colorMask != other.colorMask ||
								 colorMaterial != other.colorMaterial ||
								 wireFrame != other.wireFrame ||
								 pointCloud != other.pointCloud ||
								 gouraudShading != other.gouraudShading ||
								 lighting != other.lighting ||
								 zWriteEnable != other.zWriteEnable ||
								 backfaceCulling != other.backfaceCulling ||
								 frontfaceCulling != other.frontfaceCulling ||
								 blendOperation != other.blendOperation;

				for(u32 i=0;i < MATERIAL_MAX_TEXTURES;i++)
					different |= (textureLayer[i] != other.textureLayer[i]);

				return different;
			}

			bool operator == (const SMaterial& other) const
			{
				return !(*this != other);
			}

			bool isTransparent() const
			{
				return (materialType == EMT_TRANSPARENT_ADD_COLOR ||
						materialType == EMT_TRANSPARENT_ALPHA_CHANNEL ||
						materialType == EMT_TRANSPARENT_VERTEX_ALPHA);
			}
		};

		extern SMaterial identityMaterial;
	}
}
#endif /* SMATERIAL_H_ */
