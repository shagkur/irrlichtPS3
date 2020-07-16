/*
 * imageloaderpng.cpp
 *
 *  Created on: Feb 5, 2013
 *      Author: mike
 */

#include "irrtypes.h"
#include "imageloaderpng.h"
#include "readfile.h"
#include "image.h"

#include <pngdec/pngdec.h>

struct _ctrlStrmArg
{
	irr::io::IReadFile *file;
	void *strmBuffer;
};

extern "C" {
	static void* png_malloc(u32 size,void *usrdata)
	{
		return malloc(size);
	}

	static void png_free(void *ptr,void *usrdata)
	{
		free(ptr);
	}

	static s32 png_ctrlstrm(pngDecStreamInfo *info,pngDecStreamParam *param,void *usrdata)
	{
		s32 streamSize;
		struct _ctrlStrmArg *arg = (struct _ctrlStrmArg*)usrdata;

		streamSize = arg->file->read(arg->strmBuffer, PNG_STREAMBUF_SIZE);

		param->strm_ptr = arg->strmBuffer;
		param->strm_size = streamSize;

		return PNGDEC_ERROR_OK;
	}
}

namespace irr
{
	namespace video
	{
		CImageLoaderPNG::CImageLoaderPNG()
		: _pngDecHandle(-1)
		{
			pngDecThreadInParam inThrdParam;
			pngDecThreadOutParam outThrdParam;

			sysModuleLoad(SYSMODULE_PNGDEC);

			inThrdParam.spu_enable = PNGDEC_SPU_THREAD_DISABLE;
			inThrdParam.ppu_prio = 512;
			inThrdParam.spu_prio = 200;
			inThrdParam.malloc_func = (pngCbCtrlMalloc)__get_opd32(png_malloc);
			inThrdParam.malloc_arg = NULL;
			inThrdParam.free_func = (pngCbCtrlFree)__get_opd32(png_free);
			inThrdParam.free_arg = NULL;

			pngDecCreate(&_pngDecHandle, &inThrdParam, &outThrdParam);
		}

		CImageLoaderPNG::~CImageLoaderPNG()
		{
			pngDecDestroy(_pngDecHandle);

			sysModuleUnload(SYSMODULE_PNGDEC);
		}

		bool CImageLoaderPNG::isLoadableFileExtension(const io::path& filename) const
		{
			return core::hasFileExtension(filename, "png");
		}

		bool CImageLoaderPNG::isLoadableFileFormat(io::IReadFile *file) const
		{
			if(file == NULL) return false;
			return true;
		}

		IImage* CImageLoaderPNG::loadImage(io::IReadFile *file) const
		{
			if(file == NULL) return NULL;

			s32 ret;
			s32 subHandle;
			pngDecSource src;
			pngDecOpnInfo openInfo;
			pngDecOpnParam openParam;
			pngDecCtrlStrm ctrlStrm;
			struct _ctrlStrmArg cbStrmArg;
			IImage *image = NULL;

			s32 streamSize = file->read((void*)_pngStrmBuffer, PNG_STREAMBUF_SIZE);
			if(streamSize <= 0) return NULL;

			src.stream_sel = PNGDEC_BUFFER;
			src.file_name = NULL;
			src.file_offset = 0;
			src.file_size = 0;
			src.stream_ptr = (void*)_pngStrmBuffer;
			src.stream_size = streamSize;
			src.spu_enable = PNGDEC_SPU_THREAD_DISABLE;

			cbStrmArg.file = file;
			cbStrmArg.strmBuffer = (void*)_pngStrmBuffer;

			ctrlStrm.stream_func = (pngCbCtrlStrm)__get_opd32(png_ctrlstrm);
			ctrlStrm.stream_arg = &cbStrmArg;

			openParam.select_chunk = 0;

			ret = pngDecExtOpen(_pngDecHandle, &subHandle, &src, &openInfo, &ctrlStrm, &openParam);
			if(ret == PNGDEC_ERROR_OK) {
				pngDecInfo decInfo;
				pngDecExtInfo extInfo;
				pngDecInParam inParam;
				pngDecOutParam outParam;
				pngDecExtInParam extInParam;
				pngDecExtOutParam extOutParam;

				ret = pngDecExtReadHeader(_pngDecHandle, subHandle, &decInfo, &extInfo);
				if(ret == PNGDEC_ERROR_OK) {
					inParam.cmd_ptr = NULL;
					inParam.output_mode = PNGDEC_TOP_TO_BOTTOM;
					inParam.color_space = PNGDEC_ARGB;
					inParam.bit_depth = 8;
					inParam.pack_flag = PNGDEC_1BYTE_PER_1PIXEL;
					if(decInfo.color_space == PNGDEC_GRAYSCALE_ALPHA || decInfo.color_space == PNGDEC_RGBA || decInfo.chunk_info&0x10)
						inParam.alpha_select = PNGDEC_STREAM_ALPHA;
					else
						inParam.alpha_select = PNGDEC_FIX_ALPHA;
					inParam.alpha = 0xff;

					extInParam.buffer_mode = PNGDEC_LINE_MODE;
					extInParam.output_counts = 0;
					extInParam.spu_mode = PNGDEC_RECEIVE_EVENT;

					ret = pngDecExtSetParameter(_pngDecHandle, subHandle, &inParam, &outParam, &extInParam, &extOutParam);
				}

				if(ret == PNGDEC_ERROR_OK) {
					image = new CImage(ECF_A8R8G8B8, core::dimension2d<u32>(outParam.width, outParam.height));
					if(image != NULL) {
						pngDecDataInfo decDataInfo;
						pngDecDataCtrlParam decDataCtrl;
						u8 *data = (u8*)image->lock();

						decDataCtrl.output_bytes_per_line = image->getPitch();
						ret = pngDecExtDecodeData(_pngDecHandle, subHandle, data, &decDataCtrl, &decDataInfo, NULL, NULL);
						if(ret != PNGDEC_ERROR_OK || decDataInfo.decode_status != PNGDEC_STATUS_FINISH) {
							image->drop();
							image = NULL;
						} else
							image->unlock();
					}
				}
				pngDecClose(_pngDecHandle, subHandle);
			}

			return image;
		}

		IImageLoader* createImageLoaderPNG()
		{
			return new CImageLoaderPNG();
		}
	}
}
