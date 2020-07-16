/*
 * imageloaderjpg.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: mike
 */

#include "imageloaderjpg.h"
#include "ireadfile.h"
#include "image.h"
#include "os.h"
#include "irrstring.h"

#include <jpgdec/jpgdec.h>

struct _ctrlStrmArg
{
	irr::io::IReadFile *file;
	void *strmBuffer;
};

extern "C" {
	static void* jpg_malloc(u32 size, void *usrdata)
	{
		return malloc(size);
	}

	static void jpg_free(void *ptr, void *usrdata)
	{
		free(ptr);
	}

	static s32 jpg_ctrlstrm(jpgDecStrmInfo *info, jpgDecStrmParam *param, void *usrdata)
	{
		u32 streamSize;
		struct _ctrlStrmArg *arg = (struct _ctrlStrmArg*)usrdata;

		streamSize = arg->file->read(arg->strmBuffer, JPG_STREAMBUF_SIZE);

		param->strm_ptr = arg->strmBuffer;
		param->strm_size = streamSize;

		return JPGDEC_ERROR_OK;
	}
}

namespace irr
{
	namespace video
	{
		CImageLoaderJPG::CImageLoaderJPG()
		: _jpgDecHandle(-1), _jpgDecCommand(JPGDEC_CONTINUE)
		{
			jpgDecThreadInParam inThrdParam;
			jpgDecThreadOutParam outThrdParam;

			sysModuleLoad(SYSMODULE_JPGDEC);

			inThrdParam.spu_enable = JPGDEC_SPU_THREAD_DISABLE;
			inThrdParam.ppu_prio = 512;
			inThrdParam.spu_prio = 200;
			inThrdParam.malloc_func = (jpgCbCtrlMalloc)__get_opd32(jpg_malloc);
			inThrdParam.malloc_arg = NULL;
			inThrdParam.free_func = (jpgCbCtrlFree)__get_opd32(jpg_free);
			inThrdParam.free_arg = NULL;

			jpgDecCreate(&_jpgDecHandle, &inThrdParam, &outThrdParam);
		}

		CImageLoaderJPG::~CImageLoaderJPG()
		{
			jpgDecDestroy(_jpgDecHandle);

			sysModuleUnload(SYSMODULE_JPGDEC);
		}

		bool CImageLoaderJPG::isLoadableFileExtension(const io::path& filename) const
		{
			return core::hasFileExtension(filename, "jpg", "jpeg");
		}

		bool CImageLoaderJPG::isLoadableFileFormat(io::IReadFile *file) const
		{
			if(file == NULL) return false;
			return true;
		}

		IImage* CImageLoaderJPG::loadImage(io::IReadFile *file) const
		{
			if(file == NULL) return NULL;

			s32 ret;
			s32 subHandle;
			jpgDecSource src;
			jpgDecOpnInfo openInfo;
			jpgDecCtrlStrm ctrlStrm;
			struct _ctrlStrmArg cbStrmArg;
			IImage *image = NULL;


			s32 streamSize = file->read((void*)_jpgStrmBuffer, JPG_STREAMBUF_SIZE);
			if(streamSize <= 0) return NULL;

			src.stream_sel = JPGDEC_BUFFER;
			src.file_name = NULL;
			src.file_offset = 0;
			src.file_size = 0;
			src.stream_ptr = (void*)_jpgStrmBuffer;
			src.stream_size = streamSize;
			src.spu_enable = JPGDEC_SPU_THREAD_DISABLE;

			cbStrmArg.file = file;
			cbStrmArg.strmBuffer = (void*)_jpgStrmBuffer;

			ctrlStrm.strm_func = (jpgCbCtrlStrm)__get_opd32(jpg_ctrlstrm);
			ctrlStrm.strm_arg = &cbStrmArg;

			ret = jpgDecExtOpen(_jpgDecHandle, &subHandle, &src, &openInfo, &ctrlStrm);
			if(ret == JPGDEC_ERROR_OK) {
				jpgDecInfo decInfo;
				jpgDecExtInfo extInfo;
				jpgDecInParam inParam;
				jpgDecOutParam outParam;
				jpgDecExtInParam extInParam;
				jpgDecExtOutParam extOutParam;

				ret = jpgDecExtReadHeader(_jpgDecHandle, subHandle, &decInfo, &extInfo);
				if(ret == JPGDEC_ERROR_OK) {
					inParam.cmd_ptr = (vu32*)&_jpgDecCommand;
					inParam.quality_mode = JPGDEC_FAST;
					inParam.output_mode = JPGDEC_TOP_TO_BOTTOM;
					inParam.color_space = JPGDEC_ARGB;
					inParam.down_scale = 1;
					inParam.alpha = 0xff;

					if(extInfo.coeff_buffer_size > 0) {
						extInParam.coeff_buffer = malloc(extInfo.coeff_buffer_size);
					} else
						extInParam.coeff_buffer = NULL;

					extInParam.buffer_mode = JPGDEC_LINE_MODE;
					extInParam.output_counts = 0;
					extInParam.spu_mode = JPGDEC_RECEIVE_EVENT;

					ret = jpgDecExtSetParameter(_jpgDecHandle, subHandle, &inParam, &outParam, &extInParam, &extOutParam);
				}

				if(ret == JPGDEC_ERROR_OK) {
					image = new CImage(ECF_A8R8G8B8, core::dimension2d<u32>(outParam.width, outParam.height));
					if(image != NULL) {
						jpgDecDataInfo decDataInfo;
						jpgDecDataCtrlParam decDataCtrl;
						u8 *data = (u8*)image->lock();

						decDataCtrl.output_bytes_per_line = image->getPitch();
						ret = jpgDecExtDecodeData(_jpgDecHandle, subHandle, data, &decDataCtrl, &decDataInfo, NULL, NULL);
						if(ret != JPGDEC_ERROR_OK || decDataInfo.decode_status != JPGDEC_STATUS_FINISH) {
							image->drop();
							image = NULL;
						} else
							image->unlock();
					}
				}
				jpgDecClose(_jpgDecHandle, subHandle);
			}

			return image;
		}

		IImageLoader* createImageLoaderJPG()
		{
			return new CImageLoaderJPG();
		}
	}
}
