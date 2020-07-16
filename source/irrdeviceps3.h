/*
 * irrdeviceps3.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef IRRDEVICEPS3_H_
#define IRRDEVICEPS3_H_

#include "irrlichtdevice.h"
#include "irrdevicestub.h"

#define IRR_MAX_PAD_NUM			4

#define SYSTEM_FONT_MAX			16
#define USER_FONT_MAX			(32 - SYSTEM_FONT_MAX)
#define FONT_FILE_CACHE_SIZE	(1*1024*1024)

namespace irr
{
	enum EPAD_BUTTONS
	{
		PAD_BUTTON_LEFT = 0,
		PAD_BUTTON_DOWN,
		PAD_BUTTON_RIGHT,
		PAD_BUTTON_UP,
		PAD_BUTTON_START,
		PAD_BUTTON_R3,
		PAD_BUTTON_L3,
		PAD_BUTTON_SELECT,
		PAD_BUTTON_SQUARE,
		PAD_BUTTON_CROSS,
		PAD_BUTTON_CIRCLE,
		PAD_BUTTON_TRIANGLE,
		PAD_BUTTON_R1,
		PAD_BUTTON_L1,
		PAD_BUTTON_R2,
		PAD_BUTTON_L2
	};

	struct SJoypadPS3Control;

	class CIrrDevicePS3 : public CIrrDeviceStub
	{
		friend struct SJoypadPS3Control;

		struct Fonts_t
		{
			s32 sysFontMax;
			font systemFont[SYSTEM_FONT_MAX];

			s32 userFontMax;
			font userFont[USER_FONT_MAX];

			u32 openState;
		};

	public:
		CIrrDevicePS3(const SIrrlichtCreationParameters& param);
		virtual ~CIrrDevicePS3();

		virtual bool run() override;

		virtual void terminate() override;

		mars_context* getMARSContext() { return mMars; }

	private:
		void loadFontEngineLibraries();
		void initializeFontEngine();
		void loadSystemFonts();

		void initializeMarsContext();

		static void program_exit_callback();
		static void sysutil_exit_callback(u64 status, u64 param, void *userdata);

		static void* fonts_malloc(void *object,u32 size);
		static void* fonts_realloc(void *object,void *p,u32 size);
		static void* fonts_calloc(void *object,u32 numb,u32 block_size);
		static void fonts_free(void *object,void *ptr);

	private:
		static const u32 SPU_THREAD_GROUP_PRIORITY = 250;

		void *_fontFileCache;
		const fontLibrary *_fontLibrary;
		fontEntry _userFontEntries[USER_FONT_MAX];
		Fonts_t _fonts;
		mars_context* mMars;
		SJoypadPS3Control *_joyControl;

		static bool _running;
		static CIrrDevicePS3 *_instance;
	};
}

#endif /* IRRDEVICEPS3_H_ */
