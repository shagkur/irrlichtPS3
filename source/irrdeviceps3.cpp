/*
 * irrdeviceps3.cpp
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#include "irrtypes.h"
#include "os.h"
#include "rsxdriver.h"
#include "irrdeviceps3.h"
#include "sirrcreationparameters.h"

#include <sys/thread.h>

#define FONT_FILE_CACHE_SIZE			(1*1024*1024)

namespace irr
{
	namespace video
	{
		IVideoDriver* createRSXDriver(const SIrrlichtCreationParameters& param, io::IFileSystem *fs, IrrlichtDevice *device);
	}

	struct SJoypadPS3Control
	{
		CIrrDevicePS3 *_device;

		u8 _oldPadStatus[IRR_MAX_PAD_NUM];
		SEvent _lastEvent[IRR_MAX_PAD_NUM];
		bool _sensorModeEnabled[IRR_MAX_PAD_NUM];

		SJoypadPS3Control(CIrrDevicePS3 *device)
		: _device(device)
		{
			ioPadInit(IRR_MAX_PAD_NUM);

			for(u32 i=0;i < IRR_MAX_PAD_NUM;i++) {
				_oldPadStatus[i] = 0;
				_sensorModeEnabled[i] = false;
			}
		}

		void pollJoypads()
		{
			padInfo padinfo;
			padData paddata;

			ioPadGetInfo(&padinfo);
			for(u32 i=0;i < padinfo.connected;i++) {
				if(padinfo.status[i]) {
					SEvent event;

					if(_oldPadStatus[i] == 0) {
						bool bNewPad = false;
						s32 ret = ioPadGetData(i, &paddata);

						if(ret == 0)
							bNewPad = true;

						if(bNewPad) {
							if(ioPadInfoSensorMode(i) == PAD_INFO_SUPPORTED_SENSOR_MODE) {
								ioPadSetSensorMode(i, PAD_SENSOR_MODE_ON);
								_sensorModeEnabled[i] = true;
							} else
								_sensorModeEnabled[i] = false;
						}
					}

					s32 ret = ioPadGetData(i, &paddata);
					if(ret == 0 && paddata.len > 0) {
						if(paddata.BTN_START) CIrrDevicePS3::_running = false;

						event.eventType = EET_JOYPAD_INPUT_EVENT;
						event.joypadEvent.joypad = i;
						event.joypadEvent.buttonStates = buttonMask(paddata);

						event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_LSTICK_X] = calcValues(((paddata.ANA_L_H&0x00ff)*2 - 255), 0.001f, 0.0f, 0.0002f, SEvent::SJoypadEvent::AXIS_LSTICK_X);
						event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_LSTICK_Y] = calcValues(((paddata.ANA_L_V&0x00ff)*2 - 255), 0.001f, 0.0f, 0.0002f, SEvent::SJoypadEvent::AXIS_LSTICK_Y);

						event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_RSTICK_X] = calcValues(((paddata.ANA_R_H&0x00ff)*2 - 255), 0.1f, 0.0f, 0.02f, SEvent::SJoypadEvent::AXIS_RSTICK_X);
						event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_RSTICK_Y] = calcValues(((paddata.ANA_R_V&0x00ff)*2 - 255), -0.1f, 0.0f, 0.02f, SEvent::SJoypadEvent::AXIS_RSTICK_Y);

						if(_sensorModeEnabled[i]) {
							event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_SENSOR_X] = calcValues(((paddata.SENSOR_X&0x03ff)*2 - 1023), 0.001f, 0.0f, 0.0002f, SEvent::SJoypadEvent::AXIS_SENSOR_X);
							event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_SENSOR_Y] = calcValues(((paddata.SENSOR_Y&0x03ff)*2 - 1023), 0.001f, 0.0f, 0.0002f, SEvent::SJoypadEvent::AXIS_SENSOR_Y);
							event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_SENSOR_Z] = calcValues(((paddata.SENSOR_Z&0x03ff)*2 - 1023), 0.001f, 0.0f, 0.0002f, SEvent::SJoypadEvent::AXIS_SENSOR_Z);
							event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_SENSOR_W] = calcValues(((paddata.SENSOR_G&0x03ff)*2 - 1023), 0.001f, 0.0f, 0.0002f, SEvent::SJoypadEvent::AXIS_SENSOR_W);
						} else {
							event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_SENSOR_X] = 0;
							event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_SENSOR_Y] = 0;
							event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_SENSOR_Z] = 0;
							event.joypadEvent.axis[SEvent::SJoypadEvent::AXIS_SENSOR_W] = 0;
						}

						_device->postEventFromUser(event);
						_lastEvent[i] = event;
					}
				}
				_oldPadStatus[i] = padinfo.status[i];
			}
		}

		f32 calcValues(s32 val, f32 gain, f32 bias, f32 deadZone, u32 type)
		{
			f32 fVal = 0.0f;

			switch(type) {
				case SEvent::SJoypadEvent::AXIS_LSTICK_X:
				case SEvent::SJoypadEvent::AXIS_LSTICK_Y:
				case SEvent::SJoypadEvent::AXIS_RSTICK_X:
				case SEvent::SJoypadEvent::AXIS_RSTICK_Y:
					fVal = ((f32)val)/255.0f;
					break;
				case SEvent::SJoypadEvent::AXIS_SENSOR_X:
				case SEvent::SJoypadEvent::AXIS_SENSOR_Y:
				case SEvent::SJoypadEvent::AXIS_SENSOR_Z:
				case SEvent::SJoypadEvent::AXIS_SENSOR_W:
					fVal = ((f32)val)/1023.0f;
					break;
			}

			fVal = fVal*gain + bias;
			if(fabsf(fVal) < deadZone) fVal = 0.0f;

			return fVal;
		}

		u32 buttonMask(const padData& data)
		{
			u32 mask = 0;
			if(data.BTN_LEFT) mask |= (1 << PAD_BUTTON_LEFT);
			if(data.BTN_RIGHT) mask |= (1 << PAD_BUTTON_RIGHT);
			if(data.BTN_UP) mask |= (1 << PAD_BUTTON_UP);
			if(data.BTN_DOWN) mask |= (1 << PAD_BUTTON_DOWN);
			if(data.BTN_CROSS) mask |= (1 << PAD_BUTTON_CROSS);
			if(data.BTN_SQUARE) mask |= (1 << PAD_BUTTON_SQUARE);
			if(data.BTN_TRIANGLE) mask |= (1 << PAD_BUTTON_TRIANGLE);
			if(data.BTN_CIRCLE) mask |= (1 << PAD_BUTTON_CIRCLE);
			if(data.BTN_START) mask |= (1 << PAD_BUTTON_START);
			if(data.BTN_SELECT) mask |= (1 << PAD_BUTTON_SELECT);
			if(data.BTN_R1) mask |= (1 << PAD_BUTTON_R1);
			if(data.BTN_L1) mask |= (1 << PAD_BUTTON_L1);
			if(data.BTN_R2) mask |= (1 << PAD_BUTTON_R2);
			if(data.BTN_L2) mask |= (1 << PAD_BUTTON_L2);
			if(data.BTN_R3) mask |= (1 << PAD_BUTTON_R3);
			if(data.BTN_L3) mask |= (1 << PAD_BUTTON_L3);
			return mask;
		}
	};

	bool CIrrDevicePS3::_running = false;
	CIrrDevicePS3* CIrrDevicePS3::_instance = NULL;

	static s32 getSecondaryPPUThreadPrio(s32 *prio)
	{
		s32 ret;
		sys_ppu_thread_t currentThread;

		ret = sysThreadGetId(&currentThread);
		if(ret) return ret;

		ret = sysThreadGetPriority(currentThread, prio);
		if(ret) return ret;

		*prio = *prio - 1;
		return 0;
	}

	CIrrDevicePS3::CIrrDevicePS3(const SIrrlichtCreationParameters& param)
	: CIrrDeviceStub(param), _fontFileCache(NULL)
	{
		_instance = this;

		sysModuleLoad(SYSMODULE_FS);

		sysSpuInitialize(NUM_SPUS, 0);

		initializeMarsContext();

		loadFontEngineLibraries();
		initializeFontEngine();
		loadSystemFonts();

		atexit(CIrrDevicePS3::program_exit_callback);
		sysUtilRegisterCallback(0, CIrrDevicePS3::sysutil_exit_callback, NULL);

		_joyControl = new SJoypadPS3Control(this);

		_videoDriver = video::createRSXDriver(param, _fileSystem, this);
		if(_videoDriver == NULL) return;

		createGUIAndScene();

		CIrrDevicePS3::_running = true;
	}

	CIrrDevicePS3::~CIrrDevicePS3()
	{
		if(_videoDriver != NULL) _videoDriver->drop();

		mars_context_destroy(mMars);

		sysSpuPrintfFinalize();
	}

	bool CIrrDevicePS3::run()
	{
		if(CIrrDevicePS3::_running) {
			os::Timer::tick();

			sysUtilCheckCallback();

			_joyControl->pollJoypads();

			return true;
		}
		return false;
	}

	void CIrrDevicePS3::loadFontEngineLibraries()
	{
		s32 ret;

		ret = sysModuleLoad(SYSMODULE_FONT);
		if(ret == 0) {
			ret = sysModuleLoad(SYSMODULE_FREETYPE);
			if(ret == 0) {
				ret = sysModuleLoad(SYSMODULE_FONTFT);
				if(ret == 0) return;

				sysModuleUnload(SYSMODULE_FREETYPE);
			}
			sysModuleUnload(SYSMODULE_FONT);
		}
	}

	void CIrrDevicePS3::initializeFontEngine()
	{
		s32 ret;
		fontConfig config;
		fontLibraryConfigFT ftConfig;

		_fontFileCache = new u8[FONT_FILE_CACHE_SIZE];

		fontConfig_initialize(&config);

		config.fileCache.buffer = (u32*)_fontFileCache;
		config.fileCache.size = FONT_FILE_CACHE_SIZE;
		config.userFontEntries = _userFontEntries;
		config.userFontEntryMax = USER_FONT_MAX;
		config.flags = 0;

		ret = fontInit(&config);
		if(ret == 0) {
			fontLibraryConfigFT_initialize(&ftConfig);

			ftConfig.memoryIF.object = NULL;
			ftConfig.memoryIF.malloc_func = (fontMallocCallback)__get_opd32(CIrrDevicePS3::fonts_malloc);
			ftConfig.memoryIF.realloc_func = (fontReallocCallback)__get_opd32(CIrrDevicePS3::fonts_realloc);
			ftConfig.memoryIF.calloc_func = (fontCallocCallback)__get_opd32(CIrrDevicePS3::fonts_calloc);
			ftConfig.memoryIF.free_func = (fontFreeCallback)__get_opd32(CIrrDevicePS3::fonts_free);

			ret = fontInitLibraryFreeType(&ftConfig, &_fontLibrary);
			ASSERT(ret == 0);

			_fonts.openState = 0;
		}
	}

	void CIrrDevicePS3::loadSystemFonts()
	{
		s32 ret;

		static struct {
			u32 isMemory;
			s32 fontsetType;
		} openSystemFonts[SYSTEM_FONT_MAX] = {
			{ 0, FONT_TYPE_DEFAULT_GOTHIC_LATIN_SET },
			{ 1, FONT_TYPE_DEFAULT_SANS_SERIF },
			{ 1, FONT_TYPE_DEFAULT_SERIF }
		};

		_fonts.sysFontMax = 3;
		for(s32 i=0;i < _fonts.sysFontMax;i++) {
			if(!openSystemFonts[i].isMemory) continue;

			fontType type;

			type.type = openSystemFonts[i].fontsetType;
			type.map = FONT_MAP_UNICODE;
			ret = fontOpenFontsetOnMemory(_fontLibrary, &type, &_fonts.systemFont[i]);
			ASSERT(ret == 0);

			fontSetResolutionDpi(&_fonts.systemFont[i], 72, 72);
			fontSetScalePoint(&_fonts.systemFont[i], 26.0f, 26.0f);

			_fonts.openState |= (1<<i);
		}

		for(s32 i=0;i < _fonts.sysFontMax;i++) {
			if(openSystemFonts[i].isMemory) continue;

			fontType type;

			type.type = openSystemFonts[i].fontsetType;
			type.map = FONT_MAP_UNICODE;
			ret = fontOpenFontset(_fontLibrary, &type, &_fonts.systemFont[i]);
			ASSERT(ret == 0);

			fontSetResolutionDpi(&_fonts.systemFont[i], 72, 72);
			fontSetScalePoint(&_fonts.systemFont[i], 26.0f, 26.0f);

			_fonts.openState |= (1<<i);
		}
	}

	void CIrrDevicePS3::initializeMarsContext()
	{
		s32 ppu_thread_prio;

		getSecondaryPPUThreadPrio(&ppu_thread_prio);

		sysSpuPrintfInitialize(ppu_thread_prio, NULL);

		mars_context_create(&mMars, NUM_SPUS, SPU_THREAD_GROUP_PRIORITY, ppu_thread_prio);
	}

	void CIrrDevicePS3::terminate()
	{
		video::CRSXDriver *driver = dynamic_cast<video::CRSXDriver*>(_videoDriver);

		driver->finish();

		ioPadEnd();

		sysModuleUnload(SYSMODULE_FONTFT);
		sysModuleUnload(SYSMODULE_FREETYPE);
		sysModuleUnload(SYSMODULE_FONT);
		sysModuleUnload(SYSMODULE_FS);
	}

	void CIrrDevicePS3::program_exit_callback()
	{
		irr::CIrrDevicePS3::_instance->terminate();
	}

	void CIrrDevicePS3::sysutil_exit_callback(u64 status, u64 param, void *userdata)
	{
		switch(status) {
			case SYSUTIL_EXIT_GAME:
				irr::CIrrDevicePS3::_running = false;
				break;
			case SYSUTIL_DRAW_BEGIN:
			case SYSUTIL_DRAW_END:
				break;
			default:
				break;
		}
	}

	void* CIrrDevicePS3::fonts_malloc(void *object,u32 size)
	{
		(void)object;
		return malloc(size);
	}

	void* CIrrDevicePS3::fonts_realloc(void *object,void *p,u32 size)
	{
		(void)object;
		return realloc(p,size);
	}

	void* CIrrDevicePS3::fonts_calloc(void *object,u32 numb,u32 block_size)
	{
		(void)object;
		return calloc(numb, block_size);
	}

	void CIrrDevicePS3::fonts_free(void *object,void *ptr)
	{
		(void)object;
		free(ptr);
	}

	extern "C" IrrlichtDevice* createDeviceEx(const SIrrlichtCreationParameters& param)
	{
		return new CIrrDevicePS3(param);
	}

}

