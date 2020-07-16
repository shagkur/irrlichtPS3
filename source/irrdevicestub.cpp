/*
 * irrdevicestub.cpp
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#include "irrtypes.h"
#include "irrdevicestub.h"
#include "iscenemanager.h"
#include "ifilesystem.h"
#include "timer.h"

namespace irr
{
	CIrrDeviceStub::CIrrDeviceStub(const SIrrlichtCreationParameters& param)
	: IrrlichtDevice(), _videoDriver(NULL), _sceneManager(NULL), _userReceiver(param.receiver),
	  _inputReceivingSceneManager(NULL), _creationParams(param)
	{
		_timer = new CTimer();
		_fileSystem = io::createFileSystem();
	}

	CIrrDeviceStub::~CIrrDeviceStub()
	{
		_fileSystem->drop();
		_timer->drop();

		if(_sceneManager) _sceneManager->drop();
	}

	video::IVideoDriver* CIrrDeviceStub::getVideoDriver()
	{
		return _videoDriver;
	}

	io::IFileSystem* CIrrDeviceStub::getFileSystem()
	{
		return _fileSystem;
	}

	scene::ISceneManager* CIrrDeviceStub::getSceneManager()
	{
		return _sceneManager;
	}

	void CIrrDeviceStub::setEventReceiver(IEventReceiver *receiver)
	{
		_userReceiver = receiver;
	}

	IEventReceiver* CIrrDeviceStub::getEventReceiver()
	{
		return _userReceiver;
	}

	ITimer* CIrrDeviceStub::getTimer()
	{
		return _timer;
	}

	bool CIrrDeviceStub::postEventFromUser(const SEvent& event)
	{
		bool absorbed = false;

		if(_userReceiver != NULL)
			absorbed = _userReceiver->onEvent(event);

		scene::ISceneManager *inputReceiver = _inputReceivingSceneManager;
		if(inputReceiver == NULL)
			inputReceiver = _sceneManager;

		if(!absorbed && inputReceiver != NULL)
			absorbed = inputReceiver->postEventFromUser(event);

		return absorbed;
	}

	void CIrrDeviceStub::createGUIAndScene()
	{
		_sceneManager = scene::createSceneManager(_videoDriver, _fileSystem);
	}
}
