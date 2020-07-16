/*
 * irrdevicestub.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef IRRDEVICESTUB_H_
#define IRRDEVICESTUB_H_

#include "irrlichtdevice.h"
#include "sirrcreationparameters.h"

namespace irr
{
	namespace io
	{
		IFileSystem* createFileSystem();
	}

	namespace scene
	{
		ISceneManager* createSceneManager(video::IVideoDriver *driver, io::IFileSystem *fs);
	}

	class CIrrDeviceStub : public IrrlichtDevice
	{
	public:
		CIrrDeviceStub(const SIrrlichtCreationParameters& param);
		virtual ~CIrrDeviceStub();

		virtual video::IVideoDriver* getVideoDriver();

		virtual io::IFileSystem* getFileSystem();

		virtual scene::ISceneManager* getSceneManager();

		virtual void setEventReceiver(IEventReceiver *receiver);

		virtual IEventReceiver* getEventReceiver();

		virtual ITimer* getTimer();

		virtual bool postEventFromUser(const SEvent& event);

	protected:
		void createGUIAndScene();

		ITimer *_timer;
		video::IVideoDriver *_videoDriver;
		scene::ISceneManager *_sceneManager;
		io::IFileSystem *_fileSystem;
		IEventReceiver *_userReceiver;
		scene::ISceneManager *_inputReceivingSceneManager;

		SIrrlichtCreationParameters _creationParams;
	};
}

#endif /* IRRDEVICESTUB_H_ */
