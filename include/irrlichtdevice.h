/*
 * irrlichtdevice.h
 *
 *  Created on: Jan 31, 2013
 *      Author: mike
 */

#ifndef IRRLICHTDEVICE_H_
#define IRRLICHTDEVICE_H_

#include "irefcounter.h"
#include "itimer.h"
#include "ieventreceiver.h"
#include "ivideodriver.h"
#include "itimer.h"

namespace irr
{
	namespace io
	{
		class IFileSystem;
	}

	namespace scene
	{
		class ISceneManager;
	}

	class IrrlichtDevice : public virtual IRefCounter
	{
	public:
		virtual bool run() = 0;

		virtual void terminate() = 0;
		
		virtual video::IVideoDriver* getVideoDriver() = 0;

		virtual io::IFileSystem* getFileSystem() = 0;

		virtual scene::ISceneManager* getSceneManager() = 0;

		virtual void setEventReceiver(IEventReceiver *receiver) = 0;

		virtual IEventReceiver* getEventReceiver() = 0;

		virtual ITimer* getTimer() = 0;

		virtual bool postEventFromUser(const SEvent& event) = 0;
	};
}

#endif /* IRRLICHTDEVICE_H_ */
