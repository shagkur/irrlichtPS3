/*
 * iguienvironment.h
 *
 *  Created on: Mar 8, 2013
 *      Author: mike
 */

#ifndef IGUIENVIRONMENT_H_
#define IGUIENVIRONMENT_H_

#include "irefcounter.h"
#include "rect.h"
#include "ieventreceiver.h"
#include "path.h"

namespace irr
{
	class IEventReceiver;

	namespace io
	{
		class IReadFile;
		class IFileSystem;
	}

	namespace video
	{
		class IVideoDriver;
		class ITexture;
	}

	namespace gui
	{
		class IGUIElement;
		class IGUIFont;
		class IGUISpriteBank;
		class IGUIStaticText;
		class IGUIElementFactory;

		class IGUIEnvironment : public virtual IRefCounter
		{
		public:
			virtual void drawAll() = 0;

			virtual video::IVideoDriver* getVideoDriver() const = 0;

			virtual io::IFileSystem* getFileSystem() const = 0;

			virtual void clear() = 0;

			virtual bool postEventFromUser(const SEvent& event) = 0;

			virtual void setUserEventReceiver(IEventReceiver *evr) = 0;


		};
	}
}


#endif /* IGUIENVIRONMENT_H_ */
