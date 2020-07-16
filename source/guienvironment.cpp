/*
 * guienvironment.cpp
 *
 *  Created on: Mar 8, 2013
 *      Author: mike
 */

#include "guienvironment.h"
#include "ivideodriver.h"
#include "os.h"

namespace irr
{
	namespace gui
	{
		const io::path CGUIEnvironment::defaultFontName = "#DefaultFont";

		CGUIEnvironment::CGUIEnvironment(io::IFileSystem *fs, video::IVideoDriver *driver)
		: _driver(driver), _fileSystem(fs)
		{
			if(_driver != NULL) _driver->grab();
			if(_fileSystem != NULL) _fileSystem->grab();
		}

		CGUIEnvironment::~CGUIEnvironment()
		{
			if(_fileSystem != NULL) _fileSystem->drop();
			if(_driver != NULL) _driver->drop();
		}

		void CGUIEnvironment::loadBuiltInFont()
		{

		}

		void CGUIEnvironment::drawAll()
		{

		}

		video::IVideoDriver* CGUIEnvironment::getVideoDriver() const
		{
			return _driver;
		}

		io::IFileSystem* CGUIEnvironment::getFileSystem() const
		{
			return _fileSystem;
		}

		void CGUIEnvironment::clear()
		{

		}

		void CGUIEnvironment::setUserEventReceiver(IEventReceiver *evr)
		{
			_userReceiver = evr;
		}

		bool CGUIEnvironment::postEventFromUser(const SEvent& event)
		{
			return false;
		}
	}
}
