/*
 * guienvironment.h
 *
 *  Created on: Mar 8, 2013
 *      Author: mike
 */

#ifndef GUIENVIRONMENT_H_
#define GUIENVIRONMENT_H_

#include "iguienvironment.h"
#include "irrarray.h"
#include "ifilesystem.h"

namespace irr
{
	namespace gui
	{
		class CGUIEnvironment : public IGUIEnvironment
		{
		public:
			CGUIEnvironment(io::IFileSystem *fs, video::IVideoDriver *driver);
			virtual ~CGUIEnvironment();

			virtual void drawAll();

			virtual video::IVideoDriver* getVideoDriver() const;

			virtual io::IFileSystem* getFileSystem() const;

			virtual bool postEventFromUser(const SEvent& event);

			virtual void setUserEventReceiver(IEventReceiver *evr);

			virtual void clear();

		private:
			void loadBuiltInFont();

			struct SFont
			{
				io::SNamedPath namedPath;
				IGUIFont *font;

				bool operator < (const SFont& other) const
				{
					return (namedPath < other.namedPath);
				}
			};

			struct SSpriteBank
			{
				io::SNamedPath namedPath;
				IGUISpriteBank *bank;

				bool operator < (const SSpriteBank& other) const
				{
					return (namedPath < other.namedPath);
				}
			};

			core::array< SFont > _fonts;
			core::array< SSpriteBank > _banks;
			video::IVideoDriver *_driver;
			io::IFileSystem *_fileSystem;
			IEventReceiver *_userReceiver;

			static const io::path defaultFontName;
		};
	}
}

#endif /* GUIENVIRONMENT_H_ */
