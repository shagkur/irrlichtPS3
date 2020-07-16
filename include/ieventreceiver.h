/*
 * ieventreceiver.h
 *
 *  Created on: Feb 15, 2013
 *      Author: mike
 */

#ifndef IEVENTRECEIVER_H_
#define IEVENTRECEIVER_H_

#include "irrstring.h"

namespace irr
{
	namespace gui
	{

	}

	enum EEVENT_TYPE
	{
		EET_GUI_EVENT = 0,

		EET_MOUSE_INPUT_EVENT,

		EET_KEY_INPUT_EVENT,

		EET_JOYSTICK_INPUT_EVENT,

		EET_JOYPAD_INPUT_EVENT,

		EGUIET_FORCE_32_BIT = 0x7fffffff
	};

	struct SEvent
	{
		struct SJoypadEvent
		{
			enum
			{
				NUMBER_OF_BUTTONS = 32,

				AXIS_LSTICK_X = 0,
				AXIS_LSTICK_Y,
				AXIS_RSTICK_X,
				AXIS_RSTICK_Y,
				AXIS_SENSOR_X,
				AXIS_SENSOR_Y,
				AXIS_SENSOR_Z,
				AXIS_SENSOR_W,			// used for gyro information

				NUMBER_OF_AXES
			};

			s8 joypad;

			u32 buttonStates;

			f32 axis[NUMBER_OF_AXES];

			bool isButtonPressed(u32 button) const
			{
				if(button >= (u32)NUMBER_OF_BUTTONS)
					return false;

				return (buttonStates&(1<<button)) ? true : false;
			}
		};

		EEVENT_TYPE eventType;

		union
		{
			struct SJoypadEvent joypadEvent;
		};
	};

	class IEventReceiver
	{
	public:
		virtual ~IEventReceiver() {}

		virtual bool onEvent(const SEvent& event) = 0;
	};
}


#endif /* IEVENTRECEIVER_H_ */
