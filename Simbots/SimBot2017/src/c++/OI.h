#ifndef OI_H
#define OI_H

#include "WPILib.h"

class OI
{
protected:

public:
	Joystick *stick;
	OI();
	Joystick *GetJoystick();
};


#endif
