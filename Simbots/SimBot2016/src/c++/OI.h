/*
 * OI.h
 *
 *  Created on: Jun 3, 2014
 *      Author: alex
 */

#ifndef OI_H_
#define OI_H_

#include "WPILib.h"

	enum {
		SHOOTING,
		LOADING,
	};

class OI {
public:

    JoystickButton* d_right; // step angle up (shooter or loader)
    JoystickButton* d_left;  // step angle down (shooter or loader)
    JoystickButton* d_up;    // shoot or grab ball (mode dependent)
    JoystickButton* d_down;  // mode switch button
    JoystickButton* d_mode;  // mode switch button
    JoystickButton* d_target;  // mode switch button
	Joystick* joy;

	OI();
	// Subclasses to bind right button depending on mode
	class RightBtnCmnd1 : public Button {
		OI *oi;
	public:
		RightBtnCmnd1(OI* p){ oi=p;}
		bool Get(){ return oi->mode==SHOOTING && oi->d_right->Get();}
	};
	class RightBtnCmnd2 : public Button {
		OI *oi;
	public:
		RightBtnCmnd2(OI* p){ oi=p;}
		bool Get(){ return oi->mode==LOADING && oi->d_right->Get();}
	};

	// Subclasses to bind left button depending on mode
	class LeftBtnCmnd1 : public Button {
		OI *oi;
	public:
		LeftBtnCmnd1(OI* p){ oi=p;}
		bool Get(){ return oi->mode==SHOOTING && oi->d_left->Get();}
	};
	class LeftBtnCmnd2 : public Button {
		OI *oi;
	public:
		LeftBtnCmnd2(OI* p){ oi=p;}
		bool Get(){ return oi->mode==LOADING && oi->d_left->Get();}
	};

	// Subclasses to bind up button depending on mode
	class UpBtnCmnd1 : public Button {
		OI *oi;
	public:
		UpBtnCmnd1(OI* p){ oi=p;}
		bool Get(){ return oi->mode==SHOOTING && oi->d_up->Get();}
	};
	class UpBtnCmnd2 : public Button {
		OI *oi;
	public:
		UpBtnCmnd2(OI* p){ oi=p;}
		bool Get(){ return oi->mode==LOADING && oi->d_up->Get();}
	};

	// Subclasses to bind up button depending on mode
	class DownBtnCmnd1 : public Button {
		OI *oi;
	public:
		DownBtnCmnd1(OI* p){ oi=p;}
		bool Get(){ return oi->mode==SHOOTING && oi->d_down->Get();}
	};
	class DownBtnCmnd2 : public Button {
		OI *oi;
	public:
		DownBtnCmnd2(OI* p){ oi=p;}
		bool Get(){ return oi->mode==LOADING && oi->d_down->Get();}
	};

	RightBtnCmnd1 rightBtnCmnd1;
	RightBtnCmnd2 rightBtnCmnd2;
	LeftBtnCmnd1 leftBtnCmnd1;
	LeftBtnCmnd2 leftBtnCmnd2;
	UpBtnCmnd1 upBtnCmnd1;
	UpBtnCmnd2 upBtnCmnd2;
	DownBtnCmnd1 downBtnCmnd1;
	DownBtnCmnd2 downBtnCmnd2;
	static int mode;
public:
	Joystick* GetJoystick();
	static void SetMode(int m) { mode=m;}
	static int GetMode() { return mode;}
};

#endif /* OI_H_ */
