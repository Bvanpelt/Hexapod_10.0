//*****************************************************************************
//  DipSwitches.h: Implemented as a Singleton class
//
//  Class definition for our DipSwitches class
//*****************************************************************************
#ifndef __DIP_SWITCHES_H__
#define __DIP_SWITCHES_H__

#include <Arduino.h>

//*****************************************************************************
//*****************************************************************************
//  class FaultsAndAlerts
//*****************************************************************************
//*****************************************************************************
class DipSwitches {

//*****************************************************************************
//  Member data
//*****************************************************************************
public:
static uint32_t		mSchedulingTimer;

private:
static uint16_t		mCurrentSwitchSettings; 

//*****************************************************************************
//  Member functions
//*****************************************************************************
public:
static void late_setup(uint32_t iNow);
static void timedLoop (uint32_t iNow);

static uint16_t getSettings();

private:
static uint16_t readSwitches();

//*****************************************************************************
// End-of-class, followed by end of ifndef
};
#endif
