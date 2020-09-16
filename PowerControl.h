//*****************************************************************************
//  PowerControl.h: Implemented as a Singleton class
//
//  Class definition for our PowerControl class
//*****************************************************************************
#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#include <Arduino.h>
#include <pins_arduino.h>

//*****************************************************************************
//*****************************************************************************
//  class PowerControl
//*****************************************************************************
//*****************************************************************************
class PowerControl {

//*****************************************************************************
//  Constants and enumerations
//*****************************************************************************
enum {
    cLow                =  0,       // Digital pin: Low
    cHigh               =  1,       // Digital pin: High
    
    cVSensePin          = A2,       // Analog:  Voltage-sense pin (see pins_arduino.h)
    cButtonSense        = 21,       // Digital: Button sense
    cPowerControl       = 20,       // Digital: Power control
    
    cButtonSenseInterval    = 200,  // Time-to-sense-the-button  interval (in ms)
    cVoltageSenseInterval   = 2000, // Time-to-sense-the-voltage interval (in ms)
    cSwitchDebounceSamples  = 4,
    cActivityTimerInterval  = 15 * 60 * 1000,

    cSwitch_Unknown     = -1,
    cSwitch_Closed      =  0,    // Startup or forced
    cSwitch_Open        =  1
};

// Note: cVoltageAdjFactor, cMinimumVoltage, and cMaximumVoltage, as floats,
//       are defined in PowerControl.cpp

//*****************************************************************************
//  Member data
//*****************************************************************************
public:
static uint32_t     mSchedulingTimer;

private:
static uint32_t     mButtonSenseTimer;
static uint32_t     mVoltageSenseTimer;
static uint32_t     mActivityTimer;

static int          mSwitchState;
static int          mPriorSwitchState;
static uint8_t      mSwitchTransitionCounter;

// Note: Values for cVoltageAdjFactor and cMinimumVoltage are defined in PowerControl.cpp
static float        cVoltageAdjFactor;
static float        cMinimumVoltage;
static float        cMaximumVoltage;

static float        mLastBatteryVoltage;
static uint32_t     mLastLedSetting;

static int          mLastRclawSetting;

//*****************************************************************************
//  Member functions
//*****************************************************************************
public:
static void  early_setup(uint32_t iNow);
static void  late_setup(uint32_t iNow);
static void  timedLoop (uint32_t iNow);
static void  setPowerOff();
static float getLastBatteryVoltage();
static void  setActivity( uint32_t iNow );

protected:
static void handleSwitchStateChange(uint32_t iNow, int iPriorState, int iNewState);

//*****************************************************************************
// End-of-class, followed by end of ifndef
};
#endif
