//*****************************************************************************
//  PowerControl.cpp: Implemented as a Singleton class
//
//  Class implementations for the PowerControl class
//*****************************************************************************
#include "PowerControl.h"
#include "LedServices.h"
#include "FaultsAndAlerts.h"

//*****************************************************************************
//  External data and procedure references
//*****************************************************************************
extern char * createDateTimeString() ;

//*****************************************************************************
//  Voltage-->Led-State and margins
//*****************************************************************************
struct VoltageState {
    float       mLowerBoundary;
    float       mHigherBoundary;
    float       mMargin;            // Percentage hystersis
    uint32_t    mLedState;
};

struct VoltageState mVStates [] = {
    { 0.00,  10.20, 5, cContinuousSweep    | cRedMask},     
    {10.20,  10.85, 5, cSweepToThisThenDim | cRedMask},
    {10.85,  11.70, 5, cSweepToThisThenDim | cYellowMask}, 
    {11.70,  12.80, 5, cSweepToThisThenDim | cGreenMask},
    {12.80,  14.00, 5, cContinuousSweep    | cGreenMask},
    {14.00, 100.00, 5, cContinuousSweep    | cRedMask}
};

//*****************************************************************************
//  Static member instantiation & initialization
//*****************************************************************************
uint32_t     PowerControl::mSchedulingTimer = 0;

uint32_t     PowerControl::mActivityTimer;
uint32_t     PowerControl::mButtonSenseTimer;
uint32_t     PowerControl::mVoltageSenseTimer;
int          PowerControl::mSwitchState;
int          PowerControl::mPriorSwitchState;
uint8_t      PowerControl::mSwitchTransitionCounter;
uint32_t     PowerControl::mLastLedSetting;
float        PowerControl::mLastBatteryVoltage = 0;
int          PowerControl::mLastRclawSetting = -1;

                                //---------------------------------------------
                                //  Float constants
                                //---------------------------------------------
float        PowerControl::cVoltageAdjFactor = 46.6501; // Value for HexaTeen 10.0 board
float        PowerControl::cMinimumVoltage   = 10.0;
float        PowerControl::cMaximumVoltage   = 14.00;

//*****************************************************************************
//  early_setup
//*****************************************************************************
void PowerControl::early_setup(uint32_t iNow) {
                                //---------------------------------------------
                                // First up!
                                //
                                // Set up the control-control output
                                // and set its level so we hold the power mosfet
                                // on after the power button is released.
                                //---------------------------------------------
    pinMode(cPowerControl, OUTPUT);
    digitalWrite(cPowerControl, cHigh);
}

//*****************************************************************************
//  late_setup
//*****************************************************************************
void PowerControl::late_setup(uint32_t iNow) {
                                //---------------------------------------------
                                // Set up the button-sense pin as a digital input
                                //---------------------------------------------
    pinMode(cButtonSense, INPUT);

                                //---------------------------------------------
                                // Init our timers. counters, & states
                                //---------------------------------------------
    mButtonSenseTimer  = iNow + cButtonSenseInterval;
    mVoltageSenseTimer = iNow + cVoltageSenseInterval;

    mSwitchState                = cSwitch_Unknown;
    mPriorSwitchState           = cSwitch_Unknown;
    mSwitchTransitionCounter    = 0;
    mActivityTimer              = iNow;
}

//*****************************************************************************
//  handleSwitchStateChange
//
//  Note: Sensing the switch returns "1" if it's closed.
//*****************************************************************************
void PowerControl::handleSwitchStateChange(uint32_t iNow, int iPriorState, int iNewState) {
    const char * pPriorStateText = "Unknown";
    if (iPriorState == 0) {
        pPriorStateText = "Open";
    } else if (iPriorState == 1) {
        pPriorStateText = "Closed";
    }
    
    const char * pNewStateText = "Unknown";
    if (iNewState == 0) {
        pNewStateText = "Open";
    } else if (iNewState == 1) {
        pNewStateText = "Closed";
    }
    
    Serial1.print  ( createDateTimeString() );
    Serial1.print  (": Pwr-Switch state change: Was [");
    Serial1.print  (pPriorStateText);
    Serial1.print  ("], now [");
    Serial1.print  (pNewStateText);
    Serial1.println("]");

    if ((iPriorState == 0 /* Open */ ) && (iNewState == 1 /* Closed */ )) {
        setPowerOff();
 //       FaultsAndAlerts::fastFault("Power shutdown via switch!", "P");
    }
}

//*****************************************************************************
//  getLastBatteryVoltage
//*****************************************************************************
float PowerControl::getLastBatteryVoltage() {
    return mLastBatteryVoltage;
}

//*****************************************************************************
//  setPowerOff
//*****************************************************************************
void PowerControl::setPowerOff() {
    digitalWrite(cPowerControl, cLow);
    while (1) { }
}

//*****************************************************************************
//  setActivity
//
//  Used to restart our Activity timer. If the timer expires we "Fault" and
//  shut down
//*****************************************************************************
void PowerControl::setActivity( uint32_t iNow ) {
    mActivityTimer = iNow;
}

//*****************************************************************************
//  timedLoop
//*****************************************************************************
void PowerControl::timedLoop (uint32_t iNow) {
    mSchedulingTimer = iNow + 101;

//-----------------------------------------------------------------------------
//  Perform Activity timing
//
//  If we have an activity fault we'll set the activity timer to -1. We'll use 
//  this magic value to stop us from repeatedly faulting.
//-----------------------------------------------------------------------------
    if ( ( mActivityTimer != 0xFFFFFFFF ) &&
         ( ( mActivityTimer + cActivityTimerInterval ) < iNow ) ) {

        FaultsAndAlerts::fault("Inactivity!", "I");
        mActivityTimer = 0xFFFFFFFF; // So we don't repeatedly Fault
    }

//-----------------------------------------------------------------------------
//  Perform power-switch sensing
//-----------------------------------------------------------------------------
    if (iNow >= mButtonSenseTimer) {
                                //---------------------------------------------
                                // Restart our sample-interval timer
                                //---------------------------------------------
        mButtonSenseTimer = iNow + cButtonSenseInterval;

                                //---------------------------------------------
                                // Read the switch. De-bounce its state.
                                //---------------------------------------------
        int wSwitchState = digitalRead(cButtonSense);
        if (wSwitchState != mPriorSwitchState) {
            mPriorSwitchState = wSwitchState;
            mSwitchTransitionCounter = 0;
        }
        ++mSwitchTransitionCounter;
                                //---------------------------------------------
                                // If the switch is in a steady state, check to
                                // see if the steady state is different from the
                                // prior steady-state value. If we have a new
                                // steady-state, handle the official switch-state
                                // transition
                                //---------------------------------------------
        if (mSwitchTransitionCounter >= cSwitchDebounceSamples) {
            --mSwitchTransitionCounter;
            if (wSwitchState != mSwitchState) {
                mSwitchTransitionCounter = 0;
                handleSwitchStateChange(iNow, mSwitchState, wSwitchState);
                mSwitchState = wSwitchState;
            }
        }
    }

//-----------------------------------------------------------------------------
//  Perform voltage sensing
//-----------------------------------------------------------------------------
    if (iNow >= mVoltageSenseTimer) {
                                //---------------------------------------------
                                // Return if its not time to do a voltage check
                                //---------------------------------------------
        mVoltageSenseTimer = iNow + cVoltageSenseInterval;
                                //---------------------------------------------
                                // Determine the battery voltage
                                //---------------------------------------------
        uint16_t wSensorValueAsUShort = analogRead(cVSensePin);
        float wWork = wSensorValueAsUShort;
        mLastBatteryVoltage = wWork / cVoltageAdjFactor;
        // Serial1.printf("Batt sense: %f, computed v: %f\n", wWork, mLastBatteryVoltage);
                                //---------------------------------------------
                                // Update the Power led based on the batt voltage
                                // TODO: Use margin values to de-bounce!
                                //---------------------------------------------
        struct VoltageState *pEntry = mVStates;
        int wTableCount = sizeof(mVStates) / sizeof(mVStates[0]);
        for ( int wIndex = 0; wIndex < wTableCount; ++wIndex, ++pEntry) {
            if ((mLastBatteryVoltage >= pEntry->mLowerBoundary)  &&
                (mLastBatteryVoltage <= pEntry->mHigherBoundary) ){
                if (pEntry->mLedState != mLastLedSetting) {
                    mLastLedSetting = pEntry->mLedState;
                    LedServices_ledRequest(cLed_Power, mLastLedSetting);
// Serial1.printf("Batt Led: wIndex [%d] wTableCount [%d] pEntry->mLedState [%08X] mLastLedSetting [%08X] Set Led to [%08X]\n",
//        wIndex, wTableCount, pEntry->mLedState, mLastLedSetting, mLastLedSetting);
                }
                break;
            }
        }
                                //---------------------------------------------
                                // Fault if the batt voltage is too low or too high
                                //---------------------------------------------
        if ((mLastBatteryVoltage < cMinimumVoltage) ||
            (mLastBatteryVoltage > cMaximumVoltage) ){
            FaultsAndAlerts::fault("Bad battery voltage!", "V");
        }
    }

}