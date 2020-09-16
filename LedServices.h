//*****************************************************************************
//  LedServices.h
//*****************************************************************************
#ifndef __LED_SERVICES_H__
#define __LED_SERVICES_H__

#include "Appldd.h"

//*****************************************************************************
//  Public LED enumerations
//*****************************************************************************

                                //---------------------------------------------
                                // Color definitions
                                //---------------------------------------------
const uint32_t   cBlackMask    = 0x000000;

const uint32_t   cRedMask      = 0xFF0000;
const uint32_t   cGreenMask    = 0x00FF00;
const uint32_t   cBlueMask     = 0x0000FF;

const uint32_t   cYellowMask   = cRedMask   | cGreenMask;
const uint32_t   cCyanMask     = cGreenMask | cBlueMask;
const uint32_t   cMagentaMask  = cRedMask   | cBlueMask;

                                //---------------------------------------------
                                // Commands: These are supplied by our callers
                                //           in the 0xFF000000 bits. Don't use
                                //           a command of 0...
                                //---------------------------------------------
const uint32_t   cCommandMask            = 0xFF << 24;
const uint32_t   cSetThisColor           = 0x01 << 24;
const uint32_t   cSetThisColorDim        = 0x02 << 24;
const uint32_t   cSweepToThisThenDim     = 0x03 << 24;
const uint32_t   cSweepToThisThenBlack   = 0x04 << 24;
const uint32_t   cContinuousSweep        = 0x05 << 24;
const uint32_t   cStartup                = 0xFF << 24; // Internal use (Init flag)

                                //---------------------------------------------
                                // LED identifiers
                                //---------------------------------------------
const int   cNumberOfLeds           = 6;    // "7" stops the occassional Blue PWR led issue!
                                            // ... I don't know why!

const int   cLed_Servo              = 0;
const int   cLed_XBee               = 1;
const int   cLed_Mpu                = 2;
const int   cLed_Unused1            = 3;
const int   cLed_Unused2            = 4;
const int   cLed_Power              = 5;
                                //---------------------------------------------
                                // Max color intensity. Remember - the brighter
                                // the intensity value the slower the sweep
                                // becomes!
                                //---------------------------------------------
const uint8_t   cLed_MaxIntensity   = 0x1F;
                                //---------------------------------------------
                                // Color-intensity-sweep parameters
                                //---------------------------------------------
const uint8_t   cSweepDelta         =  1;   // Color change per LED update. If you want to
                                            // ... change this you'll need to re-code the sweep logic
const uint8_t   cDimValue           =  1;   // When moving to DIM, stop at this value

//*****************************************************************************
//  Globally-available functions
//*****************************************************************************
void LedServices_late_setup(uint32_t iNow);

void LedServices_AllLeds(uint32_t iCommandAndColor);
void LedServices_ledRequest(int iLedId, uint32_t iCommandAndColor);

//*****************************************************************************
#endif
