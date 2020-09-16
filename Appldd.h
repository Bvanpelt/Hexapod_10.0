//*****************************************************************************
//  appldd.h: HexaTeen 10.0 Application Data Definitions
//
//*****************************************************************************
#ifndef __HEXATEEN_10_0_APPLDD_H__
#define __HEXATEEN_10_0_APPLDD_H__

#include <Arduino.h>
#include <pins_arduino.h>

//*****************************************************************************
//  Interrupt priorities
//*****************************************************************************
enum {
    cLedPitPriority     = 200
};

//*****************************************************************************
//  Scheduler constants
//*****************************************************************************
enum {
    cDontSchedule   = 0xFFFFFFFF    // Use this value for Scheduling Timer to
                                    // ... stop scheduling
};

//*****************************************************************************
//  CPU Led definitions
//
//  Per the Teensy 3.5 schematic, the onboard led (used as the cpu led) is
//  driven by D13, aka PTC5
//*****************************************************************************
enum {
    cOff            = 0,    // LED: Off
    cOn             = 1,    // LED: On

    cLed_Cpu        = 13,   // Orange; On Teensy
};

//*****************************************************************************
//  Endian structures. Teensy is little-endian
//*****************************************************************************
typedef union {
    uint16_t  mUnsignedShort;
    uint16_t  mSignedShort;
    uint8_t   mByte[2];     // LSB is [0]
} Endian16;

typedef union {
    uint32_t  mUnsignedWord;
    int32_t   mSignedWord;
    uint16_t  mShort[2];    // LSShort is [0], MSShort is [1]
    uint8_t   mByte[4];     // LSB is [0]
} Endian, * pEndian;

typedef union {
    uint64_t mULong;
    int64_t  mLong;
    Endian   mWord[2];       // LSW is [0], MSW is [1]
} Endian64;

//*****************************************************************************
#endif
