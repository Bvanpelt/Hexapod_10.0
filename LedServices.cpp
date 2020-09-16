//*****************************************************************************
//  LedServices.cpp - For v2 and compatible Logic boards
//
//  This logic drives the NeoPixels of our ogic board. On
//  this board the LEDs are driven by the Teensy PWM output
//  
//  Each NeoPixel uses a 24-bit PWM pulse train, absorbing the first 24 it
//  finds and sending subsequent bits on to the next Neopixel in line. We have to
//  provide a 80+ uSec reset "Low" time, which we accomplish by invoking the
//  update function no more than once per ms.
//
//  We'll use a DMA channel to drive a channel of the Flex Timer Module (FTM),
//  running the FTM in Edge-Aligned PWM (EPWM) mode. The pulse period will be
//  preset (via MOD - CNTIN + 1) and the duty-cycle set by DMAing appropriate
//  "On Time" values.
//
//  Note: The counter has only 16 bits
//-----------------------------------------------------------------------------
// This code assumes the Teensy is little-endian !
//-----------------------------------------------------------------------------
//  SK6812 data sheet details: https://cdn-shop.adafruit.com/product-files/1138/SK6812+LED+datasheet+.pdf
//
//  Bit time: 1.25 uSec. "0" and "1" Hi-times are computed (and massaged a bit to
//  ensure solid operations)
//
//  24-bit data structure:
//  * R7 - R0, then G7 - G0, then B7 - B0
//  * Bit 7 of each color is the MSbit
//
//  Reset:
//  * 80 uSecs of LOW time
//  * Allow 24 extra bit times for each LED
//  * Note that the FTM won't generate DMA requests after sending the first
//    0%-duty-cycle bit, so we'll have to run a reset timer of some sort.
//    See "Edge-Aligned PWM (EPWM) mode" in the manual.
//
//-----------------------------------------------------------------------------
//  Total transmission time (For 10 LEDs):
//  240 data bits 
//  304 trailing bit times as reset
//  ==> 544 bits at 1.25us/bit    =  680us
//  ==> Theoretical update rate = 1,470 updates/sec
//-----------------------------------------------------------------------------
//  Color-Data array:
//  * 10 32-bit words, one for each LED, in sequence
//  * Arranged (MSB - LSB) as Flag byte, Red byte, Green byte, Blue byte
//  * The Flag byte will be used for fading, blinking, etc., requests
//
//  Timer-Data array (For 10 LEDs)
//  * 240 32-bit words (960 bytes)
//  * 10 Leds at 24 bits per led, followed by one 0% duty-cycle bit
//-----------------------------------------------------------------------------
//  Reset timing: https://blog.particle.io/heads-up-ws2812b-neopixels-are-about-to-change/
//  "4. “Old & New chips are exactly the same as in Timing, Data transmission and
//  Data structure. However, the Reset Time increased from >50us to >280us. It won’t
//  cause wrong reset while interruption, and what’s more, it supports the lower
//  frequency and inexpensive MCU. When the “Reset Time<280us”, don’t mixed up and
//  kindly please make necessary amends.”"
//*****************************************************************************
#include "LedServices.h"

#include <imxrt.h>
#include <DMAChannel.h>
#include <core_pins.h>
#include <util/atomic.h>

#include "Appldd.h"

//*****************************************************************************
//  External data and procedure references
//*****************************************************************************
void * get32ByteAlignedBuffer ( int iLength ) ;


//*****************************************************************************
//  Forward declarations
//*****************************************************************************
static void LedServices_TimedLoop ( ); // No time needed, as it's driven by am IntervalTimer

//*****************************************************************************
//  Local enumerations and constants
//
//  These are used internally only!
//*****************************************************************************
#define     DmaDataType     uint16_t

                                //---------------------------------------------
                                // PWM constants
                                //---------------------------------------------
static const float  cBitFrequency   = 800000.0;     // 800k Hz, or 1.25 uSec per bit
static const int    cOutputPin      = 3;

// FlexPWM4_2_B   3 
/*static*/ IMXRT_FLEXPWM_t * const pFlexPwm = & IMXRT_FLEXPWM4;
/*static*/ const int cFlexSubmoduleNumber   = 2;

                                //---------------------------------------------
                                // Derived PWM constants
                                //---------------------------------------------
static const int  cFlexMask = 1 << cFlexSubmoduleNumber;

                                //---------------------------------------------
                                // LED constants
                                //---------------------------------------------
static const int cBitsPerLed       = 24;
static const int cDataBits         = (cNumberOfLeds * cBitsPerLed);
static const int cResetBits        = 240;   // Per note above, 224. Upped to 240
                                            // to allow some slack
static const int cTotalBits        = (cDataBits + cResetBits);

static const int cUpdateInterval   = 41; // Led update interval in milliseconds

//*****************************************************************************
//  LED data 
//*****************************************************************************
typedef struct _LedData {
    int         mHavePendingCommandAndColor;
    uint32_t    mPendingCommandAndColor;

    int         mSweepDirection;
    uint32_t    mCurrentCommandAndColor;
    uint32_t    mCurrentColor;
    uint32_t    mPriorColor;

} LedData, *pLedData;

//*****************************************************************************
//  Local data
//*****************************************************************************
//uint32_t                    LedServices_SchedulingTimer;

static DMAChannel           mTxDma;

static IntervalTimer        mIntervalTimer;

                                //---------------------------------------------
                                // Computed (at setup) Flex PWM values
                                //---------------------------------------------
static uint32_t             mTicksPerBit;
static uint16_t             mD0HighTime;
static uint16_t             mD1HighTime;

                                //---------------------------------------------
                                // Driver-is-busy flag
                                //---------------------------------------------
static volatile int         mDriverIsBusy;

                                //---------------------------------------------
                                // Led data
                                //---------------------------------------------
static LedData              mLedData [ cNumberOfLeds ];

static volatile uint32_t    mAllLedOverride;    // Stores the commanded All-Leds value

                                //---------------------------------------------
                                // Output bit array, DMA'd to FlexPwm. Aligned
                                // to 32-byte boundary.
                                // 
                                // MAKE SURE THIS DATATYPE IS THE SAME
                                // WIDTH AS THE fLEXPWM REGISTER !!!
                                //---------------------------------------------
static DmaDataType * mTimerDataArray;

//*****************************************************************************
//  dma_complete_isr: DMA-complete interrupt handler
//
//  Note: The requisite post-data LED reset time is created by not scheduling
//        PWM data outputs any more often than once every 2ms...
//*****************************************************************************
static void dma_complete_isr() {
                                //---------------------------------------------
                                // Turn ON the cpu LED. It gets turned off in
                                // the NULL task
                                //---------------------------------------------
    digitalWriteFast(cLed_Cpu, cOn);

                                //---------------------------------------------
                                // Clear the DMA interrupt
                                //---------------------------------------------
    mTxDma.clearInterrupt();
                                //---------------------------------------------
                                // Leave the FlexPwm free-running with an VAL5
                                // count of 0 (which gives us a steady LOW output)
                                //---------------------------------------------
    pFlexPwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(cFlexMask);
    pFlexPwm->SM[cFlexSubmoduleNumber].VAL5 = 0 ;
    pFlexPwm->MCTRL |= FLEXPWM_MCTRL_LDOK(cFlexMask);

                                //---------------------------------------------
                                // Flag the driver as "not busy"
                                //---------------------------------------------
    mDriverIsBusy = 0;
}

//*****************************************************************************
//  start_tx: Start sending bits
//*****************************************************************************
static void start_tx() {
    mDriverIsBusy = 1;

    // Flush the cache to memory ( Ref the comments regarding this call in imxrt.h )
    arm_dcache_flush_delete ( mTimerDataArray, cTotalBits * sizeof( mTimerDataArray[0] ) );

    // Enable the DMA
    mTxDma.sourceBuffer( mTimerDataArray, cTotalBits * sizeof( mTimerDataArray[0] ) );
    mTxDma.disableOnCompletion();
    mTxDma.enable();
  }

//*****************************************************************************
//  LedServices_late_setup
//*****************************************************************************
void LedServices_late_setup(uint32_t iNow) {

//-----------------------------------------------------------------------------
//  Flag the driver as "not busy"
//-----------------------------------------------------------------------------
    mDriverIsBusy = 0;

//-----------------------------------------------------------------------------  
// Init our local color buffer, status', etc.
//-----------------------------------------------------------------------------  
    mAllLedOverride     = 0;
    pLedData pLed       = mLedData;
    for ( int wIndex = 0; wIndex < cNumberOfLeds; ++wIndex, ++pLed ) {
        pLed->mHavePendingCommandAndColor   = 0;
        pLed->mSweepDirection               = 0;
        pLed->mPriorColor                   = cStartup;
    }

//-----------------------------------------------------------------------------  
//  Set up the FlexPwm PWM generator
//-----------------------------------------------------------------------------
//  Note: This code is cooked out from Paul's pwm.c code's routines:
//  * The table: const struct pwm_pin_info_struct pwm_pin_info[] = {...
//    - #define M(a, b) ((((a) - 1) << 4) | (b))
//    - struct pwm_pin_info_struct {
//        uint8_t type;    // 0=no pwm, 1=flexpwm, 2=quad
//        uint8_t module;  // 0-3, 0-3
//        uint8_t channel; // 0=X, 1=A, 2=B
//        uint8_t muxval;  //
//      };
//    - This entry: { 1, M(4, 2), 2, 1},  // FlexPWM4_2_B   3  // EMC_05    
//          - type:     1
//          - module:   FlexPwm #4, Submodule 2
//          - channel:  2
//          - muxval:   1
//  * void pwm_init(void)
//  * void flexpwm_init(IMXRT_FLEXPWM_t *p)
//  * void analogWriteFrequency(uint8_t pin, float frequency)
//  * void flexpwmFrequency(IMXRT_FLEXPWM_t *p, unsigned int submodule, uint8_t channel, float frequency)
//-----------------------------------------------------------------------------
//  Per the reference manual 55.4.1.2:
//  * The waveform counter starts at the INIT register's value, rises to VAL1, and then restarts
//  * Changes from High to Low at the VAL5 time
//  * From Figure 55-3. Edge Aligned Example (INIT=VAL2=VAL4)
//-----------------------------------------------------------------------------
    uint32_t prescale   = 0;
    mTicksPerBit        = (uint32_t)((float)F_BUS_ACTUAL / cBitFrequency + 0.5f);;

    // Compute the pre-scalar value, and then limit the resulting values
    while ( ( mTicksPerBit > 65535 ) && ( prescale < 7 ) ) {
        mTicksPerBit = mTicksPerBit >> 1;
        ++prescale;;
    }
    if ( mTicksPerBit > 65535 ) mTicksPerBit = 65535;
    if ( mTicksPerBit < 2 )     mTicksPerBit = 2;

    // Compute the "high" tick-counts for both '0' and '1' bit values
    mD0HighTime        = mTicksPerBit * 40 / 125;    // 0.4 uSec
    mD1HighTime        = mTicksPerBit * 80 / 125;    // 0.8 uSec

    // Set up the Flex PWM
    // Note: We rely on Paul's initialization in flexpwm_init (in pwm.c)
    // Note: We'll start the 4/5 timers free-running with a 0 value, which will
    //       ... give us a LOW output
    pFlexPwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(cFlexMask);

    pFlexPwm->SM[cFlexSubmoduleNumber].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescale);

    pFlexPwm->SM[cFlexSubmoduleNumber].INIT = 0;
    pFlexPwm->SM[cFlexSubmoduleNumber].VAL0 = 0;
    pFlexPwm->SM[cFlexSubmoduleNumber].VAL2 = 0;
    pFlexPwm->SM[cFlexSubmoduleNumber].VAL4 = 0;

    pFlexPwm->SM[cFlexSubmoduleNumber].VAL1    = mTicksPerBit - 1;
    pFlexPwm->SM[cFlexSubmoduleNumber].VAL3    = 0; // 2/3 timer, unused
    pFlexPwm->SM[cFlexSubmoduleNumber].VAL5    = 0; // 4/5 timer. Creates initial steady Low output

    pFlexPwm->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(cFlexMask);

    pFlexPwm->MCTRL |= FLEXPWM_MCTRL_LDOK(cFlexMask);

    // Configure the path between the PWM output and the physical pin
    uint8_t     wMuxVal = 1;    // (From the table in pwm.c...)
    *(portConfigRegister(cOutputPin)) = wMuxVal;

//-----------------------------------------------------------------------------
//   DMA setup
//-----------------------------------------------------------------------------  
    mTxDma.begin(true);
    mTxDma.destination( pFlexPwm->SM[cFlexSubmoduleNumber].VAL5 );
    mTxDma.interruptAtCompletion();
    mTxDma.attachInterrupt(dma_complete_isr);
    mTxDma.triggerAtHardwareEvent( DMAMUX_SOURCE_FLEXPWM4_WRITE2 );

    // Enable DMA requests from the Flex PWM. Enabling Interrupts as well is not needed.
    pFlexPwm->SM[cFlexSubmoduleNumber].DMAEN |= FLEXPWM_SMDMAEN_VALDE;

//-----------------------------------------------------------------------------
// Allocate our bit-image DMA buffer, aligning it to a 32-byte boundary:
// https://forum.pjrc.com/threads/57326-T4-0-Memory-trying-to-make-sense-of-the-different-regions?p=220632&viewfull=1#post220632
//-----------------------------------------------------------------------------
    mTimerDataArray = ( DmaDataType * ) get32ByteAlignedBuffer(
                            cTotalBits * sizeof ( mTimerDataArray [0] ) );

//-----------------------------------------------------------------------------
// Init the Reset-data portion of the Timer Data Array. This will remain
// unchanged while we run
//-----------------------------------------------------------------------------
    DmaDataType * pTimerDataArray = & mTimerDataArray [ cDataBits ];
    for (uint32_t wCount = 0; wCount < cResetBits; ++wCount, ++pTimerDataArray) {
        *pTimerDataArray = 0;   // 0 VAL5 value should give 0% duty cycle
    }

//-----------------------------------------------------------------------------
//  Arrange for our initial multi-color single-sweep
//-----------------------------------------------------------------------------
    LedServices_ledRequest( 0, cSweepToThisThenBlack | cRedMask );
    LedServices_ledRequest( 1, cSweepToThisThenBlack | cGreenMask );
    LedServices_ledRequest( 2, cSweepToThisThenBlack | cBlueMask );
    LedServices_ledRequest( 3, cSweepToThisThenBlack | cCyanMask );
    LedServices_ledRequest( 4, cSweepToThisThenBlack | cMagentaMask );
    LedServices_ledRequest( 5, cSweepToThisThenBlack | cYellowMask );

//-----------------------------------------------------------------------------
//  Start the Led update process, AND AFTERWARD start the Interval Timer. We'll
//  use hthe Intreval Timer for better visual timing accuracy
//-----------------------------------------------------------------------------
    LedServices_TimedLoop ( );

    mIntervalTimer.begin ( LedServices_TimedLoop, cUpdateInterval * 1000 );
}

//*****************************************************************************
//  constructTimerDataArray
//*****************************************************************************
inline void constructTimerDataArray() {
    pLedData pLed               = mLedData;
    uint16_t  *pTimerDataArray  = mTimerDataArray;

//-----------------------------------------------------------------------------
// Build the LED-data portion of the Timer Data Array. The reset portion is
// initialized in our setup code
//-----------------------------------------------------------------------------
    for (int wLedNumber = 0; wLedNumber < cNumberOfLeds; ++wLedNumber, ++pLed ) {
        //---------------------------------------------------------------------
        // Skip updating this LED's bits if it's value hasn't changed
        //---------------------------------------------------------------------
        if ( ( pLed->mCurrentColor == pLed->mPriorColor ) &&
             ( pLed->mPriorColor != cStartup )            ){
            pTimerDataArray += cBitsPerLed;
            continue;
        }
        pLed->mPriorColor = pLed->mCurrentColor;

        //---------------------------------------------------------------------
        // Update the image bits for this led
        //---------------------------------------------------------------------
        uint32_t  wCurrentLedColors = pLed->mCurrentColor;
        uint32_t  wCurrentBitMask = 0x00800000;
// Leave this here - it's helpful for debugging !!!
//if ( wLedNumber == 5 ) Serial1.printf("LastPwr Led update: %08X\n", wCurrentLedColors );
        for ( uint32_t wBitNumber = 0;
              wBitNumber < cBitsPerLed;
              ++wBitNumber, wCurrentBitMask >>= 1, ++pTimerDataArray ) {
            *pTimerDataArray = mD0HighTime;
            if (wCurrentLedColors & wCurrentBitMask) *pTimerDataArray = mD1HighTime;
        }
    }
}

//*****************************************************************************
//  sweepUp: Sweep a Led up in intensity as needed
//*****************************************************************************
inline void sweepUp( pLedData pLed ) {
    pEndian pCommandedColor = ( pEndian ) & pLed->mCurrentCommandAndColor;
    pEndian pCurrentColor   = ( pEndian ) & pLed->mCurrentColor;

    if ( pCommandedColor->mByte[0] ) {
        if ( pCommandedColor->mByte[0] > pCurrentColor->mByte[0] ) {
             pCurrentColor->mByte[0] += cSweepDelta;
        } else {
            pLed->mSweepDirection = -1;
        }
    }

    if ( pCommandedColor->mByte[1] ) {
        if ( pCommandedColor->mByte[1] > pCurrentColor->mByte[1] ) {
             pCurrentColor->mByte[1] += cSweepDelta;
        } else {
            pLed->mSweepDirection = -1;
        }
    }

    if ( pCommandedColor->mByte[2] ) {
        if ( pCommandedColor->mByte[2] > pCurrentColor->mByte[2] ) {
             pCurrentColor->mByte[2] += cSweepDelta;
        } else {
            pLed->mSweepDirection = -1;
        }
    }
}

//*****************************************************************************
//  sweepDown: Sweep a Led down in intensity as needed
//*****************************************************************************
inline void sweepDown( pLedData pLed ) {
    pEndian pCommandedColor = ( pEndian ) & pLed->mCurrentCommandAndColor;
    pEndian pCurrentColor   = ( pEndian ) & pLed->mCurrentColor;

    char wLeastValue = cDimValue;
    if ( ( pCommandedColor->mUnsignedWord & cCommandMask ) == cSweepToThisThenBlack ) {
        wLeastValue = 0;
    }

    if ( pCommandedColor->mByte[0] ) {
        pCurrentColor->mByte[0] -= cSweepDelta;
        if ( pCurrentColor->mByte[0] <= wLeastValue ) {
            pCurrentColor->mByte[0] = wLeastValue;
            pLed->mSweepDirection = 0;
        }
    }

    if ( pCommandedColor->mByte[1] ) {
        pCurrentColor->mByte[1] -= cSweepDelta;
        if ( pCurrentColor->mByte[1] <= wLeastValue ) {
            pCurrentColor->mByte[1] = wLeastValue;
            pLed->mSweepDirection = 0;
        }
    }

    if ( pCommandedColor->mByte[2] ) {
        pCurrentColor->mByte[2] -= cSweepDelta;
        if ( pCurrentColor->mByte[2] <= wLeastValue ) {
            pCurrentColor->mByte[2] = wLeastValue;
            pLed->mSweepDirection = 0;
        }
    }
}

//*****************************************************************************
//  limitByteIntensity
//  limitWordIntensity
//*****************************************************************************
inline void limitByteIntensity(uint8_t iCommandCode, uint8_t *pIntensity) {
    if ( *pIntensity > cLed_MaxIntensity ) *pIntensity = cLed_MaxIntensity;
    if ((iCommandCode == (cSetThisColorDim >> 24)) && (*pIntensity)) *pIntensity = 1;
}
//-----------------------------------------------------------------------------
inline uint32_t limitWordIntensity(uint32_t iData) {
    uint32_t    wCommandAndColor = iData;
    uint8_t     wCommandCode = ((Endian *) & wCommandAndColor)->mByte[3];
    limitByteIntensity( wCommandCode, & ((Endian *) & wCommandAndColor)->mByte[0] );
    limitByteIntensity( wCommandCode, & ((Endian *) & wCommandAndColor)->mByte[1] );
    limitByteIntensity( wCommandCode, & ((Endian *) & wCommandAndColor)->mByte[2] );
    return wCommandAndColor;
}

//*****************************************************************************
//  LedServices_allLeds
//
//  Override the individial LED settings and drive ALL leds per this command.
//  * Whenever the command-mask is non-zero this command will be accepted
//  * When the command-mask is set to 0 the normally-requested led services 
//    will be restored.
//  * No timed-services (e.g. sweeps) will be accepted
//  * If the led driver is still busy when this command is presented the
//    command will be ignored
//*****************************************************************************
void LedServices_AllLeds(uint32_t iCommandAndColor) {
    if (mDriverIsBusy)   return;

    mAllLedOverride = limitWordIntensity(iCommandAndColor);
    
    if ( mAllLedOverride ) {
        pLedData pLed = mLedData;
        for ( int wCount = 0; wCount < cNumberOfLeds; ++wCount, ++pLed ) {
            pLed->mCurrentColor = mAllLedOverride;
        }

        constructTimerDataArray();
        start_tx();
    }
}

//*****************************************************************************
//  LedServices_ledRequest: Request LED services
//
//  Request a change to a particular LED
//
//  Returns: Nothing
//*****************************************************************************
void LedServices_ledRequest(int iLedId, uint32_t iCommandAndColor) {
    if ( ( iLedId < 0 ) || ( iLedId >= cNumberOfLeds ) ) return;
    mLedData [ iLedId ].mHavePendingCommandAndColor = 1;
    mLedData [ iLedId ].mPendingCommandAndColor = limitWordIntensity ( iCommandAndColor );
}

//*****************************************************************************
//  LedServices_TimedLoop
//
//  Process new and existing-and-underway commands. If we need to do so, start
//  appropriate Led I/O
//
//-----------------------------------------------------------------------------
// Note: This routine is called at init (for initial start of Led updates), and
//       from our IntervalTimer.
//*****************************************************************************
static void LedServices_TimedLoop( ) {
//-----------------------------------------------------------------------------
//  This is actually an ISR, so we'll turn on the CPU Led
//-----------------------------------------------------------------------------
    digitalWriteFast(cLed_Cpu, cOn);

//-----------------------------------------------------------------------------
//  If it's not time for an update-check, or the driver is busy (!!!!!), just return
//-----------------------------------------------------------------------------
    if ( ( mDriverIsBusy ) || ( mAllLedOverride ) ) {
        return;
    }
    
//-----------------------------------------------------------------------------
//  Examine data for each Led, looking for required updates
//-----------------------------------------------------------------------------
    int wIoIsNeededFlag = false;
    pLedData pLed       = mLedData;
    for ( int wIndex = 0; wIndex < cNumberOfLeds; ++wIndex, ++pLed ) {
        //---------------------------------------------------------------------
        // Handle up-sweeps
        //---------------------------------------------------------------------
        if ( pLed->mSweepDirection == +1 ) {
            sweepUp( pLed );
            wIoIsNeededFlag = true;
            continue;

        //---------------------------------------------------------------------
        // Handle down-sweeps
        //---------------------------------------------------------------------
        } else if (pLed->mSweepDirection == -1 ) {
            sweepDown( pLed );
            wIoIsNeededFlag = true;
            continue;
        }

        //---------------------------------------------------------------------
        // We have no currently active sweep !!!
        //---------------------------------------------------------------------

        //---------------------------------------------------------------------
        // If we have no pending commands:
        // * If the prior sweep was to be continuous, arrange to start another
        //   up-sweep. This lets us stop a continuous sweep...
        // * Either way we don't need to do anything further.
        //---------------------------------------------------------------------
        if ( ! pLed->mHavePendingCommandAndColor ) {
            if ( ( pLed->mCurrentCommandAndColor & cCommandMask ) == cContinuousSweep ) {
                pLed->mHavePendingCommandAndColor = true;
                pLed->mPendingCommandAndColor = pLed->mCurrentCommandAndColor;
                wIoIsNeededFlag = true;
            }
            continue;
        }

        //---------------------------------------------------------------------
        // Implement the pending command as reqd
        //---------------------------------------------------------------------
        wIoIsNeededFlag = true;
        pLed->mHavePendingCommandAndColor = false;
        pLed->mSweepDirection = 0;
        pLed->mCurrentCommandAndColor = pLed->mPendingCommandAndColor;

        // we're done If we're to just set this color
        uint32_t mCurrentCommand = pLed->mCurrentCommandAndColor & cCommandMask;
        if ( mCurrentCommand == cSetThisColor ) continue;

        // Set to sweep UP if we're to do any kind of a sweep
        if ( mCurrentCommand != cSetThisColorDim) pLed->mSweepDirection = 1;

        // Set (only the commanded colors) to DIM
        pLed->mCurrentColor = 0;

        if ( ((pEndian) & pLed->mCurrentCommandAndColor)->mByte[0] ) {
            ((pEndian) & pLed->mCurrentColor)->mByte[0] = cDimValue;
        }

        if ( ((pEndian) & pLed->mCurrentCommandAndColor)->mByte[1] ) {
            ((pEndian) & pLed->mCurrentColor)->mByte[1] = cDimValue;
        }

        if ( ((pEndian) & pLed->mCurrentCommandAndColor)->mByte[2] ) {
            ((pEndian) & pLed->mCurrentColor)->mByte[2] = cDimValue;
        }
    }


//-----------------------------------------------------------------------------
//  Start I/O as needed
//-----------------------------------------------------------------------------
    if ( wIoIsNeededFlag ) {
        constructTimerDataArray();
        start_tx();
    }
}
