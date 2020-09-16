//*****************************************************************************
//  CoreProgram.ino
//
//  This is the core program of the HexaTeen 10.0 board and application.
//
//  We'll try to minimize the amount of code here, and rely on function-specific
//  modules for the details.
//*****************************************************************************
#include "arduino.h"
#include "pins_arduino.h"
#include <TimeLib.h>

#include "PowerControl.h"
#include "FaultsAndAlerts.h"
#include "LedServices.h"
#include "DipSwitches.h"
#include "Ax12Driver.h"
#include "Ax12Cerebellum.h"

//*****************************************************************************
//  Globally-available data
//*****************************************************************************
char gConsoleRxChar = 0;

//*****************************************************************************
//  Local data
//*****************************************************************************
static uint32_t mLastNow ;
static uint32_t mLogStatusSchedulingTimer;

//*****************************************************************************
//  get32ByteAlignedBuffer
//
//  Used for DMA. See "arm_dcache_flush_delete" et al, at the end of imxrt.h
//*****************************************************************************
void * get32ByteAlignedBuffer ( int iLength ) {
    // Round the supplied length up to a multiple of 32 bytes, matching the cpu's
    // cache-line length
    int wLength =  ( iLength + 31 ) & ( ( uint32_t ) ~ 0x1F );

    // Allocate the computed length plus enough extra bytes as needed so we can
    // ensure the allocated buffer starts on a cache-line boundary
    uint32_t pAllocation = ( uint32_t ) malloc ( wLength + 31 );
    pAllocation = ( pAllocation + 31 ) & ( ( uint32_t ) ~ 0x1F );
    
    return ( void * ) pAllocation;
}

//*****************************************************************************
//  Real-Time-Clock functions
//
//  Note: https://forum.pjrc.com/threads/60317-How-to-access-the-internal-RTC-in-a-Teensy-4-0
//*****************************************************************************
time_t getTeensy3Time() { return Teensy3Clock.get(); }

char gDateTimeString[100];

void intToAscii( int iNumber, char *& oString, int iMinCharacters ) {
    int wMsb = iNumber / 10;
    int wLsb = iNumber % 10;
    if ( ( iMinCharacters > 1 ) || ( wMsb ) ) {
        *oString++ = wMsb + '0';
    }
    *oString++ = wLsb + '0';
}

char * createDateTimeString() {
    char * oPointer = gDateTimeString;
    intToAscii ( month(),  oPointer, 1);
    *oPointer++ = '/';
    intToAscii ( day(),    oPointer, 1);
    *oPointer++ = '/';
    intToAscii ( year() - 2000,   oPointer, 1);
    *oPointer++ = ' ';
    intToAscii ( hour(),   oPointer, 2);
    *oPointer++ = ':';
    intToAscii ( minute(), oPointer, 2);
    *oPointer++ = ':';
    intToAscii ( second(), oPointer, 2);
    *oPointer++ = 0;
    return gDateTimeString;
}

//*****************************************************************************
//  checkAndRun: Inline CPU distribution
//*****************************************************************************
inline void checkAndRun (
                    const uint32_t & iNow,
                    const uint32_t & iTimer,
                    void (*iMethod) ( uint32_t ) ) {
    if ( ( iTimer != cDontSchedule ) && ( iTimer <= iNow ) ) {
        digitalWriteFast(cLed_Cpu, cOn);
        (*iMethod) ( iNow );
        digitalWriteFast(cLed_Cpu, cOff);
    } 
}

//*****************************************************************************
//  logStatus_TimedLoop: Log system status every few seconds
//*****************************************************************************
static uint32_t mLogInterval = 5000;
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void logStatus_TimedLoop ( uint32_t iNow ) {
    mLogStatusSchedulingTimer = iNow + mLogInterval;

    //-------------------------------------------------------------------------
    //  Display the system-status line
    //-------------------------------------------------------------------------
    char wMpuStatus [100];
    strcpy(wMpuStatus, "---");
 // if ( ! MpuServices::isFaulted()) sprintf(wMpuStatus, "%.1f°", MpuServices::getCurrentY());

    char wFaultChar = '-';
    if ( FaultsAndAlerts::isFaulted() ) wFaultChar = 'Y';

    int wHealthyServoCount = Ax12Cerebellum::getHealthyServoCount();

    char * pTimestampText = createDateTimeString();

    Serial1.printf("%s: Batt [%0.2fv] Fault [%c], Servos [%d] MPU [%s], DIPS [%d]\n",
          pTimestampText,
          PowerControl::getLastBatteryVoltage(),
          wFaultChar,
          wHealthyServoCount,
          wMpuStatus,
          DipSwitches::getSettings()
          );

    //-------------------------------------------------------------------------
    //  If e have un-responsive servos, list them
    //-------------------------------------------------------------------------
    if ( wHealthyServoCount < Ax12Cerebellum::cNumberOfServos ) {
        Serial1.printf("%s: Unresponsive Servo(s): ", pTimestampText );
        for ( int wIndex = 0; wIndex < Ax12Cerebellum::cNumberOfServos; ++wIndex ) {
            if ( ! Ax12Cerebellum::isServoHealthy( wIndex ) ) {
                Serial1.printf("%d ", Ax12Cerebellum::getRealServoId( wIndex ) );
            }
        }
        Serial1.printf("\n");
    }

    //-------------------------------------------------------------------------
    //  If Dip #1 is on, display detailed servo-status
    //-------------------------------------------------------------------------
    if ( DipSwitches::getSettings() & 0x01 ) {
        char wText [ 100 ];
        int wIndex, wColumn, wRealId, wPosition, wPollCount, wPollFailureCount;
        Serial1.printf("Servo Position   Fails/Polls  |  Servo Position   Fails/Polls  |  Servo Position   Fails/Polls\n");
        for ( wIndex = 0; wIndex < Ax12Cerebellum::cNumberOfServos; ) {
            for ( wColumn = 0; wColumn < 3 ; ++wColumn, ++wIndex ) {
                if ( wColumn ) Serial1.printf("  |  ");
                wRealId = Ax12Cerebellum::getRealServoId( wIndex );
                Ax12Cerebellum::getServoStatus( wIndex, wPosition, wPollCount, wPollFailureCount );
                sprintf( wText, "%d%%/%d", wPollFailureCount * 100 / wPollCount, wPollCount );
                Serial1.printf("  %2d   %4d %16s", wRealId, wPosition, wText );
            }
            Serial1.printf("\n");
        }
        Serial1.printf("\n");
    }
}

//*****************************************************************************
//  Setup
//*****************************************************************************
void setup() {
    pinMode(cLed_Cpu, OUTPUT);
    digitalWriteFast(cLed_Cpu, cOn);

    mLastNow                    = 0;
    mLogStatusSchedulingTimer   = 0;

    uint32_t wNow = millis();
                                //---------------------------------------------
                                // Perform any needed early-setup steps. and wait
                                // a moment for things to stabilize
                                //---------------------------------------------
    PowerControl::early_setup(wNow);    // First! So we hold the power on!

    while ( ! Serial1) delay(100);
    Serial1.begin(  115200    /* was 38400 */ );

//Logger::earlySetup(wNow);

    delay(100);
                                //---------------------------------------------
                                // Sync to the SRTC clock
                                //---------------------------------------------
   setSyncProvider(getTeensy3Time);

                                //---------------------------------------------
                                // Log our startup
                                //---------------------------------------------
    Serial1.printf("\n\n");

    Serial1.printf("%s: *** HexaTeen 10.0 initialization ***\n", createDateTimeString() );
      
                                //---------------------------------------------
                                // Perform normal (late) setup as needed
                                //---------------------------------------------
    LedServices_late_setup      ( wNow );
//  MpuServices::late_setup     ( wNow );
    FaultsAndAlerts::late_setup ( wNow );
    PowerControl::late_setup    ( wNow );
    DipSwitches::late_setup     ( wNow );
    Ax12Driver::late_setup      ( wNow );
    Ax12Cerebellum::late_setup  ( wNow );

                                //---------------------------------------------
                                // Again, wait a moment for things to stabilize
                                //---------------------------------------------
    Serial1.printf("%s: *** Setup() Completed ***\n", createDateTimeString() );
}

//*****************************************************************************
//  Loop
//*****************************************************************************
void loop() {

//-----------------------------------------------------------------------------
//  * Turn off the cpu led, as it may have been turned on in an interrupt handler
//-----------------------------------------------------------------------------
    digitalWriteFast(cLed_Cpu, cOff);

//-----------------------------------------------------------------------------
//  Get the time in milliseconds, since we use it all over the place. Return
//  if the time hasn't changed
//-----------------------------------------------------------------------------
    uint32_t wNow = millis();
    if ( mLastNow == wNow ) {
PowerControl::setActivity ( wNow ); // For now, reset our activity timer when we have nothing else to do
        return;
    }
    mLastNow = wNow;

//-----------------------------------------------------------------------------
//  Run any functions whose scheduling timer has expired. Each function will
//  reset its own scheduling timer
//-----------------------------------------------------------------------------
//  Leds now run under an IntervalTimer for better visual accuracy
//  extern uint32_t LedServices_SchedulingTimer;
//  checkAndRun ( wNow, LedServices_SchedulingTimer,        LedServices_TimedLoop       );

    checkAndRun ( wNow, Ax12Driver::mSchedulingTimer,       Ax12Driver::timedLoop       );
    checkAndRun ( wNow, Ax12Cerebellum::mSchedulingTimer,   Ax12Cerebellum::timedLoop   );
    checkAndRun ( wNow, mLogStatusSchedulingTimer,          logStatus_TimedLoop         );
    checkAndRun ( wNow, FaultsAndAlerts::mSchedulingTimer,  FaultsAndAlerts::timedLoop  );
    checkAndRun ( wNow, PowerControl::mSchedulingTimer,     PowerControl::timedLoop     );
    checkAndRun ( wNow, DipSwitches::mSchedulingTimer,      DipSwitches::timedLoop      );

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Serial1 keyboard commands
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//    int wSerialByte = Serial1.read();
//    if (wSerialByte != -1) {
//        gConsoleRxChar = (char) wSerialByte;
//        Serial1.printf("Serial1: Received byte [%c]\n", gConsoleRxChar);
//    }
}
