//*****************************************************************************
//  FaultsAndAlerts.h: Implemented as a Singleton class
//
//  Class definition for our FaultsAndAlerts class
//-----------------------------------------------------------------------------
//  Alerts: Something the operator needs to know about
//  Faults: Alert, plus Shutdown
//
//  * Alerts are queued; up to 16 can be pending
//  * Faults are not queued; it's one, down and out!
//  * The Fault and Alert APIs present a string that is played in Morse Code
//    three times, with an appropriate pause between each play.
//  * For now we'll just blink the LEDs RED; once we get the buzzer sorted
//    we'll "buzz" the morse code
//*****************************************************************************
#ifndef __FAULTS_AND_ALERTS_H__
#define __FAULTS_AND_ALERTS_H__

#include <Arduino.h>

//*****************************************************************************
//*****************************************************************************
//  class FaultsAndAlerts
//*****************************************************************************
//*****************************************************************************
class FaultsAndAlerts {

//*****************************************************************************
//  Constants and enumerations
//*****************************************************************************
private:
enum {
    cMorseCodeElementTime   = 125,      // Duration in ms

    cMorseState_NotPlaying  = 0,
    cMorseState_SendingTextChar,
    cMorseState_SendingMorseElement,
    cMorseState_PostElementDelay,
    cMorseState_PostCharacterDelay,
    cMorseState_PostWordDelay,
};

//*****************************************************************************
//  Member data
//*****************************************************************************
public:
static uint32_t		mSchedulingTimer;

private:
static const char * cMorseCode [];
static int          mFaultedFlag;
static const char * pOriginalMorseTextChar;
static const char * pCurrentMorseTextChar;
static const char * pCurrentMorseElement;
static int          mCurrentMorseState;

static const int    cMorseElementInterval;  // ms
static uint32_t     mMorseElementTimer;

static const int    cFaultMsgRepeatCount;
static       int    cFaultMsgRepeatCounter;

//*****************************************************************************
//  Member functions
//*****************************************************************************
public:
static void late_setup(uint32_t iNow);
static void timedLoop (uint32_t iNow);

static void fault(const char *iLoggedText, const char *iDisplayedText);
static void fastFault(const char *iLoggedText, const char *iDisplayedText);
static void alert(const char *iText);
static bool isFaulted();
static void playMorseCode(uint32_t iNow);

protected:
static void playTextCharacter(uint32_t iNow);
static void playMorseElement(uint32_t iNow);
static void startFaultNotification();

static void initNotification();         // Used to init and operate the notifications
static void setNotification(int iData); // ...we use. LEDs, buzzer, whatever!

//*****************************************************************************
// End-of-class, followed by end of ifndef
};
#endif
