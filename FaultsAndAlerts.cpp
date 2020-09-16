//*****************************************************************************
//  FaultsAndAlerts.cpp: Implemented as a Singleton class
//
//  Class implementation of the Balancing 'Bot's FaultsAndAlerts class
//*****************************************************************************
#include "FaultsAndAlerts.h"
#include "LedServices.h"
#include "PowerControl.h"

//*****************************************************************************
//  External data and procedure references
//*****************************************************************************
extern char * createDateTimeString() ;

//*****************************************************************************
//  Static member data
//*****************************************************************************
uint32_t     FaultsAndAlerts::mSchedulingTimer = 0;

int          FaultsAndAlerts::mFaultedFlag = 0;

const int    FaultsAndAlerts::cMorseElementInterval   = 200;
uint32_t     FaultsAndAlerts::mMorseElementTimer      = 0;
const char * FaultsAndAlerts::pCurrentMorseTextChar   = 0;
const char * FaultsAndAlerts::pCurrentMorseElement    = 0;
int          FaultsAndAlerts::mCurrentMorseState      = 0;

const char * FaultsAndAlerts::pOriginalMorseTextChar  = 0;
const int    FaultsAndAlerts::cFaultMsgRepeatCount    = 3;
      int    FaultsAndAlerts::cFaultMsgRepeatCounter  = 0;

//*****************************************************************************
//  Morse Code table
//
//  Arranged as A-Z, 0-9, Space
//-----------------------------------------------------------------------------
//  The word PARIS is the standard to determine Morse Code speed. Each dit (dot)
//  is one element, each dah (dash) is three elements, intra-character spacing is
//  one element, inter-character spacing is three elements, and inter-word
//  spacing is seven elements. The word PARIS is exactly 50 elements.
//*****************************************************************************
const char * FaultsAndAlerts::cMorseCode [] = {
   // A                                                      H
    ".-",   "-...", "-.-.", "-..",  ".",   "..-.", "--.", "....",

    // I                                                     P
    "..",   ".---", "-.-",  ".-..", "--",  "-.",   "---", ".--.",
    
    // Q                                                     X
    "--.-", ".-.",  "...",   "-",   "..-", "...-", ".--", "-..-",

    // Y       Z
    "-.--", "--..",

    //-----------------------------------------------------------
    // 0                                   4
    "-----", ".----", "..---", "...--", "....-",

    // 5                                   9
    ".....", "-....", "--...", "---..", "----.",
    
    //-----------------------------------------------------------
    // A handy space string
    " " 
};

//*****************************************************************************
//  initNotification
//
//  Used to init the notifications we use. LEDs, buzzer, whatever!
//*****************************************************************************
void FaultsAndAlerts::initNotification() {
    LedServices_AllLeds(cSetThisColor);
}

//*****************************************************************************
//  setNotification
//
//  Used to init the notifications we use. LEDs, buzzer, whatever!
//*****************************************************************************
void FaultsAndAlerts::setNotification(int iData) {
    uint32_t    wColorMask = 0;
    if (iData) wColorMask = cRedMask;
    LedServices_AllLeds(cSetThisColor | wColorMask);
}

//*****************************************************************************
//  setup
//*****************************************************************************
void FaultsAndAlerts::late_setup(uint32_t iNow) {
    
}

//*****************************************************************************
//  playMorseElement
//*****************************************************************************
void FaultsAndAlerts::playMorseElement(uint32_t iNow) {
    char wChar = *pCurrentMorseElement++;
    if (wChar == '.') {
        setNotification(1);
        mMorseElementTimer = iNow + (cMorseElementInterval * 1);
        mCurrentMorseState = cMorseState_SendingMorseElement;
    } else if (wChar == '-') {
        setNotification(1);
        mMorseElementTimer = iNow + (cMorseElementInterval * 3);
        mCurrentMorseState = cMorseState_SendingMorseElement;
        
    } else {
        mCurrentMorseState = cMorseState_PostWordDelay;
        mMorseElementTimer = iNow + (cMorseElementInterval * 7);
    }
}

//*****************************************************************************
//  playTextCharacter
//*****************************************************************************
void FaultsAndAlerts::playTextCharacter(uint32_t iNow) {
    ++pCurrentMorseTextChar;
    char wChar = *pCurrentMorseTextChar;
    if (wChar == 0) {
        mCurrentMorseState = cMorseState_NotPlaying;
        return;
    }
 
    if ((wChar >= 'a') && (wChar <= 'z')) wChar &= 0x5F; // Convert to uppercase;
    if ((wChar >= 'A') && (wChar <= 'Z')) {
        pCurrentMorseElement = cMorseCode[wChar - 'A'];
    } else if ((wChar >= '0') && (wChar <= '9')) {
        pCurrentMorseElement = cMorseCode[wChar - '0' + 26];
    } else {
        pCurrentMorseElement = cMorseCode[36]; // Default to a space
    }
    playMorseElement(iNow);
}

//*****************************************************************************
//  playMorseCode
//
//  The word PARIS is the standard to determine Morse Code speed. Each dit (dot)
//  is one element, each dah (dash) is three elements, intra-character spacing is
//  one element, inter-character spacing is three elements, and inter-word
//  spacing is seven elements. The word PARIS is exactly 50 elements.
//*****************************************************************************
void FaultsAndAlerts::playMorseCode(uint32_t iNow) {
                                //---------------------------------------------
                                // We shouldn't be here! Handle it gracefully!
                                //---------------------------------------------
    if (mCurrentMorseState == cMorseState_NotPlaying) {
        return;
    }
                                //---------------------------------------------
                                // Sending Text Char
                                //---------------------------------------------
    if (mCurrentMorseState == cMorseState_SendingTextChar) {
        playTextCharacter(iNow);
        
                                //---------------------------------------------
                                // SendingMorseElement
                                //---------------------------------------------
    } else if (mCurrentMorseState == cMorseState_SendingMorseElement) {
        if (iNow <= mMorseElementTimer) return;
        setNotification(0);
        mMorseElementTimer = iNow + (cMorseElementInterval * 1);
        mCurrentMorseState = cMorseState_PostElementDelay;

                                //---------------------------------------------
                                // PostElementDelay
                                //---------------------------------------------
    } else if (mCurrentMorseState == cMorseState_PostElementDelay) {
        if (iNow <= mMorseElementTimer) return;
        playMorseElement(iNow);
                                //---------------------------------------------
                                // PostCharacterDelay
                                //---------------------------------------------
    } else if (mCurrentMorseState == cMorseState_PostCharacterDelay) {
        if (iNow <= mMorseElementTimer) return;
        playTextCharacter(iNow);
                                //---------------------------------------------
                                // PostWordDelay
                                //---------------------------------------------
    } else if (mCurrentMorseState == cMorseState_PostWordDelay) {
        if (iNow <= mMorseElementTimer) return;
        playTextCharacter(iNow);
        
                                //---------------------------------------------
                                // Not a good state to be in!
                                //---------------------------------------------
    } else {
        mCurrentMorseState = cMorseState_NotPlaying;
        return;
    }
}

//*****************************************************************************
//  timedLoop
//*****************************************************************************
void FaultsAndAlerts::timedLoop (uint32_t iNow) {
    mSchedulingTimer = iNow + 99;

    if (mCurrentMorseState != cMorseState_NotPlaying) playMorseCode(iNow);
    
    if ((mFaultedFlag) && (mCurrentMorseState == cMorseState_NotPlaying)) {
        if (cFaultMsgRepeatCounter) {
            startFaultNotification();
        } else {
            digitalWriteFast(cLed_Cpu, cOff);
            PowerControl::setPowerOff();
            while (1) ;
        }
    }
}

//*****************************************************************************
//  fault: Without locking up the CPU, clear any queued alerts, generate the
//         alert, then shut down power
//*****************************************************************************
void FaultsAndAlerts::fault(const char *iLoggedText, const char *iDisplayedText) {
    mFaultedFlag = 1;

    Serial1.printf("%s: FAULT(%s): %s\n",
        createDateTimeString(), iDisplayedText, iLoggedText);
    
    pOriginalMorseTextChar  = iDisplayedText;
    cFaultMsgRepeatCounter  = cFaultMsgRepeatCount;
    
    startFaultNotification();
}

//*****************************************************************************
//  fastFault: Without locking up the CPU, clear any queued alerts, generate the
//             alert, then shut down power. We'll do only one morse-code announce
//
//  This version is to be used when the fault is expected, e.g., when the 'power
//  off' button is pressed
//*****************************************************************************
void FaultsAndAlerts::fastFault(const char *iLoggedText, const char *iDisplayedText) {
    mFaultedFlag = 1;

    Serial1.printf("%s: FAST FAULT(%s): %s\n",
        createDateTimeString(), iDisplayedText, iLoggedText);
    
    pOriginalMorseTextChar  = iDisplayedText;
    cFaultMsgRepeatCounter  = 1;
    
    startFaultNotification();
}

//*****************************************************************************
//  startFaultNotification
//*****************************************************************************
void FaultsAndAlerts::startFaultNotification() {
    initNotification();
    mCurrentMorseState = cMorseState_SendingTextChar;
    pCurrentMorseTextChar = pOriginalMorseTextChar - 1;
    if (cFaultMsgRepeatCounter > 0) --cFaultMsgRepeatCounter;
}

//*****************************************************************************
//  alert: Without locking up the CPU, queue the alert and then generate it
//*****************************************************************************
void FaultsAndAlerts::alert(const char *iText) {
    
}

//*****************************************************************************
//  isFaulted
//
//  Returns: TRUE if we're handling a fault
//*****************************************************************************
bool FaultsAndAlerts::isFaulted() {
    return mFaultedFlag;
}
