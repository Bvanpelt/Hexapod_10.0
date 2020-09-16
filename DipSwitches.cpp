//*****************************************************************************
//  DipSwitches.cpp: Implemented as a Singleton class
//
//  Class implementations for the DipSwitches class
//*****************************************************************************
#include <arduino.h>

#include "DipSwitches.h"

//*****************************************************************************
//	External data and procedure references
//*****************************************************************************
extern char * createDateTimeString() ;

//*****************************************************************************
//	Class data
//*****************************************************************************
uint32_t DipSwitches::mSchedulingTimer = 0;

uint16_t DipSwitches::mCurrentSwitchSettings; 

//*****************************************************************************
//  readSwitches
//
//	Returns: The current dipswitch settings as read from the switches
//*****************************************************************************
uint16_t DipSwitches::readSwitches() {
	uint16_t wResult = 0;
	if ( digitalReadFast( 4 ) ) wResult  |= 1;
	if ( digitalReadFast( 5 ) ) wResult  |= 2;
	if ( digitalReadFast( 6 ) ) wResult  |= 4;
	if ( digitalReadFast( 9 ) ) wResult  |= 8;
	return wResult;
}

//*****************************************************************************
//  late_setup
//*****************************************************************************
void DipSwitches::late_setup(uint32_t iNow) {
	pinMode( 4, INPUT );
	pinMode( 5, INPUT );
	pinMode( 6, INPUT );
	pinMode( 9, INPUT );
	mCurrentSwitchSettings = readSwitches();
	Serial1.printf("%s: Current DipSwitch setting is [%d]\n",
		createDateTimeString(), mCurrentSwitchSettings );
}

//*****************************************************************************
//  timedLoop
//*****************************************************************************
void DipSwitches::timedLoop (uint32_t iNow) {
	mSchedulingTimer = iNow + 1000;
	
	uint16_t wCurrentRead = readSwitches();
	if ( wCurrentRead != mCurrentSwitchSettings ) {
		mCurrentSwitchSettings = wCurrentRead;
		Serial1.printf("%s: DipSwitch setting changed to [%d]\n",
			createDateTimeString(), mCurrentSwitchSettings);
	}
}

//*****************************************************************************
//  getSettings
//
//	Returns: The current dipswitch settings as stored in memory
//*****************************************************************************
uint16_t DipSwitches::getSettings() {
	return mCurrentSwitchSettings;
}
