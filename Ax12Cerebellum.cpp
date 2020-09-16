//*************************************************************************************************
//	Ax12Cerebellum.cpp
//*************************************************************************************************
#include <arduino.h>
#include "Appldd.h"
#include "LedServices.h"
#include "Ax12Driver.h"
#include "Ax12Cerebellum.h"

//*************************************************************************************************
//	External data and procedure references
//*************************************************************************************************
extern void * get32ByteAlignedBuffer ( int iLength ) ;

//*************************************************************************************************
//	Local constants and enumerations
//*************************************************************************************************
const int cPollInterval = 41;	// Poll interval in ms. We EXPECT a round-trip or Rx-Timeout
								// ... to easily fit into this period
								// Btw, this is a prime number!

//*************************************************************************************************
//	Class data instantiations
//*************************************************************************************************
uint32_t					Ax12Cerebellum::mSchedulingTimer = 0;

Ax12Cerebellum::ServoStatus Ax12Cerebellum::mServoStatus [ cNumberOfServos ];

uint32_t					Ax12Cerebellum::mLastPollResult;
uint32_t					Ax12Cerebellum::mPollTimer;
int							Ax12Cerebellum::mRunningPollIndex;
volatile int 				Ax12Cerebellum::mHealthyServos;

Ax12Driver::pServoMessage   Ax12Cerebellum::pPollTxData;	
Ax12Driver::pServoMessage   Ax12Cerebellum::pPollRxData;	

//*************************************************************************************************
//	late_setup
//*************************************************************************************************
void Ax12Cerebellum::late_setup	( uint32_t iNow ) {
//-------------------------------------------------------------------------------------------------
//	Init our polling variables
//-------------------------------------------------------------------------------------------------
	mLastPollResult 	=  0xFFFFFFFF;
	mPollTimer 			=  0;
	mRunningPollIndex 	= -1;
	mHealthyServos		=  0;

	pServoStatus pServo = mServoStatus;
	for ( int wIndex = 0; wIndex < cNumberOfServos; ++wIndex, ++pServo ) {
		pServo->mActualServoId 				= wIndex + 1;
		pServo->mStatus 					= ServoStatus_NotResponding;
		pServo->mStatusCounter				= 0;
		pServo->mCurrentPositionFromServo	= 0;
		pServo->mPollCount					= 0;
		pServo->mPollFailures				= 0;
	}

	pPollTxData = ( Ax12Driver::pServoMessage ) get32ByteAlignedBuffer (
														sizeof ( Ax12Driver::ServoMessage ) );
	pPollRxData = ( Ax12Driver::pServoMessage ) get32ByteAlignedBuffer (
														sizeof ( Ax12Driver::ServoMessage ) );
}

//*************************************************************************************************
//	timedLoop
//*************************************************************************************************
void Ax12Cerebellum::timedLoop	( uint32_t iNow ) {
	mSchedulingTimer = iNow + cPollInterval;

// Brooks: Debugging code !!!
// mSchedulingTimer = iNow + 1000;

	pollForServoPosition ( iNow );
}

//*************************************************************************************************
//	getHealthyServoCount
//
//	Returns: The count of healthy servos
//*************************************************************************************************
int  Ax12Cerebellum::getHealthyServoCount() {
	return mHealthyServos;
}

//*************************************************************************************************
//	isServoHealthy
//
//	Returns: TRUE if the servo is responding to polls
//*************************************************************************************************
int  Ax12Cerebellum::isServoHealthy( int iServoNumberRelToZero ) {
	if ( mServoStatus[iServoNumberRelToZero].mStatus == ServoStatus_Responding ) return true;
	return false;
}

//*************************************************************************************************
//	getRealServoId
//
//	Returns: The servo's real ID (as opposed to the zero-relative value we use in code)
//*************************************************************************************************
int  Ax12Cerebellum::getRealServoId( int iServoNumberRelToZero ) {
	return mServoStatus[iServoNumberRelToZero].mActualServoId;
}

//*************************************************************************************************
//	getServoStatus
//*************************************************************************************************
void Ax12Cerebellum::getServoStatus( int   iServoNumberRelToZero,
									 int & oCurrentPosition,
									 int & oPollCount,
									 int & oPollFailureCount ) {
	pServoStatus pServo = & mServoStatus[iServoNumberRelToZero];
	oCurrentPosition  	= pServo->mCurrentPositionFromServo;
	oPollCount 			= pServo->mPollCount;
	oPollFailureCount 	= mServoStatus[iServoNumberRelToZero].mPollFailures;
}

//*************************************************************************************************
//	pollForServoPosition
//
//	We preiodically poll each servo for its position. We maintain servo I'm-alive/I'm-dead status
//	using the fact that the servo replied to us.
//*************************************************************************************************
void Ax12Cerebellum::pollForServoPosition ( uint32_t iNow ) {
//	int		mStatusCounter;	// Used to provide hysterisis between Responding and Not-Responding
//	int 	mPollFailures;

	//---------------------------------------------------------------------------------------------
	//	Interpret the last poll results
	//
	//	The Received msg should look like this:
	//		FF FF [ServoId] [04] [00] [LSB] [MSB] [Checksum]
	//	  For example:
	//		FF FF     01     04   00    7F    01      7A
	//---------------------------------------------------------------------------------------------
	pServoStatus pServo = & mServoStatus [ mRunningPollIndex ];

	int wStatus = Ax12Driver::getStatus();

	if ( wStatus == Ax12Driver::Status_Busy ) return; 	// This should NEVER happen. It implies that we
														// ... couldn't do a round-trip or a timeout
														// ... within the poll period!
/* if ( ! mRunningPollIndex) {
	const char wNoStatusText [2]  = "";
	const char wOkStatusText [6]  = " (OK)";
	const char wBadCrcText  [10]  = " (BadCrc)";

	const char * wStatusText = wNoStatusText;
	if ( wStatus == Ax12Driver::Status_LastReceiveOk ) wStatusText = wOkStatusText;
	if ( wStatus == Ax12Driver::Status_BadChecksum   ) wStatusText = wBadCrcText;
	Serial1.printf("Servo %d Tx+Rx result %d%s RxBuff: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		pServo->mActualServoId,
		wStatus, wStatusText,
		pPollRxData->mPrefix[0],
		pPollRxData->mPrefix[1],
		pPollRxData->mServoId,
		pPollRxData->mMessageLength, 
		pPollRxData->mDataBlock[0],
		pPollRxData->mDataBlock[1],
		pPollRxData->mDataBlock[2], 
		pPollRxData->mDataBlock[3]
		);
} */
	
	if ( mRunningPollIndex != -1 ) {	// <-- Skip this the first time through (at init)
		++pServo->mPollCount;
		if ( wStatus == Ax12Driver::Status_LastReceiveOk ) {
			pServo->mStatusCounter 					= 0;
			pServo->mStatus 						= ServoStatus_Responding;
			pServo->mCurrentPositionFromServo = 
						( ( pPollRxData->mDataBlock[2] & 0xFF ) << 8 )
						| ( pPollRxData->mDataBlock[1] & 0xFF );
		} else {
			++pServo->mStatusCounter;
			++pServo->mPollFailures;
			if ( pServo->mStatusCounter >= 3 ) {
				pServo->mStatusCounter 				= 3;
				pServo->mStatus 	   				= ServoStatus_NotResponding;
				pServo->mCurrentPositionFromServo	= 0;
			}
		}
	}
	
	//---------------------------------------------------------------------------------------------
	//	Set up and start the next poll
	//---------------------------------------------------------------------------------------------
	++mRunningPollIndex;

// Brooks: Debugging code !!!
// mRunningPollIndex = 0;

	if ( mRunningPollIndex >= cNumberOfServos ) mRunningPollIndex = 0;

	Ax12Driver::initTxMsgAssembly(	pPollTxData,
									mServoStatus[mRunningPollIndex].mActualServoId );
	Ax12Driver::assembleNextByte ( pPollTxData,  2 );	// Instruction code
	Ax12Driver::assembleNextByte ( pPollTxData, 36 );	// Starting register address
	Ax12Driver::assembleNextByte ( pPollTxData,  2 );	// # of byte registers to read


	// Send, then expect an 8 byte response (our two position register's values):
	//		[FF] [FF] [ID] [Len] [ResponseCode] [Position LSB] [Position MSB] [Chksum]
	Ax12Driver::sendAssembledMsg ( pPollTxData, 8, pPollRxData );

	//---------------------------------------------------------------------------------------------
	//	Interpret our status and update the Servo LED as reqd
	//---------------------------------------------------------------------------------------------
	mHealthyServos = 0;
	pServo 			= mServoStatus;
	for ( int wIndex = 0; wIndex < cNumberOfServos; ++wIndex, ++pServo ) {
		if ( pServo->mStatus == ServoStatus_Responding ) ++mHealthyServos;
	}

	uint32_t wNextLedCommandAndColor;
	if ( mHealthyServos == 0 ) {
		wNextLedCommandAndColor = cSweepToThisThenDim | cRedMask;
	} else if ( mHealthyServos < cNumberOfServos ) {
		wNextLedCommandAndColor = cSweepToThisThenDim | cYellowMask;
	} else {
		wNextLedCommandAndColor = cSetThisColorDim    | cGreenMask;
	}

	if ( wNextLedCommandAndColor == mLastPollResult ) return;
	mLastPollResult = wNextLedCommandAndColor;
	LedServices_ledRequest ( cLed_Servo, wNextLedCommandAndColor );
}
