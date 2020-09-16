//*************************************************************************************************
//  Ax12Driver.h
//
//	The AX-12 driver singleton class encapsulates the api between the hardware+
//	DMA I/O and the low-level servo code.
//
//	This code supports:
//	* Send a message
//	* Send a message and receive its response. Handle responce timeouts
//	* Return status of this driver
//-------------------------------------------------------------------------------------------------
//	On the HexaTeen 10.0 board servo Rx and Tx is provided via the Rx3/Tx3 UART
//*************************************************************************************************
#ifndef __AX12_DRIVER_H__
#define __AX12_DRIVER_H__

#include <arduino.h>
#include <DmaChannel.h>

//*************************************************************************************************
//*************************************************************************************************
//	class Ax12Driver
//*************************************************************************************************
//*************************************************************************************************
class Ax12Driver {

//*************************************************************************************************
//	Servo Message definition
//*************************************************************************************************
public:
static const uint32_t cMaxServoDataBlockSize = 300;

public:
typedef struct _ServoMessage {
	//---------------------------------------------------------------------------------------------
	// UART data. Must be first!
	//---------------------------------------------------------------------------------------------
	uint32_t	mPrefix [ 2 ];		// Always 0xFF, 0xFF
	uint32_t	mServoId;
	uint32_t	mMessageLength;		// The length of the Data Block (below) + one byte for the
									// checksum byte

									// Message data. Includes the checksum
	uint32_t	mDataBlock [ cMaxServoDataBlockSize ];

	//---------------------------------------------------------------------------------------------
	// Tx message-accumulation variables
	//---------------------------------------------------------------------------------------------
	uint16_t	mTxWorkingOffset;
	uint8_t		mChecksumAccumulation;
	uint8_t		mExpectedRxServoId;

} ServoMessage, *pServoMessage;

//*************************************************************************************************
//  Status return codes
//
//	Notes:
//	* Returned internal status of other than "Busy" is inherently "Idle"
//	* A status() return of other than "Idle" or "Busy" returns the internal status to "Idle".
//	  Subsequent calls to status() will return "Idle"
//*************************************************************************************************
public:
enum {
    Status_Idle					= 0,
    Status_Busy					= 1,
 
    Status_LastReceiveOk		= 2,
    Status_WrongServoID			= 3,
    Status_RxTimeout			= 4,
    Status_MissingFfPrefix		= 5,
    Status_BadChecksum			= 6,

    Status_RxValidationNeeded	= 100,	// Internal use only! NEVER sent in reply to a status() request!
};

//*************************************************************************************************
//	Class data
//*************************************************************************************************
public:
static uint32_t			mSchedulingTimer;

static volatile int 	mStatus;

static volatile int 	mRxCompletionTimer;
static int 				mExpectedResponseLength;
static pServoMessage 	pLastReceiveMessage;

static DMAChannel		mTxDma;
static DMAChannel		mRxDma;

//*************************************************************************************************
//	Public methods
//*************************************************************************************************
public:
static void late_setup		( const uint32_t iNow ) ;
static void timedLoop		( const uint32_t iNow ) ;

static int  getStatus();

static void initTxMsgAssembly( 	ServoMessage * pServoMessage, uint8_t iServoId );
static int  assembleNextByte ( 	ServoMessage * pServoMessage, const uint8_t  iData );

static void sendAssembledMsg ( 	ServoMessage * pTxServoMessage );
static void sendAssembledMsg ( 	ServoMessage * pTxServoMessage,
								int 		   iExpectedRxLength,
								ServoMessage * pRxServoMessage );

//*************************************************************************************************
//	Private methods
//*************************************************************************************************

//*************************************************************************************************
};		// End of class Ax12Driver
#endif
