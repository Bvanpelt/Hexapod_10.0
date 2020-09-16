//*****************************************************************************
//  Ax12Driver.cpp
//
//	The AX-12 driver singleton class encapsulates the api between the hardware+
//	DMA I/O and the low-level servo code.
//
//	This code supports:
//	* Send a message
//	* Send a message and receive its response
//	* Return status of this driver
//-------------------------------------------------------------------------------------------------
//	*** IMPORTANT NOTES FOR FUTURE USERS ***
//	1) Servo Rx and Tx is provided via Serial3 aka UART2
//	2) The DATA Tx and Rx registers (and thus our data, and ESPECIALLY our DMA)
//     ... are 32 bit values.
//	3) When using Paul's DmaChannel, the DMA 'length' is:
//		...  [the number of objects to send] * [the size of each object]
//-----------------------------------------------------------------------------
//
//   /---v---v---\      Pin 1: GND      |
//  |  o   o   o  |     Pin 2: VDD      | *** Viewed looking into the servo ***
//  |             |     Pin 3: Data     |
//  ---------------
//     1   2   3
//
//   Driving circuit reccomendation:
//   * 74HC125/126
//   * 10k pullup to +5
//
//-----------------------------------------------------------------------------
//  Servo documentation
//-----------------------------------------------------------------------------
//                              //---------------------------------------------
//                              // Local copies
//                              //---------------------------------------------
//  ../hexapod/Supporting stuff/Robotis Docs/...
//
//                              //---------------------------------------------
//                              // Servo family command structure
//                              //---------------------------------------------
// http://support.robotis.com/en/product/dynamixel/communication/dxl_instruction.htm
//
// "Generally, in the event 1 command packet is 4 byte, 26 Dynamixel can be
//  controlled simultaneously. Make sure that the length of packet does not to
//  exceed 143 bytes since the volume of receiving buffer of RX-64 is 143 bytes."
//
//                              //---------------------------------------------
//                              // Ax12 register tables, etc
//                              //---------------------------------------------
// http://support.robotis.com/en/product/dynamixel/ax_series/dxl_ax_actuator.htm
//
//                              //---------------------------------------------
//                              // This TX works, and solicits a response from
//                              // the servo.
//                              //---------------------------------------------
// From my modified ax12.cpp:
//  Ping Tx Example: 0xFF 0xFF 0x01 0x02 0x01 0xFB
//                              ID  Len  PING CSum
//                   -1-- -2-- -3-- -4-- -5-- -6--: Hence the Tx Packet len of 6   
//
//  Ping response: FF FF 01 02 00 FC
//
//    unsigned char cMessage [] = "\x01\x02\x01";
//    int  cMessageLength = 3;  // Prefix and checksum will be added automatically
//
//-----------------------------------------------------------------------------
//	UART documentation
//-----------------------------------------------------------------------------
//	*** Don't forget that Serial3 uses UART2 !!! ***
//
//*****************************************************************************
#include "arduino.h"

#include "Appldd.h"
#include "Ax12Driver.h"

//*************************************************************************************************
//	Local constants and enumerations
//*************************************************************************************************
static const int cServoDrivePin					= 2;

//*************************************************************************************************
//	Class data
//*************************************************************************************************
uint32_t					Ax12Driver::mSchedulingTimer		= 0;

volatile int 				Ax12Driver::mRxCompletionTimer		= 0;
volatile int 				Ax12Driver::mStatus					= Ax12Driver::Status_Idle;
int 						Ax12Driver::mExpectedResponseLength = 0;
Ax12Driver::pServoMessage 	Ax12Driver::pLastReceiveMessage		= NULL;

DMAChannel		Ax12Driver::mTxDma;
DMAChannel		Ax12Driver::mRxDma;

//*****************************************************************************
//  uartRxDmaIsr
//
//  Handles Rx DMA interrupt at the end of the data transfer
//  Note: https://community.nxp.com/docs/DOC-102981
//
//	Note: This routine is used by the loop(...) code when we have an Rx timeout,
//  to take advantage of its robust cleanup code.
//*****************************************************************************
static void uartRxDmaIsr(void) {
	// Turn ON the CPU LEd
    digitalWriteFast(cLed_Cpu, cOn);

	// Stop the Rx Dma. We'll disable the Rx DMA FIRST to minimize the timing
	// window that's open between loop() processing and DMA-completion interrupts
    Ax12Driver::mRxDma.clearInterrupt();
    Ax12Driver::mRxDma.disable();

    // Stop the Rx completion timer
    Ax12Driver::mRxCompletionTimer = 0;

    // Disable the UART Rx
    LPUART2_CTRL &= ~LPUART_CTRL_RE;

    // Enable the Tx drive
    digitalWriteFast( cServoDrivePin, LOW );

    // We'll leave the Rx completion validity checks to the status() routine,
    // ... rather than try to do it here in the interrupt handler
    Ax12Driver::mStatus = Ax12Driver::Status_RxValidationNeeded;
}

//*****************************************************************************
//  uartStatusIsr
//
//	Handle Tx-complete interrupts (the last Tx stop-bit is out the door)
//*****************************************************************************
static void uartStatusIsr(void) {
	// Turn ON the CPU LEd
    digitalWriteFast(cLed_Cpu, cOn);

	//-------------------------------------------------------------------------
	// Ignore a spurious interrupt
	//-------------------------------------------------------------------------
	if ( ! ( LPUART2_STAT & LPUART_STAT_TC ) ) return;

	//-------------------------------------------------------------------------
	// Disable further Tx-complete (TC) interrupts
	//-------------------------------------------------------------------------
	LPUART2_CTRL &= ~( LPUART_CTRL_TCIE );

	//-------------------------------------------------------------------------
	// Completely stop the Tx DMA
	//-------------------------------------------------------------------------
    Ax12Driver::mTxDma.disable();

	//-------------------------------------------------------------------------
	// If we're not expecting a response from a servo we're 'Idle' and done
	//-------------------------------------------------------------------------
	if ( ! Ax12Driver::mExpectedResponseLength ) {
		Ax12Driver::mStatus = Ax12Driver::Status_Idle;
		return;
	}

	//-------------------------------------------------------------------------
	// Turn around the drive-enablement and prepare to receive the servo's
	// message.
	//-------------------------------------------------------------------------
    digitalWriteFast( cServoDrivePin, HIGH );    // Disable Tx drive

	Ax12Driver::mRxDma.destinationBuffer(
	    Ax12Driver::pLastReceiveMessage->mPrefix, 
	    ( Ax12Driver::mExpectedResponseLength * sizeof ( Ax12Driver::pLastReceiveMessage->mPrefix[0] ) )
	    );
    Ax12Driver::mRxDma.enable();
	Ax12Driver::mRxDma.disableOnCompletion();

    LPUART2_CTRL |= LPUART_CTRL_RE;				// Enable the UART Rx

    Ax12Driver::mRxCompletionTimer = 1;			// Arrange to run the Rx completion timer
}

//*************************************************************************************************
//	timedLoop
//*************************************************************************************************
void Ax12Driver::timedLoop(  const uint32_t iNow ) {
	//---------------------------------------------------------------------------------------------
	// We'll run every 3ms
	//---------------------------------------------------------------------------------------------
	mSchedulingTimer = iNow + 3;

	//---------------------------------------------------------------------------------------------
	// If there's been an Rx timeout, simulate the Rx DMA completion and then override the status
	// ... to be Rx-Timeout
	//---------------------------------------------------------------------------------------------
    if ( mRxCompletionTimer ) {
    	++mRxCompletionTimer;
    	if ( mRxCompletionTimer > 4 ) {
    		uartRxDmaIsr();

    		// We'll leave the Rx completion validity checks to the status() routine,
    		// ... rather than try to do it here
    		mStatus = Status_RxTimeout;
		    mRxCompletionTimer = 0;
    	}
    }
}

//*************************************************************************************************
//	late_setup
//*************************************************************************************************
void Ax12Driver::late_setup( const uint32_t iNow ) {
	//---------------------------------------------------------------------------------------------
	// Use the standard Serial3 initialization to do the heavy lifting of initialization for us
	//---------------------------------------------------------------------------------------------
	while ( ( ! Serial3 ) && ( millis() < 2000 ) ) ;
	Serial3.begin ( 1000000 );

	//---------------------------------------------------------------------------------------------
	// Set up our direction-control pin. We'll normally leave the Tx output enabled, 
	// ... with the UART Tx marking the servo data line
	//---------------------------------------------------------------------------------------------
	pinMode ( cServoDrivePin, OUTPUT );
	digitalWriteFast( cServoDrivePin, LOW );

	//---------------------------------------------------------------------------------------------
	// Set up the Tx and Rx DMA channels, leaving both inactive. We don't bother with a Tx DMA
	// completion interrupt; we'll just use the Tx "TC" status interrupt
	//---------------------------------------------------------------------------------------------
    mTxDma.destination(LPUART2_DATA);
    mTxDma.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART2_TX);

    mRxDma.source(LPUART2_DATA);
    mRxDma.interruptAtCompletion();
    mRxDma.attachInterrupt(uartRxDmaIsr);
    mRxDma.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART2_RX);

	//---------------------------------------------------------------------------------------------
	// Modify the Uart setup to suit our needs:
	// * Rx disabled
	// * Tx enabled, idle, marking the output line
	// * Tx and Rx DMA idle
	// * UART prepared for both Tx and Rx DMA
	// * Interrupts:
	//	 - UART Tx status: The last bit of the message has gone out: Status:TC
	//	 - Dma Rx: The last byte of the expected Rx message has arrived
	//---------------------------------------------------------------------------------------------
	// Enable Tx and Rx DMA
	LPUART2_BAUD |= ( LPUART_BAUD_TDMAE | LPUART_BAUD_RDMAE );

	// Since we ourselves didn't actually set up the UART, we'll modify
	// ... the setup to be the way we want
    LPUART2_CTRL &= ~(    LPUART_CTRL_ORIE
    					| LPUART_CTRL_NEIE
    					| LPUART_CTRL_FEIE
    					| LPUART_CTRL_PEIE
    					| LPUART_CTRL_TIE
    					| LPUART_CTRL_RIE
    					| LPUART_CTRL_ILIE
    					| LPUART_CTRL_RE 		// <-- Leave the received disabled
    					| LPUART_CTRL_RWU
    					| LPUART_CTRL_SBK
    					| LPUART_CTRL_TCIE		// <-- Disable TC interrupts for now
    					);

    // Change registers that require TE and RE to be off. Since we're using Paul's initialization
    // ... code, we may need to sneak back and fiddle with some stuff...
    LPUART2_FIFO |= LPUART_FIFO_TXFLUSH;	// Flush the Tx FIFO
    LPUART2_FIFO |= LPUART_FIFO_RXFLUSH;	// Flush the Rx FIFO

    									//---------------------------------------------------------
    									// Disable the Rx FIFO, as we'll be doing DMA. It took me
    									// PAINFUL WEEKS to figure this out!
    									//---------------------------------------------------------
    LPUART2_FIFO &= ~LPUART_FIFO_RXFE;

	// Grab the UART's status IRQ
	attachInterruptVector(IRQ_LPUART2, uartStatusIsr);

    // Enable the transmitter
    LPUART2_CTRL |= ( LPUART_CTRL_TE );
}

//*************************************************************************************************
//	getStatus
//
//	Returns the current status. Resets non-Busy status to Idle
//*************************************************************************************************
int  Ax12Driver::getStatus() {
	int wResult = mStatus;	// Store it locally to make us (mostly) immune to valatility issues

//-------------------------------------------------------------------------------------------------
//	If we're busy, just tell our caller we're busy!
//-------------------------------------------------------------------------------------------------
	if ( wResult == Status_Busy ) return wResult;

//-------------------------------------------------------------------------------------------------
//	If we received a msg from a servo, validate it and set our status appropriately
//-------------------------------------------------------------------------------------------------
	while ( wResult == Status_RxValidationNeeded ) {	// Loops make breaking out easier!
		wResult = Status_LastReceiveOk;

		// Ensure the proper servo responded!
		if ( pLastReceiveMessage->mExpectedRxServoId != pLastReceiveMessage->mServoId ) { 
			wResult = Status_WrongServoID;
			break;
		}

		if ( ( pLastReceiveMessage->mPrefix[0] & 0xFF ) != 0xFF ) {
			wResult = Status_MissingFfPrefix;
			break;
		}
		if ( ( pLastReceiveMessage->mPrefix[1] & 0xFF ) != 0xFF ) {
			wResult = Status_MissingFfPrefix;
			break;
		}
		uint32_t wChecksumAccumulation = 0;
		uint32_t  * pChar = & pLastReceiveMessage->mServoId;
		for ( uint32_t wIndex = 0;
			  wIndex < (pLastReceiveMessage->mMessageLength + 1);
			  ++wIndex, ++pChar ) {
			wChecksumAccumulation += *pChar;
		}
		wChecksumAccumulation = 255 - wChecksumAccumulation;
		if ( wChecksumAccumulation != *pChar ) {
			wResult = Status_BadChecksum;
		}
		break;
	}

//-------------------------------------------------------------------------------------------------
//	Reset our local status to "Idle" and then return our computed status
//-------------------------------------------------------------------------------------------------
	mStatus = Status_Idle;
	return wResult;
}

//*************************************************************************************************
//	initTxMsgAssembly
//
//	These three routines will self-generate the 
//		* Msg prefix: FF FF 	: We provide
//		* Servo ID 				: We provide
//		* Message length 		: We provide
//		* Message 				: Supplied by our caller
//		* Trailing checksum 	: We provide
//
//	The checksum includes all bytes starting with the servo-id, excluding the checksum itself
//	The length includes all the bytes after the length character including the checksum
//*************************************************************************************************
void Ax12Driver::initTxMsgAssembly( ServoMessage * pServoMessage, uint8_t iServoId ) {
	pServoMessage->mTxWorkingOffset			= 0;

    pServoMessage->mPrefix [0] 				= 0xFF;
    pServoMessage->mPrefix [1] 				= 0xFF;
    pServoMessage->mServoId					= iServoId;
    pServoMessage->mMessageLength			= 0;

    pServoMessage->mChecksumAccumulation 	= iServoId;
}

//*************************************************************************************************
//	assembleNextByte
//
//	Returns: 0 if ok
//*************************************************************************************************
int Ax12Driver::assembleNextByte ( ServoMessage * pServoMessage, const uint8_t  iData ) {
    if (pServoMessage->mTxWorkingOffset >= cMaxServoDataBlockSize ) return true;
    pServoMessage->mDataBlock [ pServoMessage->mTxWorkingOffset++ ] = iData;
    pServoMessage->mChecksumAccumulation += iData;
    ++pServoMessage->mMessageLength;
    return false;

}

//*************************************************************************************************
//	sendAssembledMsg
//
//	Note: The 'Expected Rx Length' is the length of the ENTIRE message. In the example below, the
//        expected Rx-length is 6:
//
//  		Ping response: FF FF 01 02 00 FC
//*************************************************************************************************
void Ax12Driver::sendAssembledMsg (	ServoMessage * pTxServoMessage ) {
	sendAssembledMsg ( pTxServoMessage, 0, NULL );
}
//* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
void Ax12Driver::sendAssembledMsg (	ServoMessage * pTxServoMessage,
									int 		   iExpectedRxLength,
									ServoMessage * pRxServoMessage ) {
    //---------------------------------------------------------------------------------------------
    //	Ensure an Rx Servo Message structure was provided if the expected Rx length is non-zero
    //---------------------------------------------------------------------------------------------
	if ( ( iExpectedRxLength ) && ( ! pRxServoMessage ) ) return;

    //---------------------------------------------------------------------------------------------
    // Update the length, finish and set the checksum
    //---------------------------------------------------------------------------------------------
	// Allow room for the checksum
	++(pTxServoMessage->mMessageLength);	

	// Update the running checksum with the just-updated message length
    pTxServoMessage->mChecksumAccumulation += pTxServoMessage->mMessageLength;

    // Place the calculated checksum into the message
    pTxServoMessage->mDataBlock [ pTxServoMessage->mTxWorkingOffset++ ] = 
    			( 255 - pTxServoMessage->mChecksumAccumulation ) & 0x000000FF;

    //---------------------------------------------------------------------------------------------
    // Set up for receive as needed
    //---------------------------------------------------------------------------------------------
    Ax12Driver::mExpectedResponseLength = iExpectedRxLength;

    //---------------------------------------------------------------------------------------------
    // Flag ourselves as busy
    //---------------------------------------------------------------------------------------------
	mStatus = Status_Busy;

    //---------------------------------------------------------------------------------------------
    // Init the Rx-function variables, then, if we're expecting a response, set up the Rx itself
    //---------------------------------------------------------------------------------------------
    pLastReceiveMessage 			= NULL;
    Ax12Driver::mRxCompletionTimer 	= 0;
    mExpectedResponseLength			= iExpectedRxLength;

    if ( iExpectedRxLength ) {
    	// Save the Rx structure's address
    	pLastReceiveMessage = pRxServoMessage;

    	// Save the expected-in-the-response servo-id
    	pRxServoMessage->mExpectedRxServoId = pTxServoMessage->mServoId;

    	// In case there's unclaimed bytes in the Rx FIFO, fLush it
    	LPUART2_FIFO |= LPUART_FIFO_RXFLUSH;

    	// It's good practice to init the message-length to an impossible value so we're sure
    	// ... we actually received a message from the servo
		pRxServoMessage->mMessageLength = 0xFF;

/*// Debugging: Init the Rx buffer data
pRxServoMessage->mPrefix [ 0 ] = 0x12;
pRxServoMessage->mPrefix [ 1 ] = 0x34;
pRxServoMessage->mServoId      = 0x56;
pRxServoMessage->mMessageLength= 0x78;
pRxServoMessage->mDataBlock[0] = 0xAA;
pRxServoMessage->mDataBlock[1] = 0xBB;
pRxServoMessage->mDataBlock[2] = 0xCC;
pRxServoMessage->mDataBlock[3] = 0xDD; */

		// Flush the now-completed Rx message from cache to actual memory. We can't change it
		// ... after this point
		arm_dcache_flush_delete ( pRxServoMessage,
								  iExpectedRxLength * sizeof ( pRxServoMessage->mPrefix[0] ) );
    }

    //---------------------------------------------------------------------------------------------
    // Set up and start the Tx ( Serial3 aka UART2 )
    //---------------------------------------------------------------------------------------------
    int wActualTxLength = pTxServoMessage->mTxWorkingOffset + 4;

	// Flush the now-completed Tx message from cache to actual memory. We can't change it
	// ... after this point
	arm_dcache_flush (  pTxServoMessage,
						wActualTxLength * sizeof ( pTxServoMessage->mPrefix[0] ) );

    mTxDma.sourceBuffer( 
    	(uint32_t *) pTxServoMessage->mPrefix,
    	 wActualTxLength * sizeof ( pTxServoMessage->mPrefix[0] ) 
    		  );
    mTxDma.enable();
    mTxDma.disableOnCompletion();
 
    LPUART2_CTRL |= LPUART_CTRL_TCIE;			  // <-- Enable TC interrupts
}
