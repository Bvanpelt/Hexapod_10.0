//*************************************************************************************************
//	Ax12Cerebellum.h
//*************************************************************************************************
#ifndef __AX12_CEREBELLUM_H__
#define __AX12_CEREBELLUM_H__

#include <arduino.h>
#include "Appldd.h"
#include "Ax12Driver.h"

//*************************************************************************************************
//*************************************************************************************************
//	class Ax12Cerebellum
//*************************************************************************************************
//*************************************************************************************************
class Ax12Cerebellum {

//*************************************************************************************************
//	Constants
//*************************************************************************************************
public:
static const int cNumberOfServos 	= 18;

//*************************************************************************************************
//	ServoStatus structure and enumerations
//*************************************************************************************************
private:
enum {
	ServoStatus_NotResponding,
	ServoStatus_Responding
};

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
typedef struct _ServoStatus {
	int		mActualServoId;

	int		mStatus;
	int		mStatusCounter;	// Used to provide hysterisis between Responding and Not-Responding
	int 	mPollCount;
	int 	mPollFailures;

	int		mCurrentPositionFromServo;

} ServoStatus, *pServoStatus;

//*************************************************************************************************
//	Class data
//*************************************************************************************************
public:
static uint32_t			mSchedulingTimer;

private:
static volatile int		mHealthyServos;
static ServoStatus 		mServoStatus [ cNumberOfServos ];

	//---------------------------------------------------------------------------------------------
	// Poll-For-Servo-Position data
	//---------------------------------------------------------------------------------------------
static uint32_t					   	mLastPollResult;
static uint32_t					   	mPollTimer;
static int						   	mRunningPollIndex;
static Ax12Driver::ServoMessage   * pPollTxData;	
static Ax12Driver::ServoMessage	  * pPollRxData;	

//*************************************************************************************************
//	Class functions
//*************************************************************************************************
public:
static void late_setup	( uint32_t iNow );
static void timedLoop	( uint32_t iNow );

static int  getHealthyServoCount();
static int  isServoHealthy( int iServoNumberRelToZero );
static int  getRealServoId( int iServoNumberRelToZero );
static void getServoStatus( int   iServoNumberRelToZero,
							int & oCurrentPosition,
							int & oPollCount,
							int & oPollFailureCount );

private:
static void pollForServoPosition ( uint32_t iNow ) ;

//*************************************************************************************************
};
#endif
