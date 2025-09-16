/**
 * @file app.c
 *
 *  Created on: @date Oct 14, 2024
 *      @author Jerry Ukwela (jeu6@case.edu)
 *      @brief 	Module Applicaiton Code
 *  @defgroup RM Remote Module
 *  @{
 *  @defgroup application Module Application
 *  @brief Module Application Code
 *  @}
 *  @defgroup FESCAN FESCAN
 */

//STM Remote Module
#include "ObjDict.h"
#include "states.h"
#include "app.h"
#include "main.h"

#include "can_stm_u5.h"

//Application Specific
volatile UNS8 f_syncPulse = 0;

/**
  * @brief  Function called at the initialization of program
  * @note	Object Dictionary has already been restored by this point, module type, node, and serial number have also been set
  * @param  None
  * @retval None
  * @ingroup application
  */
void initAppTask( void )
{
	//Initialize any required variables or peripherals here
}

/**
  * @brief  Main application loop called from main.c
  * @note	main.c sleep mode causes this function to be called once per SysTick (every 1ms)
  * @param  None
  * @retval None
  * @ingroup application
  */
void updateAppTask()
{
	//This function gets called in the main.c while loop

	UNS32 i = 0x00;

	runAppTask(i);
}

/**
 * @brief This task is run on the background thread to update parameters.
 * @param param can be used for anything, and is passed down from updateAppTask()
 * @ingroup application
 */
void runAppTask( UNS32 param )
{
	// Rate limit the task so its not running constantly
	static UNS8 taskDelaySecsApp = 1; // Can be linked to the OD
    static UNS32 tDelayRefApp = 0;
	CO_Data * d = &ObjDict_Data;

	if( !isTimedOut( &tDelayRefApp, TIMEOUT_sec( taskDelaySecsApp ) ) )
	{
		return;
	}
	//ELSE.. reading delay is over
	resetTimeOut( &tDelayRefApp );

	// Continue onto do the rest of the application task
	// Update or read from the OD to cause changes in program

	if(f_syncPulse){
		// Runs on sync pulse in the main application loop
		// Can either process sync pulse in the applicaiton, or in processSYNCMessageForApp
		f_syncPulse = 0;
	}
	
	
	switch( d->nodeState )
	{
		case Waiting:
			break;
		case Stopped:
			break;
		case Mode_Patient_Control:
			break;
		case Mode_Patient_Manual:
			break;
		case Mode_X_Manual:
			break;
		case Mode_Produce_X_Manual:
			break;
		case Mode_Y_Manual:
			break;
		case Mode_Record_X:
			break;
		default:
			break;
    }
}

/**
 * @brief Lets application process SYNC message if necessary
 * @param *m 8 byte message sent with SYNC objects from PM
 * @ingroup application
 * @note Changes made to the Object Dictionary in this function will be present in the response to the network
 */
void processSYNCMessageForApp(Message* m)
{
	// syncPulse is included as an example of setting a flag
	// to be used in the app task and is not required for functionality
	f_syncPulse = 1;
}

/**
 * @brief This function is called when the module recieves an NMT to enter low power mode
 * @param none
 * @ingroup application
 */
void sleepApplication(void)
{

}

/**
  * @brief  Reinitializes application values when necessary
  * @note	Runs when node is stopped, entering waiting mode, entering stop stim mode
  * @param  None
  * @retval None
  * @ingroup application
  */
void InitAppTaskValues(void)
{
	f_syncPulse = 0;
}

/**
 * @brief Lets application process NMT_Start_Nodes (unused)
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void StartNodesFunc(CO_Data* d, Message *m)
{
  if ( d->nodeState == Hibernate)
    setState(d,Waiting);
  else
    setState(d,Unknown_state);
}

/**
 * @brief Lets application process NMT_Stop_Nodes
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void StopNodesFunc(CO_Data* d, Message *m)
{
  //no restrictions on state entry
  setState(d,Stopped);
  InitAppTaskValues();
}

/**
 * @brief Lets application process NMT_Enter_Waiting
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void EnterWaitingFunc(CO_Data* d, Message *m)
{
  //no restrictions on state entry
  setState(d,Waiting);
  StopWatchDog( d );
  InitAppTaskValues();
}

/**
 * @brief Lets application process NMT_Enter_Patient_Operation
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void EnterPatientOperationFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
    setState(d, Mode_Patient_Control);
  }
}

/**
 * @brief Lets application process NMT_Enter_X_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void EnterXManualFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting) //must be in waiting
  {
    setState(d, Mode_X_Manual);
    StartWatchDog(d, 10000);
  }
}

/**
 * @brief Lets application process NMT_Enter_Y_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void EnterYManualFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting) //must be in waiting
  {
    setState(d, Mode_Y_Manual);
    StartWatchDog(d, 10000);
  }
}

/**
 * @brief Lets application process NMT_Enter_Stop_Stim
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void EnterStopStimFunc(CO_Data* d, Message *m)
{
  //no restrictions on state entry
  setState(d, Stopped);
  InitAppTaskValues();
}

/**
 * @brief Lets application process NMT_Enter_Patient_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void EnterPatientManualFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
    setState(d, Mode_Patient_Manual);
    StartWatchDog(d, 10000);
  }
}

/**
 * @brief Lets application process NMT_Produce_X_Manual
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void EnterProduceXManualFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting && (*m).data[1] == 0) //must be NMT broadcast and in waiting
  {
     setState(d, Mode_Produce_X_Manual);
  }
}

/**
 * @brief Lets application process NMT_Enter_Record_X
 * @param *d  Pointer to the CAN data structure
 * @param *m 3 byte message sent with NMT objects from PM
 * @ingroup CANFestival FESCAN
 */
void EnterRecordXFunc(CO_Data* d, Message *m)
{
  if (d->nodeState == Waiting)  //must be in waiting
  {
    setState(d, Mode_Record_X);
  }
}
