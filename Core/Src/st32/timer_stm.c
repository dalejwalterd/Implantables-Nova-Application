/**
 * @file timer_stm.c
 *
 *  @date Created on: Oct 9, 2024
 *  @author Jerry Ukwela (jeu6@case.edu)
 *  @ingroup CANFestival
 * 	@ingroup FESCAN
 */


#include "timer.h"
#include "stm32l4xx.h"
#include "main.h"

/************************** Module variables **********************************/
// Store the last timer value to calculate the elapsed time
static volatile TIMEVAL last_time_set = 0;//TIMEVAL_MAX;
static volatile TIMEVAL elapsed_time = 0;

/**
  * @brief  Set the timer peripheral for the next interrupt
  */
void setTimer(TIMEVAL value){
	uint32_t timer = TIM2->CNT;
	elapsed_time += timer - last_time_set;

	last_time_set = 65535-value;
	TIM2->CNT = last_time_set;

	TIM2->CR1 |= 1; //Re-enable timer
}

void initTimer(void)
{

}

/**
  * @brief  Checks if a timer has timed out
  */
UNS8 isTimedOut( UNS32 *tRef, UNS32 tAlarm )
{

	if( (HAL_GetTick() - *tRef) > tAlarm )
	{
		return 1;
	}

	else
		return 0;
}

/**
  * @brief  Resets a timed out timer
  */
void resetTimeOut( UNS32 *tRef )
{
	*tRef = HAL_GetTick();
}

/**
 * @brief Return the elapsed time to tell the Stack how much time is spent since last call.
 * @return value TIMEVAL (unsigned long) the elapsed time
 */
TIMEVAL getElapsedTime(void)
{
	TIMEVAL timer = TIM2->CNT;

    if (timer > last_time_set)                    // In case the timer value is higher than the last time.
	  return (timer - last_time_set);             // Calculate the time difference
    else if (timer < last_time_set)
	  return (last_time_set - timer);             // Calculate the time difference
    else
	  return TIMEVAL_MAX;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		last_time_set = 0;
		elapsed_time = 0;
		TIM2->SR = (uint16_t)~TIM_SR_UIF;  //Clear pending interrupt
		TimeDispatch();
	}
}
