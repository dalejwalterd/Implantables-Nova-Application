/*
 * timerscfg.h
 *
 *  Created on: Oct 9, 2024
 *      Author: jenej
 */

#ifndef INC_ST32_TIMERSCFG_H_
#define INC_ST32_TIMERSCFG_H_

// Whatever your microcontroller, the timer wont work if
// TIMEVAL is not at least on 32 bits
#define TIMEVAL UNS32

// The timer of the STM32 counts from 0 to 0xFFFF (it can be
// shortened setting counter ARR register eg. to get 2ms instead of 2.048ms)
#define TIMEVAL_MAX 0xFFFF

#if (FOSC == 8000)
// The timer is incrementing every 8 us at 8 Mhz/64 clock.
#define MS_TO_TIMEVAL(ms) ((ms) * 125UL) //JML added UL otherwise may overflow
#define US_TO_TIMEVAL(us) ((us)>>3)
#endif
#if (FOSC == 4000)
// The timer is incrementing every 16 us at 4 Mhz/64 clock.
// clock will be slightly slow (63/62.5=1.008)*ms
#define MS_TO_TIMEVAL(ms) ((ms) * 63UL) //JML added UL otherwise may overflow
#define US_TO_TIMEVAL(us) ((us)>>4)
#endif
#if (FOSC == 2000)
// The timer is incrementing every 32 us at 2 Mhz/64 clock
// clock will be slightly fast (31/31.25=0.996)*ms
#define MS_TO_TIMEVAL(ms) ((ms) * 31UL) //JML added UL otherwise may overflow
#define US_TO_TIMEVAL(us) ((us)>>5)
#endif
#if (FOSC == 1000)

#define MS_TO_TIMEVAL(ms) ((ms) * 100UL) //JML added UL otherwise may overflow
#define US_TO_TIMEVAL(us) ((us)>>3)
#endif

#endif /* INC_ST32_TIMERSCFG_H_ */
