/*
 * can.h
 *
 *  Created on: Oct 9, 2024
 *      Author: jenej
 */

#ifndef INC_ST32_CAN_STM_H_
#define INC_ST32_CAN_STM_H_

// Canfestivals includes
#include "can.h"

#include "config.h"
#include "can_drv.h"
#include "ObjDict.h"

// Number of receive MOb
#define NB_RX_MOB                       2
// Number of transmit MOb
#define NB_TX_MOB                       (NB_MOB - NB_RX_MOB)

//#if (NB_TX_MOB < 1)
//#error define less RX Mobs, you must have at least 1 TX MOb!
//#elif (NB_RX_MOB < 8)
//#error define at least 8 RX MObs!
//#endif

#define START_TX_MOB                    NB_RX_MOB
#define TX_INT_MSK			((0x7F << (7 - NB_TX_MOB)) & 0x7F)

extern UNS8 txErr;
extern UNS8 rxErr;

/************************* To be called by user app ***************************/

unsigned char canInit();
unsigned char canSend(CAN_HandleTypeDef *hcan, Message *m);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
unsigned char canChangeBaudRate_driver(CAN_HandleTypeDef *hcan, char* baud);
void UpdateCANerrors(void);

#endif /* INC_ST32_CAN_STM_H_ */
