/*
 * can_stm_u5.h
 *
 *  Created on: Sept 15, 2025
 *      Author: djwalter
 */

#ifndef INC_ST32_U5_CAN_STM_H_
#define INC_ST32_U5_CAN_STM_H_

// Canfestivals includes
#include "can.h"

#include "config.h"
#include "ObjDict.h"

extern UNS8 txErr;
extern UNS8 rxErr;

/************************* To be called by user app ***************************/

unsigned char canInit();
unsigned char canSend(FDCAN_HandleTypeDef *hcan, Message *m);
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(FDCAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(FDCAN_HandleTypeDef *hcan);
unsigned char canChangeBaudRate_driver(FDCAN_HandleTypeDef *hcan, char* baud);
void UpdateCANerrors(void);

#endif /* INC_ST32_U5_CAN_STM_H_ */
