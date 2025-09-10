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
#include "ObjDict.h"

extern UNS8 txErr;
extern UNS8 rxErr;

typedef FDCAN_HandleTypeDef CAN_HandleTypeDef;

/************************* To be called by user app ***************************/

unsigned char canInit();
unsigned char canSend(CAN_HandleTypeDef *hcan, Message *m);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
unsigned char canChangeBaudRate_driver(CAN_HandleTypeDef *hcan, char* baud);
void UpdateCANerrors(void);

#endif /* INC_ST32_CAN_STM_H_ */
