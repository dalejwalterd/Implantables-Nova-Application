/**
 * @file can_stm_u5.c
 *
 *  @date Created on: Sept 15, 2025
 *  @author Dale Walter (djwalter.software@gmail.com)
 *  @ingroup CANFestival
 * 	@ingroup FESCAN
 */

#include "main.h"
#include "stm32u5xx_hal_fdcan.h"
#include "can_stm_u5.h"
#include "canfestival.h"
#include <stdlib.h>
#include "ObjDict.h"

#include "app.h"

volatile unsigned char msg_received = 0;

volatile UNS16 BitErrors = 0 ;
volatile UNS16 StuffErrors = 0;
volatile UNS16 FormErrors = 0;
volatile UNS16 OtherErrors = 0;
volatile UNS16 receivedMessages = 0;

UNS8 txErr = 0;
UNS8 rxErr = 0;

FDCAN_RxHeaderTypeDef RxHeader;
FDCAN_TxHeaderTypeDef TxHeader;

unsigned char canInit()
{
  unsigned char j;
  UNS32 PDOData[4];
  UNS8 type = 0;
  UNS32 size = 0;
  UNS32 pdata = 0;

  for ( j = 0; j < 4; j++)
  {
    getODentry( &ObjDict_Data,
                   0x1400 + j,
                   1,
                   &pdata,
                   &size,
                   &type,
                   0);
    PDOData[j] = pdata;
  }
  UNS16 identifiers[] = { 0x0080, (UNS16)getNodeId(&ObjDict_Data), (UNS16)PDOData[0], (UNS16)PDOData[1], (UNS16)PDOData[2], (UNS16)PDOData[3], \
    0x0000, 0x0140, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

  // identifiers[] can be used to assign the CAN filter, is unused here
  TxHeader.Identifier = identifiers[1];	// TODO DJW double check this change --- TxHeader.StdId = identifiers[1];

  return 1;
}

void UpdateCANerrors(void)
{

      CAN_BitErrors = BitErrors;
      CAN_StuffErrors = StuffErrors;
      CAN_FormErrors = FormErrors;
      CAN_OtherErrors = OtherErrors;
      CAN_Rx_ErrCounter = (UNS16)rxErr;
      CAN_Tx_ErrCounter = (UNS16)txErr;
//      CAN_Receive_BEI !=  (0xFF & CANGCON);
//      CAN_Interrupts_Off |= (0xFF & CANGSTA);
}

/**
 * @brief The driver send a CAN message passed from the CANopen stack
 * @param hcan, CAN Handle
 * @param *m pointer to message to send
 * @return 1 if  hardware -> CAN frame
 * @ingroup CANFestival FESCAN
 */
UNS8 canSend(FDCAN_HandleTypeDef *hcan, Message *m)
{
#if 0
  if(HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0)
  {
	  TxHeader.StdId = m->cob_id;
	  TxHeader.IDE = CAN_ID_STD;
	  if(m->rtr)
		  TxHeader.RTR = CAN_RTR_REMOTE;
	  else
		  TxHeader.RTR = CAN_RTR_DATA;

	  TxHeader.DLC = m->len;			// Set data length code
	  memcpy(&TxData, m->data, m->len);

	  if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	  {
	  	  return 0;
	  }

	  CAN_Transmit_Messages++;
	  return 1;
  }

  return 0;
#endif
}

//Rx Functions
void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
#if 0
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	CAN_Receive_Messages++;

	Message rxm;

	if(RxHeader.IDE == CAN_ID_EXT)
		return;

	rxm.cob_id = RxHeader.StdId;
	if(RxHeader.RTR == CAN_RTR_REMOTE)
		rxm.rtr = 1;
	rxm.len = RxHeader.DLC;
	memcpy(&rxm.data, RxData, RxHeader.DLC);

	canDispatch(&ObjDict_Data, &rxm);
	UpdateCANerrors();
#endif
}

void HAL_CAN_RxFifo1MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, FDCAN_RX_FIFO1, &RxHeader, RxData);
	CAN_Receive_Messages++;
}

void HAL_CAN_ErrorCallback(FDCAN_HandleTypeDef *hcan)
{
#if 0
	if(hcan->ErrorCode & HAL_CAN_ERROR_NONE)
	{
		return;
	}
	if(hcan->ErrorCode & HAL_CAN_ERROR_STF)
	{
		StuffErrors++;
	}
	if(hcan->ErrorCode & HAL_CAN_ERROR_FOR)
	{
		FormErrors++;
	}
	if(hcan->ErrorCode & HAL_CAN_ERROR_ACK)
	{
		OtherErrors++;
	}
	if((hcan->ErrorCode & HAL_CAN_ERROR_BR) ||
	   (hcan->ErrorCode & HAL_CAN_ERROR_BD))
	{
		BitErrors++;
	}
	if((hcan->ErrorCode & HAL_CAN_ERROR_TIMEOUT) ||
	   (hcan->ErrorCode & HAL_CAN_ERROR_EWG)  ||
	   (hcan->ErrorCode & HAL_CAN_ERROR_EPV)  ||
	   (hcan->ErrorCode & HAL_CAN_ERROR_TIMEOUT) ||
	   (hcan->ErrorCode & HAL_CAN_ERROR_NOT_INITIALIZED) ||
	   (hcan->ErrorCode & HAL_CAN_ERROR_NOT_READY)  ||
	   (hcan->ErrorCode & HAL_CAN_ERROR_NOT_STARTED)  ||
	   (hcan->ErrorCode & HAL_CAN_ERROR_PARAM))
	{
		OtherErrors++;
	}
	if(hcan->ErrorCode & HAL_CAN_ERROR_BOF)
	{
		OtherErrors++;
	}
	if(hcan->ErrorCode & HAL_CAN_ERROR_CRC)
	{
		OtherErrors++;
	}
	if((hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR0) || (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR1) || (hcan->ErrorCode & HAL_CAN_ERROR_TX_TERR2))
	{
		txErr++;
	}
	if((hcan->ErrorCode & HAL_CAN_ERROR_RX_FOV0) || (hcan->ErrorCode & HAL_CAN_ERROR_RX_FOV1))
	{
		rxErr++;
	}
	if(hcan->ErrorCode & HAL_CAN_ERROR_STF){
		StuffErrors++;
	}

	CAN_TotalErrors++;

	hcan->ErrorCode = 0;
#endif
}

/**
 * @brief
 * @param fd
 * @param baud
 * @return always 0
 */
unsigned char canChangeBaudRate_driver(FDCAN_HandleTypeDef *hcan, char* baud)
{
	return 0;
}
