/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "fdcan.h"
#include "adc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "eedata.h"
#include "acceltemp.h"
#include "app.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CO_ENABLE_LSS

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

uint8_t RxData[8];
uint8_t TxData[8];
uint8_t TxDataSDO[8];
uint8_t TxDataPDO[8];
uint8_t TxDataScan[8];

uint32_t TxMailbox;

UNS16 blink = 0;
UNS8 lastRequestedAddress = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void initNodeIDSerialNumber( void );
void ReadMemory( void );
UNS8 ReadLocalFlashMemory(void);
UNS8 ReadEEProm(void);
UNS8 WriteEEProm ( void );

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Callback that runs at the beginning of timer execution
  */
void start_callback(CO_Data* d, UNS32 id)
{
	setState(&ObjDict_Data, Waiting);
}

/**
  * @brief  Initializes node ID and serial number data in the object dictionary
  */
void initNodeIDSerialNumber(void)
{
	UNS8 node = *(uint8_t*)(127 * FLASH_PAGE_SIZE); // Config data lives in the last page of flash
	if(node > 0 && node <= 0x7F)
	  Status_NodeId = node;

	setNodeId(&ObjDict_Data, Status_NodeId);

	ObjDict_obj1018_Serial_Number = *(uint8_t*)(0x08004FFD) << 8 | *(uint8_t*)(0x08004FFE);
	ObjDict_obj1018_Product_Code =  *(uint8_t*)(0x08004FFF);

	HAL_Delay(START_DELAY_MS * (Status_NodeId - 1));
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  if (CheckRestoreFlag())
  {
	  //Load custom settings from EEPROM into the OD
	  RestoreValues();
  }
  else
  {
	  //Use default settings from OD rather than what is stored in EEPROM
	  //Then, overwrite EEPROM with these default settings
	  SaveValues();
  }
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  initNodeIDSerialNumber();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  initAccelerometer();
  initTemperature();
  initDiagnostics();
  initAppTask();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	updateAccelerometer();
//	updateTemperature(); //TODO: Check w/physical temp module
	updateDiagnostics();
	updateAppTask();
	Status_TestValue++;

	if(blink++ > 125){
	  if(getState(&ObjDict_Data) == Waiting)
	  {
		  ReadMemory();
	  }
	  blink = 0;
	  //TODO: Turn off heartbeat LED
	  //HAL_GPIO_WritePin();
	}
	else if(blink > 100)
	{
	  //TODO: Turn on heartbeat LED
	  //HAL_GPIO_WritePin();
	}

	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  if(HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {Error_Handler();}

  CLEAR_BIT(TIM2->CR1, TIM_CR1_CEN); //Disable the timer
  TIM2->CNT = 1; // Set count to 1

  SetAlarm(NULL, 0, start_callback, 0, 0); // Begin the timer

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : I2C2_INT_Pin */
  GPIO_InitStruct.Pin = I2C2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(I2C2_INT_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



/**
 * @brief Command processor to let CE read various memory locations
 * @param none
 */
void ReadMemory(void)
{
	// clear status
	if (memorySelect > 0 )
	  statusByteMemory = 0;

	if (memorySelect == 1)
	  statusByteMemory = ReadLocalFlashMemory();
	else if (memorySelect == 4)
	  statusByteMemory = ReadEEProm();
	else if (memorySelect == 8)
	  statusByteMemory = WriteEEProm();
	else if (memorySelect != 0)
	  statusByteMemory = 4;

	memorySelect = 0;
}
/**
 * @brief Read flash memory in CPU
 * @param none
 */
UNS8 ReadLocalFlashMemory(void)
{
  UNS8 RMDataSize = sizeof(ReadMemoryData);
  UNS16 recordSize = FLASH_RECORD_SIZE;

  if (AddressRequest > (FLASH_SIZE - recordSize))
    return 2;

  if (lastRequestedAddress != AddressRequest || (triggerReadMemory == 1))
  {
    // set pattern to clear if read
    memset( ReadMemoryData, 0xA5, sizeof(ReadMemoryData));
    ReadLocalFlashData( AddressRequest, ReadMemoryData, recordSize );
    // now copy requested address into the top of the array
    ReadMemoryData[RMDataSize - 4] = (UNS8) AddressRequest;
    ReadMemoryData[RMDataSize - 3] = (UNS8)(AddressRequest >> 8);
    ReadMemoryData[RMDataSize - 2] = (UNS8)(AddressRequest >> 16);
    ReadMemoryData[RMDataSize - 1] = (UNS8)(AddressRequest >> 24);

    lastRequestedAddress = AddressRequest;
    triggerReadMemory = 0;
    return 0;
  }
  return 1;
}
/**
 * @ingroup eeprom
 * @brief used with handshake through the OD at 0x2020 to allow remote reading of
 *        values in the emulated eeprom memory
 */
UNS8 ReadEEProm(void)
{
  UNS8 RMDataSize = sizeof(ReadMemoryData);
  UNS16 recordSize = EEPROM_RECORD_SIZE;

  if (AddressRequest > (MAX_EEPROM_MEMORY - recordSize))
    return 2;

  if (lastRequestedAddress != AddressRequest || (triggerReadMemory == 1))
  {
    // set pattern to clear if read
    memset( ReadMemoryData, 0xA5, sizeof(ReadMemoryData));
    EEPROM_read( (UNS16)AddressRequest, ReadMemoryData, recordSize );
    // now copy requested address into the top of the array
    ReadMemoryData[RMDataSize - 4] = (UNS8) AddressRequest;
    ReadMemoryData[RMDataSize - 3] = (UNS8)(AddressRequest >> 8);
    ReadMemoryData[RMDataSize - 2] = (UNS8)(AddressRequest >> 16);
    ReadMemoryData[RMDataSize - 1] = (UNS8)(AddressRequest >> 24);

    lastRequestedAddress = AddressRequest;
    triggerReadMemory = 0;
    return 0;
  }
  return 1;
}

/**
 * @ingroup eeprom
 * @brief used with handshake through the OD at 0x2020 to allow remote writing of
 *        a single value in the emulated eeprom memory
 */
UNS8 WriteEEProm ( void )
{
  UNS8 status = 0;

  if (AddressRequest > MAX_EEPROM_MEMORY) // nominal address space
  {
    return 2;
  }
  else
  {
	EEPROM_open();
    EEPROM_write((UNS16)AddressRequest, &writeByteMemory, 1);
    EEPROM_commit();
  }

  return status;

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
