/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "adc.h"
#include "usart.h"
#include "gpio.h"
#include "queue.h"

/// CONVERT TO STRING: (HAVE TO INCLUDE LIB <string.h> <stdio.h>
#include <string.h>
#include <stdio.h>

#include "dma.h"
#include "tim.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

///-----------------------------------------------------------------------------
/// TYPEDEF AND VARIABLES:

/// SIZE OF UART RECEIVED DATA BUFFERS:
#define RxBuf_SIZE   512
#define MainBuf_SIZE 2048

/// IF WE CAN WE DONT USE VOLATILE AND GLOBAL VARIABLES IN RTOS
/// HERE IS NEEDED FOR UART FUNCTION AND INTERRUPT

/// UART RECEIVED DATA BUFFERS:
volatile uint8_t RxBuf[RxBuf_SIZE];
volatile uint8_t MainBuf[MainBuf_SIZE];



///-----------------------------------------------------------------------------
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

	///------------------------------------------------------------------------------------------------------------------------------
	/// CREATE HANDLERS:
	///------------------------------------------------------------------------------------------------------------------------------

/* USER CODE END Variables */
osThreadId IDLE_DEBUG_TASKHandle;
osThreadId ADC_TAKE_TASKHandle;
osThreadId UART_TX_TASKHandle;
osThreadId UART_RX_TASKHandle;
osThreadId ADC_CONFIG_TASKHandle;
osThreadId INT_TIMER_TASKHandle;
osThreadId INT_UART_TASKHandle;
osMessageQId ADC_QUEUE_UART_TXHandle;
osMessageQId UART_RX_QUEUE_ADC_TAKHandle;
osMessageQId UART_RX_QUEUE_UART_TXHandle;
osMessageQId UART_RX_QUEUE_ADC_CONFIGHandle;
osMessageQId INT_UART_QUEUE_UART_RXHandle;
osTimerId RTOS_TIMER_1Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */


void IDLE_DEBUG_TASK_INIT(void const * argument);
void ADC_TAKE_TASK_INIT(void const * argument);
void UART_TX_TASK_INIT(void const * argument);
void UART_RX_TASK_INIT(void const * argument);
void ADC_CONFIG_TASK_INIT(void const * argument);
void INT_TIMER_TASK_INIT(void const * argument);
void INT_UART_TASK_INIT(void const * argument);
void TIMER_CALLBACK(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of RTOS_TIMER_1 */
  osTimerDef(RTOS_TIMER_1, TIMER_CALLBACK);
  RTOS_TIMER_1Handle = osTimerCreate(osTimer(RTOS_TIMER_1), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

	///------------------------------------------------------------------------------------------------------------------------------
	/// CREATE QUEUES:
	///------------------------------------------------------------------------------------------------------------------------------

  /* Create the queue(s) */
  /* definition and creation of ADC_QUEUE_UART_TX */
  osMessageQDef(ADC_QUEUE_UART_TX, 1, uint16_t);
  ADC_QUEUE_UART_TXHandle = osMessageCreate(osMessageQ(ADC_QUEUE_UART_TX), NULL);

  /* definition and creation of UART_RX_QUEUE_ADC_TAKE */
  osMessageQDef(UART_RX_QUEUE_ADC_TAK, 16, uint16_t);
  UART_RX_QUEUE_ADC_TAKHandle = osMessageCreate(osMessageQ(UART_RX_QUEUE_ADC_TAK), NULL);

  /* definition and creation of UART_RX_QUEUE_UART_TX */
  osMessageQDef(UART_RX_QUEUE_UART_TX, 1, uint8_t);
  UART_RX_QUEUE_UART_TXHandle = osMessageCreate(osMessageQ(UART_RX_QUEUE_UART_TX), NULL);

  /* definition and creation of UART_RX_QUEUE_ADC_CONFIG */
  osMessageQDef(UART_RX_QUEUE_ADC_CONFIG, 1, uint8_t);
  UART_RX_QUEUE_ADC_CONFIGHandle = osMessageCreate(osMessageQ(UART_RX_QUEUE_ADC_CONFIG), NULL);

  /* definition and creation of INT_UART_QUEUE_UART_RX */
  osMessageQDef(INT_UART_QUEUE_UART_RX, 1, uint8_t);
  INT_UART_QUEUE_UART_RXHandle = osMessageCreate(osMessageQ(INT_UART_QUEUE_UART_RX), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

	///------------------------------------------------------------------------------------------------------------------------------
	/// CREATE THREADS:
	///------------------------------------------------------------------------------------------------------------------------------

  /* Create the thread(s) */
  /* definition and creation of IDLE_DEBUG_TASK */
  osThreadDef(IDLE_DEBUG_TASK, IDLE_DEBUG_TASK_INIT, osPriorityIdle, 0, 128);
  IDLE_DEBUG_TASKHandle = osThreadCreate(osThread(IDLE_DEBUG_TASK), NULL);

  /* definition and creation of ADC_TAKE_TASK */
  osThreadDef(ADC_TAKE_TASK, ADC_TAKE_TASK_INIT, osPriorityNormal, 0, 128);
  ADC_TAKE_TASKHandle = osThreadCreate(osThread(ADC_TAKE_TASK), NULL);

  /* definition and creation of UART_TX_TASK */
  osThreadDef(UART_TX_TASK, UART_TX_TASK_INIT, osPriorityNormal, 0, 2048);
  UART_TX_TASKHandle = osThreadCreate(osThread(UART_TX_TASK), NULL);

  /* definition and creation of UART_RX_TASK */
  osThreadDef(UART_RX_TASK, UART_RX_TASK_INIT, osPriorityNormal, 0, 128);
  UART_RX_TASKHandle = osThreadCreate(osThread(UART_RX_TASK), NULL);

  /* definition and creation of ADC_CONFIG_TASK */
  osThreadDef(ADC_CONFIG_TASK, ADC_CONFIG_TASK_INIT, osPriorityBelowNormal, 0, 128);
  ADC_CONFIG_TASKHandle = osThreadCreate(osThread(ADC_CONFIG_TASK), NULL);

  /* definition and creation of INT_TIMER_TASK */
  osThreadDef(INT_TIMER_TASK, INT_TIMER_TASK_INIT, osPriorityAboveNormal, 0, 128);
  INT_TIMER_TASKHandle = osThreadCreate(osThread(INT_TIMER_TASK), NULL);

  /* definition and creation of INT_UART_TASK */
  osThreadDef(INT_UART_TASK, INT_UART_TASK_INIT, osPriorityAboveNormal, 0, 128);
  INT_UART_TASKHandle = osThreadCreate(osThread(INT_UART_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_IDLE_DEBUG_TASK_INIT */
/**
  * @brief  Function implementing the IDLE_DEBUG_TASK thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_IDLE_DEBUG_TASK_INIT */



void IDLE_DEBUG_TASK_INIT(void const * argument)
{
  /* USER CODE BEGIN IDLE_DEBUG_TASK_INIT */
  /* Infinite loop */

	///------------------------------------------------------------------------------------------------------------------------------
	/// TASK TO DEBUG - TOGGLE DIODE:
	///------------------------------------------------------------------------------------------------------------------------------

  int i = 0;

  for(;;)
  {
	  osDelay(1);
	  i = i + 1;

	  if(i == 1000)
	  {
		 i = 0;

		 /// TOGGLE FIRST RED DIODE (DEBUG DIODE):
		 HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	  }
  }
  /* USER CODE END IDLE_DEBUG_TASK_INIT */
}

/* USER CODE BEGIN Header_ADC_TAKE_TASK_INIT */
/**
* @brief Function implementing the ADC_TAKE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADC_TAKE_TASK_INIT */



void ADC_TAKE_TASK_INIT(void const * argument)
{
  /* USER CODE BEGIN ADC_TAKE_TASK_INIT */
  /* Infinite loop */

	///------------------------------------------------------------------------------------------------------------------------------
	/// TASK TO TAKE ADC MEASUREMENT AND SEND DATA TO NEXT TASKS:
	///------------------------------------------------------------------------------------------------------------------------------

    ///@param MSG[10] 10 CHARACTER BUFFER TO TRANSMIT OVER QUEUE TO UART TASK (80bits):
    ///@param RAW_ADC_VALUE VARIABLE FOR  12bit ADC READING VALUE:

	/// ADC_TAKE_TASK VARIABLES:
	uint16_t RAW_ADC_VALUE; // VARIABLE FOR  12bit ADC READING VALUE:
	char MSG[10]; //10 CHARACTER BUFFER TO TRANSMIT OVER QUEUE TO UART TASK (80bits):

	  /* Infinite loop */
	  for(;;)
	  {
		  osDelay(1);

		  ///START ADC CONVERSION:
		  HAL_ADC_Start(&hadc1); /// PASSING ADC HANDLING ADDRES

		  /// WAIT TO ADC CONVERSION TO COMPLETE:
		  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

		  ///WHEN CONVERSION IS DONE GET 12bit VALUE: (FROM 0 TO 4095)
		  RAW_ADC_VALUE = HAL_ADC_GetValue(&hadc1);

		  /// SEND DATA TO QUEUE:
		  xQueueSend(ADC_QUEUE_UART_TXHandle, &RAW_ADC_VALUE, portMAX_DELAY);

	  }

	  ///------------------------------------------------------------------------------------------------------------------------------
	  /* USER CODE END ADC_TASK_INIT */

  /* USER CODE END ADC_TAKE_TASK_INIT */
}

/* USER CODE BEGIN Header_UART_TX_TASK_INIT */
/**
* @brief Function implementing the UART_TX_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_TX_TASK_INIT */



/// TASK FOR TERMINAL INTERFACE IN PC - UART TRANSFER DATA TASK:
void UART_TX_TASK_INIT(void const * argument)
{

  /* USER CODE BEGIN UART_TX_TASK_INIT */
	/* USER CODE BEGIN UART_TASK_INIT */

	///------------------------------------------------------------------------------------------------------------------------------
	/// TASK FOR TERMINAL INTERFACE IN PC - UART TRANSFER DATA TASK:
	///------------------------------------------------------------------------------------------------------------------------------


	/// THIS TASK HAVE BIGER STACK SIZE BECAUSE OF SPRINTF FUNCTION AND FLOAT COMPUTING
	/// WITH SMALER STACK SIZE THIS DOES NOT WORK

	///@param  MODE VARIABLE TO SELECT OPERATING MODE (SINGLE,CONTINOUS,INPUT)
	///@param RAW_ADC_VALUE 12bit VALU OF ADC MEASUREMENT
	///@param VOLTAGE VARIABLE FOR OUTCOME VOLTAGE VALUE
	///@param  CONTINOUS
	///@param  MSG[10] 10 CHARACTER BUFFER (80bits) TO TRANSMIT OVER UART

		uint8_t MODE = 0;
		uint16_t RAW_ADC_VALUE; /// 12bit ADC READING
		float VOLTAGE;
		uint8_t CONTINOUS = 0;
		char MSG[10]; //10 CHARACTER BUFFER (80bits) TO TRANSMIT OVER UART

		xQueueReceive(ADC_QUEUE_UART_TXHandle, &RAW_ADC_VALUE, portMAX_DELAY);
		xQueueReceive( UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);

	  /* Infinite loop */
	  for(;;)
	  {
	    osDelay(1);

	    /// START PAGE:
	    if(MODE == 0)
	    {
	    	CONTINOUS = 0;

	        sprintf(MSG,"\033c"); /// CLEAR TERMINAL WINDOW COMAND
	        HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"SELECT MODE:  %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"CLICK S TO SINGLE MEASUREMENT  %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"CLICK C TO CONTINOUS MEASUREMENT %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"CLICK I TO SELECT ADC INPUT %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			xQueueReceive( UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
	    }

	    ///SINGLE ADC MEASUREMENT PAGE:
	    else if(MODE==1)
	    {
	    	CONTINOUS = 0;

	    	sprintf(MSG,"\033c"); /// CLEAR TERMINAL WINDOW COMAND
	        HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

	    	sprintf(MSG," CLICK S TO START %\r\n");
	    	HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

	    	sprintf(MSG,"CLICK Q TO QUIT %\r\n");
	    	HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

	    	xQueueReceive( UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
	    }

	    /// START SINGLE ADC MEASUREMENT AND DISPLAY OUTCOMES:
	    else if(MODE==2)
	    {
	    	CONTINOUS = 0;

		   ///WAIT TO NEXT CLICK:
		   xQueueReceive( UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);

		   xQueueReceive(ADC_QUEUE_UART_TXHandle, &RAW_ADC_VALUE, portMAX_DELAY);

		   RAW_ADC_VALUE = (float)RAW_ADC_VALUE;
		   VOLTAGE = RAW_ADC_VALUE/820;

		   sprintf(MSG,"\033c");/// CLEAR TERMINAL WINDOW COMAND
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"ADC VALUE: %hu\r\n", RAW_ADC_VALUE);
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"VOLTAGE:  ");
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"%.2f\n:  ", VOLTAGE);
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"V  %\r ");
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"CLICK S TO NEXT MEASUREMENT %\r\n");
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"CLICK Q TO QUIT %\r\n");
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

	    }

	    /// CONTINOUS ADC MEASUREMENT SETTINGS:
	    else if(MODE == 4)
	    {
	    	 CONTINOUS = 1;
	    	 xQueueReceive( UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
	    }

	    /// ADC SELECT PIN MODE:
	    else if(MODE == 5)
	    {
	    	CONTINOUS = 0;

	    	sprintf(MSG,"\033c"); /// CLEAR TERMINAL WINDOW COMAND
	    	HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"SELECT PIN: %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"CLICK 1 TO SELECT PIN 1 %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"CLICK 2 TO SELECT PIN 2 %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"CLICK 3 TO SELECT PIN 3 %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"CLICK 4 TO SELECT PIN 4 %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			sprintf(MSG,"CLICK Q TO QUIT %\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

			xQueueReceive( UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
	    }

	    /// CONTINUES MEASUREMENT MODE:
	    if (CONTINOUS == 1)
	    {
	       /// OUTCOME - VOLTAGE VALUE - DIPLAYED EVERY 300ms

	       osDelay(300); /// DELAY INSTEAD OF TIMER
	       xQueueReceive(ADC_QUEUE_UART_TXHandle, &RAW_ADC_VALUE, portMAX_DELAY);

		   RAW_ADC_VALUE = (float)RAW_ADC_VALUE;
		   VOLTAGE = RAW_ADC_VALUE/820;

		   sprintf(MSG,"\033c"); /// CLEAR TERMINAL WINDOW COMAND
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"ADC VALUE: %hu\r\n", RAW_ADC_VALUE);
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"VOLTAGE: %f ", VOLTAGE);
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"V  %\r\n ");
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		   sprintf(MSG,"CLICK Q TO QUIT %\r\n");
		   HAL_UART_Transmit(&huart1, (uint8_t*)MSG, strlen(MSG), HAL_MAX_DELAY);

		  /// IF THERE IS NO DATA IN 10 TICKS GO FURTHER:
	   	   xQueueReceive( UART_RX_QUEUE_UART_TXHandle, &MODE, 10);

	     }

	  }

	  ///------------------------------------------------------------------------------------------------------------------------------
	  /* USER CODE END UART_TASK_INIT */
  /* USER CODE END UART_TX_TASK_INIT */
}

/* USER CODE BEGIN Header_UART_RX_TASK_INIT */
/**
* @brief Function implementing the UART_RX_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_RX_TASK_INIT */




void UART_RX_TASK_INIT(void const * argument)
{
  /* USER CODE BEGIN UART_RX_TASK_INIT */
	  /* USER CODE BEGIN UART_READ_TASK_INIT */

	///------------------------------------------------------------------------------------------------------------------------------
	/// TASK TO PROCESS RECEIVED DATA AND SELECT OPERATING MODE:
	///------------------------------------------------------------------------------------------------------------------------------

	///@param MODE VARIABLE TO SELECT OPERATING MODE (SINGLE,CONTINOUS,INPUT)
	///@param RX_DATA RECEIVED 8bit DATA FROM UART
	///@param ADC_PIN VARIABLE TO SELECT ADC INPUT PIN

	// VARIABLES FOR THIS TASK:
	uint8_t RX_DATA; //RECEIVED 8bit DATA FROM UART
	uint8_t MODE = 0;
	uint8_t ADC_PIN;

	  /* Infinite loop */
	  for(;;)
	  {
		  osDelay(10);

		  /// RECEIVING DATA FROM QUEUE - DATA FROM UART INTERRUPT TASK:
		  xQueueReceive(INT_UART_QUEUE_UART_RXHandle, &RX_DATA, portMAX_DELAY);

		  if(MODE == 0)
		  {
			  /// 'S'- SINGLE MEASUREMENT 'S' - START
			  if(RX_DATA == 's' || RX_DATA == 'S')
			  {
				MODE = 1; ///SINGLE MEASUREMENT PAGE MODE
				xQueueSend(UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
				RX_DATA = 0;
			  }

			  /// 'C' - CONTINOUS MEASUREMENT
			  else if(RX_DATA == 'c' || RX_DATA == 'C')
			  {
				MODE = 4; ///CONTINOUS MEASUREMENT MODE
				xQueueSend(UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
			  }

			  /// 'I' - INPUT
			  else if(RX_DATA == 'i' || RX_DATA == 'I')
			  {
				MODE = 5; /// SELECT ADC PIN MODE
				xQueueSend(UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
			  }
		  }

		  /// IF MODE IS: SINGLE MEASUREMENT PAGE - THEN MAKE SINGLE MEASUREMENT:
		  if(MODE == 1)
		  {
			  if(RX_DATA == 's' || RX_DATA == 'S')
			  {
				MODE = 2; ///SINGLE MEASUREMENT 'START' MODE
				xQueueSend(UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
			  }
		  }

		  /// TAKE NEXT SINGLE MEASUREMENT:
		  if(MODE == 2)
		  {
			  if(RX_DATA == 's' || RX_DATA == 'S')
			  {
				xQueueSend(UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
			  }
		  }

		  /// SELECT ADC PIN NUMBER FROM 1 TO 4:
		  if(MODE == 5)
		  {
			  if(RX_DATA > 47 && RX_DATA < 53) /// ASCI: 0,1,2,3
			  {
				 ADC_PIN = (RX_DATA - 48);
				 xQueueSend(UART_RX_QUEUE_ADC_CONFIGHandle, &ADC_PIN, portMAX_DELAY);
			  }
		  }

		  /// RECEIVING 'Q' - BACK TO START PAGE (Q = QUIT)
		  if(RX_DATA == 'q' || RX_DATA == 'Q')
		  {
			  MODE = 0; /// START PAGE MODE
			  /// SEND DATA TO QUEUE - SELECTED WORK MODE:
			  xQueueSend(UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);
		  }

		  RX_DATA = 0;

		  /// SEND DATA TO QUEUE - SELECTED WORK MODE:
		  xQueueSend(UART_RX_QUEUE_UART_TXHandle, &MODE, portMAX_DELAY);

	  }

  /* USER CODE END UART_RX_TASK_INIT */
}

/* USER CODE BEGIN Header_ADC_CONFIG_TASK_INIT */
/**
* @brief Function implementing the ADC_CONFIG_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ADC_CONFIG_TASK_INIT */




///
/// ADC_CONFIG_TASK_INIT
void ADC_CONFIG_TASK_INIT(void const * argument)
{
  /* USER CODE BEGIN ADC_CONFIG_TASK_INIT */
  /* Infinite loop */

	///------------------------------------------------------------------------------------------------------------------------------
	/// TASK TO SELECT ADC PIN (DIODES INSTEAD OF RELAYS):
	///------------------------------------------------------------------------------------------------------------------------------

	///@param ADC_PIN VARIABLE TO SELECT ADC INPUT PIN

  uint8_t ADC_PIN = 0;

  for(;;)
  {
    osDelay(1);

    /// RECEIVING DATA FORM QUEUE:
    xQueueReceive( UART_RX_QUEUE_ADC_CONFIGHandle, &ADC_PIN, portMAX_DELAY);

    if(ADC_PIN == 1)
    {
    	/// GREEN DIODE ON - THE REST OFF:
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_RESET); //GREEN DIODE
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET); //RED DIODE
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET); //BLUE DIODE
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET); //YELLOW DIODE
    }
    if(ADC_PIN == 2)
   {
    	/// RED DIODE ON - THE REST OFF:
     	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET); //GREEN DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET); //RED DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET); //BLUE DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET); //YELLOW DIODE

   }
    if(ADC_PIN == 3)
   {
    	/// BLUE DIODE ON - THE REST OFF:
     	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET); //GREEN DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET); //RED DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET); //BLUE DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET); //YELLOW DIODE

   }
    if(ADC_PIN == 4)
   {
    	/// YELLLOW DIODE ON - THE REST OFF:
     	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,GPIO_PIN_SET); //GREEN DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET); //RED DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14,GPIO_PIN_SET); //BLUE DIODE
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET); //YELLOW DIODE
   }

  }
  /* USER CODE END ADC_CONFIG_TASK_INIT */
}

/* USER CODE BEGIN Header_INT_TIMER_TASK_INIT */
/**
* @brief Function implementing the INT_TIMER_TASK thread.
* @param argument: Not used
* @retval None
*/



/* USER CODE END Header_INT_TIMER_TASK_INIT */



void INT_TIMER_TASK_INIT(void const * argument)
{
  /* USER CODE BEGIN INT_TIMER_TASK_INIT */
  /* Infinite loop */

	///------------------------------------------------------------------------------------------------------------------------------
	/// TASK TO HANDLE THE INTERRUPT FROM TIMER - 1 SECOND:
	///------------------------------------------------------------------------------------------------------------------------------


  for(;;)
  {

	  /// TASK IS WAITING AND WILL RUN WHEN ISR OCCOUR

	  ///TASK SUSPENDS ITSELF:
	  vTaskSuspend(NULL);

	  ///HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);

	  /// START TIMER AGAIN:
	  HAL_TIM_Base_Start_IT(&htim11);

  }
  /* USER CODE END INT_TIMER_TASK_INIT */
}

/* USER CODE BEGIN Header_INT_UART_TASK_INIT */
/**
* @brief Function implementing the INT_UART_TASK thread.
* @param argument: Not used
* @retval None
*/




/* USER CODE END Header_INT_UART_TASK_INIT */



void INT_UART_TASK_INIT(void const * argument)
{
	///------------------------------------------------------------------------------------------------------------------------------
	/// TASK TO HANDLE THE INTERRUPT FROM INCOMING DATA FROM UART:
	///------------------------------------------------------------------------------------------------------------------------------

	///@param RX_DATA RECEIVED 8bit DATA FROM UART

	uint8_t RX_DATA; // 8bit DATA RECEIVED FROM UART:

	/// START THE DMA:
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) RxBuf, RxBuf_SIZE);

  for(;;)
  {
	  /// TASK IS WAITING AND WILL RUN WHEN ISR OCCOUR

	  ///TASK SUSPENDS ITSELF:
	  vTaskSuspend(NULL);

	  ///TOGGLE BLUE DIODE FOR DEBUG:
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);

	  RX_DATA = RxBuf[0];

	  /// SEND RECEIVED DATA TO QUEUE:
	  xQueueSend(INT_UART_QUEUE_UART_RXHandle, &RX_DATA, portMAX_DELAY);

	  /// START THE DMA AGAIN:
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) RxBuf, RxBuf_SIZE);
  }


  /* USER CODE END INT_UART_TASK_INIT */
}

/* TIMER_CALLBACK function */
void TIMER_CALLBACK(void const * argument)
{
  /* USER CODE BEGIN TIMER_CALLBACK */

  /* USER CODE END TIMER_CALLBACK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */






///-------------------------------------------------------------------------------------------------
///   INTERRUPTS CALLBACKS:
///-------------------------------------------------------------------------------------------------

/// CALLBACK FROM TIMER INTERRUPT - 1 SECOND:
void HAL_TIM_PeriodicElapsedCallback (TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM11)
	{
     /// WZNAWIANIE ZADANIA DO OBSLUGI TEGO PRZERWANIA:
		BaseType_t CHECK_VAR;
	 /// xTaskResumeFromISR( TASK_HANDLER)
		CHECK_VAR = xTaskResumeFromISR(INT_TIMER_TASKHandle);
		portYIELD_FROM_ISR(CHECK_VAR);
	}
}

/// CALLBAC FROM UART RECEIVING DATA INTERRUPT:
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
 ///-------------------------------------------------------
 /// WZNAWIANIE ZADANIA DO OBSLUGI TEGO PRZERWANIA:
    BaseType_t CHECK_VAR;
 /// xTaskResumeFromISR( TASK_HANDLER)
	CHECK_VAR = xTaskResumeFromISR(INT_UART_TASKHandle);
	portYIELD_FROM_ISR(CHECK_VAR);
 ///-------------------------------------------------------
}

///-------------------------------------------------------------------------------------------------
///   INTERRUPTS CALLBACKS END
///-------------------------------------------------------------------------------------------------


/* USER CODE END Application */
