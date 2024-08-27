/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Wireless Ping
  * A FreeRTOS program that utilizes a Spirit1 RF module to be a part of a
  * mesh network designed to be a simple chat. Functionality consists of
  * sending private messages, sending broadcast messages, and viewing
  * which nodes are online. The UI is rudimentary and displayed over UART.
  ******************************************************************************
**/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "spi.h"
#include "gpio.h"
#include "spsgrf.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "node.h"
#include "usart.h"
#include "utils.h"

TaskHandle_t LogOnHandler, ReceiveHandler, UpdateListHandler, HeartbeatHandler, InputHandler;
SemaphoreHandle_t xMutex;
QueueHandle_t xMessageQueue;

volatile SpiritFlagStatus xTxDoneFlag = S_RESET;
volatile SpiritFlagStatus xRxDoneFlag = S_RESET;
volatile unsigned long ulHighFrequencyTimerTicks; // used for task runtime stats

char message[MAXBUF] = {'\0'}; // for getting message from term
char username[21] = {'\0'}; // to keep my username
uint8_t loggedOn = 0; // flag to know whether or not I've logged on in USART ISR
uint16_t msgIndex = 0; // for keeping track of message index in USART ISR

void SystemClock_Config(void);

/* Task Declarations */
void logOnTask(void * argument);
void receiveTask(void *argument);
void updateListTask(void *argument);
void heartbeatTask(void *argument);
void inputTask(void *argument);

/* Method Declarations */
void configureTimerForRunTimeStats(void);
void RTOS_Stats_Timer_Init(void);

int _write(int file, char *ptr, int len) // FOR PRINTF DEBUGGING
{
   for(int i=0; i<len; i++)
      ITM_SendChar((*ptr++));
   return len;
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();

  configureTimerForRunTimeStats();
  SPSGRF_Init();
  USART_Init();
  UART_SendString(clear_screen);
  UART_SendString(move_cursor_home);
  getUsername();

  /* FreeRTOS Initialization */
  // Create Mutex
  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL) { while(1); }

  // Create Message Queue
  xMessageQueue = xQueueCreate(2, sizeof(message_t));
  if (xMessageQueue == NULL) { while(1); }

	// Create tasks
	BaseType_t retVal;
	retVal = xTaskCreate(logOnTask, "LogOnTask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 8, &LogOnHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(receiveTask, "ReceiveTask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 6, &ReceiveHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(updateListTask, "UpdateListTask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 6, &UpdateListHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(heartbeatTask, "HBTask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 6, &HeartbeatHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	retVal = xTaskCreate(inputTask, "InputTask", configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 7, &InputHandler);
	if (retVal != pdPASS) { while(1);}	// check if task creation failed

	vTaskStartScheduler();

  while (1)
  {

  }
}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/*TASK DEFINITIONS*/
void logOnTask(void *argument)
{
	// Set broadcast address
	 SpiritPktStackSetDestinationAddress(BROADCAST);

    // Send the payload
    xTxDoneFlag = S_RESET;
    uint8_t payload[22] = {0};
    uint16_t bytesToSend = createPDU(LOG_ON, (uint8_t *) payload, NULL, 0);
    SPSGRF_StartTx((uint8_t *) payload, bytesToSend);
    while(!xTxDoneFlag) {};

    SpiritCmdStrobeRx();

    vTaskDelete(NULL);
}

void receiveTask(void *argument)
{
	for(;;)
	{
		xRxDoneFlag = S_RESET;
		SPSGRF_StartRx();
		while (!xRxDoneFlag);

		if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
		{
			uint8_t srcAddress = SpiritPktStackGetReceivedSourceAddress();
			uint8_t destAddress = SpiritPktStackGetReceivedDestAddress();

			char payload[MAXBUF] = {0};
			uint16_t bytesRecv = SPSGRF_GetRxData((uint8_t *)payload);
			processPayload(srcAddress, destAddress, payload, bytesRecv);

			xSemaphoreGive(xMutex);
		}

	}
}

void inputTask(void *argument)
{
	for (;;)
	{
		if (xSemaphoreTake(xMutex, 0) == pdTRUE)
		{
			// check queue for input backlog
			message_t msg_t;
			if (xQueueReceive(xMessageQueue, &msg_t, 0) == pdTRUE)
			{
				processInput(&msg_t);
				UART_SendString("$: ");
			}

			xSemaphoreGive(xMutex);
		}

		vTaskDelay(pdMS_TO_TICKS(200)); // to ensure other tasks don't starve
	}
}

void updateListTask(void *argument)
{
	for(;;)
	{
		decrementTimers();
		removeOfflineNodes();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void heartbeatTask(void *argument)
{
	for(;;)
	{
		if (xSemaphoreTake(xMutex, 0) == pdTRUE)
		{
			SpiritCmdStrobeSabort();

			// Set broadcast address
			SpiritPktStackSetDestinationAddress(BROADCAST);

			// Send the payload
			uint8_t payload[22] = {0};
			uint16_t bytesToSend = createPDU(HEARTBEAT, (uint8_t *) payload, NULL, 0);
			xTxDoneFlag = S_RESET;
			SpiritCmdStrobeFlushTxFifo();
			SPSGRF_StartTx(payload, bytesToSend);
			while(!xTxDoneFlag);

			xSemaphoreGive(xMutex);

		    SpiritCmdStrobeRx();

			vTaskDelay(pdMS_TO_TICKS(5000));
		}

	}
}

/*-------------INTERRUPT HANDLERS------------------------------*/
void USART2_IRQHandler(void)
{
    if (USART2->ISR & USART_ISR_RXNE)
    {
    	if (!loggedOn) // not logged on, fill username buffer
    	{
    		if (USART2->RDR == '\r') // username completed
    		{
    			msgIndex = 0; //reset index
    			loggedOn = 1; // set flag high to break out of log on loop
    		}
			else if (USART2->RDR == 0x7F) // backspace
			{
				USART2->TDR = USART2->RDR; // echo

				if (msgIndex > 0)
				{
					msgIndex--;
				}
			}
    		else // still building username
    		{
    			USART2->TDR = USART2->RDR; // echo char
    			username[msgIndex] = USART2->RDR; // add to str
    			msgIndex++; // inc index
    		}
    	}
    	else // logged on, fill message buffer
    	{
    		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

			if (USART2->RDR == '\r') // message completed, send msg to Queue
			{
				UART_SendString("\r\n");
				message_t msg_t;
				memcpy(msg_t.message, message, msgIndex);
				msg_t.message[msgIndex] = '\0';
				msg_t.len = msgIndex;

				xQueueSendFromISR(xMessageQueue, &msg_t, &xHigherPriorityTaskWoken);

				//memset(message, '\0', sizeof(message)); // reset message
				msgIndex = 0; //reset message index

			}
			else if (USART2->RDR == 0x7F)// backspace
			{
				USART2->TDR = USART2->RDR; // echo
				if (msgIndex > 0)
				{
					msgIndex--;
				}
			}
			else
			{
				USART2->TDR = USART2->RDR; // copy received char to transmit buffer to echo
				message[msgIndex] = USART2->RDR; //put char into string
				msgIndex++; // increment current string index
			}

			SpiritCmdStrobeRx();
			portYIELD_FROM_ISR(&xHigherPriorityTaskWoken);
    	}
    }
}

/*GIVEN*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  SpiritIrqs xIrqStatus;

  if (GPIO_Pin != SPIRIT1_GPIO3_Pin)
  {
    return;
  }

  SpiritIrqGetStatus(&xIrqStatus);
  if (xIrqStatus.IRQ_TX_DATA_SENT)
  {
    xTxDoneFlag = S_SET;
  }
  if (xIrqStatus.IRQ_RX_DATA_READY)
  {
    xRxDoneFlag = S_SET;
  }
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    SpiritCmdStrobeRx();
  }
}

/* Configure Timer to interrupt 100 kHz (100 times every Tick) */
void RTOS_Stats_Timer_Init(void)
{
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM5EN);	// turn on TIM5
	TIM5->DIER |= (TIM_DIER_UIE);			// enable interrupts
	TIM5->SR  &= ~(TIM_SR_UIF);				// clear interrupt flag
	TIM5->ARR = CLK_FREQUENCY/100000 - 1;
	TIM5->CR1 |= TIM_CR1_CEN;			    // start timer

	// enable interrupts
	NVIC->ISER[0] = (1 << (TIM5_IRQn & 0x1F));
}

/* Timer 5 is used to collect runtime stats for FreeRTOS tasks*/
void TIM5_IRQHandler(void)
{
	TIM5->SR &= ~(TIM_SR_UIF);
	ulHighFrequencyTimerTicks++;
}

/* Built in functions for using FreeRTOS runtime stats need to be defined*/
void configureTimerForRunTimeStats(void)
{
    ulHighFrequencyTimerTicks = 0;
    RTOS_Stats_Timer_Init();
}

unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
