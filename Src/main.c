/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t UartRxBuf1[MAX_PAYLOAD];
uint8_t UartRxBuf2[MAX_PAYLOAD];
uint8_t *UartIsrBuf = UartRxBuf1;
uint8_t *UartRxBuf = 0;
uint8_t bPktReceived = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void UartRxTask(void);
void UartPrintf(const char *format, ...);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	int result;
	uint8_t event[EVT_QWIDTH];
	uint8_t payload[MAX_PAYLOAD];
	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART1_UART_Init();

	/* USER CODE BEGIN 2 */
	// initialize SerialComm
	SerialComm_Init();
	// initialize the pushbutton handler with mask byte.
	PushButton_Init(0x01);
	// start a timer routine: 10msec period, perpetual
	result = UsrTimer_Set(10, 0, UartRxTask);
	// UART output test
	UartPrintf("\r\n\t\tSystem Started: %d\r\n", result);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// check event queue
		if(Evt_DeQueue(event))
		{
			switch(event[0])
			{
			// pushbutton event ================================================
			// event[1]: button id
			// event[2]: PBTN_SCLK, _DCLK, _TCLK, _LCLK, _DOWN, _ENDN
			case EVT_PBTN_INPUT:

				if(event[2] == PBTN_SCLK)
				{
					UartPrintf("\r\nButton %d: single click.", event[1]);
				}
				else if(event[2] == PBTN_LCLK)
				{
					UartPrintf("\r\nButton %d: long click.", event[1]);
				}
				else if(event[2] == PBTN_DCLK)
				{
					UartPrintf("\r\nButton %d: double click.", event[1]);
				}
				else if(event[2] == PBTN_TCLK)
				{
					UartPrintf("\r\nButton %d: triple click.", event[1]);

					PushButton_SetMode(PUSHBTN_MODE_UDOWN, true);
					UartPrintf("\r\n --> Switch to up-down mode.");
				}
				else if(event[2] == PBTN_DOWN)
				{
					UartPrintf("\r\nButton %d: is being pressed.", event[1]);
				}
				else if(event[2] == PBTN_ENDN)
				{
					UartPrintf("\r\nButton %d: has been released.", event[1]);
					PushButton_SetMode(PUSHBTN_MODE_CLICK, true);
					UartPrintf("\r\n --> Switch to click mode.");
				}
				break;

			// uart event ======================================================
			// event[1]: payload size
			// event[2]: command
			// event[3:]: data
			case EVT_UART_RXPKT:

				if(event[2] == SYS_SRESET)
				{
					HAL_NVIC_SystemReset();
				}
				else if(event[2] == SYS_WRESET)
				{
					HAL_NVIC_SystemReset();
				}
				else if(event[2] == DIO_SETVAL)
				{
					if(event[3] == 0x01)
					{
						HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin,
								GPIO_PIN_SET);

						SerialComm_SendByte(PKT_ACK);
					}
				}
				else if(event[2] == DIO_RSTVAL)
				{
					if(event[3] == 0x01)
					{
						HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin,
								GPIO_PIN_RESET);

						SerialComm_SendByte(PKT_ACK);
					}
				}
				else if(event[2] == DIO_GETVAL)
				{
					if(event[3] == 0x01)
					{
						payload[0] = RPT_U08X01;
						payload[1] = (uint8_t)HAL_GPIO_ReadPin(
								TEST_LED_GPIO_Port, TEST_LED_Pin);
						SerialComm_SendPacket(payload, 2);
					}
				}
				break;

			default:
				break;
			}
		}
		else
		{
			// delay here is recommended not to call Evt_DeQueue too frequently
			HAL_Delay(10);
		}
	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Configure the main internal regulator output voltage
	*/
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	*/
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	*/
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : TEST_LED_Pin */
	GPIO_InitStruct.Pin = TEST_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(TEST_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TEST_BTN_Pin */
	GPIO_InitStruct.Pin = TEST_BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TEST_BTN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/** Printf() style UART output. Check the standard c library that comes
 * with the compiler for the scope of the format string it supports.
 */
void UartPrintf(const char *format, ...)
{
	char buffer[128];
	uint16_t size;
	va_list args;

	va_start(args, format);
	size = vsprintf(buffer, format, args);
	va_end(args);

	HAL_UART_Transmit(&huart1, (uint8_t*)buffer, size, 1000);
}

/** SysTick callback function override.
 */
void HAL_SYSTICK_Callback()
{
	// UsrTimer_Routine will have 1msec resolution
	UsrTimer_Routine();
}

/** This function returns the state (click/release) of the pushbuttons
 * in a uint8_t variable, each bit of which corresponds to a button.
 * Note that 1 implies the button is clicked (pressed) and 0 implies the
 * button is released.
 *
 * \return	pushbutton state packed in a uint8_t.
 */
uint8_t PushButton_Read()
{
	// button released
	if(HAL_GPIO_ReadPin(TEST_BTN_GPIO_Port, TEST_BTN_Pin))
	{
		return 0x00;
	}
	// button pressed
	else
	{
		return 0x01;
	}
}

/** UART Rx Task
 */
void UartRxTask()
{
	int i;
	uint8_t event[EVT_QWIDTH];

	// packet received
	if(bPktReceived)
	{
		// event id
		event[0] = EVT_UART_RXPKT;
		// event data size
		event[1] = UartRxBuf[1];

		// copy the payload
		for(i = 0; i< UartRxBuf[1]; i++)
		{
			event[2+i] = UartRxBuf[2+i];
		}
	
		// register the event
		Evt_EnQueue(event);
		// clear the flag
		bPktReceived = 0;
	}
}

/** Initialize SerialComm
 */
void SerialComm_Init()
{
	LL_USART_EnableIT_RXNE(huart1.Instance);
}

void SerialComm_RxRoutine()
{
	pkt_status status;

	status = SerialComm_Decoder(LL_USART_ReceiveData8(huart1.Instance),
			UartIsrBuf);

	if(status == PKT_RECEIVED)
	{
		// switch ping pong buffer
		if(UartIsrBuf == UartRxBuf1)
		{
			UartIsrBuf = UartRxBuf2;
			UartRxBuf = UartRxBuf1;
		}
		else
		{
			UartIsrBuf = UartRxBuf1;
			UartRxBuf = UartRxBuf2;
		}
		// raise flag
		bPktReceived = true;
	}
}

void SerialComm_SendByte(uint8_t byte)
{
	LL_USART_TransmitData8(huart1.Instance, byte);
}

void SerialComm_SendByteArray(uint8_t *buffer, int size)
{
	HAL_UART_Transmit(&huart1, buffer, size, 1000);
}

/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
