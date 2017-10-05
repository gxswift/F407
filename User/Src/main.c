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
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "string.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"

#include "lwip.h"
#include "httpserver-netconn.h"

#include "eth.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
//extern ETH_HandleTypeDef heth;
ETH_HandleTypeDef heth;

extern UART_HandleTypeDef huart1;


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static void vTaskTaskUserIF(void *pvParameters);
static void vTaskLed(void *pvParameters);
static void vTaskMsgPro(void *pvParameters);
static void AppTaskCreate(void);


static TaskHandle_t xHandleTaskUserIF = NULL;
static TaskHandle_t xHandleTaskLed = NULL;
static TaskHandle_t xHandleTaskMsgPro = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint32_t t = 0;
uint8_t Rx[200];
uint8_t Tx[200];
//D17 G7
void LED_Func()
{
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12);
	HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
}



void delay(uint32_t d)
{
	while(d--);
}
//-----------------------------------------------------
typedef struct
{
	uint8_t Read;
	uint8_t Write;
	uint8_t Lenth;
	uint8_t Data[256];
}RING;
RING TX,RX;

void Buffer_Init(void )
{
	memset(&RX,0,sizeof(RING));
	memset(&TX,0,sizeof(RING));
}
void Write_Data(RING *ring,uint8_t data)
{
	ring->Data[ring->Write] = data;
	ring->Write++;
	ring->Lenth++;
}
uint8_t Read_Data(RING *ring) 
{
	if(ring->Lenth)
		ring->Lenth--;
	return ring->Data[ring->Read++];
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart1, Rx, 1);
	if(RX.Lenth<255)
		Write_Data(&RX,Rx[0]);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t temp;
	if (RX.Lenth)
	{
		temp = Read_Data(&RX);
		HAL_UART_Transmit_IT(huart,&temp,1);
	//	while(HAL_OK != HAL_UART_Transmit_IT(huart,&temp,1));
	}
}
void Send(uint8_t *data)
{
	while(*data)
	{
		Write_Data(&RX,*data++);
	}
}
//-----------------------------------------------------
//printf重定向
#include "stdio.h"
int fputc(int ch, FILE *f)
 {
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,100);
	return ch;
 }

//-----------------------------------------------------
void my_smtp_result_fn(void *arg, u8_t smtp_result, u16_t srv_err, err_t err)
{
   printf("mail (%p) sent with results: 0x%02x, 0x%04x, 0x%08x\n", arg,
         smtp_result, srv_err, err);
 }
/*
 static void my_smtp_test(void)
 {
   smtp_set_server_addr("mymailserver.org");
 //-> set both username and password as NULL if no auth needed
   smtp_set_auth("username", "password");
  smtp_send_mail("sender", "recipient", "subject", "body", my_smtp_result_fn,
                  some_argument);
 }
 */

void HAL_Delay(__IO uint32_t Delay)
{
	vTaskDelay(Delay);
}
static void vTaskTaskUserIF(void *pvParameters)
{
	uint8_t temp;
	while(1)
	{

		if (RX.Lenth)//有接收到数据，开启发送中断
		{
			temp = Read_Data(&RX);
			HAL_UART_Transmit_IT(&huart1,&temp,1);
		//	while(HAL_OK != HAL_UART_Transmit_IT(&huart1,&temp,1));
		}
				vTaskDelay(100);
		/* USER CODE BEGIN 3 */
	}
}
uint32_t reg[32];
char ch[30];
static void vTaskLed(void *pvParameters)
{
	while(1)
	{
		HAL_GPIO_TogglePin(led_GPIO_Port,led_Pin);
		vTaskDelay(500);

	}
}

static void vTaskMsgPro(void *pvParameters)
{
	while(1)
	{
		vTaskDelay(300);
	}
}
static void AppTaskCreate (void)
{
	xTaskCreate(vTaskTaskUserIF,
							"vTaskTaskUserIF",
							512,
							NULL,
							1,
							&xHandleTaskUserIF);
	
	xTaskCreate(vTaskLed,
							"vTaskLed",
							512,
							NULL,
							2,
							&xHandleTaskLed);
	
	xTaskCreate(vTaskMsgPro,
							"vTaskMsgPro",
							512,
							NULL,
							3,
							&xHandleTaskMsgPro);
							
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
		__set_PRIMASK(1);
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
	HAL_GPIO_WritePin(reset_GPIO_Port,reset_Pin,0);
	delay(1000000);
	HAL_GPIO_WritePin(reset_GPIO_Port,reset_Pin,1);
	delay(1000000);
	MX_LWIP_Init();
  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
	printf("printf重定向\r\n");//重定向正常
  /* USER CODE END 2 */
/*	memcpy(Tx,"USART TEST\r\n",18);
//	HAL_UART_Transmit_IT(&huart1, Tx, sizeof(Tx));
	Send(Tx);


	
	for(uint8_t i=0;i<32;i++){
	HAL_ETH_ReadPHYRegister(&heth, i, &reg[i]);
		sprintf(ch,"寄存器%d:%#8x\r\n",i,reg[i]);
		HAL_UART_Transmit(&huart1,(uint8_t*)ch,20,200);
		delay(1000000);
	}*/
	//寄存器正常
//	HAL_UART_Receive_IT(&huart1, Rx, 1);
	#ifdef USE_DHCP
	xTaskCreate(DHCP_thread,
							"DHCP_Thread",
							512,
							NULL,
							3,
							NULL);
	#endif
	AppTaskCreate();
	
 http_server_netconn_init();
	vTaskStartScheduler();
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE END 3 */
	}
}


/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */

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
