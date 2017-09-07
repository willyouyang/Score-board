/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 31/10/2015 14:45:13
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#define Set_LED(num) HAL_GPIO_WritePin(seven_pin[num].GPIOx, seven_pin[num].on_off, GPIO_PIN_SET)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t UART_Rx_Buffer;
uint8_t UART_Tx_Buffer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void Error_Handler(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/

void SysTick_Handler(void) 
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
	
	if(TimeDelay != 0)
		TimeDelay--;
	
	if(count_field == 0)
		buzzer();
	
	if(pause_flag)
	{
		if(count_pause != 0)
			count_pause--;
		else
			buzzer();
		return ;
	}
	
	if(count_24_sec != 0)
		count_24_sec--;
	else
	{
		if(!disable_24_flag)
		{
			//stop();
			buzzer();
		}
	}
	if(count_field != 0)
		count_field--;
	
}

/*----------------------------------------------------------------------------
  delays number of tick Systicks 
 *----------------------------------------------------------------------------*/
void Delay (int ntime) {                                              
  TimeDelay = ntime;
  while (TimeDelay!=0);
}

struct control_power control[MAX_7_SEGAMENT_LED_NUMBER];
struct control_power buzzers[Buzzer_NUM];
struct control_power seven_pin[7];

/*
start  0
stop   1
pause  2
long pause 3
short pause 4
restart  5
24 sec reset 6

A+1   7
A+2   8
A+3   9
A-1   a

B+1   b
B+2   c
B+3   d
B-1   e

min++  f
min--  g

sec++  h
sec--  i
scores swap  j
*/




void reset_led(){
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
}
void control_array_init()
{
	

	control[0].GPIOx=GPIOA;
	control[0].on_off=GPIO_PIN_9;	
	
	control[1].GPIOx=GPIOA;
	control[1].on_off=GPIO_PIN_10;
	
	control[2].GPIOx=GPIOA;
	control[2].on_off=GPIO_PIN_11;
	
	control[3].GPIOx=GPIOA;
	control[3].on_off=GPIO_PIN_12;
	
	control[4].GPIOx=GPIOA;
	control[4].on_off=GPIO_PIN_15;
	
	control[5].GPIOx=GPIOB;
	control[5].on_off=GPIO_PIN_3;
	
	control[6].GPIOx=GPIOB;
	control[6].on_off=GPIO_PIN_4;
	
	control[7].GPIOx=GPIOB;
	control[7].on_off=GPIO_PIN_5;
		
	control[8].GPIOx=GPIOB;
	control[8].on_off=GPIO_PIN_6;
	
	control[9].GPIOx=GPIOB;
	control[9].on_off=GPIO_PIN_7;
	
	seven_pin[0].GPIOx=GPIOA;
	seven_pin[0].on_off=GPIO_PIN_8;
	
	seven_pin[1].GPIOx=GPIOA;
	seven_pin[1].on_off=GPIO_PIN_5;
	
	seven_pin[2].GPIOx=GPIOA;
	seven_pin[2].on_off=GPIO_PIN_6;
	
	seven_pin[3].GPIOx=GPIOA;
	seven_pin[3].on_off=GPIO_PIN_7;
	
	seven_pin[4].GPIOx=GPIOB;
	seven_pin[4].on_off=GPIO_PIN_0;
	
	seven_pin[5].GPIOx=GPIOB;
	seven_pin[5].on_off=GPIO_PIN_1;
	
	seven_pin[6].GPIOx=GPIOA;
	seven_pin[6].on_off=GPIO_PIN_4;
}

void shownumber(unsigned int No_led, unsigned int number) 
{
	
	int count=0;
	reset_led();
	for(;count<MAX_7_SEGAMENT_LED_NUMBER;count++)//switch to the target led
	{
			HAL_GPIO_WritePin(control[count].GPIOx,control[count].on_off,GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(control[No_led].GPIOx,control[No_led].on_off,GPIO_PIN_RESET);
	switch(number)
	{
		case 0: 			
			Set_LED(0);
			Set_LED(1);
			Set_LED(2);
			Set_LED(3);
			Set_LED(4);
			Set_LED(5);
		break;
		case 1:
			Set_LED(4);
			Set_LED(5);
			break;
		case 2:
			Set_LED(0);
			Set_LED(2);
			Set_LED(3);
			Set_LED(5);
			Set_LED(6);
			break;
		case 3:
			Set_LED(0);
			Set_LED(3);
			Set_LED(4);
			Set_LED(5);
			Set_LED(6);
			break;
		case 4:
			Set_LED(1);
			Set_LED(4);
			Set_LED(5);
			Set_LED(6);
			break;
		case 5:
			Set_LED(0);
			Set_LED(1);
			Set_LED(3);
			Set_LED(4);
			Set_LED(6);
			break;
		case 6:
			Set_LED(0);
			Set_LED(1);
			Set_LED(2);
			Set_LED(3);
			Set_LED(4);			
			Set_LED(6);
			break;
		case 7:
			Set_LED(0);
			Set_LED(1);
			Set_LED(4);
			Set_LED(5);
			break;
		case 8:
			Set_LED(0);
			Set_LED(1);
			Set_LED(2);
			Set_LED(3);
			Set_LED(4);
			Set_LED(5);
			Set_LED(6);
			break;
		case 9:
			Set_LED(0);
			Set_LED(1);
			Set_LED(3);
			Set_LED(4);
			Set_LED(5);
			Set_LED(6);
			break;
		default:
			break;
	}
	//HAL_Delay(2);
}
int check = 0;
void refresh_led()
{
/*	if(check == 0)
	{
		start();
		check=1;
	}
*/
/*----------------24 sec----------------*/
	unsigned int k=0;
	if(count_pause!=0 && pause_flag==1)//pause
		k=(count_pause/1000+((count_pause%1000)!=0))%60;
	else 	
		k=(count_24_sec/1000+((count_24_sec%1000)!=0))%60;//equal to ceil
	shownumber(LED_8,k/10);
	HAL_Delay(3);
	shownumber(LED_9,k%10);
		HAL_Delay(2);
/*----------------time------------------*/	
	unsigned int j=(count_field/1000+((count_field%1000)!=0));
	unsigned int min=j/60,sec=j%60;
	shownumber(LED_0,sec%10);
	HAL_Delay(3);
	shownumber(LED_1,(sec/10)%6);
	HAL_Delay(2);
	shownumber(LED_2,min%10);
	HAL_Delay(2);
	if((min/10)%10 > 0){
		shownumber(LED_3,(min/10)%10);
		HAL_Delay(2);
	}
/*--------------score 1-----------------*/
  shownumber(LED_4,score[1]/10);
		HAL_Delay(2);
	shownumber(LED_5,score[1]%10);
		HAL_Delay(2);
/*--------------score 2-----------------*/	
	shownumber(LED_6,score[0]/10);
		HAL_Delay(2);
	shownumber(LED_7,score[0]%10);
		HAL_Delay(2);
	//HAL_Delay(4);
/*	shownumber(LED_0, 8);
	shownumber(LED_1, 8);
	shownumber(LED_2, 2);
	shownumber(LED_3, 3);
	shownumber(LED_4, 4);
	shownumber(LED_5, 5);
	shownumber(LED_0, 6);
	shownumber(LED_1, 7);
	shownumber(LED_2, 8);
	shownumber(LED_3, 9); */

return ;
}

void set_count_time(unsigned int time0,unsigned int time1,unsigned int time2,unsigned int time3,unsigned int time4)
{
	field_time=count_field=time0*1000;
	field_pause=time1*1000;
	time_24=count_24_sec=time2*1000;
	time_14=14*time_24/24;
	long_pause=time3*1000;
	short_pause=time4*1000;
	count_pause=0;
	return ;
}

void modify_scores(int *target,int modify)
{
	if((*target+modify)>0)
		*target+=modify;
	if(*target<0)  //prevent the condition of "2-3=-1"
		*target=0;
	return ;
}

void stop()
{
	pause_flag^=1;
	return;
}

void start()
{
	pause_flag^=1;
	return;
}

void pause(unsigned int type)
{
	if(type==0)//field pause
	{
		count_pause=field_pause;
	}
	else if(type==1)//short pause
	{
		count_pause=short_pause;
	}
	else if(type==2)//long pause
	{
		count_pause=long_pause;
	}
	pause_flag=1;
	return ;
}

void switch_field()
{	
	score[0]=(score[0] ^ score[1]);
	score[1]=(score[0] ^ score[1]);
	score[0]=(score[0] ^ score[1]);
	switch_flag^=1;
	return ;
}

void reset_24_sec()
{
	count_24_sec=time_24;
	return ;
}

void reset_14_sec()
{
	count_24_sec=time_14;
	return ;
}

void modify_time(int min,int sec)
{
	if(pause_flag==0)return ;
	if((count_field+(min*60*1000))>=0)
	count_field+=(min*60*1000);
	if((count_field+(sec*1000))>=0)
	count_field+=(sec*1000);
	return ;
}

void restart(void)
{
	count_field=field_time;
	count_24_sec=time_24;
	pause_flag=1;
	return ;
}

void recover()
{
	
}

void buzzer()
{
	int i = 0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
	for(i=0;i<100;i++);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
	return;
}

void disable_24_sec()

{
	disable_24_flag^=1;
	count_24_sec^=24;
	return ;
}

void send_time()
{
	unsigned int j=(count_field/1000+((count_field%1000)!=0));
	unsigned int min=j/60,sec=j%60;
	char time[10]={'<','!','>',(char)((min/10)%10+'0'),(char)(min%10+'0'),':',(char)((sec/10)%6+'0'),(char)(sec%10+'0'),'\r','\n'};
	// USART_puts(USART1,time);
}


/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

   if(HAL_UART_Receive_DMA(&huart2, (uint8_t *)&UART_Rx_Buffer, 1) != HAL_OK)
  {
    Error_Handler();
  }
	if(HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&UART_Tx_Buffer, 1) != HAL_OK)
  {
    Error_Handler();
  }

	set_count_time(600,60,24,60,30);
  
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
	
	
	control_array_init();
	
	start();
  /* Infinite loop */
  while (1)
  {
		 refresh_led();
//		HAL_GPIO_WritePin(control[0].GPIOx,control[0].on_off,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(seven_pin[0].GPIOx,seven_pin[0].on_off,GPIO_PIN_SET);
		
		// Set_LED(1);
		
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

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 
                           PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 PA12 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//LED_On(1);
	// check if the USART1 receive interrupt flag was set
	//if( USART_GetITStatus(USART2, UART_IT_RXNE) )
	//{
		static uint8_t cnt = 0; // this counter is used to determine the string length
		int i = 0;
		//char test = '1';
	  char test = UART_Rx_Buffer;
		//while( i < MAX_STRLEN ){
	//		t = buffer[i++];
			switch (test) 
			{
				case '0':start();
					// HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&UART_Rx_Buffer, 1);
					break;
				case '1':stop();
					break;
				case '2':pause(0);			
					break;
				case '3':pause(1);
					break;			
				case '4':pause(2);
					break;			
				case '5':restart();
					break;			
				case '6':reset_24_sec();
					break;		
				case '7':modify_scores(&score[0^switch_flag],1);send_time();
					break;			
				case '8':modify_scores(&score[0^switch_flag],2);
					break;				
				case '9':modify_scores(&score[0^switch_flag],3);
					break;	
				case 'a':modify_scores(&score[0^switch_flag],-1);
					break;	
				case 'b':modify_scores(&score[1^switch_flag],1);
					break;	
				case 'c':modify_scores(&score[1^switch_flag],2);
					break;	
				case 'd':modify_scores(&score[1^switch_flag],3);
					break;	
				case 'e':modify_scores(&score[1^switch_flag],-1);
					break;
				case 'k':modify_scores(&score[0^switch_flag],-2);
					break;
				case 'l':modify_scores(&score[0^switch_flag],-3);
					break;
				case 'm':modify_scores(&score[1^switch_flag],-2);
					break;
				case 'n':modify_scores(&score[1^switch_flag],-3);
					break;
				case 'f':modify_time(1,0);
						break;
				case 'g':modify_time(-1,0);
						break;
				case 'h':modify_time(0,1);
						break;
				case 'i':modify_time(0,-1);
						break;
				case 'j':switch_field();
						break;
				case 'z':reset_14_sec();
						break;		
				case 'y':disable_24_sec();
						break;

				default:
					break;
		}
			
		//HAL_StatusTypeDef HAL_UART_Receive_DMA(&huart, uint8_t *pData, uint16_t Size)

	//}
	
 }

 static void Error_Handler(void)
{
  while(1)
  {
  }
}
/* USER CODE END 4 */

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
