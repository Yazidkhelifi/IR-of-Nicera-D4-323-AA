/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
#include <math.h>
#include "stm32l0xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_10BIT_ADDR  0x302
#define sueil  100
#define max_ratio_limit 3
#define t2_t1  5000
#define SDA_PORT      GPIOB
#define SDA_PIN       GPIO_PIN_11
#define SCL_PORT      GPIOB
#define SCL_PIN       GPIO_PIN_10


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int16_t sensor_value=0;
static volatile uint8_t current_buf = 0;
static char buf[2][64];

static uint32_t positif_pic_time=0;
static uint32_t negatif_pic_time=0;
static uint16_t time_between_two_pic =0;
static uint8_t get_positif_pic=0;
static uint8_t get_negatif_pic=0;

static int16_t negatif_peak_val = -sueil;
static int16_t positif_peak_val = sueil;




static enum { IDLE, positif_PEAK,negatif_PEAK, calculate_ratio } state = IDLE;

void Sensor_Set_SDA_To_Interrupt(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);

    __HAL_GPIO_EXTI_CLEAR_IT(SDA_PIN);

    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

void Sensor_Set_SDA_To_I2C(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

    GPIO_InitStruct.Pin = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    GPIO_InitStruct.Alternate = GPIO_AF6_I2C2;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}

volatile uint8_t pir_data_ready = 0;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SDA_PIN)
    {
        pir_data_ready = 1;
    }
}



void PIR_Write_Configuration(uint8_t threshold, uint8_t trigger_mode, uint8_t filter_step, uint8_t filter_type)
{
    uint8_t config_frame[7] = {0};

    config_frame[0] = 0x00;
    config_frame[1] = 0x00;
    config_frame[2] = 0x00;
    config_frame[3] = ((filter_type & 0x07) << 2) | (filter_step & 0x03);
    config_frame[4] = ((trigger_mode & 0x01) << 7) | ((threshold >> 1) & 0x7F);
    config_frame[5] = (threshold & 0x01) << 7;
    config_frame[6] = 0x00;

    HAL_I2C_Master_Transmit(&hi2c2, SENSOR_10BIT_ADDR, config_frame, 7, 100);
}


int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, 100);
    return len;
}





int16_t PIR_Read_PeakHold(void)
{
    uint8_t rx[2];
    int16_t value;
    if (HAL_I2C_Master_Receive(&hi2c2, SENSOR_10BIT_ADDR, rx, 2, 10000) != HAL_OK)
    {   return -999;}
    value = ((rx[0] & 0x0F) << 8) | rx[1];
    if (value & 0x0800)
        value |= 0xF000;
    return value;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
   HAL_Delay(1000);



  //PIR_Write_Configuration(5,0,3,1);// Type c, Step 2
  //PIR_Write_Configuration(5,0,1,0);// Type B, Step 1
  //PIR_Write_Configuration(5,0,1,7);// type A step 1
  //PIR_Write_Configuration(5,0,0,0);// Type B, Step 3

  PIR_Write_Configuration(28,0,3,1);
  HAL_Delay(100);
 // PIR_Write_Configuration(2,0,3,1);

  //Sensor_Set_SDA_To_Interrupt();
  //Sensor_Set_SDA_To_I2C();





  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
/*
	      HAL_Delay(100);
	  	  printf("Scanning...\r\n");
	  	  for (uint16_t addr = 0x300; addr <= 0x304; addr++) {
	  	      if (HAL_I2C_IsDeviceReady(&hi2c2, addr, 3, 10000) == HAL_OK) {
	  	          printf("Device found at: 0x%03X\r\n", addr);
	  	      } else {
	  	          printf("Nothing at: 0x%03X\r\n", addr);
	  	      }
	  	  }

*/

/*
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET );


	   	   	  sensor_value = PIR_Read_PeakHold();
	   	   	  current_buf = !current_buf;
			  int len = snprintf(buf[current_buf], sizeof(buf[0]), " %d\r\n", sensor_value);
			  HAL_UART_Transmit(&huart1, (uint8_t*)buf[current_buf], len, 1000);
			  HAL_Delay(60);


*/


	  /*
	  temps_filter_change = HAL_GetTick()-temps_debut;
	  if (temps_filter_change>=10000){
		  temps_filter_change=HAL_GetTick();
		   PIR_Write_Configuration(5,0,3,0); // step 2 filtre B

	  }

*/







/*

  if (pir_data_ready)
		  {
	  uint32_t startTick = HAL_GetTick();

	  while(HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN) == GPIO_PIN_RESET)
	  {
	      if((HAL_GetTick() - startTick) >= 1)
	      {
	          break;
	      }
	  }

	  Sensor_Set_SDA_To_I2C();
	  PIR_Write_Configuration(2, 0, 3, 1);
	  HAL_Delay(10);*/



	  sensor_value = PIR_Read_PeakHold();
	  current_buf = !current_buf;
	  int len = snprintf(buf[current_buf], sizeof(buf[0]), " %d\r\n", sensor_value);
	  HAL_UART_Transmit(&huart1, (uint8_t*)buf[current_buf], len, 1000);


	  switch (state) {
	      case IDLE:
	          if (sensor_value >= positif_peak_val) {
	              positif_peak_val = sensor_value;
	              get_positif_pic=1;
	              state = positif_PEAK;

	          }
	          else if (sensor_value <= negatif_peak_val){
	        	  negatif_peak_val = sensor_value;
	        	  get_negatif_pic=1;
                  state = negatif_PEAK;}
	          else { state = IDLE;}
	          break;

	      case positif_PEAK:
			  if (sensor_value > positif_peak_val){ positif_peak_val = sensor_value; positif_pic_time = HAL_GetTick();}
	              if (sensor_value < sueil) {
	            	  if ( get_negatif_pic==1){state = calculate_ratio;}
	            	  else if ( get_negatif_pic==0) {state = negatif_PEAK;}
	              }
	          break;

	      case negatif_PEAK:
	    	  if (sensor_value < negatif_peak_val){ negatif_peak_val = sensor_value; negatif_pic_time = HAL_GetTick();}
					  if (sensor_value > -sueil  ) {
						  if ( get_positif_pic==1){state = calculate_ratio;}
						  else if ( get_positif_pic==0) {state = positif_PEAK;}

					  }
				  break;

	      case calculate_ratio:
	            time_between_two_pic = (positif_pic_time > negatif_pic_time) ? (positif_pic_time - negatif_pic_time) : (negatif_pic_time - positif_pic_time);


	              if (time_between_two_pic < t2_t1 && positif_peak_val != 0 && negatif_peak_val != 0){



		          float abs_first = fabsf((float)positif_peak_val);
		          float abs_second = fabsf((float)negatif_peak_val);

		          if (abs_first >= abs_second){
		        	  float ratio1 = abs_first / abs_second;
		        	  if (ratio1 < max_ratio_limit ) {
						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
						  HAL_Delay(30);
						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		        	  }
		          }
		          else if (abs_second >  abs_first){
		        	  float ratio2 = abs_second /  abs_first;
		        	  if (ratio2 < max_ratio_limit ) {
						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
						  HAL_Delay(30);
						  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
		        	  }
		          }

	  }
	             state = IDLE;
				positif_peak_val=sueil;
				negatif_peak_val=-sueil;
				get_negatif_pic=0;
				get_positif_pic=0;
	            break;
	  }

	            //pir_data_ready = 0;
	            //PIR_Write_Configuration(2, 1, 3, 1);
	            //HAL_Delay(10);
	            //Sensor_Set_SDA_To_Interrupt();


	        //}




			   while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {
					}
		        HAL_SuspendTick();
		        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	            HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

		        HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 131, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
		        HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
		        SystemClock_Config();
		        HAL_ResumeTick();












	  /*
	  switch (detection_state) {
	      case IDLE:
	          if (sensor_value <= -sueil || sensor_value >= sueil) {
	              start_time = HAL_GetTick();
	              last_pic = (sensor_value >= sueil) ? sueil : -sueil;

	              max_val = sensor_value;
	              min_val = sensor_value;

	              detection_state = WAITING_FOR_PEAK;
	          }
	          break;

	      case WAITING_FOR_PEAK:
	          if (sensor_value > max_val) max_val = sensor_value;
	          if (sensor_value < min_val) min_val = sensor_value;

	          if ((sensor_value > sueil && last_pic == -sueil) || (sensor_value < -sueil && last_pic == sueil)) {
	              uint32_t elapsed_time = HAL_GetTick() - start_time;

	              if (elapsed_time < t2_t1) {
	                  if (min_val != 0 && max_val != 0) {
	                      float ratio1 = fabsf((float)max_val / (float)min_val);
	                      float ratio2 = fabsf((float)min_val / (float)max_val);

	                      if (ratio1 < 3.0f && ratio2 < 3.0f) {
	                          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	                          HAL_Delay(30);
	                          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	                      }
	                  }
	              }
	              last_pic = 0;
	              detection_state = IDLE;
	          }

	          if ((HAL_GetTick() - start_time) >= t2_t1) {
	              detection_state = IDLE;
	              last_pic = 0;
	          }
	          break;

	  }*/



/*
	 if (last_pic==0){
	  HAL_SuspendTick();
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 123, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	  HAL_PWR_EnterSTANDBYMode();
	}
*/

/*
	      uint32_t current_time = HAL_GetTick();

	      if (led_blink_count > 0) {
	          if (current_time - last_blink_time >= 100) {
	              led_state = !led_state;
	              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
	              last_blink_time = current_time;

	              if (!led_state) {
	                  led_blink_count--;
	              }
	          }
	      }

	      static uint32_t last_sample_time = 0;
	      if (current_time - last_sample_time >= 60) {
	          last_sample_time = current_time;

	          sensor_value = PIR_Read_PeakHold();
	          sum += abs(sensor_value);
	          n++;

	          if (n >= MAX_SAMPLES) {
	              moyenne_glissante = sum / MAX_SAMPLES;
	              current_buf = !current_buf;

	              if (moyenne_glissante > sueil) {
	                 led_blink_count = 5;
	                  int len = snprintf(buf[current_buf], sizeof(buf[0]), "Motion Detected! Value: %d\r\n", moyenne_glissante);
	                  HAL_UART_Transmit(&huart1, (uint8_t*)buf[current_buf], len, 1000);
	              }
	              else {
	                  if (led_blink_count == 0) {
	                      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	                  }
	                  int len = snprintf(buf[current_buf], sizeof(buf[0]), "No Detection, Value: %d\r\n", moyenne_glissante);
	                  HAL_UART_Transmit(&huart1, (uint8_t*)buf[current_buf], len, 1000);
	              }
	              sum = 0;
	              moyenne_glissante = 0;
	              n = 0;
	          }
	      }

*/









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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_10BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 121, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
    // Rien à écrire ici, l'appel de cette fonction valide l'interruption de la HAL
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
