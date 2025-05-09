/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>  // del sprintf
#include <string.h>  // del strlen()

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint32_t IC_Val1 = 0, IC_Val1_2 = 0;
volatile uint32_t IC_Val2 = 0, IC_Val2_2 = 0;
volatile uint32_t Difference = 0, Difference_2 = 0;
volatile uint16_t Is_First_Captured = 0, Is_First_Captured_2 = 0;  // Skirti suzinoti ar frontas nuskaitytas
volatile uint16_t Distance  = 0, Distance_2  = 0;
int16_t skirtumas_pc=0 ;


#define MAX_DISTANCE_CM 30 //Maksimalus atstumas
#define SENSOR_BUFFER_SIZE 7 //Buferio dydis
uint16_t sensor1_buffer[SENSOR_BUFFER_SIZE];
uint16_t sensor2_buffer[SENSOR_BUFFER_SIZE];
uint8_t buffer_index = 0;

// UART valdymui
//volatile uint8_t uart_ready = 1;  // 1 = laisvas, 0 = užimtas
//#define UART_MSG_LEN 32  // Maksimalus žinutes ilgis

void delay(uint16_t time){
	__HAL_TIM_SET_COUNTER(&htim2,0);
	while(__HAL_TIM_GET_COUNTER (&htim2)<time);
}//Laikmatis 10us pertraukimas (BLOKUOJANTIS)

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        // Pirmas jutiklis

        if (Is_First_Captured==0) {
            IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);//Gaunama laiko verte
            Is_First_Captured = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);//Galinio fronto nuskaitymui
        }
        else {
            IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);//Gaunama laiko verte
            __HAL_TIM_SET_COUNTER(htim, 0);//Is naujo nustatomas laikmatis

            if (IC_Val2 > IC_Val1)
                Difference = IC_Val2 - IC_Val1;
            else
                Difference = (0xffff - IC_Val1) + IC_Val2;

            Distance = Difference * 0.034 / 2;
            Is_First_Captured = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);//Priekinio fronto nuskaitymui
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);//Isjungiamas timeris
        }
    }
    else if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        // Antras jutiklis 

        if (Is_First_Captured_2==0) {
            IC_Val1_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);//Gaunama laiko verte
            Is_First_Captured_2 = 1;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);//Galinio fronto nuskaitymui
        }
        else {
            IC_Val2_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);//Gaunama laiko verte
            __HAL_TIM_SET_COUNTER(htim, 0);//Is naujo nustatomas laikmatis

            if (IC_Val2_2 > IC_Val1_2)
                Difference_2 = IC_Val2_2 - IC_Val1_2;
            else
                Difference_2 = (0xffff - IC_Val1_2) + IC_Val2_2;

            Distance_2 = Difference_2 * 0.034 / 2;
            Is_First_Captured_2 = 0;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);//Priekinio fronto nuskaitymui
            __HAL_TIM_DISABLE_IT(htim, TIM_IT_CC1);//Isjungiamas timeris
        }
    }
}

void TriggerSensor1(void) {
    // TRIG1 - PA1
    HAL_GPIO_WritePin(GPIOA, TRIG_1_Pin, GPIO_PIN_SET);
    delay(10); // 10 µs
    HAL_GPIO_WritePin(GPIOA, TRIG_1_Pin, GPIO_PIN_RESET);

    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1); // ECHO1 – TIM2 CH1
}

void TriggerSensor2(void) {
    // TRIG2 - PA5
    HAL_GPIO_WritePin(GPIOA, TRIG_2_Pin, GPIO_PIN_SET);
    delay(10); // 10 µs
    HAL_GPIO_WritePin(GPIOA, TRIG_2_Pin, GPIO_PIN_RESET);

    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1); // ECHO2 – TIM3 CH1
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//    if (huart->Instance == USART2) {
//        uart_ready = 1;  // UART vel laisvas
//    }
//}

//void Send_UART_NonBlocking(uint16_t Distance, uint16_t Distance_2, int16_t skirtumas_pc) {
//    if (uart_ready) {
//        char msg[UART_MSG_LEN];
//        int len = snprintf(msg, sizeof(msg), "D1=%u D2=%u D=%d\r\n", Distance, Distance_2, skirtumas_pc);
//        
//        uart_ready = 0;  // UART užimtas
//        HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, len);
//    }
//    // Jei UART užimtas, praleidžiame ši siuntima
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        static uint8_t cycle_counter = 0;  // Skaiciuoja 100ms intervalus

        cycle_counter++;
        
        // Duomenu parodymo i ekrana ciklo vertes atnaujinimas kas 2s (20 x 100ms)
        if (cycle_counter >= 20) {
            cycle_counter = 0;
        }

        // Jutikliu trigerinimas
        switch(cycle_counter % 2) {
            case 0:  // Lygus ciklai (0, 2, 4...)
                TriggerSensor1();  // Pirmas jutiklis
                break;
                
            case 1:  // Nelygus ciklai (1, 3, 5...)
                TriggerSensor2();  // Antras jutiklis
                break;
        }

        // Sudedam i buferi kas 200ms (kas 2 ciklus)
        if ((cycle_counter % 2) == 1) {  // 100ms, 300ms, 500ms...
            if (buffer_index < SENSOR_BUFFER_SIZE) {
                if (Distance <= MAX_DISTANCE_CM && Distance_2 <= MAX_DISTANCE_CM) {
                    sensor1_buffer[buffer_index] = Distance;
                    sensor2_buffer[buffer_index] = Distance_2;
										skirtumas_pc=Distance-Distance_2;
                    buffer_index++;
										//Send_UART_NonBlocking(Distance, Distance_2, skirtumas_pc);
										// Siunciame duomenis i kompiuteri kas 200ms
										char debug_msg[30];
										int len = sprintf(debug_msg, "S1=%3u | S2=%3u | Skrt=%2d\r\n", 
                              Distance, Distance_2, skirtumas_pc);
										HAL_UART_Transmit(&huart2, (uint8_t*)debug_msg, len, 1);

                }
            }

        }

        // Duomenu rodymas kas 2s (20 TIM6 ciklu)
        if (cycle_counter == 0) {
            // Skaiciavimai ir OLED atnaujinimas
            uint32_t sum1 = 0, sum2 = 0;
            uint8_t valid_samples = 0; //Ar vertes geros <30 cm
            
            for (int i = 0; i < SENSOR_BUFFER_SIZE; i++) {
                if (sensor1_buffer[i] <= MAX_DISTANCE_CM && sensor2_buffer[i] <= MAX_DISTANCE_CM) {
                    sum1 += sensor1_buffer[i];
                    sum2 += sensor2_buffer[i];
                    valid_samples++;
                }// Tikrinamos abieju buferiu vertes. Ar nera blogos >30. Jei geros susumuojamos, del vidurkio skaiciavimo
            }
            
            if (valid_samples > 0) {
							// Duomenu parodymas OLED ekrane
                uint16_t avg1 = sum1 / valid_samples;
                uint16_t avg2 = sum2 / valid_samples;
                int16_t diff = avg1 - avg2;

                char buf1[20], buf2[20], buf3[20];
                sprintf(buf1, "S1: %d cm", avg1);
                sprintf(buf2, "S2: %d cm", avg2);
                sprintf(buf3, "Skirtumas: %d cm", diff);

                ssd1306_Fill(Black);
                ssd1306_SetCursor(0, 0);
                ssd1306_WriteString(buf1, Font_6x8, White);
                ssd1306_SetCursor(0, 12);
                ssd1306_WriteString(buf2, Font_6x8, White);
                ssd1306_SetCursor(0, 24);
                ssd1306_WriteString(buf3, Font_6x8, White);
                ssd1306_UpdateScreen();
            }
            
            buffer_index = 0;  // Istrinti buferiu duomenis po parodymo
        }
    }
}



void OLED_Init(){
	ssd1306_Init();
	ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
	
};
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
	
  /* USER CODE BEGIN 2 */
	OLED_Init();
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	//HAL_UART_Init(&huart2); 
	//HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_TIM_Base_Start_IT(&htim6); 
	
//	ssd1306_SetCursor(0, 0); // Nustatome kursoriu i pirma eilute, pirma stulpeli
//  ssd1306_WriteString("Hello, World!", Font_7x10, White); 
//  ssd1306_UpdateScreen(); // Atnaujiname ekrana
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 31;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim3, TIM3_TI1_GPIO) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 31999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 32-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_1_Pin|TRIG_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRIG_1_Pin TRIG_2_Pin */
  GPIO_InitStruct.Pin = TRIG_1_Pin|TRIG_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
