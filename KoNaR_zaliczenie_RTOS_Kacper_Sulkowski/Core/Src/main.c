/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include "string.h"
#include "semphr.h"
#include "hts221_register_map.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define tim2_ARR 153
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//flags
static volatile bool button_pushed= false;
static volatile bool hts221_read_ready= false;

//counters
static volatile uint8_t button_push_counter =0;

//buffers
static uint8_t buffer_button_UART[64]={0};
static uint8_t buffer_hts221_UART[64]={0};
static uint8_t buffer_tmp[2]={0};
static uint8_t buffer_hmdt[2]={0};



//Interpolation consts
static	uint16_t T0_OUT=0;
static	uint16_t T1_OUT=0;
static	float T0_degC=0;
static	float T1_degC=0;

static	uint16_t H0_OUT=0;
static	uint16_t H1_OUT=0;
static	float H0_rH=0;
static	float H1_rH=0;


//Semaphores
static xSemaphoreHandle tx_UART_mutex = NULL;
static xSemaphoreHandle hts221_rw_data_accsess_mutex = NULL;



//Task Handles
static TaskHandle_t GPIO_EXTI13_IR_task_handle=NULL;
static TaskHandle_t UART2_TX_CPLT_IR_task_handle=NULL;
static TaskHandle_t I2C_MemRx_Cplt_IR_task_handle=NULL;


//static uint8_t test=0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//CALLBACKS AND CALLBACK TASKTS


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==GPIO_PIN_13)
	{
		BaseType_t check_if_yield_required;
		check_if_yield_required=xTaskResumeFromISR(GPIO_EXTI13_IR_task_handle);
		portYIELD_FROM_ISR(check_if_yield_required);
	}
}

void GPIO_EXTI13_IR_task(void* param)
{
	while(1)
	{
		vTaskSuspend(NULL);
		button_pushed=true;
		button_push_counter++;
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart2)
	{
		BaseType_t check_if_yield_required;
		check_if_yield_required=xTaskResumeFromISR(UART2_TX_CPLT_IR_task_handle);
		portYIELD_FROM_ISR(check_if_yield_required);
	}
}


void UART2_TX_CPLT_IR_task(void* param)
{
	while(1)
	{
		vTaskSuspend(NULL);
		xSemaphoreGive(tx_UART_mutex);
	}
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c==&hi2c1)
	{
		BaseType_t check_if_yield_required;
		check_if_yield_required=xTaskResumeFromISR(I2C_MemRx_Cplt_IR_task_handle);
		portYIELD_FROM_ISR(check_if_yield_required);
	}
}

void I2C_MemRx_Cplt_IR_task(void*param)
{
	while(1)
	{
		vTaskSuspend(NULL);
		xSemaphoreGive(hts221_rw_data_accsess_mutex);
	}
}





// Timers handling functions

void init_timers()
{
	HAL_TIM_Base_Start(&htim2); //inicjalizacja timera
	HAL_TIM_Base_Start_IT(&htim4);//inicjalizacja w trybie IR
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //inicjalizacja generatora PWM
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,tim2_ARR/2); //wypełnienie 50%
}





//Button handling functions

void blue_button_pushed(void* param)
{
	while(1)
	{
		if(button_pushed)
		{
			button_pushed=false;
			if(xSemaphoreTake(tx_UART_mutex,100))
			{
				snprintf((char*)buffer_button_UART, sizeof(buffer_button_UART), "ALLERT: BUTTON PUSHED!!! Counter: %u\n\r",button_push_counter);
				HAL_UART_Transmit_IT(&huart2, buffer_button_UART, strlen((char*)buffer_button_UART));
			}


		}
	}
}




//HTS221 handling functions


void hts221_read(uint8_t address,uint8_t* dest, uint8_t size)
{
	if(xSemaphoreTake(hts221_rw_data_accsess_mutex,100))
	{
		HAL_I2C_Mem_Read(&hi2c1, HTS221_I2C_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, dest, size, HAL_MAX_DELAY);
		xSemaphoreGive(hts221_rw_data_accsess_mutex);
	}
}

void hts221_read_IT(uint8_t address,uint8_t* dest, uint8_t size)
{
	if(xSemaphoreTake(hts221_rw_data_accsess_mutex,100))
	{

			HAL_I2C_Mem_Read_IT(&hi2c1, HTS221_I2C_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, dest, size);
	}
}

void hts221_write(uint8_t address, uint8_t val)
{
	if(xSemaphoreTake(hts221_rw_data_accsess_mutex,100))
	{
		HAL_I2C_Mem_Write(&hi2c1, HTS221_I2C_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, &val, sizeof(val), HAL_MAX_DELAY);
		xSemaphoreGive(hts221_rw_data_accsess_mutex);
	}
}

void hts221_check_ID()
{
	uint8_t WHO_I_AM=0;

	const char* str = "HTS221 Connected!\n\r";

	hts221_read(HTS221_WHO_AM_I, &WHO_I_AM, 1);

	if(WHO_I_AM==HTS221_I2C_ID)
	{
		if(xSemaphoreTake(tx_UART_mutex,100))
		{
			HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str),HAL_MAX_DELAY);
			xSemaphoreGive(tx_UART_mutex);
		}
	}
}

void hts221_tmp_interpol_init()
{
	uint8_t T0_degCx8=0;
	uint8_t T1_degCx8=0;
	uint8_t T1_T0_MSBx8=0;

	hts221_read(HTS221_T0_OUT_H, &buffer_tmp[1], 1);
	hts221_read(HTS221_T0_OUT_L, &buffer_tmp[0], 1);
	T0_OUT=((uint16_t)buffer_tmp[1]<<8) | (uint16_t)buffer_tmp[0];

	hts221_read(HTS221_T1_OUT_H, &buffer_tmp[1], 1);
	hts221_read(HTS221_T1_OUT_L, &buffer_tmp[0], 1);
	T1_OUT=((uint16_t)buffer_tmp[1]<<8) | (uint16_t)buffer_tmp[0];

	hts221_read(HTS221_T0_degC_x8, &T0_degCx8, 1);
	hts221_read(HTS221_T1_degC_x8, &T1_degCx8, 1);
	hts221_read(HTS221_T1_T0_MSB_x8, &T1_T0_MSBx8, 1);

	T0_degC=(T0_degCx8 + ((T1_T0_MSBx8 & 0x03)<<8))/8.f;
	T1_degC=(T1_degCx8 + ((T1_T0_MSBx8 & 0x0C)<<6))/8.f;

}

void hts221_hmd_interpol_init()
{
	uint8_t H0_rHx2=0;
	uint8_t H1_rHx2=0;

	hts221_read(HTS221_H0_OUT_H, &buffer_hmdt[1], 1);
	hts221_read(HTS221_H0_OUT_L, &buffer_hmdt[0], 1);
	H0_OUT=((uint16_t)buffer_hmdt[1]<<8) | buffer_hmdt[0];

	hts221_read(HTS221_H1_OUT_H, &buffer_hmdt[1], 1);
	hts221_read(HTS221_H1_OUT_L, &buffer_hmdt[0], 1);
	H1_OUT=((uint16_t)buffer_hmdt[1]<<8) | buffer_hmdt[0];



	hts221_read(HTS221_H0_rH_x2, &H0_rHx2, 1);
	hts221_read(HTS221_H1_rH_x2, &H1_rHx2, 1);

	H0_rH=H0_rHx2/2.f;
	H1_rH=H1_rHx2/2.f;
}

void hts221_init()
{
	hts221_check_ID();

	hts221_write(HTS221_CTRL_REG1, HTS221_PD_ON | HTS221_BDU_ON |HTS221_ODR_12HZ);

	hts221_tmp_interpol_init();

	hts221_hmd_interpol_init();
}

void hts221_data_read()
{
	hts221_read_IT(HTS221_TEMP_OUT_H, &buffer_tmp[1], 1);
	hts221_read_IT(HTS221_TEMP_OUT_L, &buffer_tmp[0], 1);
	hts221_read_IT(HTS221_HUMIDITY_OUT_H, &buffer_hmdt[1], 1);
	hts221_read_IT(HTS221_HUMIDITY_OUT_L, &buffer_hmdt[0], 1);
}

void hts221_UART_data_tx()
{
	if(xSemaphoreTake(hts221_rw_data_accsess_mutex,100))
	{
		float tmp  = ((uint16_t)buffer_tmp[1]<<8) | (uint16_t)buffer_tmp[0];

		/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		 *
		 *				!!!!!!!!!!! Tutaj oszukałem (dopisałem "65536 - ") !!!!!!!!!
		 *
		 * Albo mój czujnik jest zrąbany, albo czegoś nie rozumiem. W  suchym pomieszczeniu pokazuje 70%,
		 * a im większa wilgotość faktycznie, tym mniejszy wynik. Można to zaobserwować nawet na suchych
		 * nieprzekonwertowanych danych. W trybie debugowania poprzez Live Expresions na wektorze buffer_hmdt,
		 * w warunkach pokojowych dane prawie przepełniają 16 bitów, a kiedy się na czujnik chuchnie,
		 * to odczyt spada. Współczynnik kierunkowy interpolacji (wyznaczony zgodnie z instrukcją)
		 * jest ewidentnie dodatni, więc nie wiem o co chodzi.
		 *
		 *!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

		float hmdt =65536 - (((uint16_t)buffer_hmdt[1]<<8) | buffer_hmdt[0]);
		xSemaphoreGive(hts221_rw_data_accsess_mutex);


		tmp=((T1_degC-T0_degC)*(tmp-T0_OUT))/(T1_OUT-T0_OUT) + T0_degC;


		hmdt=(((H1_rH-H0_rH))*(hmdt-H0_OUT))/(H1_OUT-H0_OUT) + H0_rH;

		snprintf((char*)buffer_hts221_UART,sizeof(buffer_hts221_UART),"Temp: %3.4f[C]; Humid: %3.4f[%%]\n\r",tmp,hmdt);

		HAL_UART_Transmit_IT(&huart2, buffer_hts221_UART, strlen((char*)buffer_hts221_UART));
	}


}

void hts221_data_Task(void* param)
{
	while(1)
	{
		if(hts221_read_ready)
		{
			hts221_read_ready=false;

			hts221_data_read();

			hts221_UART_data_tx();
		}
	}
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  tx_UART_mutex=xSemaphoreCreateBinary();
  xSemaphoreGive(tx_UART_mutex);

  hts221_rw_data_accsess_mutex = xSemaphoreCreateBinary();
  xSemaphoreGive(hts221_rw_data_accsess_mutex);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  init_timers();
  hts221_init();
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  xTaskCreate(blue_button_pushed, "blue_button_pushed", 128, NULL, 5, NULL);
  xTaskCreate(hts221_data_Task, "hts221_data_Task", 256, NULL, 5, NULL);

  xTaskCreate(GPIO_EXTI13_IR_task, "GPIO_EXTI13_IR_task", 128, NULL, 5, &GPIO_EXTI13_IR_task_handle);
  xTaskCreate(UART2_TX_CPLT_IR_task, "UART2_TX_CPLT_IR_task", 128, NULL, 5, &UART2_TX_CPLT_IR_task_handle);
  xTaskCreate(I2C_MemRx_Cplt_IR_task, "I2C_MemRx_Cplt_IR_task", 128, NULL, 5, &I2C_MemRx_Cplt_IR_task_handle);

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 153;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 39999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 181;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(htim==&htim4)
	{
		hts221_read_ready=true;
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
