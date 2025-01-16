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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX30100_I2C_ADDR 0x57  // 7-bit I2C address of MAX30100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for temperatureTask */
osThreadId_t temperatureTaskHandle;
const osThreadAttr_t temperatureTask_attributes = {
  .name = "temperatureTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pulseTask */
osThreadId_t pulseTaskHandle;
const osThreadAttr_t pulseTask_attributes = {
  .name = "pulseTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for commTask */
osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = {
  .name = "buzzerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */


float HeartRate = 0;
float Temperature = 0;

osMutexId_t tempMutex;     // Mutex to protect shared variable (Temperature)
osMutexId_t pulseMutex;     // Mutex to protect shared variable (HeartRate)

int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void delay (uint16_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim6, 0);
	while (__HAL_TIM_GET_COUNTER(&htim6) < time)
	{
		uint32_t counter_value = __HAL_TIM_GET_COUNTER(&htim6);

	}
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
void StartTemperatureTask(void *argument);
void StartPulseTask(void *argument);
void StartCommTask(void *argument);
void StartBuzzerTask(void *argument);

/* USER CODE BEGIN PFP */
#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin low
	delay (480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
    delay (80);    // delay according to datasheet
	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay (400); // 480 us delay totally.

	return Response;
}
void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{
		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay (50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0
			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, 0);  // pull the pin LOW
			delay (50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}
uint8_t DS18B20_Read (void)
{
	uint8_t value=0;
	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_1, 0);  // pull the data pin LOW
		delay (2);  // wait for 2 us
		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_1))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay (60);  // wait for 60 us
	}
	return value;
}

void MAX30100_WriteRegister(uint8_t reg, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, MAX30100_I2C_ADDR << 1, reg, 1, &value, 1, HAL_MAX_DELAY);
}

// Read from MAX30100 Register
void MAX30100_ReadRegister(uint8_t reg, uint8_t *data, uint16_t size) {
    HAL_I2C_Mem_Read(&hi2c1, MAX30100_I2C_ADDR << 1, reg, 1, data, size, HAL_MAX_DELAY);
}

// Initialize MAX30100
void MAX30100_Init(void) {
    uint8_t mode = 0x03; // SpO2 mode
    uint8_t ledConfig = 0xFF; // Red: 50mA, IR: 50mA
    uint8_t spo2Config = 0x27; // Sampling rate: 100Hz, Pulse Width: 1600us

    // Set device mode to SpO2
    MAX30100_WriteRegister(0x06, mode);

    // Configure LED currents
    MAX30100_WriteRegister(0x09, ledConfig);

    // Configure SpO2 settings
    MAX30100_WriteRegister(0x07, spo2Config);
    // Set sampling rate to 400Hz
    MAX30100_WriteRegister(0x07, 0x27); // SpO2 configuration register for 400Hz sampling

}

// Read FIFO Data (Red and IR values)
void MAX30100_ReadFIFO(uint16_t *red, uint16_t *ir) {
    uint8_t fifoData[4];

    // Read 4 bytes from FIFO Data Register (0x05)
    MAX30100_ReadRegister(0x05, fifoData, 4);

    // Combine MSB and LSB for Red and IR values
    *red = (fifoData[0] << 8) | fifoData[1];
    *ir = (fifoData[2] << 8) | fifoData[3];
}
uint8_t MAX30100_ReadPartID(void) {
    uint8_t partID = 0;
    MAX30100_ReadRegister(0xFF, &partID, 1);
    return partID;
}

void MAX30100_DebugRegisters(void) {
    uint8_t regValue;
    MAX30100_ReadRegister(0x06, &regValue, 1);
    printf("Mode Control Register (0x06): 0x%02X\r\n", regValue);
    MAX30100_ReadRegister(0x09, &regValue, 1);
    printf("LED Configuration Register (0x09): 0x%02X\r\n", regValue);
    MAX30100_ReadRegister(0x07, &regValue, 1);
    printf("SpO2 Configuration Register (0x07): 0x%02X\r\n", regValue);
}


#define FILTER_SIZE 5
uint16_t moving_average_filter(uint16_t *buffer, uint8_t size) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return (uint16_t)(sum / size);
}

float CalculateBPM() {
    static uint32_t last_peak_time = 0;
    static uint8_t peak_count = 0;
    static uint16_t ir_buffer[FILTER_SIZE] = {0};
    static uint8_t ir_index = 0;

    uint16_t ir_value;
    MAX30100_ReadFIFO(&ir_value, NULL);

    // Update buffer for moving average
    ir_buffer[ir_index++] = ir_value;
    if (ir_index >= FILTER_SIZE) ir_index = 0;

    // Smooth signal with moving average
    uint16_t smoothed_ir = moving_average_filter(ir_buffer, FILTER_SIZE);

    // Detect peaks
    static uint16_t prev_ir = 0;
    static uint8_t is_peak = 0;

    if (smoothed_ir > prev_ir && !is_peak && smoothed_ir > 20000) {  // Threshold to ignore noise
        is_peak = 1;  // Start of a peak
        uint32_t current_time = HAL_GetTick();

        uint32_t time_diff = current_time - last_peak_time;
        last_peak_time = current_time;

        if (time_diff > 300 && time_diff < 2000) {  // Valid BPM range
            float bpm = 60000.0f / time_diff;  // Calculate BPM
            prev_ir = smoothed_ir;
            return bpm;
        }
    }

    if (smoothed_ir < prev_ir) {
        is_peak = 0;  // End of a peak
    }

    prev_ir = smoothed_ir;
    return 0.0f;  // Return 0 if no valid BPM is detected
}

#define BPM_BUFFER_SIZE 5
float bpm_buffer[BPM_BUFFER_SIZE] = {0};
uint8_t bpm_index = 0;

float SmoothBPM(float new_bpm) {
    bpm_buffer[bpm_index++] = new_bpm;
    if (bpm_index >= BPM_BUFFER_SIZE) bpm_index = 0;

    float sum = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < BPM_BUFFER_SIZE; i++) {
        if (bpm_buffer[i] > 0) {  // Ignore zero values
            sum += bpm_buffer[i];
            count++;
        }
    }
    return count > 0 ? (sum / count) : 0.0f;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of temperatureTask */
  temperatureTaskHandle = osThreadNew(StartTemperatureTask, NULL, &temperatureTask_attributes);

  /* creation of pulseTask */
  pulseTaskHandle = osThreadNew(StartPulseTask, NULL, &pulseTask_attributes);

  /* creation of commTask */
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);

  /* creation of buzzerTask */
  buzzerTaskHandle = osThreadNew(StartBuzzerTask, NULL, &buzzerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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
  hi2c1.Init.Timing = 0x00F12981;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */
	__HAL_RCC_TIM6_CLK_ENABLE();
  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 80-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
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
  HAL_TIM_Base_Start(&htim6);
  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.BaudRate = 9600;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTemperatureTask */
/**
  * @brief  Function implementing the temperatureTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTemperatureTask */
void StartTemperatureTask(void *argument)
{
  /* USER CODE BEGIN 5 */
uint8_t Temp_byte1, Temp_byte2;
		uint16_t TEMP;
		uint8_t Presence = 0;;


  /* Infinite loop */
  for(;;)
  {
	 Presence = DS18B20_Start();
	  if (Presence == 1) {
		  DS18B20_Write(0xCC);  // Skip ROM
		  DS18B20_Write(0x44);  // Convert temperature
		  HAL_Delay(1000);       // Wait for conversion to complete

		  Presence = DS18B20_Start();
		  if (Presence == 1) {
			  DS18B20_Write(0xCC);  // Skip ROM
			  DS18B20_Write(0xBE);  // Read Scratchpad

			  Temp_byte1 = DS18B20_Read();
			  Temp_byte2 = DS18B20_Read();
			  TEMP = ((Temp_byte2 << 8) | Temp_byte1);

			  // Calculate temperature
			  osMutexAcquire(tempMutex, osWaitForever);
			  Temperature = (float)TEMP / 16.0;
			  osMutexRelease(tempMutex);

			  // Print the temperature
			  printf("Temperature: %.2f°C\r\n", Temperature);
		  }
	  }


	  HAL_Delay(2000);  // Wait 3 seconds before the next reading
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartPulseTask */
/**
* @brief Function implementing the pulseTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPulseTask */
void StartPulseTask(void *argument)
{
	/* USER CODE BEGIN StartCommunicationTask */
	  float smooth_bpm = 0, raw_bpm = 0;
	  uint16_t ir_value;
	  MAX30100_Init();
	  uint8_t modeConfig = 0x03; // SpO2 mode (continuous measurement)
	  MAX30100_WriteRegister(0x06, modeConfig);
	  uint8_t partID = MAX30100_ReadPartID();
	  if (partID == 0x11) {
		  printf("MAX30100 detected!\r\n");
	  } else {
		  printf("MAX30100 not detected. Part ID: 0x%02X\n", partID);
	  }
	  MAX30100_DebugRegisters();
	  uint8_t ledConfig = 0xFF; // Red: 50mA, IR: 50mA
	  MAX30100_WriteRegister(0x09, ledConfig);
	  HAL_Delay(1);
		//char message[] = "Hello from STM32 via HC-05 Bluetooth!\r\n";
	  /* Infinite loop */
	  for(;;)
	  {
		  raw_bpm = CalculateBPM();
		  if (raw_bpm > 0) {
			  smooth_bpm = SmoothBPM(raw_bpm);
			  osMutexAcquire(pulseMutex, osWaitForever);
			  HeartRate = smooth_bpm;
			  osMutexRelease(pulseMutex);
			  printf("Heart Rate: BPM\r\n");
		  }

		  MAX30100_ReadFIFO(&ir_value, NULL);
		  if(ir_value < 10000)
		  {
			  osMutexAcquire(pulseMutex, osWaitForever);
			  HeartRate = 0;
			  osMutexRelease(pulseMutex);
		  }
		  osDelay(100);  // Wait 1 second before sending the next reading
	  }
	  /* USER CODE END StartCommunicationTask */
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the commTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */
  char messageTemp[30];  // Buffer to hold formatted temperature message
  char messagePulse[30];
  float currentTemp = 0;
  float currentBPM = 0;
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(tempMutex, osWaitForever);
	  if(Temperature < 100)
	  {
		  currentTemp = Temperature;
	  }
	  osMutexRelease(tempMutex);
	  osMutexAcquire(pulseMutex, osWaitForever);
	  currentBPM = HeartRate;
	  osMutexRelease(pulseMutex);

	  // Format the temperature into a string
	  if(currentTemp < 100)
	  {
		  snprintf(messageTemp, sizeof(messageTemp), "Temperature: %d°C\n", (int)Temperature);

		  // Transmit the formatted string over UART
		  HAL_UART_Transmit(&huart1, (uint8_t *)messageTemp, strlen(messageTemp), HAL_MAX_DELAY);
	  }


	  // Format the temperature into a string
	  if(currentBPM > 0)
	  {
		  snprintf(messagePulse, sizeof(messagePulse), "Heart Rate: %d BPM\n", (int)currentBPM);

		  // Transmit the formatted string over UART
		  HAL_UART_Transmit(&huart1, (uint8_t *)messagePulse, strlen(messagePulse), HAL_MAX_DELAY);
	  }

	  osDelay(1000);
  }
  /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartBuzzerTask */
/**
* @brief Function implementing the buzzerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBuzzerTask */
void StartBuzzerTask(void *argument)
{
  /* USER CODE BEGIN StartBuzzerTask */
	float currentTemp = 0;
	float currentBPM = 0;
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(tempMutex, osWaitForever);
	  currentTemp = Temperature;
	  osMutexRelease(tempMutex);
	  osMutexAcquire(pulseMutex, osWaitForever);
	  currentBPM = HeartRate;
	  osMutexRelease(pulseMutex);

	  if(currentBPM > 100 || currentTemp > 25)
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
	  }

	  osDelay(1000);
  }
  /* USER CODE END StartBuzzerTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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
