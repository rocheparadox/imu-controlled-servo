/* USER CODE BEGIN Header */

/*
 * Author : Roche Christopher
 * email  : rochextopher@gmail.com
 *

 *
	MIT License

	Copyright (c) 2024 Roche Christopher

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.

  */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu_6050.h"
#include "servo_driver.h"
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

I2C_HandleTypeDef *mpu6050_i2c_bus;
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

int is_data_ready = 0;
uint8_t int_status;
uint16_t dev_address = 0x68 << 1;
int16_t offset;
char buffer[12];  // Buffer to hold the converted string, enough for 32-bit int
float scaling = 65.5; // 131 65.5 32.8 16.4
float pitch;
float dt;
uint16_t fifo_count;

Servo servo;

typedef struct GyroOrientation{
	float roll; // x
	float pitch; // y
	float yaw; // z
}GyroOrientation;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void data_ready_handler(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void orient_servo(int pitch){
	if(pitch < -90){
		pitch = -90;
	}
	else if(pitch > 90){
		pitch = 90;
	}
	int deg = pitch + 90;
	rotate_servo(&servo, deg);
}

void data_ready_handler(){
	is_data_ready = 0;
	fifo_count = get_fifo_count();
	while(fifo_count < 2){
		fifo_count = get_fifo_count();
		HAL_Delay(3); // Wait for sometime before the next fifo count read. If done consecutively without delay, MPU6050 freezes.
	}

	/* calculate the angle */
	while(fifo_count != 0){
		int16_t gyrox_raw = get_unified_data_from_fifo();
		// cancel out the offset
		gyrox_raw -= offset;
		// scale the data into degrees
		gyrox_raw /= scaling;
		//delta_angle = (float)((int)(delta_angle * 10000))/10000;

		float delta_angle = gyrox_raw * dt;
		pitch += delta_angle;

		fifo_count = get_fifo_count();
		HAL_Delay(1);

		//sprintf(buffer, "\n%d %f", gyrox_raw, pitch);  // Convert float to string
		//HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
	}

	// read the interrupt status register
	orient_servo((int)pitch);
	get_interrupt_status(&int_status);


//	// read the raw gyroscope values
//    int16_t gyrox_raw; // int16_t datatype is a signed 16 bit integer which stores 2's complement values. The gyrosocope data provided by the MPU6050 is a signed 16 bit integer too.
//    //get_gyro_x(&hi2c, &gyrox_raw);
//
//	//int16_t datatype is a signed 16 bit integer which stores 2's complement values. The gyrosocope data provided by the MPU6050 is a signed 16 bit integer too.
//	int16_t gyrox_raw = get_unified_data_from_fifo(&hi2c);
//
//    //sprintf(buffer, "\n%d %d", gyrox_raw, gyrox_buffer);  // Convert float to string
//    //HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);


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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  mpu6050_i2c_bus = &hi2c1;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  /* Setup the Servo Motor and Initialize it */

  servo.servo_pwm_timer = &htim2;
  servo.current_position = 0;

  // check servo motion
  rotate_servo(&servo, 45);
  HAL_Delay(1000);


  /* Setup the MPU6050 and Initialize it*/

  uint8_t power_management = 0b00000001;
  uint8_t mpu6050_config =   0b00000110; // for DLPF
  //int sampling_rate = 100;
  uint8_t smplrt_div_value = 9;
  dt = ((float)(smplrt_div_value + 1))/(float)gyroscope_output_rate;
  uint8_t interrupt_enable = 0b00000001;
  uint8_t interrupt_pin_config = 0b10100000;
  uint8_t gyro_config = 0b00001000; // for FS_SEL
  uint8_t fifo_enable_config = 0b01000000;
  uint8_t user_control_config;

  // wake the device up, set the power management bits
  write_to_power_mgmt(&power_management);
  // set configuration bits for DLPF
  set_configuration(&mpu6050_config);
  // set gyroscope configuration
  set_gyroscope_configuration(&gyro_config);
  // set the sampling rate
  //set_sampling_rate(&hi2c, sampling_rate);
  set_smplrt_div(&smplrt_div_value);
  // set interrupt configuration bits
  set_interrupt_pin_configuration(&interrupt_pin_config);
  // set interrupt enable bits
  set_interrupt_enable(&interrupt_enable);
  get_interrupt_status(&int_status);

  /* calibrate gyroscope */

  int16_t temp_gyro_x = 0;
  // read out unwanted data.
  for(int i=0; i<10; i++){
	  while(is_data_ready != 1);
	  get_gyro_x(&temp_gyro_x);
	  get_interrupt_status(&int_status);
	  //sprintf(buffer, "\n%d", (int)temp_gyro_x);  // Convert integer to string
	  //HAL_UART_Transmit(&huart3, buffer ,strlen(buffer), HAL_MAX_DELAY);
  }

  int samples = 30;
  offset=0;
  for(int i=0; i<samples; i++){
	  while(is_data_ready != 1);
	  get_gyro_x(&temp_gyro_x);
	  offset += temp_gyro_x;
	  get_interrupt_status(&int_status);
      //sprintf(buffer, "\n%d", (int)temp_gyro_x);  // Convert integer to string
      //HAL_UART_Transmit(&huart3, buffer ,strlen(buffer), HAL_MAX_DELAY);
  }
  offset = offset/samples;

//  sprintf(buffer, "\noff: %d", (int)offset);  // Convert integer to string
//  HAL_UART_Transmit(&huart3, buffer ,strlen(buffer), 1);

  // Enable the FIFO for gyro x and gyro y
  set_fifo_enable(&fifo_enable_config);

  read_from_register(MPU6050_USER_CTRL, &user_control_config);
  // Reset the FIFO in user control
  user_control_config |= (uint8_t)(1 << 2);
  set_user_control(&user_control_config);
  // Enable FIFO in User control
  user_control_config = (uint8_t)(1 << 6);
  set_user_control(&user_control_config);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  get_interrupt_status(&int_status);
  pitch = 0.0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(is_data_ready == 1){
		  data_ready_handler();
//		  sprintf(buffer, "\n%f", pitch);
//		  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
//		  sprintf(buffer, "\n%f", pitch);  // Convert float to string
//		  HAL_UART_Transmit(&huart3, buffer, strlen(buffer), 1);
	  }
	  if(hi2c1.State != HAL_I2C_STATE_READY){
		  Error_Handler();
	  }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00707CBB;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1279;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  //uint8_t msg[] = "\nUSART Initialization Done";
  //HAL_UART_Transmit(&huart3, msg, sizeof(msg), 1);

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */

 /* Setup the interrupt pin that is connected to the DATA_RDY pin in the MPU6050 */

 /* Setup the NVIC table */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0); // PA0 is connected to EXTI0 IRQ
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Define the ISR for PA0*/
void EXTI0_IRQHandler(){
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0){
		is_data_ready = 1;
	}
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
