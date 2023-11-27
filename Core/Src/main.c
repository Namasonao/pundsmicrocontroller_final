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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsl_reg.h"
#include "helper_functions.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ACC_THRESHOLD 20
#define VEL_THRESHOLD 90
#define AVG_COUNT 10
#define STOP_THRESHOLD 12
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAG_SQ2(a,b) ((a)*(a) + (b)*(b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
stmdev_ctx_t dev_ctx;
int16_t data_raw_acceleration[3];
float acceleration_mg[3];
float acceleration_average_mg[3];
float acceleration_deviation_mg[3];
float velocity[2];
float total_move[2];
int16_t is_moving = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

int32_t read_acceleration();

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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize lsm6dsl driver */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &hi2c2;
  HAL_Delay(150); //BOOT
  uint8_t whoAmI = 0;
  printf("RESET\r\n");
  while  (lsm6dsl_device_id_get(&dev_ctx, &whoAmI));



  uint8_t test = lsm6dsl_device_id_get(&dev_ctx, &whoAmI);
  if ( whoAmI != LSM6DSL_ID ) {
	  printf("Did not recognise gyro, %i != %i; %i\r\n", whoAmI, LSM6DSL_ID, test);
	  while(1);
  } else {
	  printf("OK\r\n");
  }


  /* Restore default configuration */
  lsm6dsl_reset_set(&dev_ctx, PROPERTY_ENABLE);

  uint8_t rst;
  do {
    lsm6dsl_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm6dsl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
/* Set Output Data Rate */
  lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_12Hz5);
  lsm6dsl_gy_data_rate_set(&dev_ctx, LSM6DSL_GY_ODR_OFF);
/* Set full scale */
  lsm6dsl_xl_full_scale_set(&dev_ctx, LSM6DSL_2g);


  acceleration_average_mg[0] = 0;
  acceleration_average_mg[1] = -16;
  acceleration_average_mg[2] = 1025; // after expertimenting a bit, just starting value
  printf("Calculating resting acceleration, do not move...\r\n");
  for (int i = 0; i < AVG_COUNT; ++i) {
	  while (!read_acceleration());
	  acceleration_average_mg[i] = ((AVG_COUNT - 1) * acceleration_average_mg[i] + acceleration_mg[i]) / AVG_COUNT;
  }
  printf("Resting acceleration: %i\t%i\t%i\r\n", (int)acceleration_average_mg[0], (int)acceleration_average_mg[1], (int)acceleration_average_mg[2]);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // MOVE MOUSE: (smoothly)

	  if (!read_acceleration()) {
		  continue;
	  }
	  for (int i = 0; i < 3; ++i) {
		// CALCULATE ACCELERATION DEVIATION
		acceleration_deviation_mg[i] = acceleration_mg[i] - acceleration_average_mg[i];
	  }
	  /*printf("Acceleration [mg]:%i\t%i\t%i\t(%i\t%i\t%i)\r\n",
			  (int)acceleration_deviation_mg[0], (int)acceleration_deviation_mg[1], (int)acceleration_deviation_mg[2],
			  (int)acceleration_mg[0], (int)acceleration_mg[1], (int)acceleration_mg[2]);*/
	  if (is_moving) {
		  // MOVING LOOP
		  printf("Velocity [mg * frames]: %i\t%i\r\n", (int)velocity[0], (int)velocity[1]);
		  velocity[0] += acceleration_deviation_mg[0];
		  velocity[1] += acceleration_deviation_mg[1];
		  ++is_moving;
		  total_move[0] += velocity[0];
		  total_move[1] += velocity[1];
		  if (MAG_SQ2(acceleration_deviation_mg[0], acceleration_deviation_mg[1]) < ACC_THRESHOLD
				  && MAG_SQ2(velocity[0], velocity[1]) < VEL_THRESHOLD * VEL_THRESHOLD) {
			  is_moving = 0;
			  printf("STOP MOVING\r\n");
			  printf("TOTAL MOVE: %i\t%i\r\n", (int)total_move[0], (int)total_move[1]);
		  }
		  if (is_moving > STOP_THRESHOLD) {
			  printf("BADBADBAD---");
			  is_moving = 0;
			  printf("STOP MOVING\r\n");
			  printf("TOTAL MOVE: %i\t%i\r\n", (int)total_move[0], (int)total_move[1]);
		  }
	  } else {
		  if (MAG_SQ2(acceleration_deviation_mg[0],acceleration_deviation_mg[1]) > ACC_THRESHOLD * ACC_THRESHOLD) {
			  // START MOVING LOOP
			  printf("START MOVING\r\n");
			  total_move[0] = 0;
			  total_move[1] = 0;
			  is_moving = 1;
			  velocity[0] = acceleration_deviation_mg[0];
			  velocity[1] = acceleration_deviation_mg[1];
		  }
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
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
  hi2c2.Init.Timing = 0x00702991;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int32_t read_acceleration() {
	// READ ACCELERATION
	lsm6dsl_reg_t reg;
	lsm6dsl_status_reg_get(&dev_ctx, &reg.status_reg);
	// ACCELERATION NOT READY
	if (!reg.status_reg.xlda) {
		return 0;
	}
	/* Read magnetic field data */
	memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
	lsm6dsl_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
	acceleration_mg[0] = lsm6dsl_from_fs2g_to_mg(
				   data_raw_acceleration[0]);
	acceleration_mg[1] = lsm6dsl_from_fs2g_to_mg(
				   data_raw_acceleration[1]);
	acceleration_mg[2] = lsm6dsl_from_fs2g_to_mg(
				   data_raw_acceleration[2]);
	return 1;
}
/*
 * Write a register from the LSM6DSL sensor
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 */
int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
    HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len,
                     1000);
    return 0;
}

/*
 * Read a register from the LSM6DSL sensor
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 */
int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len){
    int32_t ret = HAL_I2C_Mem_Read(&hi2c2, LSM6DSL_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, bufp, len,
                     1000);
    if (ret != 0) {
    	printf("ERROR from platform_read: %i %li\r\n", ret, HAL_I2C_GetError(&hi2c2));

    }
    return ret;
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
