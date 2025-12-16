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
#include "adc.h"
#include "i2c.h"
#include "icache.h"
#include "memorymap.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_ICACHE_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // if (!MPU_begin(&hi2c1, AD0_LOW, AFSR_4G, GFSR_500DPS, 0.98, 0.004))
  // {
  //   while (1)
  //   {
  //     HAL_Delay(100);
  //   }
  // }

  // while (true)
  // {
  //   MPU_calcAttitude(&hi2c1);
  // }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
  // read timer value
  HAL_TIM_Base_Start(&htim1); // Start the timer
  StepperMotorConfig motorConfig = {
      .acceleration = 1000,
      .stepsPerRevolution = 200,
      .microstepping = 8,
      .timerHandle = &htim1,
      .maxStepsGap = 9000,
      .minStepsGap = 3000};

  StepperMotor motor1 = motor_create(&motorConfig, GPIOA, GPIO_PIN_10, GPIO_PIN_11, 195, true);
  StepperMotor motor2 = motor_create(&motorConfig, GPIOA, GPIO_PIN_8, GPIO_PIN_9, 195, true);
  StepperMotor motor3 = motor_create(&motorConfig, GPIOB, GPIO_PIN_14, GPIO_PIN_15, 194, true);
  Screen screen = screen_create(178, 136, &htim1, GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_3, GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1, &hadc1, ADC_CHANNEL_14, ADC_CHANNEL_0);

  // screen_calibration(&screen);
  screen_load_calibration(&screen);
  RRS3Options rrs3Options = {
      .buttomLeg = 67.5,
      .topLeg = 125.5,
      .baseR = 63,
      .platformR = 63};

  float lastX = 0;
  float lastY = 0;
  uint32_t lastTime = HAL_GetTick();

  float integralX = 0;
  float integralY = 0;
  float lastErrorX = 0;
  float lastErrorY = 0;
  
  float filteredVelX = 0;
  float filteredVelY = 0;
  
  float Kp = 0.8f; 
  float Ki = 0.05f;
  float Kd = 0.4f;   
  
  float angleA, angleB, angleC;

  motor_set_angle(&motor1, 90);
  motor_set_angle(&motor2, 90);
  motor_set_angle(&motor3, 90);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint16_t startTime = __HAL_TIM_GET_COUNTER(&htim1);
    ScreenData screenData = screen_read(&screen);
    
    float x = (screenData.x - screen.width / 2) / 400.0f;
    float y = -1 * (screenData.y - screen.height / 2) / 400.0f;
    
    uint32_t currentTime = HAL_GetTick();
    float dt = (currentTime - lastTime) / 1000.0f; // Convert to seconds
    if (dt > 0.1f) dt = 0.01f;
    lastTime = currentTime;
    
    float velocityX = (x - lastX) / dt;
    float velocityY = (y - lastY) / dt;
    
    filteredVelX = 0.3f * velocityX + 0.7f * filteredVelX;
    filteredVelY = 0.3f * velocityY + 0.7f * filteredVelY;
    
    float errorX = x;
    integralX += errorX * dt;

    if (integralX > 0.1f) integralX = 0.1f;
    if (integralX < -0.1f) integralX = -0.1f;

    float derivativeX = filteredVelX;
    float controlX = Kp * errorX + Ki * integralX + Kd * derivativeX;
    
    float errorY = y;
    integralY += errorY * dt;

    if (integralY > 0.1f) integralY = 0.1f;
    if (integralY < -0.1f) integralY = -0.1f;
    float derivativeY = filteredVelY;
    float controlY = Kp * errorY + Ki * integralY + Kd * derivativeY;
    
    if (controlX > 0.15f) controlX = 0.15f;
    if (controlX < -0.15f) controlX = -0.15f;
    if (controlY > 0.15f) controlY = 0.15f;
    if (controlY < -0.15f) controlY = -0.15f;
    
    float angleA = rrs3_calculate_angles(A, &rrs3Options, 150, controlX, controlY);
    float angleB = rrs3_calculate_angles(B, &rrs3Options, 150, controlX, controlY);
    float angleC = rrs3_calculate_angles(C, &rrs3Options, 150, controlX, controlY);
    
    lastX = x;
    lastY = y;
    
    uint16_t endTime = __HAL_TIM_GET_COUNTER(&htim1);

    motor_set_angle(&motor1, angleA);
    motor_set_angle(&motor2, angleB);
    motor_set_angle(&motor3, angleC);

    motor_run(&motor1);
    motor_run(&motor2);
    motor_run(&motor3);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 31;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 2048;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
   */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
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

#ifdef USE_FULL_ASSERT
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
