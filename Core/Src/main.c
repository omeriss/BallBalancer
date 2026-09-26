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
#include "gpio.h"
#include "i2c.h"
#include "icache.h"
#include "memorymap.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_ACCELERATION 1000
#define MOTOR_STEPS_PER_REVOLUTION 200
#define MOTOR_MICROSTEPPING 8
#define MOTOR_MAX_STEPS_GAP 9000
#define MOTOR_MIN_STEPS_GAP 3000
#define MOTOR_INITIAL_ANGLE_DEG 90

#define MOTOR1_START_ANGLE_DEG 195
#define MOTOR2_START_ANGLE_DEG 195
#define MOTOR3_START_ANGLE_DEG 195

#define SCREEN_WIDTH_PX 178
#define SCREEN_HEIGHT_PX 136

#define RRS3_BOTTOM_LEG_MM 67.5f
#define RRS3_TOP_LEG_MM 125.5f
#define RRS3_BASE_RADIUS_MM 63.0f
#define RRS3_PLATFORM_RADIUS_MM 63.0f
#define PLATFORM_HEIGHT_MM 150.0f

#define SCREEN_POSITION_SCALE 400.0f

#define PID_KP 0.5f
#define PID_KI 0.015f
#define PID_KD 0.25f
#define PID_INTEGRAL_LIMIT 0.015f
#define PID_OUTPUT_LIMIT 0.1f
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
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  StepperMotorConfig motorConfig = {.acceleration = MOTOR_ACCELERATION,
                                    .stepsPerRevolution = MOTOR_STEPS_PER_REVOLUTION,
                                    .microstepping = MOTOR_MICROSTEPPING,
                                    .timerHandle = &htim1,
                                    .maxStepsGap = MOTOR_MAX_STEPS_GAP,
                                    .minStepsGap = MOTOR_MIN_STEPS_GAP};

  StepperMotor motor1 = motor_create(&motorConfig, GPIOB, GPIO_PIN_15,
                                     GPIO_PIN_14, MOTOR1_START_ANGLE_DEG, true);
  StepperMotor motor2 = motor_create(&motorConfig, GPIOA, GPIO_PIN_9,
                                     GPIO_PIN_8, MOTOR2_START_ANGLE_DEG, true);
  StepperMotor motor3 = motor_create(&motorConfig, GPIOA, GPIO_PIN_11,
                                     GPIO_PIN_10, MOTOR3_START_ANGLE_DEG, true);
  Screen screen = screen_create(
      SCREEN_WIDTH_PX, SCREEN_HEIGHT_PX, &htim1, GPIOA, GPIO_PIN_2, GPIOA,
      GPIO_PIN_3, GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1, &hadc1,
      ADC_CHANNEL_14, ADC_CHANNEL_0);

  // screen_calibration(&screen);
  screen_load_calibration(&screen);
  RRS3Options rrs3Options = {.buttomLeg = RRS3_BOTTOM_LEG_MM,
                             .topLeg = RRS3_TOP_LEG_MM,
                             .baseR = RRS3_BASE_RADIUS_MM,
                             .platformR = RRS3_PLATFORM_RADIUS_MM};

  PIDConfig pidConfig = {.kp = PID_KP,
                         .ki = PID_KI,
                         .kd = PID_KD,
                         .integralLimit = PID_INTEGRAL_LIMIT,
                         .outputLimit = PID_OUTPUT_LIMIT};
  PID pidX = pid_create(&pidConfig);
  PID pidY = pid_create(&pidConfig);
  uint32_t lastTime = HAL_GetTick();

  motor_set_angle(&motor1, MOTOR_INITIAL_ANGLE_DEG);
  motor_set_angle(&motor2, MOTOR_INITIAL_ANGLE_DEG);
  motor_set_angle(&motor3, MOTOR_INITIAL_ANGLE_DEG);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    ScreenData screenData = screen_read(&screen);

    float x = (screenData.x - screen.width / 2) / SCREEN_POSITION_SCALE;
    float y = -1 * (screenData.y - screen.height / 2) / SCREEN_POSITION_SCALE;

    float dt = pid_compute_dt(&lastTime);

    float newX = pid_update(&pidX, x, dt);
    float newY = pid_update(&pidY, y, dt);

    float angleA = rrs3_calculate_angles(A, &rrs3Options, PLATFORM_HEIGHT_MM,
                                         newX, newY);
    float angleB = rrs3_calculate_angles(B, &rrs3Options, PLATFORM_HEIGHT_MM,
                                         newX, newY);
    float angleC = rrs3_calculate_angles(C, &rrs3Options, PLATFORM_HEIGHT_MM,
                                         newX, newY);

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
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
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
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
