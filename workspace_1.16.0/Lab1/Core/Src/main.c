/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "kalman_c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
#include <math.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *)(0xE0000000 + 4 * n)))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
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
  /* USER CODE BEGIN 2 */

  float TEST_ARRAY[] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706,
                        10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493,
                        9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356,
                        9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039,
                        10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479,
                        10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721,
                        10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814,
                        10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024,
                        9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599,
                        10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259,
                        10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732,
                        10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876,
                        10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898,
                        10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583,
                        9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608,
                        10.3465431058, 9.98446410493, 9.79376005657, 10.202518901,
                        9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804,
                        10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909,
                        9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596,
                        10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178,
                        9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226,
                        10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928,
                        9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087,
                        9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486,
                        10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991,
                        9.5799256668};

  float test_array_output_C[101];
  float test_array_output_ass[101];
  float test_array_output_cmsis[101];
  int l = 101;
  int output;
  int output2;

  ITM_Port32(31) = 1;

  for (int z = 0; z < 1000; z++)
  { // 1000 execution of kalman filter in C
    kalman_state state = {0.1, 0.1, 0.1, 5, 0};
    output = Kalmanfilter(TEST_ARRAY, test_array_output_C, &state, l);
  }
  ITM_Port32(31) = 2;

  for (int z = 0; z < 1000; z++)
  { // 1000 execution of kalman filter in ASSEMBLY
    float stateArr[5] = {0.1, 0.1, 0.1, 5, 0};
    for (int i = 0; i < l; i++)
    {
      test_array_output_ass[i] = kalman_assem(TEST_ARRAY[i], &stateArr);
    }
  }
  ITM_Port32(31) = 3;

  for (int z = 0; z < 1000; z++)
  { // 1000 execution of kalman filter in CMSIS
    kalman_state state2 = {0.1, 0.1, 0.1, 5, 0};
    output2 = KalmanfilterCMSIS(TEST_ARRAY, test_array_output_cmsis, &state2, l);
  }

  ITM_Port32(31) = 4;

  //if kalman function in C or in CMSIS return -1, then we had an error
  if(output == -1 || output2 == -1){
	  return -1;
  }

  // SUBTRACTION OF DATA POINTS IN C
  float output_differences[101];
  for (int z = 0; z < 101; z++)
  {
    output_differences[z] = TEST_ARRAY[z] - test_array_output_C[z];
  }

  // SUBTRACTION IN CMSIS
  float negated_output[101];
  float output_differences_CMSIS[101];
  arm_negate_f32(test_array_output_cmsis, negated_output, 101);
  arm_add_f32(TEST_ARRAY, negated_output, output_differences_CMSIS, 101);

  float sum = 0.0, average, variance = 0.0, stddev;
  // sum of all differences
  for (int z = 0; z < 101; z++)
  {
    sum += output_differences[z];
  }
  // AVERAGE
  average = sum / 101;

  // variance
  for (int z = 0; z < 101; z++)
  {
    variance += pow(output_differences[z] - average, 2); // Sum of squared differences from the mean
  }
  variance /= 101; // Divide by number of elements to get the variance

  // STANDARD DEVIATION
  stddev = sqrt(variance);

  // AVERAGE IN CMSIS
  float cmsis_average[1];
  arm_mean_f32(output_differences_CMSIS, 101, cmsis_average);

  // STANDARD DEV IN CMSIS
  float cmsis_stddev[1];
  arm_std_f32(output_differences_CMSIS, 101, cmsis_stddev);

  // CMSIS CORRELATION
  float corr_CMSIS[2 * 101 - 1];
  arm_correlate_f32(TEST_ARRAY, 101, test_array_output_cmsis, 101, corr_CMSIS);

  // CMSIS CONVOLUTION
  float conv_CMSIS[2 * 101 - 1];
  arm_conv_f32(TEST_ARRAY, 101, test_array_output_cmsis, 101, conv_CMSIS);

  // C CONVOLUTION
  float conv_result[2 * 101 - 1];
  for (int z = 0; z < 201; z++)
  {
    conv_result[z] = 0;
  }
  for (int i = 0; i < 101; i++)
  {
    for (int z = 0; z < 101; z++)
    {
      conv_result[i + z] += TEST_ARRAY[i] * test_array_output_cmsis[z];
    }
  }

  // C CORRELATION
  float corr_result[2 * 101 - 1];
  for (int z = 0; z < 201; z++)
  {
    corr_result[z] = 0;
  }
  for (int i = 0; i < 101; i++)
  {
    for (int z = 0; z < 101; z++)
    {
      corr_result[i + z] += TEST_ARRAY[i] * test_array_output_cmsis[101 - 1 - z];
    }
  }

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

  /** Configure the main internal regulator output voltage
   */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
