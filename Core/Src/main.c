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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sensor_app.h"
#include "scheduler.h"
#include "display_app.h"
#include "telemetry_app.h"
#include "oled.h"
#include "motor_app.h"
#include "motor_board.h"

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
static void motor_task(void *context)
{
  (void)context;
  MotorApp_Update(HAL_GetTick());
}
static void sample_task(void *context)
{
  (void)context;
  SensorApp_Update(HAL_GetTick());
}
static bool sample_available(void)
{
  const SensorSample *sample = SensorApp_GetSample();
  return SensorApp_GetStatus()->ready && sample->valid &&
         (uint32_t)(HAL_GetTick() - sample->timestamp_ms) < 500;
}
static void display_task(void *context)
{
  (void)context;
  DisplayApp_Render(SensorApp_GetSample(), sample_available());
}
static void report_task(void *context)
{
  (void)context;
  TelemetryApp_Report(SensorApp_GetSample(), sample_available());
}
static void uart_task(void *context) { (void)context; TelemetryApp_Poll(); }
static void oled_task(void *context) { (void)context; DisplayApp_Poll(); }
static void sensor_retry_task(void *context)
{
  (void)context;
  if (!MotorApp_IsActive() && !SensorApp_GetStatus()->ready) SensorApp_Reconnect();
}
static void oled_retry_task(void *context)
{
  if (!MotorApp_IsActive() && !OLED_IsReady()) DisplayApp_Init(context);
}
/* 应用任务的周期统一在此配置；周期为 0 表示每轮主循环都执行。
 * 故障恢复目前会阻塞等待 MPU 复位和稳定，因此不保证硬实时性。 */
static SchedulerTask tasks[] = {
  {.function=motor_task,        .period_ms=10,   .enabled=true},
  {.function=sample_task,       .period_ms=10,   .enabled=true},
  {.function=uart_task,         .period_ms=0,    .enabled=true},
  {.function=display_task,      .period_ms=200,  .enabled=true},
  {.function=report_task,       .period_ms=200,  .enabled=true},
  {.function=oled_task,         .period_ms=0,    .enabled=true},
  {.function=sensor_retry_task, .period_ms=1000, .enabled=true},
  {.function=oled_retry_task,   .context=&hi2c2, .period_ms=2000, .enabled=true},
};
#define TASK_COUNT (sizeof(tasks) / sizeof(tasks[0]))


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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  if (!TelemetryApp_Init(&huart2)) Error_Handler();
  SensorApp_Init(&hi2c2);
  DisplayApp_Init(&hi2c2);
  MotorApp_Init(MotorBoard_Init(), HAL_GetTick());
  Scheduler_Init(tasks, TASK_COUNT, HAL_GetTick());

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Scheduler_Run(tasks, TASK_COUNT, HAL_GetTick());
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
