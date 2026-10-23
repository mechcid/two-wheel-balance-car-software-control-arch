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
#include "tim.h"
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
#include "motion_app.h"
#include "status_state_app.h"
#include "lqr_app.h"
#include "trace_app.h"
#include "debug_app.h"
#include "timebase.h"

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
static LqrTrace lqr_trace;
static ImuTrace imu_trace;
static uint32_t last_state_deadline_us;
static bool sample_available(void)
{
  const ImuSample *sample = SensorApp_GetSample();
  return SensorApp_GetStatus()->ready && sample->valid &&
         (uint32_t)(Timebase_NowUs() - sample->timestamp_us) < IMU_TIMEOUT_US;
}
static void display_task(void *context){
  //渲染任务和推送任务被分离了，这里只负责渲染，推到oled的部分被分到了oled_task部分
  (void)context;
  float angles[3];
  MotionApp_GetEuler(angles);
  bool available=sample_available() &&
      (uint32_t)(Timebase_NowUs()-MotionApp_GetStatus()->timestamp_us)<IMU_TIMEOUT_US;
  DisplayApp_Render(MotionApp_GetState(), angles, available);
}

//区分了两个串口任务，分别是串口打印任务
static void report_task(void *context){
  (void)context;
  /* 不需要哪类数据就注释对应调用；@stream 控制全部周期输出。 */

  //TelemetryApp_ReportImu(SensorApp_GetSample(), sample_available());
  //TelemetryApp_ReportAttitude(sample_available());
  //TelemetryApp_ReportMotor(MotorApp_GetStatus());
  TelemetryApp_ReportState(StatusStateApp_GetState());
}
//解析字符串命令任务
static void uart_task(void *context) { 
  (void)context; TelemetryApp_Poll(); 
}

static void oled_task(void *context){
  (void)context;
  /* 8 字节分块，允许在姿态和整车状态计算后的剩余时间刷新。 */
  //很大胆的设计，将i2c总线被mpu占用后的剩余量全都用来推oled，我并不是很喜欢这样做，后续可以看看怎么优化
  if (MotionApp_TimeUntilSampleUs() >= OLED_POLL_BUDGET_US) DisplayApp_Poll();
}
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
  {.function=uart_task,         .period_ms=0,    .enabled=true},
  {.function=display_task,      .period_ms=200,  .enabled=true},
  /* 周期输出共用 9600/8N1，500 ms 周期为命令回执留出带宽。 */
  {.function=report_task,       .period_ms=500,  .enabled=true},
  {.function=oled_task,         .period_ms=0,    .enabled=true},
  {.function=sensor_retry_task, .period_ms=1000, .enabled=true},
  {.function=oled_retry_task,   .context=&hi2c2, .period_ms=2000, .enabled=true},
};
#define TASK_COUNT (sizeof(tasks) / sizeof(tasks[0]))

//定义MPU的零点偏移和缩放倍数
static SensorCalibration calibration = {
  .accel_bias_g = {0.01, 0, 0.2},
  .accel_scale = {-1, -1, -1},//反转一下x，y，z方向上的正负
  .gyro_bias_dps = {-1.75, +1.4, -0.6},
  .gyro_scale = {1, 1, 1},
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
  
  //这里隐含着执行了一次时间基准的初始化
  if (!Timebase_Init()) Error_Handler();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  if (!TelemetryApp_Init(&huart2)) Error_Handler();
  SensorApp_Init(&hi2c2, &calibration); // 初始化传感器应用，传入 I2C 总线和校准参数
  DisplayApp_Init(&hi2c2);
  const MotorAppConfig *motor_config=MotorBoard_Init();
  MotorApp_Init(motor_config, Timebase_NowUs());
  /* 加速度输出保留三轴反号；解算内部恢复为右手坐标系中的比力。 */
  MotionConfig motion_config=MotionEstimator_DefaultConfig();
  if (!MotionApp_Init(&motion_config)) Error_Handler();
  if (motor_config && !StatusStateApp_Init(motor_config->left.radius_m,
                                           motor_config->right.radius_m)) Error_Handler();
  LqrConfig lqr_config=LqrApp_DefaultConfig();
  LqrTuning lqr_tuning=LqrApp_DefaultTuning();
  if (!LqrApp_Init(&lqr_config,&lqr_tuning)) Error_Handler();
  /* 两个记录器拥有独立的类型化数组；初始化不启动记录或电机。 */
  if (!LqrTrace_Init(&lqr_trace) || !ImuTrace_Init(&imu_trace)) Error_Handler();
  if (!DebugApp_Init(&lqr_trace,&imu_trace)) Error_Handler();
  last_state_deadline_us=Timebase_NowUs();
  Scheduler_Init(tasks, TASK_COUNT, HAL_GetTick());

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MotionApp_Poll(); /* 优先处理 200 Hz 采集截止时间；阻塞式 I2C 不在中断内执行。 */
    uint32_t now=Timebase_NowUs();
    StatusStateApp_Poll(now);
    LqrApp_Poll(now);
    uint32_t periods=(uint32_t)(now-last_state_deadline_us)/LQR_PERIOD_US;
    if (periods) {
      last_state_deadline_us+=periods*LQR_PERIOD_US;
      StatusStateApp_Update(now);
      LqrApp_Update(Timebase_NowUs());
    }
    DebugApp_Poll(Timebase_NowUs());
    Scheduler_Run(tasks, TASK_COUNT, HAL_GetTick());
    DebugApp_Poll(Timebase_NowUs());
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
