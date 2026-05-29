/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  int16_t Accel_X;
  int16_t Accel_Y;
  int16_t Accel_Z;

  int16_t Gyro_X;
  int16_t Gyro_Y;
  int16_t Gyro_Z;
} MPU6050_Data_t;

typedef enum
{
  TAP_STATE_IDLE = 0,
  TAP_STATE_WAIT_SECOND
} TapState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MPU_ADDR                (0x68 << 1)

#define MPU_DEVICE_ID_REG       0x75
#define MPU_PWR_MGMT1_REG       0x6B
#define MPU_ACCEL_CONFIG_REG    0x1C
#define MPU_GYRO_CONFIG_REG     0x1B
#define MPU_ACCEL_XOUTH_REG     0x3B

#define ACCEL_SCALE_2G          16384.0f

/*
  双击检测参数
  acc_mag = sqrt(ax^2 + ay^2 + az^2)

  静止时 acc_mag 约等于 1g。
  敲击时 acc_mag 会瞬间变大。
*/
#define TAP_ACC_THRESHOLD_G     1.60f
#define TAP_ACC_RELEASE_G       1.25f

#define TAP_MIN_INTERVAL_MS     150U
#define TAP_MAX_INTERVAL_MS     600U
#define TAP_COOLDOWN_MS         1000U

#define SCREEN_ON_TIME_MS       5000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define SCREEN_LED_ON()   HAL_GPIO_WritePin(LED_T_GPIO_Port, LED_T_Pin, GPIO_PIN_SET)
#define SCREEN_LED_OFF()  HAL_GPIO_WritePin(LED_T_GPIO_Port, LED_T_Pin, GPIO_PIN_RESET)

/* 如果你的板子 LED 是低电平亮，就改成：
#define SCREEN_LED_ON()   HAL_GPIO_WritePin(LED_T_GPIO_Port, LED_T_Pin, GPIO_PIN_RESET)
#define SCREEN_LED_OFF()  HAL_GPIO_WritePin(LED_T_GPIO_Port, LED_T_Pin, GPIO_PIN_SET)
*/

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

osMutexId_t uartMutexHandle;

static MPU6050_Data_t mpu6050_data;

/* 双击状态机变量 */
static TapState_t tap_state = TAP_STATE_IDLE;
static uint32_t first_tap_tick = 0;
static uint32_t last_trigger_tick = 0;
static uint8_t tap_armed = 1;

/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for LED_Task */
osThreadId_t LED_TaskHandle;
const osThreadAttr_t LED_Task_attributes = {
  .name = "LED_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};

/* Definitions for Log_Task */
osThreadId_t Log_TaskHandle;
const osThreadAttr_t Log_Task_attributes = {
  .name = "Log_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void app_printf(const char *format, ...);

static uint8_t MPU6050_CheckID(void);
static uint8_t MPU6050_InitBasic(void);
static uint8_t MPU6050_ReadRaw(MPU6050_Data_t *data);

static uint32_t MsToTicks(uint32_t ms);
static uint8_t Gesture_DetectDoubleTap(float ax_g, float ay_g, float az_g);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LedTaskFunc(void *argument);
void LogTaskFunc(void *argument);
void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  uartMutexHandle = osMutexNew(NULL);
  if (uartMutexHandle == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  LED_TaskHandle = osThreadNew(LedTaskFunc, NULL, &LED_Task_attributes);
  Log_TaskHandle = osThreadNew(LogTaskFunc, NULL, &Log_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  /* USER CODE END RTOS_EVENTS */
}

/**
  * @brief defaultTask
  */
void StartDefaultTask(void *argument)
{
  for (;;)
  {
    osDelay(1000);
  }
}

/**
  * @brief LED_Task
  * 这里不要再闪 LED，因为 LED 要模拟屏幕亮灭。
  */
void LedTaskFunc(void *argument)
{
  for (;;)
  {
    osDelay(1000);
  }
}

/**
  * @brief Log_Task：读取 MPU6050，检测双击，控制 LED 亮屏
  */
void LogTaskFunc(void *argument)
{
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;

  float acc_mag_sq = 0.0f;

  uint8_t double_tap = 0;
  uint8_t screen_on = 0;

  uint32_t screen_on_tick = 0;
  uint32_t now_tick = 0;

  osDelay(1000);

  SCREEN_LED_OFF();

  if (MPU6050_CheckID() != 0)
  {
    app_printf("MPU6050 check failed\r\n");

    for (;;)
    {
      osDelay(1000);
    }
  }

  if (MPU6050_InitBasic() != 0)
  {
    app_printf("MPU6050 init failed\r\n");

    for (;;)
    {
      osDelay(1000);
    }
  }

  /*
    如果你要用 VOFA+ FireWater，
    建议不要输出文字，只输出数字。
  */
  app_printf("Double tap wake demo start\r\n");

  for (;;)
  {
    double_tap = 0;

    if (MPU6050_ReadRaw(&mpu6050_data) == 0)
    {
      ax_g = (float)mpu6050_data.Accel_X / ACCEL_SCALE_2G;
      ay_g = (float)mpu6050_data.Accel_Y / ACCEL_SCALE_2G;
      az_g = (float)mpu6050_data.Accel_Z / ACCEL_SCALE_2G;

      acc_mag_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g;

      double_tap = Gesture_DetectDoubleTap(ax_g, ay_g, az_g);

      if (double_tap)
      {
        screen_on = 1;
        screen_on_tick = osKernelGetTickCount();
        SCREEN_LED_ON();
      }

      if (screen_on)
      {
        now_tick = osKernelGetTickCount();

        if ((now_tick - screen_on_tick) >= MsToTicks(SCREEN_ON_TIME_MS))
        {
          screen_on = 0;
          SCREEN_LED_OFF();
        }
      }

      /*
        VOFA+ FireWater：
        CH0 = acc_mag_sq
        CH1 = double_tap
        CH2 = screen_on

        acc_mag_sq 静止时约等于 1.0。
        敲击时会出现尖峰。
      */
      app_printf("%.3f,%d,%d\r\n",
                 acc_mag_sq,
                 double_tap,
                 screen_on);
    }

    osDelay(20);
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

static uint8_t MPU6050_CheckID(void)
{
  uint8_t who = 0;

  if (HAL_I2C_IsDeviceReady(&hi2c1, MPU_ADDR, 3, 100) != HAL_OK)
  {
    app_printf("MPU6050 not ready\r\n");
    return 1;
  }

  if (HAL_I2C_Mem_Read(&hi2c1,
                       MPU_ADDR,
                       MPU_DEVICE_ID_REG,
                       I2C_MEMADD_SIZE_8BIT,
                       &who,
                       1,
                       100) != HAL_OK)
  {
    app_printf("Read WHO_AM_I failed\r\n");
    return 2;
  }

  app_printf("WHO_AM_I = 0x%02X\r\n", who);

  if ((who == 0x68) || (who == 0x70))
  {
    return 0;
  }

  return 3;
}

static uint8_t MPU6050_InitBasic(void)
{
  uint8_t data = 0;

  /*
    PWR_MGMT_1 = 0x00
    退出睡眠模式
  */
  data = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c1,
                        MPU_ADDR,
                        MPU_PWR_MGMT1_REG,
                        I2C_MEMADD_SIZE_8BIT,
                        &data,
                        1,
                        100) != HAL_OK)
  {
    return 1;
  }

  /*
    ACCEL_CONFIG = 0x00
    加速度量程 ±2g
  */
  data = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c1,
                        MPU_ADDR,
                        MPU_ACCEL_CONFIG_REG,
                        I2C_MEMADD_SIZE_8BIT,
                        &data,
                        1,
                        100) != HAL_OK)
  {
    return 2;
  }

  /*
    GYRO_CONFIG = 0x00
    陀螺仪量程 ±250 dps
    虽然双击主要用加速度，但这里顺手初始化好。
  */
  data = 0x00;
  if (HAL_I2C_Mem_Write(&hi2c1,
                        MPU_ADDR,
                        MPU_GYRO_CONFIG_REG,
                        I2C_MEMADD_SIZE_8BIT,
                        &data,
                        1,
                        100) != HAL_OK)
  {
    return 3;
  }

  return 0;
}

static uint8_t MPU6050_ReadRaw(MPU6050_Data_t *data)
{
  uint8_t buf[14];

  if (HAL_I2C_Mem_Read(&hi2c1,
                       MPU_ADDR,
                       MPU_ACCEL_XOUTH_REG,
                       I2C_MEMADD_SIZE_8BIT,
                       buf,
                       14,
                       100) != HAL_OK)
  {
    return 1;
  }

  data->Accel_X = (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  data->Accel_Y = (int16_t)(((uint16_t)buf[2] << 8) | buf[3]);
  data->Accel_Z = (int16_t)(((uint16_t)buf[4] << 8) | buf[5]);

  data->Gyro_X = (int16_t)(((uint16_t)buf[8] << 8) | buf[9]);
  data->Gyro_Y = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
  data->Gyro_Z = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

  return 0;
}

static uint32_t MsToTicks(uint32_t ms)
{
  return (uint32_t)(((uint64_t)ms * osKernelGetTickFreq()) / 1000U);
}

/**
  * @brief 双击检测
  * @return 1：检测到双击；0：未检测到
  */
static uint8_t Gesture_DetectDoubleTap(float ax_g, float ay_g, float az_g)
{
  float acc_mag_sq;
  uint32_t now_tick;
  uint32_t dt_tick;

  uint32_t min_interval_tick = MsToTicks(TAP_MIN_INTERVAL_MS);
  uint32_t max_interval_tick = MsToTicks(TAP_MAX_INTERVAL_MS);
  uint32_t cooldown_tick     = MsToTicks(TAP_COOLDOWN_MS);

  uint8_t tap_event = 0;

  const float tap_threshold_sq = TAP_ACC_THRESHOLD_G * TAP_ACC_THRESHOLD_G;
  const float tap_release_sq   = TAP_ACC_RELEASE_G * TAP_ACC_RELEASE_G;

  now_tick = osKernelGetTickCount();

  acc_mag_sq = ax_g * ax_g + ay_g * ay_g + az_g * az_g;

  /*
    冷却时间内不再触发，防止连续亮屏。
  */
  if ((now_tick - last_trigger_tick) < cooldown_tick)
  {
    return 0;
  }

  /*
    重新武装：
    只有加速度幅值回落到 release 阈值以下，
    才允许检测下一次敲击。
    这样可以避免一次敲击的震荡被当成多次敲击。
  */
  if (acc_mag_sq < tap_release_sq)
  {
    tap_armed = 1;
  }

  /*
    检测一次新的敲击峰值。
  */
  if ((tap_armed == 1) && (acc_mag_sq > tap_threshold_sq))
  {
    tap_event = 1;
    tap_armed = 0;
  }

  if (tap_state == TAP_STATE_IDLE)
  {
    if (tap_event)
    {
      first_tap_tick = now_tick;
      tap_state = TAP_STATE_WAIT_SECOND;
    }
  }
  else if (tap_state == TAP_STATE_WAIT_SECOND)
  {
    dt_tick = now_tick - first_tap_tick;

    if (tap_event)
    {
      if ((dt_tick >= min_interval_tick) && (dt_tick <= max_interval_tick))
      {
        tap_state = TAP_STATE_IDLE;
        last_trigger_tick = now_tick;
        return 1;
      }
      else if (dt_tick > max_interval_tick)
      {
        /*
          超过最大间隔后来的敲击，当成新的第一次敲击。
        */
        first_tap_tick = now_tick;
      }
    }

    if (dt_tick > max_interval_tick)
    {
      tap_state = TAP_STATE_IDLE;
    }
  }

  return 0;
}

static void app_printf(const char *format, ...)
{
  va_list args;

  if (uartMutexHandle != NULL)
  {
    osMutexAcquire(uartMutexHandle, osWaitForever);
  }

  va_start(args, format);
  vprintf(format, args);
  va_end(args);

  if (uartMutexHandle != NULL)
  {
    osMutexRelease(uartMutexHandle);
  }
}

/* USER CODE END Application */