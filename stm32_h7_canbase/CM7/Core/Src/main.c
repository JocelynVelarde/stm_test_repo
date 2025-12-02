/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (FreeRTOS + CAN Ticks/Yaw)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "myprintf.h" 
#include "servo.h"
#include "motion.h"   
#include "constants.h"
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U)
#endif


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_FilterTypeDef sFilterConfig;
FDCAN_TxHeaderTypeDef TxHeader;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim13;
UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN PV */
// --- FreeRTOS Handles ---
osMutexId_t printfMutexHandle;
const osMutexAttr_t printfMutex_attributes = {
  .name = "PrintfMutex"
};

osMessageQueueId_t sensorQueueHandle;
const osMessageQueueAttr_t sensorQueue_attributes = {
  .name = "SensorQueue"
};

osMessageQueueId_t cameraQueueHandle;
const osMessageQueueAttr_t cameraQueue_attributes = {
  .name = "CameraQueue"
};

osThreadId_t motionTaskHandle;
const osThreadAttr_t motionTask_attributes = {
  .name = "MotionTask",
  .stack_size = 512 * 4, 
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

// Camera State
volatile float Global_Cam_X = 0.0f;
volatile float Global_Cam_Y = 0.0f;
volatile uint8_t Camera_Seen_Target = 0;

// Control and Odometry
typedef enum {
    STATE_RUNNING,
    STATE_WAITING
} RobotState_t;

Motion_t robotMotion;
RobotState_t motionState = STATE_RUNNING;
float Global_Robot_X = 0.0f;
float Global_Robot_Y = 0.0f;
volatile float Global_Robot_Yaw = 0.0f;   
volatile int32_t Global_Robot_Ticks = 0;  
uint32_t waitStartTime = 0;
volatile ControlCmd_t g_ctrl = {0.0f, 0.0f};
volatile OdomMeasurement_t gOdom = {0.0f};

// --- Navigation Path ---
Waypoint_t path[] = {
    {30.0f, 0.0f},
    {80.0f, 0.0f},
    {120.0f, 0.0f},
	  {180.0f, 80.0f}
};
int total_waypoints = sizeof(path) / sizeof(path[0]);
int current_wp_idx = 0;
uint8_t finished_path = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void StartMotionTask(void *argument);
void StartTelemetryTask(void *argument);
void StartActuatorTask(void *argument);
void setEscSpeed_us(uint16_t pulse_us);
int32_t getTicks(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
      FDCAN_RxHeaderTypeDef localHeader;
      uint8_t localRxData[8];
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &localHeader, localRxData) == HAL_OK)
      {
        if (localHeader.Identifier == CAN_ID_SENSOR || localHeader.Identifier == 0x00)
        {
          if (localHeader.DataLength == FDCAN_DLC_BYTES_8)
          {
            float tempYaw;
            int32_t tempTicks;
            
            memcpy(&tempYaw, &localRxData[0], sizeof(float));
            memcpy(&tempTicks, &localRxData[4], sizeof(int32_t));

            if (!isnan(tempYaw) && !isinf(tempYaw))
            {
              SensorData_t data;
              data.yaw = tempYaw;
              data.ticks = tempTicks;

              osMessageQueuePut(sensorQueueHandle, &data, 0, 0);
            }
          }
        }
        else if (localHeader.Identifier == CAN_ID_CAMERA)
        {
          CameraData_t camData;

          camData.x     = (int16_t)((localRxData[0] << 8) | localRxData[1]);
          camData.y     = (int16_t)((localRxData[2] << 8) | localRxData[3]);
          camData.angle = (int16_t)((localRxData[4] << 8) | localRxData[5]);

          osMessageQueuePut(cameraQueueHandle, &camData, 0, 0);
        }
      }
    }
}

int32_t getTicks(void)
{
    return Global_Robot_Ticks; 
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
  
  /* USER CODE BEGIN Boot_Mode_Sequence_0 */
  int32_t timeout;
  /* USER CODE END Boot_Mode_Sequence_0 */

  /* USER CODE BEGIN Boot_Mode_Sequence_1 */
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 ) { Error_Handler(); }
  /* USER CODE END Boot_Mode_Sequence_1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN Boot_Mode_Sequence_2 */
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0,0);
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
  if ( timeout < 0 ) { Error_Handler(); }
  /* USER CODE END Boot_Mode_Sequence_2 */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_FDCAN1_Init();
  MX_TIM13_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1); // Servo
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  // ESC

  Motion_Init(&robotMotion);
  Motion_SetPath(&robotMotion, path, total_waypoints);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  printfMutexHandle = osMutexNew(&printfMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_QUEUES */
  sensorQueueHandle = osMessageQueueNew(1, sizeof(SensorData_t), &sensorQueue_attributes);
  cameraQueueHandle = osMessageQueueNew(1, sizeof(CameraData_t), &cameraQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);
  motionTaskHandle = osThreadNew(StartMotionTask, NULL, &motionTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  while (1) {
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Motion Task: Handles Physics, PID, and ESC/Servo
  */
void StartMotionTask(void *argument)
{
  SensorData_t sensorData;
  CameraData_t cameraData;

  int32_t last_ticks = 0; 
  int32_t current_ticks = 0;
  
  osDelay(100);
  last_ticks = getTicks();

  const uint32_t TASK_PERIOD_MS = 20;
  const float DT_SECONDS = 0.02f; 
  float filtered_yaw = 0.0f;
  const float ALPHA_FILTER = 0.2f;
  
  TickType_t last_tick_time = xTaskGetTickCount();
  motionState = STATE_RUNNING;
  for(;;)
  {
    TickType_t current_tick_time = xTaskGetTickCount();
    float actual_dt = (float)(current_tick_time - last_tick_time) / (float)configTICK_RATE_HZ;
    last_tick_time = current_tick_time;
    
    if (actual_dt < 0.001f) actual_dt = DT_SECONDS;
    if (actual_dt > 0.1f)   actual_dt = DT_SECONDS;

    while (osMessageQueueGet(sensorQueueHandle, &sensorData, NULL, 0) == osOK)
    {
      float raw_yaw = sensorData.yaw;
      
      float yaw_diff = raw_yaw - filtered_yaw;
      if (yaw_diff > 180.0f) yaw_diff -= 360.0f;
      if (yaw_diff < -180.0f) yaw_diff += 360.0f;
      
      filtered_yaw = filtered_yaw + ALPHA_FILTER * yaw_diff;
      
      while (filtered_yaw >= 180.0f) filtered_yaw -= 360.0f;
      while (filtered_yaw < -180.0f) filtered_yaw += 360.0f;
      
      Global_Robot_Yaw = filtered_yaw;
      Global_Robot_Ticks = sensorData.ticks;
    }

    while (osMessageQueueGet(cameraQueueHandle, &cameraData, NULL, 0) == osOK)
    {
      Global_Cam_X = (float)cameraData.x;
      Global_Cam_Y = (float)cameraData.y;
    }

    current_ticks = getTicks();
    int32_t delta = current_ticks - last_ticks;
    last_ticks = current_ticks;

    if (delta != 0) {
      float circumference_cm = 2.0f * M_PI * WHEEL_RADIUS_M * 100.0f;
      float revolutions = (float)delta / (float)TICKS_PER_REV;
      float dist_cm = circumference_cm * revolutions;
      
      Global_Robot_X += dist_cm * cosf(Global_Robot_Yaw * DEG_TO_RAD);
      Global_Robot_Y += dist_cm * sinf(Global_Robot_Yaw * DEG_TO_RAD);
    }

    float target_x = robotMotion.target_x;
    float target_y = robotMotion.target_y;
    
    float dx_wp = target_x - Global_Robot_X;
    float dy_wp = target_y - Global_Robot_Y;
    float dist_to_wp = sqrtf(dx_wp*dx_wp + dy_wp*dy_wp);
    float current_speed = gOdom.wheel_v_mps;

    Motion_Update(&robotMotion, Global_Robot_X, Global_Robot_Y, Global_Robot_Yaw, current_speed, actual_dt);

    switch (motionState) {
      case STATE_RUNNING:
          Motion_Update(&robotMotion, Global_Robot_X, Global_Robot_Y, Global_Robot_Yaw, current_speed, actual_dt);
          if (dist_to_wp < TARGET_THRESHOLD) {
            setEscSpeed_us(1500); 
            g_ctrl.v_cmd = 0.0f;
            g_ctrl.delta_cmd = 0.0f; 
            
            HAL_GPIO_WritePin(FLAG_INDICATOR_GPIO_Port, FLAG_INDICATOR_Pin, GPIO_PIN_SET);
            
            printf(">>> REACHED WP %d at (%.1f, %.1f)\r\n", robotMotion.current_wp_idx + 1, Global_Robot_X, Global_Robot_Y);

            motionState = STATE_WAITING;
            waitStartTime = HAL_GetTick();
          }
          break;

      case STATE_WAITING:
          if (gOdom.wheel_v_mps > 0.1f) { 
            setEscSpeed_us(1350); 
          } 
          else if (gOdom.wheel_v_mps < -0.1f) {
            setEscSpeed_us(1650);
          }
          else {
            setEscSpeed_us(1500);
          }
          
          g_ctrl.v_cmd = 0.0f; // Update PID target to 0

          if (HAL_GetTick() - waitStartTime >= 2000) {
            HAL_GPIO_WritePin(FLAG_INDICATOR_GPIO_Port, FLAG_INDICATOR_Pin, GPIO_PIN_RESET);
            Motion_NextWaypoint(&robotMotion);
            if (robotMotion.path_finished) {
                printf(">>> PATH FINISHED \r\n");
            } else {
                motionState = STATE_RUNNING;
            }
          }
          break;
    }

    float dx_cam = target_x - Global_Cam_X;
    float dy_cam = target_y - Global_Cam_Y;
    float dist_cam = sqrtf(dx_cam*dx_cam + dy_cam*dy_cam);

    if (dist_cam < CAM_THRESHOLD) {
        HAL_GPIO_WritePin(FLAG_INDICATOR_GPIO_Port, FLAG_INDICATOR_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(FLAG_INDICATOR_GPIO_Port, FLAG_INDICATOR_Pin, GPIO_PIN_RESET);
    }

    osDelay(TASK_PERIOD_MS); 
  }
}

/**
  * @brief  Telemetry Task
  */
void StartTelemetryTask(void *argument)
{
  for(;;)
  {
    printf("WP: %d/%d | Odo: (%.0f, %.0f, %.1f deg) | Cam: ... \r\n", 
       robotMotion.current_wp_idx + 1,
       robotMotion.path_size,
       Global_Robot_X, Global_Robot_Y, Global_Robot_Yaw);
    
    osDelay(200);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  for(;;)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
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

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 8;
  hfdcan1.Init.NominalTimeSeg1 = 0x1F;
  hfdcan1.Init.NominalTimeSeg2 = 8;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 1;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x000;
  sFilterConfig.FilterID2 = 0x000;

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
       Error_Handler();
    }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    }

   TxHeader.Identifier = 0x111;
   TxHeader.IdType = FDCAN_STANDARD_ID;
   TxHeader.TxFrameType = FDCAN_DATA_FRAME;
   TxHeader.DataLength = FDCAN_DLC_BYTES_8 ;
   TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
   TxHeader.BitRateSwitch = FDCAN_BRS_ON;
   TxHeader.FDFormat = FDCAN_FD_CAN;
   TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
   TxHeader.MessageMarker = 0x00;
  /* USER CODE END FDCAN1_Init 2 */

}

static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 1500;
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

static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 199;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 19999;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */
  HAL_TIM_MspPostInit(&htim13);

}

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

  /* USER CODE END USART3_Init 2 */

}

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLAG_INDICATOR_GPIO_Port, FLAG_INDICATOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FLAG_INDICATOR_Pin */
  GPIO_InitStruct.Pin = FLAG_INDICATOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLAG_INDICATOR_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
