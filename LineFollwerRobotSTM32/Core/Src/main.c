
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for line-following robot
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software component, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_COUNT 5
#define MAX_SPEED 100    // Maximum speed
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int weights[5] = {-4, -2, 0, 2, 4}; // Define weights (in mm) for 5 sensors: left to right
float Kp = 2.5;                     // Proportional gain
float Ki = 0.001;                     // Integral gain
float Kd = 0.5;                    // Derivative gain
float dt = 0.005;                   // Time delta (seconds) — adjust to your system
float d = 10.0;                     // Distance between position readings (in mm)
float previous_error = 0;           // Previous error for derivative control
float integral = 0;                 // Accumulated error for integral control
float previous_position = 0;        // Previous position for heading angle
int intersection_count=0;
int grab_count=0;
int has_grabbed = 0; // Global flag to track if grab has occurred
int current_case = 0; // 0 = Case 0, 1 = Case 1, 2 = Case 2, etc.
int sensor_count = 0; // Count how many times all 5 sensors are detected in Case 1

uint8_t sensor_values[SENSOR_COUNT]; // Array to store sensor readings

float x =0;//lateral position
float u =0; //control output
int pwmL =0;
int pwmR=0;
int basespeed=0;

int j=0; // Line-loss

// UART-related variables
uint8_t rx_data;         // Received UART data
uint8_t last_command = 0;// Last received command

uint8_t uto = 'U';       // Mode control: 'U' for auto, 'u' for manual
char grip = 0;           // Gripper state: 'W' for close, 'w' for open
int speed_level = 7;     // Speed level (0-9) for manual mode

static int case1_delay_count = 0;
static int case2_stopped = 0;
static int case2_ignore_count = 0;
static int case2_post_stop_state = 0;
static int case2_post_action_count = 0;
static int case3_has_grabbed = 0;
static int turn_state = 0;
static int turn_count = 0;
static int case3_stopped = 0;
static int case3_ignore_count = 0;
static int case3_post_stop_state = 0;
static int case3_post_action_count = 0;

int caseF_active = 0;     // Flag to track if CaseF is active
int caseF_end_condition = 0; // Flag to track when CaseF should end

// New global variables for Case0 forward push
int case0_initial_push_count = 0; // Counter for initial forward push in Case0
int case0_initial_push_done = 0;  // Flag to track if initial push is complete in Case0

// New global variable for Case1 forward push
int case1_push_after_grab_done = 0; // Flag to track if forward push after grab is done

// New global variable for Case3 forward push
int case3_push_after_grab_count = 0; // Counter for forward push after grab in Case3

int case2_reset_active = 0; // Flag to track if Case2Reset is active


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void set_left_motor(int speed);
void set_right_motor(int speed);
void read_sensors();
float compute_position();
float compute_pid(float error);
float compute_heading(float x_now);
void Set_Servo_Angle(uint8_t angle);
void motor_ctr(int speedL, int speedR);
void followLine(void);
void followLine_Case0(void);
void followLine_Case1(void);
void followLine_Case2(void);
void followLine_Case3(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Set_Servo_Angle(uint8_t angle)
{
  // Map 0–180° to 500–2500us (pulse width in timer counts)
  uint16_t pulse_width = 500 + ((angle * 2000) / 180);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_width);  // Set PWM pulse width
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Start PWM for servo

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Enable UART interrupt
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief Set speed and direction for the left motor
  * @param speed: Speed value (-95 to 95). Positive for forward, negative for backward.
  * @retval None
  */
void set_left_motor(int speed)
{
  speed = (speed > MAX_SPEED) ? MAX_SPEED : (speed < -MAX_SPEED/2) ? -MAX_SPEED/2 : speed;
  TIM4->CCR2 = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (speed > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (speed < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief Set speed and direction for the right motor
  * @param speed: Speed value (-95 to 95). Positive for forward, negative for backward.
  * @retval None
  */
void set_right_motor(int speed)
{
  speed = (speed > MAX_SPEED) ? MAX_SPEED : (speed < -MAX_SPEED/2) ? -MAX_SPEED/2 : speed;
  TIM4->CCR1 = (speed >= 0) ? (uint32_t)speed : (uint32_t)(-speed);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (speed > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (speed < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_ctr(int speedL, int speedR)
{
  set_right_motor(speedR);
  set_left_motor(speedL);
}

/**
  * @brief Read sensor states
  * @retval None
  */
void read_sensors(void)
{
  sensor_values[0] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
  sensor_values[1] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
  sensor_values[2] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
  sensor_values[3] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
  sensor_values[4] = !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

  if (sensor_values[0] == 1) {
    j = 1;
  }
  else if (sensor_values[4] == 1) {
    j = 2;
  }

  if (j == 1) {
    sensor_values[0] = 1;
  }
  else if (j == 2) {
    sensor_values[4] = 1;
  }
  if (sensor_values[1] == 1 || sensor_values[2] == 1 || sensor_values[3] == 1) {
    j = 0;
  }
}

/**
  * @brief Compute lateral position
  * @retval float: Position in mm
  */
float compute_position()
{
  int sum_s = 0;
  int weighted_sum = 0;

  for (int i = 0; i < 5; i++) {
    sum_s += sensor_values[i];
    weighted_sum += sensor_values[i] * weights[i];
  }

  if (sum_s == 0)
    return 0.0; // No line detected, assume center or trigger fail-safe

  return (float)weighted_sum / sum_s;
}

/**
  * @brief Compute PID control output
  * @param error: Current error
  * @retval float: PID control output
  */
float compute_pid(float error)
{
  integral += error * dt;
  float derivative = (error - previous_error) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
  return output;
}

/**
  * @brief Compute heading angle
  * @param x_now: Current position
  * @retval float: Heading angle in radians
  */
float compute_heading(float x_now)
{
  float Q = (x_now - previous_position) / d;
  previous_position = x_now;
  return Q; // in radians (approximation)
}

void followLine(void)
{
  x = compute_position();
  u = compute_pid(x);
  if (fabs(x) < 1) {
    basespeed = 55;
  }
  else if (fabs(x) <= 2) {
    basespeed = 40;
  }
  else if (fabs(x) <= 4) {
    basespeed = 20;
  }
  pwmL = basespeed + u;
  pwmR = basespeed - u;
  motor_ctr(pwmL, pwmR);
}

void followLine_Case0(void)
{


	Set_Servo_Angle(40);

	// Step 2: Initial forward push for 200ms (40 cycles at 5ms)
	    if (!case0_initial_push_done) {
	        pwmL = 50; // Fixed speed for forward push
	        pwmR = 50;
	        motor_ctr(pwmL, pwmR);
	        case0_initial_push_count++;
	        if (case0_initial_push_count >= 40) { // 200ms / 5ms = 40 cycles
	            case0_initial_push_done = 1; // Mark push as complete
	            case0_initial_push_count = 0; // Reset counter for potential reuse
	        }
	        return; // Exit function until push is complete
	    }

  x = compute_position();
  u = compute_pid(x);
  if (fabs(x) < 1) {
    basespeed = 55;
  } else if (fabs(x) <= 2) {
    basespeed = 37;
  } else if (fabs(x) <= 4) {
    basespeed = 25
    		;
  }

  int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];
  if (!has_grabbed) {
    if (sensor_values[0] == 1 && sensor_values[1] == 1 && sensor_values[2] == 1 && sensor_values[3] == 1) {
      Set_Servo_Angle(0); // Grab to 0°
      has_grabbed = 1;    // Prevent further servo commands
      current_case = 1;
    } else if (sensor_values[4] == 1 && sensor_values[3] == 1 && sensor_values[2] == 1 && sensor_values[1] == 1) {
      Set_Servo_Angle(0); // Grab to 0°
      has_grabbed = 1;    // Prevent further servo commands
      current_case = 1;
    } else if (sensor_sum == 5 && sensor_values[2] == 1) { // Ensure center sensor is off too
      Set_Servo_Angle(0); // Grab to 0°
      has_grabbed = 1;    // Prevent further servo commands
      current_case = 1;
    }
  }

  pwmL = basespeed + u;
  pwmR = basespeed - u;
  motor_ctr(pwmL, pwmR);
}

void followLine_Case1(void)
{

  // Set new PID values for Case 1
  Kp = 15;
  Ki = 0.0;
  Kd = 0.0;
  // Reset PID state
  integral = 0;
  previous_error = 0;

  // Step 1: Forward push for 200ms (40 cycles at 5ms) after grabbing
      if (!case1_push_after_grab_done) {
          if (case1_delay_count < 110) {
              pwmL = 30; // Fixed speed for forward push
              pwmR = 30;
              motor_ctr(pwmL, pwmR);
              case1_delay_count++;
              return; // Exit until push is complete
          } else {
              case1_push_after_grab_done = 1; // Mark push as complete
              case1_delay_count = 0; // Reset counter for next phase
          }
      }

  x = compute_position();
  u = compute_pid(x);
  if (fabs(x) < 1) {
    basespeed = 40;
  } else if (fabs(x) <= 2) {
    basespeed = 38;
  } else if (fabs(x) <= 4) {
    basespeed = 40;
  }

  int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];

  // Ignore sensor_sum == 5 for 500ms (100 cycles * 5ms)
  if (case1_delay_count < 100) {
	  case1_delay_count++;
  } else if (sensor_sum == 4) {
    current_case = 2;  // Move to Case 2 when all sensors detect the line
  }

  pwmL = basespeed + u;
  pwmR = basespeed - u;
  motor_ctr(pwmL, pwmR);
}

// Modified followLine_Case2 function
void followLine_Case2(void)
{
  if (!case2_stopped) {
    Kp = 2.5;
    Ki = 0.0;
    Kd = 0.0;
    integral = 0;
    previous_error = 0;

    x = compute_position();
    u = compute_pid(x);
    if (fabs(x) < 1) {
      basespeed = 40;
    } else if (fabs(x) <= 2) {
      basespeed = 40;
    } else if (fabs(x) <= 4) {
      basespeed = 40;
    }

    int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];

    if (case2_ignore_count < 40) {
      case2_ignore_count++;
      pwmL = basespeed + u;
      pwmR = basespeed - u;
      motor_ctr(pwmL, pwmR);
    } else {
      if (sensor_sum == 5 && !case2_stopped) {
        case2_stopped = 1;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
        case2_post_stop_state = 1;
        case2_post_action_count = 0;
      } else if (!case2_stopped) {
        pwmL = basespeed + u;
        pwmR = basespeed - u;
        motor_ctr(pwmL, pwmR);
      }
    }
  } else {
    if (case2_post_stop_state == 1) {
      pwmL = 25;
      pwmR = 25;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 70) {
        case2_post_stop_state = 2;
        case2_post_action_count = 0;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    } else if (case2_post_stop_state == 2) {
      Set_Servo_Angle(40);
      case2_post_action_count++;
      if (case2_post_action_count >= 20) {
        case2_post_stop_state = 3;
        case2_post_action_count = 0;
      }
    } else if (case2_post_stop_state == 3) {
      pwmL = -35;
      pwmR = -35;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 145) {
        case2_post_stop_state = 4;
        case2_post_action_count = 0;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    } else if (case2_post_stop_state == 4) {
      pwmL = 70;
      pwmR = 30;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 100) {
        case2_post_stop_state = 5;
        current_case = 3;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    }
  }
}

void followLine_Case2Reset(void)
{
//	Set_Servo_Angle(0);
  if (!case2_stopped) {
    Kp = 2.5;
    Ki = 0.0;
    Kd = 0.0;
    integral = 0;
    previous_error = 0;

    x = compute_position();
    u = compute_pid(x);
    if (fabs(x) < 1) {
      basespeed = 40;
    } else if (fabs(x) <= 2) {
      basespeed = 40;
    } else if (fabs(x) <= 4) {
      basespeed = 40;
    }

    int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];

    if (case2_ignore_count < 50) {
      case2_ignore_count++;
      pwmL = basespeed + u;
      pwmR = basespeed - u;
      motor_ctr(pwmL, pwmR);
    } else {
      if (sensor_sum == 5 && !case2_stopped) {
        case2_stopped = 1;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
        case2_post_stop_state = 1;
        case2_post_action_count = 0;
      } else if (!case2_stopped) {
        pwmL = basespeed + u;
        pwmR = basespeed - u;
        motor_ctr(pwmL, pwmR);
      }
    }
  } else {
    if (case2_post_stop_state == 1) {
      pwmL = 25;
      pwmR = 25;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 70) {
        case2_post_stop_state = 2;
        case2_post_action_count = 0;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    } else if (case2_post_stop_state == 2) {
      Set_Servo_Angle(40);
      case2_post_action_count++;
      if (case2_post_action_count >= 20) {
        case2_post_stop_state = 3;
        case2_post_action_count = 0;
      }
    } else if (case2_post_stop_state == 3) {
      pwmL = -30;
      pwmR = -30;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 145) {
        case2_post_stop_state = 4;
        case2_post_action_count = 0;
        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    } else if (case2_post_stop_state == 4) {
      pwmL = 20;
      pwmR = -35;
      motor_ctr(pwmL, pwmR);
      case2_post_action_count++;
      if (case2_post_action_count >= 110) {
        case2_post_stop_state = 5;

        // Transition to Case 3 and reset Case2Reset flag
        current_case = 3;
        case2_reset_active = 0; // Clear the Case2Reset flag

        // Reset Case3 variables for fresh start
        case3_has_grabbed = 0;
        case3_push_after_grab_count = 0;

        pwmL = 0;
        pwmR = 0;
        motor_ctr(pwmL, pwmR);
      }
    }
  }
}
void followLine_Case3(void)
{
    if (!case3_stopped) {
        // Set PID values for normal line following
        Kp = 15;
        Ki = 0.0;
        Kd = 0.0;
        integral = 0;
        previous_error = 0;

        x = compute_position();
        u = compute_pid(x);
        if (fabs(x) < 1) {
            basespeed = 35;
        } else if (fabs(x) <= 2) {
            basespeed = 38;
        } else if (fabs(x) <= 4) {
            basespeed = 35;
        }

        int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];

        // Ignore initial sensor readings for 250ms (50 cycles * 5ms)
        if (case3_ignore_count < 50) {
            case3_ignore_count++;
            pwmL = basespeed + u;
            pwmR = basespeed - u;
            motor_ctr(pwmL, pwmR);
        } else {
            // Check for 4 sensors detection after ignore period
            if (sensor_sum == 4 && !case3_stopped) {
                case3_stopped = 1;
                pwmL = 0;
                pwmR = 0;
                motor_ctr(pwmL, pwmR);
                case3_post_stop_state = 1;
                case3_post_action_count = 0;
            } else if (!case3_stopped) {
                // Continue normal line following
                pwmL = basespeed + u;
                pwmR = basespeed - u;
                motor_ctr(pwmL, pwmR);
            }
        }
    } else {
        // Handle post-stop actions sequentially
        if (case3_post_stop_state == 1) {
            // State 1: Brief stop for 100ms (20 cycles * 5ms)
            pwmL = 0;
            pwmR = 0;
            motor_ctr(pwmL, pwmR);
            case3_post_action_count++;
            if (case3_post_action_count >= 20) {
                case3_post_stop_state = 2;
                case3_post_action_count = 0;
            }
        } else if (case3_post_stop_state == 2) {
            // State 2: Left turn for 125ms (25 cycles * 5ms)
            pwmL = -35; // Left motor backward
            pwmR = 20;  // Right motor forward
            motor_ctr(pwmL, pwmR);
            case3_post_action_count++;
            if (case3_post_action_count >= 100) {
                case3_post_stop_state = 3;
                case3_post_action_count = 0;
            }
        } else if (case3_post_stop_state == 3) {
            // State 3: Final stop and transition to Case 4
            pwmL = 0;
            pwmR = 0;
            motor_ctr(pwmL, pwmR);
            case3_post_action_count++;
            if (case3_post_action_count >= 10) { // Brief final stop
                // Transition to Case 4
                current_case = 4;

                // Reset Case3 variables for potential future use
                case3_stopped = 0;
                case3_ignore_count = 0;
                case3_post_stop_state = 0;
                case3_post_action_count = 0;

                // Stop motors
                pwmL = 0;
                pwmR = 0;
                motor_ctr(pwmL, pwmR);
            }
        }
    }
}
void followLine_Case4(void)
{
    // Set new PID values for Case4
    Kp = 10;
    Ki = 0.0;
    Kd = 0.0;
    integral = 0;
    previous_error = 0;

    x = compute_position();
    u = compute_pid(x);
    if (fabs(x) < 1) {
        basespeed = 35; // Slower base speed
    } else if (fabs(x) <= 2) {
        basespeed = 35;
    } else if (fabs(x) <= 4) {
        basespeed = 35;
    }

    static int grab_state = 0;
    static int forward_count = 0;

    if (grab_state == 0) {
        int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];
        if (sensor_sum == 5) {
            Set_Servo_Angle(0); // Grab
            grab_state = 1;
            forward_count = 0;
            pwmL = 20; // Start slow forward
            pwmR = 20;
        } else {
            pwmL = basespeed + u;
            pwmR = basespeed - u;
        }
    } else if (grab_state == 1) {
        pwmL = 30; // Slow forward
        pwmR = 30;
        motor_ctr(pwmL, pwmR);
        forward_count++;
        if (forward_count >= 500) { // 2 seconds (400 cycles * 5ms)
            grab_state = 2;
            pwmL = 0;
            pwmR = 0;
        }
    } else {
        pwmL = 0;
        pwmR = 0;
    }

    motor_ctr(pwmL, pwmR);
}

/* Complete the followLine_CaseF function */
void followLine_CaseF(void){
    // Set PID values for CaseF
    Kp = 18;
    Ki = 0.0;
    Kd = 0.0;
    integral = 0;
    previous_error = 0;

    // Read sensors and compute position
    x = compute_position();
    u = compute_pid(x);

    // Set speed based on position error
    if (fabs(x) < 1) {
        basespeed = 40;
    } else if (fabs(x) <= 2) {
        basespeed = 38;
    } else if (fabs(x) <= 4) {
        basespeed = 40;
    }

    // Calculate motor speeds
    pwmL = basespeed + u;
    pwmR = basespeed - u;
    motor_ctr(pwmL, pwmR);

    // Example end condition: when all sensors detect line (you can modify this)
    int sensor_sum = sensor_values[0] + sensor_values[1] + sensor_values[2] + sensor_values[3] + sensor_values[4];
    if (sensor_sum == 5) {
        caseF_end_condition = 1;  // Set flag to end CaseF
    }

    // Or add other end conditions as needed
    // For example, if no line is detected for a certain time:
    // if (sensor_sum == 0) {
    //     caseF_end_condition = 1;
    // }
}

// Modified execute_command function - FIXED VERSION
void execute_command(void)
{
  if (last_command != 0) {
    // Calculate basespeed properly - ensure it's always reasonable
    basespeed = (98 * speed_level) / 9;
    if (basespeed < 50) basespeed = 50; // Lower minimum for better control

    switch (last_command) {
      case 'F': motor_ctr(basespeed, basespeed); break;
      case 'B': motor_ctr(-basespeed, -basespeed); break;
      case 'L': motor_ctr(-basespeed, basespeed); break;
      case 'R': motor_ctr(basespeed, -basespeed); break;
      case 'G': motor_ctr(basespeed / 2, basespeed); break;
      case 'I': motor_ctr(basespeed, basespeed / 2); break;
      case 'H': motor_ctr(-basespeed / 2, -basespeed); break;
      case 'J': motor_ctr(-basespeed, -basespeed / 2); break;
      case 'S':
        motor_ctr(0, 0);
        last_command = 0; // Clear command after stop
        break;
      // Remove speed commands from here - they're handled in UART callback
      default:
        // Don't clear last_command for unknown commands - keep moving
        break;
    }
    // Don't automatically clear last_command - let it persist for continuous movement
  } else {
    motor_ctr(0, 0);  // Stop if no command
  }
}

/* Modified UART callback - add this to your existing HAL_UART_RxCpltCallback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        HAL_UART_Receive_IT(&huart1, &rx_data, 1); // Re-arm UART interrupt

        if (rx_data == 'W') {
            grip = 'W';          // Close gripper
            last_command = 0;
        }
        else if (rx_data == 'w') {
            grip = 'w';          // Open gripper
            last_command = 0;
        }
        else if (rx_data == 'X') {
            // Start Case2Reset when 'X' is received
            case2_reset_active = 1;
            current_case = 2; // Set to case 2 but will use Case2Reset
            uto = 'u'; // Set to auto mode
            last_command = 0;

            // Reset all Case2 variables for fresh start
            case2_stopped = 0;
            case2_ignore_count = 0;
            case2_post_stop_state = 0;
            case2_post_action_count = 0;

            // Reset PID
            integral = 0;
            previous_error = 0;
            j = 0;
        }
        else if (rx_data == 'V') {
            caseF_active = 1;
            caseF_end_condition = 0;
            uto = 'F';
            last_command = 0;
            integral = 0;
            previous_error = 0;
            j = 0;
            // Reset Case0 and Case1 variables
            case0_initial_push_count = 0;
            case0_initial_push_done = 0;
            case1_delay_count = 0;
            case1_push_after_grab_done = 0;
        }
        else if (rx_data == 'v') {
            caseF_active = 0;
            caseF_end_condition = 0;
            uto = 'U';
            last_command = 0;
            motor_ctr(0, 0);
            // Reset Case0 and Case1 variables
            case0_initial_push_count = 0;
            case0_initial_push_done = 0;
            case1_delay_count = 0;
            case1_push_after_grab_done = 0;
        }
        else if (rx_data == 'u') {
            uto = 'u';
            caseF_active = 0;
            case2_reset_active = 0; // Reset the Case2Reset flag
            last_command = 0;
            current_case = 0;
            has_grabbed = 0;
            sensor_count = 0;
            integral = 0;
            previous_error = 0;
            Kp = 2.5;
            Ki = 0.01;
            Kd = 5.0;
            j = 0;
            speed_level = 7;
            Set_Servo_Angle(40);
            case1_delay_count = 0;
            case2_stopped = 0;
            case2_ignore_count = 0;
            case2_post_stop_state = 0;
            case2_post_action_count = 0;
            case3_has_grabbed = 0;
            // Reset Case0 and Case1 variables
            case0_initial_push_count = 0;
            case0_initial_push_done = 0;
            case1_push_after_grab_done = 0;
        }
        else if (rx_data == 'U') {
            uto = 'U';
            caseF_active = 0;
            case2_reset_active = 0; // Reset the Case2Reset flag
            last_command = 0;
            // Reset Case0 and Case1 variables
            case0_initial_push_count = 0;
            case0_initial_push_done = 0;
            case1_delay_count = 0;
            case1_push_after_grab_done = 0;
        }
        else if (uto == 'U') {
            if (rx_data >= '0' && rx_data <= '9') {
                speed_level = rx_data - '0';
                if (speed_level > 9) speed_level = 9;
            }
            else if (rx_data == 'q') {
                speed_level--;
                if (speed_level < 0) speed_level = 0;
            }
            else {
                last_command = rx_data;
            }
        }
        else if (rx_data >= '0' && rx_data <= '9') {
            speed_level = rx_data - '0';
            if (speed_level > 9) speed_level = 9;
        }
        else if (rx_data == 'q') {
            speed_level--;
            if (speed_level < 0) speed_level = 0;
        }
    }
}

/* Modified Timer callback to handle Case2Reset */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        if (uto == 'u') {
            // Auto mode - execute line-following cases
            read_sensors();  // Update sensor values
            if (current_case == 0) {
                followLine_Case0();
            } else if (current_case == 1) {
                followLine_Case1();
            } else if (current_case == 2) {
                // Check if we should use Case2Reset or normal Case2
                if (case2_reset_active) {
                    followLine_Case2Reset();
                } else {
                    followLine_Case2();
                }
            } else if (current_case == 3) {
                followLine_Case3();
            }
            else if (current_case == 4){
            	followLine_Case4();

            }
        }
        else if (uto == 'F') {
            // CaseF mode - execute CaseF line following
            read_sensors();      // Update sensor values
            followLine_CaseF();  // Execute CaseF

            // Check if CaseF should end and return to manual
            if (caseF_end_condition) {
                caseF_active = 0;
                caseF_end_condition = 0;
                uto = 'U';       // Return to manual mode
                last_command = 0;
                motor_ctr(0, 0); // Stop motors
            }
        }
        else if (uto == 'U') {
            // Manual mode
            execute_command();
        }

        // Handle gripper commands (works in all modes)
        if (grip == 'W') {
            Set_Servo_Angle(0);  // Close gripper
            grip = 0;            // Clear after execution
        }
        else if (grip == 'w') {
            Set_Servo_Angle(40); // Open gripper
            grip = 0;            // Clear after execution
        }
    }
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
