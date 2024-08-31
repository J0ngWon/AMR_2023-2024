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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*SR431*/
#define MIN_PULSE 500
#define MAX_PULSE 4000

#define MOTOR1_MIN_ANGLE 0
#define MOTOR1_MAX_ANGLE 180

#define MOTOR2_MIN_ANGLE 15
#define MOTOR2_MAX_ANGLE 165

#define MOTOR3_MIN_ANGLE 0
#define MOTOR3_MAX_ANGLE 180

#define MOTOR4_MIN_ANGLE 0
#define MOTOR4_MAX_ANGLE 180

#define MOTOR5_MIN_ANGLE 0
#define MOTOR5_MAX_ANGLE 180

//#define MOTOR5_MIN_ANGLE 10
//#define MOTOR5_MAX_ANGLE 72
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	if (ch == '\n')
		HAL_UART_Transmit(&huart2, (uint8_t*) "\r", 1, 0xFFFF);
	HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void Pwm_Left(int pwm_input);
void Pwm_Right(int pwm_input);
void motorControl(int in1, int in2);
void motorControl_r(int in1, int in2);
void check_pwm() ;
void check_pwm_r() ;
void check_move_state();
void check_move_state_r();
void cmd_vel_calculate(void);
uint32_t millis(void);

float radiansToDegrees(float radians);
uint16_t angleToPulse(uint8_t angle);
uint8_t limitAngle(uint8_t angle, uint8_t minAngle, uint8_t maxAngle);
void setMotorAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle, uint8_t minAngle, uint8_t maxAngle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double test=0;
/*********************** encoder ***********************/
int targetCount = 3172; //run one cycle
volatile long long int encoderCount = 0;    //left
volatile long long int encoderCount_r = 0;  //right

volatile long long int rpm_encoderCount = 0;
volatile long long int rpm_encoderCount_r = 0;

volatile long long int pr_encoderCount = 0;    //left
volatile long long int pr_encoderCount_r = 0;  //right

double angle = 0.00198082765043492637986295295289; //will fix

double current_angle = 0;
double target_angle = 0;
double current_angle_r = 0;
double target_angle_r = 0;

/********************** PID **************************/
float speed = 0;
float speed_r = 0;
float slowdown = 0;
float slowdown_r = 0;

float Kp = 4, Kd = 1.6, Ki = 8;
float P_term = 0, D_term = 0, I_term = 0;
float pid = 0;

float Kp_a = 5, Kd_a = 1.3, Ki_a = 6.25;
float P_term_a = 0, D_term_a = 0, I_term_a = 0;
float pid_a = 0;

float error = 0;
float previous_error = 0;
float old_error = 0;

float error_a = 0;
float previous_error_a = 0;
float old_error_a = 0;

int stop_flag = 0;
/******************** Dead Reckoning ***********************/
double car_angle = 0;
double current_x = 0;
double current_y = 0;

double last_x = 0;
double last_y = 0;
double last_th = 0;

double previous_car_angle = 0;
double previous_current_x = 0;
double previous_current_y = 0;

double setting_car_angle = 0;
double setting_x = 0;
double setting_y = 0;

double delta_s = 0;
double delta_o = 0;

double last_delta_s = 0;
double last_delta_o = 0;

double input_delta_o = 0;
double pid_delta_o = 0;

double ll = 0.33;  //will fix
double rr = 0.0625; //will fix


/******************** direction *********************/
int control_1 = 0, control_2 = 1;
int control_1_r = 1, control_2_r = 0;

int move_state = 0;  // front:0 back:1
int move_state_r = 0;

/******************** USART *********************/
char rx_buffer[256];
char rx_data;
int rx_index = 0;

char motor_buffer[49];
float motor_degree[6];

double topic_x = 0;
double topic_y = 0;
double topic_th = 0;
double dt = 0;

double topic_vx = 0;
double topic_vy = 0;

float cmd_vel_x = 0;
float cmd_vel_y = 0;
float cmd_vel_z = 0;

float cmd_vel_s = 0;
float cmd_vel_th = 0;

/******************** Time *********************/
extern volatile uint32_t msTicks;
uint32_t time=0;

/******************** O.W *********************/
float radiansToDegrees(float radians) {
	return radians * (180.0 / M_PI);
}

uint16_t angleToPulse(uint8_t angle) {
	return (MIN_PULSE + (MAX_PULSE - MIN_PULSE) * (angle * 0.55555556) / 180);
}

uint8_t limitAngle(uint8_t angle, uint8_t minAngle, uint8_t maxAngle) {
	if (angle <= minAngle) return minAngle;
	if (angle >= maxAngle) return maxAngle;
	return angle;
}

void setMotorAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle, uint8_t minAngle, uint8_t maxAngle) {
	uint8_t limitedAngle = limitAngle(angle, minAngle, maxAngle);
	uint16_t pulse = angleToPulse(limitedAngle);
	__HAL_TIM_SET_COMPARE(htim, channel, pulse);
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM14_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOA, shield_power_Pin, GPIO_PIN_SET);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	motorControl(control_1, control_2);
	motorControl_r(control_1_r, control_2_r);
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		motorControl(control_1, control_2);
		motorControl_r(control_1_r, control_2_r);
		//Pwm_Left(50);
		//Pwm_Right(50);

		Pwm_Left(abs(speed));
		Pwm_Right(abs(speed_r));


		/******************** Dead Reckoning ***********************/
		if(msTicks>=39){
			time=msTicks;
			current_angle = ((rpm_encoderCount - pr_encoderCount) * angle);
			current_angle_r = ((rpm_encoderCount_r - pr_encoderCount_r) * angle);

			pr_encoderCount=rpm_encoderCount;
			pr_encoderCount_r=rpm_encoderCount_r;

			delta_s = ((current_angle_r + current_angle) * rr) / 2;
			delta_o = ((current_angle_r - current_angle) * rr) / ll;
			input_delta_o = previous_car_angle + (delta_o / 2);

			current_x = previous_current_x + cos((input_delta_o)) * delta_s;
			current_y = previous_current_y + sin((input_delta_o)) * delta_s;
			car_angle = previous_car_angle + delta_o;

			pid_delta_o = delta_o;
			dt = time;
			topic_x += (current_x - previous_current_x);
			topic_y += (current_y - previous_current_y);
			topic_th += (car_angle - previous_car_angle);
			topic_vx = (current_x - previous_current_x)*1000/dt;
			topic_vy = (current_y - previous_current_y)*1000/dt;


			printf("1 %f %f %f %f %f %f \n",topic_x,topic_y,topic_th,dt,topic_vx,topic_vy);
			printf("2 %f %f %f %f %f %f \n", motor_degree[0], motor_degree[1], motor_degree[2], motor_degree[3], motor_degree[4], motor_degree[5]);
			previous_car_angle = car_angle;
			previous_current_x = current_x;
			previous_current_y = current_y;

			encoderCount = 0;
			encoderCount_r = 0;

			last_delta_s = sqrt((current_x - last_x) * (current_x - last_x) + (current_y - last_y) * (current_y - last_y));
			last_delta_o = (car_angle - last_th);

			/******************** PID ***********************/
			error = (cmd_vel_s - (delta_s / 0.039));
			error_a = (cmd_vel_th - (pid_delta_o / 0.039));
			P_term = (error - previous_error) * Kp;
			D_term = (error - 2 * previous_error + old_error) * Kd;
			I_term = error * Ki;
			pid = P_term + D_term + I_term;

			P_term_a = (error_a - previous_error_a) * Kp_a;
			D_term_a = (error_a - 2 * previous_error_a + old_error_a) * Kd_a;
			I_term_a = error_a * Ki_a;
			pid_a = P_term_a + D_term_a + I_term_a;
			test+=pid_a;
			if (stop_flag == 0) {
				speed = speed + pid - pid_a;
				speed_r = speed_r + pid + pid_a;
			} else {
				speed = speed - (speed) / 5;
				speed_r = speed_r - (speed_r) / 5;
			}

			old_error = previous_error;
			old_error_a = previous_error_a;
			previous_error = error;
			previous_error_a = error_a;

			check_pwm();
			check_pwm_r();

			check_move_state();
			check_move_state_r();

			cmd_vel_calculate();

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 84-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 41;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 255;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 84-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 20000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|shield_power_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Left_IN2_Pin|Right_IN1_Pin|Right_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Left_IN1_GPIO_Port, Left_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin shield_power_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|shield_power_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_IN2_Pin Right_IN1_Pin Right_IN2_Pin */
  GPIO_InitStruct.Pin = Left_IN2_Pin|Right_IN1_Pin|Right_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Left_IN1_Pin */
  GPIO_InitStruct.Pin = Left_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Left_IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Left_pulse_B_Pin Left_pulse_A_Pin */
  GPIO_InitStruct.Pin = Left_pulse_B_Pin|Left_pulse_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_pulse_B_Pin Right_pulse_A_Pin */
  GPIO_InitStruct.Pin = Right_pulse_B_Pin|Right_pulse_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Pwm_Left(int pwm_input) {
	htim5.Instance->CCR1 = pwm_input;
}

void Pwm_Right(int pwm_input) {
	htim5.Instance->CCR2 = pwm_input;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* Prevent unused argument(s) compilation warning */

	if (GPIO_Pin == Left_pulse_A_Pin) {
		if (HAL_GPIO_ReadPin(GPIOB, Left_pulse_A_Pin)
				== HAL_GPIO_ReadPin(GPIOB, Left_pulse_B_Pin)) {
			encoderCount--;
			rpm_encoderCount--;

		} else {
			encoderCount++;
			rpm_encoderCount++;
		}
	} else if (GPIO_Pin == Left_pulse_B_Pin) {
		if (HAL_GPIO_ReadPin(GPIOB, Left_pulse_A_Pin)
				== HAL_GPIO_ReadPin(GPIOB, Left_pulse_B_Pin)) {
			encoderCount++;
			rpm_encoderCount++;
		} else {
			encoderCount--;
			rpm_encoderCount--;
		}
	}
	if (GPIO_Pin == Right_pulse_A_Pin) {
		if (HAL_GPIO_ReadPin(GPIOA, Right_pulse_A_Pin)
				== HAL_GPIO_ReadPin(GPIOA, Right_pulse_B_Pin)) {
			encoderCount_r++;
			rpm_encoderCount_r++;
		} else {
			encoderCount_r--;
			rpm_encoderCount_r--;
		}
	} else if (GPIO_Pin == Right_pulse_B_Pin) {
		if (HAL_GPIO_ReadPin(GPIOA, Right_pulse_A_Pin)
				== HAL_GPIO_ReadPin(GPIOA, Right_pulse_B_Pin)) {
			encoderCount_r--;
			rpm_encoderCount_r--;
		} else {
			encoderCount_r++;
			rpm_encoderCount_r++;
		}
	}
	UNUSED(GPIO_Pin);
	/* NOTE: This function Should not be modified, when the callback is needed,
			 the HAL_GPIO_EXTI_Callback could be implemented in the user file */
}

void motorControl(int in1, int in2) {
	GPIO_PinState state1=GPIO_PIN_RESET,state2=GPIO_PIN_RESET;
	if(in1==1){
		state1=GPIO_PIN_SET;
	}
	else{
		state1=GPIO_PIN_RESET;
	}
	if(in2==1){
		state2=GPIO_PIN_SET;
	}
	else{
		state2=GPIO_PIN_RESET;
	}
	HAL_GPIO_WritePin(GPIOB, Left_IN1_Pin, state1);
	HAL_GPIO_WritePin(GPIOC, Left_IN2_Pin, state2);
}

void motorControl_r(int in1, int in2) {
	GPIO_PinState state1=GPIO_PIN_RESET,state2=GPIO_PIN_RESET;
	if(in1==1){
		state1=GPIO_PIN_SET;
	}
	else{
		state1=GPIO_PIN_RESET;
	}
	if(in2==1){
		state2=GPIO_PIN_SET;
	}
	else{
		state2=GPIO_PIN_RESET;
	}
	HAL_GPIO_WritePin(GPIOC, Right_IN1_Pin, state1);
 	HAL_GPIO_WritePin(GPIOC, Right_IN2_Pin, state2);
}

void check_pwm() {
	if (speed >= 180) {
		speed = 180;
	} else if (speed <= -180) {
		speed = -180;
	}
}

void check_pwm_r() {
	if (speed_r >= 180) {
		speed_r = 180;
	} else if (speed_r <= -180) {
		speed_r = -180;
	}
}

void check_move_state() {
	if (speed >= 0) {
		move_state = 0;
		control_1 = 0, control_2 = 1;
	} else {
		move_state = 1;
		control_1 = 1, control_2 = 0;
	}
}

void check_move_state_r() {
	if (speed_r >= 0) {
		move_state_r = 0;
		control_1_r = 1, control_2_r = 0;
	} else {
		move_state_r = 1;
		control_1_r = 0, control_2_r = 1;
	}
}

void cmd_vel_calculate(void) {
	cmd_vel_s = cmd_vel_x;
	cmd_vel_th = cmd_vel_z;
	if (cmd_vel_x == 0 && cmd_vel_z == 0) {
		stop_flag = 1;
	} else {
		stop_flag = 0;
	}
}

uint32_t millis(void) {
	return msTicks;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if(msTicks>30){
			msTicks = 0;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (rx_data != '\n') {
			rx_buffer[rx_index++] = rx_data;
		}
		else {
			rx_buffer[rx_index] = '\0';
			//printf("Data: %s\n", rx_buffer);

			if (rx_buffer[0] == '1') {
				//printf("[1] Data: %s\n", rx_buffer);

				char first_number[10];

				strncpy(first_number, &rx_buffer[2], 9);
				first_number[10] = '\0';

				cmd_vel_x = atof(first_number);
				if (rx_buffer[1] == '0') { // ?��?��?�� 경우
					cmd_vel_x = -cmd_vel_x;
				}

				char second_number[10];
				strncpy(second_number, &rx_buffer[12], 9);
				second_number[10] = '\0';

				cmd_vel_z = atof(second_number);
				if (rx_buffer[11] == '0') { // ?��?��?�� 경우
					cmd_vel_z = -cmd_vel_z;
				}
			}
			else if (rx_buffer[0] == '2') {
				//printf("[2] Data: %s\n", rx_buffer);

				//Data_import
				for (int i = 0; i < 48; i++) {
					motor_buffer[i] = rx_buffer[i + 1];
				}

				//Slicing Data and char to float
				for (int i = 0; i < 6; i++) {
					char temp[9];

					for (int j = 0; j < 8; j++) {
						temp[j] = motor_buffer[i * 8 + j];
					}
					temp[8] = '\0';

					motor_degree[i] = radiansToDegrees(atof(temp));;
				}


				//move motor
				setMotorAngle(&htim14, TIM_CHANNEL_1, motor_degree[0], MOTOR1_MIN_ANGLE, MOTOR1_MAX_ANGLE);
				setMotorAngle(&htim4, TIM_CHANNEL_1, motor_degree[1], MOTOR2_MIN_ANGLE, MOTOR2_MAX_ANGLE);
				setMotorAngle(&htim3, TIM_CHANNEL_2, motor_degree[2], MOTOR3_MIN_ANGLE, MOTOR3_MAX_ANGLE);
				setMotorAngle(&htim2, TIM_CHANNEL_3, motor_degree[3], MOTOR4_MIN_ANGLE, MOTOR4_MAX_ANGLE);
				setMotorAngle(&htim3, TIM_CHANNEL_1, motor_degree[4], MOTOR5_MIN_ANGLE, MOTOR5_MAX_ANGLE);
				//setMotorAngle(&htim3, TIM_CHANNEL_1, motor_degree[4], MOTOR5_MIN_ANGLE, MOTOR5_MAX_ANGLE);
			}

			//reset huart2 buffer
			rx_index = 0;
			memset(rx_buffer, 0, sizeof(rx_buffer));
		}
	}
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1);
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
