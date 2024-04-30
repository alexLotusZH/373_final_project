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
#include"stdio.h"
#include"string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef enum {
	IDLE,
	LOST,
	NOT_DETECTED,
	TO_LEFT,
	TO_RIGHT,
	READY
} Track_State;

Track_State state;
int auto_enable = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ENABLE_GPIO GPIOA
#define ENABLE_PIN GPIO_PIN_8

#define DIR1_GPIO GPIOB
#define DIR1_PIN GPIO_PIN_10
#define DIR2_GPIO GPIOB
#define DIR2_PIN GPIO_PIN_4
#define DIR3_GPIO GPIOB
#define DIR3_PIN GPIO_PIN_5
#define DIR4_GPIO GPIOB
#define DIR4_PIN GPIO_PIN_3

#define STEP_GPIO GPIOA
#define STEP_PIN GPIO_PIN_7

#define HB1_GPIO  GPIOC
#define HB2_GPIO  GPIOC
#define HB3_GPIO  GPIOC
#define HB4_GPIO  GPIOC
#define HB1_PIN  GPIO_PIN_0
#define HB2_PIN GPIO_PIN_1
#define HB3_PIN GPIO_PIN_2
#define HB4_PIN GPIO_PIN_3
char answer = 0;
char answers[2] = {0};
int answerOffset = 0;

int stepState = 0;
int PINCH_FORWARD_CCR = 205;
int PINCH_BACKWARD_CCR = 320;
int PAN_STOP = 302;
int PAN_SLOW_UP = 310;
int PAN_SLOW_DOWN = 295;
int PAN_UP = 302 + 50;
int PAN_DOWN = 302 - 50;

int FAST_WHEEL_CCR = 1999;
int SLOW_WHEEL_CCR = 199;

#define FIX_TIMER_TRIGGER(handle_ptr) (__HAL_TIM_CLEAR_FLAG(handle_ptr, TIM_SR_UIF))


void HAL_GPIO_EXTI_Callback(uint16_t pin){
	if(pin == GPIO_PIN_12 || pin == GPIO_PIN_13 || pin == GPIO_PIN_14 || pin == GPIO_PIN_15){
		HAL_TIM_Base_Start_IT(&htim5);
	}
}

void step(int num)
{
	HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 1);
	HAL_Delay(5);
	HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 0);
	HAL_Delay(5);
}

void set_forward_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 1);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 1);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-1);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-1);
}
void set_backward_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 0);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 0);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-0);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-0);
}
void set_left_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 0);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 1);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-1);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-0);
}
void set_right_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 1);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 0);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-0);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-1);
}
void set_ccw_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 0);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 0);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-1);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-1);
}
void set_cw_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 1);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 1);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-0);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-0);
}

static int steps_left = 0;
static int disabled = 0;
void enable()
{
	HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 0);
}
void disable()
{
	HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 1);
}

int pan_done = 1;

int controller_move = 0;
int controller_dir = 0;
int ultra_move = 0;
int ultra_steps = 0;
int ultra_dir = 0;
int auto_move = 0;
int auto_steps = 0;
int auto_dir = 0;

void set_dir(int dir)
{
	switch(dir)
	{
	case 0:
		set_forward_dir();
		break;
	case 1:
		set_backward_dir();
		break;
	case 2:
		set_left_dir();
		break;
	case 3:
		set_right_dir();
		break;
	case 4:
		set_ccw_dir();
		break;
	case 5:
		set_cw_dir();
		break;
	}
}
static it_started = 0;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {


    if (htim->Instance == TIM11) {
    	if (controller_move == 0 && ultra_move == 0 && auto_move == 0)
		{
			HAL_TIM_Base_Stop_IT(&htim11);
			it_started = 0;
			disable();
			return;
		}
    	if (ultra_move)
    	{
    		if (stepState)
    		{
    			ultra_steps--;
    			if (ultra_steps == 0)
    			{
    				ultra_move = 0;
    			}
    		}
    		set_dir(ultra_dir);
    	}
    	else if (controller_move)
    	{

    		set_dir(controller_dir);
    	}
    	else
    	{
    		if (stepState)
			{
				auto_steps--;
				if (auto_steps == 0)
				{
					auto_move = 0;
				}
			}
			set_dir(auto_dir);
    	}
    	enable();
		HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, stepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	stepState = !stepState;
    }
    else if (htim->Instance == TIM10)
    {
    	TIM2->CCR1 = PINCH_BACKWARD_CCR;
    	HAL_TIM_Base_Stop_IT(&htim10);
    }
    else if(htim->Instance == TIM5){
		GPIO_PinState ultra1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		GPIO_PinState ultra2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		GPIO_PinState ultra3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
		GPIO_PinState ultra4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
		if (!ultra_move)
		{

			if(ultra1){
				ultra_move = 1;
				 ultra_dir = 1;
				 ultra_steps = 50;
				printf("1\n");
				if (!it_started)
				{
					it_started = 1;
					HAL_TIM_Base_Start_IT(&htim11);
				}
			}
			else if(ultra2){
				ultra_move = 1;
				ultra_dir = 0;
				ultra_steps = 50;
				printf("2\n");
				if (!it_started)
								{
									it_started = 1;
									HAL_TIM_Base_Start_IT(&htim11);
								}
			}
			else if(ultra3){
				ultra_move = 1;
				ultra_dir = 3;
				ultra_steps = 50;
				printf("3\n");
				if (!it_started)
								{
									it_started = 1;
									HAL_TIM_Base_Start_IT(&htim11);
								}
			}
			else if(ultra4)
			{
				ultra_move = 1;
				ultra_dir = 2;
				ultra_steps = 50;
				printf("4\n");
				if (!it_started)
								{
									it_started = 1;
									HAL_TIM_Base_Start_IT(&htim11);
								}
			}
		}
		HAL_TIM_Base_Stop_IT(&htim5);
	}
    else if(htim->Instance == TIM9)
    {
    	printf("tim9 it\n");
    	pan_done = 1;
    	TIM2->CCR2 = PAN_STOP;
    	HAL_TIM_Base_Stop_IT(&htim9);
    }
}
int auto_mode_on = 0;
int to_delay = 0;

void move_async(int dir, int steps)
{
	if (!auto_mode_on)
	{
		return;
	}
	while(auto_move);
	if (to_delay !=0)
	{
		HAL_Delay(to_delay);
		to_delay = 0;
	}
	auto_move = 1;
	auto_steps = steps;
	auto_dir = dir;
	if (!it_started)
	{
		it_started = 1;
		HAL_TIM_Base_Start_IT(&htim11);
	}
}

void prepare_nospin()
{
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 800;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 800;
}
void prepare_full()
{
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 1999;
	TIM3->CCR3 = 0;
	TIM3->CCR4 = 1999;
}
void prepare_topspin()
{
	TIM3->CCR1 = 0;
		TIM3->CCR2 = 499;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 1299;
}
void prepare_underspin()
{
	TIM3->CCR1 = 0;
	 TIM3->CCR2 = 1299;
	 TIM3->CCR3 = 0;
	 TIM3->CCR4 = 499;
}
void shut_down()
{
	TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 0;
}

void rotate_and_shoot()
{
	if (!auto_mode_on)
	{
		return;
	}
	prepare_topspin();
	capture_and_update();
	if (!auto_mode_on)
	{
		return;
	}
	static int is_left[] = {0, 1, 1, 0, 1, 0, 1, 1};
	auto_move = 1;
	auto_steps = 150;
	auto_dir = is_left? 4: 5;
	if (!it_started)
	{
		it_started = 1;
		HAL_TIM_Base_Start_IT(&htim11);
	}
	HAL_Delay(2000);
	if (!auto_mode_on)
	{
		return;
	}
	shoot_async();
	HAL_Delay(500);
	auto_move = 1;
	auto_steps = 150;
	auto_dir = is_left? 5:4;
	if (!it_started)
	{
		it_started = 1;
		HAL_TIM_Base_Start_IT(&htim11);
	}
	HAL_Delay(500);

}

uint16_t number = 0;


uint8_t tx_blocks_buff[]={0xae,0xc1,32,2,1,1};
uint8_t rx_blocks_buff[20];

void detect_object_location(void);
void capture_and_update(void);
void shoot_async()
{

	TIM2->CCR1 = PINCH_FORWARD_CCR;
	FIX_TIMER_TRIGGER(&htim10);
	HAL_TIM_Base_Start_IT(&htim10);
}



void auto_move_func()
{
	if (auto_mode_on == 0)
	{
		return;
	}
	prepare_topspin();
	HAL_Delay(5000);
	int rotates[] = {1, 0, 1, 1, 0, 0, 1, 0};
	static int rotate_index = 0;
	if (auto_mode_on == 0)
	{
		return;
	}
	move_async(5, 320);
	to_delay = 500;
	move_async(0, 1000);
	to_delay = 500;
	move_async(4, 320);
	int rotate = rotates[rotate_index++];
	if (rotate_index ==8)
	{
		rotate_index = 0;
	}
	capture_and_update();
	if (state == TO_LEFT || rotate)
	{
		to_delay = 500;
		move_async(4, 70);
	}
	to_delay = 500;
	move_async(1, 1);
	shoot_async();
	if (state == TO_LEFT || rotate)
	{
		to_delay = 500;
		move_async(5, 70);
	}
	to_delay = 500;
	move_async(4, 320);
	to_delay = 500;
	move_async(0, 1000);
	to_delay = 500;
	move_async(5, 320);
	to_delay = 500;
	rotate = rotates[rotate_index++];
	if (rotate_index ==8)
	{
		rotate_index = 0;
	}
	capture_and_update();
	if (state == TO_RIGHT || rotate)
	{
		to_delay = 500;
		move_async(5, 70);
	}
	to_delay = 500;
	move_async(0, 1);
	shoot_async();

	if (state == TO_RIGHT || rotate)
	{
		to_delay = 500;
		move_async(4, 70);
	}
	to_delay = 300;



//	 move_async(0, 200);
//	  rotate_and_shoot();
//	  HAL_Delay(500);
//	  move_async(2, 200);
//	  rotate_and_shoot();
//	  HAL_Delay(500);
//	  move_async(1, 200);
//	  rotate_and_shoot();
//	  HAL_Delay(500);
//	  move_async(3, 200);
//	  rotate_and_shoot();
//	  HAL_Delay(500);
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
  MX_USART6_UART_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // HAL_TIM_Base_Start_IT(&htim10);
  HAL_UART_Receive_IT(&huart6, &answer, 1);
  HAL_UART_Receive_IT(&huart2, &number, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  // HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 1);
  HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 0);
  HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 0);
  HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 0);
  HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 0);

  disable();

  auto_enable = 1;
  while (1)
  {
	  // HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 1);
	  // HAL_Delay(100);
	  // HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 0);
	  // HAL_Delay(100);

	  printf("hello world!\n");
	  auto_move_func();
	  HAL_Delay(200);


      // move_async(0, 200);
//	  rotate_and_shoot();
//	  HAL_Delay(500);
//	  move_async(2, 200);
//	  rotate_and_shoot();
//	  HAL_Delay(500);
//	  move_async(1, 200);
//	  rotate_and_shoot();
//	  HAL_Delay(500);
//	  move_async(3, 200);
//	  rotate_and_shoot();
//	  HAL_Delay(500);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  htim2.Init.Prescaler = 419;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  htim3.Init.Prescaler = 419;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3999;
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
  sConfigOC.Pulse = 2;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 3999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 8399;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2499;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 4500;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 39;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 3999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD2_Pin PA7 PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}


static int pan_pos = 0;
int pan_dir = 0;




void HAL_UART_RxCpltCallback(UART_HandleTypeDef* hadc) {

	if (hadc == &huart2)
	{
		static char numbers[4] = {0};
		static int offset = 0;


		if (number != '\n')
		{
			printf("received: %c\n", number);
			numbers[offset++] = number;

			if (offset == 3)
			{
				int intNum = atoi(numbers);
				printf("set: %d\n", intNum);
				// TIM2->CCR1 = intNum;
				TIM2->CCR2 = intNum;
				offset = 0;
			}
		}
		HAL_UART_Receive_IT(&huart2, &number, 1);

	}
	else
	{
		printf("raw: %d\n", (int)answer);
		if (answerOffset == 0 && (answer&0b10000000) == 0)
		{
			HAL_UART_Receive_IT(&huart6, &answer, 1);
			printf("Wrong!\n");
			return;
		}
		answers[answerOffset++] = answer;
		if (answerOffset == 2)
		{
			answerOffset = 0;
		}
		else
		{
			HAL_UART_Receive_IT(&huart6, &answer, 1);
			return;
		}
		printf("received: %d\n", *(uint16_t*)(&answers));
		static int hold = 0;
		static int hold_64 = 0;
		static int hold_128 = 0;
		uint16_t short_answers = *(uint16_t*)(&answers) - 128;
		printf("processed: %d\n", short_answers);
		switch(short_answers)
		{
		case 1:
			controller_dir = 4;
			break;
		case 2:
			controller_dir = 5;
			break;
		case 4:
			controller_dir = 0;
			break;
		case 8:
			controller_dir = 1;
			break;
		case 16:
			shoot_async();
			break;
		case 32:
			static int spin_state = 0;

			if (!hold)
			{
				hold = 1;
				spin_state = (spin_state + 1)%5;
			}
			if (spin_state == 0)
			{
				shut_down();
			}
			else if(spin_state == 1)
			{
				prepare_nospin();
			}
			else if(spin_state == 2)
			{
				prepare_full();
			}
			else if (spin_state == 3)
			{
				prepare_topspin();
			}
			else
			{
				prepare_underspin();
			}
			break;
		case 64:
			if (!hold_64 && pan_pos > 0 && pan_done)
			{
				hold_64 = 1;
				pan_done = 0;
				pan_pos--;
				pan_dir = 0;
				TIM2->CCR2 = PAN_DOWN;
				FIX_TIMER_TRIGGER(&htim9);
				HAL_TIM_Base_Start_IT(&htim9);
			}
			break;
		case 256*16:
			if (!hold_128 && pan_pos < 3 && pan_done)
			{
				hold_128 = 1;
				pan_done = 0;
				pan_pos++;
				pan_dir = 1;
				TIM2->CCR2 = PAN_UP;
				FIX_TIMER_TRIGGER(&htim9);
				HAL_TIM_Base_Start_IT(&htim9);
			}
			break;
		case 256:
			auto_mode_on = 1;
			break;
		case 512:
			auto_mode_on = 0;
			auto_move = 0;
			auto_steps = 0;
			break;

		}


		if ((short_answers & 0b1111) == 0)
		{
			controller_move = 0;
		}
		if ((short_answers & 0b1111) != 0)
		{
			controller_move = 1;
			HAL_TIM_Base_Start_IT(&htim11);
			it_started = 1;
		}
		if (short_answers != 64)
		{
			hold_64 = 0;
		}
		if (short_answers != 256*16)
		{
			hold_128 = 0;
		}
		disabled = (short_answers & 0b1111) == 0;
		if ((short_answers & 32) == 0)
		{
			hold = 0;
		}
		 HAL_UART_Receive_IT(&huart6, &answer, 1);
	}
 }



void detect_object_location()
{
	const int sig = 1;
	const int tolerate_x = 30;   // tolerance in x direction in pixels
	const int tolerate_y = 20;   // tolerance in x direction in pixels
	int x_pos, y_pos;

	HAL_UART_Transmit(&huart1, tx_blocks_buff, 6, 1000);
	HAL_UART_Receive(&huart1, rx_blocks_buff, 20, 1000);
	HAL_Delay(10);
	int sanity = 0;
	int if_detected = 0;
	sanity = (rx_blocks_buff[0] == 175) && (rx_blocks_buff[1] == 193) && (rx_blocks_buff[2] == 33);
	if_detected = (rx_blocks_buff[3] == 14);

	for(int i = 0; i< 20; i++){
		printf("%d   ", rx_blocks_buff[i]);
	}
	printf("\n\r");

	// light a led for debug if pixy cam is offline
	if (sanity == 1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // switch on the led
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		return;
	}

	if (if_detected == 1) {
		x_pos = rx_blocks_buff[8];
		y_pos = rx_blocks_buff[10];
		printf("target detected at (%d,%d)! \n\r", x_pos, y_pos);
		//
	} else {
		printf("target not detected! \n\r");
		// write to gpio to trigger a interrupt
		return;
	}

	HAL_Delay(100);
}

void capture_and_update() {
	HAL_UART_Transmit(&huart1, tx_blocks_buff, 6, 1000);
	HAL_UART_Receive(&huart1, rx_blocks_buff, 20, 1000);
	HAL_Delay(5);

	for(int i = 0; i< 20; i++){
		printf("%d   ", rx_blocks_buff[i]);
	}
	printf("\n");

	const int tolerate_x = 30;   // tolerance in x direction in pixels
	const int ref_x = 157;
	const int tolerate_y = 20;   // tolerance in x direction in pixels
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	switch (state) {
	case IDLE:
		if (enable) {
			state = LOST;
		}
		break;
	case LOST:
		if ((rx_blocks_buff[0] == 175) && (rx_blocks_buff[1] == 193) && (rx_blocks_buff[2] == 33)) {
			state = NOT_DETECTED;
		}
		if (!enable) {
			state = IDLE;
		}
		break;
	case NOT_DETECTED:

	case TO_RIGHT:

	case TO_LEFT:
			if (!enable) {
				state = IDLE;
				break;
			}
			if ((rx_blocks_buff[0] == 175) && (rx_blocks_buff[1] == 193) && (rx_blocks_buff[2] == 33)) {
				if (rx_blocks_buff[3] == 14) {
					// detected
					int x_pos = rx_blocks_buff[8];
					int y_pos = rx_blocks_buff[10];
					printf("target detected at (%d,%d)! \n\r", x_pos, y_pos);
					if (x_pos > ref_x + tolerate_x) {
						state = TO_RIGHT;
					} else if (x_pos < ref_x - tolerate_x) {
						state = TO_LEFT;
					} else {
						state = READY;
					}
				} else {
					state = NOT_DETECTED;
				}
			} else {
				state = LOST;
			}
			break;
	case READY:
		// set the gpio for launching the ball
		// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//		enable = 0;
		state = IDLE;
		break;
	}

	printf("current state is %d \n\r", state);

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
