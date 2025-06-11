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
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SPR 2  // Seconds per revolution Microseconds between steps (adjust for speed control)
#define STEP_DELAY 20 //78 -> 1 second per rotation, linear scaling (156 is 2 seconds per rotation)
/* Debounce variables */
#define DEBOUNCE_DELAY 35         // Debounce delay in milliseconds (adjust as needed)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum {
    IDLE,
	MOTOR_ACTUATION,
	RECIEVE_DATA,
	CONFIRM_RECIEVE_DATA,
	TRIGGER_INTERRUPT,
	TRANSMIT_DATA,
	CALIBRATION,
}State;

volatile State state = IDLE;

const uint8_t START_BYTE = 0x24;

//Creates the byte used to confirm if the received bytes are a valid message
uint8_t check = 0;

//RPI will send desired position in the form of steps
typedef struct{
	int z_step;
	int y_step;
	int x_step;
	int rotation_step;
	int nail_step;
	uint8_t nail;
	uint8_t vacuum;
	uint8_t cal;
}position_data;

volatile uint8_t position_data_recieve_buf[sizeof(position_data)];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART2){
		if(check == START_BYTE){
			check = 0;
			HAL_UART_Receive(huart, position_data_recieve_buf, sizeof(position_data_recieve_buf), HAL_MAX_DELAY);
			state = RECIEVE_DATA;
		}else {
			state = TRIGGER_INTERRUPT;
		}
	}
}

position_data position_data_recieve;

void unpack_position_data(volatile uint8_t* buffer, position_data* data){
	memcpy(data, buffer, sizeof(position_data));
}

typedef struct{
	  int step;
	  uint8_t run; //purely dependent on the limit switch
}stepper_motor;

 typedef struct{
	  GPIO_TypeDef *direction_port;
	  uint16_t direction_pin;
	  GPIO_TypeDef *step_port;
	  uint16_t step_pin;
	  uint8_t ccw_dir;
	  uint8_t cw_dir;

	  stepper_motor stepper;
 }motor_data;

volatile motor_data motor_z;
volatile motor_data motor_y;
volatile motor_data motor_x;
volatile motor_data motor_r;
volatile motor_data motor_n;


/* Debounce variables */
uint32_t last_interrupt_time = 0; // Timestamp of the last interrupt

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//z motors
    if (GPIO_Pin == GPIO_PIN_6 || GPIO_Pin == GPIO_PIN_0) // Check if the interrupt is from PA6
    {
        uint32_t current_time = HAL_GetTick(); // Get current time in ms
        if ((current_time - last_interrupt_time) > DEBOUNCE_DELAY)
        {
            // Rising edge detected (after debounce)
            last_interrupt_time = current_time;

            motor_z.stepper.run = !motor_z.stepper.run;//HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); // Confirms that its pull up? - high until clicked?
//            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, motor_z.stepper.run);
            if(motor_z.stepper.run){
            	state = TRANSMIT_DATA;
            }
        }
    } //y motors
    else if (GPIO_Pin == GPIO_PIN_1 || GPIO_Pin == GPIO_PIN_2) // Check if the interrupt is from PA6
	{
		uint32_t current_time = HAL_GetTick(); // Get current time in ms
		if ((current_time - last_interrupt_time) > DEBOUNCE_DELAY)
		{
			// Rising edge detected (after debounce)
			last_interrupt_time = current_time;

			motor_y.stepper.run = !motor_y.stepper.run;//HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); // Confirms that its pull up? - high until clicked?
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, motor_y.stepper.run);
			if(motor_y.stepper.run){
				state = TRANSMIT_DATA;
			}
		}
	}//x motors
    else if (GPIO_Pin == GPIO_PIN_3 || GPIO_Pin == GPIO_PIN_4) // Check if the interrupt is from PA6
	{
		uint32_t current_time = HAL_GetTick(); // Get current time in ms
		if ((current_time - last_interrupt_time) > DEBOUNCE_DELAY)
		{
			// Rising edge detected (after debounce)
			last_interrupt_time = current_time;

			motor_x.stepper.run = !motor_x.stepper.run;//HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); // Confirms that its pull up? - high until clicked?
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, motor_x.stepper.run);
			if(motor_x.stepper.run){
				state = TRANSMIT_DATA;
			}
		}
	}//r motors
    else if (GPIO_Pin == GPIO_PIN_5 || GPIO_Pin == GPIO_PIN_7) // Check if the interrupt is from PA6
	{
		uint32_t current_time = HAL_GetTick(); // Get current time in ms
		if ((current_time - last_interrupt_time) > DEBOUNCE_DELAY)
		{
			// Rising edge detected (after debounce)
			last_interrupt_time = current_time;

			motor_r.stepper.run = !motor_r.stepper.run;//HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); // Confirms that its pull up? - high until clicked?
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, motor_r.stepper.run);
			if(motor_r.stepper.run){
				state = TRANSMIT_DATA;
			}
		}
	}//n motor
    else if (GPIO_Pin == GPIO_PIN_8 || GPIO_Pin == GPIO_PIN_9) // Check if the interrupt is from PA6
	{
		uint32_t current_time = HAL_GetTick(); // Get current time in ms
		if ((current_time - last_interrupt_time) > DEBOUNCE_DELAY)
		{
			// Rising edge detected (after debounce)
			last_interrupt_time = current_time;

			motor_n.stepper.run = !motor_n.stepper.run;//HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6); // Confirms that its pull up? - high until clicked?
//			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, motor_r.stepper.run);
			if(motor_n.stepper.run){
				state = TRANSMIT_DATA;
			}
		}
	}
}

void delayMicroseconds(uint16_t us) {
    // Reset the timer counter
    __HAL_TIM_SET_COUNTER(&htim6, 0);
    // Start the timer
    HAL_TIM_Base_Start(&htim6);

    // Wait until the timer counter reaches the desired delay in microseconds
    while (__HAL_TIM_GET_COUNTER(&htim6) < us);

    // Stop the timer after the delay is completed
    HAL_TIM_Base_Stop(&htim6);
}

void stepper_move_steps(motor_data* motor, uint32_t steps, uint8_t direction) { // CW - 1, CCW - 0

    if(direction){
		HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, motor->cw_dir);
    }else{
    	HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, motor->ccw_dir);
    }

    for (int i = 0; i < steps && motor->stepper.run; i++) {
    	if(direction){
    		motor->stepper.step++;
    	}else{
    		motor->stepper.step--;
    	}
    	// Generate a step pulse
        HAL_GPIO_WritePin(motor->step_port, motor->step_pin, 1);   // Set PA1 high
//        delayMicroseconds((SPR*1000000 / (steps*2)));                // Short delay (half of STEP_DELAY)
        delayMicroseconds(STEP_DELAY);

        HAL_GPIO_WritePin(motor->step_port, motor->step_pin, 0); // Set PA1 low
//        delayMicroseconds((SPR*1000000 / (steps*2)));                 // Short delay (half of STEP_DELAY)
        delayMicroseconds(STEP_DELAY);
    }


    if(!motor->stepper.run){
    	direction = !direction;

        if(direction){
    		HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, motor->cw_dir);
        }else{
        	HAL_GPIO_WritePin(motor->direction_port, motor->direction_pin, motor->ccw_dir);
        }

        while(!motor->stepper.run) {
        	if(direction){
        		motor->stepper.step++;
        	}else{
        		motor->stepper.step--;
        	}
        	// Generate a step pulse
            HAL_GPIO_WritePin(motor->step_port, motor->step_pin, 1);   // Set PA1 high
    //        delayMicroseconds((SPR*1000000 / (steps*2)));                // Short delay (half of STEP_DELAY)
            delayMicroseconds(STEP_DELAY);

            HAL_GPIO_WritePin(motor->step_port, motor->step_pin, 0); // Set PA1 low
    //        delayMicroseconds((SPR*1000000 / (steps*2)));                 // Short delay (half of STEP_DELAY)
            delayMicroseconds(STEP_DELAY);
        }
    }
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
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  //init
  motor_z.direction_port = GPIOA;
  motor_z.direction_pin = GPIO_PIN_4;
  motor_z.step_port = GPIOA;
  motor_z.step_pin = GPIO_PIN_1;
  motor_z.cw_dir = 1;
  motor_z.ccw_dir = 0;
  motor_z.stepper.run = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);

  motor_y.direction_port = GPIOA;
  motor_y.direction_pin = GPIO_PIN_7;
  motor_y.step_port = GPIOA;
  motor_y.step_pin = GPIO_PIN_6;
  motor_y.cw_dir = 1;
  motor_y.ccw_dir = 0;
  motor_y.stepper.run = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

  motor_x.direction_port = GPIOA;
  motor_x.direction_pin = GPIO_PIN_9;
  motor_x.step_port = GPIOA;
  motor_x.step_pin = GPIO_PIN_8;
  motor_x.cw_dir = 1;
  motor_x.ccw_dir = 0;
  motor_x.stepper.run = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4);

  motor_r.direction_port = GPIOA;
  motor_r.direction_pin = GPIO_PIN_11;
  motor_r.step_port = GPIOA;
  motor_r.step_pin = GPIO_PIN_10;
  motor_r.cw_dir = 1;
  motor_r.ccw_dir = 0;
  motor_r.stepper.run = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) && HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);

  motor_n.direction_port = GPIOC;
  motor_n.direction_pin = GPIO_PIN_10;
  motor_n.step_port = GPIOC;
  motor_n.step_pin = GPIO_PIN_9;
  motor_n.cw_dir = 1;
  motor_n.ccw_dir = 0;
  motor_n.stepper.run = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6));

  uint32_t target_steps = 0;
  uint8_t dir = 0;

  uint8_t motor_data_transmit[5*sizeof(stepper_motor)];  // Array to hold `step` and `run`


//  stepper_move_steps(&motor_1, target_steps, motor_1.cw); // Move the motor by target_steps steps

//  HAL_Delay(1000);  // Wait for 1 second

//  Stepper_SetDirection(0);   // Set direction to counterclockwise (0)
//  stepper_move_steps(&motor_1, target_steps, motor_1.ccw); // Move the motor by target_steps steps


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6));
	  switch(state){
		  case IDLE:
			  state = TRIGGER_INTERRUPT;
//			  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1){
//				  state = CALIBRATION;
//				  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1){
//
//				  }
//			  }
			  break;
		  case CALIBRATION:
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0); // vacuum

			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0); // nailgun
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);

			  stepper_move_steps(&motor_z, UINT32_MAX, 0);
			  stepper_move_steps(&motor_y, UINT32_MAX, 0);
			  stepper_move_steps(&motor_x, UINT32_MAX, 0);
			  stepper_move_steps(&motor_r, UINT32_MAX, 0);
			  stepper_move_steps(&motor_n, UINT32_MAX, 0);

			  motor_z.stepper.step = 0;
			  motor_y.stepper.step = 0;
			  motor_x.stepper.step = 0;
			  motor_z.stepper.step = 0;
			  motor_r.stepper.step = 0;
			  motor_n.stepper.step = 0;

			  state = TRANSMIT_DATA;
//			  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1){
//				  state = TRIGGER_INTERRUPT;
//				  while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != 1){
//
//				  }
//				  stepper_move_steps(&motor_z, UINT32_MAX, 0);
//				  stepper_move_steps(&motor_y, UINT32_MAX, 0);
//				  stepper_move_steps(&motor_x, UINT32_MAX, 0);
//
//				  motor_z.stepper.step = 0;
//				  motor_y.stepper.step = 0;
//				  motor_x.stepper.step = 0;
//			  }
			  break;
		  case TRIGGER_INTERRUPT:
			  //Will not queue interrupts
			  HAL_UART_Receive_IT(&huart2, &check, 1);
			  break;
		  case RECIEVE_DATA:
			  unpack_position_data(position_data_recieve_buf, &position_data_recieve);
			  state = CONFIRM_RECIEVE_DATA;
			  break;
		  case CONFIRM_RECIEVE_DATA:
//			  HAL_UART_Transmit(&huart2, (uint8_t*)&START_BYTE, sizeof(START_BYTE), HAL_MAX_DELAY);
//			  HAL_UART_Transmit(&huart2, (uint8_t*)&position_data_recieve, sizeof(position_data_recieve), HAL_MAX_DELAY);
			  state = MOTOR_ACTUATION;
			  break;
		  case TRANSMIT_DATA:
//			  HAL_UART_Transmit(&huart2, (uint8_t*)&START_BYTE, sizeof(START_BYTE), HAL_MAX_DELAY);
//  			  HAL_UART_Transmit(&huart2, (uint8_t*)&motor_x, sizeof(motor_x), HAL_MAX_DELAY);

			  // Populate the array
			  memcpy(motor_data_transmit, &motor_z.stepper, sizeof(stepper_motor));
			  memcpy(motor_data_transmit + sizeof(stepper_motor), &motor_y.stepper, sizeof(stepper_motor));
			  memcpy(motor_data_transmit + 2*sizeof(stepper_motor), &motor_x.stepper, sizeof(stepper_motor));
			  memcpy(motor_data_transmit + 3*sizeof(stepper_motor), &motor_r.stepper, sizeof(stepper_motor));
			  memcpy(motor_data_transmit + 4*sizeof(stepper_motor), &motor_n.stepper, sizeof(stepper_motor));

			  // Transmit the array
			  HAL_UART_Transmit(&huart2, motor_data_transmit, sizeof(motor_data_transmit), HAL_MAX_DELAY);
			  state = TRIGGER_INTERRUPT;
			  break;
		  case MOTOR_ACTUATION:
			  if(position_data_recieve.cal){
				  state = CALIBRATION;
			  }else {
//				  target_steps = abs(position_data_recieve.z_step- motor_z.stepper.step); //make this position specific
//				  if(target_steps > 0){
//					  if (position_data_recieve.z_step - motor_z.stepper.step > 0) {
//						  dir = 1;  // Positive direction
//					  }else {
//						  dir = 0;  // Negative direction
//					  }
//					  stepper_move_steps(&motor_z, target_steps, dir);
//				  }

				  target_steps = abs(position_data_recieve.y_step- motor_y.stepper.step); //make this position specific
				  if(target_steps > 0){
					  if (position_data_recieve.y_step - motor_y.stepper.step > 0) {
						  dir = 1;  // Positive direction
					  }else {
						  dir = 0;  // Negative direction
					  }
					  stepper_move_steps(&motor_y, target_steps, dir);
				  }

				  target_steps = abs(position_data_recieve.x_step- motor_x.stepper.step); //make this position specific
				  if(target_steps > 0){
					  if (position_data_recieve.x_step - motor_x.stepper.step > 0) {
						  dir = 1;  // Positive direction
					  }else {
						  dir = 0;  // Negative direction
					  }
					  stepper_move_steps(&motor_x, target_steps, dir);
				  }

				  target_steps = abs(position_data_recieve.z_step- motor_z.stepper.step); //make this position specific
				  if(target_steps > 0){
					  if (position_data_recieve.z_step - motor_z.stepper.step > 0) {
						  dir = 1;  // Positive direction
					  }else {
						  dir = 0;  // Negative direction
					  }
					  stepper_move_steps(&motor_z, target_steps, dir);
				  }

				  target_steps = abs(position_data_recieve.rotation_step- motor_r.stepper.step); //make this position specific
				  if(target_steps > 0){
					  if (position_data_recieve.rotation_step - motor_r.stepper.step > 0) {
						  dir = 1;  // Positive direction
					  }else {
						  dir = 0;  // Negative direction
					  }
					  stepper_move_steps(&motor_r, target_steps, dir);
				  }

				  target_steps = abs(position_data_recieve.nail_step- motor_n.stepper.step); //make this position specific
				  if(target_steps > 0){
					  if (position_data_recieve.nail_step - motor_n.stepper.step > 0) {
						  dir = 1;  // Positive direction
					  }else {
						  dir = 0;  // Negative direction
					  }
					  stepper_move_steps(&motor_n, target_steps, dir);
				  }

				  //Vacuume switch
				  if(position_data_recieve.vacuum){
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);
				  }else{
					  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
				  }

				  if(position_data_recieve.nail){
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
				  }else{
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);
					  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
				  }
				  state = TRANSMIT_DATA;
			  }
			  break;
	  }

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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 LD2_Pin PA6
                           PA7 PA8 PA9 PA10
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|LD2_Pin|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

