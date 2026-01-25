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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    uint8_t btn_flag;   // 1 byte
    float   lx;         // 4 bytes
    float   ly;         // 4 bytes
    float   rx;         // 4 bytes
    float   ry;         // 4 bytes
} Packet;

_Static_assert(sizeof(Packet) == 17, "Packet size mismatch");


typedef struct {
    uint8_t up        :1;
    uint8_t down      :1;
    uint8_t left      :1;
    uint8_t right     :1;
    uint8_t triangle  :1;
    uint8_t cross     :1;
    uint8_t square    :1;
    uint8_t circle    :1;
}BitfieldButtonStatusUsr;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX(a,b) ((a) > (b) ? (a) : (b));


#define STX								0xAA


#define INITIAL_ANGLE					0
#define STEP_ANGLE						1
#define MIN_ANGLE						INITIAL_ANGLE
#define MAX_ANGLE						180
#define SERVO_DELAY						20
#define POS_UP							1
#define POS_DOWN						2



#define m1_dir_pin						GPIO_PIN_12
#define m2_dir_pin						GPIO_PIN_13
#define m3_dir_pin						GPIO_PIN_14
#define m4_dir_pin						GPIO_PIN_15

#define m1_pwm_pin						TIM_CHANNEL_1	//PC6
#define m2_pwm_pin						TIM_CHANNEL_2	//PB5
#define m3_pwm_pin						TIM_CHANNEL_3	//PB0
#define m4_pwm_pin						TIM_CHANNEL_4	//PB1

#define PNEUMATIC_PORT					GPIOD
#define PNEUMATIC_PIN_1					GPIO_PIN_0
#define PNEUMATIC_PIN_2					GPIO_PIN_1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned long current = 0, previous = 0;


uint8_t rx_val;
Packet rx_pkt;
uint8_t ch, len;

float LX_usr;
float LY_usr;
float RX_usr;
float RY_usr;

BitfieldButtonStatusUsr btnStatus;

uint8_t curr_angle = 0;


//const uint8_t m1_dir_pin;
//const uint8_t m2_dir_pin;
//const uint8_t m3_dir_pin;
//const uint8_t m4_dir_pin;


float m1_pwm, m2_pwm, m3_pwm, m4_pwm;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
uint32_t millis(void);

void recieve_uart();

void motor_set_speed(TIM_HandleTypeDef *htim, uint32_t channel, float speed);
void motor_set_speed255(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t val);

void Servo_WriteAngle(uint8_t angle);
void servo_handler(uint8_t pos);

int lo_4_wheel_handler();
int lo_4_wheel_calculation(int vx, int vy, int omega);
void lo_4_wheel_run(uint16_t dir_pin, uint8_t mot_pin, float pwm);

void pnuematic_actuation();

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




  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  printf("STM32 Ready\n");
  Servo_WriteAngle(INITIAL_ANGLE);
  while (1)
  {
	  recieve_uart();
//	  printf("LX = %.2f | LY = %.2f", LX_usr, LY_usr);
	  lo_4_wheel_handler();
//	  rx_pkt.lx = 0;
//	  rx_pkt.ly = 0;
//	  rx_pkt.ry = 0;
//	  rx_pkt.ry = 0;
//
//	  LX_usr = 0;
//	  LY_usr = 0;
//	  RX_usr = 0;
//	  RY_usr = 0;

	  pnuematic_actuation();
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_10|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD6_Pin|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_3|Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 LD4_Pin LD3_Pin
                           LD5_Pin LD6_Pin PD0 PD1
                           PD3 Audio_RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|LD4_Pin|LD3_Pin
                          |LD5_Pin|LD6_Pin|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_3|Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void recieve_uart(){
	while(1){
		 do {
			  HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY);
		 }while (ch != STX);

		// Read length
		HAL_UART_Receive(&huart2, &len, 1, HAL_MAX_DELAY);
		if (len != sizeof(Packet)) {
			 continue;
	}

		// Read payload directly into struct
		HAL_UART_Receive(&huart2, (uint8_t*)&rx_pkt, len, HAL_MAX_DELAY);
		break;
	}
	// Use values directly
	if (rx_pkt.btn_flag & (1 << 7)) {
		printf("Circle pressed\n");
		btnStatus.circle = 1;
	}
	if (rx_pkt.btn_flag & (1 << 6)) {
		printf("Square pressed\n");
		btnStatus.square = 1;
	}
	if (rx_pkt.btn_flag & (1 << 5)) {
		printf("Cross pressed\n");
		btnStatus.cross = 1;
	}
	if (rx_pkt.btn_flag & (1 << 4)) {
		printf("Triangle pressed\n");
		btnStatus.triangle = 1;
	}
	if (rx_pkt.btn_flag & (1 << 3)) {
		printf("Right pressed\n");
		btnStatus.right = 1;
	}
	if (rx_pkt.btn_flag & (1 << 2)) {
		printf("Left pressed\n");
		btnStatus.left = 1;
	}
	if (rx_pkt.btn_flag & (1 << 1)) {
		printf("Down pressed\n");
		btnStatus.down = 1;
	}
	if (rx_pkt.btn_flag & (1 << 0)) {
		printf("Up pressed\n");
		btnStatus.up = 1;
	}

	LX_usr = rx_pkt.lx;
	LY_usr = rx_pkt.ly;
	RX_usr = rx_pkt.rx;
	RY_usr = rx_pkt.ry;

//	printf("FLAG = %02X | LX = %.2f | LY = %.2f | RX = %.2f | RY = %.2f\n", rx_pkt.btn_flag,  rx_pkt.lx, rx_pkt.ly, rx_pkt.rx, rx_pkt.ry);
}




void Servo_WriteAngle(uint8_t angle){
	//Clamp value 0-180
	if(angle > 180)angle=180;

	//Map 0-180 -> 1000 - 2000us
	uint16_t pulse = 1000 + (angle * 1000)/180;

	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}




void servo_handler(uint8_t pos){
	if(curr_angle >= MAX_ANGLE){
	  curr_angle = MAX_ANGLE;
	  printf("Current Angle: %d\n", curr_angle);
	}else if(curr_angle <= MIN_ANGLE){
	  curr_angle = 0;
	  printf("Current Angle: %d\n", curr_angle);
	}

	if(pos == POS_UP){
		HAL_Delay(SERVO_DELAY);
		Servo_WriteAngle((curr_angle+=STEP_ANGLE));
	}else if(pos == POS_DOWN){
		HAL_Delay(SERVO_DELAY);
		Servo_WriteAngle((curr_angle-=STEP_ANGLE));
	}
}





int lo_4_wheel_handler(){
	int x = LY_usr;
	int y = LX_usr;
	int w = RX_usr;

    if(abs(x) < 20) x = 0;
    if(abs(y) < 20) y = 0;
    if(abs(w) < 20) w = 0;

    int vx = (x * 255) / 127;
    int vy = (y * 255) / 127;
    int omega = (w * 255) / 127;

    lo_4_wheel_calculation(vx, vy, omega);

    lo_4_wheel_run(m1_dir_pin, m1_pwm_pin, m1_pwm);
    lo_4_wheel_run(m2_dir_pin, m2_pwm_pin, m2_pwm);
    lo_4_wheel_run(m3_dir_pin, m3_pwm_pin, m3_pwm);
    lo_4_wheel_run(m4_dir_pin, m4_pwm_pin, m4_pwm);
    return 0;
}


int lo_4_wheel_calculation(int vx, int vy, int omega){
	m1_pwm = (vx+vy+omega);
	m2_pwm = (vx-vy-omega);
	m3_pwm = (-vx-vy+omega);
	m4_pwm = (-vx+vy-omega);

	// ---------- NORMALIZATION (CRTT) ----------
	float maxraw_1 = MAX(fabs(m1_pwm), fabs(m2_pwm));
	float maxraw_2 = MAX(fabs(m3_pwm), fabs(m4_pwm));
	float maxraw = MAX(maxraw_1, maxraw_2);

	if(maxraw > 255.0){
	float scale = 255.0 / maxraw;
	m1_pwm = (m1_pwm * scale);
	m2_pwm = (m2_pwm * scale);
	m3_pwm = (m3_pwm * scale);
	m4_pwm = (m4_pwm * scale);
	}
	// -----------------------------------------


	current = millis();
	if(current-previous >= 1000)
	{
//		printf("m1 = %f", m1_pwm);
//		printf("m2 = %f", m2_pwm);
//		printf("m3 = %f", m3_pwm);
//		printf("m4 = %f", m4_pwm);
		printf("m1 = %.2f  |  m2 = %.2f  |  m3 = %.2f |  m4 = %.2f\n", m1_pwm, m2_pwm, m3_pwm, m4_pwm);
		previous = millis();
	}

//	printf("m1 = %f  |  m2 = %f  |  m3 = %f |  m4 = %f", m1_pwm, m2_pwm, m3_pwm, m4_pwm);
	return 0;
}

void lo_4_wheel_run(uint16_t dir_pin, uint8_t mot_pin, float pwm){
	if(pwm > 0){
		HAL_GPIO_WritePin(GPIOD, dir_pin, SET);
	}else{
		HAL_GPIO_WritePin(GPIOD, dir_pin, RESET);
		pwm = abs(pwm);
	}
//	printf("pwm = %.2f\n", pwm);
	motor_set_speed255(&htim3, mot_pin, pwm);
}



//Speed value 0.0 <--> 1.0
void motor_set_speed(TIM_HandleTypeDef *htim, uint32_t channel, float speed)
{
    // speed: 0.0 → 1.0
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (uint32_t)((arr + 1) * speed);

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}


//This function allows to do analogwrite() like behavior val = (0 - 255)
void motor_set_speed255(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t val)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    uint32_t ccr = (val * (arr + 1)) / 255;

    __HAL_TIM_SET_COMPARE(htim, channel, ccr);
}





void pnuematic_actuation()
{
	if(btnStatus.circle == 1){
		HAL_GPIO_WritePin(PNEUMATIC_PORT, PNEUMATIC_PIN_1, SET);
		HAL_GPIO_WritePin(PNEUMATIC_PORT, PNEUMATIC_PIN_2, RESET);
	}else if(btnStatus.square == 1){
		HAL_GPIO_WritePin(PNEUMATIC_PORT, PNEUMATIC_PIN_1, RESET);
		HAL_GPIO_WritePin(PNEUMATIC_PORT, PNEUMATIC_PIN_2, SET);
	}
}


uint32_t millis(void){
	return HAL_GetTick();
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
