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
#include "ili9341.h"

#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANCHO_LCD 320
#define ALTO_LCD 240

#define ANCHO_CARRO 42
#define ALTO_CARRO 78

#define ANCHO_PARQUEO 52
#define ALTO_PARQUEO 80
#define ANCHO_BORDE 4

#define DIM_DISPLAY 50

#define POSX_DIS 240
#define POSY_DIS 120

#define DIM_LUZ 20

#define FILA1 40
#define FILA2 ALTO_LCD - ALTO_PARQUEO
#define FILA1_LUZ 10
#define FILA2_LUZ ALTO_LCD - ALTO_PARQUEO - DIM_LUZ - 10

#define COLUMNA1 5
#define COLUMNA2 5 + 1*(ANCHO_CARRO + 6)
#define COLUMNA3 5 + 2*(ANCHO_CARRO + 6)
#define COLUMNA4 5 + 3*(ANCHO_CARRO + 6)

#define COLUMNA1_LUZ COLUMNA1 + 11
#define COLUMNA2_LUZ COLUMNA2 + 11
#define COLUMNA3_LUZ COLUMNA3 + 11
#define COLUMNA4_LUZ COLUMNA4 + 11
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern uint8_t carro[], parqueo[];
extern uint8_t luz_roja[], luz_verde[];
extern uint8_t dis_0[], dis_1[], dis_2[], dis_3[], dis_4[], dis_5[], dis_6[], dis_7[], dis_8[];

int parqueo_dis = 8;
// Variables para almacenar los bits recibidos
int p5 = 0, p6 = 0, p7 = 0, p8 = 0;

#define TXBUFFERSIZE 4
#define RXBUFFERSIZE 4

uint8_t aTxBuffer[TXBUFFERSIZE];
uint8_t aRxBuffer[RXBUFFERSIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void transmit_uart(char* string);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void transmit_uart(char* string) {
    uint32_t len = strlen(string);
    HAL_UART_Transmit(&huart2, (uint8_t*) string, len, HAL_MAX_DELAY);
    HAL_Delay(10); // Pequeño retraso para que la terminal procese la línea
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_I2C_EnableListen_IT(&hi2c1) != HAL_OK){
	  Error_Handler();
  }

	 LCD_Init();

	 FillRect(0, 0, ANCHO_LCD, ALTO_LCD, 0xdedb);

	 int x = 0;
	 	 for (int i = 0; i < 4; i++) {
	 		 LCD_Bitmap(x, FILA1, ANCHO_PARQUEO, ALTO_PARQUEO, parqueo);
	 		 LCD_Bitmap(x, FILA2, ANCHO_PARQUEO, ALTO_PARQUEO, parqueo);
	 		 x += ANCHO_PARQUEO - ANCHO_BORDE;
	 	 }
	 x = 5 + 11;
	 for (int i = 0; i < 4; i++) {
			 LCD_Bitmap(x, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
			 LCD_Bitmap(x, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
			 x += ANCHO_PARQUEO - ANCHO_BORDE;
	 }

	 LCD_Print("PARQUEOS", 200, 70, 1, 0x0, 0xdedb);
	 LCD_Print("DISPONIBLES", 200, 90, 1, 0x0, 0xdedb);
	 LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_8);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//UpdateParkingBits(aRxBuffer[0]);

	//PARQUEO 1
	if(HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0){
		LCD_Bitmap(COLUMNA1, FILA1, ANCHO_CARRO, ALTO_CARRO, carro);
		LCD_Bitmap(COLUMNA1_LUZ, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_roja);
		HAL_GPIO_WritePin(RP1_GPIO_Port, RP1_Pin, 1);
		HAL_GPIO_WritePin(VP1_GPIO_Port, VP1_Pin, 0);
		parqueo_dis--;
	 }else{
		LCD_Bitmap(COLUMNA1_LUZ, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
		FillRect(COLUMNA1, FILA1, ANCHO_CARRO, ALTO_CARRO, 0xdedb);
		HAL_GPIO_WritePin(VP1_GPIO_Port, VP1_Pin, 1);
		HAL_GPIO_WritePin(RP1_GPIO_Port, RP1_Pin, 0);
	 }
	//PARQUEO 2
	 if(HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0){
		LCD_Bitmap(COLUMNA2, FILA1, ANCHO_CARRO, ALTO_CARRO, carro);
		LCD_Bitmap(COLUMNA2_LUZ, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_roja);
		HAL_GPIO_WritePin(RP2_GPIO_Port, RP2_Pin, 1);
		HAL_GPIO_WritePin(VP2_GPIO_Port, VP2_Pin, 0);
		parqueo_dis--;
	 }else{
		LCD_Bitmap(COLUMNA2_LUZ, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
		FillRect(COLUMNA2, FILA1, ANCHO_CARRO, ALTO_CARRO, 0xdedb);
		HAL_GPIO_WritePin(VP2_GPIO_Port, VP2_Pin, 1);
		HAL_GPIO_WritePin(RP2_GPIO_Port, RP2_Pin, 0);
	 }
	//PARQUEO 3
	 if(HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0){
		LCD_Bitmap(COLUMNA3, FILA1, ANCHO_CARRO, ALTO_CARRO, carro);
		LCD_Bitmap(COLUMNA3_LUZ, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_roja);
		HAL_GPIO_WritePin(RP3_GPIO_Port, RP3_Pin, 1);
		HAL_GPIO_WritePin(VP3_GPIO_Port, VP3_Pin, 0);
		parqueo_dis--;
	 }else{
		LCD_Bitmap(COLUMNA3_LUZ, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
		FillRect(COLUMNA3, FILA1, ANCHO_CARRO, ALTO_CARRO, 0xdedb);
		HAL_GPIO_WritePin(VP3_GPIO_Port, VP3_Pin, 1);
		HAL_GPIO_WritePin(RP3_GPIO_Port, RP3_Pin, 0);
	 }
	//PARQUEO 4
	 if(HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0){
		LCD_Bitmap(COLUMNA4, FILA1, ANCHO_CARRO, ALTO_CARRO, carro);
		LCD_Bitmap(COLUMNA4_LUZ, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_roja);
		HAL_GPIO_WritePin(RP4_GPIO_Port, RP4_Pin, 1);
		HAL_GPIO_WritePin(VP4_GPIO_Port, VP4_Pin, 0);
		parqueo_dis--;
	 }else{
		LCD_Bitmap(COLUMNA4_LUZ, FILA1_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
		FillRect(COLUMNA4, FILA1, ANCHO_CARRO, ALTO_CARRO, 0xdedb);
		HAL_GPIO_WritePin(VP4_GPIO_Port, VP4_Pin, 1);
		HAL_GPIO_WritePin(RP4_GPIO_Port, RP4_Pin, 0);
	 }

	 if(p5 == 1){
		LCD_Bitmap(COLUMNA1, FILA2, ANCHO_CARRO, ALTO_CARRO, carro);
		LCD_Bitmap(COLUMNA1_LUZ, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_roja);
		parqueo_dis--;
	 }else{
		LCD_Bitmap(COLUMNA1_LUZ, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
		FillRect(COLUMNA1, FILA2, ANCHO_CARRO, ALTO_CARRO, 0xdedb);
	 }

	 if(p6 == 1){
		LCD_Bitmap(COLUMNA2, FILA2, ANCHO_CARRO, ALTO_CARRO, carro);
		LCD_Bitmap(COLUMNA2_LUZ, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_roja);
		parqueo_dis--;
	 }else{
		LCD_Bitmap(COLUMNA2_LUZ, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
		FillRect(COLUMNA2, FILA2, ANCHO_CARRO, ALTO_CARRO, 0xdedb);
	 }

	 if(p7 == 1){
		LCD_Bitmap(COLUMNA3, FILA2, ANCHO_CARRO, ALTO_CARRO, carro);
		LCD_Bitmap(COLUMNA3_LUZ, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_roja);
		parqueo_dis--;
	 }else{
		LCD_Bitmap(COLUMNA3_LUZ, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
		FillRect(COLUMNA3, FILA2, ANCHO_CARRO, ALTO_CARRO, 0xdedb);
	 }

	 if(p8 == 1){
		LCD_Bitmap(COLUMNA4, FILA2, ANCHO_CARRO, ALTO_CARRO, carro);
		LCD_Bitmap(COLUMNA4_LUZ, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_roja);
		parqueo_dis--;
	 }else{
		LCD_Bitmap(COLUMNA4_LUZ, FILA2_LUZ, DIM_LUZ, DIM_LUZ, luz_verde);
		FillRect(COLUMNA4, FILA2, ANCHO_CARRO, ALTO_CARRO, 0xdedb);
	 }

	 switch(parqueo_dis) {
	         case 0: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_0); parqueo_dis = 8; break;
	         case 1: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_1); parqueo_dis = 8; break;
	         case 2: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_2); parqueo_dis = 8; break;
	         case 3: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_3); parqueo_dis = 8; break;
	         case 4: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_4); parqueo_dis = 8; break;
	         case 5: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_5); parqueo_dis = 8; break;
	         case 6: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_6); parqueo_dis = 8; break;
	         case 7: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_7); parqueo_dis = 8; break;
	         case 8: LCD_Bitmap(POSX_DIS, POSY_DIS, DIM_DISPLAY, DIM_DISPLAY, dis_8); parqueo_dis = 8; break;
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hi2c1.Init.OwnAddress1 = 170;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|VP1_Pin|RP1_Pin|LCD_D1_Pin
                          |VP3_Pin|RP3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin|VP4_Pin|RP4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|VP2_Pin|RP2_Pin|LCD_D6_Pin
                          |LCD_D3_Pin|LCD_D5_Pin|LCD_D4_Pin|SD_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VP1_Pin RP1_Pin VP3_Pin RP3_Pin */
  GPIO_InitStruct.Pin = VP1_Pin|RP1_Pin|VP3_Pin|RP3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin SD_SS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin|SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : VP2_Pin RP2_Pin */
  GPIO_InitStruct.Pin = VP2_Pin|RP2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : S1_Pin S2_Pin S3_Pin S4_Pin */
  GPIO_InitStruct.Pin = S1_Pin|S2_Pin|S3_Pin|S4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : VP4_Pin RP4_Pin */
  GPIO_InitStruct.Pin = VP4_Pin|RP4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c);
}

// Mandar
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {

	if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1 &&
	    HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 1 &&
	    HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 1 &&
	    HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 1) {
	    aTxBuffer[0] = 0;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 1) {
	    aTxBuffer[0] = 1;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 1) {
	    aTxBuffer[0] = 2;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 1) {
	    aTxBuffer[0] = 3;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 1) {
	    aTxBuffer[0] = 4;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 1) {
	    aTxBuffer[0] = 5;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 1) {
	    aTxBuffer[0] = 6;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 1) {
	    aTxBuffer[0] = 7;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) {
	    aTxBuffer[0] = 8;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) {
	    aTxBuffer[0] = 9;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) {
	    aTxBuffer[0] = 10;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) {
	    aTxBuffer[0] = 11;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) {
	    aTxBuffer[0] = 12;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) {
	    aTxBuffer[0] = 13;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 1 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) {
	    aTxBuffer[0] = 14;
	} else if (HAL_GPIO_ReadPin(S1_GPIO_Port, S1_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S2_GPIO_Port, S2_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S3_GPIO_Port, S3_Pin) == 0 &&
	           HAL_GPIO_ReadPin(S4_GPIO_Port, S4_Pin) == 0) {
	    aTxBuffer[0] = 15;
	}

	    // Volver a habilitar la recepción/transmisión I2C
	    HAL_I2C_Slave_Transmit_IT(I2cHandle, aTxBuffer, 1);
}

// Recibir
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	if (aRxBuffer[0] == 0) {        // 0000
	        p5 = 0; p6 = 0; p7 = 0; p8 = 0;
	    } else if (aRxBuffer[0] == 1) { // 1000
	        p5 = 1; p6 = 0; p7 = 0; p8 = 0;
	    } else if (aRxBuffer[0] == 2) { // 0100
	        p5 = 0; p6 = 1; p7 = 0; p8 = 0;
	    } else if (aRxBuffer[0] == 3) { // 0010
	        p5 = 0; p6 = 0; p7 = 1; p8 = 0;
	    } else if (aRxBuffer[0] == 4) { // 0001
	        p5 = 0; p6 = 0; p7 = 0; p8 = 1;
	    } else if (aRxBuffer[0] == 5) { // 1100
	        p5 = 1; p6 = 1; p7 = 0; p8 = 0;
	    } else if (aRxBuffer[0] == 6) { // 0110
	        p5 = 0; p6 = 1; p7 = 1; p8 = 0;
	    } else if (aRxBuffer[0] == 7) { // 0011
	        p5 = 0; p6 = 0; p7 = 1; p8 = 1;
	    } else if (aRxBuffer[0] == 8) { // 1001
	        p5 = 1; p6 = 0; p7 = 0; p8 = 1;
	    } else if (aRxBuffer[0] == 9) { // 1010
	        p5 = 1; p6 = 0; p7 = 1; p8 = 0;
	    } else if (aRxBuffer[0] == 10) { // 0101
	        p5 = 0; p6 = 1; p7 = 0; p8 = 1;
	    } else if (aRxBuffer[0] == 11) { // 1110
	        p5 = 1; p6 = 1; p7 = 1; p8 = 0;
	    } else if (aRxBuffer[0] == 12) { // 0111
	        p5 = 0; p6 = 1; p7 = 1; p8 = 1;
	    } else if (aRxBuffer[0] == 13) { // 1011
	        p5 = 1; p6 = 0; p7 = 1; p8 = 1;
	    } else if (aRxBuffer[0] == 14) { // 1101
	        p5 = 1; p6 = 1; p7 = 0; p8 = 1;
	    } else if (aRxBuffer[0] == 15) { // 1111
	        p5 = 1; p6 = 1; p7 = 1; p8 = 1;
	    } else {                   // Para valores fuera del rango
	        p5 = 0; p6 = 0; p7 = 0; p8 = 0;
	    }

	// Volver a habilitar la recepción/transmisión I2C
	HAL_I2C_Slave_Receive_IT(I2cHandle, aRxBuffer, 1);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode) {
    if (TransferDirection == I2C_DIRECTION_TRANSMIT){
        /*##- Put I2C peripheral in reception process ###########################*/
        if (HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, (uint8_t*) aRxBuffer, 1, I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
            /* Transfer error in reception process */
            Error_Handler();
        }
    } else if (TransferDirection == I2C_DIRECTION_RECEIVE) {
        /*##- Start the transmission process ####################################*/
        /* While the I2C in reception process, user can transmit data through
           "aTxBuffer" buffer */
        if (HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, (uint8_t*) aTxBuffer, 1, I2C_FIRST_AND_LAST_FRAME) != HAL_OK) {
            /* Transfer error in transmission process */
            Error_Handler();
        }
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2CHandle) {
    /** Error_Handler() function is called when error occurs.
     * 1- When Slave doesn't acknowledge its address, Master restarts communication.
     * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't
     */
    if (HAL_I2C_GetError(I2CHandle) != HAL_I2C_ERROR_AF) {
        Error_Handler();
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
	while (1) {
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
