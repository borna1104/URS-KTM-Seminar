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
#include "spi.h"
#include "tim.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "globals.h"
#include "string.h"
#include "rc522.h"
#include "stdio.h"
#include "stm32f4xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RC522_NSS_PIN       GPIO_PIN_4
#define RC522_NSS_PORT      GPIOA
#define RC522_RST_PIN       GPIO_PIN_3
#define RC522_RST_PORT      GPIOA
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t status;
uint8_t str[MAX_LEN]; // Max_LEN = 16
uint8_t sNum[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void RC522_WriteRegister(uint8_t reg, uint8_t data);
uint8_t RC522_ReadRegister(uint8_t reg);
void RC522_Init(void);
void SystemClock_Config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void I2C1_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;  // Enable I2C1 clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable GPIOB clock

    // Configure PB8 and PB9 as Alternate Function Open-Drain
    GPIOB->MODER |= (2 << (8 * 2)) | (2 << (9 * 2)); // AF Mode
    GPIOB->OTYPER |= (1 << 8) | (1 << 9); // Open-drain
    GPIOB->AFR[1] |= (4 << (0)) | (4 << (4)); // AF4 for I2C1

    // Reset I2C1
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // Configure I2C speed for 45MHz APB1 (Standard mode: 100kHz)
    I2C1->CR2 = 45; // APB1 Clock (MHz)
    I2C1->CCR = 225; // Standard mode: (45MHz / (2 * 100kHz))
    I2C1->TRISE = 46; // (45MHz / 1MHz) + 1

    // Enable I2C
    I2C1->CR1 |= I2C_CR1_PE;
}
void I2C1_Write(uint8_t addr, uint8_t data) {
    while (I2C1->SR2 & I2C_SR2_BUSY); // Wait until I2C is not busy
    I2C1->CR1 |= I2C_CR1_START; // Generate START
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for start bit

    I2C1->DR = (addr << 1); // Send Address + Write bit
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait for Address ACK
    (void)I2C1->SR2; // Clear ADDR flag

    I2C1->DR = data; // Send data
    while (!(I2C1->SR1 & I2C_SR1_BTF)); // Wait for transmission

    I2C1->CR1 |= I2C_CR1_STOP; // Generate STOP condition
}
void LCD_SendNibble(uint8_t data, uint8_t rs) {
    uint8_t byteToSend = (data & 0xF0) | (rs ? 0x01 : 0x00) | 0x08; // Keep backlight ON
    I2C1_Write(0x27, byteToSend | 0x04);  // E = 1 (Enable high)
    for (volatile int i = 0; i < 500; i++); // Short delay
    I2C1_Write(0x27, byteToSend & ~0x04);  // E = 0 (Enable low)
}
void LCD_SendByte(uint8_t data, uint8_t rs) {
    LCD_SendNibble(data & 0xF0, rs); // High nibble
    LCD_SendNibble((data << 4) & 0xF0, rs); // Low nibble
}
void LCD_Init(void) {
    // Wait for LCD to power up
    for (volatile int i = 0; i < 50000; i++);

    LCD_SendNibble(0x30, 0);  // Wake up
    for (volatile int i = 0; i < 5000; i++);

    LCD_SendNibble(0x30, 0);
    for (volatile int i = 0; i < 5000; i++);

    LCD_SendNibble(0x30, 0);
    for (volatile int i = 0; i < 5000; i++);

    LCD_SendNibble(0x20, 0);  // Set 4-bit mode
    for (volatile int i = 0; i < 5000; i++);

    LCD_SendByte(0x28, 0);  // Function set: 4-bit, 2-line, 5x8 font
    LCD_SendByte(0x0C, 0);  // Display ON, Cursor OFF
    LCD_SendByte(0x06, 0);  // Entry mode (move cursor right)
    LCD_SendByte(0x01, 0);  // Clear display
    for (volatile int i = 0; i < 50000; i++);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
	void LCD_Print(char *str) {
	    while (*str) {
	        LCD_SendByte(*str++, 1);
	    }
	}

	    I2C1_Init();
	    LCD_Init();

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
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);




  /* USER CODE END 2 */
  /* Infinite loop */
  while (1)
  {

	int x;
	  status = MFRC522_Request(PICC_REQIDL, str);
		if (status == MI_OK) {
			// Card detected, now perform anti-collision to get UID
			status = MFRC522_Anticoll(str);
			if (status == MI_OK) {
				memcpy(sNum, str, 5);
				HAL_Delay(100);

				// Wrong Card
				if (str[0] == 198 && str[1] == 123 && str[2] == 193
						&& str[3] == 1 && str[4] == 125 && fault == 1) {
					  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
					    LCD_Init();
					    LCD_Print("Wrong Card");
		 			    for(x=0; x<10; x=x+1)
		 			    {
		 			    	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
		 			      __HAL_TIM_SET_AUTORELOAD(&htim3, 1000);
		 			      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);
		 			      HAL_Delay(100);
		 			    }
		 			   HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
					for (int i = 0; i < sizeof(str); i++) {
						str[i] = '\0';
					}
				}




				// Clear Fault
				if (str[0] == 122 && str[1] == 158 && str[2] == 4
						&& str[3] == 4 && str[4] == 228) {
					if (fault == 1) {
					LCD_Init();
					LCD_Print("Fault Cleared");
					fault = 0;
					HAL_Delay(500);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
					for (int i = 0; i < sizeof(str); i++) {
						str[i] = '\0';
					}}





				}
				// Start
				if ((str[0] == 122 && str[1] == 158 && str[2] == 4
						&& str[3] == 4 && str[4] == 228)) {
						if (fault == 0){
							I2C1_Init();
							LCD_Init();
							LCD_Print("Admin");
							HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
					while (1){
				      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 250);
				      HAL_Delay(350);
				      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 750);
				      HAL_Delay(175);
				      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 1250);
				      HAL_Delay(175);
				      if (fault == 1) {
						LCD_Init();
						LCD_Print("Fault");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
						for (int i = 0; i < sizeof(str); i++) {
							str[i] = '\0';
						}
						break;
				      }
					}}}

				if (str[0] == 198 && str[1] == 123 && str[2] == 193
						&& str[3] == 1 && str[4] == 125) {
						if (fault == 0){
							I2C1_Init();
							LCD_Init();
							LCD_Print("Standard");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
					while (1){
				      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 250);
				      HAL_Delay(350);
				      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 750);
				      HAL_Delay(175);
				      __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2, 1250);
				      HAL_Delay(175);
				      if (fault == 1) {
						LCD_Init();
						LCD_Print("Fault");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
						for (int i = 0; i < sizeof(str); i++) {
							str[i] = '\0';
						}
						break;
				      }
					}
				}}




					//Clear string buffer
					for (int i = 0; i < sizeof(str); i++) {
						str[i] = '\0';
					}
				}









			}
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/* USER CODE BEGIN 4 */
void RC522_Init(void) {
	// Set RST pin LOW and HIGH for reset
	HAL_GPIO_WritePin(RC522_RST_PORT, RC522_RST_PIN, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(RC522_RST_PORT, RC522_RST_PIN, GPIO_PIN_SET);
	HAL_Delay(50);
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
