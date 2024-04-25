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
#include <stdio.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
void _Error_Handler(char* file, int line);
int32_t ReadX();
int32_t ReadY();
void Write(uint32_t addr);
/* USER CODE BEGIN PFP */

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
	//HAL_Init();
 	SystemClock_Config();
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	//LEDs
	GPIOC->MODER |= (1 << 12) | (1 << 18) | (1 << 14) | (1 << 16); //green and red LEDs
  	GPIOC->MODER &= ~(1<< 13);
  	GPIOC->MODER &= ~(1 << 19);
  	GPIOC->MODER &= ~(1 << 15);
	GPIOC->MODER &= ~(1 << 17);

 	GPIOC->OTYPER &= ~(1 << 13);
	GPIOC->OTYPER &= ~(1 << 12);
	GPIOC->OTYPER &= ~(1 << 18);
	GPIOC->OTYPER &= ~(1 << 19);
	GPIOC->OTYPER &= ~(1 << 14);
	GPIOC->OTYPER &= ~(1 << 15);
	GPIOC->OTYPER &= ~(1 << 16);
	GPIOC->OTYPER &= ~(1 << 17);

	GPIOC->OSPEEDR &= ~(1 << 12); 
	GPIOC->OSPEEDR &= ~(1 << 18);
	GPIOC->OSPEEDR &= ~(1 << 14);
	GPIOC->OSPEEDR &= ~(1 << 16);

	GPIOC->PUPDR &= ~(1 << 13);
	GPIOC->PUPDR &= ~(1 << 12);
	GPIOC->PUPDR &= ~(1 << 18);
	GPIOC->PUPDR &= ~(1 << 19);
	GPIOC->PUPDR &= ~(1 << 14);
	GPIOC->PUPDR &= ~(1 << 15);
	GPIOC->PUPDR &= ~(1 << 16);
	GPIOC->PUPDR &= ~(1 << 17);
	
	
	
	//Part 1
	//Setting up PB11 and PB13
	/*GPIOB->MODER &= ~(GPIO_MODER_MODER11_0 | GPIO_MODER_MODER13_0);
  GPIOB->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1);

  GPIOB->OTYPER |= GPIO_OTYPER_OT_11;
  GPIOB->OTYPER |= GPIO_OTYPER_OT_13;
	
	GPIOB->AFR[1] |= 1 << 12;
	GPIOB->AFR[1] |= 5 << 20;
	
	//Setting up PB14
  GPIOB->MODER |= GPIO_MODER_MODER14_0;
  GPIOB->MODER &= ~(GPIO_MODER_MODER14_1);

  GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_14);
	
	GPIOB->ODR |= GPIO_ODR_14;

	//Setting up PC0
	GPIOC->MODER |= GPIO_MODER_MODER0_0;
	GPIOC->MODER &= ~(GPIO_MODER_MODER0_1);

	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_0);

	GPIOC->ODR |= GPIO_ODR_0;
	
	//PB15 on input mode
  GPIOB->MODER &= ~(GPIO_MODER_MODER15_0 | GPIO_MODER_MODER15_1);
	
	//Setting up I2C2
	I2C2->TIMINGR |= (1 << 28) | (0x13 << 20) | (0xF << 16) | (0x2 << 8) | (0x4 << 20);
	
	I2C2->CR1 |= I2C_CR1_PE;
		
	I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
	I2C2->CR2 |= (1 << 16) | (0x69 << 1);

	// RD_WRN to write
	I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
	// Start
	I2C2->CR2 |= I2C_CR2_START;

	
	//Waiting for: TXIS flag.
	while(!((I2C2->ISR >> 1) & 1)){
	}

	//Write the address of the “WHO_AM_I”
	I2C2->TXDR |= 0x0F;

	//Waiting for: TC flag
	while(!((I2C2->ISR >> 6) & 1)){
	}

	//Reload CR2
	I2C2->CR2 |= (1 << 10) | (1 << 13);

	//Waiting for: RXNE
	while(!((I2C2->ISR >> 2) & 1)){
	}

	//Waiting for: TC flag
	while(!((I2C2->ISR >> 6) & 1)){
	}
	
	if(I2C2->RXDR == 0xD3) {
		GPIOC->ODR |= (1 << 9);
	}
	I2C2->CR2 |= I2C_CR2_STOP; */
	
	//Part 2
	// Following transmit I2C protocol beginning flowchart
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (2 << 16) | (0x69 << 1);
    // RD_WRN to write
    I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
    // Start
    I2C2->CR2 |= I2C_CR2_START;

    // wait until TXIS or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_TXIS)) {
		}
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // Error State
    // write CTRL_REG1 into I2C transmit register &
    I2C2->TXDR |= 0x20;

    // wait until TXIS or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_TXIS)) {
		}
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // Error State
    // write CTRL_REG1 to set PD to Xen & Yen
    I2C2->TXDR |= 0xB;
    while (!(I2C2->ISR & I2C_ISR_TC)) {
		}
    // stop
    // I2C2->CR2 |= I2C_CR2_STOP;

    int x_data;
		int y_data;
    while (1) {
        // Reset LEDS
        /* x Axis Read & Write */
        x_data = ReadX();

        /* y Axis Read & Write */
        y_data = ReadY();
        int max = 0;
				
        /* turning LEDs on*/
        if (abs(x_data) > abs(y_data)) {
            if (x_data > max)  // turns on orange LED if X is +
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
            if (x_data < -max)  // turns on green LED if X is -
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
        } else {
            if (y_data > max)  // turns on red LED if Y is +
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
            if (y_data < -max)  // turns on blue LED if Y is -
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
        }
        // delay for reading
        HAL_Delay(100);
    }
}

int32_t ReadX() {
    int16_t x_axis;
    Write(0xA8);
    // Reload the CR2 register
    // setting SADD & NBYTES
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (2 << 16) | (0x69 << 1);
    // reset RD_WRN to read
    I2C2->CR2 |= I2C_CR2_RD_WRN;
    // reset start bit
    I2C2->CR2 |= I2C_CR2_START;

    // wait until RXNE or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_RXNE)) {
		}
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    x_axis = I2C2->RXDR;
    while (!(I2C2->ISR & I2C_ISR_RXNE)) {
		}
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    x_axis |= (I2C2->RXDR << 8);
    while (!(I2C2->ISR & I2C_ISR_TC)) {
		}			/* loop waiting for TC */
    return x_axis;
}

int32_t ReadY() {
    int16_t y_axis;
    Write(0xAA);
    // Reload the CR2 register
    // setting SADD & NBYTES
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (2 << 16) | (0x69 << 1);
    // reset RD_WRN to read
    I2C2->CR2 |= I2C_CR2_RD_WRN;
    // reset start bit
    I2C2->CR2 |= I2C_CR2_START;

    // wait until RXNE or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_RXNE)) {
		}
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    y_axis = I2C2->RXDR;
    while (!(I2C2->ISR & I2C_ISR_RXNE)) {
		}
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    y_axis |= (I2C2->RXDR << 8);
    while (!(I2C2->ISR & I2C_ISR_TC)) {
		}
    return y_axis;
}

void Write(uint32_t addr) {
    // Following transmit I2C protocol beginning flowchart
    I2C2->CR2 &= ~((0x7F << 16) | (0x3FF << 0));
    I2C2->CR2 |= (1 << 16) | (0x69 << 1);
    // RD_WRN to write
    I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
    // Start
    I2C2->CR2 |= I2C_CR2_START;

    // wait until TXIS or NACKF flags are set
    while (!(I2C2->ISR & I2C_ISR_TXIS)) {
		}
    if (I2C2->ISR & I2C_ISR_NACKF)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    // write CTRL_REG1 into I2C transmit register
    I2C2->TXDR |= addr;
    while (!(I2C2->ISR & I2C_ISR_TC)) {
		}
    I2C2->CR2 |= I2C_CR2_STOP;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
