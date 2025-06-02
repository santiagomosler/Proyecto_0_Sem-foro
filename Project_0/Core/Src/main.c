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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h> // variable booleanas
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TESPERA 2000 // periodo de espera en ms entre las transiciones
#define TREBOTES 20 // intervalo de rebotes en ms
#define TSONDEO 200 // retardo de sondeo en ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void CambiarRojoVerde(void); // Transición de rojo a verde
void CambiarVerdeRojo(void); // Transición de verde a rojo
bool VerificarPedido(void); // Determinar si el botón fue pulsado
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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  // Nos aseguramos de apagar las luces amarilla y verde, y de encender la roja
  HAL_GPIO_WritePin(LUZAMARILLA_GPIO_Port, LUZAMARILLA_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LUZVERDE_GPIO_Port, LUZVERDE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LUZROJA_GPIO_Port, LUZROJA_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Esperamos por una solicitud de cambio de estado.
    while(!VerificarPedido()) // Comprobamos si hay una solicitud haciendo una lectura filtrada del botón
    HAL_Delay(TSONDEO); // Si no la hay, esperamos por un tiempo y volvemos a comprobar
    // Al recibir la solicitud, cambiamos de estado de ROJO a VERDE
    CambiarRojoVerde();
    // Esperamos por una solicitud de cambio de estado.
    while(!VerificarPedido()) // Comprobamos si hay una solicitud haciendo una lectura filtrada del botón
    HAL_Delay(TSONDEO); // Si no la hay, esperamos por un tiempo y volvemos a comprobar
    // Al recibir la solicitud, cambiamos de estado de VERDE a ROJO
    CambiarVerdeRojo();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LUZROJA_Pin|LUZVERDE_Pin|LUZAMARILLA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BOTONCAMBIO_Pin */
  GPIO_InitStruct.Pin = BOTONCAMBIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOTONCAMBIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LUZROJA_Pin LUZVERDE_Pin LUZAMARILLA_Pin */
  GPIO_InitStruct.Pin = LUZROJA_Pin|LUZVERDE_Pin|LUZAMARILLA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CambiarRojoVerde(void){ // Transición de rojo a verde
//La luz roja se encuentra encendia. Encendemos la amarilla.
HAL_GPIO_WritePin(LUZAMARILLA_GPIO_Port, LUZAMARILLA_Pin, GPIO_PIN_SET);
// Esperamos un timepo
HAL_Delay(TESPERA); // Retardo de TESPERA milisegundos
// Apagamos las luces roja y amarilla, y encendemos la verde
HAL_GPIO_WritePin(LUZROJA_GPIO_Port, LUZROJA_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(LUZAMARILLA_GPIO_Port, LUZAMARILLA_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(LUZVERDE_GPIO_Port, LUZVERDE_Pin, GPIO_PIN_SET);
}
void CambiarVerdeRojo(void){ // Transición de verde a rojo
// La luz verde se encuentra encendida. La apagamos, y encendemos la amarilla
HAL_GPIO_WritePin(LUZVERDE_GPIO_Port, LUZVERDE_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(LUZAMARILLA_GPIO_Port, LUZAMARILLA_Pin, GPIO_PIN_SET);
// Esperamos un tiempo
HAL_Delay(TESPERA); // Retardo de TESPERA milisegundos
// Apagamos la luz amarilla y encendemos la roja
HAL_GPIO_WritePin(LUZAMARILLA_GPIO_Port, LUZAMARILLA_Pin, GPIO_PIN_RESET);
HAL_GPIO_WritePin(LUZROJA_GPIO_Port, LUZROJA_Pin, GPIO_PIN_SET);
}
// Determinar si el botón fue pulsado
bool VerificarPedido(void){
// Se crea e inicializa un registro de lectura válida del botón
static GPIO_PinState ultima_lectura_valida = GPIO_PIN_SET; //botón liberado
// Se crean variables para lecturas intermedias
GPIO_PinState lectura1, lectura2;
// Se crea una variable booleana para indicar si hay un pedido
bool pedido = false; // No hay pedido hasta que se pulsa el botón
// Se lee el estado del botón
lectura1 = HAL_GPIO_ReadPin(BOTONCAMBIO_GPIO_Port, BOTONCAMBIO_Pin);
// Si hubo un cambio
if (lectura1 != ultima_lectura_valida){
// Se espera un tiempo para filtrar los rebotes
HAL_Delay(TREBOTES); // Retardo de TREBOTES milisegundos
// Se lee nuevamente el estado del botón
lectura2 = HAL_GPIO_ReadPin(BOTONCAMBIO_GPIO_Port, BOTONCAMBIO_Pin);
// Si ambas lecturas son iguales, se considera una lectura válida
if(lectura2 == lectura1)
ultima_lectura_valida = lectura2;
// Si el botón pasó de liberado a pulsado (1-->0), hubo un pedido de cambio de estado
if (ultima_lectura_valida == GPIO_PIN_RESET)
pedido = true;
}
return pedido;
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
