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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TENCENDIDO 2000 // Tiempo en ms que permanecen encendidas las luces al iniciar el sistema
#define TAPAGADO 1000 // Tiempo en ms que permanecen apagadas las luces antes iniciar las transiciones
#define TCAMBIO 20 // Tiempo en ms entre cambios de los ciclos de trabajo de las señales PWM
#define VALCAMBIO 640 // Valor (en LSB de resolución de PWM) para los cambios de los ciclos de trabajo
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void inicio(void); // Prueba inicial del sistema
void transicion1(void); // Disminuye rojo, aumenta verde
void transicion2(void); // Disminuye verde, aumenta rojo
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Nos aseguramos de apagar las luces amarilla y verde, y de encender la roja
  inicio(); // Prueba inicial del sistema
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  transicion1(); // Progresiva y gradualmente disminuye rojo, aumenta verde
	  transicion2(); // Progresiva y gradualmente disminuye verde, aumenta rojo
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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 63999;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Prueba inicial del sistema
void inicio(void){
// Obtenemos el valor máximo de ciclo de trabajo de la configuración del hgardware
uint16_t ciclomaximo = TIM4->ARR; // Valor máximo de ciclo de trabajo
// Configuramos el PWM para que inicie con los LED encendidos
TIM4->CCR1 = ciclomaximo; // LED verde al 100 %
TIM4->CCR2 = ciclomaximo; // LED rojo al 100 %
// Arrancamos el PWM
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Inicio de la modulación PWM, LED verde
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Inicio de la modulación PWM, LED rojo
// Mantenemos los LED encendidos por un tiempo
HAL_Delay(TENCENDIDO); // Retardo de TENCENDIDO milisegundos
// Apagamos los LED
TIM4->CCR1 = 0; // LED verde al 0 %
TIM4->CCR2 = 0; // LED rojo al 0 %
// Mantenemos los LED apagados por un tiempo
HAL_Delay(TAPAGADO); // Retardo de TAPAGADO milisegundos
//Encendemos el LED rojo
TIM4->CCR2 = ciclomaximo; // LED rojo al 100 %
}

// Disminuye rojo, aumenta verde
void transicion1(void){
// Obtenemos valores de la configuración del hgardware
uint16_t ciclomaximo = TIM4->ARR; // Valor máximo de ciclo de trabajo
uint16_t cicloverde = TIM4->CCR1; // Valor actual de ciclo de trabajo en LED verde
uint16_t ciclorojo = TIM4->CCR2; // Valor actual de ciclo de trabajo en LED rojo
do{
// Esperamos un tiempo antes de cambiar las intencidades de lso LED
HAL_Delay(TCAMBIO); // Espera por TCAMBIO milisegndos
// Disminuimos un poco la intensidad del LED rojo
ciclorojo = (ciclorojo > VALCAMBIO) ? ciclorojo - VALCAMBIO : 0; // Decremento con saturación en 0 %
TIM4->CCR2 = ciclorojo; // Actualiza el ciclo de trabajo en LED rojo
// Aumentaqmos un poco la intensidad del LED verde
cicloverde = (ciclomaximo - cicloverde > VALCAMBIO) ? cicloverde + VALCAMBIO :
ciclomaximo; // Incremento con saturación en 100 %
TIM4->CCR1 = cicloverde; // Actualiza el ciclo de trabajo en LED verde
// Verficamos si repetimos o invertimos
}while( ciclorojo > 0 && cicloverde < ciclomaximo ); // Repetimos mientras no se alcance un límite
}

// Disminuye verde, aumenta rojo
void transicion2(void){
// Obtenemos valores de la configuración del hgardware
uint16_t ciclomaximo = TIM4->ARR; // Valor máximo de ciclo de trabajo
uint16_t cicloverde = TIM4->CCR1; // Valor actual de ciclo de trabajo en LED verde
uint16_t ciclorojo = TIM4->CCR2; // Valor actual de ciclo de trabajo en LED rojo
do{
// Esperamos un tiempo antes de cambiar las intencidades de lso LED
HAL_Delay(TCAMBIO); // Espera por TCAMBIO milisegundos
// Disminuimos un poco la intensidad del LED verde
cicloverde = (cicloverde > VALCAMBIO) ? cicloverde - VALCAMBIO : 0; // Decremento con saturación en 0 %
TIM4->CCR1 = cicloverde; // Actualiza el ciclo de trabajo en LED rojo
// Aumentaqmos un poco la intensidad del LED verde
ciclorojo = (ciclomaximo - ciclorojo > VALCAMBIO) ? ciclorojo + VALCAMBIO :
ciclomaximo; // Incremento con saturación en 100 %
TIM4->CCR2 = ciclorojo; // Actualiza el ciclo de trabajo en LED rojo
// Verficamos si repetimos o invertimos
}while( cicloverde > 0 && ciclorojo < ciclomaximo ); // Repetimos mientras no se alcance un límite
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
