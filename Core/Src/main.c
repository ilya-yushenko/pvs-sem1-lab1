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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ctype.h>
#include "stdio.h"
#include "string.h"
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
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId vLEDUpdateTestHandle;
osThreadId vUARTTaskHandle;
/* USER CODE BEGIN PV */
#define NUM_LEDS 4                		// Number of LEDs
#define FULL_PERIOD 1000          		// Full period in milliseconds (1 second)
#define TIMER_PERIOD 250          		// Timer period (250 ms)
#define HALF_PERIOD (FULL_PERIOD / 2) 	// Half cycle (500 ms)

#define MIN_FREQUENCY 0.1				// Min frequency (0.1 Hz)
#define MAX_FREQUENCY 9.9         		// Max frequency (9.9 Hz)
#define RX_BUFFER_SIZE 16         		// UART receive buffer size

#define NUM_TASKS 3

uint8_t led_states[NUM_LEDS];			// Array for storing LED states
uint16_t led_counter = 0;     			// LED cycling counter
uint16_t phase_shift = HALF_PERIOD;		// Initial phase shift (180 degrees)

float frequency = 1.0;      			// Starting frequency (1 Hz)

uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t command_received = 0;
uint8_t rx_index = 0;
volatile uint8_t btn_pressed_flag = 0;

typedef struct {
	void (*task_function)(void); // Указатель на функцию задачи
	uint32_t period; // Период выполнения задачи
	uint32_t last_execution_time; // Время последнего выполнения задачи
} Task;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
void StartLEDUpdateTask(void const * argument);
void StartUARTTask(void const * argument);

/* USER CODE BEGIN PFP */
void Task_LEDs_Control(void);
void Task_Command_Processing(void);
void Task_Button_Handler(void);
void Schedule_Tasks(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
Task tasks[NUM_TASKS] = {
    {Task_LEDs_Control, 0, 0},
    {Task_Command_Processing, 0, 0},
    {Task_Button_Handler, 0, 0}
};

void updateTimerPeriod(void) {
	uint16_t timer_period = (uint16_t) (frequency * 1000); // Time period in milliseconds

    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->ARR = (timer_period / 4) -1; // We divide the period into 4 parts, since we have one full period of diode glow, which is 4 timer clocks
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void Task_Command_Processing(void) {
    char command[RX_BUFFER_SIZE];
    strncpy(command, rx_buffer, RX_BUFFER_SIZE);
    command[RX_BUFFER_SIZE - 1] = 0; // Guarantee a null character at the end

    // Here is the command to restore the registry
    for (uint8_t i = 0; i < strlen(command); i++) {
        command[i] = tolower(command[i]);
    }

	if (rx_index != command_received) {
		double new_frequency;
		if (sscanf(command, "f=%lf", &new_frequency) == 1) {
			if (new_frequency >= MIN_FREQUENCY && new_frequency <= MAX_FREQUENCY) {
				frequency = new_frequency;
				updateTimerPeriod();
				char message[32];
				sprintf(message, "Frequency changed to %.1f Hz\r\n", frequency);
				HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message), HAL_MAX_DELAY);
			} else {
				char error_message[] = "Error: Frequency must be in the range from 0.1 to 9.9 Hz\r\n";
				HAL_UART_Transmit(&huart2, (uint8_t*) error_message, strlen(error_message), HAL_MAX_DELAY);
			}
		} else {
			char error_message[] = "Error: Invalid command format. Use the format 'F=x.x'\r\n";
			HAL_UART_Transmit(&huart2, (uint8_t*) error_message, strlen(error_message), HAL_MAX_DELAY);
		}
	}

    rx_index = 0;
    command_received = 0;
}

void Task_LEDs_Control(void) {
    for (int i = 0; i < NUM_LEDS; i++) {
        uint16_t offset = i * phase_shift; // Phase offset for each LED
        uint16_t phase = (led_counter + offset) % FULL_PERIOD; // LED Phase Calculation

        // Checking the condition for turning the LED on/off
        if (phase < HALF_PERIOD)
        {
            led_states[i] = 1; // Turn on LED
        }
        else
        {
            led_states[i] = 0; // Turn off LED
        }
    }

    led_counter += TIMER_PERIOD; // Increment the counter by the timer period for the next iteration
    if (led_counter >= FULL_PERIOD)
    {
        led_counter = 0; // Reset the counter after a full period
    }

    // LED control
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, led_states[0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, led_states[1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, led_states[2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, led_states[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Task_Button_Handler(void) {
    if (btn_pressed_flag)
    {
        phase_shift = (phase_shift == HALF_PERIOD) ? HALF_PERIOD / 2 : HALF_PERIOD;
        btn_pressed_flag = 0;
    }
}

void Schedule_Tasks(void) {
	uint32_t currentTime = HAL_GetTick();

	for (int i = 0; i < NUM_TASKS; i++) {
		if (tasks[i].period == 0 || currentTime - tasks[i].last_execution_time >= tasks[i].period) {
			tasks[i].task_function();
			tasks[i].last_execution_time = currentTime;
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin) {
		HAL_GPIO_TogglePin(LED_MODE_GPIO_Port, LED_MODE_Pin);
	    btn_pressed_flag = 1;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) { // Check that this is an interrupt from the desired UART
        rx_buffer[rx_index++] = huart->Instance->RDR; // Write the received byte to the buffer

        // Check if the received byte is a newline character
        if (rx_buffer[rx_index - 1] == '\n' || rx_buffer[rx_index - 1] == '\r') {
            rx_buffer[rx_index - 1] = 0; // Replace the newline character with a null character
            command_received = 1; // Set the flag that the command has been received
            tasks[1].task_function();
        } else if (rx_index == RX_BUFFER_SIZE - 1) {
            rx_index = 0; // Reset the buffer index if the buffer is full
        }

        HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_buffer[rx_index], 1);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buffer, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of vLEDUpdateTest */
  osThreadDef(vLEDUpdateTest, StartLEDUpdateTask, osPriorityNormal, 0, 128);
  vLEDUpdateTestHandle = osThreadCreate(osThread(vLEDUpdateTest), NULL);

  /* definition and creation of vUARTTask */
  osThreadDef(vUARTTask, StartUARTTask, osPriorityIdle, 0, 128);
  vUARTTaskHandle = osThreadCreate(osThread(vUARTTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  htim2.Init.Prescaler = 63999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_MODE_GPIO_Port, LED_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_MODE_Pin */
  GPIO_InitStruct.Pin = LED_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_MODE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLEDUpdateTask */
/**
  * @brief  Function implementing the vLEDUpdateTest thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLEDUpdateTask */
void StartLEDUpdateTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
* @brief Function implementing the vUARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void const * argument)
{
  /* USER CODE BEGIN StartUARTTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartUARTTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM2)
  {
    tasks[0].task_function();
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
