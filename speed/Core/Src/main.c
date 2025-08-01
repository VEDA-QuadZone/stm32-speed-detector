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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define A1_Pin GPIO_PIN_0
#define A1_Port GPIOA

#define A2_Pin GPIO_PIN_1
#define A2_Port GPIOA

#define B1_Pin GPIO_PIN_5
#define B1_Port GPIOB

#define B2_Pin GPIO_PIN_6
#define B2_Port GPIOB

// 거리 설정 (단위 : M)
#define SENSOR_DISTANCE_M 0.5f

// 한번에 감지할 수 있는 차량 수
#define MAX_CARS 10

// 센서 중복 감지
#define MIN_DETECTION_INTERVAL 3000  // 3초

// 유효한 픅정 시간
#define MAX_WAIT_TIME_MS 10000 // 10초

// r과속 기준 속도 정의 (단위 : km/h)
#define OVERSPEED_THRESHOLD_KMPH 1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

// 시간 저장용
uint32_t start_queue_A[MAX_CARS];
uint32_t start_queue_B[MAX_CARS];
int queue_head_A = 0, queue_tail_A = 0;
int queue_head_B = 0, queue_tail_B = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
void process_sensor_event(uint16_t event_pin, uint32_t* last_detection_time,
                          uint32_t* start_queue, int* queue_head, int* queue_tail,
                          const char* lane_prefix_char);

void check_queue_timeouts(void);

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
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  /* UART 안정화 대기 및 첫 송신 테스트 */
  HAL_Delay(100);  // UART 초기화 직후 안정화 시간
  // char start_msg[] = "Sensor status check started\r\n";
  // HAL_UART_Transmit(&huart6, (uint8_t*)start_msg, strlen(start_msg), HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* 전역 변수 */

  while (1)
  {
    /* USER CODE END WHILE */
	    check_queue_timeouts();
	    HAL_Delay(10); // 적절한 지연 시간
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // A1: PA0 → EXTI0
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  // A2: PA1 → EXTI1
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  // B1/B2: PB5, PB6 → EXTI9_5
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// HAL_GPIO_EXTI_Callback 함수 수정
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // 각 핀별로 마지막 감지 시간 저장 (static 변수로 선언하여 함수 호출 간에 값 유지)
    // 이 static 변수들은 이제 process_sensor_event 함수로 직접 전달됩니다.
    static uint32_t last_detection_time_A1 = 0;
    static uint32_t last_detection_time_A2 = 0;
    static uint32_t last_detection_time_B1 = 0;
    static uint32_t last_detection_time_B2 = 0;

    if (GPIO_Pin == A1_Pin) {
        process_sensor_event(A1_Pin, &last_detection_time_A1, start_queue_A, &queue_head_A, &queue_tail_A, "[A");
    }
    else if (GPIO_Pin == A2_Pin) {
        process_sensor_event(A2_Pin, &last_detection_time_A2, start_queue_A, &queue_head_A, &queue_tail_A, "[A");
    }
    else if (GPIO_Pin == B1_Pin) {
        process_sensor_event(B1_Pin, &last_detection_time_B1, start_queue_B, &queue_head_B, &queue_tail_B, "[B");
    }
    else if (GPIO_Pin == B2_Pin) {
        process_sensor_event(B2_Pin, &last_detection_time_B2, start_queue_B, &queue_head_B, &queue_tail_B, "[B");
    }
}

void process_sensor_event(uint16_t event_pin, uint32_t* last_detection_time,
                          uint32_t* start_queue, int* queue_head, int* queue_tail,
                          const char* lane_prefix_char)
{
    uint32_t now = HAL_GetTick();
    char msg[100];
    memset(msg, 0, sizeof(msg));
    char lane_char = lane_prefix_char[1]; // "[A" -> 'A' 추출 (예: 'A' 또는 'B')

    // 1. 중복 감지 방지 로직 (수신된 메시지는 STM32 내부 디버깅용으로만 사용하거나 제거)
    if ((now - *last_detection_time) < MIN_DETECTION_INTERVAL) {
        // 주석 처리하여 라즈베리파이로 보내지 않음:
        // sprintf(msg, "[%c%d] 감지 무시됨 (간격: %lu ms)\r\n", lane_char, (event_pin == A1_Pin || event_pin == B1_Pin) ? 1 : 2, now - *last_detection_time);
        // HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return; // 해당 인터럽트 처리 종료
    }
    *last_detection_time = now; // 마지막 감지 시간 업데이트

    // 2. A1/B1 (진입 센서) 처리
    // 진입 센서 감지 시에는 아직 속도 측정이 완료되지 않았으므로, 라즈베리파이로 메시지를 보내지 않습니다.
    if (event_pin == A1_Pin || event_pin == B1_Pin) {
        int next_tail = (*queue_tail + 1) % MAX_CARS;
        if (next_tail != *queue_head) {
            start_queue[*queue_tail] = now;
            *queue_tail = next_tail;
            // 주석 처리하여 라즈베리파이로 보내지 않음:
            // sprintf(msg, "[%c1] 차량 감지 (시간: %lu ms)\r\n", lane_char, now);
            // HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        } else {
            // 큐 오버플로우 메시지도 라즈베리파이로 보내지 않음:
            // sprintf(msg, "[%c1] 큐가 가득 찼습니다.\r\n", lane_char);
            // HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
    // 3. A2/B2 (종료 센서) 처리 - 이 시점에서 속도 계산 및 과속 판단 후 메시지 전송
    else if (event_pin == A2_Pin || event_pin == B2_Pin) {
        if (*queue_head != *queue_tail) { // 큐에 시작 시간이 있는 경우
            uint32_t start_time = start_queue[*queue_head];
            *queue_head = (*queue_head + 1) % MAX_CARS; // 큐에서 항목 제거

            uint32_t delta = (now >= start_time) ? (now - start_time) : 0; // 시간 차이 계산

            // 유효한 측정 시간 범위 내에서 속도 계산
            if (delta > 0 && delta < MAX_WAIT_TIME_MS) {
                float speed_mps = SENSOR_DISTANCE_M / ((float)delta / 1000.0f); // m/s
                float speed_kmph = speed_mps * 3.6f; // km/h

                // 4. 과속 여부 판단 및 라즈베리파이로 메시지 전송
                if (speed_kmph > OVERSPEED_THRESHOLD_KMPH) {
                    if (lane_char == 'A') {
                        sprintf(msg, "%.2f IN\r\n", speed_kmph); // A차선 과속: "속도 IN"
                    } else if (lane_char == 'B') {
                        sprintf(msg, "%.2f OUT\r\n", speed_kmph); // B차선 과속: "속도 OUT"
                    }
                    HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                }
                // 과속이 아닌 경우 (speed_kmph <= OVERSPEED_THRESHOLD_KMPH) 메시지를 보내지 않습니다.
            } else {
                // 시간 정보 오류인 경우에도 라즈베리파이로 메시지를 보내지 않음:
                // sprintf(msg, "[%c2] 시간 정보 오류 또는 초과\r\n", lane_char);
                // HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            }
        } else {
            // 큐에 데이터 없음 (A2/B2가 A1/B1 없이 감지된 경우)
            // 라즈베리파이로 메시지를 보내지 않음:
            // sprintf(msg, "[%c2] 큐에 데이터 없음\r\n", lane_char);
            // HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        }
    }
}

void check_queue_timeouts(void) {
    char msg[100];
    uint32_t now = HAL_GetTick();

    // A 레인 타임아웃 처리
    if (queue_head_A != queue_tail_A) {
        uint32_t oldest_start_time_A = start_queue_A[queue_head_A];
        if ((now - oldest_start_time_A) > MAX_WAIT_TIME_MS) {
            //sprintf(msg, "[A] A2 타임아웃: 차량 무시됨 (대기: %lu ms)\r\n", now - oldest_start_time_A);
            // HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            queue_head_A = (queue_head_A + 1) % MAX_CARS;
        }
    }

    // B 레인 타임아웃 처리
    if (queue_head_B != queue_tail_B) {
        uint32_t oldest_start_time_B = start_queue_B[queue_head_B];
        if ((now - oldest_start_time_B) > MAX_WAIT_TIME_MS) {
            // sprintf(msg, "[B] B2 타임아웃: 차량 무시됨 (대기: %lu ms)\r\n", now - oldest_start_time_B);
            // HAL_UART_Transmit(&huart6, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            queue_head_B = (queue_head_B + 1) % MAX_CARS;
        }
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
