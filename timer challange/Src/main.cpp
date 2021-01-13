#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include "setup.h"
#include "functions.h"
#include "simpleQueue.h"
#define FULLSPEEDF 1720
#define FULLSPEEDB 1280
#define STOPMOTOR 1500

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .attr_bits = osThreadDetached,
    .cb_mem = NULL,
    .cb_size = 0,
    .stack_mem = NULL,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
    .tz_module = 0,
    .reserved = 0};

//communication
    osThreadId_t comTaskHandle;
const osThreadAttr_t comTask_attributes = {
    .name = "comTask",
    .attr_bits = osThreadDetached,
    .cb_mem = NULL,
    .cb_size = 0,
    .stack_mem = NULL,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
    .tz_module = 0,
    .reserved = 0};

    // //sensors
    // osThreadId_t sensorTaskHandle;
    // const osThreadAttr_t sensorTask_attributes = {
    //   .name = "sensorTask",
    //   .attr_bits = osThreadDetached,
    //   .cb_mem = NULL,
    //   .cb_size = 0,
    //   .stack_mem = NULL,
    //   .stack_size = 128 * 4,
    //   .priority = (osPriority_t)osPriorityNormal,
    //   .tz_module = 0,
    //   .reserved = 0
    //   };

    // //PID Controller
    // osThreadId_t PIDTaskHandle;
    // const osThreadAttr_t PIDTask_attributes = {
    //   .name = "PIDTask",
    //   .attr_bits = osThreadDetached,
    //   .cb_mem = NULL,
    //   .cb_size = 0,
    //   .stack_mem = NULL,
    //   .stack_size = 128 * 4,
    //   .priority = (osPriority_t)osPriorityNormal,
    //   .tz_module = 0,
    //   .reserved = 0
    //   };
      
    // //Motor Controller
    // osThreadId_t motorTaskHandle;
    // const osThreadAttr_t motorTask_attributes = {
    //   .name = "motorTask",
    //   .attr_bits = osThreadDetached,
    //   .cb_mem = NULL,
    //   .cb_size = 0,
    //   .stack_mem = NULL,
    //   .stack_size = 128 * 4,
    //   .priority = (osPriority_t)osPriorityNormal,
    //   .tz_module = 0,
    //   .reserved = 0
    //   };
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);
void StartComTask(void *argument);


void startSystem(){
  //led pa5
  GPIOA->ODR |= GPIO_ODR_7; 
  TIM2->CCR1 = FULLSPEEDF;
  TIM3->CCR1 = FULLSPEEDB; 
}
void stopSystem(){
  TIM2->CCR1 = STOPMOTOR;
  TIM3->CCR1 = STOPMOTOR; 
  GPIOA->ODR &= ~GPIO_ODR_7;
}
static char msgBuf[80];
bool isOn = false;
int speedMotor1 = FULLSPEEDF;
int speedMotor2 = FULLSPEEDB;

void handleOnOff(){
  if (!isOn)
  {
    startSystem();
  }else{
    stopSystem();
  }
  isOn = !isOn;
}

extern "C" void EXTI0_IRQHandler(void)  // Do not forget the ‘extern “C”’ in case of C++
{
  EXTI->PR |= EXTI_PR_PR0;  
  handleOnOff();
}


extern "C" void EXTI1_IRQHandler(void)  // Do not forget the ‘extern “C”’ in case of C++
{
  EXTI->PR |= EXTI_PR_PR1;  
  if ((GPIOB->IDR & GPIO_IDR_4) == 16)
  {
    // zet snelheid van de motoren omhoog motor 1 > 1280 motor 2 < 1720
    if(speedMotor1 > FULLSPEEDB){
      speedMotor1 -= 10;
    }
    if(speedMotor2 < FULLSPEEDF){
      speedMotor2+= 10;
    }
  }
  else
  {
    // zet snelheid van de motoren omhoog motor 1 < 1720 motor 2 > 1280
    if(speedMotor1 < FULLSPEEDF){
      speedMotor1+= 10;
    }

    if(speedMotor2 > FULLSPEEDB){
      speedMotor2-= 10;
    }
  }
  if(isOn){
  TIM2->CCR1 = speedMotor1;
  TIM3->CCR1 = speedMotor2; 
  }
}
/**
  * @brief  The application entry point.
  * @retval int
  */
SimpleQueue* queue;
int sensorValues[] = {0,0,0,0,0};


void setupQueue(){
  queue = NULL;
}
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

//
    setupServos();
  setupLeds();
  setupControls();
  setupEncoder();
  setupSensors();
  setupQueue();
//

  sprintf(msgBuf, "%s", "Hello World!\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  /* threads */
  osKernelInitialize();
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  comTaskHandle = osThreadNew(StartComTask, NULL, &comTask_attributes);
  osKernelStart();

  while (1)
  {

  }
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


void StartDefaultTask(void *argument)
{
  sprintf(msgBuf, "test \n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
      

  addToQueue(&queue, (char*)"test");
sprintf(msgBuf, "test \n");
    HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

//     sprintf(msgBuf, "value: %s \n", retrieveFromQueue(&queue));
//     HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
//        sprintf(msgBuf, "value: %s \n", retrieveFromQueue(&queue));
//     HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    
    readSensors(sensorValues);
    sprintf(msgBuf, "Yooo: %d %d %d %d %d\r\n", sensorValues[0], sensorValues[1],sensorValues[2],sensorValues[3],sensorValues[4]);
    HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
  // osDelay(1000);
    // calculatePID();
    // setMotor();
    // setMotor();
  }
}

void StartComTask(void *argument){
  for (;;)
  {
    char in[8] = {'\0'}; 
    HAL_UART_Receive(&huart2, (uint8_t *)in, 8, 1000); 
    
    if(strcmp(in, "on") == 0){
      startSystem();
      isOn = true;
      in[0] = '\0';
    }else if(strcmp(in, "off") == 0){
      stopSystem();
      isOn = false;
      in[0] = '\0';
    }
    
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 38400;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */




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

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
