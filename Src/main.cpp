#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

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
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

void setupServos(){
  //servo pin 13
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->PSC = 71; // 100.000 clockspeed -> 1 step per 10 micro seconden
  TIM3->CCMR1 = TIM3->CCMR1 & ~TIM_CCMR1_CC1S;
  TIM3->CCMR1 = (TIM3->CCMR1 & ~TIM_CCMR1_OC1M) | (0b0110<<TIM_CCMR1_OC1M_Pos);
  TIM3->ARR = 20000; // 2000 ipv 20.000 want we hebben stappen van 10 micro seconden.
  // TIM3->CCR1 = 128; //128 x 10 = 1280 micro seconden -> snelst met de klok mee
  // TIM3->CCR1 = 172; // 172 x 10 = 1720 micro seconden -> snelst tegen de klok in
  TIM3->CCR1 = 1500; // 150 x 10 = 1500 micro seconden -> stop
  TIM3->CCER = TIM3->CCER | TIM_CCER_CC1E;
  TIM3->CR1 |= TIM3->CR1 | TIM_CR1_CEN;
  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER6) | (0b10 << GPIO_MODER_MODER6_Pos);
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL6) | (0b0010 << GPIO_AFRL_AFRL6_Pos);

  //servo pin 12
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 71;
  TIM2->CCMR1 = TIM2->CCMR1 & ~TIM_CCMR1_CC1S;
  TIM2->CCMR1 = (TIM2->CCMR1 & ~TIM_CCMR1_OC1M) | (0b0110<<TIM_CCMR1_OC1M_Pos);
  TIM2->ARR = 20000;
  // TIM2->CCR1 = 172;
  TIM2->CCR1 = 1500;
  TIM2->CCER = TIM2->CCER | TIM_CCER_CC1E;

  TIM2->CR1 |= TIM2->CR1 | TIM_CR1_CEN;
  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER5) | (0b10 << GPIO_MODER_MODER5_Pos);
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL5) | (0b0001 << GPIO_AFRL_AFRL5_Pos);
}

void setupLeds(){
  //pa13
    GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER7) | (0b01 << GPIO_MODER_MODER7_Pos);       // set pin PA11 to output.
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT_7;  
}

void setupControls(){
  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER0) | (0b00 << GPIO_MODER_MODER0_Pos);       // set pin PA0 to input.
  GPIOA->PUPDR = (GPIOA->PUPDR & ~GPIO_PUPDR_PUPDR0) | (0b01 << GPIO_PUPDR_PUPDR0_Pos);       //pullup

  //interrupt
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~SYSCFG_EXTICR1_EXTI0) | (0b0000 << SYSCFG_EXTICR1_EXTI0_Pos);
  EXTI->FTSR |= EXTI_FTSR_TR0;   // Set interrupt EXTI* trigger to falling edge
  EXTI->IMR = EXTI_IMR_MR0;     // Unmask EXTI* line
  NVIC_EnableIRQ(EXTI0_IRQn);     
}

void setupEncoder(){
  //INTERRUPT pin encoder HOR (PB4)
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER4) | (0b00 << GPIO_MODER_MODER4_Pos); // set pin PA0 to input.
  GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR4) | (0b00 << GPIO_PUPDR_PUPDR4_Pos);
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI2) | (0b0000 << SYSCFG_EXTICR1_EXTI2_Pos);
  EXTI->FTSR |= EXTI_FTSR_TR1; // Set interrupt EXTI* trigger to falling edge
  EXTI->IMR |= EXTI_IMR_MR1;   // Unmask EXTI* line
  NVIC_EnableIRQ(EXTI1_IRQn);

  //NON INTERRUPT pin motor HOR (PB5)
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER3) | (0b00 << GPIO_MODER_MODER3_Pos); // set pin PA1 to input.
  GPIOB->PUPDR |= (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR3) | (0b00 << GPIO_PUPDR_PUPDR3_Pos);
}

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

extern "C" void EXTI0_IRQHandler(void)  // Do not forget the ‘extern “C”’ in case of C++
{
  EXTI->PR |= EXTI_PR_PR0;  
  if (!isOn)
  {
    startSystem();
  }else{
    stopSystem();
  }
  isOn = !isOn;
}

int speedMotor1 = FULLSPEEDF;
int speedMotor2 = FULLSPEEDB;
extern "C" void EXTI1_IRQHandler(void)  // Do not forget the ‘extern “C”’ in case of C++
{
  EXTI->PR |= EXTI_PR_PR1;  
  if ((GPIOB->IDR & GPIO_IDR_5) == 32)
  {
    // zet waardes omlaag
    if(speedMotor1 > FULLSPEEDB){
      speedMotor1--;
    }

    if(speedMotor2 < FULLSPEEDF){
      speedMotor2++;
    }
  }
  else
  {
    // zet waardes omhoog
  }
}
/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf(msgBuf, "%s", "Hello World!\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);

  setupServos();
  setupLeds();
  setupControls();
  setupEncoder();

  while (1)
  {

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

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
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
