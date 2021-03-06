#include "setup.h"

void setupServos(){
  //servo pin 13
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->PSC = 71; // 100.000 clockspeed -> 1 step per 1 micro seconden
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
  //INTERRUPT pin encoder HOR (pa1)
  GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER1) | (0b00 << GPIO_MODER_MODER1_Pos); // set pin PA1 to input.
  GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR1) | (0b00 << GPIO_PUPDR_PUPDR1_Pos);
  SYSCFG->EXTICR[0] = (SYSCFG->EXTICR[0] & ~SYSCFG_EXTICR1_EXTI1) | (0b0000 << SYSCFG_EXTICR1_EXTI1_Pos);
  EXTI->FTSR |= EXTI_FTSR_TR1; // Set interrupt EXTI* trigger to falling edge
  EXTI->IMR |= EXTI_IMR_MR1;   // Unmask EXTI* line
  NVIC_EnableIRQ(EXTI1_IRQn);

  //NON INTERRUPT pin motor HOR (PB4)
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER4) | (0b00 << GPIO_MODER_MODER4_Pos); 
  GPIOB->PUPDR |= (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR4) | (0b00 << GPIO_PUPDR_PUPDR4_Pos);
}

void setupSensors(){
  //sensor pa10 / d2
  GPIOA->MODER |= (GPIOA->MODER & ~GPIO_MODER_MODER10) | (0b00 << GPIO_MODER_MODER10_Pos); // set pin PA1 to input.
  GPIOA->PUPDR |= (GPIOA->PUPDR & ~GPIO_PUPDR_PUPDR10) | (0b00 << GPIO_PUPDR_PUPDR10_Pos);
  //sensor pb3 / d3
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER3) | (0b00 << GPIO_MODER_MODER3_Pos); // set pin PA1 to input.
  GPIOB->PUPDR |= (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR3) | (0b00 << GPIO_PUPDR_PUPDR3_Pos);
//sensor pb5 / d4
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER5) | (0b00 << GPIO_MODER_MODER5_Pos); // set pin PA1 to input.
  GPIOB->PUPDR |= (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR5) | (0b00 << GPIO_PUPDR_PUPDR5_Pos);
  //sensor pb10
  GPIOB->MODER |= (GPIOB->MODER & ~GPIO_MODER_MODER10) | (0b00 << GPIO_MODER_MODER10_Pos); // set pin PA1 to input.
  GPIOB->PUPDR |= (GPIOB->PUPDR & ~GPIO_PUPDR_PUPDR10) | (0b00 << GPIO_PUPDR_PUPDR10_Pos);
  //sensor pa8 / d7
  GPIOA->MODER |= (GPIOA->MODER & ~GPIO_MODER_MODER8) | (0b00 << GPIO_MODER_MODER8_Pos); // set pin PA1 to input.
  GPIOA->PUPDR |= (GPIOA->PUPDR & ~GPIO_PUPDR_PUPDR8) | (0b00 << GPIO_PUPDR_PUPDR8_Pos);
}