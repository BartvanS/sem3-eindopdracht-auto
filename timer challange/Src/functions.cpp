#include "functions.h"

void readSensors(int values[5]){
    // a10
  if((GPIOA->IDR & GPIO_IDR_10) == 1024){
      values[0] = 1;
  }else{
      values[0] = 0;
  }
    // b3
  if((GPIOB->IDR & GPIO_IDR_3) == 8){
      values[1] = 1;
  }else{
      values[1] = 0;
  }
    // b5
  if((GPIOB->IDR & GPIO_IDR_5) == 32){
      values[2] = 1;
  }else{
      values[2] = 0;
  }
    //b10
  if((GPIOB->IDR & GPIO_IDR_10) == 1024){
      values[3] = 1;
  }else{
      values[3] = 0;
  }
    //a8
  if((GPIOA->IDR & GPIO_IDR_8) == 256){
      values[4] = 1;
  }else{
      values[4] = 0;
  }
}