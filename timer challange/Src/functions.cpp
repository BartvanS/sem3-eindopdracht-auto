#include "functions.h"

void readSensors(int values[5]){
    // a10
  if((GPIOA->IDR & GPIO_IDR_10) == 1024){
      values[0] = 0;
  }else{
      values[0] = 1;
  }
    // b3
  if((GPIOB->IDR & GPIO_IDR_3) == 8){
      values[1] = 0;
  }else{
      values[1] = 1;
  }
    // b5
  if((GPIOB->IDR & GPIO_IDR_5) == 32){
      values[2] = 0;
  }else{
      values[2] = 1;
  }
    //b10
  if((GPIOB->IDR & GPIO_IDR_10) == 1024){
      values[3] = 0;
  }else{
      values[3] = 1;
  }
    //a8
  if((GPIOA->IDR & GPIO_IDR_8) == 256){
      values[4] = 0;
  }else{
      values[4] = 1;
  }
}

int calcError(int values[5]){
    int error = 0;

    if((values[0]== 0 )&&(values[1]== 0 )&&(values[2]== 0 )&&(values[3]== 0 )&&(values[4]== 1 )) error = 4;

    else if((values[0]== 0 )&&(values[1]== 0 )&&(values[2]== 0 )&&(values[3]== 1 )&&(values[4]== 1 )) error = 3; 

    else if((values[0]== 0 )&&(values[1]== 0 )&&(values[2]== 0 )&&(values[3]== 1 )&&(values[4]== 0 )) error = 2;

    else if((values[0]== 0 )&&(values[1]== 0 )&&(values[2]== 1 )&&(values[3]== 1 )&&(values[4]== 0 )) error = 1;

    else if((values[0]== 0 )&&(values[1]== 0 )&&(values[2]== 1 )&&(values[3]== 0 )&&(values[4]== 0 )) error = 0;

    else if((values[0]== 0 )&&(values[1]== 1 )&&(values[2]== 1 )&&(values[3]== 0 )&&(values[4]== 0 )) error =- 1;

    else if((values[0]== 0 )&&(values[1]== 1 )&&(values[2]== 0 )&&(values[3]== 0 )&&(values[4]== 0 )) error = -2;

    else if((values[0]== 1 )&&(values[1]== 1 )&&(values[2]== 0 )&&(values[3]== 0 )&&(values[4]== 0 )) error = -3;

    else if((values[0]== 1 )&&(values[1]== 0 )&&(values[2]== 0 )&&(values[3]== 0 )&&(values[4]== 0 )) error = -4;

    return error;
}

int P = 0;
int I = 0;
int D = 0;
int previousError = 0;
int counter = 0;
int calculatePID(int error){
       if (counter >= 10)
    {
        I = 0;
        counter = 0;
    }
    P = error;
    I = I + error;
    D = error - previousError;
    previousError = error;
    counter++;
 
    return (Kp*P) + (Ki*I) + (Kd*D);
}