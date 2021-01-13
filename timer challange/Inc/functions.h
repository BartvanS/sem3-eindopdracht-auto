#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#define sensorCount 5

#define Ki 0
#define Kp 25
#define Kd 25

#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

void readSensors(int values[sensorCount]);
int calcError(int values[sensorCount]);
int calculatePID(int error);

#endif