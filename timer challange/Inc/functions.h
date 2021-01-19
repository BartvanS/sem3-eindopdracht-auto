#ifndef FUNCTIONS_H
#define FUNCTIONS_H
#define sensorCount 5

#define Kp 30
#define Ki 0
#define Kd 0

#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

void readSensors(int values[sensorCount]);
int calcError(int values[sensorCount]);
int calculatePID(int error);

#endif