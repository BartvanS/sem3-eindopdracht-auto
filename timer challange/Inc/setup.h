#ifndef SETUP_H
#define SETUP_H

#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

void setupServos();
void setupLeds();
void setupControls();
void setupEncoder();
void setupSensors();

#endif