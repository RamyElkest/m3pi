#include "LPC17xx.h"

#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1

void Compute();
 
void SetTunings(float Kp, float Ki, float Kd);
 
void SetSampleTime(uint32_t NewSampleTime);
 
void SetOutputLimits(uint8_t limit);
  
void Initialize();

void SetMode(uint8_t Mode);

void SetControllerDirection(uint8_t Direction);
