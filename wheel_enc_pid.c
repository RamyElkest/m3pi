#include "debug_frmwrk.h"
#include "LPC17xx.h"
#include "modules.h"
#include "mechanics.h"
#include "m3pi.h"
#include "wheel_enc_pid.h"

/*working variables*/
unsigned long lastTime;
float Input, Output, Setpoint;
float ITerm, lastInput;
float kp, ki, kd;
int SampleTime = 1000; //1 sec
float outMin, outMax;
uint8_t inAuto = 0;
 
int controllerDirection = DIRECT;
 
void Compute()
{
   if(!inAuto) return;
   uint64_t now = millis();
   uint32_t timeChange = (now - lastTime);
   if(timeChange >= SampleTime)
   {
	  Input = right_wheel_enc.ticks - left_wheel_enc.ticks;
	  _DBG("Input: ");_DBD16(right_wheel_enc.ticks);_DBG(" - ");
	  _DBD16(left_wheel_enc.ticks);_DBG_("");
      /*Compute all the working error variables*/
      float error = Input;
      ITerm += (ki * error);
      if(ITerm > outMax) ITerm = outMax;
      else if(ITerm < outMin) ITerm = outMin;
      float dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm - kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
	  /*Set Motors*/
	  if(Output < 0) {
	 	left_motor(outMax + Output);
	  	right_motor(outMax);
	  } else {
	 	left_motor(outMax);
	 	right_motor(outMax - Output);
	  }

      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
}
 
void SetTunings(float Kp, float Ki, float Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
 
  float SampleTimeInSec = ((float)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection == REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
 
void SetSampleTime(uint32_t NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      float ratio  = (float)NewSampleTime
                      / (float)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(uint8_t limit)
{
   outMin = -limit;
   outMax = limit;
 
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
  
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}

void SetMode(uint8_t Mode)
{
    uint8_t newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}

void SetControllerDirection(uint8_t Direction)
{
   controllerDirection = Direction;
}
