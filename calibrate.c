/* Calibrate.c
 *
 * Calibrates m3pi sensors, orientation, position, internal motors, etc.
 *
 * Calibration works by first calbrating the front sensor and rotates to equate the rest
 *
 * @author Ramy Elkest
 */
#include "debug_frmwrk.h"
#include "lpc_types.h"
#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_gpio.h"
#include "m3pi.h"
#include "modules.h"
#include <stdlib.h>

#define CALIB_NUM 8 // Number of calibration points
#define CALIB_RES 10 // Number of centimeters between calibration points

/*----------------- Global Variables ------------------------------------*/

// create file for outputting adc values
//FILE * pFile;
char outputPOS[5];
uint16_t sensor_lut[CALIB_NUM];
uint8_t count = 0;


/*----------------- INTERRUPT SERVICE ROUTINES --------------------------*/

void TIMER0_IRQHandler(void)
{
        if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT)== SET)
        {
				int pos;
        		uint16_t adcVal;
				pos = line_position();
				adcVal = read_analog(0);	// Read ADC 0.0
				
				if(read_analog(0) > (uint16_t)2500) led(1,1<<18,1);

				
				if(count % 10 == 0) {
					int i;

					_DBG("ADC channel output: ");
					for(i=0;i<4;i++)
					{
						if(i==3) i++;
						_DBD16(read_analog(i));
						_DBG(" ");
					}
						
					// Output debug
					/*_DBG("POS value on channel 0: ");
					_DBD16(pos);
					_DBG(" ");
					_DBD16(adcVal);
					_DBG_("");
					
					// Output pos to screen
					cls();
					sprintf(outputPOS, "%d", adcVal);
					print(outputPOS, 5);
					count = 0;
					*/
				}
				count++;
				
        }
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
}

// The ADC interrupt handler, gets called every time an A to D conversion completes.
void ADC_IRQHandler (void)
{
	// Currently not used
}

// GPIO interrupt handler. External interrupt 3, Digital sensor 
void EINT3_IRQHandler(void)
{
	// Digital sensor on default settings: pin 18, port 0, rising edge
	if(GPIO_GetIntStatus(0, 18, 1))
	{
		GPIO_ClearInt(0,(1<<18));	// Clear the interrupt
		stop();
		while(1);
	}
}
/*----------------- Calibates sensor angles ---------------*/
/******************
Calibrate Angles finds the relative angles of sensors
1. Rotate until one of the sensors hits 2v
2. Read sensor while rotating 

******************/
int generateLUT(void)
{	
	uint8_t i;
	for(i=0;i<CALIB_NUM;i++)
	{
		// Go to next line and Align
		nextLine();
		// Take readings
		sensor_lut[i] = read_analog(5);
	}
	return 0;
	
}

/**
 *	Linear Interpolation using sensor lut and return cm from sensor values
 *
 *	y = (x-x_0)(y_1-y_0)/(x_1-x_0) + y_0
 *	y0, 
 **/

uint8_t getDist(uint16_t sensor_value)
{	
	if(sensor_value < sensor_lut[0]) return 0;
	if(sensor_value > sensor_lut[CALIB_NUM-1]) return 80;
	
	uint16_t y0,y1,x0,x1,x;
	uint8_t i;

	// Find upper index of sensor_value
	for(i=0;i<CALIB_NUM;i++) if(sensor_value < sensor_lut[i]) break;

	// Values use i+1, therefore upper value as lower value
	x = sensor_value;
	y0 = (i)*CALIB_RES; y1 = (i+1)*CALIB_RES;
	x0 = sensor_lut[i-1]; x1 = sensor_lut[i];
	
 	return ((x-x0)*(y1-y0))/(x1-x0) + y0;
}

/*---------------- Custom --------------------------------*/
void custom(void)
{
	_DBG_("Starting..");

	//led(1,1<<18,1);
	//wallFollow(0,2);
	//uint8_t wtf[6];
	//compass_trx(0x28, wtf);

	
	generateLUT();
	getDist(read_analog(5));

	while(1);
	
	_DBG_("Finished..");
}

/*----------------- Main Function --------------------------*/
int main(void)
{
        /* Initialize debug via UART0
         * � 115200bps
         * � 8 data bit
         * � No parity
         * � 1 stop bit
         * � No flow control
         */
        debug_frmwrk_init();

		// initalise timer0(mc0) (for regular interrupt)
        //timer0_init();
        
        // initialise m3pi interface
        m3pi_init();

        // initialise adc
		// (Port, Pin, Func, Channel)
		adc_init(0, 23, 1, 0);	//ADC 0.0
		adc_init(0, 24, 1, 1);	//ADC 0.1
		adc_init(0, 25, 1, 2);	//ADC 0.2
		adc_init(1, 30, 3, 4);	//ADC 0.4
		adc_init(1, 31, 3, 5);	//ADC 0.5
		
		// initialise digital sensor
		digital_init(0, 18);	//GPIO 0.18
		
		// Initialise system tick (for delays)
		systick_init(10, DISABLE);

		// Initialise Gyroscope
		compass_init();
		
		//Initialise MBED LED1 (1.18)
		led_init(1, (1<<18)|(1<<20)|(1<<21)|(1<<23));
		
		// custom movement..
		custom();

	   // Enable interrupts globally.
	   __enable_irq();
        while (1);
        return 1;

}

















