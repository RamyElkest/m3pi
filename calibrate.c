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
#include "compass.h"
#include "m3pi.h"
#include "modules.h"
#include "mechanics.h"
#include <stdlib.h>



/*----------------- Global Variables ------------------------------------*/

// create file for outputting adc values
//FILE * pFile;
char outputPOS[5];
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
void dump_sensor(uint8_t channel)
{
	uint16_t i;
	for(i=0; i<10000; i++)
	{
		_DBD16(read_analog(channel)); _DBG_("");
		delay_ms(100);
	}	
}

/*---------------- Custom --------------------------------*/
void custom(void)
{
	//_DBG_("Starting..");

	//led(1,1<<18,1);
	//wallFollow(0,2);
	//uint8_t wtf[6];
	//compass_trx(0x28, wtf);
	
	//generateLUT();

	while(1){
		short readings[9];
		float test[3];
		uint8_t i, status = _accmagGetStatus();
		_DBG("Status: ");_DBD(status);_DBG_(" ");
		compass_read(readings );
		for(i=0;i<9;i++) {
			_DBD16(abs(readings[i])); _DBG(" ");
		}
		delay_ms(200);
	}

/*
	uint16_t sensors[5];
	uint32_t ticks = 0;
	while(1) {
		sensors_line_position(sensors);
		_DBD16(sensors[2]); _DBG_("");
		while(sensors[2] > 1900) sensors_line_position(sensors);
		while(sensors[2] <= 1900) sensors_line_position(sensors);
		ticks++;
		_DBD16(ticks);
	}
	forward(60);
	sleep(2);
	forward(127);
	sleep(1);
	forward(10);
	sleep(2);
	stop();

	while(1) ;

	uint8_t x = 0;
	while(1) {
		if (x == 255) {
			//_DBD16(getDist(read_analog(5)));_DBG_("");
			_DBD16(getAngleFromSensors(LEFT_SENSORS));_DBG_("");
			x=0;
		}
		x++;
	}
*/
/*
	while(1) {
		led(1,1<<18,1);
		delay_ms(1000);
		led(1,1<<18,0);
		led(1,1<<20,1);
		sleep(2);
		led(1,1<<20,0);
	}
*/
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
		systick_init();

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

















