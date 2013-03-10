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
#include "imu.h"
#include "m3pi.h"
#include "modules.h"
#include "mechanics.h"
#include "LocalFileSystem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>



/*----------------- Global Variables ------------------------------------*/

// create file for outputting adc values
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
	// Right/Left? Wheel Encoder settings: pin 4, port , rising edge
	if(GPIO_GetIntStatus(2, 4, 1))
	{
		right_wheel_enc.ticks += (right_wheel_enc.direction == 0) ? 1 : -1;	// If direction == 0 (moving forwards) add 1 to ticks, else sub 1
		GPIO_ClearInt(2,(1<<4));	// Clear the interrupt
	}
	// Right/Left? Wheel Encoder settings: pin 4, port , rising edge
	if(GPIO_GetIntStatus(2, 5, 1))
	{
		left_wheel_enc.ticks += (left_wheel_enc.direction == 0) ? 1 : -1;	// If direction == 0 (moving forwards) add 1 to ticks, else sub 1
		GPIO_ClearInt(2,(1<<5));	// Clear the interrupt
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
	unsigned char send = 'a';
	play_sound(1, &send);
	_DBG("# Sensor "); _DBD(channel); _DBG_("");
	for(i=0; i<128; i++)
	{
		_DBD16(read_analog(channel)); _DBG(", ");
		delay_ms(1);
	}	
	play_sound(1, &send);
}
void dump_sensor_file(uint8_t channel, uint8_t cm)
{
	char str[9];
	uint16_t i, tmp;
	play_sound(1, (unsigned char *)"a");
	for(i=0; i<128; i++)
	{
		tmp = read_analog(channel);
		sprintf(str,"%d %d\r\n", cm, tmp);
		LocalFileHandle_write(str, (int)strlen(str));
	}
	LocalFileHandle_write("\r\n", 2);
	play_sound(1, (unsigned char *)"d");
}

void calibrate_analog( void ) {
	
	uint8_t i, j;
	FILEHANDLE myMap;

	for(j=0;j<5;j++) {
		switch(j){
			case 0:
				myMap = LocalFileSystem_open("sensor5.txt", O_WRONLY);
				break;
			case 1:
				myMap = LocalFileSystem_open("sensor0.txt", O_WRONLY);
				break;
			case 2:
				myMap = LocalFileSystem_open("sensor2.txt", O_WRONLY);
				break;
			case 3:
				myMap = LocalFileSystem_open("sensor1.txt", O_WRONLY);
				break;
			case 4:
				myMap = LocalFileSystem_open("sensor4.txt", O_WRONLY);
				break;
		}
		for(i=5; i<=85; i+=5) {
		switch(j){
			case 0:
				dump_sensor_file(5, i);
				break;
			case 1:
				dump_sensor_file(0, i);
				break;
			case 2:
				dump_sensor_file(2, i);
				break;
			case 3:
				dump_sensor_file(1, i);
				break;
			case 4:
				dump_sensor_file(4, i);
				break;
		}
			sleep(3);
		}
	LocalFileHandle_close();
	}
}

/*---------------- Custom --------------------------------*/
void custom(void)
{
	_DBG_("Starting..");

//	calibrate_analog();
/*
	play_sound(1, (unsigned char *)"a");
	myMap = LocalFileSystem_open("my_map.txt", O_WRONLY);
	_DBG_("Opened..");
	LocalFileHandle_write("hey my name is Jennifer", 23);
	_DBG_("Wrote..");
	LocalFileHandle_close();
	_DBG_("Closed..");
	play_sound(1, (unsigned char *)"d");
	myMap = LocalFileSystem_open("my_map.txt", O_RDONLY);
	_DBG_("Opened again..");
	LocalFileHandle_read(read, 10);
	_DBG_("Read..");
	LocalFileHandle_close();
	_DBG_("Closed again..");

	
	
	while(1) {
	imu_update();
	_DBD16(adcAvg[0]); _DBG(", ");
	_DBD16(adcAvg[1]); _DBG(", ");
	_DBD16(adcAvg[2]); _DBG_(" ");
	}

/*
	//newWallFollow(LEFT_SENSORS);
	dump_sensor(5); //10
	sleep(10);
	dump_sensor(5); //15
	sleep(10);
	dump_sensor(5); //20
	sleep(10);
	dump_sensor(5); //25
	sleep(10);
	dump_sensor(5); //30
*/
	while(1) {
		_DBD16(read_analog(0)); _DBG(" . ");
		_DBD16(read_analog(1)); _DBG(" . ");
		_DBD16(read_analog(2)); _DBG(" . ");
		_DBD16(read_analog(4)); _DBG(" . ");
		_DBD16(read_analog(5)); _DBG_("");
		delay_ms(500);
	}

char str[50];
	while(1) {
		sprintf(str, "%d %f\0", read_analog(5), getDist(read_analog(5)));
		_DBG(str); _DBG_(" .");
		sleep(1);
	}


	while(1);

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
		digital_init(2, 4);		// Wheel encoder (Right/Left?)
		digital_init(2, 5);		// Wheel encoder (Right/Left?)
		
		// Initialise system tick (for delays)
		systick_init();

		// Initialise Gyroscope
		//compass_init();
		//imu_init();
		
		//Initialise MBED LED1 (1.18)
		led_init(1, (1<<18)|(1<<20)|(1<<21)|(1<<23));
		
		// custom movement..
		custom();

	   // Enable interrupts globally.
	   __enable_irq();
        while (1);
        return 1;

}

















