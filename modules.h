/*	modules.h
 *
 */

#ifndef MODULES_H
#define MODULES_H

#include "lpc_types.h"


/*	Serial.c
 *
 *	Initialises UART3 for MBED to 3pi interface
 */
void serputc(uint8_t tmp);

uint8_t sergetc(void);

void m3pi_init();


/* TIMER0 INITIALISATION
 *
 * Initialise timer0
 */
void timer0_init();

/*	ADC
 *
 *	Initialises adc channel 0 for sensor to MBED interface
 *
 *  NOTE: Sharp GP2Y0A21YK0F has timing 38.3ms ± 9.6ms
 */
void adc_init (uint8_t portNum, uint8_t pinNum, uint8_t funcNum, uint8_t channelNum);

/* digital GPIO
 *
 *	Initialises a GPIO for a digital sensor
 */
void digital_init (uint8_t port, uint8_t pinNum);

/* Compass Initialisation
 *
 *	Initialises a I2C for a compass (accelerometer, gyroscope, magnetometer)
 *
 *	Uses external interrupt 3, rising edge
 *
 *	@param GPIO pin number
 *	@param GPIO pin port
 */
void compass_init ();
uint8_t compass_trx(uint8_t send, uint8_t *receive);

/* Initialise system tick
 *
 * @param time interval in milliseconds
 * @param counter functional state, ENABLE / DISABLE
 * @param interrupt functional state, ENABLE / DISABLE
 */
void systick_init(uint32_t time, FunctionalState interrupt);

/* Delay in ms
*/
void delay_ms(uint32_t end_time);

/* LED GPIO
 *
 *	Initialises a GPIO for a LED
 *
 *	@param GPIO port number
 *	@param GPIO pin(s) number/map
 */
void led_init(uint8_t port, uint32_t pins);

/* LED on/off
 *
 *	Switches MBED LED on/off
 *
 *	@param LED pin
 *	@param GPIO pin(s) number/map
 *	@param boolean 1 - ON ; 0 - OFF
 */
void led(uint8_t port, uint32_t pins, uint8_t state);

/* Read Analogue sensor value
 *
 * Start ADC conversion, poll and return result
 *
 * @param ADC channel, 0-7
 * @return unsigned int 32 bit resolution
 */
 uint32_t read_analog (uint8_t channelNum);
 
 /* Align robot with horizontal line
 *
 * Called only when one of sensors 0/4 is on a horizontal line
 */
void align (void);

/* Move to next horizontal line
 *
 *
 *
 */
void nextLine (void);

/* Follow Wall PID
 *
 *
 * @param channelA, ADC channel to read first sensor
 * @param channelB, ADC channel to read second sensor
 */

void wallFollow(uint8_t channelA, uint8_t channelB);
 
#endif
