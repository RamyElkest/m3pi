/* m3pi Library
 *
 * Copyright (c) 2007-2010 cstyles
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "m3pi.h"
#include "modules.h"
#include <stdlib.h>

// Implement when needed
/*
void reset () {
    _nrst = 0;
    wait (0.01);
    _nrst = 1;
    wait (0.1);
}
*/

void left_motor (int8_t speed) {
    motor(0,speed);
}

void right_motor (int8_t speed) {
    motor(1,speed);
}

void forward (int8_t speed) {
    motor(0,speed);
    motor(1,speed);
}

void backward (int8_t speed) {
    motor(0,-1*speed);
    motor(1,-1*speed);
}

void left (int8_t speed) {
    motor(0,speed);
    motor(1,-1*speed);
}

void right (int8_t speed) {
    motor(0,-1*speed);
    motor(1,speed);
}

void stop (void) {
    motor(0,0);
    motor(1,0);
}

void motor (int motor, int8_t speed) {
    char opcode = 0x0;
    if (speed > 0) {
        if (motor==1)
            opcode = M1_FORWARD;
        else
            opcode = M2_FORWARD;
    } else {
        if (motor==1)
            opcode = M1_BACKWARD;
        else
            opcode = M2_BACKWARD;
    }
	speed = abs(speed);
	if (speed > 127) speed = 127;
    unsigned char arg = speed;

    serputc(opcode);
    serputc(arg);
}

float battery() {
    serputc(SEND_BATTERY_MILLIVOLTS);
    char lowbyte = sergetc();
    char hibyte  = sergetc();
    float v = ((lowbyte + (hibyte << 8))/1000.0);
    return(v);
}

uint16_t line_position() {
    uint16_t pos = 0;
    serputc(SEND_LINE_POSITION);
    pos = sergetc();
    pos += (sergetc() << 8);
    
    //float fpos = ((float)pos - 2048.0)/2048.0;
    return(pos);
}

uint16_t sensors_line_position(uint16_t* sensors) {

	uint8_t i, on_line = 0;
	uint32_t avg = 0; // this is for the weighted total, which is long before division
	uint16_t sum = 0, // this is for the denominator which is <= 64000
			 last_value=0, // assume initially that the line is left.
			 value;   // this is to store individual sensor value, for optimisation
  
	// Send opcode to get calibrated sensor values
    //serputc(SEND_CALIBRATED_SENSOR_VALUES);
    serputc(SEND_RAW_SENSOR_VALUES);
	
	for(i=0; i<5; i++) {
		// Get sensor value from 3pi
		value = sergetc();
		value += (sergetc() << 8);
		
		sensors[i] = value;
		
		if(i>0 && i < 4) {
			// Calculate an average for all sensors
			// keep track of whether we see the line at all
			if(value > 200) {
				on_line = 1;
			}
			// only average in values that are above a noise threshold
			if(value > 50) {
				avg += (uint32_t)(value) * (i * 1000);
				sum += value;
			}
		}
	}
	
	if(!on_line)
	{
		// If it last read to the left of center, return 0.
		if(last_value < 2000)
			return 0;
		// If it last read to the right of center, return the max.
		else
			return(4000);
	}
	
	last_value = avg/sum;
	return last_value;
}

char sensor_auto_calibrate() {
    serputc(AUTO_CALIBRATE);
    return(sergetc());
}


void calibrate(void) {
    serputc(PI_CALIBRATE);
}

void reset_calibration() {
    serputc(LINE_SENSORS_RESET_CALIBRATION);
}

void PID_start(int max_speed, int a, int b, int c, int d) {
	serputc(SET_PID);
    serputc(max_speed);
    serputc(a);
    serputc(b);
    serputc(c);
    serputc(d);
}

void PID_stop() {
    serputc(STOP_PID);
}

float pot_voltage(void) {
    int volt = 0;
    serputc(SEND_TRIMPOT);
    volt = sergetc();
    volt += sergetc() << 8;
    return(volt);
}

// Implement if needed
/*
void leds(int val) {

    BusOut _leds(p20,p19,p18,p17,p16,p15,p14,p13);
    _leds = val;
}
*/

void play_sound(uint8_t length, unsigned char *play)
{
	uint8_t i=0;

    serputc(DO_PLAY);
    serputc(length);

	while(i< length) {
		serputc(play[i]);
		i++;
	}
} 

void locate(int x, int y) {
    serputc(DO_LCD_GOTO_XY);
    serputc(x);
    serputc(y);
}

void cls(void) {
    serputc(DO_CLEAR);
}

int print (char* text, int length) {
	int i;
    serputc(DO_PRINT);  
    serputc(length);       
    for (i = 0 ; i < length ; i++) {
        serputc(text[i]); 
    }
    return(0);
}

int _putc (int c) {
    serputc(DO_PRINT);  
    serputc(0x1);       
    serputc(c);         
    //wait (0.001);
    return(c);
}

int _getc (void) {
    char r = sergetc();
    return(r);
}

