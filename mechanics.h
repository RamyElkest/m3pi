/*	mechanics.h
 *
 *	
 */

#ifndef MECHANICS_H
#define MECHANICS_H

#include "lpc_types.h"

#define PI 3.14159265	// Pi

#define MIDDLE_SENSOR 0 // Middle sensor
#define RIGHT_SENSORS 1 // Right sensors
#define LEFT_SENSORS 2 // Left Sensors
#define FRONT_ANALOG 5 // Analog sensor in the front
#define LEFT_F_ANALOG 2 // Analog in the front right
#define LEFT_B_ANALOG 0 // Analog in the back right
#define RIGHT_F_ANALOG 1 // Analog in the back right
#define RIGHT_B_ANALOG 4 // Analog in the back left
#define LEFT_DIFFERENCE 8 // Distance between left sensors in cm
#define RIGHT_DIFFERENCE 8.5 // Distance between right sensors in cm
#define avg(x,y) {(x+y)/2}	// Average macro

#define CALIB_NUM 4 // Number of calibration points
#define CALIB_RES 10 // Number of centimeters between calibration points

#define k_y 0.5	//  K_y used in wall following to calculate kapa
#define k_theta 2.0	// K_@ used in wall following to calculate kapa, Has to be 4 * k_y

struct wheel_enc {
	uint8_t direction;	// Direction of the wheel. 0 - Forwards, 1 - backwards
	int64_t ticks;	// Number of ticks per wheel
};

struct wheel_enc left_wheel_enc;
struct wheel_enc right_wheel_enc;

/**
 *	Generates sensors' look up table
 */
int generateLUT( void );

/**
 *	Linear Interpolation using sensor lut and return cm from sensor values
 *
 *	y = (x-x_0)(y_1-y_0)/(x_1-x_0) + y_0
 *	y0, 
 **/
uint8_t getDist(uint16_t sensor_value);

/**
 *	Return the angle from teh difference between the sensors
 *	
 *	@param uint8_t indicating desired group of sensors, 0 - Middle sensor, 1 - Right Sensor, 2 - Left Sensor 
 */
float getAngleFromSensors(uint8_t whichSensors);


#endif //MECHANICS_H
