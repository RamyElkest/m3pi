/*	mechanics.c
 *
 */

#include "debug_frmwrk.h"
#include "LPC17xx.h"
//#include "m3pi.h"
#include "modules.h"
#include "mechanics.h"
#include <math.h>


uint16_t sensor_lut[CALIB_NUM];


/**
 *	Generates sensors' look up table
 */
int generateLUT( void )
{	
	uint8_t i;
	for(i=0;i<CALIB_NUM;i++)
	{
		// Go to next line and Align
		nextLine();
		// Sleep for 1 second
		sleep(1);
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
	uint8_t const MAX_DIST = CALIB_NUM * CALIB_RES; // Capped at 80 since sensor does not go beyond	

	if(sensor_value < 200) return 0;	// Minimum threshold
	
	uint16_t y0,y1,x0,x1,x;
	uint8_t i;

	// Find upper index of sensor_value
	for(i=1;i<CALIB_NUM;i++) if(sensor_value < sensor_lut[i]) break;

	// Values use i+1, therefore upper value as lower value
	x = sensor_value;
	y0 = MAX_DIST - ((i-1) * CALIB_RES); y1 = MAX_DIST - (i * CALIB_RES);
	x0 = sensor_lut[i-1]; x1 = sensor_lut[i];
	
 	return ((x-x0)*(y1-y0))/(x1-x0)+ y0;
}

/**
 *	Return the angle from teh difference between the sensors
 *	
 *	@param uint8_t indicating desired group of sensors, 0 - Middle sensor, 1 - Right Sensor, 2 - Left Sensor 
 */
float getAngleFromSensors(uint8_t whichSensors)
{
	float angle = 0,
			difference;
	switch(whichSensors)
	{
		case RIGHT_SENSORS:
			difference = getDist(read_analog(RIGHT_F_ANALOG)) - getDist(read_analog(RIGHT_B_ANALOG));
			angle = atan2(difference, (float)RIGHT_DIFFERENCE) * 180 / PI;
			break;

		case LEFT_SENSORS:
			difference = getDist(read_analog(LEFT_F_ANALOG)) - getDist(read_analog(LEFT_B_ANALOG));
			angle = atan2(difference, (float)LEFT_DIFFERENCE) * 180 / PI;
			break;
	}
	return angle;
}



/**
 *	Kinemetic equations
 *	We want to yank out .y. (y dot) and .@. (Theta dot)
 *
 * Key equations:
 * .y. = V@
 * .@. = V K_@ K_y (y_d - y) - K_@ V @
 * 
 *
 * Variables:
 * V : Speed
 * @ : Current orientation
 * @_d : Demand orientation
 * y : current offset (from the wall)
 * y_d : Demand offset (from the wall)
 * K_y : constant
 * K_@: constant
 * 
 */

/**
 *	Different Kinemetic equation
 *	
 * Key Equations
 * .x. = R/2 (v_r + v_l) cos@ 
 * .y. = R/2 (v_r + v_l) sin@
 * .@. = R/L (v_r - v_l)
 *
 * v_r = (2v + wL) / 2R
 * v_l = (2v - wL) / 2R
 *
 * .x. = v cos@
 * .y. = v sin@
 * .@. = w
 *
 * Variables
 * R : Wheels radius
 * L : Distance between the wheels
 * v_r : Right wheel velocity
 * v_l : Left wheel velocity
 * w : angular velocity (omega)
 */
 
 /**
  * Dynamics: new state from previous state
  *
  * key equations:
  * x' = x + D_c cos@
  * y' = y + D_c sin@
  * @' = @ + (D_r - D_l) / L
  * 
  * Variables:
  * D_l : Left wheel ticks
  * D_r : Right wheel ticks
  * D_c : centre (D_r + D_l) / 2
  * D = 2 pi R (delta ticks) / N
  */
