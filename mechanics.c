/*	mechanics.c
 *
 */

#include "debug_frmwrk.h"
#include "LPC17xx.h"
#include "modules.h"
#include "m3pi.h"
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
 *	@param uint8_t indicating desired group of sensors, 0 - Middle sensor, 1 - Right Sensors, 2 - Left Sensors
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


/* Follow Wall PID
 *
 *
 * @param channelA, ADC channel to read first sensor
 * @param channelB, ADC channel to read second sensor
 */

void newWallFollow(uint8_t whichSensors)
{
	int16_t proportional,		// P
			last_proportional=0,// Previous proporitonal value
			derivative,			// D
			power_difference,
			max = 20,			// Maximum speed
			last_pd=0;			// Previous power_derivative
	int32_t integral=0;			// I
	int64_t temp = 0;			// Temp 64 bit
	int16_t sensors[2], 		// Array to hold sensor values
			 i;
	uint16_t count=0;
	float y, kapa;
	const uint8_t y_d = 20;	// 20 cms from the wall

	// Loop until stopped by external interrupt
	while(1)
	{		
		switch(whichSensors) {
		case RIGHT_SENSORS:
			sensors[0] = read_analog(RIGHT_F_ANALOG);
			sensors[1] = read_analog(RIGHT_B_ANALOG);
			break;
		case LEFT_SENSORS:
			sensors[0] = read_analog(LEFT_F_ANALOG);
			sensors[1] = read_analog(LEFT_B_ANALOG);
			break;
		}

		y = (sensors[0] + sensors[1]) / 2;
		
		// Calculate Kapa
		// K = -k_y k_@ (y - y_d) - k_@ @
		kapa = (y_d - y) * k_y * k_theta;
		kapa -= k_theta * getAngleFromSensors(whichSensors);
		
		proportional = kapa;

		//proportional = (sensors[0] * sensors[1]) - ((int16_t)2000 * sensors[0]) - (sensors[1] - (int16_t)2000);

		// Compute the derivative (change) and integral (sum) of the
		// position.
		derivative = proportional - last_proportional;
		integral += proportional/1000;

		// Remember the last position.
		last_proportional = proportional;

		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the right.  If it is a negative number, the robot will
		// turn to the left, and the magnitude of the number determines
		// the sharpness of the turn.
		power_difference = proportional/10 /* + integral/1000 + derivative*1/2*/;


		if(count == 1000)
		{
		_DBG("PID: ");
		_DBD16(sensors[0]);
		_DBG(" ");
		_DBD16(sensors[1]);
		_DBG(" ");
		_DBD16(proportional);
		_DBG(" ");
		_DBD16(derivative);
		_DBG(" ");
		_DBD16(integral);
		_DBG(" ");
		_DBD16(power_difference);
		_DBG_(" ");
		count=0;
		}

		// Compute the actual motor settings.  We never set either motor


		// to a negative value.
		if(power_difference > max)
			power_difference = max;
		if(power_difference < -max)
			power_difference = -max;

		if(power_difference < 0) {
			right_motor(max+power_difference);
			left_motor(max);
		} else {
			right_motor(max);
			left_motor(max-power_difference);
		}

		last_pd = power_difference;
		count++;
	}
	stop();
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
 void  cornerise()
 {
 	turn_right(180);
	while(getDist(read_analog(MIDDLE_SENSOR)) < 10) backward(10);
	while(getDist(read_analog(MIDDLE_SENSOR)) > 10) forward(10);
	stop();
	turn_right(90);
	while(getDist(read_analog(MIDDLE_SENSOR)) < 10) backward(10);
	while(getDist(read_analog(MIDDLE_SENSOR)) > 10) forward(10);
	stop();
	turn_right(90);
}
  
  // Given a map, inital and goal state, go.
  // Given a map, traverse the map
  // Assumes in corner
void traverseMap (int8_t **map, location* state)
{
	// cornerise
	cornerise();
	
	int8_t x = state->x;
	int8_t y = state->y;
	
	wallFollow();
	
}
  
  
  
  
  
  
