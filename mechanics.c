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
uint8_t getDist2(uint16_t sensor_value)
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

float getDist(uint16_t sensor_value)
{
	// y = 1.8 * z^4 - 11 * z^3 + 21 * z^2 - 23 * z + 27
	// z = (x - 1100)/8400
	static const float a = 1.8;
	static const uint8_t b = 11, c = 21, d = 23, e = 27;	
	float z = (sensor_value - 1100.) / 840.;
	
	float ans = (a * pow(z,4)) - (b * pow(z,3)) + (c * pow(z,2)) - (d * z) + e;

	return (ans < 0) ? 0 : ans;
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

/**
 *	WHEEL ENCODER CONTROLS
 */

// Turn right wheel a number of ticks
void tick_right_wheel(uint8_t numberOfTicks)
{
	uint64_t currentWheelTicks = right_wheel_enc.ticks;
	right_motor(DEFAULT_SPEED);
	while(right_wheel_enc.ticks < currentWheelTicks + numberOfTicks);
	stop();
}

// Turn left wheel a number of ticks
void tick_left_wheel(uint8_t numberOfTicks)
{
	uint64_t currentWheelTicks = left_wheel_enc.ticks;
	left_motor(DEFAULT_SPEED);
	while(left_wheel_enc.ticks < currentWheelTicks + numberOfTicks);
	stop();
}

// Turn left wheel a number of ticks
void tick_wheels(uint8_t numberOfTicks)
{
	uint64_t currentRightWheelTicks = right_wheel_enc.ticks;
	uint64_t currentLeftWheelTicks = left_wheel_enc.ticks;
	forward(DEFAULT_SPEED);
	while((left_wheel_enc.ticks < currentLeftWheelTicks + numberOfTicks) && 
		  (right_wheel_enc.ticks < currentRightWheelTicks + numberOfTicks));
	stop();
}

void turn_right_wheel(uint8_t numberOfTurns)
{
	tick_right_wheel(TOTAL_TICKS);
}

void turn_left_wheel(uint8_t numberOfTurns)
{
	tick_left_wheel(TOTAL_TICKS);
}

/**
 *	1. Check if total ticks has been covered
 *	2. Read Analog Sensors
 *	 i. Check for obstacles (dist < 5cm)
 *	ii. Check if right/left in range, if in range:
 *	  a. Get Angle and correct compass angle
 *	  b. Get Dist and correct state error / map
 */

void turn_wheels(uint8_t numberOfTurns)
{
	tick_wheels(TOTAL_TICKS);
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
			max = 30,			// Maximum speed
			last_pd=0;			// Previous power_derivative
	int32_t integral=0;			// I
	int64_t temp = 0;			// Temp 64 bit
	uint16_t count=0,
			 i;
	float y, kapa,
		  sensors[2]; 		// Array to hold sensor values
	const uint8_t y_d = 20;	// 20 cms from the wall

	// Loop until stopped by external interrupt
	while(getDist(read_analog(FRONT_ANALOG)) > WHEEL_CIRCUM)
	{
		switch(whichSensors) {
		case RIGHT_SENSORS:
			sensors[0] = getDist(read_analog(RIGHT_F_ANALOG));
			sensors[1] = getDist(read_analog(RIGHT_B_ANALOG));
			break;
		case LEFT_SENSORS:
			sensors[0] = getDist(read_analog(LEFT_F_ANALOG));
			sensors[1] = getDist(read_analog(LEFT_B_ANALOG));
			break;
		}

		y = (sensors[0] + sensors[1]) / 2;
		
		// Calculate Kapa
		// K = -k_y k_@ (y - y_d) - k_@ @
		kapa = (y_d - y) * -k_y * k_theta;
		kapa += k_theta * getAngleFromSensors(whichSensors);
		
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
		power_difference = proportional/4 /* + integral/1000 + derivative*1/2*/;


		if(count == 10)
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
			left_motor(max+power_difference);
			right_motor(max);
		} else {
			left_motor(max);
			right_motor(max-power_difference);
		}

		last_pd = power_difference;
		count++;
	}
	stop();
}

/* Follow Wall PID
 *
 *
 * @param channelA, ADC channel to read first sensor
 * @param channelB, ADC channel to read second sensor
 */

void mehWallFollow(uint8_t whichSensors, uint8_t distance)
{
	int16_t proportional,		// P
			last_proportional=0,// Previous proporitonal value
			derivative,			// D
			power_difference,
			max = 50,			// Maximum speed
			last_pd=0;			// Previous power_derivative
	int32_t integral=0;			// I
	int64_t temp = 0,			// Temp 64 bit
			dist = pow(distance,2);
	float sensors[2]; 		// Array to hold sensor values
	uint16_t count=0,
			 i;

	// Loop until stopped by external interrupt
	while(getDist(read_analog(FRONT_ANALOG)) > WHEEL_CIRCUM)
	{
		switch(whichSensors) {
		case RIGHT_SENSORS:
			sensors[0] = getDist(read_analog(RIGHT_F_ANALOG));
			sensors[1] = getDist(read_analog(RIGHT_B_ANALOG));
			break;
		case LEFT_SENSORS:
			sensors[0] = getDist(read_analog(LEFT_F_ANALOG));
			sensors[1] = getDist(read_analog(LEFT_B_ANALOG));
			break;
		}

		temp = (sensors[0] * sensors[1]) - (int64_t)dist;
		proportional = temp;

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
		power_difference = proportional/1000 + integral/1000 + derivative*1/2;


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

//cornerise(){};
/*
void getMap(uint32_t *raw_map) {
	// Wall Follow
	// Update Raw Map
	// Turn

	
}




void mapping() {
	uint32_t raw_map[500], x, y;
	uint8_t * my_map;
	char tmp[8];
	cornerise();
	getMap(raw_map);
	initMap(raw_map, my_map, &x, &y);
	LocalFileSystem_open("myMap.txt", O_WRONLY);
	sprintf(tmp, "%d %d\r\n", x, y);
	LocalFileHandle_write(tmp, (int)strlen(tmp));
	LocalFileHandle_write(my_map, (int)(x * y));
}

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
  */
  
  
  

