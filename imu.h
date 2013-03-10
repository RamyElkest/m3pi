#ifndef IMU__H
#define IMU__H

/*
How to use this module in other projects.

Input variables are: 
        adcAvg[0..6]  ADC readings of 3 axis accelerometer and 3 axis gyroscope (they are calculated in the background by adcutil.h)
        interval_us - interval in microseconds since last call to imu_update

Output variables are:
        DcmEst[0..2] which are the direction cosine of the X,Y,Z axis

First you must initialize the module with: 
        imu_init();

Then call periodically every 5-20ms:
        imu_update(interval_us);
        it is assumed that you also update periodicall the adcAvg[0..5] array 

*/

#define ACC_WEIGHT_MAX 0.02                     //maximum accelerometer weight in accelerometer-gyro fusion formula
                                                                        //this value is tuned-up experimentally: if you get too much noise - decrease it
                                                                        //if you get a delayed response of the filtered values - increase it
                                                                        //starting with a value of  0.01 .. 0.05 will work for most sensors

#define ACC_ERR_MAX  0.3                        //maximum allowable error(external acceleration) where accWeight becomes 0
#define count250us (SystemCoreClock/4000)		// 250 us from System core clock

extern float adcAvg[3];							//get values from compass

//-------------------------------------------------------------------
//Get accelerometer reading (accelration expressed in g)
//-------------------------------------------------------------------
float getAcclOutput(unsigned char);

//-------------------------------------------------------------------
//Get gyro reading (rate of rotation expressed in deg/ms) 
//-------------------------------------------------------------------
float getGyroOutput(unsigned char);

//bring dcm matrix in order - adjust values to make orthonormal (or at least closer to orthonormal)
//Note: dcm and dcmResult can be the same
//void dcm_orthonormalize(float[][]);


//rotate DCM matrix by a small rotation given by angular rotation vector w
//see http://gentlenav.googlecode.com/files/DCMDraft2.pdf
//void dcm_rotate(float[][], float[]);


//-------------------------------------------------------------------
// imu_init
//-------------------------------------------------------------------
void imu_init();



//-------------------------------------------------------------------
// imu_update
//-------------------------------------------------------------------
#define ACC_WEIGHT 0.01         //accelerometer data weight relative to gyro's weight of 1
#define MAG_WEIGHT 0.0          //magnetometer data weight relative to gyro's weight of 1

void imu_update();
        
#endif
