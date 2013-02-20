/* mbed L3G4200D Library version 0beta1
 * Copyright (c) 2012 bengo
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

#ifndef GYRO_H
#define GYRO_H

#include "modules.h"

// Device registers addresses
// I2C address
#define GYR_ADDRESS 0xd2 

// register addresses
#define GYR_WHO_AM_I 0x0f
#define GYR_CTRL_REG1 0x20
#define GYR_CTRL_REG2 0x21
#define GYR_CTRL_REG3 0x22
#define GYR_CTRL_REG4 0x23
#define GYR_CTRL_REG5 0x24
#define GYR_REFERENCE 0x25
#define GYR_OUT_TEMP 0x26
#define GYR_STATUS_REG 0x27
#define GYR_OUT_X_L 0x28
#define GYR_OUT_X_H 0x29
#define GYR_OUT_Y_L 0x2a
#define GYR_OUT_Y_H 0x2b
#define GYR_OUT_Z_L 0x2c
#define GYR_OUT_Z_H 0x2d
#define GYR_FIFO_CTRL_REG 0x2e
#define GYR_FIFO_SRC_REG 0x2f
#define GYR_INT1_CFG 0x30
#define GYR_INT1_SRC 0x31
#define GYR_INT1_THS_XH 0x32
#define GYR_INT1_THS_XL 0x33
#define GYR_INT1_THS_YH 0x34
#define GYR_INT1_THS_YL 0x35
#define GYR_INT1_THS_ZH 0x36
#define GYR_INT1_THS_ZL 0x37
#define GYR_INT1_DURATION 0x38


   /**
    * Create an L3G4200D object connected to the specified I2C pins
    * @param sda I2C SDA pin
    * @param scl I2C SCL pin
    */
   void gyr_init( void );
  
   /**
    * Return status code of prevoius function call
    */
   int _gyrGetStatus( void ); 
   
   /**
    * Read specified register content
    * @param reg register address
    */  
   int _gyrRegisterRead(  int reg );

   /**
    * Write to specified register
    * @param reg register address
    * @parma data data to be written
    */     
   void _gyrRegisterWrite( int reg, char data );
  
   /**
    * read gyroscope vector
    */
   void _gyrRead( short *output );

   /**
    * Read angular velogity (in degrees per second)
    */
   void _angularVelocity( float *output );
   
   /**
    * Read temperature (in celsius)
    */
   int _temperature( void );  

#endif //  L3G4200D_h
