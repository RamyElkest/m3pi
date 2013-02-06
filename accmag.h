/* mbed LSM303 Library version 0beta1
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

#ifndef ACCMAG_H
#define ACCMAG_H

#include "modules.h"

// Device registers addresses
// I2C addresses
#define ACC_ADDRESS 0x30
#define MAG_ADDRESS 0x3c

// register addresses
#define ACC_CTRL_REG1 0x20
#define ACC_CTRL_REG2 0x21
#define ACC_CTRL_REC3 0x22
#define ACC_CTRL_REG4 0x23
#define ACC_CTRL_REG5 0x24
#define ACC_HP_FILTER_RESET 0x25
#define ACC_REFERENCE 0x26
#define ACCaccmag_status_REG 0x27
#define ACC_OUT_X_L 0x28
#define ACC_OUT_X_H 0x29
#define ACC_OUT_Y_L 0x2a
#define ACC_OUT_Y_H 0x2b
#define ACC_OUT_Z_L 0x2c
#define ACC_OUT_Z_H 0x2d
#define ACC_INT1_CFG 0x30
#define ACC_INT1_SOURCE 0x31
#define ACC_INT1_THS 0x32
#define ACC_INT1_DURATION 0x33
#define ACC_INT2_CFG 0x34
#define ACC_INT2_SOURCE 0x35
#define ACC_INT2_THS 0x36
#define ACC_INT2_DURATION 0x37
#define MAG_CRA_REG 0x00
#define MAG_CRB_REG 0x01
#define MAG_MR_REG 0x02
#define MAG_OUT_X_H 0x03
#define MAG_OUT_X_L 0x04
#define MAG_OUT_Y_H 0x07
#define MAG_OUT_Y_L 0x08
#define MAG_OUT_Z_H 0x05
#define MAG_OUT_Z_L 0x6
#define MAG_SR_REG 0x9
#define MAG_IRA_REG 0xa
#define MAG_IRB_REG 0xb
#define MAG_IRC_REG 0xc
#define MAG_WHO_AM_I 0xf


   /**
    * Create an LSM303 object connected to the specified I2C pins
    * @param sda I2C SDA pin
    * @param scl I2C SCL pin
    */  
   void accmag_init( void );

   /**
    * Return status code of prevoius function call
    */  
   int _accmagGetStatus( void );
  
   /**
    * Read specified accelerometer register content
    * @param reg register address
    */ 
   int _accRegisterRead( int reg );

   /**
    * Write to specified accelerometer register
    * @param reg register address
    * @parma data data to be written
    */  
   void _accRegisterWrite( int reg, char data );

   /**
    * Read specified magnetometer register content
    * @param reg register address
    */   
   int _magRegisterRead( int reg );

   /**
    * Write to specified magnetometer register
    * @param reg register address
    * @parma data data to be written
    */   
   void _magRegisterWrite( int reg, char data );
  
   /**
    * Read accelerometer vector
    */
   void _accRead( short *output );
   
    /**
    * Read acceleration 
    */
   void _acceleration( float *output );

   /**
    * Read magnetometer vector
    */
   void _magRead( short *output );
   
   /**
    * Read magnetic field vector
    */
    void _magneticField( float *output );

#endif // LSM303_h
