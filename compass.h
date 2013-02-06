/* mbed Minimu_9 Library version 0beta1
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

#ifndef COMPASS_H
#define COMPASS_H

#include "modules.h"
#include "gyro.h"
#include "accmag.h"

/** Minimu-9 accelerometer, magnetometer, gyroscope sensor controller library
 *
 * Example:
 * @code
 * #include "mbed.h"
 * #include <minimu9.h>
 * #include <vector>
 * 
 * Serial pc(USBTX, USBRX); // tx, rx
 * 
 * int main() {
 *     
 *    minimu9 minimu( p28, p27 );
 *    
 *    int status = minimu.getStatus();
 *    if( status == 0 ) {
 *       for( int i=0; i<100000; i++ ) {
 *          std::vector<int> data = minimu.read();
 *          if( status == 0 ) {    
 *             pc.printf("\x1b[2J\x1b[f\x1b[33mLSM303 minimu read status: \x1b[32m%d\n\r", minimu.getStatus() );
 *          }
 *          else {
 *             pc.printf("\x1b[2J\x1b[f\x1b[33mLSM303 minimu read status: \x1b[31m%d\n\r", minimu.getStatus() );
 *          }
 *          
 *          pc.printf("\x1b[33mLSM303   acc: \x1b[37m%8d %8d %8d\n\r", (short)data[0], (short)data[1], (short)data[2] );
 *          pc.printf("\x1b[33mLSM303   mag: \x1b[37m%8d %8d %8d\n\r", (short)data[3], (short)data[4], (short)data[5] );
 *          pc.printf("\x1b[33mL3G4200D gyr: \x1b[37m%8d %8d %8d\n\r", (short)data[6], (short)data[7], (short)data[8] );
 *          wait( 1 );
 *       }
 *    }
 *    return( 0 );
 * }
 * @endcode
*/


   /**
    * Create a minimu9 object connected to the specified I2C pins
    * @param sda I2C SDA pin
    * @param scl I2C SCL pin
    */
   void compass_init( void );
   
   /**
    * Return status code of prevoius function call
    */
   int compass_getStatus( void );

   /**
    * Read all three sensors vectors (accelerometer, magnetometer, gyroscope)
    */
   short* compass_read( void );
   
    /**
     * Read accelerometer vector
     */
   void compass_accRead( short *output );
   
   /**
    * Read acceleration (in g units)
    */
   void compass_acceleration( float *output );
   
   /**
    * Read magnetometer vector
    */
   void compass_magRead( short *output );
   
   /**
    * Read magnetic filed (in gauss)
    */
   void compass_magneticField( float *output );
   
   /**
    * Read gyroscope vector
    */
   void compass_gyrRead( short *output );
   
   /**
    * Read angular velogity (in degrees per second)
    */
   void compass_angularVelocity( float *output );
   
    /**
    * Read temperature (in celsius)
    */
   int compass_temperature( void );

#endif // minimu9_h
