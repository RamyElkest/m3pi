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
 #include "gyro.h"

   int gyr_status;
   char gyr_bytes[7];

// -----------------------------------------------
void gyr_init( void ) {

   // Check that you're talking with an L3G4200D device
   if( _gyrRegisterRead( GYR_WHO_AM_I ) == 0xd3 ) {
      gyr_status = 0;
   }
   else {
      gyr_status = 1;
      return;
   }
   
   // Enable normal mode... 
   _gyrRegisterWrite( GYR_CTRL_REG1, 0x0f );

}

// -----------------------------------------------
int _gyrGetStatus( void ) { return( gyr_status ); }

// -----------------------------------------------
int _gyrRegisterRead(  int reg ) {
  gyr_bytes[0] = reg & 0xff;
  gyr_status = _i2c_write( GYR_ADDRESS, gyr_bytes, 1 );
  if( gyr_status ==  0 ) {
    gyr_status = _i2c_read(  GYR_ADDRESS + 1, gyr_bytes, 1 );
    return( gyr_bytes[0] );
  }
  return( 0 );
}

// -----------------------------------------------  
void _gyrRegisterWrite( int reg, char data ) {
  gyr_bytes[0] = reg & 0xff;
  gyr_bytes[1] = data & 0xff;
  gyr_status = _i2c_write( GYR_ADDRESS, gyr_bytes, 2 );
}

// -----------------------------------------------  
void _gyrRead( short *output ) {
	int i;
   gyr_bytes[0] = GYR_OUT_X_L | (1<<7);
   gyr_status = _i2c_write( GYR_ADDRESS, gyr_bytes, 1 );
   if( gyr_status == 0 ) {
      gyr_status = _i2c_read( GYR_ADDRESS + 1, gyr_bytes, 6 );
      if( gyr_status == 0 ) {
         for( i=0; i<3; i++ ) {
            output[i] = (short) (gyr_bytes[2*i] | ( gyr_bytes[2*i+1] << 8 ));
         }
      }
   }
}

// -----------------------------------------------  
void _angularVelocity( float *output ) {
	int i;
   const float fs[] = { 250., 500., 2000., 2000. }; 
   float fullscale =  fs[ ( _gyrRegisterRead( GYR_CTRL_REG4 ) >> 4 ) & 0x3 ];
   short g[3]; _gyrRead( g );
   if( gyr_status == 0 ) {
      for( i=0; i<3; i++ ) {
         output[i] = (float) g[i] / 32768. * fullscale;
      }
   }
}


// -----------------------------------------------  
int _temperature( void ) {

   gyr_bytes[0] = GYR_OUT_TEMP;
   gyr_status = _i2c_write( GYR_ADDRESS, gyr_bytes, 1 );
   if( gyr_status == 0 ) {
      gyr_status = _i2c_read( GYR_ADDRESS + 1, gyr_bytes, 1 );
      if( gyr_status == 0 ) {
         return( (int) gyr_bytes[0] );
      }
   }
   return( 0 );
}
