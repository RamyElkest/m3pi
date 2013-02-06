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
 
#include "accmag.h"
#include "lpc_types.h"
#include <math.h>


   int accmag_status;
   int accmag_SA0Pad;
   char accmag_bytes[7];

// -------------------------------------------
void accmag_init( void ){
   // Get SA0 pin status
   accmag_bytes[0] = ACC_CTRL_REG1;
   _i2c_write( ACC_ADDRESS, accmag_bytes, 1 );
   int sa0low = _i2c_read( ACC_ADDRESS+1, accmag_bytes, 1 );
   accmag_bytes[0] = ACC_CTRL_REG1;
   _i2c_write( ACC_ADDRESS+2, accmag_bytes, 1 );
   int sa0hig = _i2c_read( ACC_ADDRESS+2+1, accmag_bytes, 1 );
   if( sa0low == 0 && sa0hig != 0 ) {
      accmag_SA0Pad = 0x0;
   }
   else if( sa0low != 0 && sa0hig == 0 ) {
      accmag_SA0Pad = 0x2;
   }
   else {
      accmag_status = 1;
      return;
   }
   // Check that you're talking with an LM303DLM device
   accmag_bytes[0] = MAG_WHO_AM_I;
   _i2c_write( MAG_ADDRESS, accmag_bytes, 1 );
   accmag_status = _i2c_read( MAG_ADDRESS+1, accmag_bytes, 1 );   
   if( accmag_bytes[0] == 0x3c ) {
      accmag_status = 0;
   }
   else {
      accmag_status = 1;
      return;
   }
   // Enable normal mode... 
   // ... On accelerometer
   _accRegisterWrite( ACC_CTRL_REG1, 0x27 );
   if( accmag_status != 0 ) {
      return;  
   }
   // ... And on magnetometer
   _magRegisterWrite( MAG_MR_REG, 0x00 );
}

// -------------------------------------------
int _accmagGetStatus( void ) { return( accmag_status ); }

// -------------------------------------------
int _accRegisterRead( int reg ) {
  accmag_bytes[0] = reg & 0xff;
  accmag_status = _i2c_write( ACC_ADDRESS + accmag_SA0Pad, accmag_bytes, 1 );
  if( accmag_status ==  0 ) {
    accmag_status = _i2c_read(  ACC_ADDRESS + accmag_SA0Pad + 1, accmag_bytes, 1 );
    return( accmag_bytes[0] );
  }
  return( 0 );
}

// -------------------------------------------  
void _accRegisterWrite( int reg, char data ) {
  accmag_bytes[0] = reg & 0xff;
  accmag_bytes[1] = data & 0xff;
  accmag_status = _i2c_write( ACC_ADDRESS + accmag_SA0Pad, accmag_bytes, 2 );
}

// -------------------------------------------  
int _magRegisterRead( int reg ) {
  accmag_bytes[0] = reg & 0xff;
  accmag_status = _i2c_write( MAG_ADDRESS, accmag_bytes, 1 );
  if( accmag_status ==  0 ) {
    accmag_status = _i2c_read(  MAG_ADDRESS + 1, accmag_bytes, 1 );
    return( accmag_bytes[0] );
  }
  return( 0 );
}

// -------------------------------------------  
void _magRegisterWrite( int reg, char data ) {
  accmag_bytes[0] = reg & 0xff;
  accmag_bytes[1] = data & 0xff;
  accmag_status = _i2c_write( MAG_ADDRESS, accmag_bytes, 2 );
}


// -------------------------------------------
void _accRead( short *output ) {
	int i;
   accmag_bytes[0] = ACC_OUT_X_L | (1<<7);
   accmag_status = _i2c_write( ACC_ADDRESS + accmag_SA0Pad, accmag_bytes, 1 );
   if( accmag_status == 0 ) {
      accmag_status = _i2c_read( ACC_ADDRESS + accmag_SA0Pad + 1, accmag_bytes, 6 );
      if( accmag_status == 0 ) {
         for( i=0; i<3; i++ ) {
            output[i] = ( (short) accmag_bytes[2*i] | (short) accmag_bytes[2*i+1] << 8 );
         }
      }
   } 
}

// -------------------------------------------
void _acceleration( float *output ) {

	int i;
   const float cal[3][2] = { {  16291.5, -16245.4 }, {  16819.0, -16253.0 }, {  16994.8, -15525.6 } };
   
   int fs = ( _accRegisterRead( ACC_CTRL_REG4 ) >> 4 ) & 0x3;
   short a[3]; _accRead( a );
   if( accmag_status == 0 ) {
      for( i=0; i<3; i++ ) {
         output[i] = output[i] * ( (cal[i][0] - cal[i][1]) / 32768. ) + (cal[i][0]+cal[i][1])/2.;
         output[i] = (float) a[i]  * pow(2.,(fs+1)) / 32768.;
      }
   }
}  

// -------------------------------------------
void _magRead( short *output ) {
   accmag_bytes[0] = MAG_OUT_X_H;
   accmag_status = _i2c_write( MAG_ADDRESS, accmag_bytes, 1 );
   if( accmag_status == 0 ) {
      accmag_status = _i2c_read( MAG_ADDRESS + 1, accmag_bytes, 6 );
      if( accmag_status == 0 ) {
         output[0] = (short) accmag_bytes[0] << 8 | (short) accmag_bytes[1];
         output[1] = (short) accmag_bytes[4] << 8 | (short) accmag_bytes[5];
         output[2] = (short) accmag_bytes[2] << 8 | (short) accmag_bytes[3];
      }
   } 
}
 
// ------------------------------------------- 
void _magneticField( float *output ) {

   float gainxy[] = { 1100., 855., 670., 450., 400., 330., 230. };
   float gainz[]  = {  980., 760., 600., 400., 355., 295., 205. };
   
   int gn = ( _magRegisterRead( MAG_CRB_REG ) >> 5 ) & 0x7;
   short m[3]; _magRead( m );
   if( accmag_status == 0 ) {
      output[0] = (float) m[0] / gainxy[gn-1];
      output[1] = (float) m[1] / gainxy[gn-1];
      output[2] = (float) m[2] / gainz[gn-1];
   }
}
