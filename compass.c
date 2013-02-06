/* mbed Minimu-9 Library version 0beta1
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

#include "compass.h"

int compass_status;   

// -------------------------------------------  
void compass_init( void ) {
   i2c_init();
   gyr_init();
   accmag_init();
   compass_status = _accmagGetStatus() * _gyrGetStatus();
}

// -------------------------------------------
short* compass_read( void ) {
   short data[] = {0,0,0,0,0,0,0,0,0},
		 acc[] = {0,0,0},
		 mag[] = {0,0,0},
		 gyr[] = {0,0,0},
		 i;
   _accRead( acc );
   compass_status = _accmagGetStatus();
   if( compass_status == 0 ) {
      _magRead( mag );
      compass_status = _accmagGetStatus();
      if( compass_status == 0 ) {
         _gyrRead( gyr );
         compass_status = _gyrGetStatus();
         if( compass_status == 0 ) {
            for( i=0; i<3; i++ ) {
               data[i  ] = acc[i];
               data[i+3] = mag[i];
               data[i+6] = gyr[i];
            }
         }
      }
   } 
   return( data );
}

// -------------------------------------------
int compass_getStatus( void ) { return( compass_status ); }

// -------------------------------------------
void compass_accRead( short *output ) { return( _accRead( output ) ); }

// -------------------------------------------
void compass_acceleration( float *output ) { return( _acceleration( output ) ); } 

// -------------------------------------------
void compass_magRead( short *output ) { return( _magRead( output ) ); }

// -------------------------------------------
void compass_magneticField( float *output ) { return( _magneticField( output ) ); }

// -------------------------------------------
void compass_gyrRead( short *output ) { return( _gyrRead( output ) ); }

// -------------------------------------------
void compass_angularVelocity( float *output ) { return( _angularVelocity( output ) ); }

// -------------------------------------------
int compass_temperature( void ) { return( _temperature() ); }




