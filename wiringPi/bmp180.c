/*
 * bmp180.c:
 *	Extend wiringPi with the BMP180 I2C Pressure and Temperature
 *	sensor. This is used in the Pi Weather Station
 *	Copyright (c) 2016 Gordon Henderson
 *
 *	Information from the document held at:
 *		http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
 *	was very useful when building this code.
 *
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"

#include "bmp180.h"

#undef	DEBUG

#define	I2C_ADDRESS	0x77
#define	BMP180_OSS	   0


// Static calibration data
//	The down-side of this is that there can only be one BMP180 in
//	a system - which is practice isn't an issue as it's I2C
//	address is fixed.

static  int16_t AC1, AC2, AC3 ;
static uint16_t AC4, AC5, AC6 ;
static  int16_t VB1, VB2 ;
static  int16_t  MB,  MC, MD ;

static double c5, c6, mc, md, x0, x1, x2, yy0, yy1, yy2, p0, p1, p2 ;

// Pressure & Temp variables

uint32_t cPress, cTemp ;

static int altitude ;

/*
 * read16:
 *	Quick hack to read the 16-bit data with the correct endian
 *********************************************************************************
 */

uint16_t read16 (int fd, int reg)
{
  return (wiringPiI2CReadReg8 (fd, reg) <<  8) | wiringPiI2CReadReg8 (fd, reg + 1) ;

}


/*
 * bmp180ReadTempPress:
 *	Does the hard work of reading the sensor
 *********************************************************************************
 */

static void bmp180ReadTempPress (int fd)
{
  double fTemp, fPress ;
  double tu, a ;
  double pu, s, x, y, z ;

  uint8_t data [4] ;

// Start a temperature sensor reading

  wiringPiI2CWriteReg8 (fd, 0xF4, 0x2E) ;
  delay (5) ;

// Read the raw data

  data [0] = wiringPiI2CReadReg8 (fd, 0xF6) ;
  data [1] = wiringPiI2CReadReg8 (fd, 0xF7) ;

// And calculate...

  tu = (data [0] * 256.0) + data [1] ;

  a = c5 * (tu - c6) ;
  fTemp = a + (mc / (a + md)) ;
  cTemp = (int)(10.0 * fTemp + 0.55) ;

#ifdef	DEBUG
  printf ("fTemp: %f, cTemp: %6d\n", fTemp, cTemp) ;
#endif

// Start a pressure snsor reading

  wiringPiI2CWriteReg8 (fd, 0xF4, 0x34 | (BMP180_OSS << 6)) ;
  delay (5) ;

// Read the raw data

  data [0] = wiringPiI2CReadReg8 (fd, 0xF6) ;
  data [1] = wiringPiI2CReadReg8 (fd, 0xF7) ;
  data [2] = wiringPiI2CReadReg8 (fd, 0xF8) ;

// And calculate...

  pu = ((double)data [0] * 256.0) + (double)data [1] + ((double)data [2] / 256.0) ;
  s = fTemp - 25.0 ;
  x = (x2 * s * s) + (x1 * s) + x0 ;
  y = (yy2 * s * s) + (yy1 * s) + yy0 ;
  z = (pu - x) / y ;
  fPress = (p2 * z * z) + (p1 * z) + p0 ;
  cPress = (int)(10.0 * fPress + 0.55);

#ifdef	DEBUG
  printf ("fPress: %f, cPress: %6d\n", fPress, cPress) ;
#endif
}


/*
 * myAnalogWrite:
 *	Write to a fake register to represent the height above sea level
 *	so that the peudo millibar register can read the pressure in mB
 *********************************************************************************
 */

static void myAnalogWrite (struct wiringPiNodeStruct *node, int pin, int value)
{
  int chan = pin - node->pinBase ;

  if (chan == 0)
    altitude = value ;
}

/*
 * myAnalogRead:
 *********************************************************************************
 */

static int myAnalogRead (struct wiringPiNodeStruct *node, int pin)
{
  double compensate = 1.0 + (0.02 * altitude / 170);
  int chan = pin - node->pinBase ;

  bmp180ReadTempPress (node->fd) ;

  /**/ if (chan == 0)	// Read Temperature
    return cTemp ;
  else if (chan == 1)	// Measured Pressure
    return cPress ;
  else if (chan == 2)	// Estimated Pressure at sea level (drops to 98% at 170m)
    return (int)(compensate * (double)cPress + 0.5);
  else
    return -9999 ;

}


/*
 * bmp180Setup:
 *	Create a new instance of a PCF8591 I2C GPIO interface. We know it
 *	has 4 pins, (4 analog inputs and 1 analog output which we'll shadow
 *	input 0) so all we need to know here is the I2C address and the
 *	user-defined pin base.
 *********************************************************************************
 */

int bmp180Setup (const int pinBase)
{
  double c3, c4, b1 ;
  int fd ;
  struct wiringPiNodeStruct *node ;

  if ((fd = wiringPiI2CSetup (I2C_ADDRESS)) < 0)
    return FALSE ;

  node = wiringPiNewNode (pinBase, 4) ;

  node->fd          = fd ;
  node->analogRead  = myAnalogRead ;
  node->analogWrite = myAnalogWrite ;

// Read calibration data

  AC1 = read16 (fd, 0xAA) ;
  AC2 = read16 (fd, 0xAC) ;
  AC3 = read16 (fd, 0xAE) ;
  AC4 = read16 (fd, 0xB0) ;
  AC5 = read16 (fd, 0xB2) ;
  AC6 = read16 (fd, 0xB4) ;
  VB1 = read16 (fd, 0xB6) ;
  VB2 = read16 (fd, 0xB8) ;
   MB = read16 (fd, 0xBA) ;
   MC = read16 (fd, 0xBC) ;
   MD = read16 (fd, 0xBE) ;

// Calculate coefficients
  c3 = 160.0 *  AC3 / (1<<15);
  c4 = 1.0e-3 * AC4 / (1<<15);
  b1 = 160.0 * 160.0 * VB1 / (1<<30);
  c5 = 1.0 * AC5 / (160.0 * (1<<15)) ;
  c6 = 1.0 * AC6 ;
  mc = 1.0 * (1<<11) /(160.0 * 160.0) * MC ;
  md = 1.0 * MD / 160.0 ;
  x0 = 1.0 * AC1 ;
  x1 = 160.0 * AC2 / (1 << 13);
  x2 = 160.0 * 160.0 * VB2 / (1<<25);
  yy0 = c4 * (1<<15) ;
  yy1 = c4 * c3 ;
  yy2 = c4 * b1 ;
  p0 = (3791.0 - 8.0) / 1600.0 ;
  p1 = 1.0 - 7357.0 / (1<<20) ;
  p2 = 3038.0 * 100.0 / (1.0 * (1<<18) * (1<<18)) ;

  return TRUE ;
}
