// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
#define _MPU6050_6AXIS_MOTIONAPPS20_H_

#include "../I2Cdev/I2Cdev.h"
#include "helper_3dmath.h"
#include "MPU6050.h"
// MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
#define MPU6050_INCLUDE_DMP_MOTIONAPPS20


#include <xc.h>
#include <stdint.h>
#include <string.h>
/* Source is from the InvenSense MotionApps v2 demo code. Original source is
 * unavailable, unless you happen to be amazing as decompiling binary by
 * hand (in which case, please contact me, and I'm totally serious).
 *
 * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
 * DMP reverse-engineering he did to help make this bit of wizardry
 * possible.
 */

// NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
// Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
// after moving string constants to flash memory storage using the F()
// compiler macro (Arduino IDE 1.0+ required).

uint8_t MPU6050_dmpInitialize();
bool MPU6050_dmpPacketAvailable();
uint8_t MPU6050_dmpGetAccel32(int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetAccel16(int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetAccel(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion32(int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion16(int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetQuaternion(Quaternion *q, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro32(int32_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro16(int16_t *data, const uint8_t* packet);
uint8_t MPU6050_dmpGetGyro(VectorInt16 *v, const uint8_t* packet);
uint8_t MPU6050_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
uint8_t MPU6050_dmpGetGravity(VectorFloat *v, Quaternion *q);
uint8_t MPU6050_dmpGetEuler(float *data, Quaternion *q);
uint8_t MPU6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
uint8_t MPU6050_dmpProcessFIFOPacket(const unsigned char *dmpData);
uint8_t MPU6050_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);
uint16_t MPU6050_dmpGetFIFOPacketSize();

#endif 
