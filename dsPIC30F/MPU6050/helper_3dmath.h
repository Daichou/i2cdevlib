// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

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

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_
#include <math.h>
#include <stdint.h>

struct quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct vectorInt16_t {
    int16_t x;
    int16_t y;
    int16_t z;
};
struct vectorfloat{
    float x;
    float y;
    float z;
};

typedef struct quaternion Quaternion;
typedef struct vectorInt16_t VectorInt16;
typedef struct vectorfloat VectorFloat;

void Quaternion_Init( Quaternion* A);
Quaternion Quaternion_Construct( Quaternion* A ,float nw, float nx, float ny, float nz);
Quaternion Quaternion_getProduct( Quaternion* A ,Quaternion* B);
Quaternion Quaternion_getConjugate( Quaternion* A);
float Quaternion_getMagnitude( Quaternion* A );
void Quaternion_normalize( Quaternion* A);
Quaternion Quaternion_getNormalized( Quaternion* A );

void VectorInt16_Init(VectorInt16* A );
void VectorInt16_Construct(VectorInt16* A , int16_t nx, int16_t ny, int16_t nz);
float VectorInt16_getMagnitude(VectorInt16* A);
void VectorInt16_normalize(VectorInt16* A);
VectorInt16 VectorInt16_getNormalized(VectorInt16* A );
void VectorInt16_rotate(VectorInt16* A ,Quaternion *q);
VectorInt16 VectorInt16_getRotated(VectorInt16* A,Quaternion *q);

void VectorFloat_Init(VectorFloat* A);
void VectorFloat_Construct(VectorFloat* A , float nx, float ny, float nz);
float VectorFloat_getMagnitude(VectorFloat* A);
void VectorFloat_normalize(VectorFloat* A);
VectorFloat getNormalized(VectorFloat* A);
void VectorFloat_rotate(VectorFloat* A , Quaternion *q);
VectorFloat VectorFloat_getRotated(VectorFloat* A , Quaternion *q); 

#endif /* _HELPER_3DMATH_H_ */
