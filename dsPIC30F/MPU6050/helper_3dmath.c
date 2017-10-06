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

#include "helper_3dmath.h"
#include <math.h>
#include <stdint.h>


void Quaternion_Init( Quaternion* A) {
    A->w = 1.0f;
    A->x = 0.0f;
    A->y = 0.0f;
    A->z = 0.0f;
}
Quaternion Quaternion_Construct( Quaternion* A ,float nw, float nx, float ny, float nz) {
    A->w = nw;
    A->x = nx;
    A->y = ny;
    A->z = nz;
    return *A;
}
Quaternion Quaternion_getProduct( Quaternion* A ,Quaternion* B) {
    // Quaternion multiplication is defined by:
    //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
    Quaternion outcome;
    Quaternion_Construct(&outcome,
        A->w*B->w - A->x*B->x - A->y*B->y - A->z*B->z,  // new w
        A->w*B->x + A->x*B->w + A->y*B->z - A->z*B->y,  // new x
        A->w*B->y - A->x*B->z + A->y*B->w + A->z*B->x,  // new y
        A->w*B->z + A->x*B->y - A->y*B->x + A->z*B->w); // new z
    return outcome;
}

Quaternion Quaternion_getConjugate( Quaternion* A) {
    Quaternion outcome;
    return Quaternion_Construct(&outcome,A->w, - A->x, - A->y, - A->z);
}

float Quaternion_getMagnitude( Quaternion* A ) {
    return sqrt(A->w*A->w + A->x*A->x + A->y*A->y + A->z*A->z);
}

void Quaternion_normalize( Quaternion* A) {
    float m = Quaternion_getMagnitude(A);
    A->w /= m;
    A->x /= m;
    A->y /= m;
    A->z /= m;
}

Quaternion Quaternion_getNormalized( Quaternion* A ) {
    Quaternion r;
    Quaternion_Construct(&r,A->w,A->x,A->y,A->z);
    Quaternion_normalize(&r);
    return r;
}

void VectorInt16_Init(VectorInt16* A ) {
    A->x = 0;
    A->y = 0;
    A->z = 0;
}

void VectorInt16_Construct(VectorInt16* A , int16_t nx, int16_t ny, int16_t nz) {
    A->x = nx;
    A->y = ny;
    A->z = nz;
}

float VectorInt16_getMagnitude(VectorInt16* A) {
    return sqrt(A->x*A->x + A->y*A->y + A->z*A->z);
}

void VectorInt16_normalize(VectorInt16* A) {
    float m = VectorInt16_getMagnitude(A);
    A->x /= m;
    A->y /= m;
    A->z /= m;
}

VectorInt16 VectorInt16_getNormalized(VectorInt16* A ) {
    VectorInt16 r;
    VectorInt16_Construct(&r,A->x,A->y,A->z);
    VectorInt16_normalize(&r);
    return r;
}

void VectorInt16_rotate(VectorInt16* A ,Quaternion *q) {
    // http://www.cprogramming.com/tutorial/3d/quaternions.html
    // http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
    // http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
    // ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

    // P_out = q * P_in * conj(q)
    // - P_out is the output vector
    // - q is the orientation quaternion
    // - P_in is the input vector (a*aReal)
    // - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
    Quaternion p;
    Quaternion_Construct(&p , 0 , A->x , A->y , A->z);

    // quaternion multiplication: q * p, stored back in p
    p = Quaternion_getProduct(q,&p);

    // quaternion multiplication: p * conj(q), stored back in p
    Quaternion tmp = Quaternion_getConjugate(q);
    p = Quaternion_getProduct(&p , &tmp);

    // p quaternion is now [0, x', y', z']
    A->x = p.x;
    A->y = p.y;
    A->z = p.z;
}

VectorInt16 VectorInt16_getRotated(VectorInt16* A,Quaternion *q) {
    VectorInt16 r;
    VectorInt16_Construct(&r,A->x, A->y, A->z);
    VectorInt16_rotate(&r,q);
    return r;
}



void VectorFloat_Init(VectorFloat* A) {
    A->x = 0;
    A->y = 0;
    A->z = 0;
}

void VectorFloat_Construct(VectorFloat* A , float nx, float ny, float nz) {
    A->x = nx;
    A->y = ny;
    A->z = nz;
}

float VectorFloat_getMagnitude(VectorFloat* A) {
    return sqrt(A->x*A->x + A->y*A->y + A->z*A->z);
}

void VectorFloat_normalize(VectorFloat* A) {
    float m = VectorFloat_getMagnitude(A);
    A->x /= m;
    A->y /= m;
    A->z /= m;
}

VectorFloat getNormalized(VectorFloat* A) {
    VectorFloat r;
    VectorFloat_Construct(&r, A->x, A->y, A->z);
    VectorFloat_normalize(&r);
    return r;
}

void VectorFloat_rotate(VectorFloat* A , Quaternion *q) {
    Quaternion p;
    Quaternion_Construct(&p, 0 , A->x, A->y, A->z);

    // quaternion multiplication: q * p, stored back in p
    //p = q -> getProduct(p);
    p = Quaternion_getProduct(q,&p);
    // quaternion multiplication: p * conj(q), stored back in p
    //p = p.getProduct(q -> getConjugate());
    Quaternion tmp = Quaternion_getConjugate(q);
    p = Quaternion_getProduct(&p,&tmp);
    // p quaternion is now [0, x', y', z']
    A->x = p.x;
    A->y = p.y;
    A->z = p.z;
}

VectorFloat VectorFloat_getRotated(VectorFloat* A , Quaternion *q) {
    VectorFloat r;
    VectorFloat_Construct(&r,A->x, A->y, A->z);
    VectorFloat_rotate(&r,q);
    return r;
}


