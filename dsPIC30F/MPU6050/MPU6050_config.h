
#ifndef _MPU6050_CONFIG_H_
#define _MPU6050_CONFIG_H_

#define FCY 20000000            //Please Setting your FCY = (FOSC/4)
#include <libpic30.h>
#define PROGMEM __attribute__((space(prog)))
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

#endif
