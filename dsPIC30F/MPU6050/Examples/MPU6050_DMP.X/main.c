/*
 * File:   main.c
 * Author: tommycc
 *
 * Created on March 1, 2017, 3:49 PM
 */



// DSPIC30F4013 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FOSFPR = ECIO       // Oscillator (ECIO w/PLL 8x)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_OFF         // POR Timer Value (Timer Disabled)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_OFF         // PBOR Enable (Disabled)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define true 1
#define false 0
#define _XTAL_FREQ 200000000


#define M_PI 3.141592653589793238462643
#define PI 3.1415926535897932384626433832795

//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL
//#define OUTPUT_TEAPOT

#include <xc.h>
#include<uart.h>
#include<stdio.h>
#include<i2c.h>
#include<ports.h>
#include<stdint.h>
#include<libpic30.h>
#include"../../MPU6050.h"
#include"../../../I2Cdev/I2Cdev.h"
//#include"../../MPU6050_6Axis_MotionApps20.h"
/* Received data is stored in array Buf  */
char Buf[80];
char* Receivedddata = Buf;
int16_t ax, ay, az, gx, gy, gz;
volatile unsigned int mpuInterrupt = false;
uint8_t devStatus;
uint8_t mpuInitStatus;
uint16_t packetSize;
uint16_t fifocount;
uint8_t fifoBuffer[64];
uint8_t dmpReady = false;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


/* This is UART1 transmit ISR */
void _ISR _U2TXInterrupt(void)
{  
   IFS1bits.U2TXIF = 0;
}

void _ISR _INT0Interrupt(void)
{  
   IFS0bits.INT0IF = 0;//clear interrupt
   mpuInterrupt = true;
}

/*******************************************************************************/
//  delay us using for-loop
/*******************************************************************************/
void delay_us( unsigned int usec )
{
	unsigned int i;
	//?   ?   ?   ?   ?   ?   ?   ?   ?   ?              //40 MIPS ,
	for ( i = 0 ; i < usec * 2;
	        i++ ) {                //for-loop 8Tcy -> 1us -> add two NOP()
		asm("NOP");
		asm("NOP");
	}
}

/*******************************************************************************/
// delay ms using Timer 1 interrupt
/*******************************************************************************/
void delay_ms( unsigned int msec )
{
	int i = 0;
	for (i = 0; i < msec; i++)
		delay_us(1000);
}

/*******************************************************************************/
//Init
/*******************************************************************************/
void Init()
{
    TRISCbits.TRISC14 = 0;
    LATCbits.LATC14 = 1;
    /* Data to be transmitted using UART communication module */
    TRISFbits.TRISF3 = 0;
    TRISFbits.TRISF2 = 0;
    char Buffer[80];
    /* Holds the value of baud register   */
    unsigned int baudvalue;
    /* Holds the value of uart config reg */
    unsigned int U2MODEvalue;
    /* Holds the information regarding uart
    TX & RX interrupt modes */
    unsigned int U2STAvalue;
    CloseI2C();
    /* Turn off UART1module */
    CloseUART2();
    /* Configure uart1 transmit interrupt */
    ConfigIntUART2(  UART_TX_INT_DIS & UART_TX_INT_PR2);
    /* Configure UART1 module to transmit 8 bit data with one stopbit. Also Enable loopback mode  */
    baudvalue = 15;//129;  //9600
    U2MODEvalue = UART_EN & UART_IDLE_CON &
                  UART_DIS_WAKE & UART_DIS_LOOPBACK  &
                  UART_EN_ABAUD & UART_NO_PAR_8BIT  &
                  UART_1STOPBIT;
    U2STAvalue  = UART_INT_TX_BUF_EMPTY  &
                  UART_TX_PIN_NORMAL &
                  UART_TX_ENABLE & UART_INT_RX_3_4_FUL &
                  UART_ADR_DETECT_DIS &
                  UART_RX_OVERRUN_CLEAR;
    unsigned int I2C_config1,I2C_config2;
    I2C_config1 = (I2C_ON & I2C_IDLE_CON & I2C_CLK_HLD &
             I2C_IPMI_DIS & I2C_7BIT_ADD &
             I2C_SLW_DIS & I2C_SM_DIS &
             I2C_GCALL_DIS & I2C_STR_EN &
             I2C_ACK & I2C_ACK_EN & I2C_RCV_EN &
             I2C_STOP_DIS & I2C_RESTART_EN &
             I2C_START_DIS);
    I2C_config2 = 381;
    ConfigIntI2C(MI2C_INT_OFF & SI2C_INT_OFF);
    OpenI2C(I2C_config1,I2C_config2);
    OpenUART2(U2MODEvalue, U2STAvalue, baudvalue);
    ConfigINT0(RISING_EDGE_INT & EXT_INT_ENABLE & GLOBAL_INT_ENABLE & EXT_INT_PRI_1);

    MPU6050(MPU6050_ADDRESS_AD0_LOW);
    printf(Buf,"Initialize MPU6050\n\0");
    MPU6050_initialize();

    unsigned char MPU6050_ID = MPU6050_getDeviceID();

    printf("Testing device connections...\n");
    printf("MPU6050_ID = 0x%X\n",MPU6050_ID);
    printf(MPU6050_testConnection() ? "MPU6050 connection successful\r\n" :
        "MPU6050 connection failed\r\n");

    //if (!MPU6050_testConnection())
     //   continue;

    printf("Reading offset\n");
    printf("Xaccel= %d\n",MPU6050_getXAccelOffset());
    printf("Yaccel= %d\n",MPU6050_getYAccelOffset());
    printf("Zaccel= %d\n",MPU6050_getZAccelOffset());
    printf("Xgyro= %d\n",MPU6050_getXGyroOffset());
    printf("Ygyro= %d\n",MPU6050_getYGyroOffset());
    printf("Zgyro= %d\n",MPU6050_getZGyroOffset());
    printf("Initialize DMP.....\n");

    devStatus = MPU6050_dmpInitialize();

    if (devStatus == 0) {
        printf("DMP Enabled.....\n");
        MPU6050_setDMPEnabled(true);
        mpuInitStatus = MPU6050_getIntStatus();
        
        printf("DMP ready,wait for first interupt\n");
        dmpReady = true;

        packetSize = MPU6050_dmpGetFIFOPacketSize();
    } else {
        printf("DMP Initialize failed!!!\n");
        printf("devStatus = %u\n",devStatus);
        printf("\n");
    } 

    MPU6050_setXGyroOffset(220);
    MPU6050_setYGyroOffset(76);
    MPU6050_setZGyroOffset(-85);

    printf("Reading Updated offset\n");
    printf("Xaccel= %d\n",MPU6050_getXAccelOffset());
    printf("Yaccel= %d\n",MPU6050_getYAccelOffset());
    printf("Zaccel= %d\n",MPU6050_getZAccelOffset());
    printf("Xgyro= %d\n",MPU6050_getXGyroOffset());
    printf("Ygyro= %d\n",MPU6050_getYGyroOffset());
    printf("Zgyro= %d\n",MPU6050_getZGyroOffset());
    MPU6050_setSleepEnabled(false);
}


int main(void)
{
    __C30_UART=2;
    Init();
    if (!dmpReady) {
        while(1);
    }
    while(1){
        while(!mpuInterrupt && fifocount < packetSize){
        
        }
        mpuInterrupt = false;
        mpuInitStatus = MPU6050_getIntStatus();

        fifocount = MPU6050_getFIFOCount();
        if ( (mpuInitStatus & 0x10) || (fifocount == 1024) ){
            MPU6050_resetFIFO();
            printf("FIFO overflow!!");
        } else if ( mpuInitStatus && 0x02 )
        {
            while ( fifocount < packetSize ) fifocount = MPU6050_getFIFOCount();

            MPU6050_getFIFOBytes(fifoBuffer,packetSize);
#ifdef OUTPUT_READABLE_QUATERNION
            MPU6050_dmpGetQuaternion(&q, fifoBuffer);
            printf("quat\t%f\t%f\t%f\t%f\n",q.w,q.x,q.y,q.z);
#endif

#ifdef OUTPUT_READABLE_EULER
            MPU6050_dmpGetQuaternion(&q, fifoBuffer);
            MPU6050_dmpGetEuler(euler,&q);
            printf("Euler\t%f\t%f\t%f\n",euler[0] * 180/M_PI,euler[1] * 180/M_PI,euler[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
            MPU6050_dmpGetQuaternion(&q, fifoBuffer);
            MPU6050_dmpGetGravity(&gravity,&q);
            MPU6050_dmpGetYawPitchRoll(ypr,&q,&gravity);
            printf("ypr\t%f\t%f\t%f\n",ypr[0] * 180/M_PI,ypr[1] * 180/M_PI,ypr[2] * 180/M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
            MPU6050_dmpGetQuaternion(&q, fifoBuffer);
            MPU6050_dmpGetAccel(&aa,fifoBuffer);
            MPU6050_dmpGetGravity(&gravity,&q);
            MPU6050_dmpGetLinearAccel(&aaReal,&aa,&gravity);
            printf("areal\t%d\t%d\t%d\n",aaReal.x,aaReal.y,aaReal.z); 
#endif

#ifdef OUTPUT_READABLE_WORDACCEL
            MPU6050_dmpGetQuaternion(&q, fifoBuffer);
            MPU6050_dmpGetAccel(&aa,fifoBuffer);
            MPU6050_dmpGetGravity(&gravity,&q);
            MPU6050_dmpGetLinearAccel(&aaReal,&aa,&gravity);
            MPU6050_dmpGetLinearAccelInWorld(&aaWorld,&aaReal,&q);
            printf("aworld\t%d\t%d\t%d\n",aaWorld.x,aaWorld.y,aaWorld.z);
#endif

#ifdef OUTPUT_TEAPOT
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            putsUART2 ((unsigned int *)fifoBuffer);
            while(BusyUART2());
            teapotPacket[11]++;
#endif
        }
        // Blink LED to indicate activity
        LATCbits.LATC14 ^= 1;

        //CloseUART2();

    }
    return 0;
}
