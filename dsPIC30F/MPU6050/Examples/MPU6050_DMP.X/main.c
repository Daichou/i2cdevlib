/*
 * File:   LEDMain.c
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

#include <xc.h>
#include<uart.h>
#include<stdio.h>
#include<i2c.h>
#include<ports.h>
#include<stdint.h>
#include<libpic30.h>
#include"../../MPU6050.h"
#include"../../../I2Cdev/I2Cdev.h"
/* Received data is stored in array Buf  */
char Buf[80];
char* Receivedddata = Buf;
int16_t ax, ay, az, gx, gy, gz;
volatile unsigned int mpuInterrupt = false;
uint8_t devStatus;
/* This is UART1 transmit ISR */
void __attribute__((__interrupt__)) _U2TXInterrupt(void)
{  
   IFS1bits.U2TXIF = 0;
}

void __attribute__((__interrupt__)) _INT0Interrupt(void)
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
    
    ConfigINT0(RISING_EDGE_INT & INT_ENABLE & GLOBAL_INT_ENABLE & EXT_INT_PRI_1);
    MPU6050(MPU6050_ADDRESS_AD0_LOW);
    printf(Buf,"Initialize MPU6050\n\0");
    MPU6050_initialize();

    unsigned char MPU6050_ID = MPU6050_getDeviceID();

    printf("Testing device connections...\n\0");
    
    printf("MPU6050_ID = 0x%X\n",MPU6050_ID);
    
    printf(MPU6050_testConnection() ? "MPU6050 connection successful\r\n" :
        "MPU6050 connection failed\r\n");
    
    if (!MPU6050_testConnection())
        continue;

    printf("Reading offset\n");
    
    printf("Xaccel= %d\n",MPU6050_getXAccelOffset());
    
    printf("Yaccel= %d\n",MPU6050_getYAccelOffset());
    
    printf("Zaccel= %d\n",MPU6050_getZAccelOffset());
    
    printf("Xgyro= %d\n",MPU6050_getXGyroOffset());
    
    printf("Ygyro= %d\n",MPU6050_getYGyroOffset());
    
    printf("Zgyro= %d\n",MPU6050_getZGyroOffset());
    
    printf("Initialize DMP.....\n");
    
    MPU6050_dmpInitialize();

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
    while(1){

        while (true) {
           // Read raw accel/gyro measurements from device
            MPU6050_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

            // Display tab-separated accel/gyro x/y/z values
            printf("a/g:\t%d\t%d\t%d\t%d\t%d\t%d\r\n", ax, ay, az, gx, gy, gz);
            // Blink LED to indicate activity
            LATCbits.LATC14 ^= 1;

            delay_ms(25);
            delay_ms(25);
        }
        //CloseUART2();

    }
    return 0;
}
