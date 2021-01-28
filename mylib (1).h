/* mylib.h*/ 


#ifndef INCFILE1_H_			// this flag is to avoid multiple inclusion of the same header 
#define INCFILE1_H_


#define F_CPU 20000000UL

#include <stdio.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>



///////////////////////////////////////////////////////////////////////
//
// UART Functions
//
///////////////////////////////////////////////////////////////////////
#define BAUD_115200		115200UL
#define UBRR_VALUE		(F_CPU/16/BAUD_115200)	// 10 (the data-sheet formula is inaccurate).

#define BUF_SIZE	128
typedef struct{									// define data type.
	unsigned char buf[BUF_SIZE];
	int head;
	int tail;
	int data_size;
	int overrun;
	unsigned char flag;
} ring_buf_t;

void uart_init();								// function names declared before use.
void uart_putchar(unsigned char data);
void uart_putstr(char *str);

///////////////////////////////////////////////////////////////////////
//
// Timer1 ISR
//
///////////////////////////////////////////////////////////////////////
#define OCR1_VALUE_1HZ	19531
#define OCR1_VALUE_10HZ	1953

void timer1_init();

///////////////////////////////////////////////////////////////////////
//
// TWI functions 
//
///////////////////////////////////////////////////////////////////////
void twi_init();
int twi_write(uint8_t addr, uint8_t sub_addr, uint8_t ch);
int twi_read(uint8_t addr, uint8_t sub_addr, uint8_t *data, uint8_t size);

///////////////////////////////////////////////////////////////////////
//
// IMU functions
//
///////////////////////////////////////////////////////////////////////
#define WHO_AM_I_M		0x0F
#define MAG_ADDR		0x1E
#define IMU_ADDR		0x6B	// IMU datasheet @ page 30
#define BARO_ADDR		0x5D	// LPS25HB datasheet @ page 25

#define CTRL_REG1_G		0x10
#define STATUS_REG		0x17
#define OUT_X_G			0x18
#define	OUT_X_XL		0x28

#define CTRL_REG1_G5	5
#define STATUS_GDA0		0
#define STATUS_XLDA1	1

#define PI				3.141592
#define MAG_RESOL		(0.14*0.001)	// gauss
#define ACCL_RESOL		(0.061*0.001)	// g
#define GYRO_RESOL		(8.75*0.001)	// deg

typedef struct {
	float x;
	float y;
	float z;
}vec_t;

typedef struct {
	vec_t accel;
	vec_t gyro;
	vec_t mag;
	vec_t ang_accl;
	vec_t ang_gyro;
	float baro;
	float temperature;
	float dt;
}imu_t;

void mag_init();
int imu_init();
int init_all();

#endif /* INCFILE1_H_ */