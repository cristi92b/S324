/*
 * Super324P.c
 *
 * Created: 7/22/2013 5:16:24 PM
 *  Author: Cristi
 */ 


#define F_CPU 20000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


#define setHigh(port,pin) port|=(1<<pin)
#define setLow(port,pin) port&=~(1<<pin)
#define toggle(port,pin) port^=(1<<pin)
#define check(port,pin) ((port&(1<<pin))!=0)


#define DELAY _delay_ms(250)


#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

/***************************************

Configure Connections

****************************************/

#define DATA_PORT PORTA
#define DATA_DDR DDRA
#define CONTROL_PORT PORTB
#define CONTROL_DDR DDRB

#define SCK PB0 //shift clock
#define SCL PB1 //master reset, active-low (clear)
#define RCK PB2 //latch clock (latch)
#define BLANK PB3 //active-low, output enable(blank)



typedef struct cube{
	uint8_t p[8];
}CUBE;

typedef struct reg_data{
	uint8_t array[8];
}REG_DATA;


/*************************

Global variables

**************************/

uint8_t random_flag=0;

/**************************

Register operations

*************************/


void init_74HC595()
{
	DATA_DDR = 0xff;
	CONTROL_DDR = 0xff;
	setHigh(CONTROL_PORT,SCL);
	setLow(CONTROL_PORT,BLANK);
}

void SCK_pulse()
{
	setHigh(CONTROL_PORT,SCK);
	setLow(CONTROL_PORT,SCK);
}


void RCK_pulse()
{
	setHigh(CONTROL_PORT,RCK);
	setLow(CONTROL_PORT,RCK);
}

void update_storage_reg()
{
	RCK_pulse();
}

void clear_shift_reg()
{
	setLow(CONTROL_PORT,SCL);
	setHigh(CONTROL_PORT,SCL);
}

void clear_all()
{
	clear_shift_reg();
	update_storage_reg();
}

void blank_enable()
{
	setLow(CONTROL_PORT,BLANK);
}

void blank_disable()
{
	setHigh(CONTROL_PORT,BLANK);
}

void write_bit(uint8_t data) //8 * 1bit
{
	DATA_PORT=data;
	SCK_pulse();
}


void write_7bits(REG_DATA data)
{
	uint8_t i;
	for(i=0;i<7;i++)
	{
		write_bit(data.array[i]);
	}
}


void write_byte(REG_DATA data)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		write_bit(data.array[i]);
	}
}

REG_DATA cube2reg_data(CUBE x)
{
	REG_DATA y;
	uint8_t t,i,j;
	for(i=0;i<8;i++)
	{
		t=0;
		for(j=0;j<8;j++)
		{
			if(check(x.p[j],i))
			setHigh(t,j);
		}
		y.array[i]=t;
	}
	return y;
}

/***************************

Interrupts configuration

****************************/


ISR(INT0_vect)
{
	//code
}

ISR(INT1_vect)
{
	//code
}

void init_interrupt()
{
	//configure registers
	sei();
}

void disable_interrupt()
{
	cli();
}



/*************************

Cube modes

CUBE data type: {line,line,line,line,line,line,line,level}

*******************************/


void mode_00()
{
	int i,n=8;
	CUBE x[]={
		{0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x01},
		{0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x02},
		{0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x04},
		{0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x08},
		{0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x10},
		{0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x20},
		{0x7f,0x00,0x7f,0x00,0x7f,0x00,0x7f,0x40},
		{0x00,0x7f,0x00,0x7f,0x00,0x7f,0x00,0x80}
		};
		while(1)
		{
			for(i=0;i<n;i++)
			{
				write_7bits(cube2reg_data(x[i]));
				update_storage_reg();
				DELAY;
			}
		}		
}

int main()
{
	init_74HC595();
	mode_00();
	return 0;
}
