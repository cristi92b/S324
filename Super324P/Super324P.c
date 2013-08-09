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

#define BCD_DDR DDRC
#define BCD_PORT PORTC
#define INTERRUPT_DDR DDRD
#define INTERRUPT_PORT PORTD

#define RAND_LED PC4

#define BOARD_TEST_PIN PC7

//To be implemented in later release:
//automated increase/decrease mode after x seconds
//#define RAND_LEVEL2 PD0
//#define ASC_DESC PD1

typedef struct cube{
	uint8_t p[8];
}CUBE;

typedef struct reg_data{
	uint8_t array[8];
}REG_DATA;


/*************************

Global variables

**************************/

#define last_mode 9

uint8_t random_flag=0;
uint8_t led_mode=0;

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
	if(random_flag==0)
	{
		if(led_mode>=last_mode)
		{
			led_mode=0;
		}
		else
		{
			led_mode++;
		}
		non_random_operation(led_mode);
	}
}

ISR(INT1_vect)
{
	if(random_flag==0)
	{
	//enable
	setHigh(BCD_PORT,RAND_LED);
	random_flag=1;
	random_operation();
	{
	else
	{
	//disable
	setLow(BCD_PORT,RAND_LED);
	random_flag=0;
	non_random_operation(led_mode);
	}
}

void init_interrupt()
{
	EICRA|=(1<<ISC01)|(1<<ISC11); //trigger on falling edge for INT0 and INT1
	EIMSK|=(1<<INT0)|(1<<INT1);   //enable interrupt pins
	sei();
}

void disable_interrupt()
{
	cli();
}

void non_random_operation(uint8_t x)
{
	update_BCD(x);
	switch(x)
	{
		case 0 : mode_00(); break;
		case 1 : mode_00(); break;
		//...
		default : mode_00();
	}
}

void random_operation()
{
	
	//to be continued
}

/**************************

BCD Control

***************************/

void update_BCD(uint8_t x)
{
	x&=0x0f;
	if(check(BCD_PORT,RAND_LED)) setHigh(x,RAND_LED);
	BCD_PORT=x;
}

void init_BCD()
{
	BCD_DDR=0xff;
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

uint8_t check_board()
{
	if(check(PINC,BOARD_TEST_PIN)) return 1;
	else return 0;
}

void check_ok_signal()
{
	setHigh(BCD_PORT,RAND_LED);
	_delay_ms(100);
	setLow(BCD_PORT,RAND_LED);
	_delay_ms(100);
	setHigh(BCD_PORT,RAND_LED);
	_delay_ms(100);
	setLow(BCD_PORT,RAND_LED);
	_delay_ms(100);
	setHigh(BCD_PORT,RAND_LED);
	_delay_ms(100);
	setLow(BCD_PORT,RAND_LED);
}

int main()
{	
	init_74HC595();
	if(check_board())
	{
		
		init_BCD();
		check_ok_signal();
		init_interrupt();
		non_random_operation(led_mode);
	}
	else
	{
		mode_00();
	}		
	return 0;
}
