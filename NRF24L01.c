/* project: A driver for the NRF24L01 wireless module
	chip: NRF24L01
	PB7		SCK
	PB6		MISO
	PB5		MOSI
	PB4		CSN ( SS of the SPI chip select ) active low
	PB3		CE ( chip enable ) 	continuous high: 	receive
								high pulse:			send
								continuous low:		standby
	PD2		IRQ ( external interrupt 0 input ) active low
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "nRF24L01.h"


/* ================= define global variable ===============*/

unsigned char message[] = { 0x0b, 0x0c, 0x0d, 0x0e };
unsigned char buf[32] = { 0 };
unsigned char volatile mes_index = 0;
unsigned char volatile mes = 12;
unsigned char led_flag = 0x10;
unsigned int volatile cnt = 0x0f;
unsigned char cnt2 = 0;
unsigned int reg_d = 0;

unsigned char tx_addr[] = { 0x01, 0x03, 0x05, 0x07, 0x09 };
unsigned char rx_addr0[] = { 0x02, 0x04, 0x06, 0x08, 0x0A };

/* ================= function ================== */

unsigned char NUMTOSEG7(unsigned char DATA)
{ unsigned char AA;
	switch (DATA)
	{ 
		case 0: AA=0xc0;break;  // ‘0’
		case 1: AA=0xf9;break;  // ‘1’
		case 2: AA=0xa4;break;  // ‘2’
		case 3: AA=0xb0;break;  // ‘3’
		case 4: AA=0x99;break;  // ‘4’
		case 5: AA=0x92;break;  // ‘5’
		case 6: AA=0x82;break;  // ‘6’
		case 7: AA=0xf8;break;  // ‘7’
		case 8: AA=0x80;break;  // ‘8’
		case 9: AA=0x90;break;  // ‘9’
		case 10: AA=0x88;break; // ‘A’
		case 11: AA=0x83;break; // ‘B’
		case 12: AA=0xc6;break; // ‘C’
		case 13: AA=0xa1;break; // ‘D’
		case 14: AA=0x86;break; // ‘E’
		case 15: AA=0x8e;break; // ‘F’
		case '-':AA=0xdf;break; // 破折号
		case '_':AA=0xf7;break; // 下划线
		case ' ':AA=0xff;break; // 消隐
		default: AA=0xff;
	}
	return(AA);
}

void display_led(unsigned char seg,unsigned char sel)
{
  unsigned char i;
    
  //先将 sel 数据送74hc595
  PORTA &= ~(1<<PA7);         // PA7=0; rclk=0
  for (i=0;i<8;i++)
    {
	  if ((sel & 0x80) == 0)  //最高位送 U2 SER 端
	    PORTA &= ~(1<<PA5);   // PA5=0
	  else
	    PORTA |= (1<<PA5);    //  PA5=1
	  
	  PORTA &= ~(1<<PA6);     //PA6=0
	  PORTA |= (1<<PA6);      //PA6=1  srclk=1，产生移位时钟信号
	  
	  sel <<= 1;              //sel 左移一位
    } 
  //再将 seg 数据送74hc595
  for (i=0;i<8;i++)
    {
	  if ((seg & 0x80) == 0)  //最高位送 U2 SER 端
	    PORTA &= ~(1<<PA5);   //PA5=0
	  else
	    PORTA |= (1<<PA5);    //PA5=1
		   
	  PORTA &= ~(1<<PA6);     //PA6=0
	  PORTA |= (1<<PA6);      //PA6=1  srclk=1，产生移位时钟信号
	  
	  seg <<= 1;              //seg 左移一位 
    } 
  PORTA |= (1<<PA7);      //  PA7=1; rclk=1
  PORTA &= ~(1<<PA7);     //  PA7=0; rclk=0，产生锁存输出信号
}

void SPI_master_init(void)
{
	PORTB = 0x00;
	DDRB = (1<<PB3)|(1<<PB4)|(1<<PB5)|(1<<PB7);	// PB3,4,5,7 are output
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);	// LSB first: |(1<<DORD)
}

void Port_init( void )
{
	PORTA = 0x00; 
	DDRA  = 0xE0; //PA口 PA7、PA6、PA5为输出
	PORTC = 0xf0; //PC口 PC7、PC6、PC5、PC4接上拉电阻
	DDRC  = 0x00; //PC口 为输入
	PORTD = 0x00;
	DDRD = 0x00; // PD2 is input
}

//call this routine to initialize all peripherals
void init_devices(void)
{
	//stop errant interrupts until set up
	cli(); //disable all interrupts
	SPI_master_init();
	Port_init();
	MCUCR = (1<<ISC01);	// int0 external interrupt on falling edge of int0
	GICR  = (1<<INT0);	// enable the external interrupt int0
	sei(); //re-enable interrupts
	//all peripherals are now initialized
}

uint8_t SPI_transition(uint8_t data)
{
	SPDR = data;

	//this is an alternative choice with the Interrupt
	//if use ISR, then don't use this
	//if not use ISR, use this to waite.

	while( !(SPSR & (1<<SPIF)) )
		;
	SPSR |= (1<<SPIF);

	return SPDR;
}

void SPI_w_reg ( uint8_t reg, uint8_t val )
{
	PORTB &= ~(1<<PB4);
	SPI_transition( W_REG | reg );
	SPI_transition( val );
	PORTB |= (1<<PB4);
}

uint16_t SPI_r_reg ( uint8_t reg )
{
	uint16_t tmp;
	PORTB &= ~(1<<PB4);
	tmp = SPI_transition( R_REG | reg );
	tmp <<= 8;
	tmp |= SPI_transition( 5 );
	PORTB |= (1<<PB4);
	return tmp;
}

void SPI_write_data( uint8_t addr, uint8_t* ptr, uint8_t num )
{
	uint8_t i;
	PORTB &= ~(1<<PB4);
	SPI_transition( addr );
	for( i = 0; i < num; i++ )
	{
		SPI_transition( *ptr );
		ptr++;
	}
	PORTB |= (1<<PB4);
	return;
}

void SPI_read_data ( uint8_t addr, uint8_t* ptr, uint8_t num )
{
	uint8_t i;
	PORTB &= ~(1<<PB4);
	for ( i = 0; i < num ; i++ )
	{
		ptr[i] = SPI_transition( 0 );
	}
	PORTB |= (1<<PB4);
}

void Tx_Packet( uint8_t* rx_buf, uint8_t length )
{
	PORTB &= ~(1<<PB3);	// disable nRF

	PORTB &= ~(1<<PB4);
	SPI_transition( FLUSH_TX );
	PORTB |= (1<<PB4);
	SPI_write_data(W_REG + TX_ADDR, tx_addr , 5); 
	SPI_write_data( W_TX_PAYLOAD, rx_buf, length );
	SPI_w_reg( CONF_REG, 0x6e); 	

	PORTB |= (1<<PB3);	// enable nRF
}

void init_nRF24L01(void)
{
	PORTB &= ~(1<<PB3);    // chip disable
	_delay_us( 10 );
	PORTB |= (1<<PB4);   // Spi disable  

	SPI_w_reg( CONF_REG, 0x6e);
	SPI_w_reg( EN_AA, 0x01);      
	SPI_w_reg( EN_RXADDR, 0x01);    
	SPI_w_reg( SETUP_AW, 0x03 );
	SPI_w_reg( SETUP_RETR, 0x15 );	// wait 500uS, retransmit 5 times
	SPI_w_reg( RF_CH, 0);
	SPI_w_reg( RF_SETUP, 0x07);  
	SPI_w_reg( RX_PW_P0, 4);        
	SPI_write_data(W_REG + TX_ADDR, tx_addr , 5);    
	SPI_write_data(W_REG + RX_ADDR_P0, rx_addr0, 5); 
	PORTB |= (1<<PB3);	// chip enable
}

ISR( INT0_vect )
{
	PORTB &= ~(1<<PB3);	// disable nRF
	
	PORTB &= ~(1<<PB4);
	SPI_transition( FLUSH_TX );
	PORTB |= (1<<PB4);

	SPI_w_reg( STATUS , 0x10 );
	//cnt ++; if(cnt > 4) cnt = 0;
	
	cnt = SPI_r_reg ( STATUS );
	cnt >>= 4;

	display_led( NUMTOSEG7( cnt ), 0x08 );
	_delay_ms( 100 );
}

int main()
{
	init_devices();
	init_nRF24L01();
	
		
	while(1)
	{	
		

		cnt2 ++;
		cnt2 %= 16;
		display_led( NUMTOSEG7( cnt2 ), 0xf1 );
		_delay_ms(500);
		Tx_Packet( message, 4 );
		_delay_ms(10);

		cnt = SPI_r_reg ( FIFO_STATUS );
		reg_d = cnt & 0xff00;
		reg_d >>= 8;	// STATUS
		display_led( NUMTOSEG7( reg_d ), 0xf2 );
		_delay_ms( 500 );
		
		reg_d = cnt & 0x00f0;	// reg high
		reg_d >>= 4;
		display_led( NUMTOSEG7( reg_d ), 0xf4 );
		_delay_ms( 500 );

		reg_d = cnt & 0x000f;	// reg low
		display_led( NUMTOSEG7( reg_d ), 0xf8 );
		_delay_ms( 500 );

	}
}
	

